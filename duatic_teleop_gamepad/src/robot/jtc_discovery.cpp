/*
 * Copyright 2026 Duatic AG
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "duatic_teleop_gamepad/robot/jtc_discovery.hpp"

#include <algorithm>
#include <chrono>
#include <iterator>
#include <string>
#include <utility>

#include <rcpputils/split.hpp>

namespace duatic_teleop_gamepad
{

namespace
{

constexpr const char* kTopicSuffix = "joint_trajectory";
constexpr const char* kJointsParameter = "joints";
constexpr const char* kAllowMovingEndParameter = "allow_nonzero_velocity_at_trajectory_end";

/// How long a parameter request may stay outstanding before the controller is asked again.
/// A controller that dies after its service answered ready never completes its request, and
/// without a retry that controller would stay undiscovered for the lifetime of the node.
constexpr std::chrono::seconds kJointsRequestTimeout{ 5 };

}  // namespace

std::vector<JtcTopic> select_jtc_topics(const std::vector<std::string>& topic_names, const std::string& node_namespace)
{
  std::vector<JtcTopic> selected;

  const auto expected_namespace = rcpputils::split(node_namespace, '/', true);
  const auto prefix_length = std::string(kTrajectoryControllerBase).size();

  for (const auto& topic : topic_names) {
    const auto segments = rcpputils::split(topic, '/', true);

    // Expect <namespace...>/<controller>/joint_trajectory.
    if (segments.size() != expected_namespace.size() + 2) {
      continue;
    }

    if (!std::equal(expected_namespace.begin(), expected_namespace.end(), segments.begin())) {
      continue;
    }

    const std::string& controller = segments[segments.size() - 2];
    if (!controller.starts_with(kTrajectoryControllerBase) || segments.back() != kTopicSuffix) {
      continue;
    }

    // The component is whatever the controller's name carries after the prefix. A flat
    // single-arm robot spawns the bare controller and so has no component name at all,
    // which is the same empty name its trajectory topic is keyed under.
    std::string component = controller.substr(prefix_length);
    if (!component.empty()) {
      // Only an underscore separates the two, so "joint_trajectory_controller2" is a
      // different controller rather than the component "2".
      if (component.front() != '_') {
        continue;
      }
      component.erase(0, 1);
    }

    selected.push_back({ topic, controller, component });
  }

  std::sort(selected.begin(), selected.end(),
            [](const JtcTopic& a, const JtcTopic& b) { return a.topic < b.topic; });
  return selected;
}

JtcDiscovery::JtcDiscovery(rclcpp::Node& node, std::function<void()> on_changed)
  : node_(node), on_changed_(std::move(on_changed))
{
}

void JtcDiscovery::reconcile()
{
  std::vector<std::string> topic_names;
  for (const auto& entry : node_.get_topic_names_and_types()) {
    topic_names.push_back(entry.first);
  }

  const auto wanted = select_jtc_topics(topic_names, node_.get_effective_namespace());

  const auto still_wanted = [&wanted](const Target& target) {
    return std::any_of(wanted.begin(), wanted.end(),
                       [&target](const JtcTopic& topic) { return topic.topic == target.topic; });
  };

  const auto removed = std::remove_if(targets_.begin(), targets_.end(),
                                      [&still_wanted](const Target& target) { return !still_wanted(target); });
  const bool dropped_any = removed != targets_.end();
  targets_.erase(removed, targets_.end());

  for (const auto& topic : wanted) {
    const bool known = std::any_of(targets_.begin(), targets_.end(),
                                   [&topic](const Target& target) { return target.topic == topic.topic; });

    const auto pending = requests_in_flight_.find(topic.controller);
    const bool waiting = pending != requests_in_flight_.end() &&
                         node_.now() - pending->second < rclcpp::Duration(kJointsRequestTimeout);

    if (!known && !waiting) {
      request_joints(topic);
    }
  }

  if (dropped_any) {
    on_changed_();
  }
}

void JtcDiscovery::request_joints(const JtcTopic& jtc_topic)
{
  auto& client = parameter_clients_[jtc_topic.controller];
  if (!client) {
    client = std::make_shared<rclcpp::AsyncParametersClient>(&node_, jtc_topic.controller);
  }

  // The controller's parameter services come up after its topics do, so a topic without
  // them yet is simply retried on the next pass rather than waited for.
  if (!client->service_is_ready()) {
    return;
  }

  requests_in_flight_[jtc_topic.controller] = node_.now();

  client->get_parameters(
      { kJointsParameter, kAllowMovingEndParameter },
      [this, jtc_topic](std::shared_future<std::vector<rclcpp::Parameter>> future) {
        requests_in_flight_.erase(jtc_topic.controller);

        // A retry may have gone out before this one landed, so the answer that arrives
        // second must not register the topic twice.
        const bool known = std::any_of(targets_.begin(), targets_.end(), [&jtc_topic](const Target& target) {
          return target.topic == jtc_topic.topic;
        });
        if (known) {
          return;
        }

        const auto parameters = future.get();
        if (parameters.size() != 2 || parameters[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
          RCLCPP_ERROR(node_.get_logger(), "Controller %s does not report a 'joints' parameter; it cannot be jogged",
                       jtc_topic.controller.c_str());
          return;
        }

        const auto joints = parameters[0].as_string_array();
        if (joints.empty()) {
          RCLCPP_ERROR(node_.get_logger(), "Controller %s reports an empty 'joints' parameter",
                       jtc_topic.controller.c_str());
          return;
        }

        // Every point in the jog stream ends while still moving, which a controller left at
        // the default rejects outright, dropping every message. Say so by name here rather
        // than leaving the operator to work out why a healthy-looking node moves nothing.
        const bool allows_moving_end =
            parameters[1].get_type() == rclcpp::ParameterType::PARAMETER_BOOL && parameters[1].as_bool();
        if (!allows_moving_end) {
          RCLCPP_ERROR(node_.get_logger(),
                       "Controller %s has allow_nonzero_velocity_at_trajectory_end unset, so it will reject every "
                       "jog command. Set it to true in the controller config.",
                       jtc_topic.controller.c_str());
        }

        targets_.push_back({ jtc_topic.topic, jtc_topic.component, joints });
        std::sort(targets_.begin(), targets_.end(),
                  [](const Target& a, const Target& b) { return a.component < b.component; });

        RCLCPP_INFO(node_.get_logger(), "Discovered %s driving %zu joints", jtc_topic.controller.c_str(),
                    joints.size());
        on_changed_();
      });
}

const std::vector<JtcDiscovery::Target>& JtcDiscovery::targets() const
{
  return targets_;
}

bool JtcDiscovery::drives(const std::string& component) const
{
  return std::any_of(targets_.begin(), targets_.end(),
                     [&component](const Target& target) { return target.component == component; });
}

}  // namespace duatic_teleop_gamepad
