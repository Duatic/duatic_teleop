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

#include "duatic_teleop_gamepad/jtc_discovery.hpp"

#include <algorithm>
#include <utility>

namespace duatic_teleop_gamepad
{

namespace
{

constexpr const char* kControllerPrefix = "joint_trajectory_controller";
constexpr const char* kTopicSuffix = "joint_trajectory";
constexpr const char* kJointsParameter = "joints";
constexpr const char* kAllowMovingEndParameter = "allow_nonzero_velocity_at_trajectory_end";

std::vector<std::string> split(const std::string& value, char separator)
{
  std::vector<std::string> parts;
  std::string part;

  for (const char character : value) {
    if (character == separator) {
      if (!part.empty()) {
        parts.push_back(part);
        part.clear();
      }
    } else {
      part.push_back(character);
    }
  }

  if (!part.empty()) {
    parts.push_back(part);
  }

  return parts;
}

bool starts_with(const std::string& value, const std::string& prefix)
{
  return value.rfind(prefix, 0) == 0;
}

const std::vector<std::string>& no_joints()
{
  static const std::vector<std::string> empty;
  return empty;
}

}  // namespace

std::vector<JtcTopic> select_jtc_topics(const std::vector<std::string>& topic_names,
                                        const std::vector<std::string>& component_names,
                                        const std::string& node_namespace)
{
  std::vector<JtcTopic> selected;

  if (component_names.empty()) {
    return selected;
  }

  // A component with no name is a flat single-arm robot, whose controller topic carries no
  // component suffix to match against, so every candidate topic belongs to it.
  const bool accept_any =
      std::any_of(component_names.begin(), component_names.end(), [](const std::string& name) { return name.empty(); });

  const auto expected_namespace = split(node_namespace, '/');

  for (const auto& topic : topic_names) {
    const auto segments = split(topic, '/');

    // Expect <namespace...>/<controller>/joint_trajectory.
    if (segments.size() != expected_namespace.size() + 2) {
      continue;
    }

    if (!std::equal(expected_namespace.begin(), expected_namespace.end(), segments.begin())) {
      continue;
    }

    const std::string& controller = segments[segments.size() - 2];
    if (!starts_with(controller, kControllerPrefix) || segments.back() != kTopicSuffix) {
      continue;
    }

    // The controller's name is the prefix plus the component, exactly. Matching on a
    // trailing substring instead would let the component "left" claim the controller for
    // "arm_left".
    const bool wanted = accept_any || std::any_of(component_names.begin(), component_names.end(),
                                                  [&controller](const std::string& component) {
                                                    return !component.empty() &&
                                                           controller == kControllerPrefix + std::string("_") +
                                                                             component;
                                                  });

    if (wanted) {
      selected.push_back({ topic, controller });
    }
  }

  std::sort(selected.begin(), selected.end(),
            [](const JtcTopic& a, const JtcTopic& b) { return a.topic < b.topic; });
  return selected;
}

JtcDiscovery::JtcDiscovery(rclcpp::Node& node, std::function<void()> on_changed)
  : node_(node), on_changed_(std::move(on_changed))
{
}

void JtcDiscovery::reconcile(const std::vector<std::string>& component_names)
{
  std::vector<std::string> topic_names;
  for (const auto& entry : node_.get_topic_names_and_types()) {
    topic_names.push_back(entry.first);
  }

  const auto wanted = select_jtc_topics(topic_names, component_names, node_.get_effective_namespace());

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

    if (!known && requests_in_flight_.count(topic.controller) == 0) {
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

  requests_in_flight_.insert(jtc_topic.controller);

  client->get_parameters(
      { kJointsParameter, kAllowMovingEndParameter },
      [this, jtc_topic](std::shared_future<std::vector<rclcpp::Parameter>> future) {
        requests_in_flight_.erase(jtc_topic.controller);

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

        targets_.push_back({ jtc_topic.topic, jtc_topic.controller, joints });
        std::sort(targets_.begin(), targets_.end(),
                  [](const Target& a, const Target& b) { return a.topic < b.topic; });

        RCLCPP_INFO(node_.get_logger(), "Discovered %s driving %zu joints", jtc_topic.controller.c_str(),
                    joints.size());
        on_changed_();
      });
}

const std::vector<JtcDiscovery::Target>& JtcDiscovery::targets() const
{
  return targets_;
}

const std::vector<std::string>& JtcDiscovery::joints_for(const std::string& topic) const
{
  const auto it = std::find_if(targets_.begin(), targets_.end(),
                               [&topic](const Target& target) { return target.topic == topic; });

  return it == targets_.end() ? no_joints() : it->joints;
}

}  // namespace duatic_teleop_gamepad
