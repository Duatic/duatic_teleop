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

#pragma once

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace duatic_teleop_gamepad
{

/// Name prefix every joint trajectory controller carries. It is both how they are found on
/// the graph and how the modes that need them name them to the controller manager.
inline constexpr const char* kTrajectoryControllerBase = "joint_trajectory_controller";

/// One joint_trajectory topic and the controller that owns it.
struct JtcTopic
{
  std::string topic;
  std::string controller;
  std::string component;
};

/// @brief Pick out every joint_trajectory topic on the graph, naming the component each
///   one drives.
/// @param topic_names Every topic currently on the graph.
/// @param node_namespace Namespace to search under, "/" for none.
///
/// The component is read off the controller's own name rather than checked against a list
/// of components the robot is believed to have: a spawned trajectory controller is the
/// authority on what can be jogged, and its name already says what it drives.
std::vector<JtcTopic> select_jtc_topics(const std::vector<std::string>& topic_names,
                                        const std::string& node_namespace);

/// Works out which joint trajectory controllers exist and which joints each one drives.
///
/// The joint order comes from the controller's own "joints" parameter rather than from the
/// joint states, because it decides which stick drives which joint, and the joint states
/// carry no meaningful order.
///
/// Written as a reconcile loop rather than one-shot discovery: every call compares what is
/// on the graph against what is already known and asks only for the difference. A
/// controller that starts late is picked up on a later pass instead of having to be caught
/// by a retry loop at startup, and nothing blocks while waiting for it.
class JtcDiscovery
{
public:
  struct Target
  {
    std::string topic;
    std::string component;
    std::vector<std::string> joints;
  };

  /// @param node Node the parameter clients are created on.
  /// @param on_changed Called whenever the target list changes, so publishers can follow.
  JtcDiscovery(rclcpp::Node& node, std::function<void()> on_changed);

  /// Bring the known targets in line with the controllers currently on the graph. Cheap
  /// and idempotent, so it suits being called periodically.
  void reconcile();

  /// The controllers found so far, sorted by component, so what is first does not depend
  /// on which controller happened to answer first.
  const std::vector<Target>& targets() const;

  /// Whether a trajectory controller drives this component, which is what makes it
  /// focusable.
  bool drives(const std::string& component) const;

private:
  void request_joints(const JtcTopic& jtc_topic);

  rclcpp::Node& node_;
  std::function<void()> on_changed_;

  std::unordered_map<std::string, rclcpp::AsyncParametersClient::SharedPtr> parameter_clients_;

  /// When each outstanding parameter request was sent, so one that never lands can be
  /// retried instead of shutting its controller out permanently.
  std::unordered_map<std::string, rclcpp::Time> requests_in_flight_;
  std::vector<Target> targets_;
};

}  // namespace duatic_teleop_gamepad
