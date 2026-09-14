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

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>

#include "duatic_teleop_gamepad/gamepad_config.hpp"
#include "duatic_teleop_gamepad/input/gamepad_input.hpp"
#include "duatic_teleop_gamepad/modes/mode_manager.hpp"
#include "duatic_teleop_gamepad/robot/controller_manager_client.hpp"
#include "duatic_teleop_gamepad/robot/joint_state_cache.hpp"
#include "duatic_teleop_gamepad/robot/jtc_discovery.hpp"

namespace duatic_teleop_gamepad
{

/// Drives a Duatic robot from a gamepad.
///
/// Owns the plumbing and nothing else: the subscriptions, the timers, and the focus the
/// modes act on. What the sticks mean is the active mode's business, and which mode that
/// is is the ModeManager's.
///
/// Comes up immediately and wires itself as the robot appears, rather than blocking in the
/// constructor until it does. Everything it depends on, the robot's controllers and its
/// trajectory topics, is re-checked on the discovery timer, so a component or controller
/// that arrives late is picked up, and one that goes away is dropped, without the node
/// having to be restarted.
class GamepadNode : public rclcpp::Node
{
public:
  GamepadNode();

private:
  void on_joy(sensor_msgs::msg::Joy::ConstSharedPtr msg);
  void on_joint_states(sensor_msgs::msg::JointState::ConstSharedPtr msg);

  /// Read the gamepad and act on it. Runs at the input rate.
  void process_input();

  /// Advance and publish the active mode's command stream. Runs at the jog publish rate.
  void publish_mode();

  /// Re-check the robot's controllers and its topics. Runs at the discovery rate.
  void discover();

  /// Follow the trajectory controllers the robot has spawned.
  void on_robot_changed();

  void update_focus(const GamepadInput& input);
  void set_focus(const std::string& component);

  void send_rumble(double intensity);

  GamepadConfig config_;

  JointStateCache joint_states_;
  std::unique_ptr<ControllerManagerClient> controllers_;
  std::unique_ptr<JtcDiscovery> discovery_;
  std::unique_ptr<ModeManager> modes_;

  std::string focus_;

  /// Whether the operator has picked a focus, after which discovery stops choosing one.
  bool focus_chosen_{ false };

  sensor_msgs::msg::Joy::ConstSharedPtr latest_joy_;
  bool deadman_{ false };
  bool freeze_{ true };
  bool rumbling_{ false };
  ButtonEdge switch_button_;
  ButtonEdge focus_button_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr feedback_pub_;

  rclcpp::TimerBase::SharedPtr input_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr discovery_timer_;
};

}  // namespace duatic_teleop_gamepad
