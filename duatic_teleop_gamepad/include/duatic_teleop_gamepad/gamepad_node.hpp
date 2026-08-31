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

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "duatic_teleop_gamepad/controller_manager_client.hpp"
#include "duatic_teleop_gamepad/drive_ramp.hpp"
#include "duatic_teleop_gamepad/gamepad_config.hpp"
#include "duatic_teleop_gamepad/gripper_toggle.hpp"
#include "duatic_teleop_gamepad/jog_group.hpp"
#include "duatic_teleop_gamepad/joint_state_cache.hpp"
#include "duatic_teleop_gamepad/jtc_discovery.hpp"
#include "duatic_teleop_gamepad/robot_model.hpp"
#include "duatic_teleop_gamepad/teleop_mode.hpp"

namespace duatic_teleop_gamepad
{

/// Drives a Duatic robot from a gamepad.
///
/// Comes up immediately and wires itself as the robot appears, rather than blocking in the
/// constructor until it does. Everything it depends on -- the robot's shape, its
/// controllers, its trajectory topics -- is re-checked on the discovery timer, so a
/// component or controller that arrives late is picked up, and one that goes away is
/// dropped, without the node having to be restarted.
class GamepadNode : public rclcpp::Node
{
public:
  GamepadNode();

private:
  /// A jog group and the publisher its commands go out on, kept together so the two can
  /// never fall out of step.
  struct JogTarget
  {
    JogGroup group;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher;
  };

  void on_joy(sensor_msgs::msg::Joy::ConstSharedPtr msg);
  void on_joint_states(sensor_msgs::msg::JointState::ConstSharedPtr msg);

  /// Read the gamepad and act on it. Runs at the input rate.
  void process_input();

  /// Advance and publish the trajectory stream. Runs at the jog publish rate.
  void publish_jog();

  /// Re-check the robot's shape, its controllers and its topics. Runs at the discovery rate.
  void discover();

  void rebuild_jog_groups();
  void rebuild_gripper_publishers();

  void apply_mode(TeleopMode mode);
  void reset_active_mode();
  void update_focus(const sensor_msgs::msg::Joy& msg);
  void set_focus(const std::string& component);
  void send_rumble(double intensity);
  void stop_move_commands();

  /// The jog target for the focused component, or nullptr when there is none.
  JogTarget* focused_target();

  GamepadConfig config_;

  JointStateCache joint_states_;
  RobotModel robot_;
  std::unique_ptr<ControllerManagerClient> controllers_;
  std::unique_ptr<JtcDiscovery> discovery_;

  /// Keyed by the component each group drives, which is what focus selects.
  std::map<std::string, JogTarget> jog_groups_;
  DriveRamp drive_;
  GripperToggle gripper_;

  std::string focus_;
  std::vector<TeleopMode> available_;
  std::optional<TeleopMode> mode_;
  std::optional<TeleopMode> pending_mode_;
  rclcpp::Time pending_since_;

  sensor_msgs::msg::Joy::ConstSharedPtr latest_joy_;
  bool deadman_{ false };
  bool freeze_{ true };
  bool switch_held_{ false };
  bool move_command_active_{ false };
  bool drive_active_{ false };
  bool rumbling_{ false };
  bool dpad_held_{ false };

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr move_home_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr move_sleep_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr feedback_pub_;

  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr> gripper_pubs_;

  rclcpp::TimerBase::SharedPtr input_timer_;
  rclcpp::TimerBase::SharedPtr jog_timer_;
  rclcpp::TimerBase::SharedPtr discovery_timer_;
};

}  // namespace duatic_teleop_gamepad
