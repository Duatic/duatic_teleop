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

#include "duatic_teleop_gamepad/gamepad_config.hpp"

#include <cstddef>

namespace duatic_teleop_gamepad
{

GamepadConfig declare_config(rclcpp::Node& node)
{
  GamepadConfig config;

  config.buttons.dead_man_switch = node.declare_parameter("button_mapping.dead_man_switch", 10);
  config.buttons.move_home = node.declare_parameter("button_mapping.move_home", 3);
  config.buttons.move_sleep = node.declare_parameter("button_mapping.move_sleep", 1);
  config.buttons.switch_mode = node.declare_parameter("button_mapping.switch_controller", 6);
  config.buttons.gripper = node.declare_parameter("button_mapping.gripper_control", 0);
  config.buttons.wrist_rotation_left = node.declare_parameter("button_mapping.wrist_rotation_left", 7);
  config.buttons.wrist_rotation_right = node.declare_parameter("button_mapping.wrist_rotation_right", 8);

  config.axes.left_x = node.declare_parameter("axis_mapping.left_joystick.x", 0);
  config.axes.left_y = node.declare_parameter("axis_mapping.left_joystick.y", 1);
  config.axes.right_x = node.declare_parameter("axis_mapping.right_joystick.x", 2);
  config.axes.right_y = node.declare_parameter("axis_mapping.right_joystick.y", 3);
  config.axes.trigger_left = node.declare_parameter("axis_mapping.triggers.left", 4);
  config.axes.trigger_right = node.declare_parameter("axis_mapping.triggers.right", 5);

  config.dpad.axis_x = node.declare_parameter("dpad_mapping.axes.x", 6);
  config.dpad.axis_y = node.declare_parameter("dpad_mapping.axes.y", 7);
  config.dpad.button_up = node.declare_parameter("dpad_mapping.buttons.up", 11);
  config.dpad.button_down = node.declare_parameter("dpad_mapping.buttons.down", 12);
  config.dpad.button_left = node.declare_parameter("dpad_mapping.buttons.left", 13);
  config.dpad.button_right = node.declare_parameter("dpad_mapping.buttons.right", 14);

  config.dpad.focus_up = node.declare_parameter<std::string>("dpad_mapping.focus_targets.up", "hip");
  config.dpad.focus_down = node.declare_parameter<std::string>("dpad_mapping.focus_targets.down", "platform");
  config.dpad.focus_left = node.declare_parameter<std::string>("dpad_mapping.focus_targets.left", "arm_right");
  config.dpad.focus_right = node.declare_parameter<std::string>("dpad_mapping.focus_targets.right", "arm_left");

  config.stick.max_velocity = node.declare_parameter("jog.max_velocity", 1.0);
  config.stick.deadzone = node.declare_parameter("jog.deadzone", 0.1);
  config.stick.dominant_axis_threshold = node.declare_parameter("jog.dominant_axis_threshold", 0.6);
  config.jog.max_acceleration = node.declare_parameter("jog.max_acceleration", 5.0);
  config.jog.max_position_offset = node.declare_parameter("jog.max_position_offset", 0.1);

  config.drive.max_velocity = node.declare_parameter("drive.max_velocity", 0.6);
  config.drive.acceleration = node.declare_parameter("drive.acceleration", 0.5);
  config.drive.deceleration = node.declare_parameter("drive.deceleration", 1.0);
  config.drive.deadzone = node.declare_parameter("drive.deadzone", 0.05);

  config.input_rate = node.declare_parameter("input_rate", 100.0);
  config.jog_publish_rate = node.declare_parameter("jog_publish_rate", 100.0);
  config.discovery_rate = node.declare_parameter("discovery_rate", 1.0);
  config.joint_state_timeout = node.declare_parameter("joint_state_timeout", 0.5);

  config.managed_controllers = node.declare_parameter<std::vector<std::string>>(
      "managed_controllers", { "freedrive_controller", "joint_trajectory_controller", "mecanum_drive_controller",
                               "platform_velocity_controller", "freeze_controller" });
  config.protected_controllers = node.declare_parameter<std::vector<std::string>>(
      "protected_controllers", { "mecanum_drive_controller", "platform_velocity_controller" });

  return config;
}

double axis_value(const sensor_msgs::msg::Joy& msg, int index, double fallback)
{
  if (index < 0 || static_cast<std::size_t>(index) >= msg.axes.size()) {
    return fallback;
  }

  return msg.axes[static_cast<std::size_t>(index)];
}

bool button_pressed(const sensor_msgs::msg::Joy& msg, int index)
{
  if (index < 0 || static_cast<std::size_t>(index) >= msg.buttons.size()) {
    return false;
  }

  return msg.buttons[static_cast<std::size_t>(index)] != 0;
}

StickInput read_sticks(const sensor_msgs::msg::Joy& msg, const AxisMapping& axes, const ButtonMapping& buttons)
{
  StickInput input;

  input.left_x = axis_value(msg, axes.left_x);
  input.left_y = axis_value(msg, axes.left_y);
  input.right_x = axis_value(msg, axes.right_x);
  input.right_y = axis_value(msg, axes.right_y);
  input.trigger_left = axis_value(msg, axes.trigger_left);
  input.trigger_right = axis_value(msg, axes.trigger_right);
  input.wrist_left = button_pressed(msg, buttons.wrist_rotation_left);
  input.wrist_right = button_pressed(msg, buttons.wrist_rotation_right);

  return input;
}

}  // namespace duatic_teleop_gamepad
