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

namespace duatic_teleop_gamepad
{

namespace
{

/// @brief Declare a rate parameter, keeping the default when the value is not a frequency.
///
/// Every rate ends up as a timer period and as the dt the ramps integrate over, and a zero
/// or negative one turns both into nonsense, so a bad value is refused at the one place it
/// enters the node rather than guarded at each use.
double declare_rate(rclcpp::Node& node, const std::string& name, double fallback)
{
  const double value = node.declare_parameter(name, fallback);
  if (value > 0.0) {
    return value;
  }

  RCLCPP_ERROR(node.get_logger(), "%s must be greater than zero, ignoring %.3f and using %.1f Hz", name.c_str(), value,
               fallback);
  return fallback;
}

}  // namespace

GamepadConfig declare_config(rclcpp::Node& node)
{
  // Each parameter's default is the struct's own initializer, so the defaults exist once in
  // C++ rather than once here and once in the type.
  GamepadConfig config;

  config.buttons.dead_man_switch =
      node.declare_parameter("button_mapping.dead_man_switch", config.buttons.dead_man_switch);
  config.buttons.switch_mode = node.declare_parameter("button_mapping.switch_controller", config.buttons.switch_mode);
  config.buttons.wrist_rotation_left =
      node.declare_parameter("button_mapping.wrist_rotation_left", config.buttons.wrist_rotation_left);
  config.buttons.wrist_rotation_right =
      node.declare_parameter("button_mapping.wrist_rotation_right", config.buttons.wrist_rotation_right);

  config.axes.left_x = node.declare_parameter("axis_mapping.left_joystick.x", config.axes.left_x);
  config.axes.left_y = node.declare_parameter("axis_mapping.left_joystick.y", config.axes.left_y);
  config.axes.right_x = node.declare_parameter("axis_mapping.right_joystick.x", config.axes.right_x);
  config.axes.right_y = node.declare_parameter("axis_mapping.right_joystick.y", config.axes.right_y);
  config.axes.trigger_left = node.declare_parameter("axis_mapping.triggers.left", config.axes.trigger_left);
  config.axes.trigger_right = node.declare_parameter("axis_mapping.triggers.right", config.axes.trigger_right);

  config.dpad.axis_x = node.declare_parameter("dpad_mapping.axes.x", config.dpad.axis_x);
  config.dpad.axis_y = node.declare_parameter("dpad_mapping.axes.y", config.dpad.axis_y);
  config.dpad.button_up = node.declare_parameter("dpad_mapping.buttons.up", config.dpad.button_up);
  config.dpad.button_down = node.declare_parameter("dpad_mapping.buttons.down", config.dpad.button_down);
  config.dpad.button_left = node.declare_parameter("dpad_mapping.buttons.left", config.dpad.button_left);
  config.dpad.button_right = node.declare_parameter("dpad_mapping.buttons.right", config.dpad.button_right);

  config.dpad.focus_up = node.declare_parameter("dpad_mapping.focus_targets.up", config.dpad.focus_up);
  config.dpad.focus_down = node.declare_parameter("dpad_mapping.focus_targets.down", config.dpad.focus_down);
  config.dpad.focus_left = node.declare_parameter("dpad_mapping.focus_targets.left", config.dpad.focus_left);
  config.dpad.focus_right = node.declare_parameter("dpad_mapping.focus_targets.right", config.dpad.focus_right);

  config.stick.max_velocity = node.declare_parameter("jog.max_velocity", config.stick.max_velocity);
  config.stick.deadzone = node.declare_parameter("jog.deadzone", config.stick.deadzone);
  config.stick.dominant_axis_threshold =
      node.declare_parameter("jog.dominant_axis_threshold", config.stick.dominant_axis_threshold);
  config.jog.max_acceleration = node.declare_parameter("jog.max_acceleration", config.jog.max_acceleration);
  config.jog.max_position_offset = node.declare_parameter("jog.max_position_offset", config.jog.max_position_offset);

  config.drive.max_velocity = node.declare_parameter("drive.max_velocity", config.drive.max_velocity);
  config.drive.acceleration = node.declare_parameter("drive.acceleration", config.drive.acceleration);
  config.drive.deceleration = node.declare_parameter("drive.deceleration", config.drive.deceleration);
  config.drive.deadzone = node.declare_parameter("drive.deadzone", config.drive.deadzone);

  config.input_rate = declare_rate(node, "input_rate", config.input_rate);
  config.jog_publish_rate = declare_rate(node, "jog_publish_rate", config.jog_publish_rate);
  config.discovery_rate = declare_rate(node, "discovery_rate", config.discovery_rate);
  config.joint_state_timeout = node.declare_parameter("joint_state_timeout", config.joint_state_timeout);

  config.managed_controllers = node.declare_parameter("managed_controllers", config.managed_controllers);
  config.protected_controllers = node.declare_parameter("protected_controllers", config.protected_controllers);

  return config;
}

}  // namespace duatic_teleop_gamepad
