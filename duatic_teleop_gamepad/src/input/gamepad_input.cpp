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

#include "duatic_teleop_gamepad/input/gamepad_input.hpp"

#include <algorithm>
#include <cmath>

namespace duatic_teleop_gamepad
{

namespace
{

/// Deflection past which a D-Pad reported as an axis counts as held.
constexpr double kDpadAxisThreshold = 0.5;

double beyond(double deflection, double deadzone, double max_velocity)
{
  return std::abs(deflection) > deadzone ? deflection * max_velocity : 0.0;
}

/// @brief Deadzone for one axis of a stick, raised while the other axis of that stick is
/// committed.
///
/// Which axis leads is decided against the fixed threshold rather than against the other
/// axis' deflection, because the two axes of a stick held on a diagonal sit within noise of
/// each other and comparing them makes the lead change hands every reading.
///
/// @param other Deflection of the other axis of the same stick.
double effective_deadzone(double other, const StickLimits& limits)
{
  return std::abs(other) > limits.dominant_axis_threshold ? limits.dominant_axis_threshold : limits.deadzone;
}

double wrist_deflection(const StickInput& input)
{
  if (input.wrist_left) {
    return -1.0;
  }
  if (input.wrist_right) {
    return 1.0;
  }
  return 0.0;
}

}  // namespace

double axis_value(const sensor_msgs::msg::Joy& msg, int index)
{
  if (index < 0 || static_cast<std::size_t>(index) >= msg.axes.size()) {
    return 0.0;
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

std::string dpad_focus(const sensor_msgs::msg::Joy& msg, const DpadMapping& dpad)
{
  const double axis_x = axis_value(msg, dpad.axis_x);
  const double axis_y = axis_value(msg, dpad.axis_y);

  if (button_pressed(msg, dpad.button_up) || axis_y > kDpadAxisThreshold) {
    return dpad.focus_up;
  }
  if (button_pressed(msg, dpad.button_down) || axis_y < -kDpadAxisThreshold) {
    return dpad.focus_down;
  }
  if (button_pressed(msg, dpad.button_left) || axis_x > kDpadAxisThreshold) {
    return dpad.focus_left;
  }
  if (button_pressed(msg, dpad.button_right) || axis_x < -kDpadAxisThreshold) {
    return dpad.focus_right;
  }

  return {};
}

GamepadInput read_input(const sensor_msgs::msg::Joy& msg, const ButtonMapping& buttons, const AxisMapping& axes,
                        const DpadMapping& dpad)
{
  GamepadInput input;

  input.sticks = read_sticks(msg, axes, buttons);
  input.deadman = button_pressed(msg, buttons.dead_man_switch);
  input.switch_mode = button_pressed(msg, buttons.switch_mode);
  input.focus_request = dpad_focus(msg, dpad);

  return input;
}

std::vector<double> stick_to_velocities(const StickInput& input, std::size_t joint_count, const StickLimits& limits)
{
  std::vector<double> velocities(joint_count, 0.0);

  for (std::size_t joint = 0; joint < joint_count; ++joint) {
    switch (joint) {
      case 0:
        velocities[joint] = beyond(input.left_x, effective_deadzone(input.left_y, limits), limits.max_velocity);
        break;
      case 1:
        velocities[joint] = beyond(input.left_y, effective_deadzone(input.left_x, limits), limits.max_velocity);
        break;
      case 2:
        velocities[joint] = beyond(input.right_y, effective_deadzone(input.right_x, limits), limits.max_velocity);
        break;
      case 3:
        velocities[joint] = beyond(input.right_x, effective_deadzone(input.right_y, limits), limits.max_velocity);
        break;
      case 4: {
        // The triggers work against each other on one joint, so neither can lead the other
        // in the sense the sticks do. Their difference spans twice what a single axis does,
        // and is clamped back to one axis' worth so full opposing triggers command the
        // configured maximum rather than double it.
        const double opposed = std::clamp(input.trigger_right - input.trigger_left, -1.0, 1.0);
        velocities[joint] = beyond(opposed, limits.deadzone, limits.max_velocity);
        break;
      }
      case 5:
        velocities[joint] = beyond(wrist_deflection(input), limits.deadzone, limits.max_velocity);
        break;
      default:
        break;
    }
  }

  return velocities;
}

}  // namespace duatic_teleop_gamepad
