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

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "duatic_teleop_gamepad/drive_ramp.hpp"
#include "duatic_teleop_gamepad/jog_group.hpp"
#include "duatic_teleop_gamepad/stick_mapping.hpp"

namespace duatic_teleop_gamepad
{

/// Which Joy button index each function sits on.
struct ButtonMapping
{
  int dead_man_switch{ 10 };
  int move_home{ 3 };
  int move_sleep{ 1 };
  int switch_mode{ 6 };
  int gripper{ 0 };
  int wrist_rotation_left{ 7 };
  int wrist_rotation_right{ 8 };
};

/// Which Joy axis index each stick and trigger sits on.
struct AxisMapping
{
  int left_x{ 0 };
  int left_y{ 1 };
  int right_x{ 2 };
  int right_y{ 3 };
  int trigger_left{ 4 };
  int trigger_right{ 5 };
};

/// The D-Pad, and the component each direction focuses.
///
/// Both forms are read every time: Xbox-style pads report the D-Pad as a pair of axes and
/// PlayStation-style pads as four buttons, and which one a given pad uses is not known
/// ahead of time.
struct DpadMapping
{
  int axis_x{ 6 };
  int axis_y{ 7 };
  int button_up{ 11 };
  int button_down{ 12 };
  int button_left{ 13 };
  int button_right{ 14 };

  /// The component each direction focuses. An empty name leaves that direction unassigned.
  std::string focus_up{ "hip" };
  std::string focus_down;
  std::string focus_left{ "arm_right" };
  std::string focus_right{ "arm_left" };
};

struct GamepadConfig
{
  ButtonMapping buttons;
  AxisMapping axes;
  DpadMapping dpad;

  StickLimits stick;
  JogLimits jog;
  DriveLimits drive;

  /// Rate at which the joystick is read and mode logic runs, in Hz.
  double input_rate{ 100.0 };

  /// Rate at which the trajectory stream is published, in Hz.
  double jog_publish_rate{ 100.0 };

  /// Rate at which controllers, topics and the robot's shape are re-checked, in Hz.
  double discovery_rate{ 1.0 };

  /// How long a joint state stays valid after its last update, in seconds.
  double joint_state_timeout{ 0.5 };

  /// Name prefixes of the controllers this node may switch. Anything else is invisible to
  /// it and can never end up in a deactivation set.
  std::vector<std::string> managed_controllers{ "freedrive_controller", "joint_trajectory_controller",
                                                "mecanum_drive_controller", "platform_velocity_controller",
                                                "freeze_controller" };

  /// Name prefixes never to deactivate, for controllers that lose state when stopped.
  std::vector<std::string> protected_controllers{ "mecanum_drive_controller", "platform_velocity_controller" };
};

/// Declare every parameter on the node and read the result back.
GamepadConfig declare_config(rclcpp::Node& node);

/// @brief Read one axis, returning zero when the pad does not report it.
///
/// Pads report fewer axes and buttons than a config may map, and an out-of-range read in
/// the input path would throw out of the timer callback that drives teleop.
double axis_value(const sensor_msgs::msg::Joy& msg, int index);

/// Read one button, returning false when the pad does not report it.
bool button_pressed(const sensor_msgs::msg::Joy& msg, int index);

/// Collect the controls that drive jogging out of a Joy message.
StickInput read_sticks(const sensor_msgs::msg::Joy& msg, const AxisMapping& axes, const ButtonMapping& buttons);

}  // namespace duatic_teleop_gamepad
