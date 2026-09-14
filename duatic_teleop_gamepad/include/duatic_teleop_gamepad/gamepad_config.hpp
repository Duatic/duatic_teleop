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

#include "duatic_teleop_gamepad/input/gamepad_input.hpp"

namespace duatic_teleop_gamepad
{

struct JogLimits
{
  /// Slew-rate limit on the commanded joint velocity, in rad/s^2.
  double max_acceleration{ 5.0 };

  /// How far the commanded position may run ahead of the measured one, in rad. Reaching it
  /// means the arm is not keeping up, and the command is held back to stay within it.
  double max_position_offset{ 0.1 };
};

struct DriveLimits
{
  /// Platform speed at full stick deflection, in m/s and rad/s.
  double max_velocity{ 0.6 };

  double acceleration{ 0.5 };

  /// Kept above acceleration so the platform can always shed speed faster than it gains it.
  double deceleration{ 1.0 };

  double deadzone{ 0.05 };
};

/// Everything the node reads from parameters.
///
/// The limits live here rather than beside the code that applies them, because every one of
/// them is a ROS parameter, and because a mode header that had to reach back into this one
/// for them could not also be included by it.
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

}  // namespace duatic_teleop_gamepad
