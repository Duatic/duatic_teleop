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

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "duatic_teleop_gamepad/modes/base_mode.hpp"

namespace duatic_teleop_gamepad
{

struct DriveCommand
{
  double linear_x{ 0.0 };
  double linear_y{ 0.0 };
  double angular_z{ 0.0 };
};

/// @brief Move one velocity component towards its target within the acceleration limits.
/// @param dt Time since the last step, in seconds.
double ramp_velocity(double current, double target, double dt, const DriveLimits& limits);

/// The platform velocity command, ramped rather than followed straight from the sticks.
class DriveRamp
{
public:
  /// Advance the command towards what the sticks are asking for.
  const DriveCommand& advance(const StickInput& input, double dt, const DriveLimits& limits);

  /// @brief Drop to a standstill at once, bypassing the deceleration limit.
  ///
  /// Releasing the deadman or hitting the E-Stop has to stop the platform now, not over a
  /// ramp; the smoothing exists for comfort while driving, not for stopping.
  const DriveCommand& stop();

private:
  DriveCommand command_;
};

/// Drives the mobile platform from the sticks.
///
/// Commands go out on the input tick rather than on the publish tick: the platform follows
/// a velocity until it is given another one, so there is nothing to stream, and the ramp's
/// dt is the interval the sticks were read over.
class DriveMode : public BaseMode
{
public:
  explicit DriveMode(TeleopContext context);

  std::string name() const override;
  const std::vector<std::string>& controller_bases() const override;
  std::vector<std::string> required_controllers(const ControllerSnapshot& controllers) const override;
  void on_input(const GamepadInput& input, double dt) override;
  void reset() override;

private:
  void publish_twist(const DriveCommand& command);

  TeleopContext context_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  DriveRamp ramp_;

  /// Whether a command has gone out since the last reset, which is what decides if a
  /// stopping one is owed.
  bool driving_{ false };
};

}  // namespace duatic_teleop_gamepad
