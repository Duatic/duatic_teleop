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

#include "duatic_teleop_gamepad/drive_ramp.hpp"

#include <algorithm>
#include <cmath>

namespace duatic_teleop_gamepad
{

namespace
{

/// @brief Reduce one stick axis to a usable deflection.
///
/// Sanitising here rather than at every arithmetic step is what lets the ramp below stay
/// plain arithmetic: a joystick driver that emits a NaN cannot reach it.
double deflection(double value, double deadzone)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }

  const double clamped = std::clamp(value, -1.0, 1.0);
  return std::abs(clamped) < deadzone ? 0.0 : clamped;
}

}  // namespace

double ramp_velocity(double current, double target, double dt, const DriveLimits& limits)
{
  // Reversing counts as slowing: it has to pass through a stop, and the part before the
  // stop should shed speed at the deceleration limit rather than the acceleration one.
  const bool slowing = std::abs(target) < std::abs(current) || target * current < 0.0;
  const double max_step = (slowing ? limits.deceleration : limits.acceleration) * dt;
  const double error = target - current;

  if (std::abs(error) <= max_step) {
    return target;
  }

  return current + std::copysign(max_step, error);
}

const DriveCommand& DriveRamp::advance(const StickInput& input, double dt, const DriveLimits& limits)
{
  const double forward = deflection(input.left_y, limits.deadzone) * limits.max_velocity;
  const double strafe = deflection(input.left_x, limits.deadzone) * limits.max_velocity;
  const double turn = deflection(input.right_x, limits.deadzone) * limits.max_velocity;

  command_.linear_x = ramp_velocity(command_.linear_x, forward, dt, limits);
  command_.linear_y = ramp_velocity(command_.linear_y, strafe, dt, limits);
  command_.angular_z = ramp_velocity(command_.angular_z, turn, dt, limits);

  return command_;
}

const DriveCommand& DriveRamp::stop()
{
  command_ = DriveCommand{};
  return command_;
}

const DriveCommand& DriveRamp::command() const
{
  return command_;
}

}  // namespace duatic_teleop_gamepad
