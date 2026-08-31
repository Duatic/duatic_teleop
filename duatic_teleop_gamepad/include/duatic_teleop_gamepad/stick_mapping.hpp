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

#include <cstddef>
#include <vector>

namespace duatic_teleop_gamepad
{

/// The gamepad controls that drive joint jogging, already read off the Joy message.
struct StickInput
{
  double left_x{ 0.0 };
  double left_y{ 0.0 };
  double right_x{ 0.0 };
  double right_y{ 0.0 };
  double trigger_left{ 0.0 };
  double trigger_right{ 0.0 };
  bool wrist_left{ false };
  bool wrist_right{ false };
};

struct StickLimits
{
  /// Joint velocity commanded at full deflection, in rad/s.
  double max_velocity{ 1.0 };

  /// Deflection below which an axis reads as centred.
  double deadzone{ 0.1 };

  /// Deflection the quieter axis of a stick must exceed while the other one leads, which
  /// is what stops a diagonal push creeping both of that stick's joints at once. Set it
  /// equal to deadzone to let both axes drive together.
  double dominant_axis_threshold{ 0.6 };
};

/// @brief Work out the velocity each joint should ramp towards.
/// @param joint_count Number of joints the controller drives; the first six are mapped and
///   any beyond that stay still.
/// @return One velocity per joint, in the controller's own joint order.
std::vector<double> stick_to_velocities(const StickInput& input, std::size_t joint_count, const StickLimits& limits);

}  // namespace duatic_teleop_gamepad
