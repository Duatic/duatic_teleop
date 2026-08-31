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

#include "duatic_teleop_gamepad/stick_mapping.hpp"

#include <cmath>

namespace duatic_teleop_gamepad
{

namespace
{

double beyond(double deflection, double deadzone, double max_velocity)
{
  return std::abs(deflection) > deadzone ? deflection * max_velocity : 0.0;
}

/// @brief Deadzone for one axis of a stick, raised while the other axis of that stick leads.
/// @param self Deflection of the axis being mapped.
/// @param other Deflection of the other axis of the same stick.
double effective_deadzone(double self, double other, const StickLimits& limits)
{
  return std::abs(other) > std::abs(self) ? limits.dominant_axis_threshold : limits.deadzone;
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

std::vector<double> stick_to_velocities(const StickInput& input, std::size_t joint_count, const StickLimits& limits)
{
  std::vector<double> velocities(joint_count, 0.0);

  for (std::size_t joint = 0; joint < joint_count; ++joint) {
    switch (joint) {
      case 0:
        velocities[joint] =
            beyond(input.left_x, effective_deadzone(input.left_x, input.left_y, limits), limits.max_velocity);
        break;
      case 1:
        velocities[joint] =
            beyond(input.left_y, effective_deadzone(input.left_y, input.left_x, limits), limits.max_velocity);
        break;
      case 2:
        velocities[joint] =
            beyond(input.right_y, effective_deadzone(input.right_y, input.right_x, limits), limits.max_velocity);
        break;
      case 3:
        velocities[joint] =
            beyond(input.right_x, effective_deadzone(input.right_x, input.right_y, limits), limits.max_velocity);
        break;
      case 4:
        // The triggers work against each other on one joint, so neither can lead the other
        // in the sense the sticks do.
        velocities[joint] = beyond(input.trigger_right - input.trigger_left, limits.deadzone, limits.max_velocity);
        break;
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
