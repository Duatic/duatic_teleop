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

#include "duatic_teleop_gamepad/joint_state_cache.hpp"

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

/// The jog command for one joint trajectory controller.
///
/// Holds the streamed position command and advances it once per publish period: the
/// velocity ramps towards what the sticks ask for, the position integrates that velocity,
/// and the result is held within reach of where the arm actually is.
class JogGroup
{
public:
  JogGroup(std::string topic, std::vector<std::string> joints);

  const std::string& topic() const;
  const std::vector<std::string>& joints() const;

  /// @brief Seed the command from the measured state and stop all motion.
  /// @return false if any joint is missing from the states, in which case the group holds
  ///   no usable command and tick() will not produce one.
  bool reset(const JointStateCache& states);

  /// Set the velocity each joint ramps towards, in the controller's joint order. A shorter
  /// list than the group has joints leaves the remaining joints stationary.
  void set_target_velocities(const std::vector<double>& velocities);

  /// Ramp every joint down to a stop, as releasing the deadman does.
  void release();

  /// @brief Advance the command by one publish period.
  /// @param dt Publish period in seconds.
  /// @param states Measured joint positions, used to hold the command within reach.
  /// @return true if a trajectory should be published this tick. Stays true for one tick
  ///   after motion stops, so the controller is told about the stop.
  bool tick(double dt, const JogLimits& limits, const JointStateCache& states);

  const std::vector<double>& commanded_positions() const;
  const std::vector<double>& commanded_velocities() const;

  /// Whether the arm is failing to keep up, i.e. the command is being held back.
  bool lagging() const;

private:
  std::string topic_;
  std::vector<std::string> joints_;

  std::vector<double> commanded_positions_;
  std::vector<double> commanded_velocities_;
  std::vector<double> target_velocities_;

  bool ready_{ false };
  bool idle_{ true };
  bool lagging_{ false };
};

}  // namespace duatic_teleop_gamepad
