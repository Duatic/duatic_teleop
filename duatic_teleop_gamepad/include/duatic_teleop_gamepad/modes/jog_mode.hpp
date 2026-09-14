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

#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "duatic_teleop_gamepad/modes/base_mode.hpp"

namespace duatic_teleop_gamepad
{

/// The jog command for one joint trajectory controller.
///
/// Holds the streamed position command and advances it once per publish period: the
/// velocity ramps towards what the sticks ask for, the position integrates that velocity,
/// and the result is held within reach of where the arm actually is.
class JogGroup
{
public:
  explicit JogGroup(std::vector<std::string> joints);

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
  std::vector<std::string> joints_;

  std::vector<double> commanded_positions_;
  std::vector<double> commanded_velocities_;
  std::vector<double> target_velocities_;

  bool ready_{ false };
  bool idle_{ true };
  bool lagging_{ false };
};

/// Jogs the focused component's joints straight from the sticks.
///
/// One group per trajectory controller the robot spawned, of which only the focused one is
/// ever commanded. The others keep their command seeded from the measured state, so
/// focusing one is not the moment its command has to be built.
class JogMode : public BaseMode
{
public:
  explicit JogMode(TeleopContext context);

  std::string name() const override;
  const std::vector<std::string>& controller_bases() const override;
  void on_input(const GamepadInput& input, double dt) override;
  void publish(double dt) override;
  void reset() override;
  void set_focus(const std::string& component) override;
  void on_robot_changed() override;
  double feedback() const override;

private:
  /// A jog group and the publisher its commands go out on, kept together so the two can
  /// never fall out of step.
  struct Target
  {
    JogGroup group;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher;
  };

  /// The target for the focused component, or nullptr when there is none.
  Target* focused();

  TeleopContext context_;

  /// Keyed by the component each group drives, which is what focus selects.
  std::map<std::string, Target> targets_;
  std::string focus_;
  double feedback_{ 0.0 };
};

}  // namespace duatic_teleop_gamepad
