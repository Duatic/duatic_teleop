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

#include "duatic_teleop_gamepad/modes/jog_mode.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "duatic_teleop_gamepad/robot/jtc_discovery.hpp"

namespace duatic_teleop_gamepad
{

JogGroup::JogGroup(std::vector<std::string> joints) : joints_(std::move(joints))
{
  commanded_positions_.assign(joints_.size(), 0.0);
  commanded_velocities_.assign(joints_.size(), 0.0);
  target_velocities_.assign(joints_.size(), 0.0);
}

const std::vector<std::string>& JogGroup::joints() const
{
  return joints_;
}

bool JogGroup::reset(const JointStateCache& states)
{
  std::fill(commanded_velocities_.begin(), commanded_velocities_.end(), 0.0);
  std::fill(target_velocities_.begin(), target_velocities_.end(), 0.0);
  idle_ = true;
  lagging_ = false;

  // Every command is an absolute position measured from where the arm is now, so without
  // the measured state there is nothing to seed from. Refusing to become ready is what
  // keeps a fabricated position from being commanded.
  if (!states.has_all(joints_)) {
    ready_ = false;
    return false;
  }

  for (std::size_t i = 0; i < joints_.size(); ++i) {
    commanded_positions_[i] = states.position(joints_[i]).value();
  }

  ready_ = true;
  return true;
}

void JogGroup::set_target_velocities(const std::vector<double>& velocities)
{
  const auto count = std::min(velocities.size(), target_velocities_.size());

  std::copy(velocities.begin(), velocities.begin() + static_cast<std::ptrdiff_t>(count), target_velocities_.begin());
  std::fill(target_velocities_.begin() + static_cast<std::ptrdiff_t>(count), target_velocities_.end(), 0.0);
}

void JogGroup::release()
{
  std::fill(target_velocities_.begin(), target_velocities_.end(), 0.0);
}

bool JogGroup::tick(double dt, const JogLimits& limits, const JointStateCache& states)
{
  if (!ready_ || !states.has_all(joints_)) {
    return false;
  }

  const double max_velocity_step = limits.max_acceleration * dt;
  bool moving = false;
  lagging_ = false;

  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const double velocity_error = target_velocities_[i] - commanded_velocities_[i];
    commanded_velocities_[i] += std::clamp(velocity_error, -max_velocity_step, max_velocity_step);

    if (commanded_velocities_[i] == 0.0) {
      continue;
    }

    commanded_positions_[i] += commanded_velocities_[i] * dt;
    moving = true;

    const double measured = states.position(joints_[i]).value();
    const double offset = commanded_positions_[i] - measured;

    if (std::abs(offset) > limits.max_position_offset) {
      commanded_positions_[i] = measured + std::copysign(limits.max_position_offset, offset);
      lagging_ = true;
    }
  }

  // One more publish after motion stops, so the controller is told about the stop rather
  // than being left holding the last moving command.
  const bool publish = moving || !idle_;
  idle_ = !moving;
  return publish;
}

const std::vector<double>& JogGroup::commanded_positions() const
{
  return commanded_positions_;
}

const std::vector<double>& JogGroup::commanded_velocities() const
{
  return commanded_velocities_;
}

bool JogGroup::lagging() const
{
  return lagging_;
}

JogMode::JogMode(TeleopContext context) : context_(context)
{
}

std::string JogMode::name() const
{
  return "jog";
}

const std::vector<std::string>& JogMode::controller_bases() const
{
  static const std::vector<std::string> bases = { kTrajectoryControllerBase };
  return bases;
}

void JogMode::on_input(const GamepadInput& input, [[maybe_unused]] double dt)
{
  auto* target = focused();
  if (target == nullptr) {
    return;
  }

  if (!input.motion_allowed) {
    target->group.release();
    return;
  }

  target->group.set_target_velocities(
      stick_to_velocities(input.sticks, target->group.joints().size(), context_.config.stick));
}

void JogMode::publish(double dt)
{
  feedback_ = 0.0;

  auto* target = focused();
  if (target == nullptr || !target->group.tick(dt, context_.config.jog, context_.joint_states)) {
    return;
  }

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = target->group.joints();

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = target->group.commanded_positions();
  point.velocities = target->group.commanded_velocities();
  point.time_from_start = rclcpp::Duration::from_seconds(dt);
  trajectory.points.push_back(std::move(point));

  target->publisher->publish(trajectory);

  // The arm failing to keep up is the one thing the operator cannot see from the stick.
  feedback_ = target->group.lagging() ? 1.0 : 0.0;
}

void JogMode::reset()
{
  feedback_ = 0.0;

  for (auto& entry : targets_) {
    entry.second.group.reset(context_.joint_states);
  }
}

void JogMode::set_focus(const std::string& component)
{
  focus_ = component;
}

void JogMode::on_robot_changed()
{
  targets_.clear();

  for (const auto& target : context_.discovery.targets()) {
    targets_.emplace(target.component,
                     Target{ JogGroup(target.joints), context_.node.create_publisher<
                                                          trajectory_msgs::msg::JointTrajectory>(target.topic, 10) });
  }
}

double JogMode::feedback() const
{
  return feedback_;
}

JogMode::Target* JogMode::focused()
{
  const auto match = targets_.find(focus_);
  return match == targets_.end() ? nullptr : &match->second;
}

}  // namespace duatic_teleop_gamepad
