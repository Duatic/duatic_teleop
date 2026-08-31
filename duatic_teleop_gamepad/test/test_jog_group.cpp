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

#include <gtest/gtest.h>

#include "duatic_teleop_gamepad/jog_group.hpp"

using duatic_teleop_gamepad::JogGroup;
using duatic_teleop_gamepad::JogLimits;
using duatic_teleop_gamepad::JointStateCache;

namespace
{

constexpr double kDt = 0.01;

JointStateCache states_with(const std::vector<std::string>& names, const std::vector<double>& positions)
{
  JointStateCache cache(rclcpp::get_logger("test"), rclcpp::Duration::from_seconds(1.0));

  sensor_msgs::msg::JointState msg;
  msg.name = names;
  msg.position = positions;
  cache.update(msg, rclcpp::Time(0, 0, RCL_ROS_TIME));

  return cache;
}

JogGroup two_joint_group()
{
  return JogGroup("/joint_trajectory_controller_arm_left/joint_trajectory", { "a", "b" });
}

/// Run enough ticks that a slew-limited velocity has certainly reached its target.
void settle(JogGroup& group, const JogLimits& limits, const JointStateCache& states, int ticks = 200)
{
  for (int i = 0; i < ticks; ++i) {
    group.tick(kDt, limits, states);
  }
}

}  // namespace

TEST(JogGroup, ResetSeedsTheCommandFromTheMeasuredState)
{
  auto group = two_joint_group();
  const auto states = states_with({ "a", "b" }, { 1.5, -0.5 });

  ASSERT_TRUE(group.reset(states));
  EXPECT_EQ(group.commanded_positions(), (std::vector<double>{ 1.5, -0.5 }));
  EXPECT_EQ(group.commanded_velocities(), (std::vector<double>{ 0.0, 0.0 }));
}

TEST(JogGroup, ResetRefusesWhenAJointIsMissing)
{
  auto group = two_joint_group();
  const auto states = states_with({ "a" }, { 1.5 });

  // The regression this guards: seeding a missing joint to 0.0 fabricates an absolute
  // position, and publishing it commands the arm to drive there.
  EXPECT_FALSE(group.reset(states));
}

TEST(JogGroup, TickProducesNothingUntilReset)
{
  auto group = two_joint_group();
  const auto states = states_with({ "a", "b" }, { 0.0, 0.0 });

  group.set_target_velocities({ 1.0, 0.0 });
  EXPECT_FALSE(group.tick(kDt, {}, states));
}

TEST(JogGroup, TickStopsWhenAJointDisappears)
{
  auto group = two_joint_group();
  const auto full = states_with({ "a", "b" }, { 0.0, 0.0 });
  ASSERT_TRUE(group.reset(full));

  group.set_target_velocities({ 1.0, 0.0 });
  EXPECT_TRUE(group.tick(kDt, {}, full));

  // A publisher dropping out mid-jog must not leave the command integrating blind.
  const auto partial = states_with({ "a" }, { 0.0 });
  EXPECT_FALSE(group.tick(kDt, {}, partial));
}

TEST(JogGroup, VelocityRampsAtTheAccelerationLimit)
{
  auto group = two_joint_group();
  const auto states = states_with({ "a", "b" }, { 0.0, 0.0 });
  ASSERT_TRUE(group.reset(states));

  const JogLimits limits{ 5.0, 100.0 };
  group.set_target_velocities({ 1.0, 0.0 });

  group.tick(kDt, limits, states);
  EXPECT_DOUBLE_EQ(group.commanded_velocities()[0], 0.05);

  group.tick(kDt, limits, states);
  EXPECT_DOUBLE_EQ(group.commanded_velocities()[0], 0.10);
}

TEST(JogGroup, VelocityStopsAtTheTarget)
{
  auto group = two_joint_group();
  const auto states = states_with({ "a", "b" }, { 0.0, 0.0 });
  ASSERT_TRUE(group.reset(states));

  const JogLimits limits{ 5.0, 100.0 };
  group.set_target_velocities({ 0.2, 0.0 });
  settle(group, limits, states);

  EXPECT_DOUBLE_EQ(group.commanded_velocities()[0], 0.2);
}

TEST(JogGroup, PositionIntegratesTheCommandedVelocity)
{
  auto group = two_joint_group();
  const auto states = states_with({ "a", "b" }, { 0.0, 0.0 });
  ASSERT_TRUE(group.reset(states));

  const JogLimits limits{ 1000.0, 100.0 };  // reach the target velocity in one tick
  group.set_target_velocities({ 1.0, 0.0 });

  group.tick(kDt, limits, states);
  EXPECT_DOUBLE_EQ(group.commanded_positions()[0], 1.0 * kDt);
  EXPECT_DOUBLE_EQ(group.commanded_positions()[1], 0.0);
}

TEST(JogGroup, CommandIsHeldWithinReachOfTheArm)
{
  auto group = two_joint_group();
  const auto states = states_with({ "a", "b" }, { 0.0, 0.0 });
  ASSERT_TRUE(group.reset(states));

  const JogLimits limits{ 1000.0, 0.1 };
  group.set_target_velocities({ 1.0, 0.0 });

  // The arm never moves, so the command runs up against the offset limit and stays there.
  settle(group, limits, states);

  EXPECT_DOUBLE_EQ(group.commanded_positions()[0], 0.1);
  EXPECT_TRUE(group.lagging());
}

TEST(JogGroup, LaggingClearsOnceTheArmCatchesUp)
{
  auto group = two_joint_group();
  auto states = states_with({ "a", "b" }, { 0.0, 0.0 });
  ASSERT_TRUE(group.reset(states));

  const JogLimits limits{ 1000.0, 0.1 };
  group.set_target_velocities({ 1.0, 0.0 });
  settle(group, limits, states);
  ASSERT_TRUE(group.lagging());

  const auto caught_up = states_with({ "a", "b" }, { 0.1, 0.0 });
  group.tick(kDt, limits, caught_up);
  EXPECT_FALSE(group.lagging());
}

TEST(JogGroup, ReleaseRampsDownRatherThanStopping)
{
  auto group = two_joint_group();
  const auto states = states_with({ "a", "b" }, { 0.0, 0.0 });
  ASSERT_TRUE(group.reset(states));

  const JogLimits limits{ 5.0, 100.0 };
  group.set_target_velocities({ 1.0, 0.0 });
  settle(group, limits, states);
  ASSERT_DOUBLE_EQ(group.commanded_velocities()[0], 1.0);

  group.release();
  group.tick(kDt, limits, states);

  EXPECT_DOUBLE_EQ(group.commanded_velocities()[0], 0.95);
}

TEST(JogGroup, PublishesOnceMoreAfterMotionStops)
{
  auto group = two_joint_group();
  const auto states = states_with({ "a", "b" }, { 0.0, 0.0 });
  ASSERT_TRUE(group.reset(states));

  const JogLimits limits{ 1000.0, 100.0 };

  EXPECT_FALSE(group.tick(kDt, limits, states));  // idle from the start, nothing to say

  group.set_target_velocities({ 1.0, 0.0 });
  EXPECT_TRUE(group.tick(kDt, limits, states));

  group.release();
  EXPECT_TRUE(group.tick(kDt, limits, states));   // the stop itself
  EXPECT_FALSE(group.tick(kDt, limits, states));  // and nothing after it
}

TEST(JogGroup, ShorterTargetListLeavesRemainingJointsStill)
{
  auto group = two_joint_group();
  const auto states = states_with({ "a", "b" }, { 0.0, 0.0 });
  ASSERT_TRUE(group.reset(states));

  const JogLimits limits{ 1000.0, 100.0 };
  group.set_target_velocities({ 1.0 });
  group.tick(kDt, limits, states);

  EXPECT_DOUBLE_EQ(group.commanded_velocities()[1], 0.0);
  EXPECT_DOUBLE_EQ(group.commanded_positions()[1], 0.0);
}
