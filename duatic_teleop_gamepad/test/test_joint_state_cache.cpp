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

#include "duatic_teleop_gamepad/joint_state_cache.hpp"

using duatic_teleop_gamepad::JointStateCache;

namespace
{

sensor_msgs::msg::JointState make_state(const std::vector<std::string>& names, const std::vector<double>& positions)
{
  sensor_msgs::msg::JointState msg;
  msg.name = names;
  msg.position = positions;
  return msg;
}

rclcpp::Time at(double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
}

JointStateCache make_cache(double stale_after_seconds = 1.0)
{
  return JointStateCache(rclcpp::get_logger("test"), rclcpp::Duration::from_seconds(stale_after_seconds));
}

}  // namespace

TEST(JointStateCache, MergesPublishersDescribingDifferentJoints)
{
  auto cache = make_cache();

  cache.update(make_state({ "arm_left/shoulder_lift" }, { 1.0 }), at(0.0));
  cache.update(make_state({ "gripper_left/finger" }, { 0.5 }), at(0.0));

  EXPECT_EQ(cache.position("arm_left/shoulder_lift"), 1.0);
  EXPECT_EQ(cache.position("gripper_left/finger"), 0.5);
}

TEST(JointStateCache, PartialMessageDoesNotEvictAnotherPublishersJoints)
{
  auto cache = make_cache();
  cache.update(make_state({ "a", "b" }, { 1.0, 2.0 }), at(0.0));

  // The regression this guards: a cache that assigned rather than merged would be left
  // holding only "c" and the arm would vanish mid-jog.
  cache.update(make_state({ "c" }, { 3.0 }), at(0.1));

  EXPECT_EQ(cache.position("a"), 1.0);
  EXPECT_EQ(cache.position("b"), 2.0);
  EXPECT_EQ(cache.position("c"), 3.0);
}

TEST(JointStateCache, LaterMessageOverwritesTheSameJoint)
{
  auto cache = make_cache();

  cache.update(make_state({ "a" }, { 1.0 }), at(0.0));
  cache.update(make_state({ "a" }, { 2.0 }), at(0.1));

  EXPECT_EQ(cache.position("a"), 2.0);
}

TEST(JointStateCache, IgnoresMessagesWhoseArraysDisagree)
{
  auto cache = make_cache();
  cache.update(make_state({ "a" }, { 1.0 }), at(0.0));

  cache.update(make_state({ "a", "b" }, { 9.0 }), at(0.1));

  EXPECT_EQ(cache.position("a"), 1.0);
  EXPECT_FALSE(cache.position("b").has_value());
}

TEST(JointStateCache, IgnoresMessagesCarryingNoPositions)
{
  auto cache = make_cache();
  cache.update(make_state({ "a" }, { 1.0 }), at(0.0));

  // Legal per sensor_msgs/JointState: a publisher may send only velocity or effort.
  cache.update(make_state({ "a" }, {}), at(0.1));

  EXPECT_EQ(cache.position("a"), 1.0);
}

TEST(JointStateCache, ExpiresEntriesWhosePublisherStopped)
{
  auto cache = make_cache(1.0);
  cache.update(make_state({ "a", "b" }, { 1.0, 2.0 }), at(0.0));

  // Only "a" keeps being published.
  cache.update(make_state({ "a" }, { 1.1 }), at(0.9));

  // "b" was last seen at 0.0, so it survives until the window closes at 1.0.
  EXPECT_FALSE(cache.expire(at(0.95)));
  EXPECT_TRUE(cache.expire(at(1.5)));

  EXPECT_EQ(cache.position("a"), 1.1);
  EXPECT_FALSE(cache.position("b").has_value());
}

TEST(JointStateCache, HasAllRequiresEveryJoint)
{
  auto cache = make_cache();
  cache.update(make_state({ "a", "b" }, { 1.0, 2.0 }), at(0.0));

  EXPECT_TRUE(cache.has_all({ "a", "b" }));
  EXPECT_FALSE(cache.has_all({ "a", "c" }));
  EXPECT_TRUE(cache.has_all({}));
}
