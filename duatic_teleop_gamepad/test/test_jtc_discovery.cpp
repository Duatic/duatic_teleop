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

#include "duatic_teleop_gamepad/jtc_discovery.hpp"

using duatic_teleop_gamepad::JtcTopic;
using duatic_teleop_gamepad::select_jtc_topics;

namespace
{

/// The topics a dxtr stack exposes, including the ones that must not be selected.
const std::vector<std::string> kDxtrTopics = {
  "/joint_states",
  "/joy",
  "/joint_trajectory_controller_arm_left/joint_trajectory",
  "/joint_trajectory_controller_arm_left/controller_state",
  "/joint_trajectory_controller_arm_right/joint_trajectory",
  "/joint_trajectory_controller_hip/joint_trajectory",
  "/mecanum_drive_controller/reference",
  "/cartesian_pose_controller_arm_left/target_pose",
};

std::vector<std::string> topics_of(const std::vector<JtcTopic>& selected)
{
  std::vector<std::string> topics;
  for (const auto& entry : selected) {
    topics.push_back(entry.topic);
  }
  return topics;
}

}  // namespace

TEST(SelectJtcTopics, PicksOnlyTheRequestedComponents)
{
  const auto selected = select_jtc_topics(kDxtrTopics, { "arm_left", "arm_right" }, "/");

  EXPECT_EQ(topics_of(selected), (std::vector<std::string>{ "/joint_trajectory_controller_arm_left/joint_trajectory",
                                                            "/joint_trajectory_controller_arm_right/"
                                                            "joint_trajectory" }));
}

TEST(SelectJtcTopics, ReportsTheOwningController)
{
  const auto selected = select_jtc_topics(kDxtrTopics, { "hip" }, "/");

  ASSERT_EQ(selected.size(), 1u);
  EXPECT_EQ(selected.front().controller, "joint_trajectory_controller_hip");
}

TEST(SelectJtcTopics, IgnoresOtherTopicsOfTheSameController)
{
  const auto selected = select_jtc_topics(kDxtrTopics, { "arm_left" }, "/");

  // controller_state sits under the same controller and must not be mistaken for a
  // command topic.
  EXPECT_EQ(topics_of(selected),
            (std::vector<std::string>{ "/joint_trajectory_controller_arm_left/joint_trajectory" }));
}

TEST(SelectJtcTopics, IgnoresOtherControllers)
{
  const auto selected = select_jtc_topics(kDxtrTopics, { "arm_left", "arm_right", "hip", "platform" }, "/");

  EXPECT_EQ(selected.size(), 3u);
}

TEST(SelectJtcTopics, NoComponentsSelectsNothing)
{
  EXPECT_TRUE(select_jtc_topics(kDxtrTopics, {}, "/").empty());
}

TEST(SelectJtcTopics, AFlatRobotKeepsItsUnsuffixedTopic)
{
  const std::vector<std::string> topics = { "/joint_trajectory_controller/joint_trajectory" };

  // A single-arm robot's component has no name, so there is no suffix to match on.
  EXPECT_EQ(topics_of(select_jtc_topics(topics, { "" }, "/")),
            (std::vector<std::string>{ "/joint_trajectory_controller/joint_trajectory" }));
}

TEST(SelectJtcTopics, ANamedComponentDoesNotMatchAnUnsuffixedController)
{
  const std::vector<std::string> topics = { "/joint_trajectory_controller/joint_trajectory" };

  EXPECT_TRUE(select_jtc_topics(topics, { "arm_left" }, "/").empty());
}

TEST(SelectJtcTopics, RespectsANamespace)
{
  const std::vector<std::string> topics = {
    "/robot2/joint_trajectory_controller_arm_left/joint_trajectory",
    "/joint_trajectory_controller_arm_left/joint_trajectory",
  };

  EXPECT_EQ(topics_of(select_jtc_topics(topics, { "arm_left" }, "/robot2")),
            (std::vector<std::string>{ "/robot2/joint_trajectory_controller_arm_left/joint_trajectory" }));

  EXPECT_EQ(topics_of(select_jtc_topics(topics, { "arm_left" }, "/")),
            (std::vector<std::string>{ "/joint_trajectory_controller_arm_left/joint_trajectory" }));
}

TEST(SelectJtcTopics, ComponentNamesMatchOnAWholeSuffix)
{
  const std::vector<std::string> topics = { "/joint_trajectory_controller_arm_left/joint_trajectory" };

  // "arm" is a prefix of "arm_left" but not the component this controller drives.
  EXPECT_TRUE(select_jtc_topics(topics, { "arm" }, "/").empty());
  EXPECT_TRUE(select_jtc_topics(topics, { "left" }, "/").empty());
}

TEST(SelectJtcTopics, ResultIsSortedRegardlessOfGraphOrder)
{
  std::vector<std::string> shuffled = kDxtrTopics;
  std::reverse(shuffled.begin(), shuffled.end());

  EXPECT_EQ(topics_of(select_jtc_topics(shuffled, { "arm_left", "arm_right", "hip" }, "/")),
            topics_of(select_jtc_topics(kDxtrTopics, { "arm_left", "arm_right", "hip" }, "/")));
}
