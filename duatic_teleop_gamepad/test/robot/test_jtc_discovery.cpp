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

#include "duatic_teleop_gamepad/robot/jtc_discovery.hpp"

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

TEST(SelectJtcTopics, PicksEveryTrajectoryController)
{
  const auto selected = select_jtc_topics(kDxtrTopics, "/");

  EXPECT_EQ(topics_of(selected), (std::vector<std::string>{ "/joint_trajectory_controller_arm_left/joint_trajectory",
                                                            "/joint_trajectory_controller_arm_right/joint_trajectory",
                                                            "/joint_trajectory_controller_hip/joint_trajectory" }));
}

TEST(SelectJtcTopics, NamesTheComponentFromTheControllerSuffix)
{
  const auto selected = select_jtc_topics(kDxtrTopics, "/");

  std::vector<std::string> components;
  for (const auto& topic : selected) {
    components.push_back(topic.component);
  }

  EXPECT_EQ(components, (std::vector<std::string>{ "arm_left", "arm_right", "hip" }));
}

TEST(SelectJtcTopics, ReportsTheOwningController)
{
  const auto selected = select_jtc_topics({ "/joint_trajectory_controller_hip/joint_trajectory" }, "/");

  ASSERT_EQ(selected.size(), 1u);
  EXPECT_EQ(selected.front().controller, "joint_trajectory_controller_hip");
}

TEST(SelectJtcTopics, IgnoresOtherTopicsOfTheSameController)
{
  const auto selected = select_jtc_topics(kDxtrTopics, "/");

  // controller_state sits under the same controller and must not be mistaken for a
  // command topic.
  for (const auto& topic : topics_of(selected)) {
    EXPECT_NE(topic.find("/joint_trajectory"), std::string::npos);
    EXPECT_EQ(topic.find("controller_state"), std::string::npos);
  }
}

TEST(SelectJtcTopics, IgnoresOtherControllers)
{
  EXPECT_TRUE(select_jtc_topics({ "/mecanum_drive_controller/joint_trajectory" }, "/").empty());
}

TEST(SelectJtcTopics, NoTopicsSelectsNothing)
{
  EXPECT_TRUE(select_jtc_topics({}, "/").empty());
}

TEST(SelectJtcTopics, AFlatRobotKeepsItsUnsuffixedTopic)
{
  const std::vector<std::string> topics = { "/joint_trajectory_controller/joint_trajectory" };

  // A single-arm robot spawns the bare controller, so its component has no name.
  const auto selected = select_jtc_topics(topics, "/");

  ASSERT_EQ(selected.size(), 1u);
  EXPECT_TRUE(selected.front().component.empty());
}

TEST(SelectJtcTopics, RespectsANamespace)
{
  const std::vector<std::string> topics = {
    "/robot2/joint_trajectory_controller_arm_left/joint_trajectory",
    "/joint_trajectory_controller_arm_left/joint_trajectory",
  };

  EXPECT_EQ(topics_of(select_jtc_topics(topics, "/robot2")),
            (std::vector<std::string>{ "/robot2/joint_trajectory_controller_arm_left/joint_trajectory" }));

  EXPECT_EQ(topics_of(select_jtc_topics(topics, "/")),
            (std::vector<std::string>{ "/joint_trajectory_controller_arm_left/joint_trajectory" }));
}

TEST(SelectJtcTopics, TheComponentIsSeparatedByAnUnderscore)
{
  // "joint_trajectory_controller2" is a different controller, not the component "2".
  EXPECT_TRUE(select_jtc_topics({ "/joint_trajectory_controller2/joint_trajectory" }, "/").empty());
}

TEST(SelectJtcTopics, ResultIsSortedRegardlessOfGraphOrder)
{
  std::vector<std::string> shuffled = kDxtrTopics;
  std::reverse(shuffled.begin(), shuffled.end());

  EXPECT_EQ(topics_of(select_jtc_topics(shuffled, "/")), topics_of(select_jtc_topics(kDxtrTopics, "/")));
}
