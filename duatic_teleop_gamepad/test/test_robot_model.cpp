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

#include "duatic_teleop_gamepad/robot_model.hpp"

using duatic_teleop_gamepad::ComponentType;
using duatic_teleop_gamepad::component_from_topic;
using duatic_teleop_gamepad::RobotModel;

namespace
{

/// The joint set the dxtr example publishes, in the order the broadcaster lists it.
const std::vector<std::string> kDxtrJoints = {
  "arm_left/shoulder_lift",   "arm_left/shoulder_rotation",  "arm_left/shoulder_flexion",
  "arm_left/elbow_flexion",   "arm_left/forearm_rotation",   "arm_left/wrist_flexion",
  "arm_left/wrist_rotation",  "arm_right/shoulder_lift",     "arm_right/shoulder_rotation",
  "arm_right/shoulder_flexion", "arm_right/elbow_flexion",   "arm_right/forearm_rotation",
  "arm_right/wrist_flexion",  "arm_right/wrist_rotation",    "hip_yaw",
  "hip_pitch",                "wheel_front_left",            "wheel_front_right",
  "wheel_back_right",         "wheel_back_left",
};

}  // namespace

TEST(RobotModelClassify, NamespacedArmJointsUseTheirPrefix)
{
  EXPECT_EQ(RobotModel::classify("arm_left/shoulder_lift"),
            std::make_pair(std::string("arm_left"), ComponentType::Arm));
  EXPECT_EQ(RobotModel::classify("arm_right/wrist_rotation"),
            std::make_pair(std::string("arm_right"), ComponentType::Arm));
}

TEST(RobotModelClassify, HandPrefixNamesAnArm)
{
  // A hand is jogged like an arm, and its own joints carry no arm keyword, so the prefix is
  // what identifies it.
  EXPECT_EQ(RobotModel::classify("hand_left/grip_joint"),
            std::make_pair(std::string("hand_left"), ComponentType::Arm));
}

TEST(RobotModelClassify, FlatJointsFallBackToKeywords)
{
  EXPECT_EQ(RobotModel::classify("hip_yaw"), std::make_pair(std::string("hip"), ComponentType::Hip));
  EXPECT_EQ(RobotModel::classify("wheel_front_left"),
            std::make_pair(std::string("platform"), ComponentType::Platform));
}

TEST(RobotModelClassify, FlatArmJointsHaveNoComponentName)
{
  // A single-arm robot exposes "/joint_trajectory_controller/joint_trajectory" with no
  // component suffix, so the component it belongs to must be unnamed.
  EXPECT_EQ(RobotModel::classify("shoulder_lift"), std::make_pair(std::string(), ComponentType::Arm));
}

TEST(RobotModelClassify, JointsTheGamepadDoesNotDriveAreMisc)
{
  EXPECT_EQ(RobotModel::classify("some_unknown_joint"), std::make_pair(std::string("misc"), ComponentType::Misc));

  // A head and a gripper's own finger joints are not things the gamepad drives, and naming
  // them would create a component nothing focuses. Grippers are found by their command
  // topic, which names the arm they belong to.
  EXPECT_EQ(RobotModel::classify("head_pan"), std::make_pair(std::string("misc"), ComponentType::Misc));
  EXPECT_EQ(RobotModel::classify("clamp_left_finger_joint"),
            std::make_pair(std::string("misc"), ComponentType::Misc));
}

TEST(RobotModel, DerivesTheDxtrComponents)
{
  RobotModel model;
  EXPECT_TRUE(model.rebuild(kDxtrJoints));

  EXPECT_EQ(model.component_names(ComponentType::Arm), (std::vector<std::string>{ "arm_left", "arm_right" }));
  EXPECT_EQ(model.component_names(ComponentType::Hip), (std::vector<std::string>{ "hip" }));
  EXPECT_EQ(model.component_names(ComponentType::Platform), (std::vector<std::string>{ "platform" }));
}

TEST(RobotModel, PicksUpAComponentThatArrivesLate)
{
  // The failure this guards: a model latched on the first message would report no arms
  // forever if the gripper's publisher happened to be discovered first.
  RobotModel model;

  model.rebuild({ "clamp_left_finger_joint" });
  EXPECT_TRUE(model.component_names(ComponentType::Arm).empty());

  EXPECT_TRUE(model.rebuild({ "clamp_left_finger_joint", "arm_left/shoulder_lift" }));
  EXPECT_EQ(model.component_names(ComponentType::Arm), (std::vector<std::string>{ "arm_left" }));
}

TEST(RobotModel, DropsAComponentThatGoesAway)
{
  RobotModel model;
  model.rebuild({ "arm_left/shoulder_lift", "hip_yaw" });
  ASSERT_TRUE(model.has_component("hip"));

  EXPECT_TRUE(model.rebuild({ "arm_left/shoulder_lift" }));
  EXPECT_FALSE(model.has_component("hip"));
}

TEST(RobotModel, RebuildReportsNoChangeForTheSameJoints)
{
  RobotModel model;
  model.rebuild(kDxtrJoints);

  EXPECT_FALSE(model.rebuild(kDxtrJoints));
}

TEST(RobotModel, RebuildIsIndependentOfJointOrder)
{
  RobotModel model;
  model.rebuild(kDxtrJoints);

  std::vector<std::string> shuffled = kDxtrJoints;
  std::reverse(shuffled.begin(), shuffled.end());

  EXPECT_FALSE(model.rebuild(shuffled));
}

TEST(ComponentFromTopic, ReadsTheLeadingSegment)
{
  EXPECT_EQ(component_from_topic("/arm_left/gripper_controller/commands", "gripper_controller/commands"), "arm_left");
}

TEST(ComponentFromTopic, RejectsATopicOfTheWrongShape)
{
  // Extra namespace in front, so the leading segment is not the component.
  EXPECT_TRUE(component_from_topic("/robot2/arm_left/gripper_controller/commands", "gripper_controller/commands")
                  .empty());

  // No component segment at all.
  EXPECT_TRUE(component_from_topic("/gripper_controller/commands", "gripper_controller/commands").empty());
}

TEST(ComponentFromTopic, RejectsADifferentController)
{
  EXPECT_TRUE(component_from_topic("/arm_left/other_controller/commands", "gripper_controller/commands").empty());
}

TEST(ComponentFromTopic, DoesNotMatchOnATrailingSubstring)
{
  // Searching backwards from the suffix would happily return "my_gripper" here; the shape
  // check is what makes the answer the whole leading segment or nothing.
  EXPECT_EQ(component_from_topic("/my_arm_left/gripper_controller/commands", "gripper_controller/commands"),
            "my_arm_left");
}
