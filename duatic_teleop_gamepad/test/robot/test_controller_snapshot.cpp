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

#include "duatic_teleop_gamepad/robot/controller_snapshot.hpp"

using duatic_teleop_gamepad::ControllerSnapshot;
using duatic_teleop_gamepad::ControllerState;

namespace
{

const std::vector<std::string> kManagedBases = {
  "freedrive_controller", "joint_trajectory_controller", "mecanum_drive_controller",
  "platform_velocity_controller", "freeze_controller",
};

/// The controllers the dxtr example loads, with the states it comes up in.
std::vector<ControllerState> dxtr_controllers()
{
  return {
    { "duadrive_status_broadcaster", "active" },
    { "joint_state_broadcaster", "active" },
    { "freeze_controller", "inactive" },
    { "brake_release_controller", "inactive" },
    { "mecanum_drive_controller", "active" },
    { "freedrive_controller", "inactive" },
    { "gravity_compensation_controller", "active" },
    { "joint_trajectory_controller_hip", "inactive" },
    { "freeze_controller_hip", "inactive" },
    { "joint_trajectory_controller_arm_left", "inactive" },
    { "virtual_fts_arm_left", "active" },
    { "freeze_controller_arm_left", "inactive" },
    { "joint_trajectory_controller_arm_right", "inactive" },
    { "virtual_fts_arm_right", "active" },
    { "freeze_controller_arm_right", "inactive" },
  };
}

bool contains(const std::vector<std::string>& values, const std::string& value)
{
  return std::find(values.begin(), values.end(), value) != values.end();
}

}  // namespace

TEST(ControllerSnapshot, DefaultHasNoData)
{
  EXPECT_FALSE(ControllerSnapshot().has_data());
  EXPECT_TRUE(ControllerSnapshot::build({}, kManagedBases).has_data());
}

TEST(ControllerSnapshot, UnmanagedControllersAreInvisible)
{
  const auto snapshot = ControllerSnapshot::build(dxtr_controllers(), kManagedBases);

  // The switching logic deactivates active controllers the next mode does not need, so a
  // broadcaster showing up here would eventually be switched off underneath the robot.
  EXPECT_FALSE(contains(snapshot.active(), "joint_state_broadcaster"));
  EXPECT_FALSE(contains(snapshot.active(), "duadrive_status_broadcaster"));
  EXPECT_FALSE(contains(snapshot.active(), "gravity_compensation_controller"));
  EXPECT_FALSE(contains(snapshot.active(), "virtual_fts_arm_left"));

  EXPECT_EQ(snapshot.active(), (std::vector<std::string>{ "mecanum_drive_controller" }));
}

TEST(ControllerSnapshot, GlobalFreezeDecidesTheEStop)
{
  auto controllers = dxtr_controllers();
  EXPECT_FALSE(ControllerSnapshot::build(controllers, kManagedBases).freeze_active());

  controllers[2].state = "active";  // freeze_controller
  EXPECT_TRUE(ControllerSnapshot::build(controllers, kManagedBases).freeze_active());
}

TEST(ControllerSnapshot, PerComponentFreezeIsNotTheEStop)
{
  auto controllers = dxtr_controllers();
  controllers[8].state = "active";   // freeze_controller_hip
  controllers[11].state = "active";  // freeze_controller_arm_left

  // These are switched as part of ordinary mode changes; only the global one stops the
  // robot, and reading them as an E-Stop would wedge teleop off permanently.
  EXPECT_FALSE(ControllerSnapshot::build(controllers, kManagedBases).freeze_active());
}

TEST(ControllerSnapshot, WithoutAGlobalFreezeAnyActiveFreezeCounts)
{
  const std::vector<ControllerState> controllers = {
    { "freeze_controller_arm_left", "inactive" },
    { "freeze_controller_arm_right", "active" },
  };

  EXPECT_TRUE(ControllerSnapshot::build(controllers, kManagedBases).freeze_active());
}

TEST(ControllerSnapshot, NoFreezeControllerMeansNoEStop)
{
  const std::vector<ControllerState> controllers = { { "mecanum_drive_controller", "active" } };
  EXPECT_FALSE(ControllerSnapshot::build(controllers, kManagedBases).freeze_active());
}

TEST(ControllerSnapshot, ABaseNameResolvesToEveryInstance)
{
  const auto snapshot = ControllerSnapshot::build(dxtr_controllers(), kManagedBases);

  EXPECT_EQ(snapshot.matching({ "joint_trajectory_controller" }),
            (std::vector<std::string>{ "joint_trajectory_controller_arm_left",
                                       "joint_trajectory_controller_arm_right",
                                       "joint_trajectory_controller_hip" }));
}

TEST(ControllerSnapshot, AnExactNameResolvesToOneController)
{
  const auto snapshot = ControllerSnapshot::build(dxtr_controllers(), kManagedBases);

  EXPECT_EQ(snapshot.matching({ "joint_trajectory_controller_hip" }),
            (std::vector<std::string>{ "joint_trajectory_controller_hip" }));
}

TEST(ControllerSnapshot, MatchingDeduplicatesOverlappingRequests)
{
  const auto snapshot = ControllerSnapshot::build(dxtr_controllers(), kManagedBases);

  EXPECT_EQ(snapshot.matching({ "joint_trajectory_controller", "joint_trajectory_controller_hip" }),
            (std::vector<std::string>{ "joint_trajectory_controller_arm_left",
                                       "joint_trajectory_controller_arm_right",
                                       "joint_trajectory_controller_hip" }));
}

TEST(ControllerSnapshot, MatchingIgnoresControllersThatAreNotLoaded)
{
  const auto snapshot = ControllerSnapshot::build(dxtr_controllers(), kManagedBases);

  EXPECT_TRUE(snapshot.matching({ "platform_velocity_controller" }).empty());
}

TEST(ControllerSnapshot, IsActiveReflectsTheListing)
{
  const auto snapshot = ControllerSnapshot::build(dxtr_controllers(), kManagedBases);

  EXPECT_TRUE(snapshot.is_active("mecanum_drive_controller"));
  EXPECT_FALSE(snapshot.is_active("joint_trajectory_controller_arm_left"));
  EXPECT_FALSE(snapshot.is_active("joint_state_broadcaster"));
}
