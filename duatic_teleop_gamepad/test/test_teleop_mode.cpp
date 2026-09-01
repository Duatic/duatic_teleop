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

#include "duatic_teleop_gamepad/teleop_mode.hpp"

using duatic_teleop_gamepad::available_modes;
using duatic_teleop_gamepad::ControllerSnapshot;
using duatic_teleop_gamepad::ControllerState;
using duatic_teleop_gamepad::infer_mode;
using duatic_teleop_gamepad::next_mode;
using duatic_teleop_gamepad::plan_switch;
using duatic_teleop_gamepad::required_controllers;
using duatic_teleop_gamepad::TeleopMode;

namespace
{

const std::vector<std::string> kManagedBases = {
  "freedrive_controller", "joint_trajectory_controller", "mecanum_drive_controller",
  "platform_velocity_controller", "freeze_controller",
};

const std::vector<std::string> kProtectedBases = { "mecanum_drive_controller", "platform_velocity_controller" };

/// A dxtr listing with every controller inactive unless named.
ControllerSnapshot snapshot_with(const std::vector<std::string>& active)
{
  std::vector<ControllerState> controllers = {
    { "freedrive_controller", "inactive" },
    { "joint_trajectory_controller_arm_left", "inactive" },
    { "joint_trajectory_controller_arm_right", "inactive" },
    { "joint_trajectory_controller_hip", "inactive" },
    { "mecanum_drive_controller", "inactive" },
    { "joint_state_broadcaster", "active" },
  };

  for (auto& controller : controllers) {
    if (std::find(active.begin(), active.end(), controller.name) != active.end()) {
      controller.state = "active";
    }
  }

  return ControllerSnapshot::build(controllers, kManagedBases);
}

/// A listing of exactly these controllers, all inactive.
ControllerSnapshot snapshot_of(const std::vector<std::string>& names)
{
  std::vector<ControllerState> controllers;
  for (const auto& name : names) {
    controllers.push_back({ name, "inactive" });
  }

  return ControllerSnapshot::build(controllers, kManagedBases);
}

const std::vector<std::string> kAllJtcs = { "joint_trajectory_controller_arm_left",
                                            "joint_trajectory_controller_arm_right",
                                            "joint_trajectory_controller_hip" };

}  // namespace

TEST(TeleopMode, AMobileManipulatorOffersEverything)
{
  EXPECT_EQ(available_modes(snapshot_with({})),
            (std::vector<TeleopMode>{ TeleopMode::Freedrive, TeleopMode::Jog, TeleopMode::Drive }));
}

TEST(TeleopMode, WithoutADriveControllerThereIsNoDriving)
{
  EXPECT_EQ(available_modes(snapshot_of({ "freedrive_controller", "joint_trajectory_controller_arm_left" })),
            (std::vector<TeleopMode>{ TeleopMode::Freedrive, TeleopMode::Jog }));
}

TEST(TeleopMode, ADriveControllerAloneOffersOnlyDriving)
{
  EXPECT_EQ(available_modes(snapshot_of({ "mecanum_drive_controller" })),
            (std::vector<TeleopMode>{ TeleopMode::Drive }));
}

TEST(TeleopMode, AModeNeedsItsOwnControllerLoaded)
{
  // Trajectory controllers are there, but nothing spawned a freedrive controller.
  EXPECT_EQ(available_modes(snapshot_of({ "joint_trajectory_controller_arm_left", "mecanum_drive_controller" })),
            (std::vector<TeleopMode>{ TeleopMode::Jog, TeleopMode::Drive }));
}

TEST(TeleopMode, AHipIsJoggableWithoutAnyArm)
{
  // The joints behind a controller are its own business; a trajectory controller existing
  // is the whole test for whether jogging is on offer.
  EXPECT_EQ(available_modes(snapshot_of({ "joint_trajectory_controller_hip" })),
            (std::vector<TeleopMode>{ TeleopMode::Jog }));
}

TEST(TeleopMode, DrivingKeepsTheArmsHeld)
{
  auto required = required_controllers(TeleopMode::Drive, snapshot_with({}));

  // Without the trajectory controllers the arms go slack the moment the base moves.
  EXPECT_EQ(required, (std::vector<std::string>{ "joint_trajectory_controller_arm_left",
                                                 "joint_trajectory_controller_arm_right",
                                                 "joint_trajectory_controller_hip", "mecanum_drive_controller" }));
}

TEST(TeleopMode, InferReadsTheActiveControllers)
{
  const auto available = available_modes(snapshot_with({}));

  EXPECT_EQ(infer_mode(available, snapshot_with({ "freedrive_controller" })), TeleopMode::Freedrive);
  EXPECT_EQ(infer_mode(available, snapshot_with(kAllJtcs)), TeleopMode::Jog);
}

TEST(TeleopMode, DrivingWithHeldArmsIsNotMistakenForJogging)
{
  const auto available = available_modes(snapshot_with({}));

  auto active = kAllJtcs;
  active.push_back("mecanum_drive_controller");

  EXPECT_EQ(infer_mode(available, snapshot_with(active)), TeleopMode::Drive);
}

TEST(TeleopMode, InferNeedsEveryRequiredControllerActive)
{
  const auto available = available_modes(snapshot_with({}));

  // Only one of the three trajectory controllers is up, so the robot is not jogging.
  EXPECT_FALSE(infer_mode(available, snapshot_with({ "joint_trajectory_controller_arm_left" })).has_value());
}

TEST(TeleopMode, InferReportsNothingWhenIdle)
{
  const auto available = available_modes(snapshot_with({}));
  EXPECT_FALSE(infer_mode(available, snapshot_with({})).has_value());
}

TEST(TeleopMode, CyclingWrapsAround)
{
  const std::vector<TeleopMode> available = { TeleopMode::Freedrive, TeleopMode::Jog, TeleopMode::Drive };

  EXPECT_EQ(next_mode(available, TeleopMode::Freedrive), TeleopMode::Jog);
  EXPECT_EQ(next_mode(available, TeleopMode::Jog), TeleopMode::Drive);
  EXPECT_EQ(next_mode(available, TeleopMode::Drive), TeleopMode::Freedrive);
}

TEST(TeleopMode, CyclingStartsAtTheFirstAvailableMode)
{
  const std::vector<TeleopMode> available = { TeleopMode::Jog, TeleopMode::Drive };

  EXPECT_EQ(next_mode(available, std::nullopt), TeleopMode::Jog);

  // Freedrive went away, so the cycle restarts rather than following a mode that is gone.
  EXPECT_EQ(next_mode(available, TeleopMode::Freedrive), TeleopMode::Jog);
}

TEST(TeleopMode, CyclingWithNothingAvailableGivesNothing)
{
  EXPECT_FALSE(next_mode({}, std::nullopt).has_value());
  EXPECT_FALSE(next_mode({}, TeleopMode::Jog).has_value());
}

TEST(TeleopMode, SwitchActivatesWhatIsMissingAndStopsWhatIsSpare)
{
  const auto plan = plan_switch({ "a", "b" }, { "b", "c" }, {});

  EXPECT_EQ(plan.activate, (std::vector<std::string>{ "a" }));
  EXPECT_EQ(plan.deactivate, (std::vector<std::string>{ "c" }));
}

TEST(TeleopMode, SwitchLeavesProtectedControllersRunning)
{
  const auto plan = plan_switch({ "freedrive_controller" }, { "mecanum_drive_controller" }, kProtectedBases);

  // Stopping the drive controller loses its odometry, so it is left alone even though
  // freedrive has no use for it.
  EXPECT_TRUE(plan.deactivate.empty());
  EXPECT_EQ(plan.activate, (std::vector<std::string>{ "freedrive_controller" }));
}

TEST(TeleopMode, SwitchToTheSameModeDoesNothing)
{
  const auto plan = plan_switch(kAllJtcs, kAllJtcs, kProtectedBases);

  EXPECT_TRUE(plan.activate.empty());
  EXPECT_TRUE(plan.deactivate.empty());
}

TEST(TeleopMode, SwitchFromJoggingToDrivingKeepsTheTrajectoryControllers)
{
  const auto snapshot = snapshot_with(kAllJtcs);
  const auto plan =
      plan_switch(required_controllers(TeleopMode::Drive, snapshot), snapshot.active(), kProtectedBases);

  EXPECT_EQ(plan.activate, (std::vector<std::string>{ "mecanum_drive_controller" }));
  EXPECT_TRUE(plan.deactivate.empty());
}
