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

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "duatic_teleop_gamepad/modes/drive_mode.hpp"
#include "duatic_teleop_gamepad/modes/freedrive_mode.hpp"
#include "duatic_teleop_gamepad/modes/jog_mode.hpp"
#include "duatic_teleop_gamepad/modes/mode_manager.hpp"

using duatic_teleop_gamepad::ControllerSnapshot;
using duatic_teleop_gamepad::ControllerState;
using duatic_teleop_gamepad::DriveMode;
using duatic_teleop_gamepad::FreedriveMode;
using duatic_teleop_gamepad::GamepadConfig;
using duatic_teleop_gamepad::JogMode;
using duatic_teleop_gamepad::JointStateCache;
using duatic_teleop_gamepad::JtcDiscovery;
using duatic_teleop_gamepad::ModeManager;
using duatic_teleop_gamepad::TeleopContext;
using duatic_teleop_gamepad::BaseMode;

namespace
{

const std::vector<std::string> kManagedBases = {
  "freedrive_controller", "joint_trajectory_controller", "mecanum_drive_controller",
  "platform_velocity_controller", "freeze_controller",
};

const std::vector<std::string> kProtectedBases = { "mecanum_drive_controller", "platform_velocity_controller" };

const std::vector<std::string> kAllJtcs = { "joint_trajectory_controller_arm_left",
                                            "joint_trajectory_controller_arm_right",
                                            "joint_trajectory_controller_hip" };

/// A mobile manipulator's listing, with every controller inactive unless named.
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

rclcpp::Time at(double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
}

/// The real three modes, wired to a node that has no robot on the other end.
///
/// Only what the modes say about controllers is under test here, which is the half that
/// decides what the operator is offered. The publishing half is exercised through the
/// pieces the modes are made of, which need no node at all.
class Modes : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  Modes()
    : node_("test_teleop_modes")
    , joint_states_(node_.get_logger(), rclcpp::Duration::from_seconds(1.0))
    , discovery_(node_, []() {})
    , context_{ node_, config_, joint_states_, discovery_ }
  {
    std::vector<std::unique_ptr<BaseMode>> owned;
    owned.push_back(std::make_unique<FreedriveMode>());
    owned.push_back(std::make_unique<JogMode>(context_));
    owned.push_back(std::make_unique<DriveMode>(context_));

    manager_ = std::make_unique<ModeManager>(std::move(owned), node_.get_logger(), kProtectedBases,
                                             rclcpp::Duration::from_seconds(3.0));
  }

  /// The modes on offer, named rather than addressed, so a failure says which ones.
  std::vector<std::string> available()
  {
    std::vector<std::string> names;
    for (const auto* mode : manager_->available()) {
      names.push_back(mode->name());
    }
    return names;
  }

  GamepadConfig config_;
  rclcpp::Node node_;
  JointStateCache joint_states_;
  JtcDiscovery discovery_;
  TeleopContext context_;
  std::unique_ptr<ModeManager> manager_;
};

}  // namespace

TEST_F(Modes, AMobileManipulatorOffersEverything)
{
  manager_->refresh(snapshot_with({}), at(0.0));

  EXPECT_EQ(available(), (std::vector<std::string>{ "freedrive", "jog", "drive" }));
}

TEST_F(Modes, WithoutADriveControllerThereIsNoDriving)
{
  manager_->refresh(snapshot_of({ "freedrive_controller", "joint_trajectory_controller_arm_left" }), at(0.0));

  EXPECT_EQ(available(), (std::vector<std::string>{ "freedrive", "jog" }));
}

TEST_F(Modes, ADriveControllerAloneOffersOnlyDriving)
{
  manager_->refresh(snapshot_of({ "mecanum_drive_controller" }), at(0.0));

  EXPECT_EQ(available(), (std::vector<std::string>{ "drive" }));
}

TEST_F(Modes, AHipIsJoggableWithoutAnyArm)
{
  // The joints behind a controller are its own business; a trajectory controller existing
  // is the whole test for whether jogging is on offer.
  manager_->refresh(snapshot_of({ "joint_trajectory_controller_hip" }), at(0.0));

  EXPECT_EQ(available(), (std::vector<std::string>{ "jog" }));
}

TEST_F(Modes, DrivingKeepsTheArmsHeld)
{
  const DriveMode drive(context_);
  const auto required = drive.required_controllers(snapshot_with({}));

  // Without the trajectory controllers the arms go slack the moment the base moves.
  EXPECT_EQ(required, (std::vector<std::string>{ "joint_trajectory_controller_arm_left",
                                                 "joint_trajectory_controller_arm_right",
                                                 "joint_trajectory_controller_hip", "mecanum_drive_controller" }));
}

TEST_F(Modes, JoggingNeedsEveryTrajectoryController)
{
  const JogMode jog(context_);

  EXPECT_EQ(jog.required_controllers(snapshot_with({})), kAllJtcs);
}

TEST_F(Modes, DrivingWithHeldArmsIsNotMistakenForJogging)
{
  auto active = kAllJtcs;
  active.push_back("mecanum_drive_controller");

  manager_->refresh(snapshot_with(active), at(0.0));

  ASSERT_NE(manager_->active(), nullptr);
  EXPECT_EQ(manager_->active()->name(), "drive");
}

TEST_F(Modes, SwitchingFromJoggingToDrivingKeepsTheTrajectoryControllers)
{
  const auto snapshot = snapshot_with(kAllJtcs);
  manager_->refresh(snapshot, at(0.0));
  ASSERT_NE(manager_->active(), nullptr);
  ASSERT_EQ(manager_->active()->name(), "jog");

  const auto plan = manager_->request_next(snapshot, at(1.0));

  ASSERT_TRUE(plan.has_value());
  EXPECT_EQ(plan->activate, (std::vector<std::string>{ "mecanum_drive_controller" }));
  EXPECT_TRUE(plan->deactivate.empty());
}
