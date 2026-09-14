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
#include <utility>
#include <vector>

#include "duatic_teleop_gamepad/modes/mode_manager.hpp"

using duatic_teleop_gamepad::ControllerSnapshot;
using duatic_teleop_gamepad::ControllerState;
using duatic_teleop_gamepad::ModeManager;
using duatic_teleop_gamepad::BaseMode;

namespace
{

const std::vector<std::string> kManagedBases = { "alpha_controller", "beta_controller", "gamma_controller",
                                                 "freeze_controller" };

/// A mode with no robot behind it, so what is under test is the manager's own decisions
/// rather than any real mode's.
class FakeMode : public BaseMode
{
public:
  FakeMode(std::string name, std::vector<std::string> bases) : name_(std::move(name)), bases_(std::move(bases))
  {
  }

  std::string name() const override
  {
    return name_;
  }

  const std::vector<std::string>& controller_bases() const override
  {
    return bases_;
  }

  void reset() override
  {
    ++resets;
  }

  int resets{ 0 };

private:
  std::string name_;
  std::vector<std::string> bases_;
};

/// A mode that needs another one's controllers as well, the way driving holds the arms.
class DemandingMode : public FakeMode
{
public:
  using FakeMode::FakeMode;

  std::vector<std::string> required_controllers(const ControllerSnapshot& controllers) const override
  {
    auto required = FakeMode::required_controllers(controllers);
    const auto also = controllers.matching({ "alpha_controller" });
    required.insert(required.end(), also.begin(), also.end());
    return required;
  }
};

ControllerSnapshot snapshot_with(const std::vector<std::string>& active)
{
  std::vector<ControllerState> controllers = {
    { "alpha_controller_left", "inactive" },
    { "alpha_controller_right", "inactive" },
    { "beta_controller", "inactive" },
    { "gamma_controller", "inactive" },
    { "freeze_controller", "inactive" },
    { "freeze_controller_arm", "inactive" },
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

/// The three-mode cycle the robot has, with gamma standing in for the mode that also needs
/// alpha's controllers.
struct Fixture
{
  Fixture()
  {
    auto alpha = std::make_unique<FakeMode>("alpha", std::vector<std::string>{ "alpha_controller" });
    auto beta = std::make_unique<FakeMode>("beta", std::vector<std::string>{ "beta_controller" });
    auto gamma = std::make_unique<DemandingMode>("gamma", std::vector<std::string>{ "gamma_controller" });

    modes = { alpha.get(), beta.get(), gamma.get() };

    std::vector<std::unique_ptr<BaseMode>> owned;
    owned.push_back(std::move(alpha));
    owned.push_back(std::move(beta));
    owned.push_back(std::move(gamma));

    manager = std::make_unique<ModeManager>(std::move(owned), rclcpp::get_logger("test"),
                                            std::vector<std::string>{}, rclcpp::Duration::from_seconds(3.0));
  }

  std::vector<BaseMode*> modes;
  std::unique_ptr<ModeManager> manager;
};

}  // namespace

TEST(ModeManager, AvailabilityFollowsTheLoadedControllers)
{
  Fixture fixture;

  fixture.manager->refresh(snapshot_of({ "alpha_controller_left", "gamma_controller" }), at(0.0));

  EXPECT_EQ(fixture.manager->available(), (std::vector<BaseMode*>{ fixture.modes[0], fixture.modes[2] }));
}

TEST(ModeManager, AModeWithoutItsControllerIsNotOffered)
{
  Fixture fixture;

  fixture.manager->refresh(snapshot_of({ "beta_controller" }), at(0.0));

  EXPECT_EQ(fixture.manager->available(), (std::vector<BaseMode*>{ fixture.modes[1] }));
}

TEST(ModeManager, NothingIsActiveUntilTheControllersSaySo)
{
  Fixture fixture;

  fixture.manager->refresh(snapshot_with({}), at(0.0));

  EXPECT_EQ(fixture.manager->active(), nullptr);
}

TEST(ModeManager, AdoptsWhatTheControllersAreRunning)
{
  Fixture fixture;

  fixture.manager->refresh(snapshot_with({ "beta_controller" }), at(0.0));

  EXPECT_EQ(fixture.manager->active(), fixture.modes[1]);
}

TEST(ModeManager, AdoptingResetsTheModes)
{
  Fixture fixture;

  fixture.manager->refresh(snapshot_with({ "beta_controller" }), at(0.0));

  // Every mode, not just the adopted one, so none of them resumes against a stale target.
  for (const auto* mode : fixture.modes) {
    EXPECT_EQ(static_cast<const FakeMode*>(mode)->resets, 1);
  }
}

TEST(ModeManager, TheMostDemandingSatisfiedModeWins)
{
  Fixture fixture;

  // Gamma needs alpha's controllers as well, so a listing that satisfies both is gamma.
  fixture.manager->refresh(snapshot_with({ "alpha_controller_left", "alpha_controller_right", "gamma_controller" }),
                           at(0.0));

  EXPECT_EQ(fixture.manager->active(), fixture.modes[2]);
}

TEST(ModeManager, AdoptionNeedsEveryRequiredControllerActive)
{
  Fixture fixture;

  // Only one of alpha's two controllers is up, so the robot is not in alpha.
  fixture.manager->refresh(snapshot_with({ "alpha_controller_left" }), at(0.0));

  EXPECT_EQ(fixture.manager->active(), nullptr);
}

TEST(ModeManager, FollowsSomethingElseSwitchingControllers)
{
  Fixture fixture;
  fixture.manager->refresh(snapshot_with({ "beta_controller" }), at(0.0));

  fixture.manager->refresh(snapshot_with({ "alpha_controller_left", "alpha_controller_right" }), at(1.0));

  EXPECT_EQ(fixture.manager->active(), fixture.modes[0]);
}

TEST(ModeManager, CyclingWrapsAround)
{
  Fixture fixture;
  const auto snapshot = snapshot_with({ "beta_controller" });
  fixture.manager->refresh(snapshot, at(0.0));
  ASSERT_EQ(fixture.manager->active(), fixture.modes[1]);

  EXPECT_TRUE(fixture.manager->request_next(snapshot, at(1.0)).has_value());
  fixture.manager->refresh(snapshot_with({ "gamma_controller", "alpha_controller_left", "alpha_controller_right" }),
                           at(2.0));
  ASSERT_EQ(fixture.manager->active(), fixture.modes[2]);

  EXPECT_TRUE(fixture.manager->request_next(snapshot, at(3.0)).has_value());
  fixture.manager->refresh(snapshot_with({ "alpha_controller_left", "alpha_controller_right" }), at(4.0));
  EXPECT_EQ(fixture.manager->active(), fixture.modes[0]);
}

TEST(ModeManager, CyclingStartsAtTheFirstAvailableMode)
{
  Fixture fixture;
  const auto snapshot = snapshot_of({ "beta_controller", "gamma_controller" });
  fixture.manager->refresh(snapshot, at(0.0));

  const auto plan = fixture.manager->request_next(snapshot, at(1.0));

  ASSERT_TRUE(plan.has_value());
  EXPECT_EQ(plan->activate, (std::vector<std::string>{ "beta_controller" }));
}

TEST(ModeManager, CyclingWithNothingAvailableGivesNothing)
{
  Fixture fixture;
  const auto snapshot = snapshot_of({});
  fixture.manager->refresh(snapshot, at(0.0));

  EXPECT_FALSE(fixture.manager->request_next(snapshot, at(1.0)).has_value());
}

TEST(ModeManager, ARequestedModeStandsUntilItsControllersRun)
{
  Fixture fixture;
  const auto before = snapshot_with({ "beta_controller" });
  fixture.manager->refresh(before, at(0.0));
  ASSERT_TRUE(fixture.manager->request_next(before, at(1.0)).has_value());

  // The switch is in flight: the old controllers are still the active ones. Adopting them
  // again here would undo the switch that was just asked for.
  fixture.manager->refresh(before, at(1.5));
  EXPECT_EQ(fixture.manager->active(), fixture.modes[1]);

  fixture.manager->refresh(snapshot_with({ "gamma_controller", "alpha_controller_left", "alpha_controller_right" }),
                           at(2.0));
  EXPECT_EQ(fixture.manager->active(), fixture.modes[2]);
}

TEST(ModeManager, ASecondPressSkipsTheModeStillSwitching)
{
  Fixture fixture;
  const auto snapshot = snapshot_with({ "beta_controller" });
  fixture.manager->refresh(snapshot, at(0.0));
  ASSERT_EQ(fixture.manager->active(), fixture.modes[1]);
  ASSERT_TRUE(fixture.manager->request_next(snapshot, at(1.0)).has_value());

  // Asked for gamma and, before it came up, asked again. The second press moves on from
  // gamma rather than from beta, which would ask for gamma a second time and swallow it.
  const auto plan = fixture.manager->request_next(snapshot, at(1.2));

  ASSERT_TRUE(plan.has_value());
  EXPECT_EQ(plan->activate,
            (std::vector<std::string>{ "alpha_controller_left", "alpha_controller_right" }));

  fixture.manager->refresh(snapshot_with({ "alpha_controller_left", "alpha_controller_right" }), at(2.0));
  EXPECT_EQ(fixture.manager->active(), fixture.modes[0]);
}

TEST(ModeManager, ASecondPressRestartsTheTimeout)
{
  Fixture fixture;
  const auto snapshot = snapshot_with({ "beta_controller" });
  fixture.manager->refresh(snapshot, at(0.0));
  ASSERT_TRUE(fixture.manager->request_next(snapshot, at(1.0)).has_value());
  ASSERT_TRUE(fixture.manager->request_next(snapshot, at(2.5)).has_value());

  // Past the first request's three seconds but inside the second's, so the second still
  // stands. The listing satisfies gamma as well, and gamma is the more demanding mode, so
  // only a request that is still live keeps alpha from losing to it.
  fixture.manager->refresh(
      snapshot_with({ "alpha_controller_left", "alpha_controller_right", "gamma_controller" }), at(4.5));
  EXPECT_EQ(fixture.manager->active(), fixture.modes[0]);
}

TEST(ModeManager, ARequestedModeThatNeverStartsIsGivenUpOn)
{
  Fixture fixture;
  const auto snapshot = snapshot_with({ "beta_controller" });
  fixture.manager->refresh(snapshot, at(0.0));
  ASSERT_TRUE(fixture.manager->request_next(snapshot, at(1.0)).has_value());

  fixture.manager->refresh(snapshot, at(10.0));

  // Past the timeout the request is dropped, and the next refresh is free to adopt again.
  EXPECT_EQ(fixture.manager->active(), fixture.modes[1]);
  fixture.manager->refresh(snapshot_with({ "alpha_controller_left", "alpha_controller_right" }), at(11.0));
  EXPECT_EQ(fixture.manager->active(), fixture.modes[0]);
}

TEST(ModeManager, AFailedSwitchFreezesTheRobot)
{
  Fixture fixture;
  const auto snapshot = snapshot_with({ "beta_controller" });
  fixture.manager->refresh(snapshot, at(0.0));
  ASSERT_TRUE(fixture.manager->request_next(snapshot, at(1.0)).has_value());

  const auto plan = fixture.manager->refresh(snapshot, at(10.0));

  // Every freeze controller the robot has, and nothing deactivated, which is the switch
  // the E-Stop node asks for.
  ASSERT_TRUE(plan.has_value());
  EXPECT_EQ(plan->activate, (std::vector<std::string>{ "freeze_controller", "freeze_controller_arm" }));
  EXPECT_TRUE(plan->deactivate.empty());
}

TEST(ModeManager, AFailedSwitchStopsEveryMode)
{
  Fixture fixture;
  const auto snapshot = snapshot_with({ "beta_controller" });
  fixture.manager->refresh(snapshot, at(0.0));
  ASSERT_TRUE(fixture.manager->request_next(snapshot, at(1.0)).has_value());
  const int before = static_cast<const FakeMode*>(fixture.modes[1])->resets;

  fixture.manager->refresh(snapshot, at(10.0));

  // Freezing holds the robot, but what the modes were commanding has to stop as well.
  EXPECT_EQ(static_cast<const FakeMode*>(fixture.modes[1])->resets, before + 1);
}

TEST(ModeManager, ASwitchThatTakesEffectFreezesNothing)
{
  Fixture fixture;
  const auto snapshot = snapshot_with({ "beta_controller" });
  fixture.manager->refresh(snapshot, at(0.0));
  ASSERT_TRUE(fixture.manager->request_next(snapshot, at(1.0)).has_value());

  const auto plan = fixture.manager->refresh(
      snapshot_with({ "gamma_controller", "alpha_controller_left", "alpha_controller_right" }), at(2.0));

  EXPECT_FALSE(plan.has_value());
}

TEST(ModeManager, AnExplicitChoiceOutranksAMoreDemandingMode)
{
  Fixture fixture;
  const auto both = snapshot_with({ "alpha_controller_left", "alpha_controller_right", "gamma_controller" });
  fixture.manager->refresh(both, at(0.0));
  ASSERT_EQ(fixture.manager->active(), fixture.modes[2]);

  // Alpha's controllers satisfy gamma too, so re-reading the listing unconditionally would
  // take the operator back to gamma the moment they chose alpha.
  ASSERT_TRUE(fixture.manager->request_next(both, at(1.0)).has_value());
  fixture.manager->refresh(both, at(2.0));
  ASSERT_EQ(fixture.manager->active(), fixture.modes[0]);

  fixture.manager->refresh(both, at(3.0));
  EXPECT_EQ(fixture.manager->active(), fixture.modes[0]);
}

TEST(ModeManager, AskingForASwitchStopsTheModeBeingLeft)
{
  Fixture fixture;
  const auto snapshot = snapshot_with({ "beta_controller" });
  fixture.manager->refresh(snapshot, at(0.0));
  const int before = static_cast<const FakeMode*>(fixture.modes[1])->resets;

  fixture.manager->request_next(snapshot, at(1.0));

  // Not when the new controllers happen to come up, which is however long that takes.
  EXPECT_EQ(static_cast<const FakeMode*>(fixture.modes[1])->resets, before + 1);
}
