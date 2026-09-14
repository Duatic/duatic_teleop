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

#include "duatic_teleop_gamepad/modes/mode_manager.hpp"

using duatic_teleop_gamepad::plan_switch;

namespace
{

const std::vector<std::string> kProtectedBases = { "mecanum_drive_controller", "platform_velocity_controller" };

}  // namespace

TEST(PlanSwitch, ActivatesWhatIsMissingAndStopsWhatIsSpare)
{
  const auto plan = plan_switch({ "a", "b" }, { "b", "c" }, {});

  EXPECT_EQ(plan.activate, (std::vector<std::string>{ "a" }));
  EXPECT_EQ(plan.deactivate, (std::vector<std::string>{ "c" }));
}

TEST(PlanSwitch, LeavesProtectedControllersRunning)
{
  const auto plan = plan_switch({ "freedrive_controller" }, { "mecanum_drive_controller" }, kProtectedBases);

  // Stopping the drive controller loses its odometry, so it is left alone even though
  // freedrive has no use for it.
  EXPECT_TRUE(plan.deactivate.empty());
  EXPECT_EQ(plan.activate, (std::vector<std::string>{ "freedrive_controller" }));
}

TEST(PlanSwitch, ProtectionMatchesOnThePrefix)
{
  const auto plan = plan_switch({}, { "mecanum_drive_controller_front" }, kProtectedBases);

  EXPECT_TRUE(plan.deactivate.empty());
}

TEST(PlanSwitch, SwitchingToWhatIsAlreadyRunningDoesNothing)
{
  const std::vector<std::string> running = { "a", "b", "c" };
  const auto plan = plan_switch(running, running, kProtectedBases);

  EXPECT_TRUE(plan.activate.empty());
  EXPECT_TRUE(plan.deactivate.empty());
}
