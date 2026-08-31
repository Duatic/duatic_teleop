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

#include "duatic_teleop_gamepad/gripper_toggle.hpp"

using duatic_teleop_gamepad::GripperToggle;

TEST(GripperToggle, TogglesOnThePressNotTheRelease)
{
  GripperToggle toggle;

  EXPECT_EQ(toggle.update("arm_left", true), 1.0);
  EXPECT_FALSE(toggle.update("arm_left", false).has_value());
  EXPECT_EQ(toggle.update("arm_left", true), 0.0);
}

TEST(GripperToggle, HoldingTheButtonDoesNotRepeat)
{
  GripperToggle toggle;

  EXPECT_EQ(toggle.update("arm_left", true), 1.0);
  EXPECT_FALSE(toggle.update("arm_left", true).has_value());
  EXPECT_FALSE(toggle.update("arm_left", true).has_value());
  EXPECT_TRUE(toggle.is_open("arm_left"));
}

TEST(GripperToggle, ComponentsHoldSeparateStates)
{
  GripperToggle toggle;

  toggle.update("arm_left", true);
  toggle.update("arm_left", false);

  EXPECT_TRUE(toggle.is_open("arm_left"));
  EXPECT_FALSE(toggle.is_open("arm_right"));

  EXPECT_EQ(toggle.update("arm_right", true), 1.0);
  EXPECT_TRUE(toggle.is_open("arm_left"));
  EXPECT_TRUE(toggle.is_open("arm_right"));
}

TEST(GripperToggle, ChangingFocusWhileHeldDoesNotToggle)
{
  GripperToggle toggle;

  ASSERT_EQ(toggle.update("arm_left", true), 1.0);

  // The button never came up, so moving focus must not count as a fresh press.
  EXPECT_FALSE(toggle.update("arm_right", true).has_value());
  EXPECT_FALSE(toggle.is_open("arm_right"));
}

TEST(GripperToggle, WithoutAFocusedComponentNothingIsCommanded)
{
  GripperToggle toggle;

  EXPECT_FALSE(toggle.update("", true).has_value());
}

TEST(GripperToggle, AnUntouchedGripperReadsAsClosed)
{
  const GripperToggle toggle;
  EXPECT_FALSE(toggle.is_open("arm_left"));
}
