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

#include "duatic_teleop_gamepad/input/gamepad_input.hpp"

using duatic_teleop_gamepad::StickInput;
using duatic_teleop_gamepad::StickLimits;
using duatic_teleop_gamepad::stick_to_velocities;

namespace
{

constexpr std::size_t kSevenJoints = 7;

StickLimits limits()
{
  return StickLimits{ 1.0, 0.1, 0.6 };
}

/// Limits under which neither axis of a stick suppresses the other.
StickLimits without_dominance()
{
  return StickLimits{ 1.0, 0.1, 0.1 };
}

}  // namespace

TEST(StickMapping, EachControlDrivesItsOwnJoint)
{
  StickInput input;
  input.left_x = 1.0;
  input.right_y = -1.0;

  const auto velocities = stick_to_velocities(input, kSevenJoints, limits());

  EXPECT_DOUBLE_EQ(velocities[0], 1.0);
  EXPECT_DOUBLE_EQ(velocities[1], 0.0);
  EXPECT_DOUBLE_EQ(velocities[2], -1.0);
  EXPECT_DOUBLE_EQ(velocities[3], 0.0);
}

TEST(StickMapping, DeflectionScalesByTheVelocityLimit)
{
  StickInput input;
  input.left_x = 0.5;

  StickLimits scaled = limits();
  scaled.max_velocity = 2.0;

  EXPECT_DOUBLE_EQ(stick_to_velocities(input, kSevenJoints, scaled)[0], 1.0);
}

TEST(StickMapping, DeflectionInsideTheDeadzoneIsIgnored)
{
  StickInput input;
  input.left_x = 0.05;

  EXPECT_DOUBLE_EQ(stick_to_velocities(input, kSevenJoints, limits())[0], 0.0);
}

TEST(StickMapping, TheLeadingAxisSuppressesTheOtherOne)
{
  StickInput input;
  input.left_y = 0.9;
  input.left_x = 0.3;

  // A diagonal push drives only the joint belonging to the axis that is furthest from
  // centre, so the arm does not creep on two joints at once.
  const auto velocities = stick_to_velocities(input, kSevenJoints, limits());

  EXPECT_DOUBLE_EQ(velocities[1], 0.9);
  EXPECT_DOUBLE_EQ(velocities[0], 0.0);
}

TEST(StickMapping, TheSuppressedAxisStillWinsPastTheThreshold)
{
  StickInput input;
  input.left_y = 0.9;
  input.left_x = 0.7;

  // Deliberate: a genuinely committed diagonal drives both, so the suppression is a
  // deadzone and not a lockout.
  const auto velocities = stick_to_velocities(input, kSevenJoints, limits());

  EXPECT_DOUBLE_EQ(velocities[1], 0.9);
  EXPECT_DOUBLE_EQ(velocities[0], 0.7);
}

TEST(StickMapping, AnUncommittedDiagonalIsNotDecidedByWhichAxisIsLarger)
{
  // The two axes of a stick held on a diagonal sit within noise of each other, so deciding
  // the lead by comparing them hands it back and forth at the input rate and the joints
  // rattle. Neither axis is committed here, so both drive whichever one happens to lead.
  StickInput leading_x;
  leading_x.left_x = 0.41;
  leading_x.left_y = 0.40;

  StickInput leading_y;
  leading_y.left_x = 0.40;
  leading_y.left_y = 0.41;

  const auto with_x = stick_to_velocities(leading_x, kSevenJoints, limits());
  const auto with_y = stick_to_velocities(leading_y, kSevenJoints, limits());

  EXPECT_DOUBLE_EQ(with_x[0], 0.41);
  EXPECT_DOUBLE_EQ(with_x[1], 0.40);
  EXPECT_DOUBLE_EQ(with_y[0], 0.40);
  EXPECT_DOUBLE_EQ(with_y[1], 0.41);
}

TEST(StickMapping, DominanceCanBeTurnedOff)
{
  StickInput input;
  input.left_y = 0.9;
  input.left_x = 0.3;

  const auto velocities = stick_to_velocities(input, kSevenJoints, without_dominance());

  EXPECT_DOUBLE_EQ(velocities[1], 0.9);
  EXPECT_DOUBLE_EQ(velocities[0], 0.3);
}

TEST(StickMapping, TheSticksDoNotSuppressEachOther)
{
  StickInput input;
  input.left_y = 0.9;
  input.right_x = 0.3;

  const auto velocities = stick_to_velocities(input, kSevenJoints, limits());

  EXPECT_DOUBLE_EQ(velocities[1], 0.9);
  EXPECT_DOUBLE_EQ(velocities[3], 0.3);
}

TEST(StickMapping, TriggersWorkAgainstEachOther)
{
  StickInput input;
  input.trigger_right = 1.0;
  input.trigger_left = 0.0;
  EXPECT_DOUBLE_EQ(stick_to_velocities(input, kSevenJoints, limits())[4], 1.0);

  input.trigger_left = 1.0;
  EXPECT_DOUBLE_EQ(stick_to_velocities(input, kSevenJoints, limits())[4], 0.0);

  input.trigger_right = 0.0;
  EXPECT_DOUBLE_EQ(stick_to_velocities(input, kSevenJoints, limits())[4], -1.0);
}

TEST(StickMapping, RestingTriggersCommandNothing)
{
  StickInput input;

  // Triggers commonly rest at -1.0 rather than 0.0, and the difference is what matters.
  input.trigger_left = -1.0;
  input.trigger_right = -1.0;

  EXPECT_DOUBLE_EQ(stick_to_velocities(input, kSevenJoints, limits())[4], 0.0);
}

TEST(StickMapping, WristButtonsDriveTheLastJoint)
{
  StickInput input;

  input.wrist_left = true;
  EXPECT_DOUBLE_EQ(stick_to_velocities(input, kSevenJoints, limits())[5], -1.0);

  input.wrist_left = false;
  input.wrist_right = true;
  EXPECT_DOUBLE_EQ(stick_to_velocities(input, kSevenJoints, limits())[5], 1.0);

  // Both at once cancels rather than picking one arbitrarily.
  input.wrist_left = true;
  EXPECT_DOUBLE_EQ(stick_to_velocities(input, kSevenJoints, limits())[5], -1.0);
}

TEST(StickMapping, JointsBeyondTheMappedSixStayStill)
{
  StickInput input;
  input.left_x = 1.0;
  input.left_y = 1.0;
  input.right_x = 1.0;
  input.right_y = 1.0;
  input.trigger_right = 1.0;
  input.wrist_right = true;

  const auto velocities = stick_to_velocities(input, kSevenJoints, limits());

  ASSERT_EQ(velocities.size(), kSevenJoints);
  EXPECT_DOUBLE_EQ(velocities[6], 0.0);
}

TEST(StickMapping, AShortControllerOnlyGetsTheJointsItHas)
{
  StickInput input;
  input.left_x = 1.0;

  EXPECT_EQ(stick_to_velocities(input, 2, limits()).size(), 2u);
}
