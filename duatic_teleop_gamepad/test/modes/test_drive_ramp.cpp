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

#include <cmath>
#include <limits>

#include "duatic_teleop_gamepad/modes/drive_mode.hpp"

using duatic_teleop_gamepad::DriveLimits;
using duatic_teleop_gamepad::DriveRamp;
using duatic_teleop_gamepad::ramp_velocity;
using duatic_teleop_gamepad::StickInput;

namespace
{

constexpr double kDt = 0.01;

DriveLimits limits()
{
  return DriveLimits{ 0.6, 0.5, 1.0, 0.05 };
}

StickInput forward(double deflection)
{
  StickInput input;
  input.left_y = deflection;
  return input;
}

}  // namespace

TEST(RampVelocity, AcceleratesAtTheAccelerationLimit)
{
  EXPECT_DOUBLE_EQ(ramp_velocity(0.0, 1.0, kDt, limits()), 0.005);
}

TEST(RampVelocity, DeceleratesAtTheDecelerationLimit)
{
  EXPECT_DOUBLE_EQ(ramp_velocity(1.0, 0.0, kDt, limits()), 1.0 - 0.01);
}

TEST(RampVelocity, ReversingUsesTheDecelerationLimit)
{
  // Reversing passes through a stop, and the part before the stop is slowing down.
  EXPECT_DOUBLE_EQ(ramp_velocity(0.5, -0.5, kDt, limits()), 0.5 - 0.01);
}

TEST(RampVelocity, LandsExactlyOnTheTarget)
{
  EXPECT_DOUBLE_EQ(ramp_velocity(0.5, 0.502, kDt, limits()), 0.502);
  EXPECT_DOUBLE_EQ(ramp_velocity(0.5, 0.5, kDt, limits()), 0.5);
}

TEST(DriveRamp, StartsStopped)
{
  DriveRamp ramp;
  const auto& command = ramp.stop();

  EXPECT_DOUBLE_EQ(command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(command.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(command.angular_z, 0.0);
}

TEST(DriveRamp, SticksMapToTheirAxes)
{
  DriveRamp ramp;
  StickInput input;
  input.left_y = 1.0;
  input.left_x = -1.0;
  input.right_x = 1.0;

  // One step is enough to see the sign of each axis.
  const auto& command = ramp.advance(input, kDt, limits());

  EXPECT_GT(command.linear_x, 0.0);
  EXPECT_LT(command.linear_y, 0.0);
  EXPECT_GT(command.angular_z, 0.0);
}

TEST(DriveRamp, ReachesTheVelocityLimitAndStaysThere)
{
  DriveRamp ramp;
  double linear_x = 0.0;

  for (int i = 0; i < 1000; ++i) {
    linear_x = ramp.advance(forward(1.0), kDt, limits()).linear_x;
  }

  EXPECT_DOUBLE_EQ(linear_x, 0.6);
}

TEST(DriveRamp, DeflectionInsideTheDeadzoneIsAStop)
{
  DriveRamp ramp;
  double linear_x = 0.0;

  for (int i = 0; i < 1000; ++i) {
    linear_x = ramp.advance(forward(1.0), kDt, limits()).linear_x;
  }
  ASSERT_GT(linear_x, 0.0);

  for (int i = 0; i < 1000; ++i) {
    linear_x = ramp.advance(forward(0.01), kDt, limits()).linear_x;
  }

  EXPECT_DOUBLE_EQ(linear_x, 0.0);
}

TEST(DriveRamp, StopIsImmediate)
{
  DriveRamp ramp;
  double linear_x = 0.0;

  for (int i = 0; i < 1000; ++i) {
    linear_x = ramp.advance(forward(1.0), kDt, limits()).linear_x;
  }
  ASSERT_GT(linear_x, 0.0);

  // Releasing the deadman has to stop the platform now, not over the deceleration ramp.
  EXPECT_DOUBLE_EQ(ramp.stop().linear_x, 0.0);
}

TEST(DriveRamp, ANonFiniteAxisIsTreatedAsCentred)
{
  DriveRamp ramp;
  double linear_x = 0.0;

  for (int i = 0; i < 1000; ++i) {
    linear_x = ramp.advance(forward(1.0), kDt, limits()).linear_x;
  }
  ASSERT_GT(linear_x, 0.0);

  // A driver emitting NaN must not be able to poison the command with it.
  for (int i = 0; i < 1000; ++i) {
    linear_x = ramp.advance(forward(std::numeric_limits<double>::quiet_NaN()), kDt, limits()).linear_x;
  }

  EXPECT_TRUE(std::isfinite(linear_x));
  EXPECT_DOUBLE_EQ(linear_x, 0.0);
}

TEST(DriveRamp, DeflectionBeyondFullScaleIsClamped)
{
  DriveRamp ramp;
  double linear_x = 0.0;

  for (int i = 0; i < 1000; ++i) {
    linear_x = ramp.advance(forward(5.0), kDt, limits()).linear_x;
  }

  EXPECT_DOUBLE_EQ(linear_x, 0.6);
}
