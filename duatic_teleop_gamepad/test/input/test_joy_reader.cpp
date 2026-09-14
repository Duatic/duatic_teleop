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

using duatic_teleop_gamepad::AxisMapping;
using duatic_teleop_gamepad::axis_value;
using duatic_teleop_gamepad::button_pressed;
using duatic_teleop_gamepad::ButtonMapping;
using duatic_teleop_gamepad::dpad_focus;
using duatic_teleop_gamepad::DpadMapping;
using duatic_teleop_gamepad::read_input;

namespace
{

/// A pad reporting the eight axes and fifteen buttons the defaults are written for.
sensor_msgs::msg::Joy make_joy()
{
  sensor_msgs::msg::Joy msg;
  msg.axes.assign(8, 0.0F);
  msg.buttons.assign(15, 0);
  return msg;
}

}  // namespace

TEST(JoyReader, AxesAndButtonsAreReadFromTheirIndices)
{
  auto msg = make_joy();
  msg.axes[3] = 0.75F;
  msg.buttons[10] = 1;

  EXPECT_DOUBLE_EQ(axis_value(msg, 3), 0.75);
  EXPECT_TRUE(button_pressed(msg, 10));
  EXPECT_FALSE(button_pressed(msg, 9));
}

TEST(JoyReader, AnIndexThePadDoesNotReportIsIdle)
{
  sensor_msgs::msg::Joy msg;
  msg.axes.assign(2, 1.0F);
  msg.buttons.assign(2, 1);

  // A pad with fewer axes than the config maps must not throw out of the input timer.
  EXPECT_DOUBLE_EQ(axis_value(msg, 7), 0.0);
  EXPECT_FALSE(button_pressed(msg, 14));
}

TEST(JoyReader, ANegativeIndexIsIdle)
{
  const auto msg = make_joy();

  EXPECT_DOUBLE_EQ(axis_value(msg, -1), 0.0);
  EXPECT_FALSE(button_pressed(msg, -1));
}

TEST(JoyReader, APlaystationStyleDpadReportsButtons)
{
  const DpadMapping dpad;
  auto msg = make_joy();
  msg.buttons[dpad.button_left] = 1;

  EXPECT_EQ(dpad_focus(msg, dpad), dpad.focus_left);
}

TEST(JoyReader, AnXboxStyleDpadReportsAxes)
{
  const DpadMapping dpad;
  auto msg = make_joy();
  msg.axes[dpad.axis_y] = 1.0F;

  // The same direction as the button form above reaches the same focus target.
  EXPECT_EQ(dpad_focus(msg, dpad), dpad.focus_up);
}

TEST(JoyReader, AnAxisRestingOffCentreIsNotADirection)
{
  const DpadMapping dpad;
  auto msg = make_joy();
  msg.axes[dpad.axis_x] = 0.4F;

  EXPECT_TRUE(dpad_focus(msg, dpad).empty());
}

TEST(JoyReader, AnUnassignedDirectionNamesNothing)
{
  const DpadMapping dpad;
  auto msg = make_joy();
  msg.buttons[dpad.button_down] = 1;

  // Down has no focus target by default, which is how a direction is left unassigned.
  EXPECT_TRUE(dpad_focus(msg, dpad).empty());
}

TEST(JoyReader, ReadInputCollectsEveryControl)
{
  const ButtonMapping buttons;
  const AxisMapping axes;
  const DpadMapping dpad;

  auto msg = make_joy();
  msg.buttons[buttons.dead_man_switch] = 1;
  msg.buttons[buttons.wrist_rotation_right] = 1;
  msg.buttons[dpad.button_right] = 1;
  msg.axes[axes.left_x] = -0.5F;

  const auto input = read_input(msg, buttons, axes, dpad);

  EXPECT_TRUE(input.deadman);
  EXPECT_TRUE(input.sticks.wrist_right);
  EXPECT_FALSE(input.switch_mode);
  EXPECT_DOUBLE_EQ(input.sticks.left_x, -0.5);
  EXPECT_EQ(input.focus_request, dpad.focus_right);

  // Whether motion is allowed also depends on the E-Stop, which the pad knows nothing of.
  EXPECT_FALSE(input.motion_allowed);
}
