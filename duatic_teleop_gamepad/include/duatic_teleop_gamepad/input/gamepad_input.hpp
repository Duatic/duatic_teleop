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

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <sensor_msgs/msg/joy.hpp>

namespace duatic_teleop_gamepad
{

/// Which Joy button index each function sits on.
struct ButtonMapping
{
  int dead_man_switch{ 10 };
  int switch_mode{ 6 };
  int wrist_rotation_left{ 7 };
  int wrist_rotation_right{ 8 };
};

/// Which Joy axis index each stick and trigger sits on.
struct AxisMapping
{
  int left_x{ 0 };
  int left_y{ 1 };
  int right_x{ 2 };
  int right_y{ 3 };
  int trigger_left{ 4 };
  int trigger_right{ 5 };
};

/// The D-Pad, and the component each direction focuses.
///
/// Both forms are read every time: Xbox-style pads report the D-Pad as a pair of axes and
/// PlayStation-style pads as four buttons, and which one a given pad uses is not known
/// ahead of time.
struct DpadMapping
{
  int axis_x{ 6 };
  int axis_y{ 7 };
  int button_up{ 11 };
  int button_down{ 12 };
  int button_left{ 13 };
  int button_right{ 14 };

  /// The component each direction focuses. An empty name leaves that direction unassigned.
  std::string focus_up{ "hip" };
  std::string focus_down;
  std::string focus_left{ "arm_right" };
  std::string focus_right{ "arm_left" };
};

/// The gamepad controls that drive joint jogging, already read off the Joy message.
struct StickInput
{
  double left_x{ 0.0 };
  double left_y{ 0.0 };
  double right_x{ 0.0 };
  double right_y{ 0.0 };
  double trigger_left{ 0.0 };
  double trigger_right{ 0.0 };
  bool wrist_left{ false };
  bool wrist_right{ false };
};

struct StickLimits
{
  /// Joint velocity commanded at full deflection, in rad/s.
  double max_velocity{ 1.0 };

  /// Deflection below which an axis reads as centred.
  double deadzone{ 0.1 };

  /// Deflection at which an axis counts as committed, and the deadzone its neighbour on
  /// the same stick then has to clear, which is what stops a light diagonal creeping both
  /// of that stick's joints at once. Set it equal to deadzone to let both axes drive
  /// together.
  double dominant_axis_threshold{ 0.6 };
};

/// One gamepad reading, reduced to what the teleop logic acts on.
///
/// Every button and axis the node cares about is read once per tick into this, so the
/// logic downstream works on names rather than on indices into a Joy message, and a pad
/// that reports fewer axes than the config maps is dealt with in one place.
struct GamepadInput
{
  StickInput sticks;

  bool deadman{ false };
  bool switch_mode{ false };

  /// Component named by the held D-Pad direction, empty when no direction is held.
  std::string focus_request;

  /// Whether motion may be commanded. Filled in by the node rather than read off the pad,
  /// because it also depends on the E-Stop.
  bool motion_allowed{ false };
};

/// Tracks one button so that holding it registers a single press.
class ButtonEdge
{
public:
  /// @return true on the tick the button goes down, false while it is held or released.
  bool pressed(bool down)
  {
    const bool edge = down && !was_down_;
    was_down_ = down;
    return edge;
  }

private:
  bool was_down_{ false };
};

/// @brief Read one axis, returning zero when the pad does not report it.
///
/// Pads report fewer axes and buttons than a config may map, and an out-of-range read in
/// the input path would throw out of the timer callback that drives teleop.
double axis_value(const sensor_msgs::msg::Joy& msg, int index);

/// Read one button, returning false when the pad does not report it.
bool button_pressed(const sensor_msgs::msg::Joy& msg, int index);

/// Collect the controls that drive jogging out of a Joy message.
StickInput read_sticks(const sensor_msgs::msg::Joy& msg, const AxisMapping& axes, const ButtonMapping& buttons);

/// @brief The component the held D-Pad direction focuses, empty when none is held.
///
/// Both pad styles are reduced to one direction here, so the rest of the node has a single
/// mapping to the focus targets rather than one per pad style.
std::string dpad_focus(const sensor_msgs::msg::Joy& msg, const DpadMapping& dpad);

/// Read one Joy message into everything the teleop logic acts on.
GamepadInput read_input(const sensor_msgs::msg::Joy& msg, const ButtonMapping& buttons, const AxisMapping& axes,
                        const DpadMapping& dpad);

/// @brief Work out the velocity each joint should ramp towards.
/// @param joint_count Number of joints the controller drives; the first six are mapped and
///   any beyond that stay still.
/// @return One velocity per joint, in the controller's own joint order.
std::vector<double> stick_to_velocities(const StickInput& input, std::size_t joint_count, const StickLimits& limits);

}  // namespace duatic_teleop_gamepad
