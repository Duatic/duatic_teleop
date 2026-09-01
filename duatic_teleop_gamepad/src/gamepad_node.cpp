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

#include "duatic_teleop_gamepad/gamepad_node.hpp"

#include <algorithm>
#include <chrono>
#include <string>
#include <utility>

namespace duatic_teleop_gamepad
{

namespace
{

constexpr const char* kGripperTopicSuffix = "gripper_controller/commands";

/// How long to wait for a requested mode's controllers before giving up on the switch.
constexpr std::chrono::seconds kModeSwitchTimeout{ 3 };

std::chrono::nanoseconds period_from_rate(double rate_hz)
{
  return std::chrono::nanoseconds(static_cast<int64_t>(1e9 / rate_hz));
}

}  // namespace

GamepadNode::GamepadNode()
  : rclcpp::Node("gamepad_interface")
  , config_(declare_config(*this))
  , joint_states_(get_logger(), rclcpp::Duration::from_seconds(config_.joint_state_timeout))
  , pending_since_(now())
{
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, [this](sensor_msgs::msg::Joy::ConstSharedPtr msg) { on_joy(std::move(msg)); });
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) { on_joint_states(std::move(msg)); });

  move_home_pub_ = create_publisher<std_msgs::msg::Bool>("move_home", 10);
  move_sleep_pub_ = create_publisher<std_msgs::msg::Bool>("move_sleep", 10);
  drive_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_smoothed", 10);
  feedback_pub_ = create_publisher<sensor_msgs::msg::JoyFeedback>("joy/set_feedback", 10);

  controllers_ = std::make_unique<ControllerManagerClient>(*this, config_.managed_controllers,
                                                           period_from_rate(config_.discovery_rate));
  discovery_ = std::make_unique<JtcDiscovery>(*this, [this]() { rebuild_jog_groups(); });

  input_timer_ = create_wall_timer(period_from_rate(config_.input_rate), [this]() { process_input(); });
  jog_timer_ = create_wall_timer(period_from_rate(config_.jog_publish_rate), [this]() { publish_jog(); });
  discovery_timer_ = create_wall_timer(period_from_rate(config_.discovery_rate), [this]() { discover(); });

  RCLCPP_INFO(get_logger(), "Gamepad interface started (input %.0f Hz, jog %.0f Hz), waiting for the robot",
              config_.input_rate, config_.jog_publish_rate);
}

void GamepadNode::on_joy(sensor_msgs::msg::Joy::ConstSharedPtr msg)
{
  latest_joy_ = std::move(msg);
}

void GamepadNode::on_joint_states(sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  joint_states_.update(*msg, now());
}

void GamepadNode::discover()
{
  joint_states_.expire(now());
  discovery_->reconcile();

  // Unconditionally, because a gripper controller spawned after the robot's joint set has
  // settled changes no joint name, and keying this off a component change would leave that
  // gripper without a publisher for the lifetime of the node. Known components are skipped
  // inside, so repeating the scan costs a topic listing.
  rebuild_gripper_publishers();

  const auto& snapshot = controllers_->snapshot();
  if (!snapshot.has_data()) {
    return;
  }

  available_ = available_modes(snapshot);

  // A requested mode owns the state machine until its controllers are actually running,
  // otherwise adopting whatever is active right now would undo the switch that is still in
  // flight.
  if (pending_mode_) {
    const auto required = required_controllers(*pending_mode_, snapshot);
    const bool satisfied = !required.empty() &&
                           std::all_of(required.begin(), required.end(),
                                       [&snapshot](const std::string& name) { return snapshot.is_active(name); });

    if (satisfied) {
      mode_ = *pending_mode_;
      pending_mode_.reset();
      RCLCPP_INFO(get_logger(), "Mode is now %s", to_string(*mode_).c_str());
      reset_active_mode();
    } else if (now() - pending_since_ > rclcpp::Duration(kModeSwitchTimeout)) {
      RCLCPP_WARN(get_logger(), "Switching to %s did not take effect", to_string(*pending_mode_).c_str());
      pending_mode_.reset();
    }
    return;
  }

  // An explicit choice stands as long as it still holds. Several modes can be satisfied at
  // once -- driving needs everything jogging does plus the base, and the base controller is
  // protected from ever being stopped -- so re-reading the controllers unconditionally
  // would let the more demanding mode win an argument the operator already settled.
  if (mode_) {
    const auto required = required_controllers(*mode_, snapshot);
    const bool still_holds = !required.empty() &&
                             std::all_of(required.begin(), required.end(),
                                         [&snapshot](const std::string& name) { return snapshot.is_active(name); });
    if (still_holds) {
      return;
    }
  }

  // Otherwise adopt what the controllers say the robot is doing, which covers both starting
  // up into a running robot and something else switching controllers underneath.
  const auto inferred = infer_mode(available_, snapshot);
  if (inferred && inferred != mode_) {
    mode_ = inferred;
    RCLCPP_INFO(get_logger(), "Adopted mode %s from the active controllers", to_string(*mode_).c_str());
    reset_active_mode();
  }
}

void GamepadNode::rebuild_jog_groups()
{
  jog_groups_.clear();

  for (const auto& target : discovery_->targets()) {
    jog_groups_.emplace(target.component,
                        JogTarget{ JogGroup(target.topic, target.joints),
                                   create_publisher<trajectory_msgs::msg::JointTrajectory>(target.topic, 10) });
  }

  // Re-applied on every rebuild until the operator picks for themselves, because the jog
  // groups appear one parameter response at a time and in no particular order. Settling on
  // whichever one happened to register first would leave the focus on the hip whenever its
  // controller answered before the arms.
  if (!focus_chosen_ && !jog_groups_.empty()) {
    set_focus(jog_groups_.count("arm_left") != 0 ? "arm_left" : jog_groups_.begin()->first);
  }

  reset_active_mode();
}

void GamepadNode::rebuild_gripper_publishers()
{
  for (const auto& entry : get_topic_names_and_types()) {
    const auto component = component_from_topic(entry.first, kGripperTopicSuffix);

    // Checked against the jog groups, so a topic that merely looks like one cannot register
    // a gripper for a component the gamepad cannot even focus.
    if (component.empty() || jog_groups_.count(component) == 0 || gripper_pubs_.count(component) != 0) {
      continue;
    }

    gripper_pubs_[component] = create_publisher<std_msgs::msg::Float64MultiArray>(entry.first, 1);
    RCLCPP_INFO(get_logger(), "Found a gripper for %s on %s", component.c_str(), entry.first.c_str());
  }
}

GamepadNode::JogTarget* GamepadNode::focused_target()
{
  const auto match = jog_groups_.find(focus_);
  return match == jog_groups_.end() ? nullptr : &match->second;
}

void GamepadNode::process_input()
{
  if (!latest_joy_) {
    return;
  }

  const auto& joy = *latest_joy_;
  const bool was_frozen = freeze_;

  // Until the controller manager has answered once there is no way to tell an engaged
  // E-Stop from a clear one, and the safe reading of the two is the engaged one.
  const auto& snapshot = controllers_->snapshot();
  freeze_ = !snapshot.has_data() || snapshot.freeze_active();

  update_focus(joy);

  const bool was_held = deadman_;
  deadman_ = button_pressed(joy, config_.buttons.dead_man_switch);
  const bool can_move = deadman_ && !freeze_;

  // Reset on every edge that changes whether motion is allowed, so the first command after
  // one always starts from where the robot actually is.
  if (deadman_ != was_held || freeze_ != was_frozen) {
    reset_active_mode();
  }

  if (!can_move) {
    if (move_command_active_) {
      stop_move_commands();
    }
  } else if (button_pressed(joy, config_.buttons.move_home)) {
    move_home_pub_->publish(std_msgs::msg::Bool().set__data(true));
    move_command_active_ = true;
    return;
  } else if (button_pressed(joy, config_.buttons.move_sleep)) {
    move_sleep_pub_->publish(std_msgs::msg::Bool().set__data(true));
    move_command_active_ = true;
    return;
  } else if (move_command_active_) {
    stop_move_commands();
  }

  const bool switch_pressed = button_pressed(joy, config_.buttons.switch_mode);
  if (switch_pressed && !switch_held_) {
    // A switch deactivates every managed controller the next mode does not need, and while
    // the E-Stop is engaged that set includes the freeze controllers holding the robot. So
    // the switch waits for the E-Stop to clear rather than taking it down.
    if (freeze_) {
      RCLCPP_WARN(get_logger(), "Cannot switch modes while the E-Stop is engaged");
    } else if (const auto next = next_mode(available_, mode_)) {
      apply_mode(*next);
    } else {
      RCLCPP_WARN(get_logger(), "No teleop mode is available to switch to");
    }
  }
  switch_held_ = switch_pressed;
  if (switch_held_) {
    return;
  }

  // The gripper commands motion, so it is held to the same deadman and E-Stop conditions
  // as every other command.
  if (can_move) {
    if (const auto position = gripper_.update(focus_, button_pressed(joy, config_.buttons.gripper))) {
      const auto publisher = gripper_pubs_.find(focus_);
      if (publisher != gripper_pubs_.end()) {
        publisher->second->publish(std_msgs::msg::Float64MultiArray().set__data({ *position }));
      }
    }
  }

  if (!mode_) {
    return;
  }

  switch (*mode_) {
    case TeleopMode::Jog:
      if (auto* target = focused_target()) {
        if (can_move) {
          target->group.set_target_velocities(stick_to_velocities(read_sticks(joy, config_.axes, config_.buttons),
                                                                  target->group.joints().size(), config_.stick));
        } else {
          target->group.release();
        }
      }
      break;

    case TeleopMode::Drive:
      // Stopping is reset_active_mode()'s job, which every edge that ends a drive goes
      // through, so the stream simply ends here rather than repeating the zero forever.
      if (can_move) {
        publish_drive(drive_.advance(read_sticks(joy, config_.axes, config_.buttons), 1.0 / config_.input_rate,
                                     config_.drive));
        drive_active_ = true;
      }
      break;

    case TeleopMode::Freedrive:
      // The controller does the work; the gamepad only had to activate it.
      break;
  }
}

void GamepadNode::publish_jog()
{
  if (freeze_ || mode_ != TeleopMode::Jog) {
    return;
  }

  auto* target = focused_target();
  if (target == nullptr) {
    return;
  }

  if (!target->group.tick(1.0 / config_.jog_publish_rate, config_.jog, joint_states_)) {
    send_rumble(0.0);
    return;
  }

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = target->group.joints();

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = target->group.commanded_positions();
  point.velocities = target->group.commanded_velocities();
  point.time_from_start = rclcpp::Duration::from_seconds(1.0 / config_.jog_publish_rate);
  trajectory.points.push_back(std::move(point));

  target->publisher->publish(trajectory);

  send_rumble(target->group.lagging() ? 1.0 : 0.0);
}

void GamepadNode::apply_mode(TeleopMode mode)
{
  const auto& snapshot = controllers_->snapshot();
  const auto plan =
      plan_switch(required_controllers(mode, snapshot), snapshot.active(), config_.protected_controllers);

  RCLCPP_INFO(get_logger(), "Switching to %s", to_string(mode).c_str());
  controllers_->switch_controllers(plan.activate, plan.deactivate);

  // The mode being left stops as soon as the operator asks for the switch, rather than
  // whenever the new controllers happen to come up.
  reset_active_mode();

  pending_mode_ = mode;
  pending_since_ = now();
}

void GamepadNode::publish_drive(const DriveCommand& command)
{
  geometry_msgs::msg::TwistStamped twist;
  twist.header.stamp = now();
  twist.header.frame_id = "base_link";
  twist.twist.linear.x = command.linear_x;
  twist.twist.linear.y = command.linear_y;
  twist.twist.angular.z = command.angular_z;
  drive_pub_->publish(twist);
}

void GamepadNode::reset_active_mode()
{
  // Every edge that ends a drive comes through here -- the deadman, the E-Stop, a mode
  // change -- and the platform holds its last command until it is told otherwise, so the
  // stopping zero has to go out rather than only being zeroed in the ramp.
  const bool was_driving = drive_active_;
  drive_active_ = false;

  const auto& stopped = drive_.stop();
  if (was_driving) {
    publish_drive(stopped);
  }

  send_rumble(0.0);

  for (auto& entry : jog_groups_) {
    entry.second.group.reset(joint_states_);
  }
}

void GamepadNode::set_focus(const std::string& component)
{
  // An unset target is how a D-Pad direction is left unassigned.
  if (component.empty() || component == focus_) {
    return;
  }

  if (jog_groups_.count(component) == 0) {
    RCLCPP_WARN(get_logger(), "Cannot focus %s: no trajectory controller drives it", component.c_str());
    return;
  }

  focus_ = component;
  RCLCPP_INFO(get_logger(), "Focus is now %s", focus_.c_str());

  reset_active_mode();
  send_rumble(0.5);
}

void GamepadNode::update_focus(const sensor_msgs::msg::Joy& msg)
{
  // Xbox-style pads report the D-Pad as a pair of axes and PlayStation-style ones as four
  // buttons. Both are reduced to one direction here so there is a single edge to detect and
  // a single mapping to the focus targets, rather than one of each per pad style.
  const double axis_x = axis_value(msg, config_.dpad.axis_x);
  const double axis_y = axis_value(msg, config_.dpad.axis_y);

  const auto direction = [&]() -> const std::string* {
    if (button_pressed(msg, config_.dpad.button_up) || axis_y > 0.5) {
      return &config_.dpad.focus_up;
    }
    if (button_pressed(msg, config_.dpad.button_down) || axis_y < -0.5) {
      return &config_.dpad.focus_down;
    }
    if (button_pressed(msg, config_.dpad.button_left) || axis_x > 0.5) {
      return &config_.dpad.focus_left;
    }
    if (button_pressed(msg, config_.dpad.button_right) || axis_x < -0.5) {
      return &config_.dpad.focus_right;
    }
    return nullptr;
  }();

  if (direction != nullptr && !dpad_held_) {
    focus_chosen_ = true;
    set_focus(*direction);
  }

  dpad_held_ = direction != nullptr;
}

void GamepadNode::send_rumble(double intensity)
{
  // Rumble persists until it is changed, so it needs a message on each edge and none in
  // between.
  const bool wanted = intensity > 0.0;
  if (wanted == rumbling_) {
    return;
  }

  rumbling_ = wanted;

  sensor_msgs::msg::JoyFeedback feedback;
  feedback.type = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
  feedback.id = 0;
  feedback.intensity = static_cast<float>(intensity);
  feedback_pub_->publish(feedback);
}

void GamepadNode::stop_move_commands()
{
  move_home_pub_->publish(std_msgs::msg::Bool().set__data(false));
  move_sleep_pub_->publish(std_msgs::msg::Bool().set__data(false));
  move_command_active_ = false;
  reset_active_mode();
}

}  // namespace duatic_teleop_gamepad
