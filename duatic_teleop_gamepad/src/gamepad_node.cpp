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

constexpr const char* kPlatformFocus = "platform";
constexpr const char* kGripperTopicSuffix = "/gripper_controller/commands";

/// How long to wait for a requested mode's controllers before giving up on the switch.
constexpr std::chrono::seconds kModeSwitchTimeout{ 3 };

std::chrono::nanoseconds period_from_rate(double rate_hz)
{
  return std::chrono::nanoseconds(static_cast<int64_t>(1e9 / rate_hz));
}

/// The component a controller topic drives, taken from the controller's name suffix.
std::string component_of(const std::string& topic, const std::string& suffix)
{
  const auto end = topic.rfind(suffix);
  if (end == std::string::npos) {
    return {};
  }

  const auto start = topic.rfind('/', end - 1);
  return start == std::string::npos ? topic.substr(0, end) : topic.substr(start + 1, end - start - 1);
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
  const bool expired = joint_states_.expire(now());
  const bool components_changed = robot_.rebuild(joint_states_.joint_names()) || expired;

  auto components = robot_.component_names(ComponentType::Arm);
  const auto hips = robot_.component_names(ComponentType::Hip);
  components.insert(components.end(), hips.begin(), hips.end());

  discovery_->reconcile(components);

  if (components_changed) {
    rebuild_gripper_publishers();
  }

  const auto& snapshot = controllers_->snapshot();
  if (!snapshot.has_data()) {
    return;
  }

  available_ = available_modes(robot_, snapshot);

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
  jog_pubs_.clear();

  for (const auto& target : discovery_->targets()) {
    jog_groups_.emplace(target.component, JogGroup(target.topic, target.joints));
    jog_pubs_[target.topic] = create_publisher<trajectory_msgs::msg::JointTrajectory>(target.topic, 10);
  }

  if (focus_.empty() && !jog_groups_.empty()) {
    const auto arms = robot_.component_names(ComponentType::Arm);
    set_focus(std::find(arms.begin(), arms.end(), "arm_left") != arms.end() ? "arm_left"
              : arms.empty()                                               ? jog_groups_.begin()->first
                                                                           : arms.front());
  }

  reset_active_mode();
}

void GamepadNode::rebuild_gripper_publishers()
{
  for (const auto& entry : get_topic_names_and_types()) {
    const auto& topic = entry.first;
    if (topic.size() <= std::string(kGripperTopicSuffix).size() ||
        topic.compare(topic.size() - std::string(kGripperTopicSuffix).size(), std::string::npos,
                      kGripperTopicSuffix) != 0) {
      continue;
    }

    const auto component = component_of(topic, kGripperTopicSuffix);
    if (!component.empty() && gripper_pubs_.count(component) == 0) {
      gripper_pubs_[component] = create_publisher<std_msgs::msg::Float64MultiArray>(topic, 1);
      RCLCPP_INFO(get_logger(), "Found a gripper for %s on %s", component.c_str(), topic.c_str());
    }
  }
}

JogGroup* GamepadNode::focused_group()
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
  freeze_ = controllers_->snapshot().freeze_active();

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
    if (const auto next = next_mode(available_, mode_)) {
      apply_mode(*next);
    } else {
      RCLCPP_WARN(get_logger(), "No teleop mode is available to switch to");
    }
  }
  switch_held_ = switch_pressed;
  if (switch_held_) {
    return;
  }

  if (!freeze_) {
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
      if (auto* group = focused_group()) {
        if (can_move) {
          group->set_target_velocities(
              stick_to_velocities(read_sticks(joy, config_.axes, config_.buttons), group->joints().size(),
                                  config_.stick));
        } else {
          group->release();
        }
      }
      break;

    case TeleopMode::Drive: {
      // Once stopped there is nothing to say, so the stream ends with the zero that stops
      // the platform rather than repeating it forever.
      if (!can_move && !drive_active_) {
        break;
      }

      const auto& command = can_move ? drive_.advance(read_sticks(joy, config_.axes, config_.buttons),
                                                      1.0 / config_.input_rate, config_.drive)
                                     : drive_.stop();
      drive_active_ = can_move;

      geometry_msgs::msg::TwistStamped twist;
      twist.header.stamp = now();
      twist.header.frame_id = "base_link";
      twist.twist.linear.x = command.linear_x;
      twist.twist.linear.y = command.linear_y;
      twist.twist.angular.z = command.angular_z;
      drive_pub_->publish(twist);
      break;
    }

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

  auto* group = focused_group();
  if (group == nullptr) {
    return;
  }

  if (!group->tick(1.0 / config_.jog_publish_rate, config_.jog, joint_states_)) {
    send_rumble(0.0);
    return;
  }

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = group->joints();

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = group->commanded_positions();
  point.velocities = group->commanded_velocities();
  point.time_from_start = rclcpp::Duration::from_seconds(1.0 / config_.jog_publish_rate);
  trajectory.points.push_back(std::move(point));

  jog_pubs_[group->topic()]->publish(trajectory);

  send_rumble(group->lagging() ? 1.0 : 0.0);
}

void GamepadNode::apply_mode(TeleopMode mode)
{
  const auto& snapshot = controllers_->snapshot();
  const auto plan =
      plan_switch(required_controllers(mode, snapshot), snapshot.active(), config_.protected_controllers);

  RCLCPP_INFO(get_logger(), "Switching to %s", to_string(mode).c_str());
  controllers_->switch_controllers(plan.activate, plan.deactivate);

  pending_mode_ = mode;
  pending_since_ = now();
}

void GamepadNode::reset_active_mode()
{
  drive_.stop();
  send_rumble(0.0);

  for (auto& entry : jog_groups_) {
    entry.second.reset(joint_states_);
  }
}

void GamepadNode::set_focus(const std::string& component)
{
  if (component.empty() || component == focus_) {
    return;
  }

  if (component != kPlatformFocus && !robot_.has_component(component)) {
    RCLCPP_WARN(get_logger(), "Cannot focus %s: this robot has no such component", component.c_str());
    return;
  }

  focus_ = component;
  RCLCPP_INFO(get_logger(), "Focus is now %s", focus_.c_str());

  reset_active_mode();
  send_rumble(0.5);
}

void GamepadNode::update_focus(const sensor_msgs::msg::Joy& msg)
{
  // Xbox-style pads report the D-Pad as axes and PlayStation-style ones as buttons, so both
  // are read and whichever the pad actually populates wins.
  const double axis_x = axis_value(msg, config_.dpad.axis_x);
  const double axis_y = axis_value(msg, config_.dpad.axis_y);

  const bool up = button_pressed(msg, config_.dpad.button_up);
  const bool down = button_pressed(msg, config_.dpad.button_down);
  const bool left = button_pressed(msg, config_.dpad.button_left);
  const bool right = button_pressed(msg, config_.dpad.button_right);
  const bool any_button = up || down || left || right;

  if (any_button && !dpad_button_held_) {
    set_focus(up      ? config_.dpad.focus_up
              : down  ? config_.dpad.focus_down
              : left  ? config_.dpad.focus_left
                      : config_.dpad.focus_right);
  }
  dpad_button_held_ = any_button;

  if (axis_y > 0.5 && dpad_y_ <= 0.5) {
    set_focus(config_.dpad.focus_up);
  } else if (axis_y < -0.5 && dpad_y_ >= -0.5) {
    set_focus(config_.dpad.focus_down);
  } else if (axis_x > 0.5 && dpad_x_ <= 0.5) {
    set_focus(config_.dpad.focus_left);
  } else if (axis_x < -0.5 && dpad_x_ >= -0.5) {
    set_focus(config_.dpad.focus_right);
  }

  dpad_x_ = axis_x;
  dpad_y_ = axis_y;
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
