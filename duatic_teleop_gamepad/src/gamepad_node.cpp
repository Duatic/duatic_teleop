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

#include <chrono>
#include <utility>
#include <vector>

#include "duatic_teleop_gamepad/input/gamepad_input.hpp"
#include "duatic_teleop_gamepad/modes/drive_mode.hpp"
#include "duatic_teleop_gamepad/modes/freedrive_mode.hpp"
#include "duatic_teleop_gamepad/modes/jog_mode.hpp"

namespace duatic_teleop_gamepad
{

namespace
{

/// The component focused when the operator has not picked one, where the robot has it.
constexpr const char* kPreferredFocus = "arm_left";

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
{
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, [this](sensor_msgs::msg::Joy::ConstSharedPtr msg) { on_joy(std::move(msg)); });
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) { on_joint_states(std::move(msg)); });

  feedback_pub_ = create_publisher<sensor_msgs::msg::JoyFeedback>("joy/set_feedback", 10);

  controllers_ = std::make_unique<ControllerManagerClient>(*this, config_.managed_controllers,
                                                           period_from_rate(config_.discovery_rate));
  discovery_ = std::make_unique<JtcDiscovery>(*this, [this]() { on_robot_changed(); });

  // The order is the order the mode button cycles in.
  const TeleopContext context{ *this, config_, joint_states_, *discovery_ };
  std::vector<std::unique_ptr<BaseMode>> modes;
  modes.push_back(std::make_unique<FreedriveMode>());
  modes.push_back(std::make_unique<JogMode>(context));
  modes.push_back(std::make_unique<DriveMode>(context));
  modes_ = std::make_unique<ModeManager>(std::move(modes), get_logger(), config_.protected_controllers,
                                         rclcpp::Duration(kModeSwitchTimeout));

  input_timer_ = create_wall_timer(period_from_rate(config_.input_rate), [this]() { process_input(); });
  publish_timer_ = create_wall_timer(period_from_rate(config_.jog_publish_rate), [this]() { publish_mode(); });
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

  const auto& snapshot = controllers_->snapshot();
  if (!snapshot.has_data()) {
    return;
  }

  if (const auto plan = modes_->refresh(snapshot, now())) {
    controllers_->switch_controllers(plan->activate, plan->deactivate);
  }
}

void GamepadNode::on_robot_changed()
{
  modes_->on_robot_changed();

  // Re-applied on every change until the operator picks for themselves, because the
  // trajectory controllers answer one parameter request at a time and in no particular
  // order. Settling on whichever one happened to answer first would leave the focus on the
  // hip whenever its controller beat the arms to it.
  const auto& targets = discovery_->targets();
  if (!focus_chosen_ && !targets.empty()) {
    set_focus(discovery_->drives(kPreferredFocus) ? kPreferredFocus : targets.front().component);
  }

  modes_->reset();
}

void GamepadNode::process_input()
{
  if (!latest_joy_) {
    return;
  }

  // Until the controller manager has answered once there is no way to tell an engaged
  // E-Stop from a clear one, and the safe reading of the two is the engaged one.
  const auto& snapshot = controllers_->snapshot();
  const bool was_frozen = freeze_;
  freeze_ = !snapshot.has_data() || snapshot.freeze_active();

  auto input = read_input(*latest_joy_, config_.buttons, config_.axes, config_.dpad);
  input.motion_allowed = input.deadman && !freeze_;

  update_focus(input);

  // Reset on every edge that changes whether motion is allowed, so the first command after
  // one always starts from where the robot actually is.
  const bool was_held = deadman_;
  deadman_ = input.deadman;
  if (deadman_ != was_held || freeze_ != was_frozen) {
    modes_->reset();
  }

  if (switch_button_.pressed(input.switch_mode)) {
    // A switch deactivates every managed controller the next mode does not need, and while
    // the E-Stop is engaged that set includes the freeze controllers holding the robot. So
    // the switch waits for the E-Stop to clear rather than taking it down.
    if (freeze_) {
      RCLCPP_WARN(get_logger(), "Cannot switch modes while the E-Stop is engaged");
    } else if (const auto plan = modes_->request_next(snapshot, now())) {
      controllers_->switch_controllers(plan->activate, plan->deactivate);
    } else {
      RCLCPP_WARN(get_logger(), "No teleop mode is available to switch to");
    }
  }

  if (input.switch_mode) {
    return;
  }

  if (auto* mode = modes_->active()) {
    mode->on_input(input, 1.0 / config_.input_rate);
  }
}

void GamepadNode::publish_mode()
{
  if (freeze_) {
    return;
  }

  auto* mode = modes_->active();
  if (mode == nullptr) {
    return;
  }

  mode->publish(1.0 / config_.jog_publish_rate);
  send_rumble(mode->feedback());
}

void GamepadNode::set_focus(const std::string& component)
{
  // An unset target is how a D-Pad direction is left unassigned.
  if (component.empty() || component == focus_) {
    return;
  }

  if (!discovery_->drives(component)) {
    RCLCPP_WARN(get_logger(), "Cannot focus %s: no trajectory controller drives it", component.c_str());
    return;
  }

  focus_ = component;
  RCLCPP_INFO(get_logger(), "Focus is now %s", focus_.c_str());

  modes_->set_focus(focus_);
  modes_->reset();
  send_rumble(0.5);
}

void GamepadNode::update_focus(const GamepadInput& input)
{
  if (focus_button_.pressed(!input.focus_request.empty())) {
    focus_chosen_ = true;
    set_focus(input.focus_request);
  }
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

}  // namespace duatic_teleop_gamepad
