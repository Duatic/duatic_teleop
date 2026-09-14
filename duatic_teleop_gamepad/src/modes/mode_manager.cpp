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

#include "duatic_teleop_gamepad/modes/mode_manager.hpp"

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <utility>

#include <rclcpp/logging.hpp>

namespace duatic_teleop_gamepad
{

namespace
{

/// @brief The switch that holds the robot where it is.
///
/// Every freeze controller the robot has, activated, and nothing deactivated: the same
/// thing the E-Stop node asks for, so that a robot frozen this way is in the state the rest
/// of the stack already knows how to recover from.
ControllerSwitch freeze_switch(const ControllerSnapshot& controllers)
{
  return ControllerSwitch{ controllers.matching({ kFreezeControllerBase }), {} };
}

/// Whether a mode's controllers are all running. An empty requirement means the mode has
/// nothing loaded to back it, which is not the same as being satisfied by nothing.
bool all_active(const std::vector<std::string>& required, const ControllerSnapshot& controllers)
{
  return !required.empty() && std::all_of(required.begin(), required.end(), [&controllers](const std::string& name) {
           return controllers.is_active(name);
         });
}

}  // namespace

ControllerSwitch plan_switch(const std::vector<std::string>& needed, const std::vector<std::string>& active,
                             const std::vector<std::string>& protected_bases)
{
  ControllerSwitch plan;

  for (const auto& name : needed) {
    if (std::find(active.begin(), active.end(), name) == active.end()) {
      plan.activate.push_back(name);
    }
  }

  for (const auto& name : active) {
    if (std::find(needed.begin(), needed.end(), name) != needed.end()) {
      continue;
    }

    const bool is_protected = std::any_of(protected_bases.begin(), protected_bases.end(),
                                          [&name](const std::string& base) { return name.starts_with(base); });
    if (!is_protected) {
      plan.deactivate.push_back(name);
    }
  }

  std::sort(plan.activate.begin(), plan.activate.end());
  std::sort(plan.deactivate.begin(), plan.deactivate.end());
  return plan;
}

ModeManager::ModeManager(std::vector<std::unique_ptr<BaseMode>> modes, rclcpp::Logger logger,
                         std::vector<std::string> protected_bases, rclcpp::Duration switch_timeout)
  : modes_(std::move(modes))
  , logger_(std::move(logger))
  , protected_bases_(std::move(protected_bases))
  , switch_timeout_(switch_timeout)
{
}

BaseMode* ModeManager::active()
{
  return active_;
}

const std::vector<BaseMode*>& ModeManager::available() const
{
  return available_;
}

std::optional<ControllerSwitch> ModeManager::refresh(const ControllerSnapshot& controllers,
                                                     const rclcpp::Time& now)
{
  available_.clear();
  for (const auto& mode : modes_) {
    if (!controllers.matching(mode->controller_bases()).empty()) {
      available_.push_back(mode.get());
    }
  }

  // A requested mode owns the decision until its controllers are actually running,
  // otherwise adopting whatever is active right now would undo the switch that is still in
  // flight.
  if (pending_ != nullptr) {
    if (satisfied(*pending_, controllers)) {
      active_ = std::exchange(pending_, nullptr);
      RCLCPP_INFO(logger_, "Mode is now %s", active_->name().c_str());
      reset();
    } else if (now - pending_since_ > switch_timeout_) {
      // A switch that only half happened can leave the arms with no controller holding
      // them, and from here there is no way to tell that from a switch that was refused
      // outright. So the robot is frozen rather than left in whatever state it reached.
      RCLCPP_ERROR(logger_, "Switching to %s did not take effect, engaging the freeze controllers",
                   pending_->name().c_str());
      pending_ = nullptr;
      reset();
      return freeze_switch(controllers);
    }
    return std::nullopt;
  }

  // An explicit choice stands as long as it still holds. Several modes can be satisfied at
  // once (driving needs everything jogging does plus the base, and the base controller is
  // protected from ever being stopped), so re-reading the controllers unconditionally would
  // let the more demanding mode win an argument the operator already settled.
  if (active_ != nullptr && satisfied(*active_, controllers)) {
    return std::nullopt;
  }

  // Otherwise adopt what the controllers say the robot is doing, which covers both starting
  // up into a running robot and something else switching controllers underneath.
  auto* inferred = infer(controllers);
  if (inferred != nullptr && inferred != active_) {
    active_ = inferred;
    RCLCPP_INFO(logger_, "Adopted mode %s from the active controllers", active_->name().c_str());
    reset();
  }

  return std::nullopt;
}

std::optional<ControllerSwitch> ModeManager::request_next(const ControllerSnapshot& controllers,
                                                          const rclcpp::Time& now)
{
  if (available_.empty()) {
    return std::nullopt;
  }

  // Where the cycle moves on from is the mode that was last asked for, so a second press
  // while a switch is still in flight skips a mode rather than asking for the same one
  // again.
  //
  // A mode whose controllers went away is no longer in the cycle, so the cycle restarts
  // rather than following a mode that is gone.
  auto* current = pending_ != nullptr ? pending_ : active_;
  const auto position = std::find(available_.begin(), available_.end(), current);
  const auto following = position == available_.end() ? available_.end() : std::next(position);
  auto* next = following == available_.end() ? available_.front() : *following;

  RCLCPP_INFO(logger_, "Switching to %s", next->name().c_str());
  auto plan = plan_switch(next->required_controllers(controllers), controllers.active(), protected_bases_);

  // The mode being left stops as soon as the operator asks for the switch, rather than
  // whenever the new controllers happen to come up.
  reset();

  pending_ = next;
  pending_since_ = now;
  return plan;
}

void ModeManager::reset()
{
  for (const auto& mode : modes_) {
    mode->reset();
  }
}

void ModeManager::set_focus(const std::string& component)
{
  for (const auto& mode : modes_) {
    mode->set_focus(component);
  }
}

void ModeManager::on_robot_changed()
{
  for (const auto& mode : modes_) {
    mode->on_robot_changed();
  }
}

bool ModeManager::satisfied(const BaseMode& mode, const ControllerSnapshot& controllers) const
{
  return all_active(mode.required_controllers(controllers), controllers);
}

BaseMode* ModeManager::infer(const ControllerSnapshot& controllers)
{
  BaseMode* best = nullptr;
  std::size_t best_size = 0;

  for (auto* mode : available_) {
    const auto required = mode->required_controllers(controllers);

    if (all_active(required, controllers) && required.size() > best_size) {
      best = mode;
      best_size = required.size();
    }
  }

  return best;
}

}  // namespace duatic_teleop_gamepad
