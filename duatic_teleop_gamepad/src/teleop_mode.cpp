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

#include "duatic_teleop_gamepad/teleop_mode.hpp"

#include <algorithm>

namespace duatic_teleop_gamepad
{

namespace
{

const std::vector<std::string>& freedrive_bases()
{
  static const std::vector<std::string> bases = { "freedrive_controller" };
  return bases;
}

const std::vector<std::string>& jog_bases()
{
  static const std::vector<std::string> bases = { "joint_trajectory_controller" };
  return bases;
}

const std::vector<std::string>& drive_bases()
{
  static const std::vector<std::string> bases = { "mecanum_drive_controller", "platform_velocity_controller" };
  return bases;
}

const std::vector<TeleopMode>& all_modes()
{
  static const std::vector<TeleopMode> modes = { TeleopMode::Freedrive, TeleopMode::Jog, TeleopMode::Drive };
  return modes;
}

bool has_arms(const RobotModel& model)
{
  return !model.component_names(ComponentType::Arm).empty();
}

bool has_platform(const RobotModel& model)
{
  return !model.component_names(ComponentType::Platform).empty();
}

void append_unique(std::vector<std::string>& into, const std::vector<std::string>& from)
{
  for (const auto& value : from) {
    if (std::find(into.begin(), into.end(), value) == into.end()) {
      into.push_back(value);
    }
  }
}

}  // namespace

std::string to_string(TeleopMode mode)
{
  switch (mode) {
    case TeleopMode::Freedrive:
      return "freedrive";
    case TeleopMode::Jog:
      return "jog";
    case TeleopMode::Drive:
      break;
  }
  return "drive";
}

std::vector<TeleopMode> available_modes(const RobotModel& model, const ControllerSnapshot& controllers)
{
  std::vector<TeleopMode> available;

  for (const auto mode : all_modes()) {
    // The robot has to be shaped for the mode and carry a controller that implements it.
    // Either half missing means the mode cannot be offered, however the other half looks.
    const bool shaped = mode == TeleopMode::Drive ? has_platform(model) : has_arms(model);
    const auto& bases = mode == TeleopMode::Freedrive ? freedrive_bases()
                        : mode == TeleopMode::Jog     ? jog_bases()
                                                      : drive_bases();

    if (shaped && !controllers.matching(bases).empty()) {
      available.push_back(mode);
    }
  }

  return available;
}

std::vector<std::string> required_controllers(TeleopMode mode, const ControllerSnapshot& controllers)
{
  switch (mode) {
    case TeleopMode::Freedrive:
      return controllers.matching(freedrive_bases());
    case TeleopMode::Jog:
      return controllers.matching(jog_bases());
    case TeleopMode::Drive:
      break;
  }

  // Driving keeps the trajectory controllers active so the arms hold their pose instead of
  // going slack while the base moves.
  auto required = controllers.matching(drive_bases());
  append_unique(required, controllers.matching(jog_bases()));
  std::sort(required.begin(), required.end());
  return required;
}

std::optional<TeleopMode> infer_mode(const std::vector<TeleopMode>& available, const ControllerSnapshot& controllers)
{
  std::optional<TeleopMode> best;
  std::size_t best_size = 0;

  for (const auto mode : available) {
    const auto required = required_controllers(mode, controllers);
    if (required.empty()) {
      continue;
    }

    const bool satisfied = std::all_of(required.begin(), required.end(),
                                       [&controllers](const std::string& name) { return controllers.is_active(name); });

    // Driving requires everything jogging does and then the base as well, so comparing how
    // much each mode demands is what stops "driving with the arms held" reading as jogging.
    if (satisfied && required.size() > best_size) {
      best = mode;
      best_size = required.size();
    }
  }

  return best;
}

std::optional<TeleopMode> next_mode(const std::vector<TeleopMode>& available, std::optional<TeleopMode> current)
{
  if (available.empty()) {
    return std::nullopt;
  }

  const auto position = current ? std::find(available.begin(), available.end(), *current) : available.end();
  if (position == available.end()) {
    return available.front();
  }

  const auto following = std::next(position);
  return following == available.end() ? available.front() : *following;
}

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

}  // namespace duatic_teleop_gamepad
