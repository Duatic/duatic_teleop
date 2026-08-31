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

#include "duatic_teleop_gamepad/controller_snapshot.hpp"

#include <algorithm>

namespace duatic_teleop_gamepad
{

namespace
{

constexpr const char* kActive = "active";
constexpr const char* kFreezePrefix = "freeze_controller";
constexpr const char* kGlobalFreeze = "freeze_controller";

bool starts_with(const std::string& value, const std::string& prefix)
{
  return value.rfind(prefix, 0) == 0;
}

/// @brief Decide whether the E-Stop is engaged.
///
/// A robot carries one global freeze controller plus one per component, and only the
/// global one is the E-Stop; the per-component ones are switched as part of normal mode
/// changes. So an exact "freeze_controller" answers the question by itself when it exists.
/// Without one, any active freeze is treated as the E-Stop, because for a stop signal the
/// safe reading of an ambiguous configuration is the engaged one.
bool determine_freeze(const std::vector<ControllerState>& controllers)
{
  bool any_freeze_active = false;

  for (const auto& controller : controllers) {
    if (!starts_with(controller.name, kFreezePrefix)) {
      continue;
    }

    if (controller.name == kGlobalFreeze) {
      return controller.state == kActive;
    }

    any_freeze_active = any_freeze_active || controller.state == kActive;
  }

  return any_freeze_active;
}

}  // namespace

ControllerSnapshot ControllerSnapshot::build(const std::vector<ControllerState>& controllers,
                                             const std::vector<std::string>& managed_bases)
{
  ControllerSnapshot snapshot;
  snapshot.has_data_ = true;
  snapshot.managed_bases_ = managed_bases;
  snapshot.freeze_active_ = determine_freeze(controllers);

  for (const auto& controller : controllers) {
    const bool managed =
        std::any_of(managed_bases.begin(), managed_bases.end(),
                    [&controller](const std::string& base) { return starts_with(controller.name, base); });
    if (!managed) {
      continue;
    }

    snapshot.managed_.push_back(controller);

    if (controller.state == kActive) {
      snapshot.active_.push_back(controller.name);
    }
  }

  std::sort(snapshot.managed_.begin(), snapshot.managed_.end(),
            [](const ControllerState& a, const ControllerState& b) { return a.name < b.name; });
  std::sort(snapshot.active_.begin(), snapshot.active_.end());

  return snapshot;
}

bool ControllerSnapshot::freeze_active() const
{
  return freeze_active_;
}

const std::vector<std::string>& ControllerSnapshot::active() const
{
  return active_;
}

bool ControllerSnapshot::is_active(const std::string& name) const
{
  return std::find(active_.begin(), active_.end(), name) != active_.end();
}

std::vector<std::string> ControllerSnapshot::matching(const std::vector<std::string>& names) const
{
  std::vector<std::string> matched;

  for (const auto& controller : managed_) {
    for (const auto& requested : names) {
      // A base name stands for every controller carrying it as a prefix, so asking for
      // "joint_trajectory_controller" reaches all of the per-component instances. Any
      // other requested name has to identify one controller exactly, so that asking for
      // a single instance cannot drag its siblings along.
      const bool is_base =
          std::find(managed_bases_.begin(), managed_bases_.end(), requested) != managed_bases_.end();
      const bool matches = is_base ? starts_with(controller.name, requested) : controller.name == requested;

      if (matches) {
        matched.push_back(controller.name);
        break;
      }
    }
  }

  std::sort(matched.begin(), matched.end());
  matched.erase(std::unique(matched.begin(), matched.end()), matched.end());
  return matched;
}

bool ControllerSnapshot::has_data() const
{
  return has_data_;
}

}  // namespace duatic_teleop_gamepad
