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

#include <string>
#include <vector>

namespace duatic_teleop_gamepad
{

/// The name and lifecycle state of one ros2_control controller.
struct ControllerState
{
  std::string name;
  std::string state;
};

/// The controller manager's state, reduced to what the teleop node acts on.
///
/// Only controllers matching one of the managed base names are visible through active()
/// and matching(). Everything else the robot runs -- broadcasters, gravity compensation,
/// force-torque publishers -- stays invisible here on purpose, because the switching logic
/// deactivates whatever it sees that the next mode does not need, and must never be able
/// to reach a controller it does not own.
class ControllerSnapshot
{
public:
  ControllerSnapshot() = default;

  /// @brief Reduce a controller_manager listing to the managed subset.
  /// @param controllers Every controller the manager reported.
  /// @param managed_bases Name prefixes of the controllers this node may switch.
  static ControllerSnapshot build(const std::vector<ControllerState>& controllers,
                                  const std::vector<std::string>& managed_bases);

  /// Whether the robot is frozen, i.e. the E-Stop is engaged.
  bool freeze_active() const;

  /// Managed controllers that are currently active, sorted.
  const std::vector<std::string>& active() const;

  bool is_active(const std::string& name) const;

  /// @brief Resolve requested controller names against what is actually loaded.
  /// @param names Either a managed base name, which resolves to every controller carrying
  ///   that prefix, or the exact name of one controller.
  /// @return The matching controller names, sorted and deduplicated.
  std::vector<std::string> matching(const std::vector<std::string>& names) const;

  /// Whether a listing has ever been reduced into this snapshot. A robot may legitimately
  /// run none of the managed controllers, so an empty active list is not the same thing as
  /// never having heard from the controller manager.
  bool has_data() const;

private:
  bool has_data_{ false };
  bool freeze_active_{ false };
  std::vector<std::string> active_;
  std::vector<ControllerState> managed_;
  std::vector<std::string> managed_bases_;
};

}  // namespace duatic_teleop_gamepad
