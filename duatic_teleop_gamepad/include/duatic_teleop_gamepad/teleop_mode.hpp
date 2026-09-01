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

#include <optional>
#include <string>
#include <vector>

#include "duatic_teleop_gamepad/controller_snapshot.hpp"

namespace duatic_teleop_gamepad
{

/// What the gamepad is currently driving. The order is the order the mode button cycles in.
enum class TeleopMode
{
  Freedrive,
  Jog,
  Drive,
};

std::string to_string(TeleopMode mode);

/// @brief Modes this robot can offer, given the controllers that are loaded.
///
/// A spawned controller is the whole test: one exists only where there is something for it
/// to drive, so a mode appears as soon as the controller backing it does.
std::vector<TeleopMode> available_modes(const ControllerSnapshot& controllers);

/// @brief Controllers that have to be active for a mode to work.
/// @return Resolved controller names, empty if none of them are loaded.
std::vector<std::string> required_controllers(TeleopMode mode, const ControllerSnapshot& controllers);

/// @brief Which mode the currently active controllers amount to.
///
/// Used to adopt whatever the robot was already doing at startup, and to notice when
/// something else has switched controllers underneath the node. Where several modes fit,
/// the most specific one wins, so driving with the arms held is not mistaken for jogging.
/// @return The mode, or nullopt if the active controllers match none of them.
std::optional<TeleopMode> infer_mode(const std::vector<TeleopMode>& available, const ControllerSnapshot& controllers);

/// The next mode in the cycle, wrapping around. Returns nullopt if nothing is available.
std::optional<TeleopMode> next_mode(const std::vector<TeleopMode>& available, std::optional<TeleopMode> current);

struct ControllerSwitch
{
  std::vector<std::string> activate;
  std::vector<std::string> deactivate;
};

/// @brief Work out the switch that leaves only what a mode needs running.
/// @param needed Controllers the target mode requires.
/// @param active Controllers currently active.
/// @param protected_bases Name prefixes never to deactivate, for controllers that lose
///   state when stopped, such as the drive controller and its odometry.
ControllerSwitch plan_switch(const std::vector<std::string>& needed, const std::vector<std::string>& active,
                             const std::vector<std::string>& protected_bases);

}  // namespace duatic_teleop_gamepad
