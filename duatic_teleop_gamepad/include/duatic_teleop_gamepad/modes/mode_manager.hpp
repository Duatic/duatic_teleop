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

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>

#include "duatic_teleop_gamepad/modes/base_mode.hpp"
#include "duatic_teleop_gamepad/robot/controller_snapshot.hpp"

namespace duatic_teleop_gamepad
{

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

/// Owns the teleop modes and decides which one is active.
///
/// Which modes exist is a property of the controllers that are loaded, and which one is
/// active can change without the operator asking for it, so the whole of that decision
/// lives here rather than spread across the node's timers.
class ModeManager
{
public:
  /// @param modes The modes on offer, in the order the mode button cycles through them.
  /// @param protected_bases Name prefixes never to deactivate on a switch.
  /// @param switch_timeout How long a requested mode waits for its controllers.
  ModeManager(std::vector<std::unique_ptr<BaseMode>> modes, rclcpp::Logger logger,
              std::vector<std::string> protected_bases, rclcpp::Duration switch_timeout);

  /// The mode the gamepad is driving, or nullptr when the robot is doing none of them.
  BaseMode* active();

  /// Modes the loaded controllers can back, in cycle order.
  const std::vector<BaseMode*>& available() const;

  /// @brief Bring the active mode in line with the controllers that are actually running.
  ///
  /// Runs at the discovery rate. This is what adopts whatever the robot was already doing
  /// at startup, and what notices when something else switches controllers underneath.
  ///
  /// @return The switch to request of the controller manager when a mode change has failed,
  ///   which engages the freeze controllers. Nothing in the ordinary case.
  std::optional<ControllerSwitch> refresh(const ControllerSnapshot& controllers, const rclcpp::Time& now);

  /// @brief Ask for the next mode in the cycle.
  /// @return The switch to request of the controller manager, or nothing when there is no
  ///   other mode to move to.
  std::optional<ControllerSwitch> request_next(const ControllerSnapshot& controllers, const rclcpp::Time& now);

  /// @brief Return every mode to a standstill, not just the active one.
  ///
  /// Called on every edge that ends motion: the deadman, the E-Stop, a change of mode or
  /// focus. Resetting all of them is what keeps a mode from resuming against a stale
  /// target when it is next entered.
  void reset();

  void set_focus(const std::string& component);
  void on_robot_changed();

private:
  /// Whether every controller the mode needs is active, and it needs any at all.
  bool satisfied(const BaseMode& mode, const ControllerSnapshot& controllers) const;

  /// @brief The mode the active controllers amount to, or nullptr for none of them.
  ///
  /// Where several fit, the most demanding one wins: driving requires everything jogging
  /// does and then the base as well, so counting controllers is what stops "driving with
  /// the arms held" reading as jogging.
  BaseMode* infer(const ControllerSnapshot& controllers);

  std::vector<std::unique_ptr<BaseMode>> modes_;
  rclcpp::Logger logger_;
  std::vector<std::string> protected_bases_;
  rclcpp::Duration switch_timeout_;

  std::vector<BaseMode*> available_;
  BaseMode* active_{ nullptr };
  BaseMode* pending_{ nullptr };
  rclcpp::Time pending_since_;
};

}  // namespace duatic_teleop_gamepad
