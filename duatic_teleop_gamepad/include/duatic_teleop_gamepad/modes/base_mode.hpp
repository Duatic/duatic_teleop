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

#include <rclcpp/rclcpp.hpp>

#include "duatic_teleop_gamepad/gamepad_config.hpp"
#include "duatic_teleop_gamepad/input/gamepad_input.hpp"
#include "duatic_teleop_gamepad/robot/controller_snapshot.hpp"
#include "duatic_teleop_gamepad/robot/joint_state_cache.hpp"
#include "duatic_teleop_gamepad/robot/jtc_discovery.hpp"

namespace duatic_teleop_gamepad
{

/// What a mode is given to work with: the node it publishes through, the configuration,
/// and the robot state it reads.
///
/// All of it belongs to the node and outlives the modes, which is why these are references
/// rather than copies: a mode reading the config or the joint states is always reading what
/// the node has now.
struct TeleopContext
{
  rclcpp::Node& node;
  const GamepadConfig& config;
  const JointStateCache& joint_states;
  const JtcDiscovery& discovery;
};

/// One thing the gamepad can drive: the arms, the platform, or the robot's own compliance.
///
/// A mode owns the controllers that back it, the state it streams, and the publisher it
/// streams on, so adding one is a new file rather than another case in every switch. The
/// node never asks what kind of mode is active; it drives whichever one the ModeManager
/// hands it.
///
/// The two update hooks run at different rates on purpose. Reading the pad is bounded by
/// the /joy topic, while streaming to a controller is bounded by what that controller
/// wants, and the two are not the same number.
class BaseMode
{
public:
  virtual ~BaseMode() = default;

  /// Name used in logs and in the README's mode table.
  virtual std::string name() const = 0;

  /// @brief Controller name prefixes that back this mode.
  ///
  /// A controller is spawned only where there is something for it to drive, so a mode is
  /// on offer exactly where one of these is loaded.
  virtual const std::vector<std::string>& controller_bases() const = 0;

  /// @brief Controllers that have to be active for this mode to work.
  ///
  /// Defaults to every loaded controller carrying one of the base names. A mode that also
  /// depends on another one's controllers extends this.
  virtual std::vector<std::string> required_controllers(const ControllerSnapshot& controllers) const
  {
    return controllers.matching(controller_bases());
  }

  /// Apply one gamepad reading. Runs at the input rate while this mode is active.
  virtual void on_input([[maybe_unused]] const GamepadInput& input, [[maybe_unused]] double dt)
  {
  }

  /// Advance and publish whatever this mode streams. Runs at the publish rate while active.
  virtual void publish([[maybe_unused]] double dt)
  {
  }

  /// Return to a standstill, discarding any target the mode was working towards.
  virtual void reset()
  {
  }

  /// Follow the operator's choice of which component to act on.
  virtual void set_focus([[maybe_unused]] const std::string& component)
  {
  }

  /// Rebuild whatever the mode derives from the robot's controllers and topics.
  virtual void on_robot_changed()
  {
  }

  /// Rumble intensity the mode is asking for, zero for none.
  virtual double feedback() const
  {
    return 0.0;
  }
};

}  // namespace duatic_teleop_gamepad
