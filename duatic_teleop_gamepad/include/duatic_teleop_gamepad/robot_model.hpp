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
#include <utility>
#include <vector>

namespace duatic_teleop_gamepad
{

enum class ComponentType
{
  Arm,
  Hip,
  Platform,
  Misc,
};

/// The name a component type is known by in the gamepad config and in topic names.
std::string to_string(ComponentType type);

struct Component
{
  std::string name;
  ComponentType type;
  std::vector<std::string> joints;
};

/// @brief The component a controller topic belongs to.
/// @param topic Full topic name, expected to be "<component>/<suffix>".
/// @param suffix The controller-and-message part that follows the component.
/// @return The component name, or empty if the topic does not have that shape.
///
/// Read from the leading segment rather than by searching backwards from the suffix, so a
/// component whose name happens to end in another one's cannot claim its topic.
std::string component_from_topic(const std::string& topic, const std::string& suffix);

/// The robot's components, derived from the joint names currently being published.
///
/// Derived afresh on every rebuild rather than latched the first time joints are seen. A
/// component whose publisher joins late is therefore picked up, and one that goes away is
/// dropped, which is what lets the node run against a /joint_states carrying several
/// publishers that come and go in an order nobody controls.
class RobotModel
{
public:
  /// Rebuild the component list from the given joint names.
  void rebuild(const std::vector<std::string>& joint_names);

  /// Names of the components of one type, in sorted order.
  std::vector<std::string> component_names(ComponentType type) const;

  /// Whether a component of this name exists, whatever its type.
  bool has_component(const std::string& name) const;

  /// @brief Work out which component a joint belongs to, and of what type.
  /// @return The component name paired with its type. A joint that matches nothing is
  ///   reported as "misc", and a flat single-arm joint gets an empty component name to
  ///   match the un-suffixed controller topic such a robot exposes.
  static std::pair<std::string, ComponentType> classify(const std::string& joint_name);

private:
  std::vector<Component> components_;
};

}  // namespace duatic_teleop_gamepad
