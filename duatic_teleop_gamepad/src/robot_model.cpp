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

#include "duatic_teleop_gamepad/robot_model.hpp"

#include <algorithm>
#include <iterator>
#include <map>
#include <optional>

#include <rcpputils/split.hpp>

namespace duatic_teleop_gamepad
{

namespace
{

struct TypeIndicators
{
  ComponentType type;
  std::vector<std::string> indicators;
};

/// The keyword each component type is recognised by.
///
/// Only the parts the gamepad actually drives are named. Anything else -- a head, a
/// gripper's own finger joints -- classifies as misc, because naming it would create a
/// component nothing focuses, jogs or grips. Grippers are found by their command topic,
/// which names the arm they belong to, not by their joints.
const std::vector<TypeIndicators>& indicator_table()
{
  static const std::vector<TypeIndicators> table = {
    { ComponentType::Arm, { "shoulder", "elbow", "forearm", "wrist" } },
    { ComponentType::Hip, { "hip" } },
    { ComponentType::Platform, { "wheel" } },
  };
  return table;
}

/// The component type whose keyword appears in the given name, if any.
std::optional<ComponentType> type_of(const std::string& name)
{
  for (const auto& entry : indicator_table()) {
    for (const auto& indicator : entry.indicators) {
      if (name.find(indicator) != std::string::npos) {
        return entry.type;
      }
    }
  }

  return std::nullopt;
}

}  // namespace

std::string to_string(ComponentType type)
{
  switch (type) {
    case ComponentType::Arm:
      return "arm";
    case ComponentType::Hip:
      return "hip";
    case ComponentType::Platform:
      return "platform";
    case ComponentType::Misc:
      break;
  }
  return "misc";
}

std::string component_from_topic(const std::string& topic, const std::string& suffix)
{
  const auto segments = rcpputils::split(topic, '/', true);
  const auto expected = rcpputils::split(suffix, '/', true);

  if (segments.size() != expected.size() + 1) {
    return {};
  }

  return std::equal(expected.begin(), expected.end(), std::next(segments.begin())) ? segments.front() : std::string{};
}

std::pair<std::string, ComponentType> RobotModel::classify(const std::string& joint_name)
{
  const auto slash = joint_name.find('/');

  // Namespaced joints such as "arm_left/shoulder_lift" name their component directly.
  if (slash != std::string::npos) {
    const std::string prefix = joint_name.substr(0, slash);
    const std::string suffix = joint_name.substr(slash + 1);

    if (prefix.starts_with("arm_") || prefix.starts_with("hand_")) {
      return { prefix, ComponentType::Arm };
    }

    if (const auto type = type_of(suffix)) {
      return { prefix, *type };
    }
  }

  // Flat joint names, as a single-arm robot publishes them.
  if (const auto type = type_of(joint_name)) {
    // An arm on a flat robot gets no component name, so the topic derived from it is the
    // un-suffixed "/joint_trajectory_controller/joint_trajectory".
    return { *type == ComponentType::Arm ? std::string{} : to_string(*type), *type };
  }

  return { "misc", ComponentType::Misc };
}

bool RobotModel::rebuild(const std::vector<std::string>& joint_names)
{
  // Ordered, so the rebuilt list can be compared against the previous one directly and
  // the component order does not wander between rebuilds.
  std::map<std::string, Component> by_name;

  for (const auto& joint_name : joint_names) {
    const auto classified = classify(joint_name);
    auto& component = by_name[classified.first];

    if (component.joints.empty()) {
      component.name = classified.first;
      component.type = classified.second;
    }

    component.joints.push_back(joint_name);
  }

  std::vector<Component> rebuilt;
  rebuilt.reserve(by_name.size());

  for (auto& entry : by_name) {
    std::sort(entry.second.joints.begin(), entry.second.joints.end());
    rebuilt.push_back(std::move(entry.second));
  }

  const bool changed =
      rebuilt.size() != components_.size() ||
      !std::equal(rebuilt.begin(), rebuilt.end(), components_.begin(), [](const Component& a, const Component& b) {
        return a.name == b.name && a.type == b.type && a.joints == b.joints;
      });

  components_ = std::move(rebuilt);
  return changed;
}

std::vector<std::string> RobotModel::component_names(ComponentType type) const
{
  std::vector<std::string> names;

  for (const auto& component : components_) {
    if (component.type == type) {
      names.push_back(component.name);
    }
  }

  return names;
}

bool RobotModel::has_component(const std::string& name) const
{
  return std::any_of(components_.begin(), components_.end(),
                     [&name](const Component& component) { return component.name == name; });
}

}  // namespace duatic_teleop_gamepad
