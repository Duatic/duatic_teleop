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

#include "duatic_teleop_gamepad/joint_state_cache.hpp"

#include <algorithm>

#include <rclcpp/logging.hpp>

namespace duatic_teleop_gamepad
{

JointStateCache::JointStateCache(rclcpp::Logger logger, rclcpp::Duration stale_after)
  : logger_(std::move(logger)), stale_after_(stale_after)
{
}

bool JointStateCache::update(const sensor_msgs::msg::JointState& msg, const rclcpp::Time& now)
{
  if (msg.position.empty()) {
    return false;
  }

  if (msg.position.size() != msg.name.size()) {
    RCLCPP_WARN_THROTTLE(logger_, throttle_clock_, 10000,
                         "Ignoring a JointState with %zu names and %zu positions: the arrays must be the same length",
                         msg.name.size(), msg.position.size());
    return false;
  }

  bool names_changed = false;
  for (std::size_t i = 0; i < msg.name.size(); ++i) {
    const auto result = entries_.insert_or_assign(msg.name[i], Entry{ msg.position[i], now });
    names_changed = names_changed || result.second;
  }

  return names_changed;
}

bool JointStateCache::expire(const rclcpp::Time& now)
{
  bool names_changed = false;

  for (auto it = entries_.begin(); it != entries_.end();) {
    if (now - it->second.last_update > stale_after_) {
      it = entries_.erase(it);
      names_changed = true;
    } else {
      ++it;
    }
  }

  return names_changed;
}

std::optional<double> JointStateCache::position(const std::string& name) const
{
  const auto it = entries_.find(name);
  if (it == entries_.end()) {
    return std::nullopt;
  }

  return it->second.position;
}

bool JointStateCache::has_all(const std::vector<std::string>& names) const
{
  return std::all_of(names.begin(), names.end(),
                     [this](const std::string& name) { return entries_.count(name) > 0; });
}

std::vector<std::string> JointStateCache::joint_names() const
{
  std::vector<std::string> names;
  names.reserve(entries_.size());

  for (const auto& entry : entries_) {
    names.push_back(entry.first);
  }

  std::sort(names.begin(), names.end());
  return names;
}

bool JointStateCache::empty() const
{
  return entries_.empty();
}

}  // namespace duatic_teleop_gamepad
