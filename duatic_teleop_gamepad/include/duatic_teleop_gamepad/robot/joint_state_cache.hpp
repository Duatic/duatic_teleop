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
#include <unordered_map>
#include <vector>

#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace duatic_teleop_gamepad
{

/// One view of the robot's joints, merged from every publisher on the topic.
///
/// /joint_states may carry several publishers that each describe a different subset of the
/// robot, arriving in any order and at different rates. Entries are merged rather than
/// replaced, so one partial message cannot hide another publisher's joints, and each entry
/// expires, so a publisher that stops does not leave its last values behind to be read as
/// current.
class JointStateCache
{
public:
  /// @param logger Where malformed messages are reported.
  /// @param stale_after How long an entry stays valid after its last update.
  JointStateCache(rclcpp::Logger logger, rclcpp::Duration stale_after);

  /// @brief Merge one message into the cache.
  /// @param msg Ignored if its position array does not match its name array. A message
  ///   carrying no positions is not an error: a publisher may legally send only velocity
  ///   or effort, and there is simply nothing here to merge.
  /// @param now Timestamp the merged entries are stamped with.
  void update(const sensor_msgs::msg::JointState& msg, const rclcpp::Time& now);

  /// @brief Drop entries that have not been updated within the staleness window.
  /// @return true if the set of known joint names changed.
  bool expire(const rclcpp::Time& now);

  /// Position of one joint, or nullopt when it is unknown.
  std::optional<double> position(const std::string& name) const;

  /// Whether every one of the given joints is known.
  bool has_all(const std::vector<std::string>& names) const;

private:
  struct Entry
  {
    double position;
    rclcpp::Time last_update;
  };

  rclcpp::Logger logger_;
  rclcpp::Clock throttle_clock_{ RCL_STEADY_TIME };
  rclcpp::Duration stale_after_;
  std::unordered_map<std::string, Entry> entries_;
};

}  // namespace duatic_teleop_gamepad
