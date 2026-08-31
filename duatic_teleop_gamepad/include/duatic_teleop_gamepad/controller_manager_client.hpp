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

#include <chrono>
#include <string>
#include <vector>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>

#include "duatic_teleop_gamepad/controller_snapshot.hpp"

namespace duatic_teleop_gamepad
{

/// Keeps an up-to-date view of the controller manager, and switches controllers.
///
/// Every service call is asynchronous. Nothing here blocks the executor: the poll skips a
/// round when the service is not up rather than waiting for it, and skips a round when the
/// previous request has not come back, so a slow controller manager cannot make requests
/// pile up behind each other.
class ControllerManagerClient
{
public:
  /// @param node Node the client and timer are created on.
  /// @param managed_bases Name prefixes of the controllers this node may switch.
  /// @param poll_period How often the controller listing is refreshed.
  ControllerManagerClient(rclcpp::Node& node, std::vector<std::string> managed_bases,
                          std::chrono::nanoseconds poll_period);

  const ControllerSnapshot& snapshot() const;

  /// @brief Activate and deactivate controllers in one switch.
  ///
  /// Names already in the requested state are dropped, and the call is skipped entirely if
  /// nothing is left to do, so this is safe to call speculatively.
  void switch_controllers(const std::vector<std::string>& activate, const std::vector<std::string>& deactivate);

private:
  using ListControllers = controller_manager_msgs::srv::ListControllers;
  using SwitchController = controller_manager_msgs::srv::SwitchController;

  void poll();

  rclcpp::Node& node_;
  std::vector<std::string> managed_bases_;

  rclcpp::Client<ListControllers>::SharedPtr list_client_;
  rclcpp::Client<SwitchController>::SharedPtr switch_client_;
  rclcpp::TimerBase::SharedPtr poll_timer_;

  ControllerSnapshot snapshot_;
  bool list_in_flight_{ false };
  rclcpp::Time list_sent_at_;
  rclcpp::Duration list_timeout_;
};

}  // namespace duatic_teleop_gamepad
