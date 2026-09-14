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

#include "duatic_teleop_gamepad/robot/controller_manager_client.hpp"

#include <algorithm>
#include <utility>

namespace duatic_teleop_gamepad
{

ControllerManagerClient::ControllerManagerClient(rclcpp::Node& node, std::vector<std::string> managed_bases,
                                                 std::chrono::nanoseconds poll_period)
  : node_(node)
  , managed_bases_(std::move(managed_bases))
  , list_sent_at_(node.now())
  , list_timeout_(rclcpp::Duration(poll_period * 5))
{
  list_client_ = node_.create_client<ListControllers>("controller_manager/list_controllers");
  switch_client_ = node_.create_client<SwitchController>("controller_manager/switch_controller");
  poll_timer_ = node_.create_wall_timer(poll_period, [this]() { poll(); });
}

const ControllerSnapshot& ControllerManagerClient::snapshot() const
{
  return snapshot_;
}

void ControllerManagerClient::poll()
{
  // service_is_ready() only reads the graph cache, unlike wait_for_service(), which would
  // stall the executor thread this timer runs on.
  if (!list_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 10000,
                         "controller_manager/list_controllers is not available yet");
    return;
  }

  if (list_in_flight_) {
    if (node_.now() - list_sent_at_ < list_timeout_) {
      return;
    }

    // If the controller manager goes away mid-request the future is never completed, and
    // an in-flight request that never lands would stop the polling for the lifetime of the
    // node, leaving the E-Stop state frozen at whatever was last read.
    list_client_->prune_pending_requests();
    RCLCPP_WARN(node_.get_logger(), "Timed out waiting for a controller listing, retrying");
  }

  list_in_flight_ = true;
  list_sent_at_ = node_.now();
  list_client_->async_send_request(std::make_shared<ListControllers::Request>(),
                                   [this](rclcpp::Client<ListControllers>::SharedFuture future) {
                                     list_in_flight_ = false;

                                     std::vector<ControllerState> controllers;
                                     for (const auto& controller : future.get()->controller) {
                                       controllers.push_back({ controller.name, controller.state });
                                     }

                                     snapshot_ = ControllerSnapshot::build(controllers, managed_bases_);
                                   });
}

void ControllerManagerClient::switch_controllers(const std::vector<std::string>& activate,
                                                 const std::vector<std::string>& deactivate)
{
  if (activate.empty() && deactivate.empty()) {
    return;
  }

  auto request = std::make_shared<SwitchController::Request>();
  request->activate_controllers = activate;
  request->deactivate_controllers = deactivate;

  if (!switch_client_->service_is_ready()) {
    RCLCPP_WARN(node_.get_logger(), "Cannot switch controllers: controller_manager/switch_controller is not available");
    return;
  }

  request->strictness = SwitchController::Request::BEST_EFFORT;

  const auto activated = request->activate_controllers;
  const auto deactivated = request->deactivate_controllers;

  switch_client_->async_send_request(
      request, [this, activated, deactivated](rclcpp::Client<SwitchController>::SharedFuture future) {
        if (!future.get()->ok) {
          RCLCPP_ERROR(node_.get_logger(), "Controller switch was rejected by the controller manager");
          return;
        }

        RCLCPP_INFO(node_.get_logger(), "Switched controllers: activated %zu, deactivated %zu", activated.size(),
                    deactivated.size());

        // The snapshot is stale the moment a switch succeeds, and the next mode decision
        // reads it, so do not wait for the next poll to catch up.
        poll();
      });
}

}  // namespace duatic_teleop_gamepad
