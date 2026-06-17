#!/usr/bin/env python3

# Copyright 2026 Duatic AG
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions, and
#    the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions, and
#    the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
#    promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import threading
import yaml
import argparse
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.experimental.events_executor import EventsExecutor

from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

from duatic_teleop_gamepad.controller_manager import ControllerManager
from duatic_teleop_gamepad.utils.gamepad_feedback import GamepadFeedback
from duatic_helpers.duatic_robots_helper import DuaticRobotsHelper


class GamepadInterface(Node):
    """Processes joystick input"""

    def __init__(self):
        super().__init__("gamepad_interface")

        self.latest_joy_msg = None
        self.joy_lock = threading.Lock()
        self.last_menu_button_state = 0
        self.move_command_active = False  # Track if move_home or move_sleep was executed
        self.deadman_active = False  # Track deadman switch state
        self.last_deadman_state = False  # Track previous deadman state for edge detection
        self.last_freeze_state = False  # Track previous freeze state for edge detection

        # Publishers
        self.move_home_pub = self.create_publisher(Bool, "move_home", 10)
        self.move_sleep_pub = self.create_publisher(Bool, "move_sleep", 10)

        # Subscribers
        self.create_subscription(Joy, "joy", self.joy_callback, 10)

        # Load gamepad mappings from YAML
        config_path = os.path.join(
            get_package_share_directory("duatic_teleop_gamepad"),
            "config",
            "gamepad_config.yaml",
        )
        with open(config_path) as file:
            config = yaml.safe_load(file)["gamepad_node"]["ros__parameters"]

        # Load configurations
        self.button_mapping = config["button_mapping"]
        self.axis_mapping = config["axis_mapping"]
        self.dpad_mapping = config.get("dpad_mapping", {})
        self.last_dpad_state = {"x": 0.0, "y": 0.0, "buttons": {}}
        self.get_logger().info(f"Loaded gamepad config: {self.button_mapping}, {self.axis_mapping}")

        self.duatic_robots_helper = DuaticRobotsHelper(self)
        self.duatic_robots_helper.wait_for_robot()
        self.controller_manager = ControllerManager(self, self.duatic_robots_helper)

        self.gamepad_feedback = GamepadFeedback(self)

        # Set the timing based on simulation or real hardware
        control_dt = self.duatic_robots_helper.get_dt()

        # The joy topic publishes at ~100 Hz, so running the full teleop pipeline faster
        # than that in Python only burns CPU without improving control. Cap the processing
        # rate. dt doubles as the integration step in the controllers, so capping it keeps
        # velocity scaling correct (fewer, larger steps -> same commanded velocity).
        min_period = 1.0 / self.MAX_TELEOP_RATE
        self.dt = max(control_dt, min_period)

        self.create_timer(self.dt, self.process_joy_input)
        self.get_logger().info(
            f"Gamepad Interface Initialized (control rate: {1.0 / self.dt:.0f} Hz)."
        )

    # Upper bound for the joystick processing/integration rate (Hz).
    MAX_TELEOP_RATE = 100.0

    def _button(self, msg, name, default=0):
        """Safely read a mapped button value, returning default if out of range.

        Guards against gamepads that report fewer buttons than the config maps,
        which would otherwise raise IndexError and kill the teleop callback.
        """
        idx = self.button_mapping.get(name)
        if idx is None or idx >= len(msg.buttons):
            return default
        return msg.buttons[idx]

    def joy_callback(self, msg: Joy):
        """Store latest joystick message."""
        with self.joy_lock:
            self.latest_joy_msg = msg

    def process_joy_input(self):
        """Process latest joystick input."""
        with self.joy_lock:
            msg = self.latest_joy_msg  # Get the latest stored joystick input

        if msg is None:
            return

        # Handle focus switching (independent of deadman)
        self._update_focus(msg)

        # Check deadman switch and track state changes
        current_deadman_state = self._button(msg, "dead_man_switch") == 1
        deadman_just_released = self.deadman_active and not current_deadman_state
        deadman_just_pressed = not self.deadman_active and current_deadman_state
        self.deadman_active = current_deadman_state

        if deadman_just_pressed:
            self._reset_current_controller()

        # 2. Handle Move Command Blocking (Safety Check)
        can_move = self.deadman_active and not self.controller_manager.is_freeze_active

        if not can_move:
            if self.move_command_active or deadman_just_released:
                self._stop_move_commands()
        else:
            # Allowed to move -> Process Home/Sleep commands
            move_home_pressed = self._button(msg, "move_home")
            move_sleep_pressed = self._button(msg, "move_sleep")

            if move_home_pressed:
                self.move_home_pub.publish(Bool(data=True))
                self.move_command_active = True
                return
            if move_sleep_pressed:
                self.move_sleep_pub.publish(Bool(data=True))
                self.move_command_active = True
                return

            if self.move_command_active:
                self._stop_move_commands()

        # Ensure switching happens only on button press (down event) and not while held down
        switch_pressed = self._button(msg, "switch_controller")
        if switch_pressed == 1 and self.last_menu_button_state == 0:
            self.controller_manager.switch_to_next_controller()
            # Reset current controller after switching
            self._reset_current_controller()

        # Wait until button is released (0) before allowing another switch
        # And don't execute anything else when the button is pressed
        self.last_menu_button_state = switch_pressed
        if self.last_menu_button_state:
            return

        freeze_active = self.controller_manager.is_freeze_active

        # The gripper commands motion, so gate it on the E-Stop/freeze as well (it already
        # checks the deadman internally).
        if not freeze_active:
            self.controller_manager.gripper_controller.process_input(msg)

        # Now get the current active controller from the controller manager:
        current_controller = self.controller_manager.get_current_controller()

        if current_controller is not None:
            freeze_entered = freeze_active and not self.last_freeze_state
            freeze_exited = not freeze_active and self.last_freeze_state

            if freeze_active:
                # Reset once on entering freeze (e.g. mecanum emits a zero command, markers
                # are cleared); nothing else to do while frozen, so don't reset every tick.
                if freeze_entered:
                    current_controller.reset()
            else:
                # Reset once on leaving freeze so the first command starts from the current
                # state. This covers the case where the deadman is already held when the
                # E-Stop clears, leaving no deadman edge to trigger the usual reset.
                if freeze_exited:
                    current_controller.reset()
                # Pass deadman state so controller knows if it can move
                current_controller.process_input(msg)

        self.last_freeze_state = freeze_active

    def _stop_move_commands(self):
        """Stop move_home and move_sleep commands and reset controller."""
        self.move_home_pub.publish(Bool(data=False))
        self.move_sleep_pub.publish(Bool(data=False))
        self.move_command_active = False
        # Reset current controller after move commands are stopped
        self._reset_current_controller()

    def _reset_current_controller(self):
        """Reset the current active controller if it exists."""
        current_controller = self.controller_manager.get_current_controller()
        if current_controller is not None:
            current_controller.reset()

    def _update_focus(self, joy_msg):
        """Update focus based on D-Pad input from config/gamepad_config.yaml."""
        if not self.dpad_mapping:
            return

        dpad_x = 0.0
        dpad_y = 0.0

        # Check Axes (Standard for Xbox/Hat-Switch D-Pads)
        axes_cfg = self.dpad_mapping.get("axes", {})
        ax_x = axes_cfg.get("x")
        ax_y = axes_cfg.get("y")

        if ax_x is not None and len(joy_msg.axes) > ax_x:
            if abs(joy_msg.axes[ax_x]) > 0.5:
                dpad_x = joy_msg.axes[ax_x]
        if ax_y is not None and len(joy_msg.axes) > ax_y:
            if abs(joy_msg.axes[ax_y]) > 0.5:
                dpad_y = joy_msg.axes[ax_y]

        # Check Buttons (Standard for PS4/PS5 D-Pads)
        btns_cfg = self.dpad_mapping.get("buttons", {})
        up_idx = btns_cfg.get("up")
        down_idx = btns_cfg.get("down")
        left_idx = btns_cfg.get("left")
        right_idx = btns_cfg.get("right")

        def btn_pressed(idx):
            if idx is None or len(joy_msg.buttons) <= idx:
                return False
            return joy_msg.buttons[idx] == 1 and self.last_dpad_state["buttons"].get(idx, 0) == 0

        if btn_pressed(up_idx):
            dpad_y = 1.0
        if btn_pressed(down_idx):
            dpad_y = -1.0
        if btn_pressed(left_idx):
            dpad_x = 1.0
        if btn_pressed(right_idx):
            dpad_x = -1.0

        # Determine target focus
        targets = self.dpad_mapping.get("focus_targets", {})
        new_focus = None

        if dpad_y > 0.5 and self.last_dpad_state["y"] <= 0.5:
            new_focus = targets.get("up")
        elif dpad_y < -0.5 and self.last_dpad_state["y"] >= -0.5:
            new_focus = targets.get("down")
        elif dpad_x > 0.5 and self.last_dpad_state["x"] <= 0.5:
            new_focus = targets.get("left")
        elif dpad_x < -0.5 and self.last_dpad_state["x"] >= -0.5:
            new_focus = targets.get("right")

        # Update last state
        self.last_dpad_state["x"] = dpad_x
        self.last_dpad_state["y"] = dpad_y
        for key, idx in btns_cfg.items():
            if idx is not None and len(joy_msg.buttons) > idx:
                self.last_dpad_state["buttons"][idx] = joy_msg.buttons[idx]

        if new_focus:
            current_controller = self.controller_manager.get_current_controller()
            if current_controller:
                # Check if component exists in the robot
                all_components = self.duatic_robots_helper.get_component_names(
                    "arm"
                ) + self.duatic_robots_helper.get_component_names("hip")

                # Special case for "platform" which might be handled by mecanum
                if new_focus in all_components or new_focus == "platform":
                    self.get_logger().info(f"Switching focus to: {new_focus}")
                    current_controller.set_focus(new_focus)
                    current_controller.reset()
                    self.controller_manager.trigger_llc_sync()
                    self.gamepad_feedback.send_feedback(intensity=0.5)


def main(args=None):

    parser = argparse.ArgumentParser()
    parsed_args, unknown = parser.parse_known_args()  # ← ignore ROS args

    rclpy.init(args=unknown)  # ← pass remaining args to rclpy
    node = GamepadInterface()

    # Event-driven executor: avoids the SingleThreadedExecutor rebuilding the full
    # wait set in Python on every wakeup, which otherwise dominates CPU when high-rate
    # topics (e.g. /joint_states at ~1 kHz) keep waking the executor.
    executor = EventsExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
