# Copyright 2026 Duatic AG
# Duatic Commercial License 1.0 (DCL-1)

from duatic_helpers.duatic_robots_helper import DuaticRobotsHelper
from duatic_helpers.duatic_jtc_helper import DuaticJTCHelper
from duatic_helpers.duatic_controller_helper import DuaticControllerHelper


class BaseController:
    """Base class for all controllers, providing logging and common methods."""

    def __init__(self, node, duatic_robots_helper: DuaticRobotsHelper, controller_helper=None):
        self.node = node
        self.log_printed = False  # Track whether the log was printed
        self.needed_capabilities = []
        self.joint_pos_offset_tolerance = 0.1

        self.duatic_robots_helper = duatic_robots_helper
        self.duatic_jtc_helper = DuaticJTCHelper(self.node)
        # Reuse the shared controller helper if provided. Each DuaticControllerHelper
        # spins up its own 10 Hz polling timer and service clients, so creating one per
        # controller multiplies executor load for no benefit.
        self.duatic_controller_helper = (
            controller_helper
            if controller_helper is not None
            else DuaticControllerHelper(self.node)
        )
        self.focused_component = "arm_left"

    def get_low_level_controllers(self):
        """Returns the name of the low-level controller this controller is based on."""
        return self.needed_low_level_controllers

    def get_focus(self):
        return self.focused_component

    def set_focus(self, focus_name):
        self.focused_component = focus_name

    def process_input(self, joy_msg):
        """Override this in child classes."""
        pass

    def reset(self):
        """Reset controller state when switching back to this controller."""
        self.log_printed = False  # Reset logging state

    def get_arm_from_topic(self, topic):
        """Extract component name from topic like '/joint_trajectory_controller_arm_left/joint_trajectory'"""
        if "arm_left" in topic:
            return "arm_left"
        elif "arm_right" in topic:
            return "arm_right"
        elif "hip" in topic:
            return "hip"
        return ""
