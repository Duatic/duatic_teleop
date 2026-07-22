# Copyright 2026 Duatic AG
# Duatic Commercial License 1.0 (DCL-1)

from duatic_teleop_gamepad.controllers.base_controller import BaseController


class FreedriveController(BaseController):
    """Handles freedrive mode."""

    def __init__(self, node, duatic_robots_helper, controller_helper=None):
        super().__init__(node, duatic_robots_helper, controller_helper)

        self.needed_capabilities = ["freedrive"]
        self.needed_low_level_controllers = ["freedrive_controller"]

    def process_input(self, joy_msg):
        pass
