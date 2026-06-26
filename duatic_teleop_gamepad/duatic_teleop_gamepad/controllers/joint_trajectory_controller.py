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

from duatic_teleop_gamepad.controllers.base_controller import BaseController
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointTrajectoryController(BaseController):
    """Handles joint trajectory control using the gamepad"""

    # Joint velocity (rad/s) commanded at full stick deflection. This is the teleop
    # speed knob: the stick value (-1..1) is scaled by this to get the target velocity.
    MAX_JOINT_VELOCITY = 1.0

    # Slew-rate limit on the commanded joint velocity (rad/s^2). Bounds how fast the
    # commanded velocity may change, giving smooth start/stop without differentiating
    # the noisy joystick signal.
    MAX_JOINT_ACCEL = 5.0

    # How far ahead (in publish cycles) each streamed trajectory point is placed. >= 2
    # keeps a valid future target so the 1 kHz controller never runs out of trajectory
    # between our updates, even if one command is late or dropped.
    TRAJECTORY_LOOKAHEAD_CYCLES = 2.0

    def __init__(self, node, duatic_robots_helper, controller_helper=None):
        super().__init__(node, duatic_robots_helper, controller_helper)
        self.needed_capabilities = ["manipulation"]
        self.node.get_logger().info("Initializing joint trajectory controller.")

        # Discover all relevant components
        self.arms = self.duatic_robots_helper.get_component_names("arm")
        self.hips = self.duatic_robots_helper.get_component_names("hip")
        self.all_components = self.arms + self.hips

        found_topics = self.duatic_jtc_helper.find_topics_for_controller(
            "joint_trajectory_controller", "joint_trajectory", self.all_components
        )
        response = self.duatic_jtc_helper.process_topics_and_extract_joint_names(found_topics)
        self.topic_to_joint_names = response[0]
        self.topic_to_commanded_positions = response[1]
        for topic, joint_names in self.topic_to_joint_names.items():
            self.topic_to_commanded_positions[topic] = [0.0] * len(joint_names)

        # Commanded joint velocities (rad/s), slew-limited and published alongside the
        # positions so the downstream controller interpolates with continuous velocity.
        self.topic_to_commanded_velocities = {
            topic: [0.0] * len(joint_names)
            for topic, joint_names in self.topic_to_joint_names.items()
        }

        # Focus management
        self.focused_component = (
            "arm_left"
            if "arm_left" in self.all_components
            else (self.all_components[0] if self.all_components else "")
        )

        # Create publishers for each joint trajectory topic
        self.joint_trajectory_publishers = {}
        for topic in self.topic_to_joint_names.keys():
            self.joint_trajectory_publishers[topic] = self.node.create_publisher(
                JointTrajectory, topic, 10
            )

        self.prefix_to_joints = {}
        self.is_joystick_idle = True

        # Live-tunable on hardware via `ros2 param set /gamepad_interface <name> <value>`.
        # Defaults come from the class constants above.
        self.node.declare_parameter("jtc_max_joint_velocity", self.MAX_JOINT_VELOCITY)
        self.node.declare_parameter("jtc_max_joint_accel", self.MAX_JOINT_ACCEL)
        self.node.declare_parameter("jtc_lookahead_cycles", self.TRAJECTORY_LOOKAHEAD_CYCLES)

        # Dominant axis tracking
        self.dominant_axis_threshold = 0.6
        self.active_axes = {
            "left_joystick": {"x": False, "y": False},
            "right_joystick": {"x": False, "y": False},
        }

        self.node.get_logger().info("Joint Trajectory Controller initialized.")

    def get_low_level_controllers(self):
        """Returns all discovered JTC controller names to keep them active simultaneously."""
        controllers = []
        for topic in self.topic_to_joint_names.keys():
            # Extract the controller name part from the topic
            # e.g., '/joint_trajectory_controller_arm_left/joint_trajectory' -> 'joint_trajectory_controller_arm_left'
            segments = topic.strip("/").split("/")
            if len(segments) >= 2:
                controllers.append(segments[-2])

        if not controllers:
            return ["joint_trajectory_controller"]

        return controllers

    def reset(self):
        """Reset commanded positions to current joint states for all topics."""
        joint_states = self.duatic_robots_helper.get_joint_states()

        for topic, joint_names in self.topic_to_joint_names.items():
            self.topic_to_commanded_positions[topic] = [
                joint_states.get(joint, 0.0) for joint in joint_names
            ]
            self.topic_to_commanded_velocities[topic] = [0.0] * len(joint_names)

    def process_input(self, msg):
        """Processes joystick input, integrates over dt, and clamps the commanded positions."""
        super().process_input(msg)

        any_axis_active = False
        deadzone = 0.1
        max_velocity = self.node.get_parameter("jtc_max_joint_velocity").value
        max_accel = self.node.get_parameter("jtc_max_joint_accel").value

        # Determine which axes are currently dominant
        left_x = (
            msg.axes[self.node.axis_mapping["left_joystick"]["x"]]
            if len(msg.axes) > self.node.axis_mapping["left_joystick"]["x"]
            else 0.0
        )
        left_y = (
            msg.axes[self.node.axis_mapping["left_joystick"]["y"]]
            if len(msg.axes) > self.node.axis_mapping["left_joystick"]["y"]
            else 0.0
        )
        right_x = (
            msg.axes[self.node.axis_mapping["right_joystick"]["x"]]
            if len(msg.axes) > self.node.axis_mapping["right_joystick"]["x"]
            else 0.0
        )
        right_y = (
            msg.axes[self.node.axis_mapping["right_joystick"]["y"]]
            if len(msg.axes) > self.node.axis_mapping["right_joystick"]["y"]
            else 0.0
        )

        self._update_dominant_axes(left_x, left_y, right_x, right_y, deadzone)

        # Process each topic
        for topic, joint_names in self.topic_to_joint_names.items():
            arm_name = self.get_arm_from_topic(topic)

            if arm_name != self.focused_component:
                continue

            # Safety: Only move if deadman is active
            if not self.node.deadman_active:
                continue

            commanded_positions = self.topic_to_commanded_positions[topic]
            commanded_velocities = self.topic_to_commanded_velocities[topic]
            for i, joint_name in enumerate(joint_names):
                axis_val = 0.0
                effective_deadzone = deadzone

                # Joint Mapping
                match i:
                    case 0:
                        axis_val = left_x
                        if (
                            self.active_axes["left_joystick"]["y"]
                            and not self.active_axes["left_joystick"]["x"]
                        ):
                            effective_deadzone = self.dominant_axis_threshold
                    case 1:
                        axis_val = left_y
                        if (
                            self.active_axes["left_joystick"]["x"]
                            and not self.active_axes["left_joystick"]["y"]
                        ):
                            effective_deadzone = self.dominant_axis_threshold
                    case 2:
                        axis_val = right_y
                        if (
                            self.active_axes["right_joystick"]["x"]
                            and not self.active_axes["right_joystick"]["y"]
                        ):
                            effective_deadzone = self.dominant_axis_threshold
                    case 3:
                        axis_val = right_x
                        if (
                            self.active_axes["right_joystick"]["y"]
                            and not self.active_axes["right_joystick"]["x"]
                        ):
                            effective_deadzone = self.dominant_axis_threshold
                    case 4:
                        left_trigger = msg.axes[self.node.axis_mapping["triggers"]["left"]]
                        right_trigger = msg.axes[self.node.axis_mapping["triggers"]["right"]]
                        axis_val = right_trigger - left_trigger
                    case 5:
                        move_left = (
                            msg.buttons[self.node.button_mapping["wrist_rotation_left"]] == 1
                        )
                        move_right = (
                            msg.buttons[self.node.button_mapping["wrist_rotation_right"]] == 1
                        )
                        if move_left:
                            axis_val = -1.0
                        elif move_right:
                            axis_val = 1.0

                # Target joint velocity (rad/s): the stick value gated by the deadzone and
                # scaled by the configurable max joint velocity (speed at full deflection).
                target_velocity = (
                    axis_val * max_velocity if abs(axis_val) > effective_deadzone else 0.0
                )

                # Slew-rate limit the velocity toward the target (bounded acceleration).
                max_dv = max_accel * self.node.dt
                dv = target_velocity - commanded_velocities[i]
                dv = max(-max_dv, min(max_dv, dv))
                commanded_velocities[i] += dv

                # Integrate the position with the limited velocity so the published
                # position and velocity stay consistent. Keep going while the velocity
                # is still ramping down so the stop stays smooth.
                if commanded_velocities[i] != 0.0:
                    current_position = self.duatic_robots_helper.get_joint_value_from_states(
                        joint_name
                    )
                    commanded_positions[i] += commanded_velocities[i] * self.node.dt
                    offset = commanded_positions[i] - current_position
                    if abs(offset) > self.joint_pos_offset_tolerance:
                        commanded_positions[i] = current_position + math.copysign(
                            self.joint_pos_offset_tolerance, offset
                        )
                        self.node.gamepad_feedback.send_feedback(intensity=1.0)
                    any_axis_active = True

            self.topic_to_commanded_positions[topic] = commanded_positions
            self.topic_to_commanded_velocities[topic] = commanded_velocities

        # Publish if active or if this is the first idle frame (to send the final state)
        if any_axis_active or not self.is_joystick_idle:
            for topic, publisher in self.joint_trajectory_publishers.items():
                arm_name = self.get_arm_from_topic(topic)
                if arm_name == self.focused_component:
                    self.publish_joint_trajectory(
                        self.topic_to_commanded_positions[topic],
                        self.topic_to_commanded_velocities[topic],
                        publisher,
                        self.topic_to_joint_names[topic],
                    )

            # Update idle state: if no axis was active, we are now idle
            self.is_joystick_idle = not any_axis_active

    def publish_joint_trajectory(
        self, target_positions, target_velocities, publisher, joint_names, speed_percentage=1.0
    ):
        """Publishes a joint trajectory message for the given positions using the provided publisher."""

        if not joint_names:
            self.node.get_logger().error("No joint names available. Cannot publish trajectory.")
            return

        if not target_positions:
            self.node.get_logger().error("No trajectory points available to publish.")
            return

        # Clamp speed percentage between 1 and 100
        speed_percentage = max(1.0, min(100.0, speed_percentage))

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = joint_names
        point = JointTrajectoryPoint()

        # Stream a constant-velocity lookahead point: place it `lookahead` seconds ahead
        # at the current commanded velocity, with time_from_start = lookahead. Position,
        # velocity and time stay mutually consistent (distance = velocity * lookahead), so
        # the 1 kHz controller interpolates a smooth constant-velocity ramp between our
        # updates instead of accelerating and braking within every cycle. The 2-cycle
        # horizon also bridges a single late or dropped command.
        lookahead = self.node.get_parameter("jtc_lookahead_cycles").value * self.node.dt
        point.positions = [
            pos + vel * lookahead for pos, vel in zip(target_positions, target_velocities)
        ]
        point.velocities = list(target_velocities)
        point.accelerations = [0.0] * len(joint_names)
        time_in_sec = lookahead
        sec = int(time_in_sec)
        nanosec = int((time_in_sec - sec) * 1e9)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec
        trajectory_msg.points.append(point)
        publisher.publish(trajectory_msg)

    def _update_dominant_axes(self, left_x, left_y, right_x, right_y, deadzone):
        """Update which axes are currently active to determine dominant axis behavior."""
        # Update active axes based on current input
        self.active_axes["left_joystick"]["x"] = abs(left_x) > deadzone
        self.active_axes["left_joystick"]["y"] = abs(left_y) > deadzone
        self.active_axes["right_joystick"]["x"] = abs(right_x) > deadzone
        self.active_axes["right_joystick"]["y"] = abs(right_y) > deadzone
