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

from duatic_helpers.duatic_param_helper import DuaticParamHelper


class JointTrajectoryController(BaseController):
    """Handles joint trajectory control using the gamepad"""

    # Joint velocity (rad/s) commanded at full stick deflection.
    MAX_JOINT_VELOCITY = 1.0

    # Slew-rate limit on the commanded joint velocity (rad/s^2).
    MAX_JOINT_ACCEL = 5.0

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

        # Target velocities: set by process_input() at ~100 Hz from the joystick.
        self.topic_to_target_velocities = {
            topic: [0.0] * len(joint_names)
            for topic, joint_names in self.topic_to_joint_names.items()
        }

        # Commanded velocities: slew-limited version of target_velocities, updated in tick().
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
        self.is_lagging = False

        self.topic_to_velocity_supported = self._discover_velocity_support()

        # Live-tunable on hardware via `ros2 param set /gamepad_interface <name> <value>`.
        self.node.declare_parameter("jtc_max_joint_velocity", self.MAX_JOINT_VELOCITY)
        self.node.declare_parameter("jtc_max_joint_accel", self.MAX_JOINT_ACCEL)

        # Dominant axis tracking
        self.dominant_axis_threshold = 0.6
        self.active_axes = {
            "left_joystick": {"x": False, "y": False},
            "right_joystick": {"x": False, "y": False},
        }

        self.node.get_logger().info("Joint Trajectory Controller initialized.")

    def _controller_ns(self, topic):
        """Return the controller name a joint_trajectory topic belongs to."""
        return topic.strip("/").split("/")[-2]

    def _discover_velocity_support(self):
        """Map each discovered topic to whether its controller accepts a moving end point.

        A JointTrajectoryController with allow_nonzero_velocity_at_trajectory_end left at
        its default rejects any trajectory whose final point has a non-zero velocity, so
        for those the stream has to fall back to zero end velocities.
        """
        param_helper = DuaticParamHelper(self.node)
        supported = {}

        for topic in self.topic_to_joint_names:
            controller_ns = self._controller_ns(topic)
            values = param_helper.get_param_values(
                controller_ns, "allow_nonzero_velocity_at_trajectory_end"
            )
            supported[topic] = bool(values[0].bool_value) if values else False

            if not supported[topic]:
                self.node.get_logger().warning(
                    f"{controller_ns} rejects a non-zero velocity at the trajectory end, so "
                    f"it is streamed zero end velocities and will decelerate between points. "
                    f"Set allow_nonzero_velocity_at_trajectory_end: true in its config for "
                    f"smooth jogging."
                )

        return supported

    def get_low_level_controllers(self):
        """Returns all discovered JTC controller names to keep them active simultaneously."""
        controllers = [self._controller_ns(topic) for topic in self.topic_to_joint_names]

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
            self.topic_to_target_velocities[topic] = [0.0] * len(joint_names)

        self.is_joystick_idle = True
        self._update_lag_feedback(False)

    def process_input(self, msg):
        """Update target velocities from joystick input (runs at ~100 Hz)."""
        super().process_input(msg)

        deadzone = 0.1
        max_velocity = self.node.get_parameter("jtc_max_joint_velocity").value

        left_x = self.node.axis(msg, "left_joystick", "x")
        left_y = self.node.axis(msg, "left_joystick", "y")
        right_x = self.node.axis(msg, "right_joystick", "x")
        right_y = self.node.axis(msg, "right_joystick", "y")

        self._update_dominant_axes(left_x, left_y, right_x, right_y, deadzone)

        for topic, joint_names in self.topic_to_joint_names.items():
            target_velocities = self.topic_to_target_velocities[topic]
            arm_name = self.get_arm_from_topic(topic)

            if arm_name != self.focused_component or not self.node.deadman_active:
                # Zero targets for non-focused topics or when deadman is released so
                # tick() slews commanded_vel back to zero smoothly.
                for i in range(len(joint_names)):
                    target_velocities[i] = 0.0
                continue

            for i, _joint_name in enumerate(joint_names):
                axis_val = 0.0
                effective_deadzone = deadzone

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
                        axis_val = self.node.axis(msg, "triggers", "right") - self.node.axis(
                            msg, "triggers", "left"
                        )
                    case 5:
                        if self.node.button(msg, "wrist_rotation_left") == 1:
                            axis_val = -1.0
                        elif self.node.button(msg, "wrist_rotation_right") == 1:
                            axis_val = 1.0

                target_velocities[i] = (
                    axis_val * max_velocity if abs(axis_val) > effective_deadzone else 0.0
                )

    def tick(self):
        """Integrate the commanded position and publish it, on the node's JTC timer.

        Applies slew-rate limiting from target_velocities (set by process_input()) to
        commanded_velocities, integrates the position, and publishes a single trajectory
        point carrying both, with time_from_start = one publish period. The controller
        interpolates between points at its own update rate, and the point's velocity is
        what lets it coast through a late message instead of braking to a stop.
        """
        dt = self.node.jtc_dt
        max_accel = self.node.get_parameter("jtc_max_joint_accel").value
        max_dv = max_accel * dt
        joint_states = self.duatic_robots_helper.get_joint_states()
        any_active = False
        lagging = False

        for topic, joint_names in self.topic_to_joint_names.items():
            if self.get_arm_from_topic(topic) != self.focused_component:
                continue

            if any(joint_name not in joint_states for joint_name in joint_names):
                self.node.get_logger().warning(
                    f"{self._controller_ns(topic)} has joints missing from the joint states; "
                    f"not commanding it.",
                    throttle_duration_sec=5.0,
                )
                continue

            target_velocities = self.topic_to_target_velocities[topic]
            commanded_velocities = self.topic_to_commanded_velocities[topic]
            commanded_positions = self.topic_to_commanded_positions[topic]

            for i, joint_name in enumerate(joint_names):
                # Slew-rate limit toward the target velocity set by the joystick.
                dv = target_velocities[i] - commanded_velocities[i]
                dv = max(-max_dv, min(max_dv, dv))
                commanded_velocities[i] += dv

                if commanded_velocities[i] == 0.0:
                    continue

                commanded_positions[i] += commanded_velocities[i] * dt
                any_active = True

                # Stop the command running away from where the arm actually is.
                offset = commanded_positions[i] - joint_states[joint_name]
                if abs(offset) > self.joint_pos_offset_tolerance:
                    commanded_positions[i] = joint_states[joint_name] + math.copysign(
                        self.joint_pos_offset_tolerance, offset
                    )
                    lagging = True

        self._update_lag_feedback(lagging)

        if any_active or not self.is_joystick_idle:
            for topic, publisher in self.joint_trajectory_publishers.items():
                if self.get_arm_from_topic(topic) == self.focused_component:
                    self.publish_joint_trajectory(topic, publisher)
            self.is_joystick_idle = not any_active

    def _update_lag_feedback(self, lagging):
        """Rumble on the edges of the position-lag state.

        Rumble set over /joy/set_feedback persists until it is changed, so it needs one
        message when the arm starts lagging behind the command and one when it catches up.
        """
        if lagging == self.is_lagging:
            return

        self.is_lagging = lagging
        self.node.gamepad_feedback.send_feedback(intensity=1.0 if lagging else 0.0)

    def publish_joint_trajectory(self, topic, publisher):
        """Publishes a single-point trajectory for streaming position control."""
        joint_names = self.topic_to_joint_names[topic]

        point = JointTrajectoryPoint()
        point.positions = list(self.topic_to_commanded_positions[topic])
        if self.topic_to_velocity_supported[topic]:
            point.velocities = list(self.topic_to_commanded_velocities[topic])
        else:
            point.velocities = [0.0] * len(joint_names)
        dt = self.node.jtc_dt
        point.time_from_start.sec = int(dt)
        point.time_from_start.nanosec = int((dt - int(dt)) * 1e9)

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = joint_names
        trajectory_msg.points.append(point)
        publisher.publish(trajectory_msg)

    def _update_dominant_axes(self, left_x, left_y, right_x, right_y, deadzone):
        """Update which axes are currently active to determine dominant axis behavior."""
        self.active_axes["left_joystick"]["x"] = abs(left_x) > deadzone
        self.active_axes["left_joystick"]["y"] = abs(left_y) > deadzone
        self.active_axes["right_joystick"]["x"] = abs(right_x) > deadzone
        self.active_axes["right_joystick"]["y"] = abs(right_y) > deadzone
