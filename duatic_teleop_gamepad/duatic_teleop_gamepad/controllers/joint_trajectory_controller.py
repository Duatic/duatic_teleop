# Copyright 2026 Duatic AG
# Duatic Commercial License 1.0 (DCL-1)

from duatic_teleop_gamepad.controllers.base_controller import BaseController
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointTrajectoryController(BaseController):
    """Handles joint trajectory control using the gamepad"""

    # Joint velocity (rad/s) commanded at full stick deflection.
    MAX_JOINT_VELOCITY = 1.0

    # Slew-rate limit on the commanded joint velocity (rad/s^2). Applied in tick() at
    # ~1 kHz so the ramp is maximally smooth regardless of the joy processing rate.
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
        # Slew-rate limiting toward these targets happens in tick() at ~1 kHz.
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

    def get_low_level_controllers(self):
        """Returns all discovered JTC controller names to keep them active simultaneously."""
        controllers = []
        for topic in self.topic_to_joint_names.keys():
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
            self.topic_to_target_velocities[topic] = [0.0] * len(joint_names)

        self.is_joystick_idle = True

    def process_input(self, msg):
        """Update target velocities from joystick input (runs at ~100 Hz).

        Only reads the joystick and sets target_velocities. Position integration
        and publishing happen in tick() at ~1 kHz so the JTC gets a continuous
        high-rate stream regardless of the joy topic rate.
        """
        super().process_input(msg)

        deadzone = 0.1
        max_velocity = self.node.get_parameter("jtc_max_joint_velocity").value

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

        for topic, joint_names in self.topic_to_joint_names.items():
            target_velocities = self.topic_to_target_velocities[topic]
            arm_name = self.get_arm_from_topic(topic)

            if arm_name != self.focused_component or not self.node.deadman_active:
                # Zero targets for non-focused topics or when deadman is released so
                # tick() slews commanded_vel back to zero smoothly.
                for i in range(len(joint_names)):
                    target_velocities[i] = 0.0
                self.topic_to_target_velocities[topic] = target_velocities
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

                target_velocities[i] = (
                    axis_val * max_velocity if abs(axis_val) > effective_deadzone else 0.0
                )

            self.topic_to_target_velocities[topic] = target_velocities

    def tick(self):
        """Integrate position and publish to JTC (runs at ~1 kHz via a dedicated timer).

        Applies slew-rate limiting from target_velocities (set at ~100 Hz) to
        commanded_velocities, integrates the position, and publishes a single
        trajectory point with time_from_start = dt_fast. At ~1 kHz the JTC
        receives a new target every controller cycle and never brakes between
        updates.
        """
        dt_fast = self.node.dt_fast
        max_accel = self.node.get_parameter("jtc_max_joint_accel").value
        max_dv = max_accel * dt_fast
        any_active = False

        for topic, joint_names in self.topic_to_joint_names.items():
            if self.get_arm_from_topic(topic) != self.focused_component:
                continue

            target_velocities = self.topic_to_target_velocities[topic]
            commanded_velocities = self.topic_to_commanded_velocities[topic]
            commanded_positions = self.topic_to_commanded_positions[topic]

            for i, joint_name in enumerate(joint_names):
                # Slew-rate limit toward the target velocity set by the joystick.
                dv = target_velocities[i] - commanded_velocities[i]
                dv = max(-max_dv, min(max_dv, dv))
                commanded_velocities[i] += dv

                if commanded_velocities[i] != 0.0:
                    current_position = self.duatic_robots_helper.get_joint_value_from_states(
                        joint_name
                    )
                    commanded_positions[i] += commanded_velocities[i] * dt_fast
                    offset = commanded_positions[i] - current_position
                    if abs(offset) > self.joint_pos_offset_tolerance:
                        commanded_positions[i] = current_position + math.copysign(
                            self.joint_pos_offset_tolerance, offset
                        )
                        self.node.gamepad_feedback.send_feedback(intensity=1.0)
                    any_active = True

            self.topic_to_commanded_velocities[topic] = commanded_velocities
            self.topic_to_commanded_positions[topic] = commanded_positions

        if any_active or not self.is_joystick_idle:
            for topic, publisher in self.joint_trajectory_publishers.items():
                if self.get_arm_from_topic(topic) == self.focused_component:
                    self.publish_joint_trajectory(
                        self.topic_to_commanded_positions[topic],
                        publisher,
                        self.topic_to_joint_names[topic],
                    )
            self.is_joystick_idle = not any_active

    def publish_joint_trajectory(self, target_positions, publisher, joint_names):
        """Publishes a single-point trajectory for streaming position control."""
        if not joint_names:
            self.node.get_logger().error("No joint names available. Cannot publish trajectory.")
            return

        if not target_positions:
            self.node.get_logger().error("No trajectory points available to publish.")
            return

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = list(target_positions)
        point.velocities = [0.0] * len(joint_names)
        point.accelerations = [0.0] * len(joint_names)
        dt_fast = self.node.dt_fast
        point.time_from_start.sec = int(dt_fast)
        point.time_from_start.nanosec = int((dt_fast - int(dt_fast)) * 1e9)
        trajectory_msg.points.append(point)
        publisher.publish(trajectory_msg)

    def _update_dominant_axes(self, left_x, left_y, right_x, right_y, deadzone):
        """Update which axes are currently active to determine dominant axis behavior."""
        self.active_axes["left_joystick"]["x"] = abs(left_x) > deadzone
        self.active_axes["left_joystick"]["y"] = abs(left_y) > deadzone
        self.active_axes["right_joystick"]["x"] = abs(right_x) > deadzone
        self.active_axes["right_joystick"]["y"] = abs(right_y) > deadzone
