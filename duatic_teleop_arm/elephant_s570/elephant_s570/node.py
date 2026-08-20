#!/usr/bin/env python3
"""ROS 2 Node for teleoperation using the Elephant Robotics MyController S570.

Subscribes to S570 joint states published via rosbridge by the Windows publisher and
publishes PoseStamped targets for the IK solver (interactive_pyroki_node).

Each arm runs its own two-state state machine:
  - REPOSITION: the arm's and the robot's current Cartesian poses (position + quaternion)
    are continuously tracked as "reference poses". Nothing is published as a target.
  - CONTROL: entered on deadman press (or, on the right arm, axis-lock activation), via
    on_control_activate(). Both reference poses are frozen at that instant. Every control
    cycle after that, the arm's ACTUAL pose (still updated live) is compared to its frozen
    reference pose as a 6DoF twist (linear + angular difference), scaled (by
    'teleop_linear_scale' / 'teleop_angular_scale'), and added directly onto the frozen
    robot reference pose to produce the target.
  - On deadman release, the arm returns to REPOSITION and stops publishing target poses.

On deadman press: deactivates freeze_controller, activates joint_trajectory_controller.
On deadman release: deactivates JTC, activates freeze_controller.

Each arm is independently controlled via a deadman switch (Button A on each side).
Hold A to teleop, release to stop.

The right arm also has an axis-lock mode (a separate button): once the robot's tracked
orientation has settled, activating it calls on_control_activate() the same way the deadman
does (a no-op if CONTROL was already entered via the deadman), then additionally locks
translation to a single axis (the robot EE's local Z axis at that instant) and freezes
orientation for the duration of the lock.

Parameters:
    arm_side:                "left", "right", or "both" (default: "both")
    dual_arm_robot:          True if the robot has two arms (default: True)
    teleop_linear_scale:     Gain applied to the arm's linear motion (default: 1.0)
    teleop_angular_scale:    Gain applied to the arm's angular motion (default: 1.0)
    target_topic_prefix:    Base topic to publish target poses to
                             (default: "/cartesian_pose_controller/target_pose")
    target_topic_suffix:    Appended after "/<side>" for a dual-arm robot, or right after
                             target_topic_prefix for a single arm (default: "")

Button mapping (per S570 arm):
    A (hold):  Deadman switch — teleop active while held
    B, C, D:   Free (future: gripper, mode switch, etc.)
    Joystick:  Free (future: speed scaling, etc.)

Usage:
    ros2 run elephant_s570 elephant_s570_node
    ros2 run elephant_s570 elephant_s570_node --ros-args -p arm_side:=left
    ros2 run elephant_s570 elephant_s570_node --ros-args -p arm_side:=right -p dual_arm_robot:=false
"""

from dataclasses import dataclass
from enum import Enum, auto

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener

from duatic_helpers.duatic_controller_helper import (
    DuaticControllerHelper,
)
from elephant_s570.fk import S570FK

import websocket
import json


def _quat_to_rotation_matrix(q_wxyz: np.ndarray) -> np.ndarray:
    """Convert quaternion [w, x, y, z] to 3x3 rotation matrix."""
    w, x, y, z = q_wxyz
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def _rotation_matrix_to_rotvec(R: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix -> rotation vector (axis * angle)."""
    cos_angle = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cos_angle)
    if angle < 1e-8:
        return np.zeros(3)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (
        2.0 * np.sin(angle)
    )
    return axis * angle


def _rotvec_to_quat(rotvec: np.ndarray) -> np.ndarray:
    """Rotation vector (axis * angle) -> unit quaternion [w, x, y, z]."""
    angle = np.linalg.norm(rotvec)
    if angle < 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = rotvec / angle
    half = angle / 2.0
    return np.array([np.cos(half), *(axis * np.sin(half))])


def _quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product of two quaternions [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


class ControlState(Enum):
    REPOSITION = auto()  # tracking reference poses live; nothing published
    CONTROL = auto()  # reference poses frozen; publishing twist-derived targets


@dataclass
class Pose6D:
    """Cartesian pose: position (3,) + unit quaternion [w, x, y, z] (4,)."""

    position: np.ndarray
    quat: np.ndarray


class S570ArmState:
    """Tracking state for one S570 arm."""

    def __init__(self, side: str):
        self.side = side
        self.state = ControlState.REPOSITION
        self.current_joints: np.ndarray | None = None
        self.deadman_held = False

        # Reference poses: continuously updated while REPOSITION, frozen while CONTROL.
        self.arm_reference_pose: Pose6D | None = None
        self.robot_reference_pose: Pose6D | None = None

        # Axis lock (right arm only, folded into the CONTROL state — see on_control_activate())
        self.axis_lock_active = False
        self.axis_lock_axis: np.ndarray | None = None  # locked axis, in robot/base frame
        self.prev_quat: np.ndarray | None = None  # for the orientation-stability gate
        self.stable_counter = 0

        # Self-collision-avoidance fallback (see _compute_target_control)
        self.prev_target_pos: np.ndarray | None = None


class ElephantS570Node(Node):
    def __init__(self):
        super().__init__("elephant_s570_node")

        # --- Parameters ---
        self.declare_parameter("arm_side", "both")
        self.declare_parameter("dual_arm_robot", True)
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("robot_ee_frame", "flange")
        self.declare_parameter("teleop_linear_scale", 1.0)
        self.declare_parameter("teleop_angular_scale", 1.0)
        self.declare_parameter("target_topic_prefix", "/cartesian_pose_controller/target_pose")
        self.declare_parameter("target_topic_suffix", "")
        # In case we want to run the inverse kinematics on a host machine
        # and then  send the jtc targets via rosbridge to the NUC:
        self.declare_parameter("uri", "ws://192.168.89.157:9090")
        self.declare_parameter("rosbridge", False)

        self.use_rosbridge = self.get_parameter("rosbridge").value
        if self.use_rosbridge:
            self.rosbridge_uri = self.get_parameter("uri").value

            self.ws = websocket.WebSocket()
            self.ws.connect(self.rosbridge_uri)
            self.advertised_topics = set()

        self.arm_side: str = self.get_parameter("arm_side").value
        self.dual_arm_robot: bool = self.get_parameter("dual_arm_robot").value

        self.robot_base_frame: str = self.get_parameter("robot_base_frame").value
        self.robot_ee_frame: str = self.get_parameter("robot_ee_frame").value
        self.teleop_linear_scale: float = float(self.get_parameter("teleop_linear_scale").value)
        self.teleop_angular_scale: float = float(self.get_parameter("teleop_angular_scale").value)
        self.target_topic_prefix: str = self.get_parameter("target_topic_prefix").value
        self.target_topic_suffix: str = self.get_parameter("target_topic_suffix").value

        if self.arm_side not in ("left", "right", "both"):
            self.get_logger().error(f"Invalid arm_side '{self.arm_side}', defaulting to 'both'")
            self.arm_side = "both"

        # --- Controller helper (created lazily on first deadman press) ---
        self.controller_helper: DuaticControllerHelper | None = None
        self._controllers_switched = False

        # --- TF listener for DynaArm EE pose lookup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- S570 FK ---
        self.fk = S570FK()
        self.get_logger().info("S570 FK model loaded from URDF")

        # --- Per-arm state ---
        self.arms: dict[str, S570ArmState] = {}
        self._sides = ["left", "right"] if self.arm_side == "both" else [self.arm_side]

        for side in self._sides:
            self.arms[side] = S570ArmState(side)

        # Single combined joint_states topic (14 joints: 7 left + 7 right)
        self.create_subscription(JointState, "/teleop_arm/joint_states", self._joint_state_cb, 10)
        self.get_logger().info("Subscribed to /teleop_arm/joint_states")

        # Buttons
        self.create_subscription(Joy, "/teleop_arm/buttons", self._buttons_cb, 10)

        # --- Button index mapping ---
        # In /teleop_arm/buttons Joy message:
        #   buttons[0-3] = Left  A, B, C, D
        #   buttons[4-7] = Right A, B, C, D
        #   axes[0-1]    = Left  Joystick X, Y
        #   axes[2-3]    = Right Joystick X, Y
        # The left-side buttons/axes are physically broken, so _buttons_cb() mirrors the
        # right-side values into the left-side slots before handling each side individually.
        self._deadman_button_index = {"left": 0, "right": 4, "grasp": 5}
        self._axis_lock_button_index = {"right": 5}
        self._joy_axes: list[float] = []

        # --- Target pose publishers ---
        self.pose_pubs: dict[str, rclpy.publisher.Publisher] = {}

        if self.dual_arm_robot:
            for side in self._sides:
                topic = f"{self.target_topic_prefix}/{side}{self.target_topic_suffix}"
                self.pose_pubs[side] = self.create_publisher(PoseStamped, topic, 10)
                self.get_logger().info(f"Publishing to {topic}")
        else:
            topic = f"{self.target_topic_prefix}{self.target_topic_suffix}"
            self.pose_pubs[self._sides[0]] = self.create_publisher(PoseStamped, topic, 10)
            self.get_logger().info(f"Publishing to {topic}")

        # --- Visualization markers ---
        # S570 FK end-effector marker (for S570 RViz — always published)
        self._s570_marker_pub = self.create_publisher(MarkerArray, "/s570/marker_visu", 10)
        # DynaArm target marker (for DynaArm RViz — only when active)
        self._target_marker_pub = self.create_publisher(MarkerArray, "/marker_visu", 10)

        # --- Visualization: republish joints with URDF names for robot_state_publisher ---
        # URDF uses joint1-7 (left) and joint8-14 (right), but incoming uses s570_left/joint_1 etc.
        self._urdf_joint_pub = self.create_publisher(JointState, "/s570/joint_states", 10)
        # Mapping: side -> list of URDF joint names
        self._urdf_joint_names = {
            "left": [f"joint{i}" for i in range(1, 8)],
            "right": [f"joint{i}" for i in range(8, 15)],
        }

        # choose between teleop methods, currently available:
        # 'mapped_actionspaces', 'remapping_deltas'
        self.teleop_method: str = "remapping_deltas"

        self.visu_pose_pubs: dict[str, rclpy.publisher.Publisher] = {}

        if self.dual_arm_robot:
            for side in self._sides:
                topic = f"/visu/target_pose/{side}"
                self.visu_pose_pubs[side] = self.create_publisher(PoseStamped, topic, 10)
        else:
            self.visu_pose_pubs[self._sides[0]] = self.create_publisher(
                PoseStamped, "/visu/target_pose", 10
            )

        # --- Publish timer (50 Hz) ---
        self.create_timer(0.02, self._publish_targets)
        self.log_counter = 0

        self.get_logger().info(
            f"Elephant S570 node ready — arm_side={self.arm_side}, "
            f"dual_arm_robot={self.dual_arm_robot}"
        )
        self.get_logger().info("Hold Button A on S570 to start teleop.")

        self.axis_pub = self.create_publisher(Marker, "/axis_lock_marker", 10)

    # ------------------------------------------------------------------ #
    #  Controller switching                                               #
    # ------------------------------------------------------------------ #

    def _any_arm_active(self) -> bool:
        return any(arm.state == ControlState.CONTROL for arm in self.arms.values())

    def _ensure_controller_helper(self) -> DuaticControllerHelper:
        """Lazily create the controller helper on first use."""
        if self.controller_helper is None:
            self.controller_helper = DuaticControllerHelper(self)
            self.get_logger().info("Controller helper initialized.")
        return self.controller_helper

    def _activate_controllers(self) -> None:
        """Deactivate freeze, activate JTC for teleop."""
        if self._controllers_switched:
            return

        helper = self._ensure_controller_helper()
        jtc_names = helper.get_all_controllers(matching_names=["joint_trajectory_controller"])
        freeze_names = helper.get_all_controllers(matching_names=["freeze_controller"])

        if jtc_names:
            self.controller_helper.switch_controller(
                activate_controllers=jtc_names,
                deactivate_controllers=freeze_names,
            )
            self.get_logger().info(
                f"Controllers: activating {jtc_names}, deactivating {freeze_names}"
            )
        else:
            self.get_logger().warn("No joint_trajectory_controller found!")

        self._controllers_switched = True

    def _deactivate_controllers(self) -> None:
        """Deactivate JTC, activate freeze."""
        if not self._controllers_switched:
            return

        helper = self._ensure_controller_helper()
        jtc_names = helper.get_all_controllers(matching_names=["joint_trajectory_controller"])
        freeze_names = helper.get_all_controllers(matching_names=["freeze_controller"])

        if freeze_names:
            self.controller_helper.switch_controller(
                activate_controllers=freeze_names,
                deactivate_controllers=jtc_names,
            )
            self.get_logger().info(
                f"Controllers: activating {freeze_names}, deactivating {jtc_names}"
            )

        self._controllers_switched = False

    # ------------------------------------------------------------------ #
    #  Callbacks                                                          #
    # ------------------------------------------------------------------ #

    def _joint_state_cb(self, msg: JointState) -> None:
        """Split combined JointState (14 joints) into per-arm state, and — while
        REPOSITION — keep both reference poses live."""
        joint_map = dict(zip(msg.name, msg.position))

        for side, arm in self.arms.items():
            prefix = f"s570_{side}/"
            joints = [joint_map.get(f"{prefix}joint_{i + 1}", 0.0) for i in range(7)]
            had_data = arm.current_joints is not None
            arm.current_joints = np.array(joints)

            if not had_data:
                self.get_logger().info(f"S570 {side} data received. Hold A to teleop.")

            if arm.state == ControlState.REPOSITION:
                self._update_reference_pose(arm, side)

    def _update_reference_pose(self, arm: S570ArmState, side: str) -> None:
        """While REPOSITION: continuously refresh both reference poses from live data."""
        pos, quat = self.fk.compute(side, arm.current_joints)
        arm.arm_reference_pose = Pose6D(pos, quat)

        tf_result = self._lookup_robot_ee_pose(side)
        if tf_result is not None:
            robot_pos, robot_quat = tf_result
            arm.robot_reference_pose = Pose6D(robot_pos, robot_quat)
        # else: TF failed this tick. robot_reference_pose is left as-is — None until the
        # first successful lookup (on_control_activate() correctly refuses to activate until
        # then), or its last successfully tracked value otherwise.

    def _buttons_cb(self, msg: Joy) -> None:
        # The left-side buttons/axes are physically broken on this controller — mirror the
        # right-side raw values into the left-side slots up front, so every side below can be
        # handled individually/symmetrically instead of special-casing "left".
        buttons = list(msg.buttons)
        axes = list(msg.axes)
        if len(buttons) >= 8:
            buttons[0:4] = buttons[4:8]
        if len(axes) >= 4:
            axes[0:2] = axes[2:4]
        self._joy_axes = axes  # not consumed yet — reserved for future speed scaling etc.

        for side, arm in self.arms.items():
            btn_idx = self._deadman_button_index[side]
            if btn_idx >= len(buttons):
                continue
            is_held = buttons[btn_idx] == 0  # Active-low

            was_held = arm.deadman_held
            arm.deadman_held = is_held

            if is_held and not was_held:
                self._arm_activate(arm)
            elif not is_held and was_held:
                self._arm_deactivate(arm)

            if side == "right":
                axis_btn_idx = self._axis_lock_button_index["right"]
                axis_pressed = axis_btn_idx < len(buttons) and buttons[axis_btn_idx] == 0
                if axis_pressed and not arm.axis_lock_active:
                    self._activate_axis_lock(arm)
                elif not axis_pressed and arm.axis_lock_active:
                    self._deactivate_axis_lock(arm)

    # ------------------------------------------------------------------ #
    #  Arm activation                                                    #
    # ------------------------------------------------------------------ #

    def _lookup_robot_ee_pose(self, side: str) -> tuple[np.ndarray, np.ndarray] | None:
        """Try to get the current DynaArm EE pose via TF."""
        # For dual-arm: arm_left/flange, arm_right/flange
        # For single-arm: flange
        if self.dual_arm_robot:
            ee_frame = f"arm_{side}/{self.robot_ee_frame}"
        else:
            ee_frame = self.robot_ee_frame

        try:
            t = self.tf_buffer.lookup_transform(self.robot_base_frame, ee_frame, Time())
            pos = np.array(
                [
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z,
                ]
            )
            quat = np.array(
                [
                    t.transform.rotation.w,
                    t.transform.rotation.x,
                    t.transform.rotation.y,
                    t.transform.rotation.z,
                ]
            )
            return pos, quat
        except Exception as e:
            self.get_logger().warn(f"TF lookup {self.robot_base_frame} → {ee_frame} failed: {e}")
            return None

    def on_control_activate(self, arm: S570ArmState) -> bool:
        """REPOSITION -> CONTROL: freeze both reference poses.

        Returns False (and stays in REPOSITION) if no reference pose is available yet.
        """
        if arm.arm_reference_pose is None or arm.robot_reference_pose is None:
            self.get_logger().warn(
                f"S570 {arm.side}: no reference pose yet, cannot activate control."
            )
            return False

        arm.state = ControlState.CONTROL
        self.get_logger().info(
            f"S570 {arm.side} CONTROL activated — "
            f"arm_ref_pos={arm.arm_reference_pose.position.round(3).tolist()}, "
            f"robot_ref_pos={arm.robot_reference_pose.position.round(3).tolist()}"
        )

        # Switch controllers on first arm activation
        self._activate_controllers()
        return True

    def on_control_deactivate(self, arm: S570ArmState) -> None:
        """CONTROL -> REPOSITION: resume tracking, stop publishing targets for this arm."""
        arm.state = ControlState.REPOSITION
        self.get_logger().info(f"S570 {arm.side} released — REPOSITION")

        # Only deactivate controllers when ALL arms are released
        if not self._any_arm_active():
            self._deactivate_controllers()

    def _arm_activate(self, arm: S570ArmState) -> None:
        self.on_control_activate(arm)

    def _arm_deactivate(self, arm: S570ArmState) -> None:
        if arm.state != ControlState.CONTROL:
            return
        # Don't drop out of CONTROL if axis-lock is still holding this arm active.
        if arm.axis_lock_active:
            return
        self.on_control_deactivate(arm)

    def _activate_axis_lock(self, arm: S570ArmState) -> None:
        if arm.robot_reference_pose is None:
            self.get_logger().warn(f"[{arm.side}] Axis lock: no reference pose yet.")
            return

        # 👉 Warten bis stabil
        if not self._is_orientation_stable(arm, arm.robot_reference_pose.quat):
            self.get_logger().info("Waiting for stable orientation...", throttle_duration_sec=1.0)
            return

        # 👉 Jetzt erst locken — reuse the deadman's own activation/calibration if it hasn't
        # already happened (e.g. axis lock pressed without holding the deadman); if CONTROL
        # was already entered via the deadman, just add the lock on top of it.
        if arm.state != ControlState.CONTROL:
            if not self.on_control_activate(arm):
                return

        R_robot = _quat_to_rotation_matrix(arm.robot_reference_pose.quat)
        z_axis = R_robot[:, 2]
        arm.axis_lock_axis = z_axis / np.linalg.norm(z_axis)
        arm.axis_lock_active = True

        self._publish_axis_marker(arm, arm.axis_lock_axis)

        self.get_logger().info(
            f"[{arm.side}] Axis lock ACTIVATED — axis={arm.axis_lock_axis.round(3)}"
        )

    def _is_orientation_stable(self, arm, current_quat, threshold=0.9995, steps=5):
        if arm.prev_quat is None:
            arm.prev_quat = current_quat
            return False

        dot = abs(np.dot(current_quat, arm.prev_quat))

        if dot > threshold:
            arm.stable_counter += 1
        else:
            arm.stable_counter = 0

        arm.prev_quat = current_quat

        return arm.stable_counter >= steps

    def _publish_axis_marker(self, arm, axis):
        pos = arm.robot_reference_pose.position

        marker = Marker()
        marker.header.frame_id = self.robot_base_frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "axis_lock"
        marker.id = 0 if arm.side == "left" else 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        start = Point(x=pos[0], y=pos[1], z=pos[2])

        scale = 0.2
        end = Point(
            x=pos[0] + axis[0] * scale,
            y=pos[1] + axis[1] * scale,
            z=pos[2] + axis[2] * scale,
        )

        marker.points = [start, end]

        marker.scale.x = 0.01
        marker.scale.y = 0.02
        marker.scale.z = 0.02

        marker.color.r = 1.0
        marker.color.a = 1.0

        self.axis_pub.publish(marker)

    def _deactivate_axis_lock(self, arm: S570ArmState) -> None:
        arm.axis_lock_active = False
        arm.axis_lock_axis = None

        self.get_logger().info(f"[{arm.side}] Axis lock DEACTIVATED")

        # Don't drop out of CONTROL if the deadman is still holding this arm active.
        if not arm.deadman_held:
            self.on_control_deactivate(arm)

    # ------------------------------------------------------------------ #
    #  FK-based target computation                                        #
    # ------------------------------------------------------------------ #

    def _compute_target_control(
        self, arm: S570ArmState, side: str
    ) -> tuple[np.ndarray, np.ndarray] | None:
        """CONTROL state: twist(arm_actual - arm_reference), scaled, added directly onto the
        frozen robot_reference_pose.

        Returns (position[3], quaternion_wxyz[4]) or None.
        """
        if (
            arm.state != ControlState.CONTROL
            or arm.current_joints is None
            or arm.arm_reference_pose is None
            or arm.robot_reference_pose is None
        ):
            return None

        # --- Arm's live actual pose (kept up to date every cycle, unlike the frozen reference) ---
        actual_pos, actual_quat = self.fk.compute(side, arm.current_joints)

        # --- 6DoF twist: how far the arm has moved since activation, in its own base frame ---
        linear_diff = actual_pos - arm.arm_reference_pose.position
        R_ref = _quat_to_rotation_matrix(arm.arm_reference_pose.quat)
        R_actual = _quat_to_rotation_matrix(actual_quat)
        angular_diff = _rotation_matrix_to_rotvec(R_actual @ R_ref.T)

        # --- Scale ---
        linear_diff = linear_diff * self.teleop_linear_scale
        angular_diff = angular_diff * self.teleop_angular_scale

        if arm.axis_lock_active:
            # Only allow motion along the locked axis; orientation stays exactly at reference.
            linear_diff = np.dot(linear_diff, arm.axis_lock_axis) * arm.axis_lock_axis
            angular_diff = np.zeros(3)

        # --- Add onto the frozen robot reference pose ---
        target_pos = arm.robot_reference_pose.position + linear_diff

        if self.log_counter % 25 == 0:
            self.get_logger().info(f"{arm.side} — " f"target pos={target_pos.round(3).tolist()}, ")

        # Add simple self collision avoidance
        # End effectors cannot drive into base
        if target_pos[0] <= 0.76:
            target_pos[0] = 0.76
        # End effectors cannot driver too close to each other
        # Minimum allowed safe distance between end effectors
        safe_distance = 0.4

        # Check distance to other end effector
        is_safe = True

        self.log_counter += 1

        for other_side, other_arm in self.arms.items():

            # Skip current arm (and any arm without a reference pose yet)
            if other_side == side or other_arm.robot_reference_pose is None:
                continue

            other_pos = other_arm.robot_reference_pose.position

            if self.log_counter % 25 == 0:
                self.get_logger().info(
                    f"{arm.side} — " f"other pos={other_pos.round(3).tolist()}, "
                )

            # 3D Euclidean distance
            old_distance = np.linalg.norm(arm.robot_reference_pose.position - other_pos)
            new_distance = np.linalg.norm(target_pos - other_pos)

            if self.log_counter % 25 == 0:
                self.get_logger().info(f"resulting distance ={new_distance.round(3)}, ")

            if new_distance < safe_distance:
                if new_distance < old_distance:
                    if self.log_counter % 25 == 0:
                        self.get_logger().error("Too close, movement not allowed")
                    is_safe = False
                else:
                    if self.log_counter % 25 == 0:
                        self.get_logger().error(
                            "Already too close, but moving away from each other allowed"
                        )
                    is_safe = True

        # Only apply movement if safe. No previous target yet (e.g. first cycle after
        # activation) means there's nothing safe to fall back to — use the computed target
        # rather than crash downstream on a None position.
        if not is_safe and arm.prev_target_pos is not None:
            target_pos = arm.prev_target_pos

        arm.prev_target_pos = target_pos

        # --- Orientation: apply the delta onto the frozen robot reference, via left-multiply
        # (matches S570FK.compute_relative()'s own R_current = R_delta @ R_home convention) ---
        delta_quat = _rotvec_to_quat(angular_diff)
        target_quat = _quat_multiply(delta_quat, arm.robot_reference_pose.quat)
        target_quat /= np.linalg.norm(target_quat)

        return target_pos, target_quat

    def _compute_target_actionspace_method(
        self, arm: S570ArmState, side: str
    ) -> tuple[np.ndarray, np.ndarray] | None:
        """Compute Cartesian target by mapping teleop action space to robot action space."""

        if arm.current_joints is None:
            return None

        # --- Teleop FK (absolute pose) ---
        teleop_pos, teleop_quat = self.fk.compute(side, arm.current_joints)

        # --- Teleop limits (LEFT reference space) ---
        t_min = np.array([0.27, 0.0, -0.3])
        t_max = np.array([0.51, 0.75, 0.1])

        # --- Robot limits ---
        r_min = np.array([0.66, 0.0, 0.51])
        r_max = np.array([1.34, 1.17, 1.86])

        # --- Mirror Y for right arm ---
        if side == "right":
            teleop_pos[1] = -teleop_pos[1]

        # --- Cap teleop input if it exceeds limits ---
        teleop_pos = np.clip(teleop_pos, t_min, t_max)

        # --- Normalize to [0,1] ---
        norm = (teleop_pos - t_min) / (t_max - t_min)

        # --- Scale to robot workspace ---
        target_pos = r_min + norm * (r_max - r_min)

        # --- Mirror Y for right arm ---
        if side == "right":
            target_pos[1] = -target_pos[1]

        # --- Orientation (align teleop coord frame with flange coord frame) ---
        target_quat = self._align_teleop_frame_with_flange_frame(teleop_quat)
        target_quat /= np.linalg.norm(target_quat)

        return target_pos, target_quat

    def _align_teleop_frame_with_flange_frame(self, quat_wxyz: np.ndarray) -> np.ndarray:
        """Remap a quaternion [w, x, y, z] from the S570's own axis convention into the
        DynaArm flange's axis convention (a fixed left-multiply by a constant q_offset).

        Only used by _compute_target_actionspace_method (an alternate, currently-unselected
        teleop method) — the default remapping-deltas control path doesn't need this at all
        (see _compute_target_control()).
        """
        # teleop basis
        x_t = np.array([1, 0, 0])
        y_t = np.array([0, 1, 0])

        # desired robot axes expressed in teleop frame
        z_r = -x_t  # teleop x → robot z
        y_r = y_t  # teleop z → robot y
        x_r = np.cross(y_r, z_r)  # enforce right-handed system

        # build rotation matrix (columns = robot axes in teleop frame)
        R_offset = np.column_stack((x_r, y_r, z_r))
        q_offset = _rotvec_to_quat(_rotation_matrix_to_rotvec(R_offset))

        return _quat_multiply(q_offset, quat_wxyz)

    # ------------------------------------------------------------------ #
    #  Publishing                                                        #
    # ------------------------------------------------------------------ #

    def _publish_targets(self) -> None:
        stamp = self.get_clock().now().to_msg()

        # Republish joint states with URDF-compatible names for robot_state_publisher
        self._publish_urdf_joints(stamp)

        # Always publish S570 FK end-effector markers (for debugging in S570 RViz)
        self._publish_s570_fk_markers(stamp)

        for side, arm in self.arms.items():
            if side not in self.pose_pubs:
                continue

            # --- Teleop mode selection ---
            if self.teleop_method == "remapping_deltas":
                result = (
                    self._compute_target_control(arm, side)
                    if arm.state == ControlState.CONTROL
                    else None
                )
            else:
                result = self._compute_target_actionspace_method(arm, side)

            if result is None:
                continue

            pos, quat = result

            msg = PoseStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self.robot_base_frame
            msg.pose.position.x = float(pos[0])
            msg.pose.position.y = float(pos[1])
            msg.pose.position.z = float(pos[2])
            msg.pose.orientation.w = float(quat[0])
            msg.pose.orientation.x = float(quat[1])
            msg.pose.orientation.y = float(quat[2])
            msg.pose.orientation.z = float(quat[3])

            if side in self.visu_pose_pubs:
                self.visu_pose_pubs[side].publish(msg)

            topic = self.pose_pubs[side].topic_name

            if self.use_rosbridge:
                ros_msg = {
                    "header": {
                        "stamp": {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)},
                        "frame_id": self.robot_base_frame,
                    },
                    "pose": {
                        "position": {"x": float(pos[0]), "y": float(pos[1]), "z": float(pos[2])},
                        "orientation": {
                            "w": float(quat[0]),
                            "x": float(quat[1]),
                            "y": float(quat[2]),
                            "z": float(quat[3]),
                        },
                    },
                }

                # 1. Advertise
                advertise_msg = {
                    "op": "advertise",
                    "topic": topic,
                    "type": "geometry_msgs/PoseStamped",
                }

                # 2. Publish
                publish_msg = {"op": "publish", "topic": topic, "msg": ros_msg}

                # rosbridge doesn't know the type if topic not yet published -> error
                # => advertise it once
                try:
                    if topic not in self.advertised_topics:
                        self.ws.send(json.dumps(advertise_msg))
                        self.advertised_topics.add(topic)
                except Exception as e:
                    self.get_logger().error(f"Advertising WebSocket send failed: {e}")

                try:
                    self.ws.send(json.dumps(publish_msg))
                except Exception as e:
                    self.get_logger().error(f"WebSocket send failed: {e}")

            if arm.state != ControlState.CONTROL:
                continue

            self.pose_pubs[side].publish(msg)

            self._publish_pose_markers(
                self._target_marker_pub, msg, self.robot_base_frame, f"target_{side}_"
            )

    def _publish_urdf_joints(self, stamp) -> None:
        """Republish current joint angles with URDF joint names for visualization."""
        names = []
        positions = []
        for side, arm in self.arms.items():
            if arm.current_joints is None:
                continue
            urdf_names = self._urdf_joint_names.get(side, [])
            names.extend(urdf_names)
            positions.extend(arm.current_joints.tolist())

        if not names:
            return

        msg = JointState()
        msg.header.stamp = stamp
        msg.name = names
        msg.position = positions
        self._urdf_joint_pub.publish(msg)

    def _publish_s570_fk_markers(self, stamp) -> None:
        """Publish FK end-effector markers for each arm (always, for S570 RViz)."""
        for side, arm in self.arms.items():
            if arm.current_joints is None:
                continue
            pos, quat = self.fk.compute(side, arm.current_joints)
            msg = PoseStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self.robot_base_frame
            msg.pose.position.x = float(pos[0])
            msg.pose.position.y = float(pos[1])
            msg.pose.position.z = float(pos[2])
            msg.pose.orientation.w = float(quat[0])
            msg.pose.orientation.x = float(quat[1])
            msg.pose.orientation.y = float(quat[2])
            msg.pose.orientation.z = float(quat[3])
            self._publish_pose_markers(
                self._s570_marker_pub, msg, self.robot_base_frame, f"fk_{side}_"
            )

    def _publish_pose_markers(
        self, publisher, pose_stamped: PoseStamped, frame: str, ns_prefix: str
    ) -> None:
        """Publish sphere + RGB axis arrows at a pose."""
        markers = MarkerArray()
        stamp = pose_stamped.header.stamp
        p = pose_stamped.pose.position
        q = pose_stamped.pose.orientation
        origin = np.array([p.x, p.y, p.z])
        R = _quat_to_rotation_matrix(np.array([q.w, q.x, q.y, q.z]))

        # Sphere
        sphere = Marker()
        sphere.header.frame_id = frame
        sphere.header.stamp = stamp
        sphere.ns = f"{ns_prefix}sphere"
        sphere.id = 1001
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose = pose_stamped.pose
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.025
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 0.0, 0.0, 1.0
        markers.markers.append(sphere)

        # RGB axis arrows
        axes = [
            (np.array([1, 0, 0]), (1.0, 0.0, 0.0), 1002),  # X red
            (np.array([0, 1, 0]), (0.0, 1.0, 0.0), 1003),  # Y green
            (np.array([0, 0, 1]), (0.0, 0.0, 1.0), 1004),  # Z blue
        ]
        for axis_vec, color, mid in axes:
            end = origin + R @ axis_vec * 0.1
            arrow = Marker()
            arrow.header.frame_id = frame
            arrow.header.stamp = stamp
            arrow.ns = f"{ns_prefix}axis_{mid}"
            arrow.id = mid
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x = 0.01
            arrow.scale.y = 0.02
            arrow.scale.z = 0.025
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = *color, 1.0
            arrow.points = [
                Point(x=origin[0], y=origin[1], z=origin[2]),
                Point(x=float(end[0]), y=float(end[1]), z=float(end[2])),
            ]
            markers.markers.append(arrow)

        publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ElephantS570Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
