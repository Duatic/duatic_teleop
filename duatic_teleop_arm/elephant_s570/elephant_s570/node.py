#!/usr/bin/env python3
"""ROS 2 Node for teleoperation using the Elephant Robotics MyController S570.

Subscribes to S570 joint states published via rosbridge by the Windows publisher,
computes FK-based relative Cartesian displacement, and publishes PoseStamped
targets for the IK solver (interactive_pyroki_node).

On deadman press: deactivates freeze_controller, activates joint_trajectory_controller.
On deadman release: deactivates JTC, activates freeze_controller.

Each arm is independently controlled via a deadman switch (Button A on each side).
Hold A to teleop, release to stop. Home pose is captured on each press of A.

Parameters:
    arm_side:           "left", "right", or "both" (default: "both")
    dual_arm_robot:     True if the robot has two arms (default: True)
    robot_home_pos_x/y/z:  Robot EE home position (default: 0.4, 0.0, 0.3)
    robot_home_quat_w/x/y/z: Robot EE home orientation (default: identity)

Button mapping (per S570 arm):
    A (hold):  Deadman switch — teleop active while held, home captured on press
    B, C, D:   Free (future: gripper, mode switch, etc.)
    Joystick:  Free (future: speed scaling, etc.)

Usage:
    ros2 run elephant_s570 elephant_s570_node
    ros2 run elephant_s570 elephant_s570_node --ros-args -p arm_side:=left
    ros2 run elephant_s570 elephant_s570_node --ros-args -p arm_side:=right -p dual_arm_robot:=false
"""

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

from duatic_dynaarm_extensions.duatic_helpers.duatic_controller_helper import (
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


class ArmPhase(Enum):
    WAITING = auto()  # No S570 data yet
    READY = auto()  # Data flowing, waiting for deadman press
    ACTIVE = auto()  # Deadman held, teleoping


class S570ArmState:
    """Tracking state for one S570 arm."""

    def __init__(self, side: str):
        self.side = side
        self.phase = ArmPhase.WAITING
        self.home_joints: np.ndarray | None = None
        self.current_joints: np.ndarray | None = None
        self.deadman_held = False
        # Robot EE pose captured on activation (from TF or parameter fallback)
        self.robot_home_pos: np.ndarray | None = None
        self.robot_home_quat: np.ndarray | None = None
        self.axis_lock_active = False
        self.axis_lock_quat: np.ndarray | None = None
        self.axis_lock_axis: np.ndarray | None = None  # z-axis in base frame
        self.prev_quat: np.ndarray | None = None
        self.stable_counter = 0


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


class ElephantS570Node(Node):
    def __init__(self):
        super().__init__("elephant_s570_node")

        # --- Parameters ---
        self.declare_parameter("arm_side", "both")
        self.declare_parameter("dual_arm_robot", True)
        self.declare_parameter("robot_home_pos_x", 0.4)
        self.declare_parameter("robot_home_pos_y", 0.0)
        self.declare_parameter("robot_home_pos_z", 0.3)
        self.declare_parameter("robot_home_quat_w", 1.0)
        self.declare_parameter("robot_home_quat_x", 0.0)
        self.declare_parameter("robot_home_quat_y", 0.0)
        self.declare_parameter("robot_home_quat_z", 0.0)
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("robot_ee_frame", "flange")
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

        self.robot_home_pos = np.array(
            [
                self.get_parameter("robot_home_pos_x").value,
                self.get_parameter("robot_home_pos_y").value,
                self.get_parameter("robot_home_pos_z").value,
            ]
        )
        self.robot_home_quat = np.array(
            [
                self.get_parameter("robot_home_quat_w").value,
                self.get_parameter("robot_home_quat_x").value,
                self.get_parameter("robot_home_quat_y").value,
                self.get_parameter("robot_home_quat_z").value,
            ]
        )
        self.robot_base_frame: str = self.get_parameter("robot_base_frame").value
        self.robot_ee_frame: str = self.get_parameter("robot_ee_frame").value

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
        self._deadman_button_index = {"left": 0, "right": 4}
        self._axis_lock_button_index = {"right": 5}

        # --- Target pose publishers ---
        base_topic = "/cartesian_pose_controller/target_pose"
        self.pose_pubs: dict[str, rclpy.publisher.Publisher] = {}

        if self.dual_arm_robot:
            for side in self._sides:
                topic = f"{base_topic}/{side}"
                self.pose_pubs[side] = self.create_publisher(PoseStamped, topic, 10)
                self.get_logger().info(f"Publishing to {topic}")
        else:
            self.pose_pubs[self._sides[0]] = self.create_publisher(PoseStamped, base_topic, 10)
            self.get_logger().info(f"Publishing to {base_topic}")

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
        self.teleop_method: str = 'remapping_deltas'

        self.visu_pose_pubs: dict[str, rclpy.publisher.Publisher] = {}

        if self.dual_arm_robot:
            for side in self._sides:
                topic = f"/visu/target_pose/{side}"
                self.visu_pose_pubs[side] = self.create_publisher(PoseStamped, topic, 10)
        else:
            self.visu_pose_pubs[self._sides[0]] = self.create_publisher(PoseStamped, "/visu/target_pose", 10)

        # --- Publish timer (50 Hz) ---
        self.create_timer(0.02, self._publish_targets)

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
        return any(arm.phase == ArmPhase.ACTIVE for arm in self.arms.values())

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
        """Split combined JointState (14 joints) into per-arm state."""
        joint_map = dict(zip(msg.name, msg.position))

        for side, arm in self.arms.items():
            prefix = f"s570_{side}/"
            joints = [joint_map.get(f"{prefix}joint_{i + 1}", 0.0) for i in range(7)]
            arm.current_joints = np.array(joints)

            if arm.phase == ArmPhase.WAITING:
                arm.phase = ArmPhase.READY
                self.get_logger().info(f"S570 {side} data received — READY. Hold A to teleop.")

    def _buttons_cb(self, msg: Joy) -> None:
        # Read the right deadman button (used for all arms since left buttons are broken)
        right_btn_idx = self._deadman_button_index["right"]
        right_pressed = (
            right_btn_idx < len(msg.buttons) and msg.buttons[right_btn_idx] == 0  # Active-low
        )

        # --- Axis lock button right ---
        axis_btn_idx = self._axis_lock_button_index["right"]
        axis_pressed = (
            axis_btn_idx < len(msg.buttons) and msg.buttons[axis_btn_idx] == 0
        )

        for side, arm in self.arms.items():
            # Left buttons are defective — use right button A for both arms
            if side == "left":
                is_held = right_pressed
            else:
                btn_idx = self._deadman_button_index[side]
                if btn_idx >= len(msg.buttons):
                    continue
                is_held = msg.buttons[btn_idx] == 0  # Active-low

            was_held = arm.deadman_held
            arm.deadman_held = is_held

            if is_held and not was_held:
                self._arm_activate(arm)
            elif not is_held and was_held:
                self._arm_deactivate(arm)

            if side == "right":
                if axis_pressed and not arm.axis_lock_active:
                    self._activate_axis_lock(arm)
                elif not axis_pressed and arm.axis_lock_active:
                    self._deactivate_axis_lock(arm)

    # ------------------------------------------------------------------ #
    #  Arm activation                                                     #
    # ------------------------------------------------------------------ #

    def _lookup_robot_ee_pose(self, side: str) -> tuple[np.ndarray, np.ndarray] | None:
        """Try to get the current DynaArm EE pose via TF."""
        # For dual-arm: arm_left/flange, arm_right/flange
        # For single-arm: flange
        if self.dual_arm_robot and self.arm_side == "both":
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

    def _arm_activate(self, arm: S570ArmState) -> None:
        if arm.phase == ArmPhase.WAITING:
            self.get_logger().warn(f"S570 {arm.side}: no data yet, cannot activate.")
            return

        arm.home_joints = arm.current_joints.copy()

        # Capture current DynaArm EE pose as robot home
        tf_result = self._lookup_robot_ee_pose(arm.side)
        if tf_result is not None:
            arm.robot_home_pos, arm.robot_home_quat = tf_result
            self.get_logger().info(
                f"S570 {arm.side} — DynaArm EE captured at "
                f"pos={arm.robot_home_pos.round(3).tolist()}, "
                f"quat={arm.robot_home_quat.round(3).tolist()}"
            )
        else:
            arm.robot_home_pos = self.robot_home_pos.copy()
            arm.robot_home_quat = self.robot_home_quat.copy()
            self.get_logger().warn(f"S570 {arm.side} — using parameter fallback for robot home")

        arm.phase = ArmPhase.ACTIVE
        self.get_logger().info(
            f"S570 {arm.side} ACTIVE — S570 home at "
            f"{np.degrees(arm.home_joints).round(1).tolist()}"
        )

        # Switch controllers on first arm activation
        self._activate_controllers()

    def _arm_deactivate(self, arm: S570ArmState) -> None:
        if arm.phase == ArmPhase.ACTIVE:
            arm.phase = ArmPhase.READY
            self.get_logger().info(f"S570 {arm.side} released — READY")

            # Only deactivate controllers when ALL arms are released
            if not self._any_arm_active():
                self._deactivate_controllers()
    
    def _activate_axis_lock(self, arm: S570ArmState) -> None:

        arm.home_joints = arm.current_joints.copy()
        
        # Capture current DynaArm EE pose as robot home
        tf_result = self._lookup_robot_ee_pose(arm.side)
        if tf_result is not None:
            arm.robot_home_pos, arm.robot_home_quat = tf_result
            self.get_logger().info(
                f"S570 {arm.side} — DynaArm EE captured at "
                f"pos={arm.robot_home_pos.round(3).tolist()}, "
                f"quat={arm.robot_home_quat.round(3).tolist()}"
            )
        else:
            arm.robot_home_pos = self.robot_home_pos.copy()
            arm.robot_home_quat = self.robot_home_quat.copy()
            self.get_logger().warn(f"S570 {arm.side} — using parameter fallback for robot home")
            self.get_logger().warn("Axis lock failed: no TF")
            return

        pos, quat = tf_result

        # 👉 Warten bis stabil
        if not self._is_orientation_stable(arm, quat):
            self.get_logger().info("Waiting for stable orientation...")
            return

        # 👉 Jetzt erst locken
        arm.axis_lock_quat = quat.copy()

        R = _quat_to_rotation_matrix(quat)
        z_axis = R[:, 2]

        arm.axis_lock_axis = z_axis / np.linalg.norm(z_axis)
        arm.axis_lock_active = True

        self._publish_axis_marker(arm, arm.axis_lock_axis)

        self.get_logger().info(
            f"[{arm.side}] Axis lock ACTIVATED — axis={arm.axis_lock_axis.round(3)}"
        )

        arm.phase = ArmPhase.ACTIVE

        # Switch controllers on first arm activation
        self._activate_controllers()
    
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
        tf_result = self._lookup_robot_ee_pose(arm.side)
        if tf_result is None:
            return

        pos, _ = tf_result

        marker = Marker()
        marker.header.frame_id = "base_link"
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
        arm.axis_lock_quat = None

        self.get_logger().info(f"[{arm.side}] Axis lock DEACTIVATED")

        arm.phase = ArmPhase.READY

        # Only deactivate controllers when ALL arms are released
        if not self._any_arm_active():
            self._deactivate_controllers()
    
    def _project_motion_to_axis(
        self,
        delta_pos: np.ndarray,
        axis: np.ndarray
    ) -> np.ndarray:
        """Project arbitrary motion onto a given axis."""
        return np.dot(delta_pos, axis) * axis

    # ------------------------------------------------------------------ #
    #  FK-based target computation                                        #
    # ------------------------------------------------------------------ #

    def _compute_target_delta_method(self, arm: S570ArmState) -> tuple[np.ndarray, np.ndarray] | None:
        """Compute Cartesian target from S570 FK relative displacement.

        Returns (position[3], quaternion_wxyz[4]) or None.
        """
        if arm.home_joints is None or arm.current_joints is None or arm.robot_home_pos is None:
            return None

        # Relative EE displacement in base frame
        delta_pos, delta_quat = self.fk.compute_relative(
            arm.side, arm.home_joints, arm.current_joints
        )

        # Apply delta to the captured DynaArm EE pose
        target_pos = arm.robot_home_pos + delta_pos

        # Orientation: delta is in base frame, so apply as delta * home
        target_quat = _quat_multiply(delta_quat, arm.robot_home_quat)
        target_quat /= np.linalg.norm(target_quat)

        return target_pos, target_quat

    def _compute_target_actionspace_method(self, arm: S570ArmState, side: str) -> tuple[np.ndarray, np.ndarray] | None:
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
            teleop_pos[1] = - teleop_pos[1]

        # --- Cap teleop input if it exceeds limits ---
        teleop_pos = np.clip(teleop_pos, t_min, t_max)

        # --- Normalize to [0,1] ---
        norm = (teleop_pos - t_min) / (t_max - t_min)

        # --- Scale to robot workspace ---
        target_pos = r_min + norm * (r_max - r_min)

        # --- Mirror Y for right arm ---
        if side == "right":
            target_pos[1] = - target_pos[1]

        # --- Orientation (unchanged) ---
        target_quat = teleop_quat.copy()
        target_quat /= np.linalg.norm(target_quat)

        return target_pos, target_quat

    def _compute_target_lock_axis(
        self,
        arm: S570ArmState,
        side: str
    ) -> tuple[np.ndarray, np.ndarray] | None:
        """Move only along locked EE axis (z-axis of flange), orientation fixed."""

        if (
            arm.current_joints is None
            or arm.home_joints is None
            or arm.robot_home_pos is None
            or arm.axis_lock_axis is None
            or arm.axis_lock_quat is None
        ):
            return None

        # --- FK current + home ---
        current_pos, _ = self.fk.compute(side, arm.current_joints)
        home_pos, _ = self.fk.compute(side, arm.home_joints)

        # --- Delta in teleop space ---
        delta_pos = current_pos - home_pos

        # --- Optional: nur X-Achse vom Teleop verwenden ---
        # (vor/zurück Bewegung isolieren)
        delta_x = delta_pos[0]

        axis = arm.axis_lock_axis
        projected = delta_x * axis

        print("axis:", axis)
        print("delta_x:", delta_x)
        print("projected:", projected)
        cos_angle = np.dot(projected, axis) / np.linalg.norm(projected)
        print("alignment:", cos_angle)

        print("robot_home_pos:", arm.robot_home_pos)
        print("axis_lock_axis:", arm.axis_lock_axis)

        target_pos = arm.robot_home_pos + projected

        # --- Orientierung bleibt fix ---
        target_quat = arm.axis_lock_quat.copy()

        return target_pos, target_quat

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
            if arm.axis_lock_active:
                result = self._compute_target_lock_axis(arm, side)
            else:
                if self.teleop_method == 'remapping_deltas':
                    result = self._compute_target_delta_method(arm)
                elif self.teleop_method == 'mapped_actionspaces':
                    result = self._compute_target_actionspace_method(arm, side)
                else:
                    result = self._compute_target_actionspace_method(arm, side)

            if result is None:
                continue

            pos, quat = result

            msg = PoseStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = "base_link"
            msg.pose.position.x = float(pos[0])
            msg.pose.position.y = float(pos[1])
            msg.pose.position.z = float(pos[2])
            msg.pose.orientation.w = float(quat[0])
            msg.pose.orientation.x = float(quat[1])
            msg.pose.orientation.y = float(quat[2])
            msg.pose.orientation.z = float(quat[3])

            if side in self.visu_pose_pubs:
                self.visu_pose_pubs[side].publish(msg)

            topic = self.visu_pose_pubs[side].topic_name

            if self.use_rosbridge:
                ros_msg = {
                    "header": {
                        "stamp": {
                            "sec": int(stamp.sec),
                            "nanosec": int(stamp.nanosec)
                        },
                        "frame_id": "base_link"
                    },
                    "pose": {
                        "position": {
                            "x": float(pos[0]),
                            "y": float(pos[1]),
                            "z": float(pos[2])
                        },
                        "orientation": {
                            "w": float(quat[0]),
                            "x": float(quat[1]),
                            "y": float(quat[2]),
                            "z": float(quat[3])
                        }
                    }
                }

                # 1. Advertise
                advertise_msg = {
                    "op": "advertise",
                    "topic": topic,
                    "type": "geometry_msgs/PoseStamped"
                }

                # 2. Publish
                publish_msg = {
                    "op": "publish",
                    "topic": topic,
                    "msg": ros_msg
                }

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

            if arm.phase != ArmPhase.ACTIVE:
                continue

            self.pose_pubs[side].publish(msg)

            self._publish_pose_markers(
                self._target_marker_pub, msg, "base_link", f"target_{side}_"
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
            msg.header.frame_id = "base_link"
            msg.pose.position.x = float(pos[0])
            msg.pose.position.y = float(pos[1])
            msg.pose.position.z = float(pos[2])
            msg.pose.orientation.w = float(quat[0])
            msg.pose.orientation.x = float(quat[1])
            msg.pose.orientation.y = float(quat[2])
            msg.pose.orientation.z = float(quat[3])
            self._publish_pose_markers(self._s570_marker_pub, msg, "base_link", f"fk_{side}_")

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
