#!/usr/bin/env python3
"""ROS 2 Node for teleoperation using the Elephant Robotics MyController S570.

Each arm: REPOSITION (live reference poses) <-> CONTROL (deadman-held; frozen poses; the
live-vs-frozen twist, scaled, applied onto the frozen robot pose; right arm's axis-lock
additionally freezes orientation and restricts motion to the robot EE's local Z axis).
Button press/release swaps freeze_controller with any auto_controller_activation match.

Parameters:
    robot_base_frame:            TF frame reference poses are tracked relative to.
    auto_controller_activation:  Controller filter auto-switched on button press
                                  (default: ["joint_trajectory_controller"]).
    linear_acceleration:         Gain on linear motion (default: 1.0).
    angular_acceleration:        Gain on angular motion (default: 1.0).
    button_mirror_mode:          ButtonMirrorMode value: "independent" (default), or
                                  "mirror_right_to_left"/"mirror_left_to_right" to duplicate
                                  one side's raw buttons+axes onto the other.
    topics_prefix:               Prepended to pose_topics/target_topics/visu topics.
    pose_topics/pose_tf/target_topics:
                                  Index-matched left/right; each side tracks pose_topics[i]
                                  if set, else TF pose_tf[i], else is disabled; publishes
                                  to target_topics[i]. Defaults reproduce the original
                                  dual-arm/TF setup.

Buttons: A (hold) = deadman; right side also has an axis-lock button.
Usage: ros2 run elephant_s570 elephant_s570_node [--ros-args -p pose_tf:="['', 'flange']"]
"""

from dataclasses import dataclass
from enum import Enum, auto
from functools import partial

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
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

# Freeze controller name, always switched opposite to 'auto_controller_activation' (see
# _activate_controllers()/_deactivate_controllers()). Not itself configurable.
FREEZE_CONTROLLER_NAME = "freeze_controller"

# Fixed side order 'pose_topics'/'pose_tf'/'target_topics' are index-matched against.
SIDES = ("left", "right")

# TF-mode poll rate for _tf_poll_cb(), decoupled from the joint_states callback's own rate.
TF_POLL_PERIOD_SEC = 0.05


def _quat_conjugate(q_wxyz: np.ndarray) -> np.ndarray:
    """Conjugate (= inverse, for a unit quaternion) of [w, x, y, z]."""
    w, x, y, z = q_wxyz
    return np.array([w, -x, -y, -z])


def _quat_to_rotvec(q_wxyz: np.ndarray) -> np.ndarray:
    """Unit quaternion [w, x, y, z] -> rotation vector (axis * angle), directly —
    no rotation matrix round-trip needed."""
    w, x, y, z = q_wxyz
    v = np.array([x, y, z])
    v_norm = np.linalg.norm(v)
    if v_norm < 1e-8:
        return np.zeros(3)
    angle = 2.0 * np.arctan2(v_norm, w)
    return (v / v_norm) * angle


def _quat_rotate_vector(q_wxyz: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate 3-vector v by unit quaternion [w, x, y, z] (no rotation matrix needed)."""
    w, x, y, z = q_wxyz
    q_vec = np.array([x, y, z])
    t = 2.0 * np.cross(q_vec, v)
    return v + w * t + np.cross(q_vec, t)


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


class ButtonMirrorMode(Enum):
    """How _buttons_cb() combines the left/right raw button+axis values before dispatching
    to each side — see the 'button_mirror_mode' parameter in the module docstring."""

    INDEPENDENT = "independent"
    MIRROR_RIGHT_TO_LEFT = "mirror_right_to_left"
    MIRROR_LEFT_TO_RIGHT = "mirror_left_to_right"


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
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("linear_acceleration", 1.0)
        self.declare_parameter("angular_acceleration", 1.0)
        # Controller(s) auto-activated/deactivated on button press — see module docstring.
        self.declare_parameter("auto_controller_activation", ["joint_trajectory_controller"])
        # One of ButtonMirrorMode's values — see module docstring.
        self.declare_parameter("button_mirror_mode", ButtonMirrorMode.INDEPENDENT.value)
        # Index-matched (index 0 = left, index 1 = right) — see module docstring.
        self.declare_parameter("topics_prefix", "/cartesian_pose_controller/target_pose")
        self.declare_parameter("pose_topics", ["", ""])
        self.declare_parameter("pose_tf", ["arm_left/flange", "arm_right/flange"])
        self.declare_parameter("target_topics", ["/left", "/right"])
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

        self.robot_base_frame: str = self.get_parameter("robot_base_frame").value
        self.linear_acceleration: float = float(self.get_parameter("linear_acceleration").value)
        self.angular_acceleration: float = float(self.get_parameter("angular_acceleration").value)
        self.auto_controller_activation_names: list[str] = self._get_string_array_param(
            "auto_controller_activation"
        )
        self.get_logger().info(
            f"'{FREEZE_CONTROLLER_NAME}' is toggled on every button press/release, "
            "regardless of auto_controller_activation."
        )
        if self.auto_controller_activation_names:
            self.get_logger().info(
                "Controller auto-activation ENABLED — additionally switching controllers "
                f"matching {self.auto_controller_activation_names} on button press/release."
            )
        else:
            self.get_logger().info(
                "Controller auto-activation DISABLED (auto_controller_activation is empty) — "
                f"only '{FREEZE_CONTROLLER_NAME}' will be toggled on button press/release."
            )

        # --- Resolve pose_topics/pose_tf/target_topics: each enabled side ends up tracked via
        # exactly one of arm_tf_frame or arm_ee_pose_topic — see module docstring. ---
        self.topics_prefix: str = self.get_parameter("topics_prefix").value
        pose_topics = self._get_string_array_param("pose_topics")
        pose_tf = self._get_string_array_param("pose_tf")
        target_topics = self._get_string_array_param("target_topics")
        # Pad a shorter-than-SIDES override with "" ("unused at this index").
        pose_topics += [""] * (len(SIDES) - len(pose_topics))
        pose_tf += [""] * (len(SIDES) - len(pose_tf))
        target_topics += [""] * (len(SIDES) - len(target_topics))

        self.arm_tf_frame: dict[str, str] = {}
        self.arm_ee_pose_topic: dict[str, str] = {}
        self.arm_target_topic: dict[str, str] = {}
        # Frame each side's target is published in — fixed at robot_base_frame for TF-mode
        # sides, kept in sync with the incoming topic's frame_id by _ee_pose_cb() otherwise.
        self.arm_pose_frame_id: dict[str, str] = {}
        self._sides: list[str] = []
        for i, side in enumerate(SIDES):
            pose_topic = f"{self.topics_prefix}{pose_topics[i]}" if pose_topics[i] else ""
            tf_frame = pose_tf[i]
            is_tf = not pose_topic
            source = tf_frame if is_tf else pose_topic

            if not source:
                self.get_logger().info(
                    f"S570 {side} arm DISABLED (neither pose_topics[{i}] nor pose_tf[{i}] is "
                    "set)."
                )
                continue

            self._sides.append(side)
            self.arm_target_topic[side] = f"{self.topics_prefix}{target_topics[i]}"
            self.arm_pose_frame_id[side] = self.robot_base_frame
            if is_tf:
                self.arm_tf_frame[side] = tf_frame
                self.get_logger().info(
                    f"S570 {side} arm ENABLED — tracking tf '{tf_frame}', publishing target "
                    f"to '{self.arm_target_topic[side]}'"
                )
            else:
                self.arm_ee_pose_topic[side] = pose_topic
                self.get_logger().info(
                    f"S570 {side} arm ENABLED — tracking pose topic '{pose_topic}', "
                    f"publishing target to '{self.arm_target_topic[side]}'"
                )

        left_topic = self.arm_target_topic.get("left")
        right_topic = self.arm_target_topic.get("right")
        if left_topic is not None and left_topic == right_topic:
            self.get_logger().error(
                f"left and right target_topics both resolve to '{left_topic}' — disabling "
                "the right arm."
            )
            self._sides.remove("right")
            self.arm_target_topic.pop("right", None)
            self.arm_pose_frame_id.pop("right", None)
            self.arm_tf_frame.pop("right", None)
            self.arm_ee_pose_topic.pop("right", None)

        if not self._sides:
            self.get_logger().warn(
                "No arms enabled — pose_topics and pose_tf are empty for both sides."
            )

        # --- Controller helper ---
        # Created eagerly (not lazily on first deadman press) so its 0.1s discovery timer has
        # time to populate before the first button press — otherwise that press could race
        # discovery and get reported as "not found" in _activate_controllers(). Always
        # created: freeze_controller is toggled on every press/release regardless of
        # auto_controller_activation.
        self.controller_helper = DuaticControllerHelper(self)
        self.get_logger().info("Controller helper initialized.")
        self._controllers_switched = False

        # --- TF listener: EE pose lookup for TF-mode sides (_tf_poll_cb()), and/or the
        # cross-frame self-collision check once both arms are active (_transform_position()) ---
        if self.arm_tf_frame or len(self._sides) >= 2:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        if self.arm_tf_frame:
            self.create_timer(TF_POLL_PERIOD_SEC, self._tf_poll_cb)

        # --- S570 FK ---
        self.fk = S570FK()
        self.get_logger().info("S570 FK model loaded from URDF")

        # --- Per-arm state ---
        self.arms: dict[str, S570ArmState] = {}

        for side in self._sides:
            self.arms[side] = S570ArmState(side)

        # --- Pose-topic subscriptions for sides in pose-topic mode (see _ee_pose_cb()) ---
        for side, topic in self.arm_ee_pose_topic.items():
            self.create_subscription(PoseStamped, topic, partial(self._ee_pose_cb, side), 10)

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
        # button_mirror_mode controls how _buttons_cb() combines these before dispatching to
        # each side — see ButtonMirrorMode / module docstring.
        try:
            self.button_mirror_mode = ButtonMirrorMode(
                self.get_parameter("button_mirror_mode").value
            )
        except ValueError:
            invalid = self.get_parameter("button_mirror_mode").value
            valid = [m.value for m in ButtonMirrorMode]
            self.button_mirror_mode = ButtonMirrorMode.INDEPENDENT
            self.get_logger().error(
                f"Invalid button_mirror_mode '{invalid}' (must be one of {valid}); "
                f"defaulting to '{self.button_mirror_mode.value}'."
            )
        self._deadman_button_index = {"left": 0, "right": 4, "grasp": 5}
        self._axis_lock_button_index = {"right": 5}
        self._joy_axes: list[float] = []

        # --- Target pose publishers ---
        self.pose_pubs: dict[str, rclpy.publisher.Publisher] = {}

        for side in self._sides:
            self.pose_pubs[side] = self.create_publisher(
                PoseStamped, self.arm_target_topic[side], 10
            )

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

        for side in self._sides:
            topic = f"/visu/target_pose/{side}"
            self.visu_pose_pubs[side] = self.create_publisher(PoseStamped, topic, 10)

        # --- Publish timer (50 Hz) ---
        self.create_timer(0.02, self._publish_targets)

        self.get_logger().info("Elephant S570 node ready.")
        self.get_logger().info("Hold Button A on S570 to start teleop.")

        self.axis_pub = self.create_publisher(Marker, "/axis_lock_marker", 10)

    def _get_string_array_param(self, name: str) -> list[str]:
        """Like list(self.get_parameter(name).value), but tolerates a "-p name:=[]" override:
        rcl can't infer STRING_ARRAY from zero elements, so it comes through as NOT_SET and
        plain get_parameter() would raise ParameterUninitializedException even though this
        parameter has a non-empty default. An explicitly-typed empty-array fallback via
        get_parameter_or() resolves it correctly instead."""
        empty = Parameter(name, Parameter.Type.STRING_ARRAY, [])
        return list(self.get_parameter_or(name, empty).value)

    # ------------------------------------------------------------------ #
    #  Controller switching                                              #
    # ------------------------------------------------------------------ #

    def _any_arm_active(self) -> bool:
        return any(arm.state == ControlState.CONTROL for arm in self.arms.values())

    def _both_arms_active(self) -> bool:
        """True only when every configured arm is simultaneously in CONTROL — the only case
        self-collision avoidance in _compute_target_control() is meaningful for."""
        return len(self.arms) >= 2 and all(
            a.state == ControlState.CONTROL for a in self.arms.values()
        )

    def _get_auto_activate_names(self) -> list[str]:
        """Controllers matching auto_controller_activation, or [] if that parameter is
        empty (avoids get_all_controllers(matching_names=[]), which returns everything
        unfiltered rather than nothing)."""
        if not self.auto_controller_activation_names:
            return []
        return self.controller_helper.get_all_controllers(
            matching_names=self.auto_controller_activation_names
        )

    def _activate_controllers(self) -> None:
        """Deactivate freeze_controller (always) and activate the auto_controller_activation
        match(es), if any, for teleop.

        Leaves _controllers_switched False (so a later call can retry) if discovery isn't
        ready yet.
        """
        if self._controllers_switched:
            return

        if not self.controller_helper.is_controller_data_ready():
            self.get_logger().warn(
                "Controller manager data not ready yet — skipping controller switch for now."
            )
            return

        freeze_names = self.controller_helper.get_all_controllers(
            matching_names=[FREEZE_CONTROLLER_NAME]
        )
        auto_activate_names = self._get_auto_activate_names()
        if self.auto_controller_activation_names and not auto_activate_names:
            self.get_logger().warn(
                f"No controller matching {self.auto_controller_activation_names} found!"
            )

        self.controller_helper.switch_controller(
            activate_controllers=auto_activate_names,
            deactivate_controllers=freeze_names,
        )
        self.get_logger().info(
            f"Controllers: activating {auto_activate_names}, deactivating {freeze_names}"
        )
        self._controllers_switched = True

    def _deactivate_controllers(self) -> None:
        """Activate freeze_controller (always) and deactivate the auto_controller_activation
        match(es), if any."""
        if not self._controllers_switched:
            return

        freeze_names = self.controller_helper.get_all_controllers(
            matching_names=[FREEZE_CONTROLLER_NAME]
        )
        auto_activate_names = self._get_auto_activate_names()

        self.controller_helper.switch_controller(
            activate_controllers=freeze_names,
            deactivate_controllers=auto_activate_names,
        )
        self.get_logger().info(
            f"Controllers: activating {freeze_names}, deactivating {auto_activate_names}"
        )
        self._controllers_switched = False

    # ------------------------------------------------------------------ #
    #  Callbacks                                                          #
    # ------------------------------------------------------------------ #

    def _joint_state_cb(self, msg: JointState) -> None:
        """Split combined JointState (14 joints) into per-arm state, and — while REPOSITION —
        keep the arm-side reference pose live. The robot-side reference pose is refreshed
        independently, by _tf_poll_cb() or _ee_pose_cb()."""
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
        """While REPOSITION: continuously refresh the arm-side reference pose from live
        joint data."""
        pos, quat = self.fk.compute(side, arm.current_joints)
        arm.arm_reference_pose = Pose6D(pos, quat)

    def _tf_poll_cb(self) -> None:
        """Periodically refresh the robot-side reference pose for every TF-mode side, while
        REPOSITION (frozen — like the pose-topic path below — while CONTROL)."""
        for side in self.arm_tf_frame:
            arm = self.arms[side]
            if arm.state != ControlState.REPOSITION:
                continue
            result = self._lookup_robot_ee_pose(side)
            if result is not None:
                pos, quat = result
                arm.robot_reference_pose = Pose6D(pos, quat)
            # else: TF failed this tick. robot_reference_pose is left as-is — None until the
            # first successful lookup (on_control_activate() correctly refuses to activate
            # until then), or its last successfully tracked value otherwise.

    def _ee_pose_cb(self, side: str, msg: PoseStamped) -> None:
        """Pose-topic mode: refresh the robot-side reference pose from the latest message,
        used exactly as received (no TF transform — its frame is remembered in
        arm_pose_frame_id and published back unchanged, see module docstring), while
        REPOSITION (frozen — like the TF path above — while CONTROL)."""
        arm = self.arms[side]
        if arm.state != ControlState.REPOSITION:
            return

        self.arm_pose_frame_id[side] = msg.header.frame_id or self.robot_base_frame
        p, o = msg.pose.position, msg.pose.orientation
        arm.robot_reference_pose = Pose6D(np.array([p.x, p.y, p.z]), np.array([o.w, o.x, o.y, o.z]))

    def _buttons_cb(self, msg: Joy) -> None:
        buttons = list(msg.buttons)
        axes = list(msg.axes)
        if self.button_mirror_mode == ButtonMirrorMode.MIRROR_RIGHT_TO_LEFT:
            if len(buttons) >= 8:
                buttons[0:4] = buttons[4:8]
            if len(axes) >= 4:
                axes[0:2] = axes[2:4]
        elif self.button_mirror_mode == ButtonMirrorMode.MIRROR_LEFT_TO_RIGHT:
            if len(buttons) >= 8:
                buttons[4:8] = buttons[0:4]
            if len(axes) >= 4:
                axes[2:4] = axes[0:2]
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
        ee_frame = self.arm_tf_frame[side]

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

    def _transform_position(
        self, position: np.ndarray, from_frame: str, to_frame: str
    ) -> np.ndarray | None:
        """Transform a 3D point from from_frame into to_frame, for comparing two arms'
        reference poses in self-collision avoidance even when tracked in different frames.
        Returns `position` unchanged if the frames are the same name, or None if the TF
        lookup between them fails."""
        if from_frame == to_frame:
            return position

        try:
            t = self.tf_buffer.lookup_transform(to_frame, from_frame, Time())
        except Exception as e:
            self.get_logger().warn(
                f"Self-collision check: TF lookup {from_frame} → {to_frame} failed: {e}",
                throttle_duration_sec=5.0,
            )
            return None

        translation = np.array(
            [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        )
        rotation = np.array(
            [
                t.transform.rotation.w,
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
            ]
        )
        return _quat_rotate_vector(rotation, position) + translation

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

        # Wait until stable before locking.
        if not self._is_orientation_stable(arm, arm.robot_reference_pose.quat):
            self.get_logger().info("Waiting for stable orientation...", throttle_duration_sec=1.0)
            return

        # Reuse the deadman's activation if it hasn't happened yet; else just add the lock.
        if arm.state != ControlState.CONTROL:
            if not self.on_control_activate(arm):
                return

        z_axis = _quat_rotate_vector(arm.robot_reference_pose.quat, np.array([0.0, 0.0, 1.0]))
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
        marker.header.frame_id = self.arm_pose_frame_id.get(arm.side, self.robot_base_frame)
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
        arm.prev_quat = None
        arm.stable_counter = 0

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
        relative_quat = _quat_multiply(actual_quat, _quat_conjugate(arm.arm_reference_pose.quat))
        angular_diff = _quat_to_rotvec(relative_quat)

        # --- Scale ---
        linear_diff = linear_diff * self.linear_acceleration
        angular_diff = angular_diff * self.angular_acceleration

        if arm.axis_lock_active:
            # Only allow motion along the locked axis; orientation stays exactly at reference.
            linear_diff = np.dot(linear_diff, arm.axis_lock_axis) * arm.axis_lock_axis
            angular_diff = np.zeros(3)

        # --- Add onto the frozen robot reference pose ---
        target_pos = arm.robot_reference_pose.position + linear_diff

        self.get_logger().info(
            f"{arm.side} — target pos={target_pos.round(3).tolist()}", throttle_duration_sec=0.5
        )

        # Self collision avoidance — geometry assumptions tuned for a dual-arm robot with
        # both arms active, so skip entirely otherwise.
        if self._both_arms_active():
            # End effectors cannot drive into base
            if target_pos[0] <= 0.76:
                target_pos[0] = 0.76

            # End effectors cannot get too close to each other.
            # Minimum allowed safe distance between end effectors
            safe_distance = 0.4
            is_safe = True
            frame_id = self.arm_pose_frame_id.get(side, self.robot_base_frame)

            for other_side, other_arm in self.arms.items():

                # Skip current arm (and any arm without a reference pose yet)
                if other_side == side or other_arm.robot_reference_pose is None:
                    continue

                # The other arm may be tracked in a different frame — bring it into this
                # arm's own frame before comparing positions.
                other_frame_id = self.arm_pose_frame_id.get(other_side, self.robot_base_frame)
                other_pos = self._transform_position(
                    other_arm.robot_reference_pose.position, other_frame_id, frame_id
                )
                if other_pos is None:
                    continue  # TF unavailable this tick — skip rather than risk a false trip

                # 3D Euclidean distance
                old_distance = np.linalg.norm(arm.robot_reference_pose.position - other_pos)
                new_distance = np.linalg.norm(target_pos - other_pos)
                self.get_logger().info(
                    f"{arm.side} — other pos={other_pos.round(3).tolist()}, "
                    f"resulting distance={new_distance.round(3)}",
                    throttle_duration_sec=0.5,
                )

                if new_distance < safe_distance:
                    if new_distance < old_distance:
                        self.get_logger().error(
                            "Too close, movement not allowed", throttle_duration_sec=0.5
                        )
                        # A violation from any other arm blocks this cycle; don't let a later,
                        # unrelated arm's "safe" verdict clear it.
                        is_safe = False
                    else:
                        self.get_logger().error(
                            "Already too close, but moving away from each other allowed",
                            throttle_duration_sec=0.5,
                        )

            # Only apply movement if safe. No previous target yet (e.g. first cycle after
            # activation) means there's nothing safe to fall back to — use the computed
            # target rather than crash downstream on a None position.
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
        # Axis correspondence: teleop x -> robot z, teleop z -> robot -x, teleop y unchanged.
        # That's a fixed -90 degree rotation about the (shared) Y axis.
        q_offset = _rotvec_to_quat(np.array([0.0, -np.pi / 2, 0.0]))

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
            # robot_base_frame for TF mode; the subscribed topic's own frame for pose-topic
            # mode (see _ee_pose_cb()).
            frame_id = self.arm_pose_frame_id.get(side, self.robot_base_frame)

            msg = self._make_pose_msg(stamp, frame_id, pos, quat)

            if side in self.visu_pose_pubs:
                self.visu_pose_pubs[side].publish(msg)

            topic = self.pose_pubs[side].topic_name

            if self.use_rosbridge:
                ros_msg = {
                    "header": {
                        "stamp": {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)},
                        "frame_id": frame_id,
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

            self._publish_pose_markers(self._target_marker_pub, msg, frame_id, f"target_{side}_")

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
            msg = self._make_pose_msg(stamp, self.robot_base_frame, pos, quat)
            self._publish_pose_markers(
                self._s570_marker_pub, msg, self.robot_base_frame, f"fk_{side}_"
            )

    @staticmethod
    def _make_pose_msg(stamp, frame_id: str, pos: np.ndarray, quat: np.ndarray) -> PoseStamped:
        """Build a PoseStamped from a position + quaternion [w, x, y, z] array pair."""
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.w = float(quat[0])
        msg.pose.orientation.x = float(quat[1])
        msg.pose.orientation.y = float(quat[2])
        msg.pose.orientation.z = float(quat[3])
        return msg

    def _publish_pose_markers(
        self, publisher, pose_stamped: PoseStamped, frame: str, ns_prefix: str
    ) -> None:
        """Publish sphere + RGB axis arrows at a pose."""
        markers = MarkerArray()
        stamp = pose_stamped.header.stamp
        p = pose_stamped.pose.position
        q = pose_stamped.pose.orientation
        origin = np.array([p.x, p.y, p.z])
        quat = np.array([q.w, q.x, q.y, q.z])

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
            end = origin + _quat_rotate_vector(quat, axis_vec) * 0.1
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
