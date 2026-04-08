#!/usr/bin/env python3
"""
ROS 2 Node for Cartesian control of DynaArm systems using PyRoki.
Supports single-arm, dual-arm, and dual-arm + hip (DuaTorso) configurations.
Robot structure is auto-detected via DuaticRobotsHelper.

Solve modes (via 'solve_mode' parameter):
  - "decoupled"          : Each arm solved independently, hip locked (default)
  - "decoupled_with_hip" : Each arm solved independently, hip joints follow arms
  - "whole_body"         : All arms + hip solved together in one optimization

Input modes (via 'use_interactive_markers' parameter):
  - True  (default): 6-DOF Interactive Markers in RViz
  - False           : PoseStamped on target_pose topics
"""

import time
import threading
from dataclasses import dataclass, field
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

from duatic_kinematics.pyroki_solver import PyrokiIKSolver
from duatic_kinematics.smoothing import smooth_and_limit
from duatic_dynaarm_extensions.duatic_helpers.duatic_robots_helper import DuaticRobotsHelper

from jaxlie import SE3


@dataclass
class ArmState:
    """Per-arm target, mask, and smoothing state."""

    name: str  # component name: "arm_left", "arm_right", "" (single)
    target_link: str  # e.g. "arm_left/flange", "flange"
    target_pos: np.ndarray = None
    target_wxyz: np.ndarray = None
    marker_name: str = ""
    marker_color: tuple = (1.0, 0.1, 0.1, 0.5)  # RGBA
    joint_mask: np.ndarray = None  # (n_actuated,) 1.0=optimize, 0.0=lock
    joint_indices: list = field(default_factory=list)  # indices of this arm's joints in full cfg
    smoothed_arm_q: np.ndarray = None  # smoothed values for this arm's joints only
    last_smoothed_arm_q: np.ndarray = None


# Marker colors per arm
ARM_COLORS = {
    "": (1.0, 0.1, 0.1, 0.5),  # single arm: red
    "arm_left": (0.1, 0.3, 1.0, 0.5),  # left: blue
    "arm_right": (0.1, 1.0, 0.3, 0.5),  # right: green
}

VALID_SOLVE_MODES = ("decoupled", "decoupled_with_hip", "whole_body")


class InteractivePyrokiNode(Node):
    def __init__(self):
        super().__init__("interactive_pyroki_node")

        self.declare_parameter("target_link_name", "flange")
        self.declare_parameter("use_interactive_markers", True)
        self.declare_parameter("solve_mode", "decoupled")

        self.target_link_name = self.get_parameter("target_link_name").value
        self.use_interactive_markers = self.get_parameter("use_interactive_markers").value
        self.solve_mode = self.get_parameter("solve_mode").value

        if self.solve_mode not in VALID_SOLVE_MODES:
            self.get_logger().warn(
                f"Invalid solve_mode '{self.solve_mode}', falling back to 'decoupled'. "
                f"Valid: {VALID_SOLVE_MODES}"
            )
            self.solve_mode = "decoupled"

        self.get_logger().info(f"Solve mode: {self.solve_mode}")

        # Robot helper for auto-detection
        self.robots_helper = DuaticRobotsHelper(self)

        # Single shared solver — initialized after URDF + robot detection
        self.solver: PyrokiIKSolver | None = None

        self.arm_states: list[ArmState] = []
        self.current_q = None
        self.joint_names = []
        self.robot_structure = None  # set after detection

        # Full-body state for velocity computation and whole_body smoothing
        self.last_full_q = None
        self.last_time = None
        self.smoothed_q = None  # used by whole_body mode
        self.last_smoothed_q = None  # used by whole_body mode

        self.alpha_filter = 0.15
        self.max_joint_velocity = 0.5  # rad/s

        self._state_lock = threading.Lock()
        self.fully_initialized = False
        self._urdf_data = None

        # Publisher map: component_name -> publisher
        self.traj_publishers: dict[str, tuple] = {}

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.desc_sub = self.create_subscription(
            String, "/robot_description", self.description_cb, qos
        )
        self.state_sub = self.create_subscription(JointState, "/joint_states", self.state_cb, 10)

        if self.use_interactive_markers:
            self.server = InteractiveMarkerServer(self, "pyroki_target")
            self.get_logger().info("Mode: Interactive Marker (RViz)")
        else:
            self.server = None
            self.get_logger().info("Mode: PoseStamped topic")

        # Start robot detection + control in background thread
        self.control_thread = threading.Thread(target=self._init_and_run, daemon=True)
        self.control_thread.start()

    def _init_and_run(self):
        """Wait for robot detection, then run main control loop."""
        self.get_logger().info("Waiting for robot detection...")
        self.robots_helper.wait_for_robot()

        self.robot_structure = self.robots_helper.robot_structure
        arm_names = self.robots_helper.get_component_names("arm")
        hip_names = self.robots_helper.get_component_names("hip")

        self.get_logger().info(
            f"Detected: structure={self.robot_structure}, arms={arm_names}, hip={hip_names}"
        )

        # Build arm states
        if self.robot_structure == "single_arm":
            self.arm_states = [
                ArmState(
                    name="",
                    target_link=self.target_link_name,
                    marker_name="pyroki_target",
                    marker_color=ARM_COLORS[""],
                )
            ]
        else:
            # multi_arm / mobile_manipulator
            for arm_name in arm_names:
                self.arm_states.append(
                    ArmState(
                        name=arm_name,
                        target_link=f"{arm_name}/flange",
                        marker_name=f"pyroki_target_{arm_name.replace('arm_', '')}",
                        marker_color=ARM_COLORS.get(arm_name, (0.5, 0.5, 0.5, 0.5)),
                    )
                )

        # Setup PoseStamped subscribers (non-interactive mode)
        if not self.use_interactive_markers:
            self._setup_pose_subscribers()

        # Setup trajectory publishers
        self._setup_publishers(arm_names, hip_names)

        # Wait for URDF and solver initialization
        while self._urdf_data is None and rclpy.ok():
            time.sleep(0.1)

        # Now run the main control loop
        self.main_loop()

    def _setup_publishers(self, arm_names, hip_names):
        """Create JointTrajectory publishers based on detected components."""
        if self.robot_structure == "single_arm":
            self.traj_publishers[""] = self.create_publisher(
                JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
            )
        else:
            for arm_name in arm_names:
                topic = f"/joint_trajectory_controller_{arm_name}/joint_trajectory"
                self.traj_publishers[arm_name] = self.create_publisher(JointTrajectory, topic, 10)

            if hip_names:
                self.traj_publishers["hip"] = self.create_publisher(
                    JointTrajectory, "/joint_trajectory_controller_hip/joint_trajectory", 10
                )

        self.get_logger().info(f"Publishers: {list(self.traj_publishers.keys())}")

    def _setup_pose_subscribers(self):
        """Create PoseStamped subscribers for non-interactive mode."""
        if self.robot_structure == "single_arm":
            self.create_subscription(
                PoseStamped,
                "/cartesian_pose_controller/target_pose",
                lambda msg: self._pose_cb(msg, 0),
                10,
            )
        else:
            for i, arm in enumerate(self.arm_states):
                suffix = arm.name.replace("arm_", "")  # "left" or "right"
                topic = f"/cartesian_pose_controller/target_pose/{suffix}"
                self.create_subscription(
                    PoseStamped, topic, lambda msg, idx=i: self._pose_cb(msg, idx), 10
                )

    def description_cb(self, msg):
        """Initialize solver from robot_description URDF."""
        if self.solver is not None:
            return

        self._urdf_data = msg.data

        # Wait until arm_states are populated by the detection thread
        while not self.arm_states and rclpy.ok():
            time.sleep(0.1)

        try:
            self.solver = PyrokiIKSolver(msg.data)
            self.joint_names = self.solver.joint_names

            self.get_logger().info(
                f"Solver initialized, joints ({len(self.joint_names)}): {self.joint_names}"
            )

            # Build per-arm joint masks and index mappings
            self._build_arm_masks()

            with self._state_lock:
                if self.current_q is None:
                    self.current_q = np.zeros(len(self.joint_names))

            # Precompute joint index mapping for publishers
            self._precompute_publisher_indices()

        except Exception as e:
            self.get_logger().error(f"Failed to initialize solver: {e}")

    def _build_arm_masks(self):
        """Build joint masks based on solve_mode."""
        n = len(self.joint_names)
        include_hip = self.solve_mode == "decoupled_with_hip"

        for arm in self.arm_states:
            if arm.name == "":
                # Single arm: all joints active
                arm.joint_mask = np.ones(n, dtype=np.float32)
                arm.joint_indices = list(range(n))
            else:
                # Multi-arm: this arm's joints + optionally hip
                mask = np.zeros(n, dtype=np.float32)
                indices = []
                # exclude clamp joints because not controlled via JointTrajectoryController
                # give error otherwise because mismatch of number joints given and expected
                excluded_joints = {
                    "arm_left/clamp_left_finger_joint",
                    "arm_right/clamp_right_finger_joint",
                }

                for i, jname in enumerate(self.joint_names):
                    self.get_logger().info(f"joint name: {jname}")

                    is_this_arm = jname.startswith(f"{arm.name}/")
                    is_hip = include_hip and jname.startswith("hip")
                    is_excluded = jname in excluded_joints

                    if (is_this_arm or is_hip) and not is_excluded:
                        mask[i] = 1.0
                        indices.append(i)
                arm.joint_mask = mask
                arm.joint_indices = indices

            self.get_logger().info(
                f"Arm__ '{arm.name or 'arm'}': mask has {int(arm.joint_mask.sum())} active joints, "
                f"indices={arm.joint_indices}"
            )

    def _precompute_publisher_indices(self):
        """Map each publisher's joints to indices in the full joint vector."""
        self._pub_joint_map: dict[str, tuple[list[str], list[int]]] = {}

        for comp_name in self.traj_publishers:
            if comp_name == "":
                comp_joints = list(self.joint_names)
            elif comp_name == "hip":
                comp_joints = [n for n in self.joint_names if n.startswith("hip")]
            else:
                comp_joints = [n for n in self.joint_names if n.startswith(f"{comp_name}/")]

            indices = [self.joint_names.index(j) for j in comp_joints]
            self._pub_joint_map[comp_name] = (comp_joints, indices)

        mapping = {k: v[0] for k, v in self._pub_joint_map.items()}
        self.get_logger().debug(f"Publisher joint mapping: {mapping}")

    def state_cb(self, msg):
        if self.joint_names is None or len(self.joint_names) == 0:
            return

        with self._state_lock:
            if self.current_q is None:
                self.current_q = np.zeros(len(self.joint_names))
            new_q = list(self.current_q)
            for i, name in enumerate(self.joint_names):
                if name in msg.name:
                    new_q[i] = msg.position[msg.name.index(name)]
            self.current_q = np.array(new_q)

        if not self.fully_initialized and self.solver is not None:
            self._initialize_targets()

    def _initialize_targets(self):
        """Compute FK to sync all targets/markers to actual robot pose on startup."""
        with self._state_lock:
            if self.current_q is None:
                return
            actual_q = self.current_q.copy()

        robot = self.solver.robot

        try:
            transforms = robot.forward_kinematics(actual_q)
            link_names = robot.links.names

            for arm in self.arm_states:
                tcp_idx = link_names.index(arm.target_link)
                pose = SE3(transforms[tcp_idx])
                arm.target_wxyz = np.array(pose.rotation().wxyz)
                arm.target_pos = np.array(pose.translation())

                solution, pos_err, ori_err = self.solver.solve(
                    arm.target_link,
                    np.array(pose.translation()),
                    np.array(pose.rotation().wxyz),
                    actual_q.copy(),
                    arm.joint_mask
                )

                # Extract this arm's joints from the solution
                arm_q = np.array([solution[i] for i in arm.joint_indices])

                self.get_logger().info(f"FK_ target to check solver output: {arm.name} arm_q: {arm_q}, pos_err: {pos_err}, ori_err: {ori_err}")

        except Exception as e:
            self.get_logger().error(f"FK failed during initialization: {e}")
            return

        if self.use_interactive_markers:
            for arm in self.arm_states:
                self._init_interactive_marker(arm)
            self.server.applyChanges()

        self.fully_initialized = True
        arm_info = ", ".join(
            f"{a.name or 'arm'}: ({a.target_pos[0]:.3f}, {a.target_pos[1]:.3f}, {a.target_pos[2]:.3f})"
            for a in self.arm_states
        )
        self.get_logger().info(f"Targets synced to actual pose. {arm_info}")

    def _init_interactive_marker(self, arm: ArmState):
        """Create a 6-DOF interactive marker for one arm."""
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = arm.marker_name
        int_marker.description = f"IK Target ({arm.name or 'arm'})"
        int_marker.pose.position.x = float(arm.target_pos[0])
        int_marker.pose.position.y = float(arm.target_pos[1])
        int_marker.pose.position.z = float(arm.target_pos[2])
        int_marker.pose.orientation.w = float(arm.target_wxyz[0])
        int_marker.pose.orientation.x = float(arm.target_wxyz[1])
        int_marker.pose.orientation.y = float(arm.target_wxyz[2])
        int_marker.pose.orientation.z = float(arm.target_wxyz[3])
        int_marker.scale = 0.2

        # Sphere visual
        sphere_marker = Marker()
        sphere_marker.type = Marker.SPHERE
        sphere_marker.scale.x = 0.05
        sphere_marker.scale.y = 0.05
        sphere_marker.scale.z = 0.05
        r, g, b, a = arm.marker_color
        sphere_marker.color.r = r
        sphere_marker.color.g = g
        sphere_marker.color.b = b
        sphere_marker.color.a = a

        sphere_control = InteractiveMarkerControl()
        sphere_control.always_visible = True
        sphere_control.markers.append(sphere_marker)
        int_marker.controls.append(sphere_control)

        # 6-DOF controls (3 rotate + 3 translate)
        for axis, quat in [("x", (1, 1, 0, 0)), ("y", (1, 0, 1, 0)), ("z", (1, 0, 0, 1))]:
            w, x, y, z = (v / (2**0.5) for v in quat)
            for mode, name_prefix in [
                (InteractiveMarkerControl.ROTATE_AXIS, "rotate"),
                (InteractiveMarkerControl.MOVE_AXIS, "move"),
            ]:
                control = InteractiveMarkerControl()
                control.name = f"{name_prefix}_{axis}"
                control.interaction_mode = mode
                control.orientation.w = w
                control.orientation.x = x
                control.orientation.y = y
                control.orientation.z = z
                int_marker.controls.append(control)

        self.server.insert(int_marker, feedback_callback=self.process_feedback)
        self.get_logger().info(f"Interactive marker '{arm.marker_name}' created.")

    def _pose_cb(self, msg, arm_index):
        """PoseStamped callback for non-interactive mode."""
        if not self.fully_initialized:
            return
        self.get_logger().info(f"_pose_cb {msg}")
        with self._state_lock:
            arm = self.arm_states[arm_index]
            arm.target_pos = np.array(
                [
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                ]
            )
            arm.target_wxyz = np.array(
                [
                    msg.pose.orientation.w,
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                ]
            )

    def process_feedback(self, feedback):
        """Handle interactive marker feedback — identify marker by name."""
        if feedback.event_type != feedback.POSE_UPDATE:
            return

        for arm in self.arm_states:
            if feedback.marker_name == arm.marker_name:
                with self._state_lock:
                    arm.target_pos = np.array(
                        [
                            feedback.pose.position.x,
                            feedback.pose.position.y,
                            feedback.pose.position.z,
                        ]
                    )
                    arm.target_wxyz = np.array(
                        [
                            feedback.pose.orientation.w,
                            feedback.pose.orientation.x,
                            feedback.pose.orientation.y,
                            feedback.pose.orientation.z,
                        ]
                    )
                return

    def _split_and_publish(self, full_q, full_velocities):
        """Split full joint solution into per-controller JointTrajectory messages."""
        stamp = self.get_clock().now().to_msg()

        for comp_name, publisher in self.traj_publishers.items():
            joint_names, indices = self._pub_joint_map[comp_name]
            if not indices:
                continue

            msg = JointTrajectory()
            msg.header.stamp = stamp
            msg.joint_names = joint_names

            p = JointTrajectoryPoint()
            p.positions = [float(full_q[i]) for i in indices]
            p.velocities = [0.0] * len(indices)
            p.time_from_start.sec = 0
            p.time_from_start.nanosec = 100_000_000

            msg.points.append(p)
            publisher.publish(msg)

    def main_loop(self):
        rate_sec = 0.04
        while rclpy.ok():
            if not self.fully_initialized:
                time.sleep(0.1)
                continue

            try:
                if self.solve_mode == "whole_body" and len(self.arm_states) > 1:
                    self._control_step_whole_body(rate_sec)
                else:
                    self._control_step_decoupled(rate_sec)
            except Exception as e:
                self.get_logger().error(f"Control loop error: {e}")

            time.sleep(rate_sec)

    def _control_step_decoupled(self, rate_sec):
        """Per-arm control: solve each arm independently, assemble, publish."""
        current_time = time.time()
        dt = current_time - (self.last_time if self.last_time else current_time - rate_sec)
        if dt <= 0:
            dt = rate_sec

        with self._state_lock:
            actual_q = self.current_q.copy()

        # Start with actual joint states (hip + uncontrolled joints stay at current values)
        full_cfg = actual_q.copy()

        for arm in self.arm_states:
            with self._state_lock:
                t_pos = arm.target_pos.copy()
                t_wxyz = arm.target_wxyz.copy()
            
            self.get_logger().info(f"{arm.name} current arm.target_pos: {t_pos}")

            # Build prev_cfg: actual_q with this arm's smoothed values overlaid
            prev_cfg = actual_q.copy()
            if arm.smoothed_arm_q is not None:
                for i, idx in enumerate(arm.joint_indices):
                    prev_cfg[idx] = arm.smoothed_arm_q[i]

            # Solve IK for this arm only (other joints locked via mask)
            solution, pos_err, ori_err = self.solver.solve(
                arm.target_link, t_pos, t_wxyz, prev_cfg, arm.joint_mask
            )

            self.get_logger().debug(
                f"[IK {arm.name or 'arm'}] pos_err={pos_err:.4f} ori_err={ori_err:.4f}",
                throttle_duration_sec=1.0,
            )

            # Extract this arm's joints from the solution
            arm_q = np.array([solution[i] for i in arm.joint_indices])

            self.get_logger().info(f"{arm.name} arm_q: {arm_q}, pos_err: {pos_err}, ori_err: {ori_err}")

            # Per-arm smoothing + velocity limiting
            arm.smoothed_arm_q = smooth_and_limit(
                arm_q,
                arm.smoothed_arm_q,
                arm.last_smoothed_arm_q,
                self.alpha_filter,
                self.max_joint_velocity,
                dt,
            )
            arm.last_smoothed_arm_q = arm.smoothed_arm_q.copy()

            # Write this arm's joints into the full configuration
            for i, idx in enumerate(arm.joint_indices):
                full_cfg[idx] = arm.smoothed_arm_q[i]

        # Compute full-body velocities
        if self.last_full_q is not None:
            velocities = (full_cfg - self.last_full_q) / dt
        else:
            velocities = np.zeros_like(full_cfg)

        self.last_full_q = full_cfg.copy()
        self.last_time = current_time

        self._split_and_publish(full_cfg, velocities)

    def _control_step_whole_body(self, rate_sec):
        """Whole-body control: solve all arms + hip simultaneously in one optimization."""
        current_time = time.time()
        dt = current_time - (self.last_time if self.last_time else current_time - rate_sec)
        if dt <= 0:
            dt = rate_sec

        with self._state_lock:
            actual_q = self.current_q.copy()
            target_positions = np.stack([arm.target_pos.copy() for arm in self.arm_states])
            target_wxyzs = np.stack([arm.target_wxyz.copy() for arm in self.arm_states])

        target_links = [arm.target_link for arm in self.arm_states]
        prev_cfg = self.smoothed_q.copy() if self.smoothed_q is not None else actual_q

        solution, errors = self.solver.solve_multi(
            target_links, target_positions, target_wxyzs, prev_cfg
        )

        err_str = ", ".join(
            f"{arm.name or 'arm'}: p={e[0]:.4f} o={e[1]:.4f}"
            for arm, e in zip(self.arm_states, errors)
        )
        self.get_logger().debug(f"[IK whole_body] {err_str}", throttle_duration_sec=1.0)

        # Whole-body smoothing + velocity limiting
        self.smoothed_q = smooth_and_limit(
            solution,
            self.smoothed_q,
            self.last_smoothed_q,
            self.alpha_filter,
            self.max_joint_velocity,
            dt,
        )
        if self.last_smoothed_q is not None:
            velocities = (self.smoothed_q - self.last_smoothed_q) / dt
        else:
            velocities = np.zeros_like(self.smoothed_q)

        self.last_smoothed_q = self.smoothed_q.copy()
        self.last_time = current_time

        self._split_and_publish(self.smoothed_q, velocities)


def main(args=None):
    rclpy.init(args=args)
    node = InteractivePyrokiNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
