#!/usr/bin/env python3
"""
ROS 2 Node that executes a sequence of predefined grasp positions using PyRoki IK.

Reads poses from a YAML file and drives the robot through them sequentially.
Supports POSE (convergence IK), LINEAR (straight line), ARC (circular arc),
HIP (direct hip joint), and JOINT (direct joint-space) motion types.

Usage:
    ros2 run duatic_teleop pose_sequence_node --ros-args \
        -p poses_file:=/path/to/poses.yaml \
        -p pause_between_poses:=2.0
"""

import time
import threading
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from duatic_kinematics.pyroki_solver import PyrokiIKSolver
from duatic_kinematics.waypoint_generators import linear_waypoints, arc_waypoints
from duatic_kinematics.smoothing import smooth_and_limit
from duatic_dynaarm_extensions.duatic_helpers.duatic_robots_helper import DuaticRobotsHelper


def xyzw_to_wxyz(x, y, z, w):
    """Convert ROS xyzw quaternion to PyRoki wxyz format."""
    return np.array([w, x, y, z], dtype=np.float32)


def parse_poses_yaml(yaml_path):
    """Parse the poses YAML file."""
    with open(yaml_path, 'r') as f:
        raw = yaml.safe_load(f)

    if raw is None:
        return []

    poses = []
    for name, data in raw.items():
        if not isinstance(data, dict):
            continue
        pose_type = data.get('type', 'POSE')
        poses.append((name, pose_type, data))

    return poses


class PoseSequenceNode(Node):
    def __init__(self):
        super().__init__('pose_sequence_node')

        self.declare_parameter('poses_file', '')
        self.declare_parameter('pause_between_poses', 2.0)
        self.declare_parameter('max_motion_duration', 10.0)
        self.declare_parameter('confirm_each_pose', False)
        self.declare_parameter('solve_mode', 'whole_body')
        self.declare_parameter('control_rate', 25.0)
        self.declare_parameter('pos_threshold', 0.005)
        self.declare_parameter('ori_threshold', 0.02)
        self.declare_parameter('alpha_filter', 0.4)
        self.declare_parameter('max_joint_velocity', 1.5)
        self.declare_parameter('self_collision_weight', 10.0)
        self.declare_parameter('linear_velocity', 0.1)
        self.declare_parameter('arc_velocity', 0.1)

        self.poses_file = self.get_parameter('poses_file').value
        self.pause_between = self.get_parameter('pause_between_poses').value
        self.max_motion_duration = self.get_parameter('max_motion_duration').value
        self.confirm_each_pose = self.get_parameter('confirm_each_pose').value
        self.solve_mode = self.get_parameter('solve_mode').value
        self.control_rate = self.get_parameter('control_rate').value
        self.pos_threshold = self.get_parameter('pos_threshold').value
        self.ori_threshold = self.get_parameter('ori_threshold').value

        if not self.poses_file:
            self.get_logger().error("No poses_file parameter provided!")
            return

        # Robot helper for auto-detection
        self.robots_helper = DuaticRobotsHelper(self)

        self.solver = None
        self.joint_names = []
        self.current_q = None
        self._urdf_data = None
        self._state_lock = threading.Lock()

        # Smoothing
        self.alpha_filter = self.get_parameter('alpha_filter').value
        self.max_joint_velocity = self.get_parameter('max_joint_velocity').value
        self.smoothed_q = None
        self.last_smoothed_q = None

        # EE distance lock: stores the scalar distance between left and right EE
        # when grasping, so it can be enforced during subsequent moves
        self._locked_ee_distance = None

        # Publishers
        self.traj_publishers = {}
        self._pub_joint_map = {}

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', self._description_cb, qos)
        self.create_subscription(JointState, '/joint_states', self._state_cb, 10)

        # Start in background
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _description_cb(self, msg):
        if self._urdf_data is not None:
            return
        self._urdf_data = msg.data

    def _state_cb(self, msg):
        if not self.joint_names:
            return
        with self._state_lock:
            if self.current_q is None:
                self.current_q = np.zeros(len(self.joint_names))
            new_q = list(self.current_q)
            for i, name in enumerate(self.joint_names):
                if name in msg.name:
                    new_q[i] = msg.position[msg.name.index(name)]
            self.current_q = np.array(new_q)

    def _run(self):
        """Main sequence: detect robot, init solver, execute poses."""
        self.get_logger().info("Waiting for robot detection...")
        self.robots_helper.wait_for_robot()

        robot_structure = self.robots_helper.robot_structure
        arm_names = self.robots_helper.get_component_names("arm")
        hip_names = self.robots_helper.get_component_names("hip")

        self.get_logger().info(
            f"Detected: structure={robot_structure}, arms={arm_names}, hip={hip_names}"
        )

        # Setup publishers
        if robot_structure == "single_arm":
            self.traj_publishers[""] = self.create_publisher(
                JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        else:
            for arm_name in arm_names:
                topic = f'/joint_trajectory_controller_{arm_name}/joint_trajectory'
                self.traj_publishers[arm_name] = self.create_publisher(
                    JointTrajectory, topic, 10)
            if hip_names:
                self.traj_publishers["hip"] = self.create_publisher(
                    JointTrajectory, '/joint_trajectory_controller_hip/joint_trajectory', 10)

        # Wait for URDF
        self.get_logger().info("Waiting for URDF...")
        while self._urdf_data is None and rclpy.ok():
            time.sleep(0.1)

        # Init solver
        self_coll_weight = self.get_parameter('self_collision_weight').value
        self.solver = PyrokiIKSolver(
            self._urdf_data, self_collision_weight=self_coll_weight
        )
        self.joint_names = self.solver.joint_names
        self.get_logger().info(
            f"Solver ready, {len(self.joint_names)} joints, "
            f"self_collision_weight={self_coll_weight}"
        )

        # Build publisher index map
        for comp_name in self.traj_publishers:
            if comp_name == "":
                comp_joints = list(self.joint_names)
            elif comp_name == "hip":
                comp_joints = [n for n in self.joint_names if n.startswith("hip")]
            else:
                comp_joints = [n for n in self.joint_names if n.startswith(f"{comp_name}/")]
            indices = [self.joint_names.index(j) for j in comp_joints]
            self._pub_joint_map[comp_name] = (comp_joints, indices)

        # Wait for joint states
        self.get_logger().info("Waiting for joint states...")
        while self.current_q is None and rclpy.ok():
            time.sleep(0.1)

        # Determine arm target links
        if robot_structure == "single_arm":
            self._left_link = "flange"
            self._right_link = None
        else:
            self._left_link = None
            self._right_link = None
            for arm_name in arm_names:
                if "left" in arm_name:
                    self._left_link = f"{arm_name}/flange"
                elif "right" in arm_name:
                    self._right_link = f"{arm_name}/flange"

        # Initialize smoothing state from current position (prevents jump on first pose)
        with self._state_lock:
            self.smoothed_q = self.current_q.copy()
            self.last_smoothed_q = self.current_q.copy()
        self.get_logger().info("Initialized from current joint positions.")

        # JIT warmup: run dummy solves so compilation happens before the sequence
        if self._left_link and self._right_link:
            self.get_logger().info("Warming up JIT (this takes a few seconds)...")
            dummy_pos = np.zeros((2, 3), dtype=np.float32)
            dummy_wxyz = np.array([[1, 0, 0, 0], [1, 0, 0, 0]], dtype=np.float32)
            dummy_cfg = self.current_q.copy().astype(np.float32)
            dummy_mask = np.ones(len(self.joint_names), dtype=np.float32)
            links = [self._left_link, self._right_link]
            self.solver.solve_multi(links, dummy_pos, dummy_wxyz, dummy_cfg, joint_mask=dummy_mask)
            self.get_logger().info("JIT warmup complete.")

        # Parse and execute poses
        poses = parse_poses_yaml(self.poses_file)
        if not poses:
            self.get_logger().error(f"No poses found in {self.poses_file}")
            return

        self.get_logger().info(f"Loaded {len(poses)} poses from {self.poses_file}")
        self.get_logger().info("=" * 60)

        for i, (name, pose_type, data) in enumerate(poses):
            if not rclpy.ok():
                break

            self.get_logger().info(f"[{i+1}/{len(poses)}] Next: {name} (type={pose_type})")

            if self.confirm_each_pose:
                self.get_logger().info(f"  Press ENTER to execute {name}, or 'q' to abort...")
                try:
                    user_input = input()
                    if user_input.strip().lower() == 'q':
                        self.get_logger().info("Aborted by user.")
                        return
                except EOFError:
                    pass

            # Handle ee_distance_lock flag
            ee_lock = data.get('ee_distance_lock', None)
            if ee_lock is True and self._locked_ee_distance is None:
                if self._left_link and self._right_link:
                    cfg = self.smoothed_q if self.smoothed_q is not None else self.current_q
                    positions, _ = self.solver.forward_kinematics(
                        cfg.astype(np.float32), [self._left_link, self._right_link]
                    )
                    self._locked_ee_distance = float(np.linalg.norm(positions[0] - positions[1]))
                    self.get_logger().info(
                        f"  EE distance locked: {self._locked_ee_distance:.4f}m"
                    )
            elif ee_lock is False:
                if self._locked_ee_distance is not None:
                    self.get_logger().info(f"  EE distance lock released")
                self._locked_ee_distance = None

            self.get_logger().info(f"  Executing {name}...")

            if pose_type == "POSE":
                self._execute_pose(name, data)
            elif pose_type == "LINEAR":
                self._execute_linear(name, data)
            elif pose_type == "ARC":
                self._execute_arc(name, data)
            elif pose_type == "HIP":
                self._execute_hip(name, data)
            elif pose_type == "JOINT":
                self._execute_joint(name, data)
            else:
                self.get_logger().warn(f"Unknown pose type '{pose_type}', skipping.")
                continue

            self.get_logger().info(f"  -> {name} reached. Pausing {self.pause_between}s...")
            time.sleep(self.pause_between)

        self.get_logger().info("=" * 60)
        self.get_logger().info("Pose sequence complete!")

    # ── Helpers ──────────────────────────────────────────────────

    def _build_joint_mask(self, data):
        """Build joint mask and hip override map from pose data.

        Returns (joint_mask, hip_overrides dict {index: value}).
        """
        mask = np.ones(len(self.joint_names), dtype=np.float32)
        hip_overrides = {}

        hip_data = data.get('hip_pose')
        if hip_data:
            joints = hip_data.get('joints', {})
            for joint_name, value in joints.items():
                if value is None or value == '':
                    continue
                for i, jname in enumerate(self.joint_names):
                    if jname == joint_name or jname == f"hip/{joint_name}" or jname.endswith(f"/{joint_name}"):
                        hip_overrides[i] = float(value)
                        mask[i] = 0.0
                        self.get_logger().info(f"  -> Hip fixed: {jname} = {value}")
                        break

        lock_joints = data.get('lock_joints')
        if lock_joints:
            for lock_name in lock_joints:
                for i, jname in enumerate(self.joint_names):
                    if jname == lock_name or jname == f"hip/{lock_name}" or jname.endswith(f"/{lock_name}"):
                        mask[i] = 0.0
                        self.get_logger().info(f"  -> Locking joint: {jname}")
                        break

        return mask, hip_overrides

    def _apply_hip_overrides(self, cfg, hip_overrides):
        """Apply hip joint overrides to a config array."""
        cfg = cfg.copy()
        for i, value in hip_overrides.items():
            cfg[i] = value
        return cfg

    def _ramp_hip_overrides(self, hip_overrides, max_delta):
        """Smoothly ramp locked hip joints towards their target values."""
        for idx, target_val in hip_overrides.items():
            diff = target_val - self.smoothed_q[idx]
            self.smoothed_q[idx] += np.clip(diff, -max_delta, max_delta)

    def _enforce_ee_distance(self, target_links, positions):
        """Adjust target positions to maintain locked EE distance.

        When _locked_ee_distance is set, this scales the left and right
        target positions along their current direction so that the distance
        matches the locked value, while keeping the midpoint unchanged.

        Args:
            target_links: list of link names
            positions: (N, 3) target positions array (modified in place)

        Returns:
            The (possibly modified) positions array.
        """
        if self._locked_ee_distance is None:
            return positions
        if len(target_links) != 2:
            return positions
        if not (self._left_link in target_links and self._right_link in target_links):
            return positions

        left_idx = target_links.index(self._left_link)
        right_idx = target_links.index(self._right_link)

        midpoint = (positions[left_idx] + positions[right_idx]) / 2.0
        direction = positions[left_idx] - positions[right_idx]
        current_dist = float(np.linalg.norm(direction))

        if current_dist < 1e-6:
            return positions

        # Scale direction to match locked distance, keep midpoint
        unit_dir = direction / current_dist
        half_offset = unit_dir * (self._locked_ee_distance / 2.0)
        positions[left_idx] = midpoint + half_offset
        positions[right_idx] = midpoint - half_offset
        return positions

    def _parse_targets(self, data):
        """Parse left/right pose targets from YAML data.

        Returns (target_links, target_positions, target_wxyzs) or None if no targets.
        Orientations are optional (returns None for wxyzs if absent).
        """
        left_data = data.get('left_pose')
        right_data = data.get('right_pose')

        target_links = []
        target_positions = []
        target_wxyzs = []
        has_orientations = True

        if left_data and self._left_link:
            lp = left_data['position']
            target_links.append(self._left_link)
            target_positions.append([lp['x'], lp['y'], lp['z']])
            lo = left_data.get('orientation')
            if lo:
                target_wxyzs.append(xyzw_to_wxyz(lo['x'], lo['y'], lo['z'], lo['w']))
            else:
                has_orientations = False

        if right_data and self._right_link:
            rp = right_data['position']
            target_links.append(self._right_link)
            target_positions.append([rp['x'], rp['y'], rp['z']])
            ro = right_data.get('orientation')
            if ro:
                target_wxyzs.append(xyzw_to_wxyz(ro['x'], ro['y'], ro['z'], ro['w']))
            else:
                has_orientations = False

        if not target_links:
            return None

        positions = np.array(target_positions, dtype=np.float32)
        wxyzs = np.array(target_wxyzs, dtype=np.float32) if has_orientations else None
        return target_links, positions, wxyzs

    def _parse_via_points(self, data):
        """Parse via points for ARC trajectories.

        Returns (N, 3) array of via positions or None.
        """
        left_via = data.get('left_via')
        right_via = data.get('right_via')
        via_positions = []

        if left_via and self._left_link:
            via_positions.append([left_via['x'], left_via['y'], left_via['z']])
        if right_via and self._right_link:
            via_positions.append([right_via['x'], right_via['y'], right_via['z']])

        if not via_positions:
            return None
        return np.array(via_positions, dtype=np.float32)

    # ── Shared Cartesian control loop ────────────────────────────

    def _execute_cartesian_trajectory(self, target_links, waypoint_gen, joint_mask, hip_overrides):
        """Run the IK control loop over a Cartesian waypoint generator.

        Used by LINEAR and ARC trajectory types.

        Args:
            target_links: list of link names for IK
            waypoint_gen: generator yielding (positions, wxyzs, info)
            joint_mask: joint mask array
            hip_overrides: dict of {joint_index: target_value}
        """
        rate_sec = 1.0 / self.control_rate

        for positions, wxyzs, info in waypoint_gen:
            if not rclpy.ok():
                return

            # Enforce EE distance constraint on target positions
            positions = self._enforce_ee_distance(target_links, positions.copy())

            prev_cfg = self.smoothed_q if self.smoothed_q is not None else self.current_q.copy()
            prev_cfg = self._apply_hip_overrides(prev_cfg, hip_overrides)

            t0 = time.monotonic()
            if len(target_links) == 1:
                solution, pos_err, ori_err = self.solver.solve(
                    target_links[0], positions[0], wxyzs[0], prev_cfg,
                    joint_mask=joint_mask
                )
            else:
                solution, errors = self.solver.solve_multi(
                    target_links, positions, wxyzs, prev_cfg,
                    joint_mask=joint_mask
                )
            solve_ms = (time.monotonic() - t0) * 1000.0
            if solve_ms > 100.0:
                self.get_logger().warn(
                    f"  [step {info['step']}] IK solve took {solve_ms:.0f}ms (>{100}ms)"
                )

            self.smoothed_q = smooth_and_limit(
                solution, self.smoothed_q, self.last_smoothed_q,
                self.alpha_filter, self.max_joint_velocity, rate_sec,
            )
            self._ramp_hip_overrides(hip_overrides, self.max_joint_velocity * rate_sec)
            self.last_smoothed_q = self.smoothed_q.copy()
            self._publish(self.smoothed_q)
            time.sleep(rate_sec)

    # ── Trajectory executors ─────────────────────────────────────

    def _execute_pose(self, name, data):
        """Execute a POSE-type position using convergence-based IK."""
        parsed = self._parse_targets(data)
        if parsed is None:
            self.get_logger().warn(f"  No valid targets for {name}, skipping.")
            return

        target_links, target_positions, target_wxyzs = parsed
        joint_mask, hip_overrides = self._build_joint_mask(data)

        # Enforce EE distance constraint on target positions
        target_positions = self._enforce_ee_distance(target_links, target_positions.copy())

        rate_sec = 1.0 / self.control_rate
        max_iterations = int(self.max_motion_duration / rate_sec)
        converged = False
        pos_err = ori_err = float('inf')
        wall_start = time.monotonic()

        for step in range(max_iterations):
            if not rclpy.ok():
                return

            prev_cfg = self.smoothed_q if self.smoothed_q is not None else self.current_q.copy()
            prev_cfg = self._apply_hip_overrides(prev_cfg, hip_overrides)

            t0 = time.monotonic()
            if len(target_links) == 1:
                solution, pos_err, ori_err = self.solver.solve(
                    target_links[0], target_positions[0], target_wxyzs[0], prev_cfg,
                    joint_mask=joint_mask
                )
            else:
                solution, errors = self.solver.solve_multi(
                    target_links, target_positions, target_wxyzs, prev_cfg,
                    joint_mask=joint_mask
                )
                pos_err = max(e[0] for e in errors)
                ori_err = max(e[1] for e in errors)
            solve_ms = (time.monotonic() - t0) * 1000.0
            if solve_ms > 100.0:
                self.get_logger().warn(
                    f"  [step {step}] IK solve took {solve_ms:.0f}ms (>{100}ms)"
                )

            self.smoothed_q = smooth_and_limit(
                solution, self.smoothed_q, self.last_smoothed_q,
                self.alpha_filter, self.max_joint_velocity, rate_sec,
            )
            self._ramp_hip_overrides(hip_overrides, self.max_joint_velocity * rate_sec)
            self.last_smoothed_q = self.smoothed_q.copy()
            self._publish(self.smoothed_q)
            time.sleep(rate_sec)

            # Convergence check
            if pos_err < self.pos_threshold and ori_err < self.ori_threshold:
                q_diff = np.max(np.abs(self.smoothed_q - solution))
                if q_diff < 0.01:
                    converged = True
                    break

            # Stagnation detection
            total_err = pos_err + ori_err
            if step == 0:
                best_err = total_err
                stagnation_count = 0
            elif total_err < best_err - 0.001:
                best_err = total_err
                stagnation_count = 0
            else:
                stagnation_count += 1
                if stagnation_count > 50:
                    break

        wall_elapsed = time.monotonic() - wall_start
        status = "converged" if converged else "stagnated" if stagnation_count > 50 else "timeout"
        self.get_logger().info(
            f"  -> {status} in {wall_elapsed:.1f}s ({step+1} steps) | pos={pos_err:.4f} ori={ori_err:.4f}"
        )

    def _execute_linear(self, name, data):
        """Execute a LINEAR-type move using the trajectory library."""
        parsed = self._parse_targets(data)
        if parsed is None:
            self.get_logger().warn(f"  No valid targets for {name}, skipping.")
            return

        target_links, target_positions, _ = parsed
        joint_mask, hip_overrides = self._build_joint_mask(data)

        # Get current EE poses via FK
        prev_cfg = self.smoothed_q if self.smoothed_q is not None else self.current_q.copy()
        start_positions, fixed_wxyzs = self.solver.forward_kinematics(
            prev_cfg.astype(np.float32), target_links
        )

        linear_vel = self.get_parameter('linear_velocity').value
        max_dist = float(np.max(np.linalg.norm(target_positions - start_positions, axis=1)))
        self.get_logger().info(f"  -> Linear move: {max_dist:.3f}m at {linear_vel} m/s")

        wall_start = time.monotonic()
        waypoints = linear_waypoints(
            start_positions, target_positions, fixed_wxyzs,
            control_rate=self.control_rate,
            linear_velocity=linear_vel,
        )
        self._execute_cartesian_trajectory(target_links, waypoints, joint_mask, hip_overrides)

        wall_elapsed = time.monotonic() - wall_start
        self.get_logger().info(f"  -> Linear move complete in {wall_elapsed:.1f}s")

    def _execute_arc(self, name, data):
        """Execute an ARC-type move using the trajectory library."""
        parsed = self._parse_targets(data)
        if parsed is None:
            self.get_logger().warn(f"  No valid targets for {name}, skipping.")
            return

        target_links, target_positions, _ = parsed
        via_positions = self._parse_via_points(data)
        if via_positions is None or via_positions.shape[0] != target_positions.shape[0]:
            self.get_logger().warn(f"  Missing or mismatched via points for {name}, skipping.")
            return

        joint_mask, hip_overrides = self._build_joint_mask(data)

        # Get current EE poses via FK
        prev_cfg = self.smoothed_q if self.smoothed_q is not None else self.current_q.copy()
        start_positions, fixed_wxyzs = self.solver.forward_kinematics(
            prev_cfg.astype(np.float32), target_links
        )

        arc_vel = self.get_parameter('arc_velocity').value
        self.get_logger().info(f"  -> Arc move at {arc_vel} m/s")

        wall_start = time.monotonic()
        waypoints = arc_waypoints(
            start_positions, via_positions, target_positions, fixed_wxyzs,
            control_rate=self.control_rate,
            arc_velocity=arc_vel,
        )
        self._execute_cartesian_trajectory(target_links, waypoints, joint_mask, hip_overrides)

        wall_elapsed = time.monotonic() - wall_start
        self.get_logger().info(f"  -> Arc move complete in {wall_elapsed:.1f}s")

    def _execute_hip(self, name, data):
        """Execute a HIP-type position by directly setting hip joints."""
        hip_data = data.get('hip_pose', {})
        joints = hip_data.get('joints', {})

        if not joints:
            self.get_logger().warn(f"  No hip joints for {name}, skipping.")
            return

        with self._state_lock:
            target_q = self.current_q.copy()

        for joint_name, value in joints.items():
            if value is None or value == '':
                continue
            for i, jname in enumerate(self.joint_names):
                if jname == joint_name or jname == f"hip/{joint_name}" or jname.endswith(f"/{joint_name}"):
                    target_q[i] = float(value)
                    self.get_logger().info(f"  -> Setting {jname} = {value}")
                    break

        if self.smoothed_q is not None:
            start_q = self.smoothed_q.copy()
        else:
            with self._state_lock:
                start_q = self.current_q.copy()

        rate_sec = 1.0 / self.control_rate
        max_iterations = int(self.max_motion_duration / rate_sec)
        duration = float(data.get('duration', 2.0))
        self.get_logger().info(f"  -> Hip move over {duration:.1f}s")

        for step in range(max_iterations):
            if not rclpy.ok():
                return

            alpha = min(1.0, (step + 1) * rate_sec / duration)
            interp_q = start_q + alpha * (target_q - start_q)

            self.smoothed_q = smooth_and_limit(
                interp_q, self.smoothed_q, self.last_smoothed_q,
                self.alpha_filter, self.max_joint_velocity, rate_sec,
            )
            self.last_smoothed_q = self.smoothed_q.copy()
            self._publish(self.smoothed_q)
            time.sleep(rate_sec)

            if np.max(np.abs(self.smoothed_q - target_q)) < 0.01:
                break

        elapsed = (step + 1) * rate_sec
        self.get_logger().info(f"  -> Hip reached in {elapsed:.1f}s")

    def _execute_joint(self, name, data):
        """Execute a JOINT-type position by interpolating all joints directly."""
        joints = data.get('joints', {})

        if not joints:
            self.get_logger().warn(f"  No joints for {name}, skipping.")
            return

        if self.smoothed_q is not None:
            start_q = self.smoothed_q.copy()
        else:
            with self._state_lock:
                start_q = self.current_q.copy()

        target_q = start_q.copy()

        for joint_name, value in joints.items():
            if value is None or value == '':
                continue
            matched = False
            for i, jname in enumerate(self.joint_names):
                if jname == joint_name or jname == f"hip/{joint_name}" or jname.endswith(f"/{joint_name}"):
                    target_q[i] = float(value)
                    matched = True
                    break
            if not matched:
                self.get_logger().warn(f"  Joint '{joint_name}' not found, skipping.")

        rate_sec = 1.0 / self.control_rate
        max_iterations = int(self.max_motion_duration / rate_sec)

        for step in range(max_iterations):
            if not rclpy.ok():
                return

            alpha = min(1.0, (step + 1) * rate_sec / 2.0)
            interp_q = start_q + alpha * (target_q - start_q)

            self.smoothed_q = smooth_and_limit(
                interp_q, self.smoothed_q, self.last_smoothed_q,
                self.alpha_filter, self.max_joint_velocity, rate_sec,
            )
            self.last_smoothed_q = self.smoothed_q.copy()
            self._publish(self.smoothed_q)
            time.sleep(rate_sec)

            if np.max(np.abs(self.smoothed_q - target_q)) < 0.01:
                break

        elapsed = (step + 1) * rate_sec
        self.get_logger().info(f"  -> Joint move reached in {elapsed:.1f}s")

    # ── Publishing ───────────────────────────────────────────────

    def _publish(self, full_q):
        """Publish joint trajectory to all controllers."""
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


def main(args=None):
    rclpy.init(args=args)
    node = PoseSequenceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
