#!/usr/bin/env python3
"""
ROS 2 Node for Cartesian control of DynaArm using PyRoki.
Supports two input modes via the 'use_interactive_markers' parameter:
  - True  (default): 6-DOF Interactive Marker in RViz
  - False           : PoseStamped on /cartesian_pose_controller/target_pose
"""

import time
import threading
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


class InteractivePyrokiNode(Node):
    def __init__(self):
        super().__init__('interactive_pyroki_node')

        self.declare_parameter('target_link_name', 'flange')
        self.declare_parameter('use_interactive_markers', True)

        self.target_link_name = self.get_parameter('target_link_name').value
        self.use_interactive_markers = self.get_parameter('use_interactive_markers').value

        self.solver = None
        self.current_q = None
        self.joint_names = []

        self.target_pos = None
        self.target_wxyz = None

        self.smoothed_q = None
        self.last_smoothed_q = None
        self.last_time = None

        self.max_pos_error = 0.04
        self.max_ori_error = 0.15
        self.alpha_filter = 0.15
        self.max_joint_velocity = 0.5  # rad/s — max allowed velocity per joint

        self._state_lock = threading.Lock()
        self.fully_initialized = False

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.desc_sub = self.create_subscription(String, '/robot_description', self.description_cb, qos)
        self.state_sub = self.create_subscription(JointState, '/joint_states', self.state_cb, 10)
        self.traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        if self.use_interactive_markers:
            self.server = InteractiveMarkerServer(self, 'pyroki_target')
            self.get_logger().info("Mode: Interactive Marker (RViz)")
        else:
            self.server = None
            self.pose_sub = self.create_subscription(
                PoseStamped, '/cartesian_pose_controller/target_pose', self.pose_cb, 10)
            self.get_logger().info("Mode: PoseStamped topic (/cartesian_pose_controller/target_pose)")

        self.control_thread = threading.Thread(target=self.main_loop, daemon=True)
        self.control_thread.start()

    def description_cb(self, msg):
        if self.solver is not None:
            return
        try:
            self.solver = PyrokiIKSolver(msg.data, self.target_link_name)
            self.joint_names = self.solver.joint_names
            with self._state_lock:
                if self.current_q is None:
                    self.current_q = np.zeros(len(self.joint_names))
            self.get_logger().info(f"Solver initialized for '{self.target_link_name}'. Waiting for joint states...")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize solver: {e}")

    def state_cb(self, msg):
        if self.solver is None:
            return

        with self._state_lock:
            new_q = list(self.current_q)
            for i, name in enumerate(self.joint_names):
                if name in msg.name:
                    new_q[i] = msg.position[msg.name.index(name)]
            self.current_q = np.array(new_q)

        if not self.fully_initialized:
            self._initialize_target()

    def _initialize_target(self):
        """Compute FK to sync target to the actual robot pose on startup."""
        with self._state_lock:
            if self.current_q is None:
                return
            actual_q = self.current_q.copy()

        try:
            transforms = self.solver.robot.forward_kinematics(actual_q)
            link_names = self.solver.robot.links.names
            tcp_idx = link_names.index(self.target_link_name)
            tcp_transform = transforms[tcp_idx]
            target_wxyz = np.array(tcp_transform[:4])
            target_pos = np.array(tcp_transform[4:])
        except Exception as e:
            self.get_logger().error(f"FK failed during initialization: {e}")
            return

        with self._state_lock:
            self.target_wxyz = target_wxyz
            self.target_pos = target_pos

        if self.use_interactive_markers:
            self._init_interactive_marker(actual_q, target_pos, target_wxyz)
        else:
            self.fully_initialized = True
            self.get_logger().info("Synced to actual pose. Waiting for PoseStamped targets.")

    def _init_interactive_marker(self, actual_q, target_pos, target_wxyz):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = "pyroki_target"
        int_marker.description = "PyRoki IK Target"
        int_marker.pose.position.x = float(target_pos[0])
        int_marker.pose.position.y = float(target_pos[1])
        int_marker.pose.position.z = float(target_pos[2])
        int_marker.pose.orientation.w = float(target_wxyz[0])
        int_marker.pose.orientation.x = float(target_wxyz[1])
        int_marker.pose.orientation.y = float(target_wxyz[2])
        int_marker.pose.orientation.z = float(target_wxyz[3])
        int_marker.scale = 0.2

        sphere_marker = Marker()
        sphere_marker.type = Marker.SPHERE
        sphere_marker.scale.x = 0.05
        sphere_marker.scale.y = 0.05
        sphere_marker.scale.z = 0.05
        sphere_marker.color.r = 1.0
        sphere_marker.color.g = 0.1
        sphere_marker.color.b = 0.1
        sphere_marker.color.a = 0.5

        sphere_control = InteractiveMarkerControl()
        sphere_control.always_visible = True
        sphere_control.markers.append(sphere_marker)
        int_marker.controls.append(sphere_control)

        for axis, quat in [('x', (1, 1, 0, 0)), ('y', (1, 0, 1, 0)), ('z', (1, 0, 0, 1))]:
            w, x, y, z = [v / (2 ** 0.5) for v in quat]  # normalize
            for mode, name_prefix in [
                (InteractiveMarkerControl.ROTATE_AXIS, 'rotate'),
                (InteractiveMarkerControl.MOVE_AXIS, 'move'),
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
        self.server.applyChanges()
        self.fully_initialized = True
        self.get_logger().info("Interactive marker initialized and synced to actual pose.")

    def pose_cb(self, msg):
        if not self.fully_initialized:
            return
        with self._state_lock:
            self.target_pos = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ])
            self.target_wxyz = np.array([
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
            ])

    def process_feedback(self, feedback):
        if feedback.event_type == feedback.POSE_UPDATE:
            self.get_logger().info(
                f"[feedback] pos=({feedback.pose.position.x:.3f}, {feedback.pose.position.y:.3f}, {feedback.pose.position.z:.3f})",
                throttle_duration_sec=0.5)
            with self._state_lock:
                self.target_pos = np.array([
                    feedback.pose.position.x,
                    feedback.pose.position.y,
                    feedback.pose.position.z,
                ])
                self.target_wxyz = np.array([
                    feedback.pose.orientation.w,
                    feedback.pose.orientation.x,
                    feedback.pose.orientation.y,
                    feedback.pose.orientation.z,
                ])

    def main_loop(self):
        rate_sec = 0.04
        while rclpy.ok():
            if not self.fully_initialized or self.solver is None:
                time.sleep(0.1)
                continue

            try:
                current_time = time.time()
                dt = current_time - (self.last_time if self.last_time else current_time - rate_sec)
                if dt <= 0:
                    dt = rate_sec

                with self._state_lock:
                    actual_q = self.current_q.copy()
                    t_pos = self.target_pos.copy()
                    t_wxyz = self.target_wxyz.copy()

                # Fix 1: seed from smoothed_q (commanded pose) not actual joint states
                q_init = self.smoothed_q.copy() if self.smoothed_q is not None else actual_q

                solution, pos_err, ori_err = self.solver.solve(t_pos, t_wxyz, q_init)

                # Fix 3: multi-start — retry with actual_q and zeros if error is large
                if pos_err > 0.1 or ori_err > 0.3:
                    for seed in [actual_q, np.zeros(len(self.joint_names))]:
                        s, pe, oe = self.solver.solve(t_pos, t_wxyz, seed)
                        if pe < pos_err:
                            solution, pos_err, ori_err = s, pe, oe

                self.get_logger().info(
                    f"[IK] pos_err={pos_err:.4f} ori_err={ori_err:.4f} target=({t_pos[0]:.3f},{t_pos[1]:.3f},{t_pos[2]:.3f})",
                    throttle_duration_sec=1.0)

                if self.smoothed_q is None:
                    self.smoothed_q = solution
                else:
                    self.smoothed_q = (self.alpha_filter * solution) + ((1.0 - self.alpha_filter) * self.smoothed_q)

                if self.last_smoothed_q is not None:
                    max_delta = self.max_joint_velocity * dt
                    delta = self.smoothed_q - self.last_smoothed_q
                    self.smoothed_q = self.last_smoothed_q + np.clip(delta, -max_delta, max_delta)
                    velocities = (self.smoothed_q - self.last_smoothed_q) / dt
                else:
                    velocities = np.zeros_like(self.smoothed_q)

                self.last_smoothed_q = self.smoothed_q.copy()
                self.last_time = current_time

                msg = JointTrajectory()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.joint_names = self.joint_names

                p = JointTrajectoryPoint()
                p.positions = self.smoothed_q.tolist()
                p.velocities = velocities.tolist()
                p.time_from_start.sec = 0
                p.time_from_start.nanosec = 100_000_000

                msg.points.append(p)
                self.traj_pub.publish(msg)

            except Exception as e:
                self.get_logger().error(f"Control loop error: {e}")

            time.sleep(rate_sec)


def main(args=None):
    rclpy.init(args=args)
    node = InteractivePyrokiNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
