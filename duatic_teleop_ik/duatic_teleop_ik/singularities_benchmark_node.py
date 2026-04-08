#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import time
import csv
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from scipy.spatial.transform import Rotation as R

import json

from ament_index_python.packages import get_package_share_directory
import os
import json
import numpy as np


class SingularitiesBenchmarkNode(Node):
    def __init__(self):
        super().__init__("singularities_benchmark_node")

        # ---------------- PARAMETERS ----------------
        self.num_targets = 200
        self.position_tolerance = 0.01
        self.velocity_tolerance = 0.02
        self.has_moved = False
        self.workspace = {
            "x": (-1.0, 1.0),
            "y": (-1.0, 1.0),
            "z": (0.1, 1.3),
        }

        # ---------------- STATE ----------------
        self.targets = []
        self.current_joint_state = None
        self.current_joint_velocities = None
        self.current_joint_positions = None

        self.current_target_index = 0
        self.target_active = False
        self.start_time = None

        # Logging
        self.log_data = []

        # ---------------- PUB/SUB ----------------
        self.target_pub = self.create_publisher(
            PoseStamped,
            "/cartesian_pose_controller/target_pose",
            10,
        )

        # publish full list of poses once (for rosbag)
        self.targets_pub = self.create_publisher(
            Float64MultiArray,
            "/benchmark/target_poses",
            1,
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_cb,
            10,
        )

        # ---------------- INIT ----------------
        self.load_targets()
        self.publish_all_targets_once()

        # main loop timer
        self.timer = self.create_timer(0.02, self.control_loop)

    # ============================================================
    # TARGET GENERATION
    # ============================================================

    def load_targets(self):
        package_path = get_package_share_directory('duatic_teleop_ik')

        file_path = os.path.join(package_path, 'scripts', 'robot_targets.json')

        with open(file_path, "r") as f:
            data = json.load(f)

        targets = []
        for entry in data:
            pos = np.array(entry["position"])
            quat = np.array(entry["orientation"])
            targets.append((pos, quat))

        print(f"Loaded {len(targets)} targets from {file_path}")
        self.targets = targets

    def publish_all_targets_once(self):
        """Publish all poses once for rosbag recording."""
        msg = Float64MultiArray()

        flat = []
        for pos, quat in self.targets:
            flat.extend(pos.tolist())
            flat.extend(quat.tolist())

        msg.data = flat
        self.targets_pub.publish(msg)

    # ============================================================
    # CALLBACKS
    # ============================================================

    def joint_state_cb(self, msg):
        self.current_joint_state = msg
        self.current_joint_positions = np.array(msg.position)
        self.current_joint_velocities = np.array(msg.velocity)

    # ============================================================
    # CONTROL LOOP
    # ============================================================

    def control_loop(self):
        if self.current_joint_state is None:
            return

        # finished
        if self.current_target_index >= len(self.targets):
            self.get_logger().info("Benchmark finished.")
            self.save_logs()
            rclpy.shutdown()
            return

        # send next target
        if not self.target_active:
            pos, quat = self.targets[self.current_target_index]
            self.publish_target(pos, quat)

            self.start_time = time.time()
            self.target_active = True
            return

        # check if reached
        if self.is_target_reached():
            duration = time.time() - self.start_time
            self.has_moved = True

            # log
            self.log_step(duration)

            self.get_logger().info(
                f"Target {self.current_target_index} with pos "
                f"[{self.targets[self.current_target_index][0][0]:.2f}, "
                f"{self.targets[self.current_target_index][0][1]:.2f}, "
                f"{self.targets[self.current_target_index][0][2]:.2f}] "
                f"reached in {duration:.3f}s"
            )

            self.current_target_index += 1
            self.target_active = False

    # ============================================================
    # CORE FUNCTIONS
    # ============================================================

    def publish_target(self, pos, quat_wxyz):
        msg = PoseStamped()
        msg.header.frame_id = "base_link"

        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])

        msg.pose.orientation.w = float(quat_wxyz[0])
        msg.pose.orientation.x = float(quat_wxyz[1])
        msg.pose.orientation.y = float(quat_wxyz[2])
        msg.pose.orientation.z = float(quat_wxyz[3])

        self.target_pub.publish(msg)

        #self.get_logger().info(f"Sent target {msg}")

    def is_target_reached(self):
        if self.current_joint_velocities is None:
            return False

        if time.time() - self.start_time < 0.5:
            return False
        
        vel_norm = np.linalg.norm(self.current_joint_velocities)

        if vel_norm > 0.05:
            self.has_moved = True

        return self.has_moved and vel_norm < self.velocity_tolerance

    def log_step(self, duration):
        self.log_data.append({
            "target_index": self.current_target_index,
            "duration": duration,
            "joint_positions": self.current_joint_positions.tolist(),
            "joint_velocities": self.current_joint_velocities.tolist(),
        })

    # ============================================================
    # SAVE
    # ============================================================

    def save_logs(self):
        filename = "benchmark_log.csv"

        self.get_logger().info(f"Saving log to {filename}")

        with open(filename, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)

            writer.writerow([
                "target_index",
                "duration",
                "joint_positions",
                "joint_velocities",
            ])

            for entry in self.log_data:
                writer.writerow([
                    entry["target_index"],
                    entry["duration"],
                    entry["joint_positions"],
                    entry["joint_velocities"],
                ])


# ============================================================
# MAIN
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = SingularitiesBenchmarkNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()