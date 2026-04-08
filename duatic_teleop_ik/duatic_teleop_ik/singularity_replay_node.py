#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import time

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import rclpy.serialization


BAG_PATH = "/ros2_ws/bags/rosbag2_2026_04_02-11_15_33"
JOINT_STATE_TOPIC = "/joint_states"


class SingularityReplayNode(Node):
    def __init__(self):
        super().__init__("singularity_replay_node")

        self.publisher = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10,
        )

        self.get_logger().info("Loading bag...")

        self.rows = self.load_joint_states_mcap(BAG_PATH)

        self.get_logger().info(f"Loaded {len(self.rows)} joint states")

        # ---- YOUR EVENTS (paste from analysis) ----
        self.events = [
            2.0,
            64.0,
            107.0,
            115.0,
            170.0,
            184.0,
            206.0,
            224.0,
            264.0,
            309.0,
            338.0,
            390.0,
            424.0,
            446.0,
            451.0,
            584.0,
            590.0,
            613.0,
            620.0,
            687.0,
        ]

        # convert to indices
        self.event_indices = self.match_times_to_indices(self.events)

        self.get_logger().info(f"Prepared {len(self.event_indices)} replay points")

        self.timer = self.create_timer(2.0, self.replay_next)

        self.current = 0

    # ============================================================
    # LOAD BAG
    # ============================================================

    def load_joint_states_mcap(self, bag_path):
        storage_options = StorageOptions(uri=bag_path, storage_id="mcap")
        converter_options = ConverterOptions("cdr", "cdr")

        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        rows = []

        while reader.has_next():
            topic, data, timestamp = reader.read_next()

            if topic == JOINT_STATE_TOPIC:
                msg = rclpy.serialization.deserialize_message(data, JointState)
                rows.append((timestamp, msg))

        return rows

    # ============================================================
    # MATCH TIMES → INDICES
    # ============================================================

    def match_times_to_indices(self, event_times):
        t0 = self.rows[0][0] * 1e-9

        indices = []

        times = [(ts * 1e-9 - t0) for ts, _ in self.rows]

        for et in event_times:
            idx = np.argmin(np.abs(np.array(times) - et))
            indices.append(idx)

        return indices

    # ============================================================
    # REPLAY
    # ============================================================

    def replay_next(self):
        if self.current >= len(self.event_indices):
            self.get_logger().info("Replay finished")
            rclpy.shutdown()
            return

        idx = self.event_indices[self.current]

        self.get_logger().info(
            f"\n=== Event {self.current+1}/{len(self.event_indices)} ==="
        )

        self.replay_segment(idx)

        self.current += 1
    
    def replay_segment(self, idx_center):
        window_sec = 2.0

        t0 = self.rows[0][0] * 1e-9
        times = np.array([ts * 1e-9 - t0 for ts, _ in self.rows])

        t_event = times[idx_center]

        # safer bounds
        idx_start = np.searchsorted(times, t_event - window_sec, side="left")
        idx_end = np.searchsorted(times, t_event + window_sec, side="right")

        idx_start = max(0, idx_start)
        idx_end = min(len(self.rows), idx_end)

        self.get_logger().info(
            f"Replaying window: {t_event-window_sec:.2f}s → {t_event+window_sec:.2f}s "
            f"(indices {idx_start}–{idx_end})"
        )

        # 🔑 CRITICAL: use real timing from bag
        prev_time = times[idx_start]

        for i in range(idx_start, idx_end):
            _, msg = self.rows[i]
            current_time = times[i]

            traj = JointTrajectory()
            traj.joint_names = list(msg.name)

            point = JointTrajectoryPoint()
            point.positions = list(msg.position)

            # 🔑 FIX: very small execution time (acts like position command)
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 1_000_000  # 1 ms

            traj.points.append(point)
            self.publisher.publish(traj)

            # 🔑 FIX: replay original timing
            SLOWDOWN = 2.0  # 2x slower → half speed

            dt = current_time - prev_time
            if dt > 0:
                time.sleep(dt * SLOWDOWN)

            prev_time = current_time


# ============================================================
# MAIN
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = SingularityReplayNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()