#!/usr/bin/env python3
"""
ROS 2 Node that forwards teleop arm data to rosbridge via WebSocket.

Subscribes:
    /teleop_arm/joint_states   (sensor_msgs/JointState)
    /teleop_arm/buttons        (sensor_msgs/Joy)

Publishes (via rosbridge):
    same topics and message types

Purpose:
    Bridge teleop arm data to external systems (e.g. web UI, remote control)
"""

import json
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState, Joy

import websocket


class TeleopRosbridgeBridge(Node):
    def __init__(self):
        super().__init__("publish_teleop_to_rosbridge_node")

        # --- Parameter ---
        self.declare_parameter("uri", "ws://192.168.89.157:9090")
        self.rosbridge_uri = self.get_parameter("uri").value

        # --- WebSocket ---
        self.ws = websocket.WebSocket()
        self.ws.connect(self.rosbridge_uri)
        self.advertised_topics = set()

        self.get_logger().info(f"Connected to rosbridge at {self.rosbridge_uri}")

        # --- Subscribers ---
        self.create_subscription(
            JointState,
            "/teleop_arm/joint_states",
            self._joint_cb,
            10,
        )

        self.create_subscription(
            Joy,
            "/teleop_arm/buttons",
            self._joy_cb,
            10,
        )

        self.get_logger().info("Subscribed to teleop topics")

    # ------------------------------------------------------------------ #
    #  Helpers                                                           #
    # ------------------------------------------------------------------ #

    def _advertise_once(self, topic: str, msg_type: str):
        if topic in self.advertised_topics:
            return

        advertise_msg = {
            "op": "advertise",
            "topic": topic,
            "type": msg_type,
        }

        try:
            self.ws.send(json.dumps(advertise_msg))
            self.advertised_topics.add(topic)
            self.get_logger().info(f"Advertised {topic} [{msg_type}]")
        except Exception as e:
            self.get_logger().error(f"Advertise failed: {e}")

    def _publish(self, topic: str, msg_dict: dict):
        publish_msg = {
            "op": "publish",
            "topic": topic,
            "msg": msg_dict,
        }

        try:
            self.ws.send(json.dumps(publish_msg))
        except Exception as e:
            self.get_logger().error(f"Publish failed: {e}")

    # ------------------------------------------------------------------ #
    #  Callbacks                                                         #
    # ------------------------------------------------------------------ #

    def _joint_cb(self, msg: JointState):
        topic = "/teleop_arm/joint_states"

        self._advertise_once(topic, "sensor_msgs/JointState")

        ros_msg = {
            "header": {
                "stamp": {
                    "sec": int(msg.header.stamp.sec),
                    "nanosec": int(msg.header.stamp.nanosec),
                },
                "frame_id": msg.header.frame_id,
            },
            "name": list(msg.name),
            "position": list(msg.position),
            "velocity": list(msg.velocity),
            "effort": list(msg.effort),
        }

        self._publish(topic, ros_msg)

    def _joy_cb(self, msg: Joy):
        topic = "/teleop_arm/buttons"

        self._advertise_once(topic, "sensor_msgs/Joy")

        ros_msg = {
            "header": {
                "stamp": {
                    "sec": int(msg.header.stamp.sec),
                    "nanosec": int(msg.header.stamp.nanosec),
                },
                "frame_id": msg.header.frame_id,
            },
            "axes": list(msg.axes),
            "buttons": list(msg.buttons),
        }

        self._publish(topic, ros_msg)


# ---------------------------------------------------------------------- #
#  Main                                                                  #
# ---------------------------------------------------------------------- #

def main(args=None):
    rclpy.init(args=args)
    node = TeleopRosbridgeBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()