#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy

from pymycobot import Exoskeleton
import serial.tools.list_ports

BAUD_RATE = 1_000_000

JOINT_NAMES = [
    "s570_left/joint_1",
    "s570_left/joint_2",
    "s570_left/joint_3",
    "s570_left/joint_4",
    "s570_left/joint_5",
    "s570_left/joint_6",
    "s570_left/joint_7",
    "s570_right/joint_1",
    "s570_right/joint_2",
    "s570_right/joint_3",
    "s570_right/joint_4",
    "s570_right/joint_5",
    "s570_right/joint_6",
    "s570_right/joint_7",
]


def auto_detect_port():
    for port in serial.tools.list_ports.comports():
        desc = (port.description or "").lower()
        if "usb" in desc or "tty" in desc or "s570" in desc:
            return port.device
    return None


class S570Publisher(Node):
    def __init__(self, port=None):
        super().__init__("s570_hardware_node")

        self.joint_pub = self.create_publisher(JointState, "/teleop_arm/joint_states", 10)
        self.button_pub = self.create_publisher(Joy, "/teleop_arm/buttons", 10)

        port = port or auto_detect_port()
        if port is None:
            self.get_logger().error("No serial port found")
            raise RuntimeError("No port")

        self.get_logger().info(f"Connecting to S570 on {port}")
        self.robot = Exoskeleton(port)

        self.get_logger().info("Starting to publish buttons and joint_states")
        self.timer = self.create_timer(1.0 / 50.0, self.loop)

    def loop(self):
        data = self.robot.get_all_data()
        if data is None:
            return

        left_data, right_data = data

        now = self.get_clock().now().to_msg()

        # --- JointState ---
        joint_msg = JointState()
        joint_msg.header.stamp = now
        joint_msg.name = JOINT_NAMES

        angles = [math.radians(left_data[i]) for i in range(7)]
        angles += [math.radians(right_data[i]) for i in range(7)]

        joint_msg.position = angles

        self.joint_pub.publish(joint_msg)

        # --- Joy ---
        joy_msg = Joy()
        joy_msg.header.stamp = now
        joy_msg.header.frame_id = "s570"

        joy_msg.buttons = [int(left_data[i]) for i in range(7, 11)] + [
            int(right_data[i]) for i in range(7, 11)
        ]

        joy_msg.axes = [
            (left_data[11] - 128.0) / 128.0,
            (left_data[12] - 128.0) / 128.0,
            (right_data[11] - 128.0) / 128.0,
            (right_data[12] - 128.0) / 128.0,
        ]

        self.button_pub.publish(joy_msg)


def main():
    rclpy.init()
    node = S570Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
