#!/usr/bin/env python3

# Copyright 2026 Duatic AG
# Duatic Commercial License 1.0 (DCL-1)

from sensor_msgs.msg import JoyFeedback
from rclpy.node import Node


class GamepadFeedback:
    """
    A helper class to send force-feedback (rumble) commands via the ROS joy feedback mechanism.
    """

    def __init__(self, node: Node):
        """
        Initializes the GamepadFeedback instance.

        :param node: The ROS node used for creating publishers and logging.
        :param topic: The topic to publish JoyFeedbackArray messages on.
        """
        self.node = node
        self.publisher = self.node.create_publisher(JoyFeedback, "joy/set_feedback", 10)

    def send_feedback(self, intensity: float):
        """
        Sends a rumble feedback command.

        :param intensity: A value between 0 and 1 indicating the feedback intensity.
        """

        feedback = JoyFeedback()
        feedback.type = JoyFeedback.TYPE_RUMBLE
        feedback.id = 0
        feedback.intensity = intensity
        self.publisher.publish(feedback)
