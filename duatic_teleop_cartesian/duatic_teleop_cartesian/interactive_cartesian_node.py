#!/usr/bin/env python3
"""
ROS 2 node that mirrors a set of Cartesian pose topics as RViz interactive markers.

Parameters:
  - pose_topics (string array): topics to subscribe to (geometry_msgs/PoseStamped).
    Each one gets its own interactive marker, initialized to the first pose received.
  - target_topics (string array): topics to publish the edited target pose to
    (geometry_msgs/PoseStamped), index-matched with pose_topics.
  - topics_prefix (string, default: the node's name): namespace prefix for all error topics,
    e.g. "<topics_prefix>/<target_topic>_error".
  - world_aligned_controls (bool, default: false): if true, the move/rotate handles of every
    interactive marker stay aligned with the global reference frame instead of rotating along
    with the marker's own orientation.

Each marker's frame is taken from the header.frame_id of the pose_topics[i] subscription
(set on the first message received) and reused for the published target pose.

For every entry i, moving the interactive marker publishes its new pose on
target_topics[i]. A companion "<target_topics[i]>_error" topic
(geometry_msgs/Twist) reports the pose error between the target and the latest
pose received on pose_topics[i].

Subscribing "<topics_prefix>/reset_marker" (std_msgs/String, comma-separated names) resets
the matching target(s) — matched against pose_topic, target_name, or marker_name — so
the next incoming pose re-initializes their marker.
"""

from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Twist
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

MARKER_COLORS = [
    (1.0, 0.1, 0.1, 0.5),
    (0.1, 0.3, 1.0, 0.5),
    (0.1, 1.0, 0.3, 0.5),
    (1.0, 0.8, 0.1, 0.5),
    (0.8, 0.1, 1.0, 0.5),
]

KNOWN_TOPIC_SUFFIXES = {"pose", "twist", "acceleration", "effort"}
NAME_PREFIX_SEPARATOR = "_"
KNOWN_TOPIC_SEPARATORS = {NAME_PREFIX_SEPARATOR, "/", "-", " "}


def derive_topic_name(topic: str) -> list[str]:
    """[device_topic_name, *prefixes] for `topic`.

    The device topic name is the last path segment after repeatedly stripping a
    trailing known frame (e.g. 'pose') or separator (e.g. '/', '_') from the end.
    The prefixes are the remaining '/'-separated path segments, in order.
    """
    name = topic
    cut_name = True
    while cut_name:
        cut_name = False
        for token in KNOWN_TOPIC_SUFFIXES | KNOWN_TOPIC_SEPARATORS:
            if name.endswith(token):
                name = name[: -len(token)]
                cut_name = True
                break
    parts = name.split("/")
    device_name = parts[-1]
    prefixes = [p for p in parts[:-1] if p]
    if not device_name:
        device_name = topic.rsplit("/", 1)[-1]
    return [device_name, *prefixes]


@dataclass
class Target:
    """Per-marker subscription/publication state."""

    index: int
    pose_topic: str
    topic_names: list[str]
    target_topic: str
    error_topic: str
    marker_name: str
    marker_color: tuple
    frame_id: str = "base_link"
    actual_pose: Pose = None
    target_pose: Pose = None
    initialized: bool = False

    @property
    def target_name(self) -> str:
        return self.topic_names[0]


def quat_to_array(q) -> np.ndarray:
    """geometry_msgs/Quaternion -> [w, x, y, z]."""
    return np.array([q.w, q.x, q.y, q.z])


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def quat_to_rotvec(q: np.ndarray) -> np.ndarray:
    """Unit quaternion [w, x, y, z] -> rotation vector (axis * angle)."""
    if q[0] < 0.0:
        q = -q
    w = np.clip(q[0], -1.0, 1.0)
    v = q[1:]
    v_norm = np.linalg.norm(v)
    angle = 2.0 * np.arctan2(v_norm, w)
    if v_norm < 1e-9:
        return np.zeros(3)
    return (v / v_norm) * angle


def pose_error(target: Pose, actual: Pose) -> Twist:
    """Twist error = target - actual (linear diff + rotation-vector diff)."""
    twist = Twist()
    twist.linear.x = target.position.x - actual.position.x
    twist.linear.y = target.position.y - actual.position.y
    twist.linear.z = target.position.z - actual.position.z

    q_target = quat_to_array(target.orientation)
    q_actual = quat_to_array(actual.orientation)
    q_err = quat_multiply(q_target, quat_conjugate(q_actual))
    q_err /= np.linalg.norm(q_err)
    rotvec = quat_to_rotvec(q_err)

    twist.angular.x = float(rotvec[0])
    twist.angular.y = float(rotvec[1])
    twist.angular.z = float(rotvec[2])
    return twist


class InteractiveCartesianNode(Node):
    def __init__(self):
        super().__init__("cartesian_teleop")

        dynamic_string_array = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("pose_topics", [], dynamic_string_array)
        self.declare_parameter("target_topics", [], dynamic_string_array)
        self.declare_parameter("topics_prefix", self.get_name())
        self.declare_parameter("world_aligned_controls", True)

        pose_topics = list(self.get_parameter("pose_topics").value)
        target_topics = list(self.get_parameter("target_topics").value)
        self.topics_prefix = self.get_parameter("topics_prefix").value.strip("/")
        self.world_aligned_controls = self.get_parameter("world_aligned_controls").value

        if not pose_topics or len(pose_topics) != len(target_topics):
            self.get_logger().error(
                "'pose_topics' and 'target_topics' must be non-empty and of equal length "
                f"(got {len(pose_topics)} and {len(target_topics)}). Node is idle."
            )
            self.targets = []
            return

        self.server = InteractiveMarkerServer(self, self.get_name())

        self.targets: list[Target] = []
        unique_target_names: dict[str, int] = {}
        for i, (pose_topic, target_topic) in enumerate(zip(pose_topics, target_topics)):
            # create target
            target = Target(
                index=i,
                pose_topic=pose_topic,
                topic_names=derive_topic_name(pose_topic),
                target_topic=target_topic,
                error_topic="",
                marker_name="",
                marker_color=MARKER_COLORS[i % len(MARKER_COLORS)],
            )
            self.get_logger().info(f"Configuring Target {target.index}: {target.target_name}")
            # ensure uniqueness
            self._ensure_unique_target_name(target, unique_target_names)
            # store
            self.targets.append(target)

        for target in self.targets:
            if target.index != -1:
                target.error_topic = f"{self.topics_prefix}/{target.target_name}_error"
                target.marker_name = f"{self.get_name()}_{target.target_name}"
                target.target_pub = self.create_publisher(PoseStamped, target.target_topic, 10)
                target.error_pub = self.create_publisher(Twist, target.error_topic, 10)
                self.create_subscription(
                    PoseStamped,
                    target.pose_topic,
                    lambda msg, idx=target.index: self._pose_cb(msg, idx),
                    10,
                )

                self.get_logger().info(
                    f"Target {target.index}: name='{target.target_name}' pose_topic='{target.pose_topic}' "
                    f"-> target_topic='{target.target_topic}' error_topic='{target.error_topic}'"
                )
            else:
                self.get_logger().warn(
                    f"Target ignored (pose_topic='{target.pose_topic}'): name could not be "
                    "disambiguated; no publisher/subscriber created."
                )

        self.create_subscription(
            String, f"{self.topics_prefix}/reset_marker", self._reset_marker_cb, 10
        )

    def _ensure_unique_target_name(
        self, target: Target, unique_target_names: dict[str, int]
    ) -> bool:
        """Recursively disambiguate `target`'s name against `unique_target_names`.

        A name is never forgotten once used: on collision, both the colliding
        target and `target` fold their nearest remaining topic-name prefix into
        their name and are re-checked recursively. The retired name's dict entry
        is kept (index set to -1) so it can never resurface. Returns False (and
        logs an error) if `target` runs out of prefixes to disambiguate itself,
        in which case its creation must be aborted.
        """
        name = target.target_name
        owner_index = unique_target_names.get(name)

        if owner_index is None:
            unique_target_names[name] = target.index
            return True
        # renaming required
        if len(target.topic_names) <= 1:
            self.get_logger().error(
                f"Target {target.index}: cannot disambiguate name '{name}', no prefixes left."
            )
            target.index = -1  # invalidate target
            return False

        # rename target
        prefix = target.topic_names.pop()
        target.topic_names[0] = f"{prefix}{NAME_PREFIX_SEPARATOR}{target.topic_names[0]}"
        self.get_logger().info(
            f"Target {target.index}: renamed '{name}' to '{target.target_name}'."
        )
        # recursively ensure target name uniqueness
        if not self._ensure_unique_target_name(target, unique_target_names):
            return False

        # rename other target
        if owner_index != -1:
            unique_target_names[name] = -1  # invalidate owner index
            if not self._ensure_unique_target_name(self.targets[owner_index], unique_target_names):
                return False

        # no error -> success
        return True

    def _reset_marker_cb(self, msg: String):
        names = [name.strip() for name in msg.data.split(",") if name.strip()]
        for name in names:
            target = next(
                (t for t in self.targets if name in (t.pose_topic, t.target_name, t.marker_name)),
                None,
            )
            if target is None:
                self.get_logger().warn(f"reset_marker: no target matches '{name}'")
                continue
            target.initialized = False
            self.get_logger().info(
                f"reset_marker: reset target {target.index} ('{target.marker_name}') matching '{name}'"
            )

    def _pose_cb(self, msg: PoseStamped, index: int):
        target = self.targets[index]
        target.actual_pose = msg.pose
        target.frame_id = msg.header.frame_id or target.frame_id

        if not target.initialized:
            target.target_pose = Pose()
            target.target_pose.position.x = msg.pose.position.x
            target.target_pose.position.y = msg.pose.position.y
            target.target_pose.position.z = msg.pose.position.z
            target.target_pose.orientation.w = msg.pose.orientation.w
            target.target_pose.orientation.x = msg.pose.orientation.x
            target.target_pose.orientation.y = msg.pose.orientation.y
            target.target_pose.orientation.z = msg.pose.orientation.z
            target.initialized = True
            self._init_interactive_marker(target)
            self.server.applyChanges()
            self.get_logger().info(f"Target {index}: marker initialized from '{target.pose_topic}'")

        self._publish_error(target)

    def _init_interactive_marker(self, target: Target):
        """Create a 6-DOF interactive marker at the target's current pose."""
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = target.frame_id
        int_marker.name = target.marker_name
        int_marker.description = f"{self.get_name()} ({target.target_name})"
        int_marker.pose = target.target_pose
        int_marker.scale = 0.2

        sphere_marker = Marker()
        sphere_marker.type = Marker.SPHERE
        sphere_marker.scale.x = 0.05
        sphere_marker.scale.y = 0.05
        sphere_marker.scale.z = 0.05
        r, g, b, a = target.marker_color
        sphere_marker.color.r = r
        sphere_marker.color.g = g
        sphere_marker.color.b = b
        sphere_marker.color.a = a

        sphere_control = InteractiveMarkerControl()
        sphere_control.always_visible = True
        sphere_control.markers.append(sphere_marker)
        int_marker.controls.append(sphere_control)

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
                control.orientation_mode = (
                    InteractiveMarkerControl.FIXED
                    if self.world_aligned_controls
                    else InteractiveMarkerControl.INHERIT
                )
                int_marker.controls.append(control)

        self.server.insert(
            int_marker,
            feedback_callback=lambda fb, idx=target.index: self._process_feedback(fb, idx),
        )

    def _process_feedback(self, feedback, index: int):
        if feedback.event_type != feedback.POSE_UPDATE:
            return

        target = self.targets[index]
        target.target_pose = feedback.pose

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = target.frame_id
        msg.pose = feedback.pose
        target.target_pub.publish(msg)

        self._publish_error(target)

    def _publish_error(self, target: Target):
        if target.target_pose is None or target.actual_pose is None:
            return
        target.error_pub.publish(pose_error(target.target_pose, target.actual_pose))


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveCartesianNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
