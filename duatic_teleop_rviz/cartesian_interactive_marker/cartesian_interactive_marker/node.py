#!/usr/bin/env python3
"""
ROS 2 node that mirrors a set of Cartesian poses as RViz interactive markers.

'pose_topics', 'pose_tf', and 'target_topics' are all index-matched (same length, same i).
At each index i, the target's "actual pose" comes from pose_topics[i] if it's non-empty;
otherwise from a live TF lookup of the pose_tf[i] frame relative to 'tf_base_frame'. Set the
unused one of the pair to "" for each index. An omitted array (default []) counts as
all-empty-strings, i.e. unused at every index.

Parameters:
  - pose_topics (string array): topics to subscribe to (geometry_msgs/PoseStamped), or "" at
    index i to use pose_tf[i] instead.
  - pose_tf (string array): TF frame names (e.g. "arm_left/flange") to poll relative to
    'tf_base_frame', used at index i only when pose_topics[i] is "".
  - target_topics (string array): topics to publish the edited target pose to
    (geometry_msgs/PoseStamped), index-matched with pose_topics/pose_tf as above.
  - tf_base_frame (string, default: "base_link"): reference frame that pose_tf lookups and
    their published target poses are expressed in.
  - topics_prefix (string, default: the node's name): namespace prefix for all error topics,
    e.g. "<topics_prefix>/<target_topic>_error".
  - world_aligned_controls (bool, default: false): if true, the move/rotate handles of every
    interactive marker stay aligned with the global reference frame instead of rotating along
    with the marker's own orientation.

Every target gets its own interactive marker, initialized to the first actual pose received —
either the first message on its pose_topics[i] subscription (whose header.frame_id is then
reused for the published target pose), or the first successful TF lookup of its pose_tf[i]
frame (whose target pose is then expressed in 'tf_base_frame').

For every entry, moving the interactive marker publishes its new pose on the matching
target_topics entry. A companion "<target_topic>_error" topic (geometry_msgs/Twist) reports
the pose error between the target and the latest actual pose (from either source).

Subscribing "<topics_prefix>/reset_marker" (std_msgs/String, comma-separated regular
expressions) resets every target(s) whose pose_topic (or TF frame name), target_name, or
marker_name matches any of the patterns, so the next actual pose re-initializes their marker.
"""

import re
from dataclasses import dataclass

from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Twist
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from tf2_ros import Buffer, TransformListener

# Poll rate for 'pose_tf' targets. TF has no per-target "new data" event to subscribe to like
# a topic does, so tf-sourced targets are instead re-checked on a fixed timer at this rate.
TF_POLL_PERIOD_SEC = 1.0 / 30.0

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
    pose_topic: str  # topic name, or (if is_tf) the TF frame name being polled
    topic_names: list[str]
    target_topic: str
    error_topic: str
    marker_name: str
    marker_color: tuple
    is_tf: bool = False  # actual pose comes from a TF lookup instead of a pose_topic subscription
    frame_id: str = "base_link"
    actual_pose: Pose = None
    target_pose: Pose = None
    initialized: bool = False

    @property
    def target_name(self) -> str:
        return self.topic_names[0]


def pose_error(target: Pose, actual: Pose) -> Twist:
    """Twist error = target - actual (linear diff + rotation-vector diff)."""
    twist = Twist()
    twist.linear.x = target.position.x - actual.position.x
    twist.linear.y = target.position.y - actual.position.y
    twist.linear.z = target.position.z - actual.position.z

    rotvec = (
        Rotation.from_quat(
            [
                target.orientation.x,
                target.orientation.y,
                target.orientation.z,
                target.orientation.w,
            ]
        )
        * Rotation.from_quat(
            [
                actual.orientation.x,
                actual.orientation.y,
                actual.orientation.z,
                actual.orientation.w,
            ]
        ).inv()
    ).as_rotvec()

    twist.angular.x = float(rotvec[0])
    twist.angular.y = float(rotvec[1])
    twist.angular.z = float(rotvec[2])
    return twist


class InteractiveCartesianNode(Node):
    def __init__(self):
        super().__init__("cartesian_teleop")

        dynamic_string_array = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("pose_topics", [], dynamic_string_array)
        self.declare_parameter("pose_tf", [], dynamic_string_array)
        self.declare_parameter("target_topics", [], dynamic_string_array)
        self.declare_parameter("tf_base_frame", "base_link")
        self.declare_parameter("topics_prefix", self.get_name())
        self.declare_parameter("world_aligned_controls", True)

        pose_topics = list(self.get_parameter("pose_topics").value)
        pose_tf = list(self.get_parameter("pose_tf").value)
        target_topics = list(self.get_parameter("target_topics").value)
        self.tf_base_frame: str = str(self.get_parameter("tf_base_frame").value)
        self.topics_prefix = self.get_parameter("topics_prefix").value.strip("/")
        self.world_aligned_controls = self.get_parameter("world_aligned_controls").value

        # 'pose_topics' and 'pose_tf' are index-matched against 'target_topics': at index i,
        # pose_topics[i] is used if non-empty, otherwise pose_tf[i] (see module docstring). An
        # omitted array is treated as all-empty-strings, i.e. "unused at every index".
        n = len(target_topics)
        pose_topics += [""] * (n - len(pose_topics))
        pose_tf += [""] * (n - len(pose_tf))

        if n == 0 or len(pose_topics) != n or len(pose_tf) != n:
            self.get_logger().error(
                "'pose_topics', 'pose_tf', and 'target_topics' must be non-empty and of equal "
                f"length (got {len(pose_topics)}, {len(pose_tf)}, {n}). Node is idle."
            )
            self.targets = []
            return

        self.server = InteractiveMarkerServer(self, self.get_name())

        self.targets: list[Target] = []
        unique_target_names: dict[str, int] = {}
        for i, (pose_topic, tf_frame, target_topic) in enumerate(
            zip(pose_topics, pose_tf, target_topics)
        ):
            is_tf = not pose_topic
            source = tf_frame if is_tf else pose_topic
            if not source:
                self.get_logger().warn(
                    f"Target {i} ignored: neither 'pose_topics[{i}]' nor 'pose_tf[{i}]' is set."
                )
                # Still append a slot (index=-1, like a failed-disambiguation target below) so
                # that self.targets[j].index == j keeps holding for every other j — indexing
                # throughout this class (including here) relies on that invariant.
                self.targets.append(
                    Target(
                        index=-1,
                        pose_topic="",
                        topic_names=["unset"],
                        target_topic=target_topic,
                        error_topic="",
                        marker_name="",
                        marker_color=MARKER_COLORS[i % len(MARKER_COLORS)],
                    )
                )
                continue

            # create target. frame_id defaults to tf_base_frame — the only reference frame this
            # node actually knows about at construction time. For a pose_topics target that's
            # just a placeholder until its first message overwrites it in _pose_cb(); for a
            # pose_tf target it's the real, permanent answer (see the module docstring).
            target = Target(
                index=i,
                pose_topic=source,
                topic_names=derive_topic_name(source),
                target_topic=target_topic,
                error_topic="",
                marker_name="",
                marker_color=MARKER_COLORS[i % len(MARKER_COLORS)],
                is_tf=is_tf,
                frame_id=self.tf_base_frame,
            )
            self.get_logger().info(f"Configuring Target {target.index}: {target.target_name}")
            # ensure uniqueness
            self._ensure_unique_target_name(target, unique_target_names)
            # store
            self.targets.append(target)

        # tf-sourced targets are re-checked by a timer instead of a subscription (see setup below)
        self._tf_targets: list[Target] = []

        for target in self.targets:
            if target.index != -1:
                target.error_topic = f"{self.topics_prefix}/{target.target_name}_error"
                target.marker_name = f"{self.get_name()}_{target.target_name}"
                target.target_pub = self.create_publisher(PoseStamped, target.target_topic, 10)
                target.error_pub = self.create_publisher(Twist, target.error_topic, 10)

                if target.is_tf:
                    # frame_id was already seeded to tf_base_frame at construction, and (unlike
                    # topic mode's header.frame_id) it never needs to change after that.
                    self._tf_targets.append(target)
                    source_desc = f"tf_frame='{target.pose_topic}' (base '{self.tf_base_frame}')"
                else:
                    self.create_subscription(
                        PoseStamped,
                        target.pose_topic,
                        lambda msg, idx=target.index: self._pose_cb(msg, idx),
                        10,
                    )
                    source_desc = f"pose_topic='{target.pose_topic}'"

                self.get_logger().info(
                    f"Target {target.index}: name='{target.target_name}' {source_desc} "
                    f"-> target_topic='{target.target_topic}' error_topic='{target.error_topic}'"
                )
            else:
                self.get_logger().warn(
                    f"Target ignored (pose_topic='{target.pose_topic}'): name could not be "
                    "disambiguated; no publisher/subscriber created."
                )

        if self._tf_targets:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_timer(TF_POLL_PERIOD_SEC, self._tf_poll_cb)

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
        patterns = [pattern.strip() for pattern in msg.data.split(",") if pattern.strip()]
        matched_indices = set()
        for pattern in patterns:
            try:
                regex = re.compile(pattern)
            except re.error as e:
                self.get_logger().warn(f"reset_marker: invalid regex '{pattern}': {e}")
                continue

            matched_indices.update(
                t.index
                for t in self.targets
                if t.index != -1
                and any(
                    regex.search(field) for field in (t.pose_topic, t.target_name, t.marker_name)
                )
            )

        if not matched_indices:
            self.get_logger().warn(f"reset_marker: no target matches '{msg.data}'")
            return

        for target_index in matched_indices:
            self.targets[target_index].initialized = False
            self.get_logger().info(
                f"reset_marker: reset target {target_index} ('{self.targets[target_index].marker_name}')"
            )

    def _pose_cb(self, msg: PoseStamped, index: int):
        target = self.targets[index]
        target.frame_id = msg.header.frame_id or target.frame_id
        self._update_actual_pose(target, msg.pose)

    def _tf_poll_cb(self):
        """Re-check every 'pose_tf' target's frame relative to 'tf_base_frame'."""
        for target in self._tf_targets:
            try:
                t = self.tf_buffer.lookup_transform(self.tf_base_frame, target.pose_topic, Time())
            except Exception as e:
                self.get_logger().warn(
                    f"Target {target.index}: TF lookup '{self.tf_base_frame}' -> "
                    f"'{target.pose_topic}' failed: {e}",
                    throttle_duration_sec=5.0,
                )
                continue

            pose = Pose()
            pose.position.x = t.transform.translation.x
            pose.position.y = t.transform.translation.y
            pose.position.z = t.transform.translation.z
            pose.orientation = t.transform.rotation
            self._update_actual_pose(target, pose)

    def _update_actual_pose(self, target: Target, pose: Pose):
        """Record a fresh actual-pose reading for `target`; initialize its marker on the first one."""
        target.actual_pose = pose

        if not target.initialized:
            target.target_pose = Pose()
            target.target_pose.position.x = pose.position.x
            target.target_pose.position.y = pose.position.y
            target.target_pose.position.z = pose.position.z
            target.target_pose.orientation.w = pose.orientation.w
            target.target_pose.orientation.x = pose.orientation.x
            target.target_pose.orientation.y = pose.orientation.y
            target.target_pose.orientation.z = pose.orientation.z
            target.initialized = True
            self._init_interactive_marker(target)
            self.server.applyChanges()
            source_desc = f"tf:{target.pose_topic}" if target.is_tf else target.pose_topic
            self.get_logger().info(
                f"Target {target.index}: marker initialized from '{source_desc}'"
            )

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

        for axis in ("x", "y", "z"):
            x, y, z, w = Rotation.from_euler(axis, 90, degrees=True).as_quat()
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
