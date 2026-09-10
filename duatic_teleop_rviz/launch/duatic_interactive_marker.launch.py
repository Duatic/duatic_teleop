# Copyright 2026 Duatic AG
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions, and
#    the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions, and
#    the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
#    promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Launch file for the duatic_interactive_marker node.

Exposes every node parameter and warns (or fails) on problematic argument combinations.

Usage:
    ros2 launch duatic_teleop_rviz duatic_interactive_marker.launch.py
    ros2 launch duatic_teleop_rviz duatic_interactive_marker.launch.py \
        pose_topics:=/cartesian_pose_controller/flange/pose \
        target_topics:=/cartesian_pose_controller/flange/target
"""

import yaml

import launch.logging
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

logger = launch.logging.get_logger("duatic_interactive_marker.launch")


def _string_list_arg(context, content):
    """Resolve launch argument `content` to a string list, for a node parameter expecting one."""
    raw = LaunchConfiguration(content).perform(context)
    try:
        value = yaml.safe_load(raw)
    except yaml.YAMLError as e:
        if raw.strip().startswith("["):
            raise ValueError(
                f"Launch argument '{content}'='{raw}' looks like an array but is not valid "
                f"YAML: {e}"
            ) from e
        value = raw
    if isinstance(value, list):
        return [str(v) for v in value]
    return [raw]


def launch_setup(context, *args, **kwargs):
    pose_topics = _string_list_arg(context, "pose_topics")
    pose_tf = _string_list_arg(context, "pose_tf")
    target_topics = _string_list_arg(context, "target_topics")
    topics_prefix = LaunchConfiguration("topics_prefix").perform(context)

    # Index-match 'pose_topics'/'pose_tf' against 'target_topics'
    n = len(target_topics)
    pose_topics += [""] * (n - len(pose_topics))
    pose_tf += [""] * (n - len(pose_tf))

    # Cross-check the resolved arguments before forwarding:
    if n == 0 or len(pose_topics) != n or len(pose_tf) != n:
        logger.warning(
            "'pose_topics', 'pose_tf', and 'target_topics' should be non-empty and of equal "
            f"length (got {len(pose_topics)}, {len(pose_tf)}, {n}); the node will stay idle."
        )
    for i, (pose_topic, tf_frame, target_topic) in enumerate(
        zip(pose_topics, pose_tf, target_topics)
    ):
        if pose_topic and tf_frame:
            logger.warning(
                f"Target {i}: both pose_topics[{i}]='{pose_topic}' and pose_tf[{i}]='{tf_frame}' "
                "are set; the node uses pose_topics and ignores pose_tf. Set one to '' to "
                "disambiguate."
            )
        elif not pose_topic and not tf_frame:
            logger.warning(
                f"Target {i}: neither pose_topics[{i}] nor pose_tf[{i}] is set; the node will "
                "ignore this target (no marker, no publishers)."
            )
        if not target_topic:
            raise ValueError(f"Target {i}: target_topics[{i}] is empty; it must be set.")

    parameters = {
        "pose_topics": pose_topics,
        "pose_tf": pose_tf,
        "target_topics": target_topics,
        "tf_base_frame": LaunchConfiguration("tf_base_frame"),
        "world_aligned_markers": LaunchConfiguration("world_aligned_markers"),
    }
    # Omitted (rather than forwarded as "") so the node falls back to its own default (its name).
    if topics_prefix:
        parameters["topics_prefix"] = topics_prefix

    duatic_interactive_marker_node = Node(
        package="duatic_teleop_rviz",
        executable="duatic_interactive_marker",
        parameters=[parameters],
        output="screen",
    )

    return [duatic_interactive_marker_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "pose_topics",
            default_value="/cartesian_pose_controller/flange/pose",
            description="Topic for target i's actual pose. Set to '' to use pose_tf[i] instead. "
            "Single topic or YAML string array.",
        ),
        DeclareLaunchArgument(
            "pose_tf",
            default_value="",
            description="TF frame (e.g. 'flange') to poll relative to tf_base_frame, used only "
            "when pose_topics[i] is ''. Single frame or YAML string array.",
        ),
        DeclareLaunchArgument(
            "target_topics",
            default_value="/cartesian_pose_controller/flange/target",
            description="Topic to publish target i's edited pose to. Single topic or YAML "
            "string array.",
        ),
        DeclareLaunchArgument(
            "tf_base_frame",
            default_value="base_link",
            description="Reference frame for pose_tf lookups and their target poses.",
        ),
        DeclareLaunchArgument(
            "topics_prefix",
            default_value="",
            description="Namespace prefix for the error topics and reset_marker. Left as '', "
            "the node uses its own name.",
        ),
        DeclareLaunchArgument(
            "world_aligned_markers",
            default_value="true",
            description="Keep marker handles aligned with the global frame instead of the "
            "marker's own orientation.",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
