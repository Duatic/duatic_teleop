# Duatic Teleop RViz

RViz-facing teleop nodes.

## Nodes

### duatic_interactive_marker

Mirrors a set of Cartesian poses as 6-DOF **RViz interactive markers**. Each marker tracks an
"actual pose" from either a `geometry_msgs/PoseStamped` topic or a live TF frame. Dragging a
marker publishes the edited pose as a "target pose", and a companion topic reports the pose
error between target and actual.

#### Parameters

`pose_topics`, `pose_tf`, and `target_topics` are index-matched (same length, same index `i`):

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `pose_topics` | string array | `[]` | Topic for target `i`'s actual pose. Set to `""` to use `pose_tf[i]` instead. |
| `pose_tf` | string array | `[]` | TF frame (e.g. `"arm_left/flange"`) to poll relative to `tf_base_frame`, used only when `pose_topics[i]` is `""`. |
| `target_topics` | string array | `[]` | Topic to publish target `i`'s edited pose to. |
| `tf_base_frame` | string | `"base_link"` | Reference frame for `pose_tf` lookups and their target poses. |
| `topics_prefix` | string | node name | Namespace prefix for the error topics and `reset_marker`. |
| `world_aligned_markers` | bool | `true` | Keep marker handles aligned with the global frame instead of the marker's own orientation. |

An omitted array counts as all-empty-strings. All three arrays must have equal, non-zero
length, otherwise the node logs an error and stays idle.

#### Behavior

Each target's marker initializes to its first received actual pose (topic message or TF
lookup). Moving it republishes on `target_topics`; a `<target_topic>_error` topic
(`geometry_msgs/Twist`) continuously reports the pose error vs. the latest actual pose.

Publishing to `<topics_prefix>/reset_marker` (`std_msgs/String`, comma-separated regexes)
resets any target whose pose source, target name, or marker name matches the given patterns

```bash
ros2 topic pub --once /duatic_interactive_marker/reset_marker std_msgs/String "data: 'flange'"
```

#### Usage

Matches the `duatic_dynaarm_single_example` mock demo's `cartesian_pose_controller`:

```bash
ros2 run duatic_teleop_rviz duatic_interactive_marker --ros-args \
  -p pose_topics:="['/cartesian_pose_controller/flange/pose']" \
  -p target_topics:="['/cartesian_pose_controller/flange/target']"
```

Or, equivalently, via the packaged launch file:

```bash
ros2 launch duatic_teleop_rviz duatic_interactive_marker.launch.py
```

Add an `InteractiveMarkers` display in RViz on the node's namespace to see and drag the marker.
