# Cartesian Interactive Marker

## Overview

This package provides a ROS 2 node that mirrors a set of Cartesian poses as 6-DOF **RViz
interactive markers**. Each marker tracks an "actual pose" coming from either a
`geometry_msgs/PoseStamped` topic or a live TF frame. Dragging a marker in RViz publishes the
edited pose as a new "target pose", and a companion topic continuously reports the pose error
between the target and the actual pose.

This is useful for interactively defining or nudging Cartesian targets (e.g. for an IK solver)
directly in RViz, while keeping a live view of how far the target currently is from where the
robot actually is.

---

## Parameters

`pose_topics`, `pose_tf`, and `target_topics` are index-matched (same length, same index `i`):

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `pose_topics` | string array | `[]` | At index `i`, the topic to subscribe to for this target's actual pose. Set to `""` to use `pose_tf[i]` instead. |
| `pose_tf` | string array | `[]` | At index `i`, the TF frame (e.g. `"arm_left/flange"`) to poll relative to `tf_base_frame`, used only when `pose_topics[i]` is `""`. |
| `target_topics` | string array | `[]` | At index `i`, the topic the edited target pose is published to. |
| `tf_base_frame` | string | `"base_link"` | Reference frame that `pose_tf` lookups (and their published target poses) are expressed in. |
| `topics_prefix` | string | node name | Namespace prefix for the error topics and the `reset_marker` subscription. |
| `world_aligned_controls` | bool | `true` | If `true`, every marker's move/rotate handles stay aligned with the global reference frame instead of rotating along with the marker's own orientation. |

An omitted array counts as all-empty-strings, i.e. unused at every index. Set the unused one of
`pose_topics[i]`/`pose_tf[i]` to `""` for each target you configure.

> **Note:**
> `pose_topics`, `pose_tf`, and `target_topics` must all be provided with equal, non-zero
> length, otherwise the node logs an error and stays idle (no markers, no publishers).

---

## Behavior

Each target gets its own interactive marker, initialized to the first actual pose received —
either the first message on its `pose_topics[i]` subscription (whose `header.frame_id` is then
reused for the published target pose), or the first successful TF lookup of its `pose_tf[i]`
frame (whose target pose is then expressed in `tf_base_frame`).

Moving a marker publishes its new pose on the matching `target_topics` entry. The companion
`<target_topic>_error` topic (`geometry_msgs/Twist`) reports the pose error (linear difference +
rotation-vector difference) between the target and the latest actual pose, updated whenever
either one changes.

### Marker naming

Marker names are derived from each target's topic/TF frame name (e.g.
`/cartesian_pose_controller/target_pose/left` → `left`). If two targets would end up with the
same name, both fold in another path segment from their topic until they're unique again.

### Resetting a marker

Publishing to `<topics_prefix>/reset_marker` (`std_msgs/String`, comma-separated regular
expressions) resets every target whose pose topic (or TF frame name), target name, or marker
name matches any of the patterns — the next actual pose received re-initializes its marker to
that pose:

```bash
ros2 topic pub --once /cartesian_teleop/reset_marker std_msgs/String "data: 'left,right'"
```

---

## Usage

```bash
ros2 run cartesian_interactive_marker cartesian_interactive_marker --ros-args \
  -p pose_topics:="['', '']" \
  -p pose_tf:="['arm_left/flange', 'arm_right/flange']" \
  -p target_topics:="['/cartesian_pose_controller/target_pose/left', '/cartesian_pose_controller/target_pose/right']" \
  -p tf_base_frame:=base_link
```

This tracks both arms' flanges via TF and publishes edited target poses on their respective
`target_pose` topics. Add an RViz `InteractiveMarkers` display pointed at the node's namespace
(default topic prefix: `/cartesian_teleop/update`) to see and drag the markers.
