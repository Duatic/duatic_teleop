# Gamepad Teleoperation

`duatic_teleop_gamepad` maps a standard gamepad (Xbox- or PS4/PS5-style) to teleoperation of a
Duatic robot: driving a mobile base, jogging an arm in joint space, freedrive, and gripper control.

Which of these are actually available at runtime depends on the robot's morphology — a
`mobile_manipulator` gets all of them, a `single_arm` or `multi_arm` robot gets manipulation and
freedrive only, a `mobile_base` gets driving only.

```{danger}
The deadman switch is the primary safety mechanism. Motion is only ever commanded while it is
held down; releasing it — or the robot entering freeze/E-Stop — immediately stops all motion.
Always keep a clear path to release the deadman switch while teleoperating.
```

## Prerequisites

* A joystick/gamepad connected to the machine running this node.
* The standard ROS 2 [`joy`](https://index.ros.org/p/joy/) node, publishing `/joy` from that
  device.
* A robot stack already running, exposing the ros2_control controllers this package looks for
  (see [Available controllers](#available-controllers) below).

## Running

Start the joystick driver and the gamepad interface:

```bash
ros2 run joy joy_node
ros2 run duatic_teleop_gamepad gamepad_interface
```

On startup the node waits for the robot to be discoverable, inspects which ros2_control
controllers are actually running, and enables only the high-level controllers whose
requirements are met.

## Controls

| Input | Function |
| --- | --- |
| Dead man switch (Right Shoulder) | Hold to allow motion. Release to stop and freeze the active controller. |
| Menu button | Switch to the next available high-level controller (Freedrive → Joint Trajectory → Platform Drive, whichever are available). |
| D-Pad | Switch which robot component is focused (e.g. `arm_left`, `arm_right`, `hip`, `platform`). Reported as axes on Xbox-style pads and as individual buttons on PS4/PS5-style pads — both are handled automatically. |
| Face Bottom | Gripper open/close for the focused arm (works independently of the active high-level controller, as long as the system isn't frozen). |
| Face Top | Move to home pose. |
| Face Right | Move to sleep pose. |

Button/axis indices are all remappable in
[`config/gamepad_config.yaml`](../config/gamepad_config.yaml) to support different gamepad
layouts.

### Available controllers

**Joint Trajectory (jog mode)** — moves the focused arm's joints directly:

| Input | Joint |
| --- | --- |
| Left stick X | Joint 1 |
| Left stick Y | Joint 2 |
| Right stick Y | Joint 3 |
| Right stick X | Joint 4 |
| Triggers (right − left) | Joint 5 |
| Left/Right stick click | Joint 6 (wrist rotation) |

Only one stick axis drives a given joint pair at a time — whichever axis is furthest from
center becomes dominant, so diagonal stick motion doesn't cause both joints to creep at once.

**Platform Drive** — mecanum-style base driving, active when `platform` is focused:

| Input | Motion |
| --- | --- |
| Left stick Y | Forward / backward |
| Left stick X | Strafe left / right |
| Right stick X | Rotate |

**Freedrive** — hands-free gravity-compensated mode; switching to it activates the robot's
`freedrive_controller`, no further gamepad input is needed while it's active.

```{tip}
Switching the active controller, changing focus, or releasing the deadman switch always resets
the newly active controller first, so motion never resumes from a stale target.
```

```{toctree}
:hidden:
```
