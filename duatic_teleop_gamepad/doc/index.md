# Gamepad Teleoperation

`duatic_teleop_gamepad` maps a standard gamepad (Xbox- or PS4/PS5-style) to teleoperation of a
Duatic robot: driving a mobile base, jogging an arm in joint space, and freedrive.

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
```

```bash
ros2 run duatic_teleop_gamepad gamepad_interface --ros-args --params-file $(ros2 pkg prefix duatic_teleop_gamepad)/share/duatic_teleop_gamepad/config/gamepad_config.yaml
```

The node starts immediately and wires itself up as the robot appears, so it can be launched
in any order relative to the robot. It keeps re-checking which controllers are loaded and
which trajectory topics exist, so a controller that arrives late is picked up and one that
goes away is dropped, without a restart.

Every setting lives in [`config/gamepad_config.yaml`](../config/gamepad_config.yaml) and is a
normal ROS parameter, so it can be overridden from a launch file or with `ros2 param set`.

## Controls

| Input | Function |
| --- | --- |
| Dead man switch (Right Shoulder) | Hold to allow motion. Release to stop and freeze the active controller. |
| Menu button | Switch to the next available high-level controller (Freedrive → Joint Trajectory → Platform Drive, whichever are available). |
| D-Pad | Focus the component the active mode acts on: up `hip`, left `arm_right`, right `arm_left`. Down is unassigned. Reported as axes on Xbox-style pads and as individual buttons on PS4/PS5-style pads — both are handled automatically. |

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

An axis past `jog.dominant_axis_threshold` counts as committed, and while one is, the
other axis of that stick has to clear the same threshold to drive its joint as well. So a
light diagonal doesn't creep both joints while a committed one still moves both. Setting
that threshold equal to `jog.deadzone` lets both axes drive together.

**Platform Drive** — mecanum-style base driving, reached with the mode button:

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

## Modes and controllers

Which modes are on offer depends on which controllers are loaded: jogging needs a trajectory
controller, freedrive needs a freedrive controller, and driving needs a drive controller. A
controller is spawned only where there is something for it to drive, so its presence is the
whole test.

The component each trajectory controller drives is read from its own name: a controller
called `joint_trajectory_controller_arm_left` drives `arm_left`, and that is the name the
D-Pad focuses. A single-arm robot spawning the bare `joint_trajectory_controller` has one
unnamed component.

`managed_controllers` lists the controllers this node is allowed to switch. Anything not
matching one of those prefixes is invisible to it and can never be deactivated by a mode
change, which is what keeps broadcasters and gravity compensation running.
`protected_controllers` lists controllers that lose state when stopped, such as the drive
controller and its odometry, and are left running across mode changes.

If another node switches controllers underneath, the gamepad follows rather than carrying on
with a mode the robot is no longer in. An explicit mode choice stands as long as its own
controllers are still running.

```{warning}
A mode switch that does not take effect within three seconds engages the freeze
controllers, because a switch that only half happened can leave the arms with nothing
holding them. The robot then has to be released the way any E-Stop is, by holding the
emergency stop button; the gamepad cannot clear it.
```

```{note}
Jogging streams trajectory points that end while still moving, which a
JointTrajectoryController rejects unless `allow_nonzero_velocity_at_trajectory_end` is true.
The node checks this for each controller it discovers and says so by name at startup if it
is missing, because otherwise the controller silently drops every command.
```
