# Duatic Teleop Gamepad

Drives a Duatic robot from a standard gamepad: jogging an arm in joint space, driving a
mobile base, and freedrive.

Operator documentation, the button map and the safety notes live in
[`doc/index.md`](./doc/index.md). This file is about the package itself.

## The node

`gamepad_interface` comes up immediately and wires itself as the robot appears, so it can be
started in any order relative to the robot stack. It keeps re-checking which controllers are
loaded and which trajectory topics exist, so a controller that arrives late is picked up and
one that goes away is dropped, without a restart.

```bash
ros2 run duatic_teleop_gamepad gamepad_interface --ros-args --params-file $(ros2 pkg prefix duatic_teleop_gamepad)/share/duatic_teleop_gamepad/config/gamepad_config.yaml
```

| Topic | Type | Direction |
| --- | --- | --- |
| `joy` | `sensor_msgs/Joy` | in, from the [`joy`](https://index.ros.org/p/joy/) node |
| `joint_states` | `sensor_msgs/JointState` | in, merged from every publisher |
| `joint_trajectory_controller_<component>/joint_trajectory` | `trajectory_msgs/JointTrajectory` | out, one per discovered controller |
| `cmd_vel_smoothed` | `geometry_msgs/TwistStamped` | out, platform velocity |
| `joy/set_feedback` | `sensor_msgs/JoyFeedback` | out, rumble |

It calls `controller_manager/list_controllers` and `controller_manager/switch_controller`,
and reads each trajectory controller's `joints` parameter. Every setting is a normal ROS
parameter; the defaults and what they mean are in
[`config/gamepad_config.yaml`](./config/gamepad_config.yaml).

## Layout

```
include/duatic_teleop_gamepad/   src/
├── gamepad_node                 the plumbing: subscriptions, timers, focus
├── gamepad_config               every parameter, declared once
├── input/                       what the pad is doing
│   └── gamepad_input              one Joy message into one GamepadInput, and the
│                                  mappings, limits and edge detection that takes
├── robot/                       what the robot is doing
│   ├── controller_manager_client  polls the listing, requests switches
│   ├── controller_snapshot        that listing, reduced to the managed controllers
│   ├── jtc_discovery              which trajectory controllers exist, and their joints
│   └── joint_state_cache          one view of the joints, merged and expiring
└── modes/                       what the gamepad is driving
    ├── base_mode                  the interface every mode implements, and the
    │                              context every mode is given to work with
    ├── mode_manager               which mode is active, and switching between them
    ├── freedrive_mode             hands the robot to its gravity compensation
    ├── jog_mode                   the jog groups, and the trajectories they stream
    └── drive_mode                 the velocity ramp, and the twists it publishes
```

Each layer only knows about the ones above it in that list: `modes/` reads `input/` and
`robot/`, and nothing reads `gamepad_node`.

One mode is one pair of files. The arithmetic each one is built from lives with it rather
than in a file of its own: `JogGroup` (the per-controller command integrator) in `jog_mode`,
`DriveRamp` (the acceleration limiter) in `drive_mode`. Both are plain classes that need no
node, which is why their tests do not need one either.

## How a stick reaches a controller

```
/joy ──> read_input() ──> GamepadInput ──> ModeManager::active() ──> the mode's publisher
          input rate                                                  publish rate
```

Three timers drive everything:

| Timer | Default | What runs |
| --- | --- | --- |
| input | `input_rate`, 100 Hz | read the pad, act on buttons, hand the reading to the active mode |
| publish | `jog_publish_rate`, 100 Hz | advance and publish the active mode's stream |
| discovery | `discovery_rate`, 1 Hz | re-check controllers and topics, settle the active mode |

The two command rates are separate on purpose: reading the pad is bounded by the `/joy`
topic, while streaming to a trajectory controller is bounded by what that controller wants
for smooth motion, which is a property of the arm.

## Modes

A mode owns the controllers that back it, the state it streams, and the publisher it streams
on. `ModeManager` owns the modes and decides which one is active: it offers the ones whose
controllers are loaded, cycles through them on the mode button, and adopts whatever the
active controllers amount to when something else switches them underneath.

A switch that has not taken effect within three seconds engages every freeze controller the
robot has, because a switch that only half happened can leave the arms with nothing holding
them, and from here a refused switch and a partial one look the same. Recovering from that
is the E-Stop's own reset, not something the gamepad can undo.

| Mode | Backed by | Streams |
| --- | --- | --- |
| Freedrive | `freedrive_controller` | nothing, the controller does the work |
| Jog | `joint_trajectory_controller*` | a trajectory point per publish tick |
| Drive | `mecanum_drive_controller`, `platform_velocity_controller` | a twist per input tick |

Adding one is a new file rather than another case in every switch. Implement `BaseMode`,
give it the controller prefixes that back it, and add it to the vector in `GamepadNode`'s
constructor, where the order is the order the mode button cycles in:

```cpp
class HoverMode : public BaseMode
{
public:
  std::string name() const override { return "hover"; }

  const std::vector<std::string>& controller_bases() const override
  {
    static const std::vector<std::string> bases = { "hover_controller" };
    return bases;
  }

  void on_input(const GamepadInput& input, double dt) override;
  void reset() override;
};
```

Only `name()` and `controller_bases()` have to be written. Everything else has a default
that does nothing, so a mode implements the hooks it actually uses: `on_input` for a mode
driven straight from the sticks, `publish` for one that streams, `reset` for one holding
state that must not survive the deadman being released.

The one rule a mode must honour is that `reset()` leaves it commanding nothing. It is called
on every edge that ends motion, and on every mode, not only the active one.

## Building and testing

```bash
colcon build --packages-select duatic_teleop_gamepad
```

```bash
colcon test --packages-select duatic_teleop_gamepad && colcon test-result --verbose
```

The tests mirror the source layout under `test/`. Almost all of them are plain unit tests
with no ROS context: the decision logic is written to take what it needs as arguments, so
controller listings, joint states and Joy messages are just values. `test_modes.cpp` is the
exception, because the real modes publish and so need a node.

What the tests cannot see is the graph: twice during this package's history a bug survived a
green suite and died on first contact with a live stack. Run against
`duatic_dxtr_example mock.launch.py` before trusting a change.
