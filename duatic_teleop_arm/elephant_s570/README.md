# Duatic Teleoperation Guide using the Elephant S570 Arm

## Overview

This guide explains how to set up and use the **Elephant S570 Teleoperation Arm** to teleoperate a Duatic robot system (DXTR or DynaArm).

The setup supports two modes:

1. **Windows Publisher Mode (Recommended)**

   * Teleoperation arm connected to a Windows laptop.
   * Joint and button states are streamed to the robot via ROS 2.

2. **Direct Ubuntu Connection**

   * Teleoperation arm connected directly to the robot computer (or Ubuntu workstation).

---

# 1. Teleoperation Arm Setup

## Hardware Connection

Connect the Elephant S570 arm to your laptop using the supplied USB cable.

> **Important:**
> Even when connected via USB, the arm still needs to be charged. Use the provided charging cable to charge it before use. It can also be charged while operating the device.

Once connected:

1. The display should light up.
2. Press the **red power button** on the left side of the display.
3. The display should show:

   * Joint angles
   * Button states

### Verification

Move the teleoperation arms manually.

The displayed joint values should change accordingly.

> **Important:**
> We only mount ports into the docker container during creation. Therefore, as soon as you plugged the USB cable into your laptop, restart your container so you find the correct ports inside.

---

## Teleoperation Button

### Important Notes

* The buttons on the **left arm are currently not working**.
* Only use the buttons on the **right arm**.

### Teleoperation Enable Button

The green LED display on the right arm is also a button.

**Behavior:**

* Press and hold → Teleoperation is active.
* Release → Robot switches to freeze mode.

### Safety Recommendation

Whenever:

* You feel unsure,
* The robot approaches an obstacle,
* There is risk of collision or damage,

simply release the button immediately.

The freeze controller will stop robot motion.

---

# 2. Windows Setup (Recommended)

## Clone Repository

```bash
git clone git@github.com:Duatic/duatic_teleop_publisher_windows.git
cd duatic_teleop_publisher_windows
```

Switch to the required branch:

```bash
git checkout feat/add-elephant-batch-files
```

---

## Install Python

If Python is not installed:

Double-click:

```text
install_python.bat
```

The script will install:

* Required Python version
* Required dependencies

---

## Start Teleoperation Publisher

Connect the Elephant arm via USB.

Double-click:

```text
start_elephant_teleop.bat
```

The script will:

1. Verify Python installation
2. Install additional dependencies
3. Start publishing ROS 2 topics

Published topics:

```text
/teleop_arm/joint_states
/teleop_arm/buttons
```

The receiver destination can be configured inside the batch file:

* DynaArm VPN
* DXTR VPN
* DXTR Office

Enable the desired target by uncommenting the corresponding line.

---

## Verify Publisher

On the receiving machine:

```bash
ros2 topic echo /teleop_arm/joint_states
```

Move the teleoperation arms.

The values should update and match the values shown on the arm display.

---

# 3. Direct Ubuntu Connection

> For DXTR systems this would require connecting the arm directly to the NUC, which is currently **not recommended**.

---

## Start Hardware Driver

Connect the arm via USB.

Run:

```bash
ros2 run elephant_s570 s570_hardware_node
```

The node automatically detects the correct serial port.

### Manual Port Selection

If automatic detection fails:

Open the node source and modify:

```python
node = S570Publisher()
```

to:

```python
node = S570Publisher(port=<PORT>)
```

---

## Verify Hardware Driver

Run:

```bash
ros2 topic echo /teleop_arm/joint_states
```

Move the teleoperation arms.

The ROS topic values should match the values displayed on the arm screen.

---

# 4. Teleoperation Conversion Node

The following steps are identical whether you are using:

* DXTR
* DynaArm

and should be executed inside the corresponding ROS container.

---

## Start Teleoperation Mapping

Run:

```bash
ros2 run elephant_s570 elephant_s570_node
```

This node:

* Converts teleop joint states into target poses via forward kinematics
* Handles controller activation/deactivation
* Supports single-arm and dual-arm setups
* Automatically detects the robot configuration

---

# 5. Teleoperation Modes

The elephant_s570 node supports two teleoperation strategies.

---

## Mode 1: Remapping Deltas (Recommended)

Default setting:

```python
self.teleop_method = 'remapping_deltas'
```

### Behavior

When the teleop button is pressed:

1. Hand poses synchronize with current robot end-effector poses.
2. Relative motion is transferred.

Example:

```text
Move hand 10 cm left
→ Robot moves 10 cm left
```

---

## Mode 2: Mapped Action Spaces

Change in duatic_teleop/duatic_teleop_arm/elephant_s570/node.py:

```python
self.teleop_method = 'mapped_actionspaces'
```

### Behavior

The teleoperation workspace is mapped onto the robot workspace.

Example:

```text
Move hand 10 cm left
→ Robot may move 20 cm or more
```

because the robot arms are larger than the teleoperation arms.

### Recommendation

Use:

```python
remapping_deltas
```

unless specifically testing workspace scaling.

---

## Verification in RViz

Visualize the target pose topics.

When using:

```text
remapping_deltas
```

and pressing the teleop button:

* Target poses should align with the current end-effector poses.

---

# 6. Robot Preparation

Before enabling inverse kinematics:

1. Put the robot into **Free Drive** mode.
2. Move both end effectors to a safe position:

   * In front of the robot
   * Away from the base
3. Return the robot to **Freeze Mode**.

This minimizes the risk of collisions during testing.

---

# 7. Start Inverse Kinematics

## Important Safety Warning

This node continuously publishes commands directly to the robot's joint trajectory controllers.

Always verify:

```text
Robot is in Freeze Mode
```

before starting it.

---

Run:

```bash
ros2 run duatic_teleop_ik interactive_pyroki_node --ros-args -p use_interactive_markers:=False
```

The node:

1. Reads target poses.
2. Solves inverse kinematics.
3. Publishes joint trajectories to the robot controllers.

---

# 8. Verify IK Output

Keep the robot in Freeze Mode.

Press the teleoperation button to synchronize target poses.

Inspect:

```bash
ros2 topic echo /joint_states
```

```bash
ros2 topic echo /joint_trajectory_controller_arm_left/joint_trajectory
```

```bash
ros2 topic echo /joint_trajectory_controller_arm_right/joint_trajectory
```

### Expected Result

The commanded joint values should be reasonably close to the actual joint states.

Example:

Good:

```text
joint_state = 1.29
commanded   = 1.22
```

Bad:

```text
joint_state = 1.29
commanded   = -0.90
```

Small deviations are expected because IK solutions are approximations.

---

# 9. Enable Robot Motion

Once the IK output looks correct:

1. Switch robot to:

   * Joint Trajectory Control Mode
2. Unfreeze the robot

A small movement is expected as the robot aligns itself with the IK solution.

### Important

Be prepared to freeze the robot immediately if:

* Large unexpected motions occur
* Joint behavior appears incorrect
* Any collision risk is observed

---

# 10. Start Teleoperation

If all previous steps completed successfully:

1. Mount the teleoperation device comfortably.
2. Hold the teleoperation button.
3. Begin teleoperating the robot.

### Emergency Stop Behavior

At any moment:

```text
Release teleoperation button
```

The freeze controller will activate and stop robot motion.

Always use this as the primary safety mechanism during teleoperation.

### Collision Avoidance

A basic collision avoidance system is currently implemented to help protect the robot during teleoperation.

#### Base Protection

The robot end effectors are restricted to remain at least **20 cm in front of the robot base**. This prevents them from being commanded into the base structure and reduces the risk of self-collisions or hardware damage.

#### End-Effector Safety Distance

Every teleoperation command is checked against a minimum safety distance between the left and right end effectors.

If the end effectors get too close to each other:

* Any movement that would further reduce the distance is rejected.
* Only movements that increase the distance between the end effectors are allowed.

This ensures that the robot cannot accidentally command the arms into each other.

#### Recovering from Collision Avoidance

If you notice that the robot is no longer following your teleoperation commands as expected, the collision avoidance system may be active.

In this case, simply move your hands away from each other until the end effectors regain a safe separation distance. Once sufficient clearance has been restored, normal teleoperation will automatically resume.
