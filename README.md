# duatic_teleop

[![Jazzy](https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/mbloechli/ca9e04f4b6ad286dc6ebc1fd2651d76b/raw/duatic_teleop-jazzy.json)](https://github.com/Duatic/duatic_teleop/actions/workflows/ci.yml)
[![Kilted](https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/mbloechli/ca9e04f4b6ad286dc6ebc1fd2651d76b/raw/duatic_teleop-kilted.json)](https://github.com/Duatic/duatic_teleop/actions/workflows/ci.yml)
[![Lyrical](https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/mbloechli/ca9e04f4b6ad286dc6ebc1fd2651d76b/raw/duatic_teleop-lyrical.json)](https://github.com/Duatic/duatic_teleop/actions/workflows/ci.yml)
[![Rolling](https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/mbloechli/ca9e04f4b6ad286dc6ebc1fd2651d76b/raw/duatic_teleop-rolling.json)](https://github.com/Duatic/duatic_teleop/actions/workflows/ci.yml)

This repository contains teleoperation interfaces for [Duatic](https://duatic.com/) robots:

* [duatic_teleop_arm](./duatic_teleop_arm) — leader-arm driver for the Elephant Robotics myController S570, publishing Cartesian pose targets via forward kinematics.
* [duatic_teleop_gamepad](./duatic_teleop_gamepad) — gamepad-based teleoperation, mapping axes and buttons to robot control commands.
* [duatic_teleop_ik](./duatic_teleop_ik) — JIT-compiled inverse-kinematics solver (PyRoki + JAX) that turns teleop Cartesian targets into joint trajectories.

# License

The contents are licensed under the BSD-3-Clause [license](LICENSE).\
Images in this repository are to be licensed separately if you want to use them for any other usecase than forking this repository.
3D models are to be licensed separately if you want to use them for any other usecase than running your purchased Duatic hardware with ROS 2.
Please open an issue in order to get in touch with us.

# Dependencies

All dependencies with their corresponding version are listed in the [repos.list](./repos.list).

| Name | Description | License
| ---  | --- | --- |
| [duatic_ros2control](https://github.com/Duatic/duatic_ros2control) | A wrapper library for integrating the Duatic DuaDrives into ros2_control hardware interfaces | BSD-3-Clause |
| [duatic_helpers](https://github.com/Duatic/duatic_helpers) | Common helper scripts and libraries, mostly ROS 2 related | BSD-3-Clause |
| [duatic_control](https://github.com/Duatic/duatic_control) | Shared controller launch and configuration package | BSD-3-Clause |
| [ethercat_sdk_master](https://github.com/Duatic/ethercat_sdk_master) | Object oriented wrapper around the soem_interface | BSD-3-Clause |
| [duatic_message_logger](https://github.com/Duatic/duatic_message_logger) | Logging library which allows logging with and without ROS | BSD-3-Clause |
| [rsl_drive_sdk](https://github.com/Duatic/rsl_drive_sdk) | Basic drive sdk for the DynaDrives | BSD-3-Clause |
| [soem_interface](https://github.com/Duatic/soem_interface) | Ethercat wrapper library around SOME | GPL v3 |
| [soem_vendor](https://github.com/Duatic/soem_vendor) | ROS 2 packaging for SOME | BSD-3-Clause |

# Usage

Each package documents its own setup and usage:

* [duatic_teleop_arm/elephant_s570](./duatic_teleop_arm/elephant_s570/README.md) — full teleoperation walkthrough, from connecting the leader arm to enabling robot motion.

For more detailed information please refer to the Duatic [documentation](https://docs.duatic.com).

# Contributing

Please see the [Contributing guide](./CONTRIBUTING.md)
