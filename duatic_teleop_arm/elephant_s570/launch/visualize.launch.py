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

"""Launch file for S570 visualization in RViz.

Starts:
  - robot_state_publisher with S570 URDF (publishes TF for /s570/joint_states)
  - elephant_s570_node (teleop node that also republishes URDF-compatible joint states)
  - RViz2 with a config showing the S570 robot model and target markers

Usage:
    ros2 launch elephant_s570 visualize.launch.py
    ros2 launch elephant_s570 visualize.launch.py arm_side:=right dual_arm_robot:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("elephant_s570")
    urdf_path = os.path.join(pkg_share, "urdf", "s570.urdf")
    rviz_config_path = os.path.join(pkg_share, "rviz", "s570.rviz")

    with open(urdf_path) as f:
        robot_description = f.read()

    arm_side_arg = DeclareLaunchArgument("arm_side", default_value="both")
    dual_arm_arg = DeclareLaunchArgument("dual_arm_robot", default_value="true")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="s570",
        parameters=[{"robot_description": robot_description}],
        remappings=[("joint_states", "/s570/joint_states")],
        output="screen",
    )

    elephant_s570_node = Node(
        package="elephant_s570",
        executable="elephant_s570_node",
        parameters=[
            {
                "arm_side": LaunchConfiguration("arm_side"),
                "dual_arm_robot": LaunchConfiguration("dual_arm_robot"),
            }
        ],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen",
    )

    return LaunchDescription(
        [
            arm_side_arg,
            dual_arm_arg,
            robot_state_publisher,
            elephant_s570_node,
            rviz_node,
        ]
    )
