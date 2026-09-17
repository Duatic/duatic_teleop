#!/usr/bin/env python3

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

"""Standalone mock DXTR + interactive_pyroki_node demo, for testing IK self-collision avoidance
locally with interactive markers, without hardware or the elephant teleop device.

Drag the 6-DOF markers in RViz (Interact tool) to move each arm's flange; self_collision_cost
resists the elbow/flange penetrating the torso. Soft cost, not a hard limit - v1, not a guarantee.

Usage:
    ros2 launch duatic_teleop_ik dxtr_mock_pyroki_demo.launch.py

RViz won't show the markers until you add an "InteractiveMarkers" display with
Update Topic "/pyroki_target/update" (one-time, not part of config_mock.rviz).
"""

import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    doc = xacro.parse(open(LaunchConfiguration("urdf_file_path").perform(context)))
    xacro.process_doc(doc, mappings={"mode": "mock", "collision": "simple"})

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": doc.toxml()}],
    )

    # Static world -> base_link chain: the mobile base isn't being driven in this test.
    odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
        output="screen",
    )
    map_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[LaunchConfiguration("controllers_config")],
        output={"stdout": "screen", "stderr": "screen"},
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("duatic_control"), "launch", "control.launch.py"])
        ),
        launch_arguments={"config_path": LaunchConfiguration("controllers_config")}.items(),
    )

    pyroki_node = Node(
        package="duatic_teleop_ik",
        executable="interactive_pyroki_node",
        output="screen",
        parameters=[
            {
                "use_interactive_markers": True,
                "solve_mode": LaunchConfiguration("solve_mode"),
                "self_collision_margin": LaunchConfiguration("self_collision_margin"),
                "self_collision_weight": LaunchConfiguration("self_collision_weight"),
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([FindPackageShare("duatic_dxtr_bringup"), "config", "config_mock.rviz"]),
        ],
        output={"both": "log"},
    )

    return [robot_state_publisher, odom_tf, map_tf, controller_manager, control, pyroki_node, rviz]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "urdf_file_path",
            default_value=get_package_share_directory("duatic_dxtr_example_description")
            + "/urdf/dxtr_example_flange.urdf.xacro",
            description="Full-DXTR URDF xacro to mock (default: bare flange, no end-effector tool)",
        ),
        DeclareLaunchArgument(
            "controllers_config",
            default_value=get_package_share_directory("duatic_teleop_ik")
            + "/config/dxtr_mock_controllers.yaml",
            description="ros2_control config with the arm JTCs active (no elephant node here to "
            "switch them in)",
        ),
        DeclareLaunchArgument(
            "solve_mode",
            default_value="decoupled",
            description="interactive_pyroki_node solve mode (default matches the elephant demo)",
        ),
        DeclareLaunchArgument("self_collision_margin", default_value="0.05"),
        DeclareLaunchArgument("self_collision_weight", default_value="50.0"),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
