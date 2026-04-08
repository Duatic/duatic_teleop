#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os


def launch_setup(context, *args, **kwargs):

    # -----------------------------
    # S570 URDF laden
    # -----------------------------
    pkg_s570 = get_package_share_directory("elephant_s570")
    urdf_path = os.path.join(pkg_s570, "urdf", "s570.urdf")

    with open(urdf_path) as f:
        s570_description = f.read()

    # -----------------------------
    # DynaArm Packages
    # -----------------------------
    pkg_dynaarm_bringup = FindPackageShare("duatic_dynaarm_bringup")
    pkg_dynaarm_description = FindPackageShare("duatic_dynaarm_description")

    dynaarm_urdf = PathJoinSubstitution(
        [
            FindPackageShare("duatic_dynaarm_single_example_description"),
            "urdf",
            "dynaarm_single_example.urdf.xacro",
        ]
    )

    # -----------------------------
    # DynaArm Bringup
    # -----------------------------
    dynaarm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_dynaarm_bringup,
                "launch",
                "mock.launch.py"
            ])
        ),
        launch_arguments={
            "namespace": "",
            "urdf_file_path": dynaarm_urdf,
            "controllers_config": LaunchConfiguration("controllers_config"),
        }.items(),
    )

    # -----------------------------
    # S570 robot_state_publisher
    # -----------------------------
    s570_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="s570",
        parameters=[{
            "robot_description": s570_description,
        }],
        remappings=[
            ("joint_states", "/s570/joint_states"),
        ],
        output="screen",
    )

    # -----------------------------
    # S570 TELEOP NODE (EINMAL!)
    # -----------------------------
    s570_node = Node(
        package="elephant_s570",
        executable="elephant_s570_node",
        parameters=[{
            "arm_side": "both",
            "dual_arm_robot": True,
            "robot_base_frame": "base_link",
            "robot_ee_frame": "flange",
        }],
        output="screen",
    )

    # -----------------------------
    # Gamepad
    # -----------------------------
    joy_node = Node(
        package="joy",
        executable="game_controller_node",
        parameters=[{"autorepeat_rate": 100.0}],
        output="screen",
    )

    # -----------------------------
    # Optional TF Fix
    # -----------------------------
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base"],
        condition=IfCondition(LaunchConfiguration("use_base_link_tf")),
    )

    # -----------------------------
    # RViz (ein einziges!)
    # -----------------------------
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([pkg_dynaarm_description, "config", "config.rviz"]),
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )

    return [
        dynaarm_bringup,
        s570_rsp,
        s570_node,
        joy_node,
        static_tf,
        rviz,
    ]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            "controllers_config",
            default_value=get_package_share_directory(
                "duatic_dynaarm_single_example"
            ) + "/config/controllers.yaml",
            description="Path to the controllers config file",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "use_base_link_tf",
            default_value="false",
        ),
        OpaqueFunction(function=launch_setup),
    ])