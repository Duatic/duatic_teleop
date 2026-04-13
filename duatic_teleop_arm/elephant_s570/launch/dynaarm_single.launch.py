from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os


def generate_launch_description():

    # =========================
    # Launch Arguments
    # =========================
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='minimal',
        description="Mode: 'minimal' or 'full'"
    )

    mode = LaunchConfiguration('mode')

    # =========================
    # Include real robot launch
    # =========================
    real_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('duatic_dynaarm_single_example'),
                'launch',
                'real.launch.py'
            )
        )
    )
    # NOTE:
    # EtherCAT bus is configured inside real.launch.py.
    # For new machines, ensure correct 'ethercat_bus' parameter is set there.

    # =========================
    # Gamepad Interface
    # IMPORTANT: gamepad should only start after controllers loaded, wait 5sec for that
    # =========================
    gamepad_node = TimerAction(
        period=10.0,  # adjust if needed (2–5s typical)
        actions=[
            Node(
                package='duatic_gamepad_interface',
                executable='gamepad_interface',
                name='gamepad_interface',
                output='screen'
            )
        ]
    )

    # =========================
    # Pyroki IK Node
    # =========================
    pyroki_node = Node(
        package='duatic_teleop_ik',
        executable='interactive_pyroki_node',
        name='interactive_pyroki_node',
        parameters=[{
            'use_interactive_markers': False
        }],
        output='screen'
    )

    # =========================
    # Elephant S570 Node (FULL mode only)
    # =========================
    elephant_node = Node(
        package='elephant_s570',
        executable='elephant_s570_node',
        name='elephant_s570_node',
        parameters=[{
            'dual_arm_robot': False
        }],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'full'"])
        )
    )

    # =========================
    # (Optional) Hardware Node Notes
    # =========================
    # Ubuntu:
    #   ros2 run elephant_s570 s570_hardware_node
    #
    # Windows:
    #   Run OUTSIDE WSL (check on which port elephant arm is connected):
    #   python -m src.main s570 --port COM5
    #   or:
    #   python -m src.main s570 --port COM5 --uri ws://<IP>:9090
    #
    #   Then inside WSL:
    #   dev_workspace/docker/rosbridge.sh
    #
    # Not auto-launched due to OS dependency.

    # =========================
    # Launch Description
    # =========================
    return LaunchDescription([
        mode_arg,

        # Core system
        real_launch,
        gamepad_node,
        pyroki_node,

        # Full mode additions
        elephant_node,
    ])