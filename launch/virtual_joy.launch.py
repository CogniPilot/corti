from pathlib import Path

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, \
        DeclareLaunchArgument, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value=['true'],
        description='use simulation time'
    )

    arg_fwd = DeclareLaunchArgument(
        'fwd',
        default_value=['0.0'],
        description='fwd throttle'
    )

    arg_turn = DeclareLaunchArgument(
        'turn',
        default_value=['0.0'],
        description='turn angle'
    )

    arg_mode = DeclareLaunchArgument(
        'mode',
        default_value=['unknown'],
        description='vehicle mode'
    )

    arg_arm = DeclareLaunchArgument(
        'arm',
        default_value=['false'],
        description='send arm command'
    )

    arg_disarm = DeclareLaunchArgument(
        'disarm',
        default_value=['false'],
        description='send disarm command'
    )

    arg_log_level = DeclareLaunchArgument(
        'log_level',
        default_value=['warn'],
        description='Logging level'
    )

    node_virtual_joy = Node(
       package='corti',
       output='screen',
       executable='virtual_joy.py',
       arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
       parameters=[
         {'use_sim_time': LaunchConfiguration('use_sim_time')},
         {'fwd': LaunchConfiguration('fwd')},
         {'turn': LaunchConfiguration('turn')},
         {'mode': LaunchConfiguration('mode')},
         {'arm': LaunchConfiguration('arm')},
         {'disarm': LaunchConfiguration('disarm')},
       ],
       remappings=[],
    )

    return LaunchDescription([
        arg_use_sim_time,
        arg_fwd,
        arg_turn,
        arg_mode,
        arg_arm,
        arg_disarm,
        arg_log_level,
        node_virtual_joy
    ])
