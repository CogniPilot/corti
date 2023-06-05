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
    # launch substiutions
    use_sim_time = LaunchConfiguration('use_sim_time')
    logger = LaunchConfiguration('log_level')

    # launch arguments
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value=['true'],
        description='use simulation time'
    )

    arg_log_level = DeclareLaunchArgument(
        'log_level',
        default_value=['warn'],
        description='Logging level'
    )

    node_corti = Node(
       name='corti',
       package='corti',
       output='screen',
       executable='rover_planner',
       arguments=['--ros-args', '--log-level', logger],
       parameters=[
         {'use_sim_time': use_sim_time},
         {'avg_vel': 1.5},
         {'vel0': 1.5},
         {'vel1': 1.5}
       ],
       remappings=[
        ("goal_pose", "goal_pose"),
        ("traj", "/cerebri/in/bezier_trajectory"),
        ("path", "path")
       ],
       on_exit=launch.actions.Shutdown()
    )

    return LaunchDescription([
        arg_use_sim_time,
        arg_log_level,
        node_corti
    ])
