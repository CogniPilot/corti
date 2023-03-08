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
    vehicle = LaunchConfiguration('vehicle')
    description = LaunchConfiguration('description')

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

    arg_vehicle = DeclareLaunchArgument(
        'vehicle',
        default_value=['mrbuggy3'],
        description='vehicle name'
    )

    arg_description = DeclareLaunchArgument(
        'description',
        default_value=['mrbuggy3_description'],
        description='vehicle decription'
    )

    # nodes
    launch_robot_description = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(description), 'launch', 'robot_description.launch.py']))
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
         {'vel1': 0.1}
       ],
       remappings=[
        #("odom", ["/model/", vehicle, "/odometry"]),
        ("goal_pose", "goal_pose"),
        ("traj", "traj"),
        ("path", "path")
       ],
       on_exit=launch.actions.Shutdown()
    )

    node_odom_to_tf = Node(
       name='odom_to_tf',
       package='corti',
       output='log',
       executable='odom_to_tf',
       arguments=[],
       parameters=[
         {'use_sim_time': use_sim_time}
       ],
       remappings=[
        ("odom", ["/model/", vehicle, "/odometry_with_covariance"])
       ]
    )

    return LaunchDescription([
        arg_use_sim_time,
        arg_log_level,
        arg_vehicle,
        arg_description,
        launch_robot_description,
        node_corti,
        node_odom_to_tf,
    ])
