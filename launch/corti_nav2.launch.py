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
    nav2_param_file = LaunchConfiguration('nav2_param_file')
    map_file = LaunchConfiguration('map_file')

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
        default_value=['mrb3s'],
        description='vehicle name'
    )

    arg_description = DeclareLaunchArgument(
        'description',
        default_value=['mrb3_description'],
        description='vehicle decription'
    )

    arg_nav2_param_file = DeclareLaunchArgument(
        'nav2_param_file',
        default_value=PathJoinSubstitution([FindPackageShare('corti'), 'config', 'mrb3s_nav2_params.yaml']),
        description='nav2 param file'
    )

    arg_map_file = DeclareLaunchArgument(
        'map_file',
        default_value=PathJoinSubstitution([FindPackageShare('corti'), 'config', 'default_map.yaml']),
        description='Map yaml file'
    )

    # nodes
    launch_robot_description = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(description), 'launch', 'robot_description.launch.py']))
    )

    launch_nav2_bringup = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])),
        launch_arguments={
            'use_namespace': 'False',
            'slam': 'True',
            'map': '/home/ben/git/cognipilot/cranium/install/corti/share/corti/config/default_map.yaml',
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([FindPackageShare('corti'), 'config', 'mrb3s_nav2_params.yaml']),
            'autostart': 'True',
            'use_respawn': 'False',
            'use_composition': 'True'
        }.items(),
    )
    
    node_odbl = Node(
           package='tf2_ros',
           output='log',
           executable='static_transform_publisher',
           parameters=[
             {'use_sim_time': use_sim_time}
           ],
           arguments=["--frame-id", "odom", "--child-frame-id", "base_link"]
        )


    return LaunchDescription([
        arg_use_sim_time,
        arg_log_level,
        arg_vehicle,
        arg_description,
        launch_robot_description,
        #launch_nav2_bringup,
        node_odbl,
    ])
