from pathlib import Path

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, \
        DeclareLaunchArgument, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    node_PID = Node(
       package='corti',
       executable='PID_ros.py',
    )

    ppm_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ppm_bridge'), 'launch', 'bridge.launch.py')),
    )

    qualisys_node = Node(
        package='qualisys_mocap',
        output='screen',
        executable='qualisys_node'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        on_exit=Shutdown(),
    )

    return LaunchDescription([
        node_PID,
        # ppm_bridge,
        qualisys_node,
        rviz_node
    ])