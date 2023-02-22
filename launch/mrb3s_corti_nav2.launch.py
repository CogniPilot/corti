from pathlib import Path

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = True
    logger = launch.substitutions.LaunchConfiguration("log_level")
    pkg_mrb3_description = Path(get_package_share_directory('mrb3_description'))
    pkg_nav2_bringup = Path(get_package_share_directory('nav2_bringup'))
    pkg_slam_toolbox = Path(get_package_share_directory('slam_toolbox'))

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(pkg_mrb3_description /  'launch/robot_description.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(pkg_nav2_bringup /  'launch/navigation_launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(pkg_slam_toolbox /  'launch/online_async_launch.py'))),
    ])
