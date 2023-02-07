import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = True
    logger = launch.substitutions.LaunchConfiguration("log_level")
    return LaunchDescription([
        Node(
           package='corti',
           output='log',
           executable='rover_planner',
           arguments=['--ros-args', '--log-level', logger],
           parameters=[{'use_sim_time': use_sim_time}],
           on_exit=launch.actions.Shutdown()
        ),
    ])
