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

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(pkg_mrb3_description /  'launch/robot_description.launch.py'))),
        Node(
           package='corti',
           output='log',
           executable='rover_planner',
           arguments=['--ros-args', '--log-level', logger],
           parameters=[
             {'use_sim_time': use_sim_time},
             {'avg_vel': 1.5},
             {'vel0': 1.5},
             {'vel1': 0.1}
           ],
           remappings=[
            ("odom", "/model/mrb3s/odometry"),
            ("goal_pose", "goal_pose"),
            ("traj", "traj"),
            ("path", "path")
           ],
           on_exit=launch.actions.Shutdown()
        ),
        Node(
           package='corti',
           output='log',
           executable='odom_to_tf',
           arguments=[],
           parameters=[
             {'use_sim_time': use_sim_time}
           ],
           remappings=[
            ("odom", "/model/mrb3s/odometry_with_covariance")
           ]
        ),
        Node(
           package='tf2_ros',
           output='log',
           executable='static_transform_publisher',
           parameters=[
             {'use_sim_time': use_sim_time}
           ],
           arguments="--x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 "
           "--frame-id mrb3s/base_footprint --child-frame-id mrb3s/RPLIDAR_A1M8/Base/lidar".split()
        ),
    ])
