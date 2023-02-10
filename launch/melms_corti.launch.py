import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
           parameters=[
             {'use_sim_time': use_sim_time},
             {'avg_vel': 3.0},
             {'vel0': 3.0},
             {'vel1': 0.1}
           ],
           remappings=[
            ("odom", "/model/melms/odometry"),
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
            ("odom", "/model/melms/odometry_with_covariance")
           ]
        ),
        Node(
           package='tf2_ros',
           output='log',
           executable='static_transform_publisher',
           parameters=[
             {'use_sim_time': use_sim_time}
           ],
           arguments=["0", "0", "0", "0", "0", "0",
            "melms/base_footprint", "melms/RPLIDAR_A1M8/Base/lidar"]
        ),
        #Node(
        #   package='tf2_ros',
        #   output='log',
        #   executable='static_transform_publisher',
        #   arguments=["0", "0", "0", "0", "0", "0",
        #    "map", "odom"]
        #),
        #IncludeLaunchDescription(PythonLaunchDescriptionSource(
        #    get_package_share_directory('nav2_bringup') +  '/launch/navigation_launch.py')),
        #IncludeLaunchDescription(PythonLaunchDescriptionSource(
        #    get_package_share_directory('slam_toolbox') +  '/launch/online_async_launch.py')),
    ])
