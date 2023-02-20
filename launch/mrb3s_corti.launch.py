import launch
from launch import LaunchDescription
from launch_ros.actions import Node


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
           arguments=[
               "0", "0", "0", "0", "0", "0",
               "mrb3s/base_footprint", "mrb3s/RPLIDAR_A1M8/Base/lidar"]
        ),
    ])
