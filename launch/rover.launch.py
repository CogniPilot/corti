import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = True
    return LaunchDescription([
        Node(
           package='joy',
           output='screen',
           executable='joy_node',
           arguments=[],
           parameters=[{'use_sim_time': use_sim_time}],
           on_exit=launch.actions.Shutdown()
        ),

        Node(
           package='ros_gz_bridge',
           output='screen',
           executable='parameter_bridge',
           arguments=[
             # Send Gazebo clock to ROS
             '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
             # joystick from ROS to Gazebo
             '/joy@sensor_msgs/msg/Joy@gz.msgs.Joy',
             # trajectory from ROS to Gazesbo
             '/traj@synapse_msgs/msg/BezierTrajectory@gz.msgs.BezierTrajectory',
             # odometry from Gazebo model to ROS
             '/model/MR_Buggy3/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
             '/model/MR_Buggy3/odometry_with_covariance@nav_msgs/msg/Odometry@gz.msgs.OdometryWithCovariance'
           ],
           parameters=[{'use_sim_time': use_sim_time}],
           on_exit=launch.actions.Shutdown()
        ),
 
        Node(
           package='corti',
           output='screen',
           executable='rover_planner',
           arguments=[],
           parameters=[{'use_sim_time': use_sim_time}],
           on_exit=launch.actions.Shutdown()
        ),
        
        Node(
           package='rviz2',
           executable='rviz2',
           arguments=['-d', get_package_share_directory('corti') + '/config/corti.rviz'],
           parameters=[{'use_sim_time': use_sim_time}],
           on_exit=launch.actions.Shutdown(),
        ),
    ])
