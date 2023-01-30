import launch
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        #SetEnvironmentVariable(name='', value=""),
        #DeclareLaunchArgument(
        #    'camera_calibration_file',
        #    default_value='file://' + get_package_share_directory('cuesr') + '/config/camera.yaml'),
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource([get_package_share_directory('realsense2_camera') ,'/launch/rs_launch.py']),
        #    launch_arguments={}.items(),
        #),

        Node(
           package='joy',
           output='screen',
           executable='joy_node',
           arguments=[],
           on_exit=launch.actions.Shutdown()
        ),

        Node(
           package='ros_gz_bridge',
           output='screen',
           executable='parameter_bridge',
           arguments=[
             # joystick from ROS to Gazebo
             '/joy@sensor_msgs/msg/Joy@gz.msgs.Joy',
             # trajectory from ROS to Gazesbo
             '/traj@synapse_msgs/msg/PolynomialTrajectory@gz.msgs.PolynomialTrajectory',
             # odometry from Gazebo model to ROS
             '/model/MR_Buggy3/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
             '/model/MR_Buggy3/odometry_with_covariance@nav_msgs/msg/Odometry@gz.msgs.OdometryWithCovariance'
           ],
           on_exit=launch.actions.Shutdown()
        ),
 
        Node(
           package='corti',
           output='screen',
           executable='rover_planner.py',
           arguments=[],
           on_exit=launch.actions.Shutdown()
        ),
        
        Node(
           package='tf2_ros',
           output='screen',
           executable='static_transform_publisher',
           arguments=["0", "0", "0", "0", "0", "0", "map", "camera_link"],
           on_exit=launch.actions.Shutdown()
        ),

        Node(
           package='rviz2',
           executable='rviz2',
           on_exit=launch.actions.Shutdown(),
           arguments=['-d', get_package_share_directory('corti') + '/config/corti.rviz']
        ),
    ])
