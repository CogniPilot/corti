#!/bin/env python3

import sys
import  numpy as np


# find pyecca, doing this for dev right now
sys.path.insert(0, '/workdir/tools/pyecca')

from pyecca.control.time_allocation import compute_trajectory

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from synapse_msgs.msg import PolynomialTrajectory


class RoverPlanner(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.pub_traj = self.create_publisher(PolynomialTrajectory, 'traj', 10)
        self.pub_path = self.create_publisher(Path, 'path', 10)
        self.sub_goal = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.sub_odom = self.create_subscription(
            Odometry, '/model/MR_Buggy3/odometry_with_covariance', self.odom_callback, 10)
        self.seq = 0  # trajectory sequence
        self.odom = Odometry()

    def odom_callback(self, msg: Odometry):
        self.odom = msg
        
    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info(f'goal callback')
        
        # check if odometry received
        if self.odom.header.stamp == 0:
            return
        
        #compute_trajectory()
        time_start = 0.0
        time_end = 1.0
        
        x0 = self.odom.pose.pose.position.x
        y0 = self.odom.pose.pose.position.y
        goal_x = msg.pose.position.x 
        goal_y = msg.pose.position.y
        
        dt = time_end - time_start
        vx = (goal_x - x0)/dt
        vy = (goal_y - y0)/dt
        
        coef_x = [0.0, 0.0, 0.0, 0.0, vx, x0]
        coef_y = [0.0, 0.0, 0.0, 0.0, vy, y0]
        
        # publish trajectory to cerebri
        traj = PolynomialTrajectory()
        traj.time_start = time_start
        traj.time_end = time_end
        traj.sequence = self.seq
        traj.x = coef_x
        traj.y = coef_y
        traj.z = [0.0]*6
        traj.yaw = [0.0]*6
        self.pub_traj.publish(traj)
        self.seq += 1

        # publish path to visualize on rviz
        path = Path()
        path.header.frame_id = 'map'
        t = np.linspace(time_start + 1e-4, time_end, 10)
        x = np.polyval(coef_x, t)
        y = np.polyval(coef_y, t)
        self.get_logger().info(f'vx: {vx}, vy: {vy}')

        yaw = np.arctan2(vy, vx)*np.ones(t.shape)
        for t0, x0, y0, yaw0 in zip(t, x, y, yaw):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x0
            pose.pose.position.y = y0
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = np.cos(yaw0/2)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = np.sin(yaw0/2)
            path.poses.append(pose)
        self.pub_path.publish(path)


def main(args=None):
    rclpy.init(args=args)

    rover_planner = RoverPlanner()

    rclpy.spin(rover_planner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rover_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
