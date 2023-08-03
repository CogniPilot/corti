#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from synapse_msgs.msg import BezierTrajectory, BezierCurve
from corti.bezier_rover_planning import generate_path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
import numpy as np
import copy

class BezierTrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('bezier_trajectory_publisher')
        self.publisher_ = self.create_publisher(
                BezierTrajectory, '/cerebri/in/bezier_trajectory', 10)
        self.pathpublisher_ = self.create_publisher(
                Path, '/path', 10)
        #self.posesub_ = self.create_subscription(
        #        PoseStamped, '/pose', 10)
        timer_period = 10  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = BezierTrajectory()
        path = Path()
        now = self.get_clock().now()
        sec, nanosec = now.seconds_nanoseconds()
        print('sec', sec)
        print('nanosec', nanosec)
        time_start = sec*1000000000 + nanosec;
        print('time start', time_start)

        msg.time_start = time_start
        
        path.header.frame_id = 'map'

        bc_t = np.array([
            [ # position
            [0, 0],  # wp0, x, y, z
            [3, 0],   # wp1, x, y, z
            [3, -3],   # wp2, x, y, z
            [3, -3],
            ],
            [ # velocity
            [.1, 0],
            [0.05, 0.1],
            [0, 0],
            [0, 0]
            ]])

        k = 10
        bez_poly_x, bez_poly_y, bez_px, bez_py, bez_vx, bez_vy = generate_path(bc_t, k)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.position.z = 0.0

        for k in range(bez_px.shape[0]):
            pose.pose.position.x = bez_px[k]
            pose.pose.position.y = bez_py[k]
            psi = np.arctan2(bez_vy[k], bez_vx[k])
            pose.pose.orientation.w = np.cos(psi/2)
            pose.pose.orientation.z = np.sin(psi/2)
            pose = copy.deepcopy(pose)
            path.poses.append(pose)

        for i in range(3):
            curve = BezierCurve()
            curve.time_stop = time_start + 5000000000*(i + 1)
            x_coef = (bez_poly_x[i*6:(i+1)*6]) 
            y_coef = (bez_poly_y[i*6:(i+1)*6]) 
            for j in range(6): #polynomial
                curve.x.append(x_coef[j])
                curve.y.append(y_coef[j])
            msg.curves.append(curve)
        
        self.publisher_.publish(msg)
        self.pathpublisher_.publish(path)

def main(args=None):
    rclpy.init(args=args)

    bezier_trajectory_publisher = BezierTrajectoryPublisher()

    rclpy.spin(bezier_trajectory_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bezier_trajectory_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
