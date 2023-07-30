#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from synapse_msgs.msg import BezierTrajectory, BezierCurve
from corti.bezier_rover_planning import generate_path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
import numpy as np

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
            [1, 2],  # wp0, x, y
            [-3, 2],   # wp1, x, y
            [-4, 3],   # wp2, x, y
            [-4, 0]
        ],
        [ # velocity
            [0, 0],
            [-0.5, 0],
            [-1, 0],
            [0, 0]

        ]])
        k = 10
        bez_poly_x, bez_poly_y, bez_traj_x, bez_traj_y = generate_path(bc_t, k)
        # v  = 5
        # r = 1
        # planner = RoverPlanner(x=0, y=0, v=v, theta=0, r=r)
        # planner.goto(10, 0, v, r)
        # planner.stop(10, 0)
        # ref_data = planner.compute_ref_data(plot=False)
        t = ref_data['t']
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.position.z = 0.0

        for k in range(t.shape[0]):
            rx = ref_data['x'](t[k]) 
            ry = ref_data['y'](t[k])
            pose.pose.position.x = rx[0]
            pose.pose.position.y = ry[0]
            #psi = np.arctan2()
            pose.pose.orientation.w = 0.0
            pose.pose.orientation.z = 0.0 #np.sin(psi/2)
            path.poses.append(pose)

        for i in range(3):
            curve = BezierCurve()
            curve.time_stop = time_start + 5000000000*(i + 1)
            poly_x = ref_data['poly_x'].poly_leg[i]
            x_coef = poly_x.coef
            poly_y = ref_data['poly_y'].poly_leg[i]
            y_coef = poly_y.coef
            for j in range(7): #polynomial
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
