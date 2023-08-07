#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from synapse_msgs.msg import BezierTrajectory, BezierCurve
from corti.bezier_rover_planning import generate_path, derive_bezier6, rover_timeOpt
from corti.TimeOptBez import find_opt_time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header
from typing import List
import numpy as np
import copy

class BezierTrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('bezier_trajectory_publisher')
        self.pub_traj = self.create_publisher(
                BezierTrajectory, '/cerebri/in/bezier_trajectory', 10)
        self.pub_path = self.create_publisher(
                Path, '/path', 10)
        self.pub_ref = self.create_publisher(
                PoseStamped, '/ref', 10)
        self.bezier6 = derive_bezier6()
        self.start_publish = False
        self.timer_reference = self.create_timer(0.1, self.publish_reference)
        self.timer_path = self.create_timer(1, self.publish_path)

    def plan_traj(self):

        # bezier boundary conditions
        bc_t = np.array([
            [ # position
            [0, 0],  # wp0, x, y, z
            [2, -2],   # wp1, x, y, z
            [2, -2],   # wp2, x, y, z
            [2, -2],
            ],
            [ # velocity
            [1, 0],
            [0, -1],
            [0, 0],
            [0, 0]
            ]])


        # solve for bezier trajectories
        k = 10
        T0_list = []
        PX_list = []
        PY_list = []
        for i in range(bc_t.shape[1] - 1):
            bc = bc_t[:, i:i+2, :]
            T0 = find_opt_time(6, bc, k, 1)[0]
            PX = np.array(self.bezier6['bezier6_solve'](bc[:, 0, 0], bc[:, 1, 0], T0)).reshape(-1)
            PY = np.array(self.bezier6['bezier6_solve'](bc[:, 0, 1], bc[:, 1, 1], T0)).reshape(-1)
            T0_list.append(T0)
            PX_list.append(PX)
            PY_list.append(PY)
        self.T0_list = T0_list
        self.PX_list = PX_list
        self.PY_list = PY_list

        # generate trajectory
        x_list = []
        y_list = []
        psi_list = []
        for leg in range(len(T0_list)):
            n_t = 100
            t_leg = np.array([np.linspace(0, T0_list[leg], n_t)])
            traj_x = np.array(self.bezier6['bezier6_traj'](t_leg, T0_list[leg], PX_list[leg])).T
            traj_y = np.array(self.bezier6['bezier6_traj'](t_leg, T0_list[leg], PY_list[leg])).T
            x = traj_x[:, 0]
            y = traj_y[:, 0]
            psi = np.arctan2(traj_y[:, 1], traj_x[:, 1])
            x_list.extend(x)
            y_list.extend(y)
            psi_list.extend(psi)

        # store trajectory in class
        self.x_list = x_list
        self.y_list = y_list
        self.psi_list = psi_list

    def publish_bezier(self):
        if not self.start_publish:
            return

        print('publish bezier')

        # send bezier trajectory to autopilot
        msg_traj = BezierTrajectory()
        msg_traj.header.frame_id = 'map'
        now = self.get_clock().now()
        sec, nanosec = now.seconds_nanoseconds()
        time_start = sec*1000000000 + nanosec;
        print('time_start', time_start)
        msg_traj.time_start = time_start
        msg_traj.header.stamp = now.to_msg()
        time_leg_start = time_start
        for leg in range(len(self.PX_list)):
            curve = BezierCurve()
            curve.time_stop = time_leg_start + int(1e9*self.T0_list[leg])
            time_leg_start = curve.time_stop
            for i in range(len(self.PX_list[leg])):
                curve.x.append(self.PX_list[leg][i])
                curve.y.append(self.PY_list[leg][i])
            msg_traj.curves.append(curve)
        self.msg_traj = msg_traj  # type: BezierTrajectory
        self.pub_traj.publish(msg_traj)
        now = self.get_clock().now()
        sec, nanosec = now.seconds_nanoseconds()
        print('end publish bezier', sec*1e9 + nanosec)

    def publish_reference(self):
        if not self.start_publish:
            return

        print('publish ref')
        now = self.get_clock().now()
        sec, nanosec = now.seconds_nanoseconds()
        time_now = sec*1000000000 + nanosec;
        print('now', time_now)

        curves = self.msg_traj.curves # type: List[BezierCurve]
        for i, curve in enumerate(curves):
            PX = curve.x
            PY = curve.y
            T0 = (curve.time_stop - self.msg_traj.time_start)*1e-9
            if time_now < curve.time_stop:
                print('on curve', i)
                break

        t = (time_now - self.msg_traj.time_start)*1e-9
        print('t', t)
        print('T0', T0)
        print('time stop', curve.time_stop*1e-9)

        traj_x = np.array(self.bezier6['bezier6_traj'](t, T0, PX)).T
        traj_y = np.array(self.bezier6['bezier6_traj'](t, T0, PY)).T

        print('traj_x', traj_x)
        print('traj_y', traj_y)

        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = now.to_msg()
        msg.pose.position.x = traj_x[0, 0]
        msg.pose.position.y = traj_y[0, 0]
        psi = np.arctan2(traj_y[0, 1], traj_x[0, 1])
        msg.pose.orientation.z = np.sin(psi/2)
        msg.pose.orientation.w = np.cos(psi/2)
        self.pub_ref.publish(msg)

    def publish_path(self):
        if not self.start_publish:
            return

        print('publishing path')
        msg_path = Path()
        msg_path.header.frame_id = 'map'
        msg_path.header.stamp = self.get_clock().now().to_msg()
        for x, y, psi in zip(self.x_list, self.y_list, self.psi_list):
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = np.cos(psi/2)
            pose.pose.orientation.z = np.sin(psi/2)
            msg_path.poses.append(pose)
        self.pub_path.publish(msg_path)


def main(args=None):
    rclpy.init(args=args)

    bezier_trajectory_publisher = BezierTrajectoryPublisher()


    bezier_trajectory_publisher.plan_traj()

    rclpy.spin_once(bezier_trajectory_publisher)
    bezier_trajectory_publisher.start_publish = True

    bezier_trajectory_publisher.publish_bezier()
    rclpy.spin(bezier_trajectory_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bezier_trajectory_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
