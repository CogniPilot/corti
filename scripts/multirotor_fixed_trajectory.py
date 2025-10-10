#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from synapse_msgs.msg import BezierTrajectory, BezierCurve
from corti.bezier_multirotor_planning import derive_bezier7, derive_bezier3
from corti.TimeOptBez import find_opt_multirotor_time
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
        self.bezier7 = derive_bezier7()
        self.bezier3 = derive_bezier3()
        self.plan_traj()
        self.get_logger().info("planned trajectory")

        self.wait_for_valid_clock()
        self.timer = self.create_timer(0.0, self.timer_callback)

    def wait_for_valid_clock(self):
        if not self.get_parameter('use_sim_time').get_parameter_value().bool_value:
            return  # wall time mode
        while rclpy.ok() and self.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(self, timeout_sec=0.1)


    def timer_callback(self):
        self.get_logger().info("timer callback")
        self.publish_bezier()
        self.publish_path()
        self.timer.cancel()
        rclpy.shutdown()

    def plan_traj(self):
        # 7 boundary conditions creates
        # 8 trajectories, which is the max
        # supported by nanopb verison of synapse_pb
        # which runs on cerebri

        # Bezier msg should not be used to do high level
        # mission planning, it is just short term
        # too keep memory small. Resend bezier trajectory
        # when needed to give extended reference trajectory

        # bezier boundary conditions
        d = 1
        vel = 1
        dt = 2*d/vel
        bc_t = np.array([
            [ # position (x, y, z)
                [0, 0, 2], # 1
                [0, -d, 1.5], # 2
                [0, 0, 2], # 3
                [0, d, 2.5], # 4
                [0, 0, 2], # 5
                [0, 0, 2], # 6
            ],
            [ # velocity
                [0, 0, 0], # 1, stop
                [-vel, 0, 0], # 2, stop
                [vel/2, vel/2, 0], # 3, take turn at given vel
                [-vel, 0, 0], # 4, take turn at given vel
                [0, 0, 0], # 5, take turn at given vel
                [0, 0, 0], # 6, stop when reach origin
            ],
            [ # accel
                [0, 0, 0], # 1
                [0, 0, 0], # 2
                [0, 0, 0], # 3
                [0, 0, 0], # 4
                [0, 0, 0], # 5
                [0, 0, 0], # 6
            ],
            [ # jerk
                [0, 0, 0], # 1
                [0, 0, 0], # 2
                [0, 0, 0], # 3
                [0, 0, 0], # 4
                [0, 0, 0], # 5
                [0, 0, 0], # 6
            ]])
        bc_psi_list = np.array([
            [ # attitude
                [0, 0, 0], # 1
                [0, 0, 0*np.deg2rad(-180)], # 2
                [0, 0, 0*np.deg2rad(90)], # 3
                [0, 0, 0*np.deg2rad(180)], # 4
                [0, 0, 0*np.deg2rad(-90)], # 5
                [0, 0, 0], # 6
            ],
            [ # angular velocity
                [0, 0, 0], # 1
                [0, 0, 0], # 2
                [0, 0, 0], # 3
                [0, 0, 0], # 4
                [0, 0, 0], # 5
                [0, 0, 0], # 6
            ]])

        # solve for bezier trajectories
        T0_list = []
        PX_list = []
        PY_list = []
        PZ_list = []
        Ppsi_list = []

        for i in range(bc_t.shape[1] - 1):
            bc = bc_t[:, i:i+2, :]
            bc_psi = bc_psi_list[:, i:i+2, :]
            T0 = dt
            PX = np.array(self.bezier7['bezier7_solve'](bc[:, 0, 0], bc[:, 1, 0], T0)).reshape(-1)
            PY = np.array(self.bezier7['bezier7_solve'](bc[:, 0, 1], bc[:, 1, 1], T0)).reshape(-1)
            PZ = np.array(self.bezier7['bezier7_solve'](bc[:, 0, 2], bc[:, 1, 2], T0)).reshape(-1)
            Ppsi = np.array(self.bezier3['bezier3_solve'](bc_psi[:, 0, 2], bc_psi[:, 1, 2], T0)).reshape(-1)
            T0_list.append(T0)
            PX_list.append(PX)
            PY_list.append(PY)
            PZ_list.append(PZ)
            Ppsi_list.append(Ppsi)

        self.T0_list = T0_list
        self.PX_list = PX_list
        self.PY_list = PY_list
        self.PZ_list = PZ_list
        self.Ppsi_list = Ppsi_list

        # generate trajectory
        x_list = []
        y_list = []
        z_list = []
        psi_list = []
        for leg in range(len(T0_list)):
            n_t = 100
            t_leg = np.array([np.linspace(0, T0_list[leg], n_t)])
            traj_x = np.array(self.bezier7['bezier7_traj'](t_leg, T0_list[leg], PX_list[leg])).T
            traj_y = np.array(self.bezier7['bezier7_traj'](t_leg, T0_list[leg], PY_list[leg])).T
            traj_z = np.array(self.bezier7['bezier7_traj'](t_leg, T0_list[leg], PZ_list[leg])).T
            traj_psi = np.array(self.bezier3['bezier3_traj'](t_leg, T0_list[leg], Ppsi_list[leg])).T
            x = traj_x[:, 0]
            y = traj_y[:, 0]
            z = traj_z[:, 0]
            psi = traj_psi[:, 0]
            x_list.extend(x)
            y_list.extend(y)
            z_list.extend(z)
            psi_list.extend(psi)

        # store trajectory in class
        self.x_list = x_list
        self.y_list = y_list
        self.z_list = z_list
        self.psi_list = psi_list

    def publish_bezier(self):
        self.get_logger().info("publish bezier")

        # send bezier trajectory to autopilot
        msg_traj = BezierTrajectory()
        msg_traj.header.frame_id = 'map'
        time_start = self.get_clock().now()
        self.get_logger().info("time start {:s}".format(str(time_start)))
        msg_traj.time_start = time_start.to_msg()
        self.get_logger().info("msg_traj {:s}".format(str(msg_traj.time_start)))
        msg_traj.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info("msg_header {:s}".format(str(msg_traj.header.stamp)))
        time_leg_start = time_start
        self.get_logger().info("leg {:s}".format(str(time_leg_start)))
        for leg in range(len(self.PX_list)):
            curve = BezierCurve()
            time_stop = time_leg_start + Duration(seconds=int(self.T0_list[leg]))
            curve.time_stop = time_stop.to_msg()
            self.get_logger().info("time stop {:s}".format(str(curve.time_stop)))
            time_leg_start = time_stop
            for i in range(len(self.PX_list[leg])):
                curve.x.append(self.PX_list[leg][i])
                curve.y.append(self.PY_list[leg][i])
                curve.z.append(self.PZ_list[leg][i])
            for j in range(len(self.Ppsi_list[leg])):
                curve.yaw.append(self.Ppsi_list[leg][j])
            msg_traj.curves.append(curve)
        self.msg_traj = msg_traj  # type: BezierTrajectory
        self.get_logger().info("msg_traj: {:s}".format(str(self.msg_traj)))
        self.pub_traj.publish(msg_traj)
        self.get_logger().info("published")
        now = self.get_clock().now()
        self.get_logger().info("now: {:s}".format(str(now)))
        sec, nanosec = now.seconds_nanoseconds()
        self.get_logger().info("end publish bezier: {:s}".format(str(sec*1e9 + nanosec)))

    def publish_path(self):
        msg_path = Path()
        msg_path.header.frame_id = 'map'
        msg_path.header.stamp = self.get_clock().now().to_msg()
        for x, y, z, psi in zip(self.x_list, self.y_list, self.z_list, self.psi_list):
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = np.cos(psi/2)
            pose.pose.orientation.z = np.sin(psi/2)
            msg_path.poses.append(pose)
        self.pub_path.publish(msg_path)


def main(args=None):
    rclpy.init(args=args)

    node = BezierTrajectoryPublisher()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
