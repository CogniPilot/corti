#!/usr/bin/env python3.12
import rclpy
from rclpy.node import Node
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
        self.start_publish = False
        # self.timer_reference = self.create_timer(0.1, self.publish_reference)
        self.timer_path = self.create_timer(1, self.publish_path)

    def plan_traj(self):

        # bezier boundary conditions
        bc_t = np.array([
            [ # position
            [0, 0, 0],  # wp0, x, y, z
            [0, 0, 2],  # wp0, x, y, z
            [0, -5, 2],
            [5, -5, 2],
            [5, 0, 2],
            [0, 0, 2],
            ],
            [ # velocity
            [0, 0, 0.1],  # wp0, x, y, z
            [0, 0, 0],
            [0.5, -0.5, 0],
            [0.5, 0.5, 0],
            [-0.5, 0.5, 0],
            [-0.5, 0, 0],
            ],
            [ # accel
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],  # wp0, x, y, z
            ],
            [ # jerk
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],  # wp0, x, y, z
            [0, 0, 0],
            [0, 0, 0],
            ]])
        bc_psi_list = np.array([
            [ # attitude
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            ],
            [ # angular velocity
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            ]])

        # solve for bezier trajectories
        k = 10
        T0_list = []
        PX_list = []
        PY_list = []
        PZ_list = []
        Ppsi_list = []
        for i in range(bc_t.shape[1] - 1):
            bc = bc_t[:, i:i+2, :]
            bc_psi = bc_psi_list[:, i:i+2, :]
            T0 = 10 #find_opt_multirotor_time(8, 4, bc, bc_psi, k, 1)[0]
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
        if not self.start_publish:
            return

        print('publish bezier')

        # send bezier trajectory to autopilot
        msg_traj = BezierTrajectory()
        msg_traj.header.frame_id = 'map'
        now = self.get_clock().now()
        print('time_start', now)
        msg_traj.time_start = now.to_msg()
        msg_traj.header.stamp = now.to_msg()
        time_leg_start = now
        for leg in range(len(self.PX_list)):
            curve = BezierCurve()
            leg_time = Duration(seconds=self.T0_list[leg])
            time_stop = time_leg_start + leg_time
            curve.time_stop = (time_stop).to_msg()
            time_leg_start = time_stop
            for i in range(len(self.PX_list[leg])):
                curve.x.append(self.PX_list[leg][i])
                curve.y.append(self.PY_list[leg][i])
                curve.z.append(self.PZ_list[leg][i])
            for j in range(len(self.Ppsi_list[leg])):
                curve.yaw.append(self.Ppsi_list[leg][j])
            msg_traj.curves.append(curve)
        print("bezier")
        self.msg_traj = msg_traj  # type: BezierTrajectory
        print("publish")
        self.pub_traj.publish(self.msg_traj)
        print("finish")
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
            PZ = curve.z
            Ppsi = curve.yaw
            T0 = (curve.time_stop - self.msg_traj.time_start)*1e-9
            if time_now < curve.time_stop:
                print('on curve', i)
                break

        t = (time_now - self.msg_traj.time_start)*1e-9
        print('t', t)
        print('T0', T0)
        print('time stop', curve.time_stop*1e-9)

        traj_x = np.array(self.bezier7['bezier7_traj'](t, T0, PX)).T
        traj_y = np.array(self.bezier7['bezier7_traj'](t, T0, PY)).T
        traj_z = np.array(self.bezier7['bezier7_traj'](t, T0, PZ)).T
        traj_psi = np.array(self.bezier3['bezier3_traj'](t, T0, Ppsi)).T

        print('traj_x', traj_x)
        print('traj_y', traj_y)
        print('traj_z', traj_z)
        print('traj_psi', traj_psi)

        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = now.to_msg()
        msg.pose.position.x = traj_x[0, 0]
        msg.pose.position.y = traj_y[0, 0]
        msg.pose.position.z = traj_z[0, 0]
        psi = traj_psi[0, 0]
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

    bezier_trajectory_publisher = BezierTrajectoryPublisher()

    bezier_trajectory_publisher.set_parameters([
        rclpy.parameter.Parameter("use_sim_time",rclpy.Parameter.Type.BOOL,True)
        ])

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
