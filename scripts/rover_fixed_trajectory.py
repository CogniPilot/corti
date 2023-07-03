#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from synapse_msgs.msg import BezierTrajectory, BezierCurve


class BezierTrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('bezier_trajectory_publisher')
        self.publisher_ = self.create_publisher(
                BezierTrajectory, '/cerebri/in/bezier_trajectory', 10)
        timer_period = 10  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = BezierTrajectory()
        now = self.get_clock().now()
        sec, nanosec = now.seconds_nanoseconds()
        print('sec', sec)
        print('nanosec', nanosec)
        time_start = sec*1000000000 + nanosec;
        print('time start', time_start)

        msg.time_start = time_start

        for i in range(3):
            curve = BezierCurve()
            curve.time_stop = time_start + 5000000000*(i + 1)
            for j in range(6):
                curve.x.append(j)
                curve.y.append(j)
            msg.curves.append(curve)
        
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    bezier_trajectory_publisher = BezierTrajectoryPublisher()

    rclpy.spin(bezier_trajectory_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
