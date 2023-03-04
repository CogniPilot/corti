#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import Joy


class VirtualJoystick(Node):

    def __init__(self):
        super().__init__('virtual_joystick')
        self.publisher_ = self.create_publisher(Joy, '/joy', 10)

        self.declare_parameter(
            'arm', False,  ParameterDescriptor(description='Arm (true/false)'))
        self.declare_parameter(
            'disarm', False,  ParameterDescriptor(description='Disarm (true/false)'))
        self.declare_parameter(
            'mode', 0, ParameterDescriptor(description='Manual(0)/ Auto(1)'))

    def send(self):
        msg = Joy()
        msg.axes = [0.0]*6
        msg.buttons = [0]*10
        arm = self.get_parameter('arm').get_parameter_value().bool_value
        disarm = self.get_parameter('disarm').get_parameter_value().bool_value
        mode = self.get_parameter('mode').get_parameter_value().integer_value

        print('arm', arm)
        print('disarm', disarm)
        print('mode', mode)

        if arm:
            msg.buttons[0] = 1

        if disarm:
            msg.buttons[1] = 1

        if mode == 1:
            msg.buttons[4] = 1
        elif mode == 0:
            msg.buttons[5] = 1

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    vjoy = VirtualJoystick()

    vjoy.send()

    vjoy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
