#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import Joy


class VirtualJoystick(Node):

    def __init__(self):
        super().__init__('virtual_joystick')
        self.publisher_ = self.create_publisher(Joy, '/cerebri/in/joy', 10)

        self.declare_parameter(
            'arm', False,  ParameterDescriptor(description='Arm (true/false)'))
        self.declare_parameter(
            'disarm', False,  ParameterDescriptor(description='Disarm (true/false)'))
        self.declare_parameter(
            'mode', 'unknown', ParameterDescriptor(description='Mode (auto, cmd_vel, manual)'))
        self.declare_parameter(
            'fwd', 0.0, ParameterDescriptor(description='fwd'))
        self.declare_parameter(
            'turn', 0.0, ParameterDescriptor(description='turn'))


    def send(self):
        msg = Joy()
        msg.axes = [0.0]*4
        msg.buttons = [0]*8
        arm = self.get_parameter('arm').get_parameter_value().bool_value
        disarm = self.get_parameter('disarm').get_parameter_value().bool_value
        mode = self.get_parameter('mode').get_parameter_value().string_value
        fwd = self.get_parameter('fwd').get_parameter_value().double_value
        turn = self.get_parameter('turn').get_parameter_value().double_value

        # set arm/disarm
        if disarm:
            msg.buttons[6] = 1
        elif arm:
            msg.buttons[7] = 1

        # set mode
        if mode == 'auto':
            msg.buttons[1] = 1
        elif mode == 'cmd_vel':
            msg.buttons[2] = 1
        elif mode == 'manual':
            msg.buttons[0] = 1
        elif mode == 'unknown':
            pass
        else:
            raise ValueError(f'unknown mode: {mode}, valid (auto, manual, cmd_vel)')

        # set movement
        msg.axes[1] = fwd
        msg.axes[3] = turn

        print(msg)

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    vjoy = VirtualJoystick()

    try:
        vjoy.send()
    except ValueError as e:
        print(e)

    vjoy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
