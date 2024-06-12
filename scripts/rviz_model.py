#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import rclpy.time
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MeshFile

class ModelPublisher(Node):

    def __init__(self):
        super().__init__('model_publisher')
        self.declare_parameter('model', rclpy.Parameter.Type.STRING)
        self.declare_parameter('use_texture', True)

        self.publisher_ = self.create_publisher(Marker, 'city', 1)
        self.msg = Marker()
        self.msg.header.frame_id = 'map'
        self.msg.type = Marker.MESH_RESOURCE
        self.msg.action = Marker.ADD
        self.msg.pose.position.x = 120.0
        self.msg.pose.position.y = 0.0
        self.msg.pose.position.z = -4.0
        
        self.msg.scale.x = 1.0
        self.msg.scale.y = 1.0
        self.msg.scale.z = 1.0
        
        use_texture = self.get_parameter('use_texture').value

        if use_texture:
            self.msg.mesh_use_embedded_materials = True
        else:
            self.msg.color.a = 0.1
            self.msg.color.r = 0.0
            self.msg.color.g = 0.0
            self.msg.color.b = 1.0

        try:
            self.msg.mesh_resource = self.get_parameter('model').value
        except rclpy.exceptions.ParameterUninitializedException as e:
            self.get_logger().error(str(e))
            rclpy.shutdown()
            exit(1)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.publisher_.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = ModelPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
