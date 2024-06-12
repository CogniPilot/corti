#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import rclpy.time
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MeshFile

class ModelPublisher(Node):

    def __init__(self):
        super().__init__('model_publisher')
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
        
        self.msg.color.a = 1.0
        self.msg.color.r = 0.0
        self.msg.color.g = 0.0
        self.msg.color.b = 1.0
        self.msg.mesh_resource = "package://corti/meshes/city.dae"
        self.publisher_.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    node = ModelPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
