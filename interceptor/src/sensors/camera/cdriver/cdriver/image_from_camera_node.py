#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class ImageFromCameraNode(Node):
    def __init__(self):
        super().__init__('image_from_camera_node')
        self.get_logger().info("Start class ImageFromCameraNode(Node)")

def main(args=None):
    rclpy.init(args=args)
    node = ImageFromCameraNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()