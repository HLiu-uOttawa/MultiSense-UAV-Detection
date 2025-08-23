#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ImageFromFolderNode(Node):
    def __init__(self):
        super().__init__('image_from_folder_node')

def main(args=None):
    rclpy.init(args=args)
    node = ImageFromFolderNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()