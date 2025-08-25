#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ImageFromFolderNode(Node):
    def __init__(self):
        super().__init__('image_from_folder_node')
        # self.get_logger().info("Start class ImageFromFolderNode(Node)")
        # Declare parameters
        self.declare_parameter("topic", "/camera/image_raw")
        
        topic = self.get_parameter("topic").get_parameter_value().string_value

        self.get_logger().info(f"Publishing images on topic: {topic}")

        topic     = self.get_parameter("topic").get_parameter_value().string_value
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.pub = self.create_publisher(Image, topic, qos)




def main(args=None):
    rclpy.init(args=args)
    try:
        node = ImageFromFolderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()