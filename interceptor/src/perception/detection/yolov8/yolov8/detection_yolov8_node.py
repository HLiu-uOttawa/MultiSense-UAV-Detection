import rclpy
from rclpy.node import Node

class DetectionYolov8Node(Node):
    def __init__(self):
        super().__init__('yolov8_detection')
        self.get_logger().info("Start class DetectionYolov8Node(Node)")

def main(args=None):
    rclpy.init(args=args)
    node = DetectionYolov8Node()
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()