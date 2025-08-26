#!/usr/bin/env python3

# "NMEA": 
# a set of communication standards and protocols developed by National 
# Marine Electronics Association, for marine navigation and GPS devices

import rclpy
from rclpy.node import Node

import serial

class NMEANode(Node):
    def __init__(self):
        super().__init__('gps_nmea_node')
        self.get_logger().info('Start the node: gps_nmea_node')
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)


        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baud = self.get_parameter("baudrate").get_parameter_value().integer_value

        self.get_logger().info(self.port)

        # serial & reader thread
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
            self.get_logger().info(f'Opened NMEA serial: {self.port} @ {self.baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial {self.port}: {e}')
            raise

def main():
    rclpy.init()
    node = NMEANode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()