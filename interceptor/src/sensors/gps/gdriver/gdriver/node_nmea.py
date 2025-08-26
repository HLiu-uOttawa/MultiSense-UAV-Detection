#!/usr/bin/env python3

# "NMEA": 
# a set of communication standards and protocols developed by National 
# Marine Electronics Association, for marine navigation and GPS devices

import rclpy
from rclpy.node import Node

import time
import serial, threading

from .utils import nmea_checksum_ok, dm_to_deg, knots_to_mps

class NMEANode(Node):
    def __init__(self):
        super().__init__('gps_nmea_node')
        self.get_logger().info('Start the node: gps_nmea_node')
        self.declare_parameter("port", "/dev/ttyACM0")
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

        self.thread = threading.Thread(target=self.reader_loop, daemon=True)
        self.thread.start()

    def reader_loop(self):
        while self.ser.open:
            try:
                line = self.ser.readline()
                # If there is nothing in the serial buffer at this moment
                if len(line) == 0: 
                    continue 
                # Convert the byte data into a string, and if there are any undecodable bytes, ignore them.
                s = line.decode(errors='ignore').strip()
                # If this line of data does not start with $, 
                # or does not contain the * checksum symbol, then it is not a standard NMEA sentence.
                if not s.startswith('$') or '*' not in s: 
                    continue
                if not nmea_checksum_ok(s): 
                    continue
                self.last_msg_time = time.time() 
                self.handle_sentence(s)
                # self.get_logger().info(line)
            except Exception as e:
                self.get_logger().error(f'NMEA read error: {e}')

    def handle_sentence(self, s: str):
        self.get_logger().info(f'In handle_sentence(), the raw data is: {s}')


    def destroy_node(self):
            self._stop = True
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass
            return super().destroy_node()

def main():
    rclpy.init()
    node = NMEANode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()