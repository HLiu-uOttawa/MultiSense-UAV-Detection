#!/usr/bin/env python3

# "NMEA": 
# a set of communication standards and protocols developed by National 
# Marine Electronics Association, for marine navigation and GPS devices

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import time, math
import serial, threading

from .utils import nmea_checksum_ok, dm_to_deg, knots_to_mps

class NMEANode(Node):
    def __init__(self):
        super().__init__('gps_nmea_node')
        self.get_logger().info('Start the node: gps_nmea_node')

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("publish_time_reference", True)
        self.declare_parameter('frame_id', 'gps_link')
        self.declare_parameter('base_sigma_pos_m', 5.0)
        self.declare_parameter('publish_velocity', True)
        self.declare_parameter('diagnostics', True)
        
        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baud = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.pub_time = self.get_parameter('publish_time_reference').value
        self.frame_id = self.get_parameter('frame_id').value
        self.pub_vel = self.get_parameter('publish_velocity').value
        self.base_sigma = self.get_parameter('base_sigma_pos_m').value
        self.use_diag = self.get_parameter('diagnostics').value

        # --- pubs ---
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub_fix = self.create_publisher(NavSatFix, '/gps/fix', qos)
        self.pub_vel_msg = self.create_publisher(TwistWithCovarianceStamped, '/gps/vel', qos) if self.pub_vel else None
        self.pub_time_ref = self.create_publisher(TimeReference, '/gps/time_reference', qos) if self.pub_time else None
        self.pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10) if self.use_diag else None


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

    def handle_sentence(self, s: str) -> None: 
        """
        Handle one NMEA sentence that has already passed checksum validation.
        Example: "$GPGGA,....*hh"
        """
        # self.get_logger().info(f'In handle_sentence(), the raw data is: {s}')
        try:
            star = s.find('*')
            if star == -1 or star <= 1:
                # -1: No '*' found in the sentence (invalid NMEA format)
                #  1: Only '$*' present, which is also an invalid sentence
                self.get_logger().warning(f'Bad NMEA format: {s!r}')
                return
            # Remove the leading '$' and the trailing checksum part
            body = s[1:star]          
            parts = body.split(',')   # Split the NMEA sentence body into fields by comma
            if not parts or not parts[0]:
                # If parts is empty or the first field (talker/sentence type) is missing,
                # log a warning and skip this sentence
                self.get_logger().warning(f'Empty talker/sentence: {s!r}')
                return
            # Extract the talker/sentence type, e.g., GPGGA / GNRMC / GPVTG
            talker = parts[0]         
            # Map NMEA sentence types to their corresponding handler functions.
            handlers = {
                'GGA': self.handle_gga,  # Global Positioning System Fix Data
                'RMC': self.handle_rmc,  # Recommended Minimum Specific GPS/Transit Data
                'VTG': self.handle_vtg if getattr(self, 'pub_vel', False) else None,  
                # Course over ground and ground speed (only if velocity publishing is enabled)
            }
            suffix = talker[-3:]
            handler = handlers.get(suffix)
            if handler is None:
                return
            handler(parts)

        except Exception as e:
            self.get_logger().warning(f'Parse error for sentence {s!r}: {e}')

    def handle_gga(self, p):
        """
        $GxGGA,utc,lat,NS,lon,EW,fix,numsats,hdop,alt,M,geoid,M,age,ref*CS
        """
        # self.get_logger().info(f'In handle_gga(), the raw data is: {p[0]}')
        lat = dm_to_deg(p[2], p[3])
        lon = dm_to_deg(p[4], p[5])
        fixq = int(p[6]) if len(p) > 6 and p[6] else 0
        nsats = int(p[7]) if len(p) > 7 and p[7] else None
        hdop = float(p[8]) if len(p) > 8 and p[8] else None
        alt = float(p[9]) if len(p) > 9 and p[9] else None

        self.last_fix_quality = fixq
        self.last_sats = nsats
        self.last_hdop = hdop
        self.last_alt_m = alt

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self.frame_id

        fix.status.service = NavSatStatus.SERVICE_GPS
        if fixq == 0:
            fix.status.status = NavSatStatus.STATUS_NO_FIX
        else:
            fix.status.status = NavSatStatus.STATUS_FIX

        fix.latitude = lat
        fix.longitude = lon
        fix.altitude = alt if alt is not None else float('nan')

        # covariance: approximate horizontal σ ≈ hdop * base_sigma
        if hdop is not None:
            sigma_h = max(hdop, 1.0) * self.base_sigma
        else:
            sigma_h = self.base_sigma * 2.0
        sigma_v = sigma_h * 2.0
        cov = [0.0]*9
        cov[0] = sigma_h**2
        cov[4] = sigma_h**2
        cov[8] = sigma_v**2
        fix.position_covariance = cov
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.pub_fix.publish(fix)

    def handle_rmc(self, p):
        """
        # $GxRMC,utc,status,lat,NS,lon,EW,sog,course,date,magvar,magE/W,mode*CS
        """
        # self.get_logger().info(f'In handle_rmc(), the raw data is: {p[0]}')
        status = p[2] if len(p) > 2 else 'V'
        if status != 'A':
            self.last_fix_quality = 0
        if self.pub_time and len(p) > 1 and p[1]:
            tr = TimeReference()
            tr.header.stamp = self.get_clock().now().to_msg()
            tr.header.frame_id = self.frame_id
            tr.source = 'NMEA-RMC'
            tr.time_ref = tr.header.stamp  # No PPS, conservatively use local sampling time
            self.pub_time_ref.publish(tr)

        if self.pub_vel:
            try:
                sog_knots = float(p[7]) if len(p) > 7 and p[7] else 0.0
                course_deg = float(p[8]) if len(p) > 8 and p[8] else None
                if sog_knots > 0 and course_deg is not None:
                    self.publish_velocity_from_speed_course(knots_to_mps(sog_knots), course_deg, source='RMC')
            except Exception:
                pass

    def handle_vtg(self, p):
        """
        VTG = Course Over Ground and Ground Speed.
        The function takes the parsed fields p (a list of strings split by commas).
        Example VTG sentence format:
        $GPVTG, cogt, T, cogm, M, sogk, N, sogkm, K, mode*CS
            p[1] = course over ground, degrees true
            p[5] = speed over ground, in knots
        """
        # self.get_logger().info(f'In handle_vtg(), the raw data is: {p[0]}')
        course_deg = float(p[1]) if len(p) > 1 and p[1] else None
        sog_knots  = float(p[5]) if len(p) > 5 and p[5] else None
        speed_mps  = knots_to_mps(sog_knots) if sog_knots is not None else None
        if course_deg is not None and speed_mps is not None:
            self.publish_velocity_from_speed_course(speed_mps, course_deg, source='VTG')

    def publish_velocity_from_speed_course(self, v_mps: float, course_deg: float, source=''):
        # ENU: x=East, y=North
        rad = math.radians(course_deg)
        v_e = v_mps * math.sin(rad)  # course=0 north → x=0,y=v
        v_n = v_mps * math.cos(rad)
        self.last_speed_mps = v_mps
        self.last_course_deg = course_deg

        tw = TwistWithCovarianceStamped()
        tw.header.stamp = self.get_clock().now().to_msg()
        tw.header.frame_id = self.frame_id
        tw.twist.twist.linear.x = v_e
        tw.twist.twist.linear.y = v_n
        tw.twist.twist.linear.z = 0.0
        sigma_v = 0.5 + (self.last_hdop or 1.0) * 0.2  # m/s
        cov = [0.0]*36
        cov[0] = sigma_v**2
        cov[7] = sigma_v**2
        cov[14] = (sigma_v*2.0)**2
        tw.twist.covariance = cov
        if self.pub_vel_msg:
            self.pub_vel_msg.publish(tw)

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