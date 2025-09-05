#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import os, re, time
from datetime import datetime
from typing import List, Optional, Tuple
from std_msgs.msg import String

# Match filenames in the form of:: TD_2025-05-02_09-30-04.630.txt
FNAME_RE = re.compile(
    r'^TD_(\d{4}-\d{2}-\d{2})_(\d{2}-\d{2}-\d{2}\.\d{3})\.txt$'
)

def parse_fname_ts(fname: str) -> Optional[datetime]:
    m = FNAME_RE.match(fname)                  
    if not m:          
                                
        return None                            
    day = m.group(1)                           # Date part, e.g. 2025-05-02
    hms_ms = m.group(2)                        # Time part, e.g. 09-30-04.630
    # Convert 09-30-04.630 -> 09:30:04.630 for easier parsing
    hms_ms = hms_ms.replace('-', ':')
    ts_str = f"{day}_{hms_ms}"                 # Combined string: 2025-05-02_09:30:04.630
    # Note: %f accepts 3-digit milliseconds (parsed as microseconds, e.g. 630000)
    return datetime.strptime(ts_str, "%Y-%m-%d_%H:%M:%S.%f")

class TDFile:
    def __init__(self, path: str, t: datetime):
        self.path = path
        self.t = t

    def __repr__(self):
        return f"TDFile(path={self.path!r}, t={self.t!r})"

    def __eq__(self, other):
        if not isinstance(other, TDFile):
            return False
        return self.path == other.path and self.t == other.t

class RadarFilePublisherNode(Node):
    def __init__(self):
        super().__init__('td_file_player')
        self.declare_parameter('directory', r'/mnt/d/Databases/2025-05-02/Case 1/2025-05-02_09-30-00/radar')
        self.declare_parameter('prefix', 'TD_')
        self.declare_parameter('topic', 'td_raw')
        self.declare_parameter('loop', False)        # Whether to loop after finishing playback
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('min_delay_ms', 10)   # Minimum delay (ms), avoid 0 trigger

        self.directory = self.get_parameter('directory').get_parameter_value().string_value
        self.prefix = self.get_parameter('prefix').get_parameter_value().string_value
        self.topic = self.get_parameter('topic').get_parameter_value().string_value 
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value
        self.speed = float(self.get_parameter('speed').get_parameter_value().double_value)
        self.min_delay = max(0.0, self.get_parameter('min_delay_ms').get_parameter_value().integer_value / 1000.0)
        
        if not self.directory or not os.path.isdir(self.directory):
            raise RuntimeError(f"Invalid directory parameter: {self.directory}")
        else:
            self.get_logger().info(f"Opening directory: {self.directory}")
        
        # -------- create a publisher --------
        self.pub = self.create_publisher(String, self.topic, 10)
        
        # -------- scan and initialize --------
        self.files: List[TDFile] = self._scan_and_sort(self.directory, self.prefix)
        if not self.files:
            raise RuntimeError(f"No matching files found in: {self.directory} (prefix='{self.prefix}')")

        self.get_logger().info(f"Found {len(self.files)} files. First: {os.path.basename(self.files[0].path)}  "
                               f"Last: {os.path.basename(self.files[-1].path)}")
        
        self.idx = 0
        self.last_t: Optional[datetime] = None
        self.timer = None

        # 
        self._schedule_next(initial=True)


    def _on_timer_once(self):
        # Cancel the current one-shot timer and schedule the next one
        try:
            self.timer.cancel()
        except Exception:
            pass
        self._schedule_next()
    
    def _publish_current(self):
        td = self.files[self.idx]
        try:
            with open(td.path, 'r', encoding='utf-8', errors='ignore') as f:
                content = f.read()
        except Exception as e:
            self.get_logger().error(f"Failed to read: {td.path}: {e}")
            return

        msg = String()
        msg.data = content
        self.pub.publish(msg)
        self.get_logger().info(f"Published: {os.path.basename(td.path)} @ {td.t.isoformat(timespec='milliseconds')}")

    def _scan_and_sort(self, directory: str, prefix: str) -> List[TDFile]:
        items: List[Tuple[str, datetime]] = []
        for fname in os.listdir(directory):
            if not fname.startswith(prefix) or not fname.endswith('.txt'):
                continue
            ts = parse_fname_ts(fname)
            if ts is None:
                self.get_logger().warn(f"Filename does not match timestamp format, skipped:{fname}")
                continue
            items.append((os.path.join(directory, fname), ts))

        items.sort(key=lambda x: x[1])  # sort by timestamps
        return [TDFile(path=p, t=t) for p, t in items]
    
    # ------- Single-shot timer callback -------
    def _on_timer(self):
        # Cancel the current timer immediately to prevent periodic triggers
        try:
            if self.timer is not None:
                self.timer.cancel()
        except Exception:
            pass

        # publish current file
        self._publish_current()

        # Increment index and schedule next
        self.idx += 1
        self._schedule_next(initial=False)

    def _schedule_next(self, initial: bool):
        # End or loop
        if self.idx >= len(self.files):
            if self.loop:
                self.get_logger().info("Reached the end. Looping...")
                self.idx = 0
                self.last_t = None
            else:
                self.get_logger().info("Reached the end. Stopping (no loop).")
                return

        # Calculate time difference from previous file
        current_t = self.files[self.idx].t
        if self.last_t is None or initial:
            delay = max(self.min_delay, 0.0)
        else:
            delta = (current_t - self.last_t).total_seconds()
            # Speed-up playback (speed > 1 => faster)
            delay = max(self.min_delay, delta / max(self.speed, 1e-9))

        # Create a single-shot timer (canceled and recreated in callback)
        self.timer = self.create_timer(delay, self._on_timer)
        self.get_logger().debug(f"Scheduled next file idx={self.idx} in {delay:.3f}s")

def main(args=None):
    rclpy.init(args=args)
    node = RadarFilePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()