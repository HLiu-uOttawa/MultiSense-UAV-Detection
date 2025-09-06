#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Image File Player (ROS 2)

- Scans a folder for images (.jpg/.jpeg/.png)
- Extracts timestamps from filenames of the form: ...YYYY-MM-DD_HH-MM-SS.mmm.(jpg|jpeg|png)
  e.g. frame_03050_2025-05-02_09-33-46.557.jpg
- Publishes images to sensor_msgs/Image on a schedule matching the inter-file time delta
- Supports speed-up and a minimum delay guard to avoid 0-interval bursts
"""

import os
import re
from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as TimeMsg
from sensor_msgs.msg import Image
from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge


# Match any prefix, only capture the date and the time with milliseconds
# Examples that match:
#   foo_2025-05-02_09-33-46.557.jpg
#   frame_03050_2025-05-02_09-33-46.557.png
IMG_RE = re.compile(
    r'.*?([0-9]{4}-[0-9]{2}-[0-9]{2})_([0-9]{2}-[0-9]{2}-[0-9]{2}\.\d{3})\.(?:jpe?g|png)$',
    re.IGNORECASE
)


def parse_fname_ts(fname: str) -> Optional[datetime]:
    """Parse naive datetime from filename. Returns None if pattern doesn't match."""
    m = IMG_RE.match(fname)
    if not m:
        return None
    day = m.group(1)                    # YYYY-MM-DD
    hms_ms = m.group(2)                 # HH-MM-SS.mmm
    hms_ms = hms_ms.replace('-', ':')   # -> HH:MM:SS.mmm
    ts_str = f"{day}_{hms_ms}"
    # Note: %f accepts 3-digit milliseconds (parsed as microseconds)
    return datetime.strptime(ts_str, "%Y-%m-%d_%H:%M:%S.%f")


@dataclass(frozen=True)
class ImgFile:
    """Container for an image path and its extracted timestamp."""
    path: str
    t: datetime


class ImageFilePlayer(Node):
    """ROS 2 node that publishes images from a folder using filename timestamps to schedule playback."""

    def __init__(self):
        super().__init__("image_file_player")

        # ---------- Parameters ----------
        self.declare_parameter('directory', '/mnt/d/Databases/2025-05-02/Case 1/2025-05-02_09-30-00/raw')
        self.declare_parameter('topic', 'image_raw')
        self.declare_parameter('loop', False)          # loop after reaching the end
        self.declare_parameter('speed', 1.0)           # playback speed multiplier (>1 = faster)
        self.declare_parameter('min_delay_ms', 10)     # minimum delay (ms) to avoid 0-interval spikes
        self.declare_parameter('encoding', 'bgr8')     # cv_bridge encoding, e.g., bgr8, mono8
        self.declare_parameter('frame_id', 'camera')   # header.frame_id
        self.declare_parameter('use_file_timestamp_in_header', True)  # use filename timestamp for header.stamp

        self.directory = self.get_parameter('directory').value
        self.topic = self.get_parameter('topic').value
        self.loop = bool(self.get_parameter('loop').value)
        self.speed = float(self.get_parameter('speed').value) or 1.0
        if self.speed <= 0:
            self.get_logger().warn(f"speed <= 0 ({self.speed}), reset to 1.0")
            self.speed = 1.0
        self.min_delay = max(0.0, int(self.get_parameter('min_delay_ms').value) / 1000.0)
        self.encoding = self.get_parameter('encoding').value
        self.frame_id = self.get_parameter('frame_id').value
        self.use_file_ts = bool(self.get_parameter('use_file_timestamp_in_header').value)

        if not self.directory or not os.path.isdir(self.directory):
            self.get_logger().fatal(f"Invalid directory: {self.directory!r}")
            raise RuntimeError(f"Invalid directory: {self.directory!r}")

        # ---------- Publishers & Utils ----------
        self.pub_img = self.create_publisher(Image, self.topic, 10)
        # Optional helper topic to expose current filename & timestamp for debugging/alignment
        self.pub_dbg = self.create_publisher(String, f"{self.topic}/filename", 10)
        self.bridge = CvBridge()

        # ---------- Scan & Initialize ----------
        self.files: List[ImgFile] = self._scan_and_sort(self.directory)
        if not self.files:
            raise RuntimeError(f"No matching images in: {self.directory}")
        self.get_logger().info(
            f"Found {len(self.files)} images. "
            f"First: {os.path.basename(self.files[0].path)}  "
            f"Last: {os.path.basename(self.files[-1].path)}"
        )

        self.idx = 0                       # playback index (internal sequence)
        self.last_t: Optional[datetime] = None
        self.timer = None

        self._schedule_next(initial=True)

    # ---------- Helpers ----------
    def _scan_and_sort(self, directory: str) -> List[ImgFile]:
        """Scan a directory for images, parse timestamps, return a list sorted by timestamp."""
        items: List[Tuple[str, datetime]] = []
        for ent in os.scandir(directory):
            if not ent.is_file():
                continue
            fname = ent.name
            if not IMG_RE.match(fname):
                continue
            ts = parse_fname_ts(fname)
            if ts is None:
                self.get_logger().warn(f"Filename does not match timestamp format, skipped: {fname}")
                continue
            items.append((ent.path, ts))

        items.sort(key=lambda x: x[1])  # sort by timestamp
        return [ImgFile(path=p, t=t) for p, t in items]

    def _datetime_to_time_msg(self, dt: datetime) -> TimeMsg:
        """Convert naive datetime to ROS 2 builtin_interfaces/Time."""
        epoch = dt.timestamp()
        sec = int(epoch)
        nsec = int(round((epoch - sec) * 1e9))
        return TimeMsg(sec=sec, nanosec=nsec)

    def _publish_current(self) -> bool:
        """Read current image, publish Image message, and optional filename debug String."""
        img_item = self.files[self.idx]
        img = cv2.imread(img_item.path, cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().error(f"Failed to read image: {img_item.path}")
            return False

        msg: Image = self.bridge.cv2_to_imgmsg(img, encoding=self.encoding)
        msg.header.frame_id = self.frame_id
        if self.use_file_ts:
            msg.header.stamp = self._datetime_to_time_msg(img_item.t)
        else:
            msg.header.stamp = self.get_clock().now().to_msg()

        self.pub_img.publish(msg)

        dbg = String()
        dbg.data = f"{os.path.basename(img_item.path)}|{img_item.t.isoformat(timespec='milliseconds')}"
        self.pub_dbg.publish(dbg)

        self.last_t = img_item.t
        self.get_logger().info(
            f"Published: {os.path.basename(img_item.path)} "
            f"@ {img_item.t.isoformat(timespec='milliseconds')}"
        )
        return True

    # ---------- Single-shot Timer Flow ----------
    def _on_timer(self):
        """Timer callback: cancel current timer, publish, advance, and schedule the next one."""
        if self.timer is not None:
            try:
                self.timer.cancel()
            except Exception:
                pass
        self.timer = None

        _ = self._publish_current()  # even on failure, we still advance to avoid deadlock
        self.idx += 1
        self._schedule_next(initial=False)

    def _schedule_next(self, initial: bool):
        """Compute next delay using inter-file timestamp delta and set a one-shot timer."""
        # End or loop
        if self.idx >= len(self.files):
            if self.loop:
                self.get_logger().info("Reached the end. Looping...")
                self.idx = 0
                self.last_t = None
            else:
                self.get_logger().info("Reached the end. Stopping (no loop).")
                return

        current_t = self.files[self.idx].t
        if self.last_t is None or initial:
            delay = max(self.min_delay, 0.0)
            reason = "initial"
        else:
            delta = (current_t - self.last_t).total_seconds()
            delay = max(self.min_delay, delta / max(self.speed, 1e-9))
            reason = f"delta={delta:.3f}s, speed={self.speed:.3f}"

        self.timer = self.create_timer(delay, self._on_timer)
        self.get_logger().debug(
            f"Scheduled next in {delay:.3f}s ({reason}) -> "
            f"{os.path.basename(self.files[self.idx].path)}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImageFilePlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
