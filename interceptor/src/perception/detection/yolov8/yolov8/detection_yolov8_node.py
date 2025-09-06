#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
YOLOv8 Image Subscriber -> Annotated Image Publisher (ROS 2)
- Subscribes to sensor_msgs/Image
- Runs YOLOv8 on a separate worker thread ("process latest only")
- Publishes annotated images at a controlled rate (target_hz)
- GPU + FP16 supported via params (device, use_half)
"""

import json
import time
from threading import Thread, Event, Lock
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time as RclpyTime

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

# Ultralytics
from ultralytics import YOLO

# Optional small speedups
import torch
torch.backends.cudnn.benchmark = True


class YoloV8ImageNode(Node):
    def __init__(self):
        super().__init__("yolov8_image_node")

        # ---------------- Params ----------------
        self.declare_parameter("image_topic", "image_raw")
        self.declare_parameter("out_image_topic", "image_annotated")
        self.declare_parameter("pub_json", True)
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("device", "cuda:0")    # 'cpu' or 'cuda:0'
        self.declare_parameter("use_half", True)      # FP16 on CUDA
        self.declare_parameter("conf_thres", 0.25)
        self.declare_parameter("iou_thres", 0.45)
        self.declare_parameter("imgsz", 640)          # inference size; 0 = keep input size
        self.declare_parameter("thickness", 2)
        self.declare_parameter("font_scale", 0.5)
        self.declare_parameter("class_filter", "")    # "0,1,2" or "" for all
        self.declare_parameter("target_hz", 30.0)     # 0 = as fast as possible
        self.declare_parameter("resize_infer", False) # if True, downscale to imgsz, map boxes back

        self.image_topic   = self.get_parameter("image_topic").value
        self.out_topic     = self.get_parameter("out_image_topic").value
        self.pub_json      = bool(self.get_parameter("pub_json").value)
        self.model_path    = self.get_parameter("model_path").value
        self.device        = self.get_parameter("device").value
        self.use_half      = bool(self.get_parameter("use_half").value)
        self.conf_thres    = float(self.get_parameter("conf_thres").value)
        self.iou_thres     = float(self.get_parameter("iou_thres").value)
        self.imgsz         = int(self.get_parameter("imgsz").value)
        self.thickness     = int(self.get_parameter("thickness").value)
        self.font_scale    = float(self.get_parameter("font_scale").value)
        self.class_filter_str = self.get_parameter("class_filter").value
        self.target_hz     = float(self.get_parameter("target_hz").value)
        self.resize_infer  = bool(self.get_parameter("resize_infer").value)

        self.class_filter = None
        if isinstance(self.class_filter_str, str) and self.class_filter_str.strip():
            try:
                self.class_filter = set(int(x.strip()) for x in self.class_filter_str.split(","))
            except Exception:
                self.get_logger().warn(f"Invalid class_filter='{self.class_filter_str}', ignored.")
                self.class_filter = None

        # ---------------- Model ----------------
        self.get_logger().info(f"Loading YOLO model: {self.model_path} on {self.device}")
        t0 = time.time()
        self.model = YOLO(self.model_path)
        self.model.to(self.device)
        self.model.fuse()  # fuse Conv+BN once
        self.half = (self.device.startswith("cuda") and self.use_half)
        # Log model device & dtype
        p = next(self.model.model.parameters())
        self.get_logger().info(
            f"torch {torch.__version__} cuda={torch.cuda.is_available()} "
            f"model_param_device={p.device} dtype={p.dtype} half={self.half}"
        )
        self.get_logger().info(f"Model loaded in {time.time() - t0:.2f}s")

        # ---------------- ROS IO ----------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.image_topic, self._on_image, qos)
        self.pub_img = self.create_publisher(Image, self.out_topic, 10)
        self.pub_det = self.create_publisher(String, f"{self.out_topic}/detections", 10) if self.pub_json else None
        self.get_logger().info(f"Subscribed: {self.image_topic} | Publishing: {self.out_topic}")

        # ---------------- Buffers & Worker ----------------
        self._latest: Optional[Tuple] = None  # (header, frame_bgr)
        self._lock = Lock()
        self._stop_evt = Event()
        self._last_pub_ns = 0
        self._interval = (1.0 / self.target_hz) if self.target_hz > 0 else 0.0

        # performance log
        self._t_infer = self.get_clock().now()

        self.worker = Thread(target=self._worker_loop, daemon=True)
        self.worker.start()
        self.get_logger().info("YOLO worker thread started.")

    # ---------------- Sub callback: store latest only ----------------
    def _on_image(self, msg: Image):
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return
        with self._lock:
            self._latest = (msg.header, frame_bgr)  # overwrite old frame

    # ---------------- Worker loop ----------------
    def _worker_loop(self):
        while not self._stop_evt.is_set():
            # get latest frame
            with self._lock:
                data = self._latest
                self._latest = None
            if data is None:
                time.sleep(0.001)
                continue

            header, frame_bgr = data
            h0, w0 = frame_bgr.shape[:2]

            # optional resize for inference
            scale_x = scale_y = 1.0
            frame_for_infer = frame_bgr
            if self.resize_infer and self.imgsz > 0:
                frame_for_infer = cv2.resize(frame_bgr, (self.imgsz, self.imgsz), interpolation=cv2.INTER_AREA)
                scale_x = w0 / float(self.imgsz)
                scale_y = h0 / float(self.imgsz)

            frame_rgb = cv2.cvtColor(frame_for_infer, cv2.COLOR_BGR2RGB)

            # prepare kwargs
            kwargs = dict(conf=self.conf_thres, iou=self.iou_thres, device=self.device, verbose=False)
            if self.imgsz > 0 and not self.resize_infer:
                kwargs["imgsz"] = self.imgsz
            if self.half:
                kwargs["half"] = True

            # inference
            t0 = time.time()
            try:
                results = self.model.predict(frame_rgb, **kwargs)
            except Exception as e:
                self.get_logger().error(f"YOLO inference error: {e}")
                continue
            torch.cuda.synchronize() if self.device.startswith("cuda") else None
            infer_ms = (time.time() - t0) * 1000.0

            annotated = frame_bgr.copy()
            det_json = {"stamp": f"{header.stamp.sec}.{header.stamp.nanosec:09d}",
                        "frame_id": header.frame_id, "detections": []}

            if results:
                res = results[0]
                if res.boxes is not None and len(res.boxes) > 0:
                    xyxy = res.boxes.xyxy.detach().cpu().numpy()
                    confs = res.boxes.conf.detach().cpu().numpy()
                    clses = res.boxes.cls.detach().cpu().numpy().astype(int)
                    names = self.model.model.names if hasattr(self.model, "model") else self.model.names

                    for i in range(len(xyxy)):
                        cls_id = int(clses[i])
                        if (self.class_filter is not None) and (cls_id not in self.class_filter):
                            continue
                        x1, y1, x2, y2 = xyxy[i]
                        # map back if resized
                        x1 = int(x1 * scale_x); y1 = int(y1 * scale_y)
                        x2 = int(x2 * scale_x); y2 = int(y2 * scale_y)
                        conf = float(confs[i])
                        label = names.get(cls_id, str(cls_id)) if isinstance(names, dict) else str(cls_id)
                        text = f"{label} {conf:.2f}"

                        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), self.thickness)
                        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, self.font_scale, 1)
                        y_text = max(y1 - 5, th + 2)
                        cv2.rectangle(annotated, (x1, y_text - th - 4), (x1 + tw + 4, y_text + 2), (0, 0, 0), -1)
                        cv2.putText(annotated, text, (x1 + 2, y_text), cv2.FONT_HERSHEY_SIMPLEX,
                                    self.font_scale, (0, 255, 0), 1, cv2.LINE_AA)

                        det_json["detections"].append({
                            "cls": cls_id, "label": label, "conf": round(conf, 4),
                            "bbox_xyxy": [x1, y1, x2, y2]
                        })

            # throttle publish to target_hz if set
            if self._interval > 0:
                now_ns = self.get_clock().now().nanoseconds
                if now_ns - self._last_pub_ns < self._interval * 1e9:
                    continue
                self._last_pub_ns = now_ns

            # publish
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            out_msg.header = header
            self.pub_img.publish(out_msg)

            if self.pub_det:
                s = String()
                s.data = json.dumps({
                    "stamp": det_json["stamp"],
                    "frame_id": det_json["frame_id"],
                    "count": len(det_json["detections"]),
                    "detections": det_json["detections"]
                }, ensure_ascii=False)
                self.pub_det.publish(s)

            # perf log
            total_ms = infer_ms  # drawing is cheap compared to infer, keep simple
            self.get_logger().debug(f"infer {infer_ms:.1f} ms -> pub ({len(det_json['detections'])} dets)")

    def destroy_node(self):
        self._stop_evt.set()
        try:
            if hasattr(self, "worker"):
                self.worker.join(timeout=1.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloV8ImageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
