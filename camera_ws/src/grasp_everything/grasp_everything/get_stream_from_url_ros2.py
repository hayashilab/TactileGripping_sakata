#!/usr/bin/env python3
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from cv_bridge import CvBridge


def ip_to_url(ip: str, port: int, cam_id: int = 0) -> str:
    return f"http://{ip}:{port}/cam_feed/{cam_id}"


class FrameReader:
    def __init__(self, cap, sleep_on_fail=0.005):
        self.cap = cap
        self.sleep_on_fail = sleep_on_fail
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._frame = None
        self._ret = False
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._thread.start()

    def _loop(self):
        while not self._stop.is_set():
            ret, frame = self.cap.read()
            if not ret or frame is None:
                time.sleep(self.sleep_on_fail)
                continue
            with self._lock:
                self._ret = ret
                self._frame = frame

    def read_latest(self):
        with self._lock:
            return self._ret, self._frame

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.0)


class TactileRgbDiffPublisher(Node):
    def __init__(self):
        super().__init__("tactile_rgb_diff_publisher")

        # ---- parameters ----
        self.declare_parameter("ip", "192.168.1.120")
        self.declare_parameter("port", 8000)
        self.declare_parameter("cam_id", 0)
        self.declare_parameter("publish_hz", 120.0)
        self.declare_parameter("ksize", 11)
        self.declare_parameter("sigma", 0.0)
        self.declare_parameter("publish_diff_vis", True)  # 可視化用bgr8も出す

        ip = self.get_parameter("ip").value
        port = int(self.get_parameter("port").value)
        cam_id = int(self.get_parameter("cam_id").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.ksize = int(self.get_parameter("ksize").value)
        self.sigma = float(self.get_parameter("sigma").value)
        self.publish_diff_vis = bool(self.get_parameter("publish_diff_vis").value)

        qos_img = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        
        # ---- publishers ----
        self.pub_raw = self.create_publisher(Image, "/tactile/raw_rgb", qos_img)
        self.pub_diff = self.create_publisher(Image, "/tactile/diff_rgb", qos_img)
        self.pub_diff_vis = self.create_publisher(Image, "/tactile/diff_rgb_vis", qos_img) \
            if self.publish_diff_vis else None

        # ---- ref reset (topic) ----
        self.ref_frame = None
        self.create_subscription(Empty, "/tactile/reset_ref", self._on_reset_ref, 10)

        # ---- capture ----
        url = ip_to_url(ip, port, cam_id)
        self.cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if not self.cap.isOpened():
            self.cap.release()
            self.cap = cv2.VideoCapture(url)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open stream: {url}")

        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.reader = FrameReader(self.cap)
        self.reader.start()

        self.bridge = CvBridge()

        period = 1.0 / max(self.publish_hz, 1e-3)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(f"Publishing /tactile/raw_rgb and /tactile/diff_rgb from {url}")

    def _on_reset_ref(self, _msg: Empty):
        self.ref_frame = None
        self.get_logger().info("Reference frame reset requested.")

    def _calc_diff_rgb(self, ref_bgr: np.ndarray, img_bgr: np.ndarray) -> np.ndarray:
        # diff = (img - blur(ref))/255 + 0.5  -> お手元コード踏襲
        ref_blur = cv2.GaussianBlur(ref_bgr, (self.ksize, self.ksize), self.sigma)
        diff = (img_bgr.astype(np.float32) - ref_blur.astype(np.float32)) / 255.0 + 0.5
        diff = np.clip(diff, 0.0, 1.0).astype(np.float32)
        return diff

    def _on_timer(self):
        ret, frame = self.reader.read_latest()
        if not ret or frame is None:
            return

        stamp = self.get_clock().now().to_msg()

        # raw_rgb publish (OpenCVはBGR)
        raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        raw_msg.header.stamp = stamp
        raw_msg.header.frame_id = "tactile_camera"
        self.pub_raw.publish(raw_msg)

        # init ref
        if self.ref_frame is None:
            self.ref_frame = frame.copy()
            return

        # diff_rgb publish (float32, 3ch)
        diff = self._calc_diff_rgb(self.ref_frame, frame)  # (H,W,3) float32 [0,1]
        diff_msg = self.bridge.cv2_to_imgmsg(diff, encoding="32FC3")
        diff_msg.header.stamp = stamp
        diff_msg.header.frame_id = "tactile_camera"
        self.pub_diff.publish(diff_msg)

        # optional: visualization diff
        if self.pub_diff_vis is not None:
            diff_vis = (diff * 255.0).clip(0, 255).astype(np.uint8)
            diff_vis_msg = self.bridge.cv2_to_imgmsg(diff_vis, encoding="bgr8")
            diff_vis_msg.header.stamp = stamp
            diff_vis_msg.header.frame_id = "tactile_camera"
            self.pub_diff_vis.publish(diff_vis_msg)

    def destroy_node(self):
        try:
            self.reader.stop()
        except Exception:
            pass
        try:
            self.cap.release()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = TactileRgbDiffPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
