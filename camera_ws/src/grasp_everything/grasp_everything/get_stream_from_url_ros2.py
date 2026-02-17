#!/usr/bin/env python3
"""
ROS2 Tactile Stream Node with Contact Detection
- Publishes raw RGB, diff RGB images
- Publishes contact state (INITIALIZING / CONTACT / NON_CONTACT)
- Provides service to start/stop dataset collection
"""
import threading
import time
import os
from datetime import datetime

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String, Int32
from std_srvs.srv import SetBool, Trigger
from cv_bridge import CvBridge


def ip_to_url(ip: str, port: int, cam_id: int = 0) -> str:
    return f"http://{ip}:{port}/cam_feed/{cam_id}"


# ============================================================================
# Contact Detector (from get_stream_from_url_fast.py)
# ============================================================================
class ContactDetector:
    """
    固定閾値法による接触検出クラス。
    
    処理フロー:
      1. 最初のn0_framesで基準画像I0を構築
      2. エネルギーE > threshold なら CONTACT と判定
    """
    INITIALIZING = "INITIALIZING"
    CONTACT = "CONTACT"
    NON_CONTACT = "NON_CONTACT"

    def __init__(self, n0_frames=10, threshold=0.02, roi=None, ksize=11, sigma=0.0):
        self.n0_frames = n0_frames
        self.threshold = threshold
        self.roi = roi  # (x, y, w, h) or None
        self.ksize = ksize
        self.sigma = sigma

        self._I0_bgr = None
        self._ref_accum = []
        self._frame_idx = -1
        self._state = self.INITIALIZING

    @property
    def state(self):
        return self._state

    def reset(self):
        """検出器をリセット（I0再構築開始）"""
        self._I0_bgr = None
        self._ref_accum = []
        self._frame_idx = -1
        self._state = self.INITIALIZING

    def _crop_roi(self, frame):
        if self.roi is None:
            return frame
        x, y, w, h = self.roi
        return frame[y:y+h, x:x+w]

    def _calc_diff_gray(self, ref_bgr, img_bgr):
        """差分画像を計算（グレースケール）"""
        ref_g = cv2.cvtColor(ref_bgr, cv2.COLOR_BGR2GRAY)
        img_g = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        ref_blur = cv2.GaussianBlur(ref_g, (self.ksize, self.ksize), self.sigma)
        diff_img = (img_g.astype(np.float32) - ref_blur.astype(np.float32)) / 255.0 + 0.5
        diff_img = np.clip(diff_img, 0.0, 1.0)
        return diff_img

    def _compute_energy(self, frame_bgr):
        """差分画像からエネルギー値を計算"""
        diff = self._calc_diff_gray(self._I0_bgr, frame_bgr)
        return float(np.mean(np.abs(diff - 0.5))) if diff is not None else None

    def update(self, frame_bgr):
        """
        1フレームを処理し、接触状態を判定する。
        
        Returns:
            (state, energy) - state: INITIALIZING/CONTACT/NON_CONTACT
        """
        self._frame_idx += 1

        if frame_bgr is None:
            return self._state, None

        frame_bgr = self._crop_roi(frame_bgr)

        # 基準画像の構築フェーズ
        if self._I0_bgr is None:
            self._ref_accum.append(frame_bgr.astype(np.float32))
            if len(self._ref_accum) >= self.n0_frames:
                I0 = np.mean(np.stack(self._ref_accum, axis=0), axis=0)
                self._I0_bgr = I0.astype(np.uint8)
                self._ref_accum = []
            return self.INITIALIZING, None

        # エネルギー計算と判定
        E = self._compute_energy(frame_bgr)
        if E is None:
            return self._state, None

        self._state = self.CONTACT if E > self.threshold else self.NON_CONTACT
        return self._state, E


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


# ============================================================================
# Async Video Saver (for dataset collection)
# ============================================================================
import queue

class AsyncAviSaver:
    """
    OpenCV VideoWriter を別スレッドで回して .avi 保存するためのクラス。
    """
    def __init__(
        self,
        out_path: str,
        fps: float = 30.0,
        fourcc: str = "MJPG",
        max_queue: int = 128,
        drop_if_full: bool = True,
        is_color: bool = True,
    ):
        self.out_path = out_path
        self.fps = float(fps)
        self.fourcc = cv2.VideoWriter_fourcc(*fourcc)
        self.max_queue = int(max_queue)
        self.drop_if_full = bool(drop_if_full)
        self.is_color = is_color

        self._q = queue.Queue(maxsize=self.max_queue)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._worker, daemon=True)

        self._writer = None
        self._opened = False
        self._dropped = 0
        self._frame_count = 0

    @property
    def dropped_frames(self) -> int:
        return self._dropped

    @property
    def frame_count(self) -> int:
        return self._frame_count

    def start(self):
        self._thread.start()
        return self

    def _ensure_writer(self, frame: np.ndarray):
        h, w = frame.shape[:2]
        self._writer = cv2.VideoWriter(self.out_path, self.fourcc, self.fps, (w, h), self.is_color)
        if not self._writer.isOpened():
            raise RuntimeError(f"VideoWriter open failed: path={self.out_path}")
        self._opened = True

    def write(self, frame: np.ndarray):
        """メインループ側から呼ぶ。キューに積むだけ。"""
        if frame is None:
            return

        if frame.ndim == 2 and self.is_color:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        try:
            if self.drop_if_full:
                self._q.put_nowait(frame.copy())
            else:
                self._q.put(frame.copy())
        except queue.Full:
            self._dropped += 1

    def _worker(self):
        while not self._stop.is_set() or not self._q.empty():
            try:
                frame = self._q.get(timeout=0.1)
            except queue.Empty:
                continue

            if not self._opened:
                self._ensure_writer(frame)

            self._writer.write(frame)
            self._frame_count += 1
            self._q.task_done()

        if self._writer is not None:
            self._writer.release()

    def close(self):
        """終了時に呼ぶ。キューを吐き切ってから閉じる。"""
        self._stop.set()
        self._thread.join(timeout=2.0)
        if self._writer is not None:
            try:
                self._writer.release()
            except Exception:
                pass


class TactileRgbDiffPublisher(Node):
    def __init__(self):
        super().__init__("tactile_rgb_diff_publisher")

        # ---- parameters ----
        self.declare_parameter("ip", "192.168.1.120")
        self.declare_parameter("port", 8000)
        self.declare_parameter("cam_id", 0)
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("ksize", 11)
        self.declare_parameter("sigma", 0.0)
        self.declare_parameter("publish_diff_vis", True)
        # Contact detection parameters
        self.declare_parameter("contact_threshold", 0.006)
        self.declare_parameter("contact_n0_frames", 10)
        # Dataset collection parameters
        self.declare_parameter("dataset_dir", "./dataset")

        ip = self.get_parameter("ip").value
        port = int(self.get_parameter("port").value)
        cam_id = int(self.get_parameter("cam_id").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.ksize = int(self.get_parameter("ksize").value)
        self.sigma = float(self.get_parameter("sigma").value)
        self.publish_diff_vis = bool(self.get_parameter("publish_diff_vis").value)
        self.contact_threshold = float(self.get_parameter("contact_threshold").value)
        self.contact_n0_frames = int(self.get_parameter("contact_n0_frames").value)
        self.dataset_dir = str(self.get_parameter("dataset_dir").value)

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
        # Contact state publisher
        self.pub_contact_state = self.create_publisher(String, "/tactile/contact_state", 10)
        self.pub_contact_energy = self.create_publisher(Int32, "/tactile/contact_energy", 10)

        # ---- ref reset (topic) ----
        self.ref_frame = None
        self.create_subscription(Empty, "/tactile/reset_ref", self._on_reset_ref, 10)

        # ---- services ----
        # Dataset collection service
        self.create_service(SetBool, "/tactile/start_collection", self._on_start_collection)
        self.create_service(Trigger, "/tactile/reset_detector", self._on_reset_detector)
        
        # Add parameter callback for dynamic output_dir change
        self.add_on_set_parameters_callback(self._on_parameter_change)

        # ---- Contact Detector ----
        self.contact_detector = ContactDetector(
            n0_frames=self.contact_n0_frames,
            threshold=self.contact_threshold,
            roi=None,
            ksize=self.ksize,
            sigma=self.sigma,
        )
        self._prev_contact_state = None

        # ---- Dataset collection state ----
        self._is_collecting = False
        self._video_saver = None
        self._diff_video_saver = None  # diff_rgb用
        self._collection_start_time = None
        self._contact_log = []  # [(timestamp, state, energy, gripper_step), ...]

        # ---- Gripper state (subscribe from gripper node) ----
        self._current_gripper_step = 0
        self.create_subscription(Int32, "/gripper/current_step", self._on_gripper_step, 10)

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

        self.get_logger().info(f"Publishing /tactile/raw_rgb, /tactile/diff_rgb, /tactile/contact_state from {url}")
        self.get_logger().info(f"Contact threshold: {self.contact_threshold}")

    def _on_parameter_change(self, params):
        """Handle dynamic parameter changes"""
        from rcl_interfaces.msg import SetParametersResult
        for param in params:
            if param.name == 'dataset_dir':
                self.dataset_dir = param.value
                self.get_logger().info(f"Output directory changed to: {self.dataset_dir}")
        return SetParametersResult(successful=True)

    def _on_gripper_step(self, msg: Int32):
        """Callback for gripper position updates"""
        self._current_gripper_step = msg.data

    def _on_reset_ref(self, _msg: Empty):
        self.ref_frame = None
        self.get_logger().info("Reference frame reset requested.")

    def _on_reset_detector(self, request, response):
        """Reset contact detector (rebuild I0)"""
        self.contact_detector.reset()
        self._prev_contact_state = None
        response.success = True
        response.message = "Contact detector reset"
        self.get_logger().info("Contact detector reset.")
        return response

    def _on_start_collection(self, request, response):
        """Start/Stop dataset collection"""
        if request.data:
            # Start collection
            if self._is_collecting:
                response.success = False
                response.message = "Already collecting"
                return response

            # Create dataset directory
            os.makedirs(self.dataset_dir, exist_ok=True)
            # Use fixed filenames (no timestamp) for integration with data_collection scripts
            video_path = os.path.join(self.dataset_dir, "gelsight_raw.avi")
            diff_video_path = os.path.join(self.dataset_dir, "gelsight_diff.avi")
            log_path = os.path.join(self.dataset_dir, "contact_log.csv")

            self._video_saver = AsyncAviSaver(out_path=video_path, fps=self.publish_hz).start()
            self._diff_video_saver = AsyncAviSaver(out_path=diff_video_path, fps=self.publish_hz).start()
            self._collection_start_time = time.time()
            self._contact_log = []
            self._log_path = log_path
            self._is_collecting = True

            # Reset detector for fresh baseline
            self.contact_detector.reset()
            self._prev_contact_state = None

            response.success = True
            response.message = f"Started collection: {video_path}"
            self.get_logger().info(f"Dataset collection started: {video_path}")
        else:
            # Stop collection
            if not self._is_collecting:
                response.success = False
                response.message = "Not collecting"
                return response

            self._is_collecting = False
            frame_count = 0
            if self._video_saver is not None:
                self._video_saver.close()
                frame_count = self._video_saver.frame_count
                self._video_saver = None
            if self._diff_video_saver is not None:
                self._diff_video_saver.close()
                self._diff_video_saver = None

            # Save contact log to CSV
            self._save_contact_log()

            response.success = True
            response.message = f"Stopped collection. Frames: {frame_count}, Logs: {len(self._contact_log)}"
            self.get_logger().info(f"Dataset collection stopped. Frames: {frame_count}")

        return response

    def _save_contact_log(self):
        """Save contact log to CSV file"""
        if not self._contact_log:
            return
        import csv
        with open(self._log_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'state', 'energy', 'gripper_step'])
            writer.writerows(self._contact_log)
        self.get_logger().info(f"Contact log saved: {self._log_path}")

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
        current_time = time.time()

        # ---- Contact Detection ----
        state, energy = self.contact_detector.update(frame)

        # Publish contact state
        state_msg = String()
        state_msg.data = state
        self.pub_contact_state.publish(state_msg)

        # Publish energy (scaled to int for easy monitoring)
        if energy is not None:
            energy_msg = Int32()
            energy_msg.data = int(energy * 10000)  # 0.0075 -> 75
            self.pub_contact_energy.publish(energy_msg)

        # Log state change
        if state != self._prev_contact_state:
            energy_str = f"{energy:.4f}" if energy is not None else "N/A"
            self.get_logger().info(f"Contact state: {state} (energy={energy_str})")
            self._prev_contact_state = state

        # ---- Dataset collection (raw frame) ----
        if self._is_collecting and self._video_saver is not None:
            self._video_saver.write(frame)
            # Log contact info
            elapsed = current_time - self._collection_start_time
            self._contact_log.append([
                f"{elapsed:.3f}",
                state,
                f"{energy:.6f}" if energy is not None else "",
                self._current_gripper_step
            ])

        # raw_rgb publish (OpenCVはBGR)
        raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        raw_msg.header.stamp = stamp
        raw_msg.header.frame_id = "tactile_camera"
        self.pub_raw.publish(raw_msg)

        # init ref (for diff visualization, separate from contact detector)
        if self.ref_frame is None:
            self.ref_frame = frame.copy()
            return

        # diff_rgb publish (float32, 3ch)
        diff = self._calc_diff_rgb(self.ref_frame, frame)  # (H,W,3) float32 [0,1]
        diff_msg = self.bridge.cv2_to_imgmsg(diff, encoding="32FC3")
        diff_msg.header.stamp = stamp
        diff_msg.header.frame_id = "tactile_camera"
        self.pub_diff.publish(diff_msg)

        # ---- Dataset collection (diff_rgb) ----
        if self._is_collecting and self._diff_video_saver is not None:
            diff_vis_save = (diff * 255.0).clip(0, 255).astype(np.uint8)
            self._diff_video_saver.write(diff_vis_save)

        # optional: visualization diff
        if self.pub_diff_vis is not None:
            diff_vis = (diff * 255.0).clip(0, 255).astype(np.uint8)
            diff_vis_msg = self.bridge.cv2_to_imgmsg(diff_vis, encoding="bgr8")
            diff_vis_msg.header.stamp = stamp
            diff_vis_msg.header.frame_id = "tactile_camera"
            self.pub_diff_vis.publish(diff_vis_msg)

    def destroy_node(self):
        # Stop dataset collection if active
        if self._is_collecting:
            if self._video_saver is not None:
                self._video_saver.close()
            if self._diff_video_saver is not None:
                self._diff_video_saver.close()
            self._save_contact_log()
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
