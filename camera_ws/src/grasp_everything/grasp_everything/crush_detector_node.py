#!/usr/bin/env python3
"""
ROS2 Crush Detector Node

学習済みResNet18モデルを使用して、tactile/diff_rgbからcrush検出を行う。
"""

import sys
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn as nn

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String
from cv_bridge import CvBridge

# モデル定義のimport
_script_dir = Path(__file__).resolve().parent
if str(_script_dir) not in sys.path:
    sys.path.insert(0, str(_script_dir))

from models.crush_models import create_crush_model


class CrushDetectorNode(Node):
    def __init__(self):
        super().__init__("crush_detector_node")

        # ---- parameters ----
        self.declare_parameter("model_path", "")
        self.declare_parameter("model_name", "resnet18")
        self.declare_parameter("image_size", 128)
        self.declare_parameter("threshold", 0.05)
        self.declare_parameter("device", "")

        model_path = self.get_parameter("model_path").value
        model_name = self.get_parameter("model_name").value
        self.image_size = int(self.get_parameter("image_size").value)
        self.threshold = float(self.get_parameter("threshold").value)
        device_param = self.get_parameter("device").value

        # デバイス設定
        if device_param:
            self.device = device_param
        else:
            self.device = "cuda" if torch.cuda.is_available() else "cpu"

        self.get_logger().info(f"Device: {self.device}")

        # モデルパスのチェック
        if not model_path:
            self.get_logger().error("model_path parameter is required!")
            raise ValueError("model_path parameter is required")

        model_path = Path(model_path)
        if not model_path.exists():
            self.get_logger().error(f"Model file not found: {model_path}")
            raise FileNotFoundError(f"Model file not found: {model_path}")

        # モデル読み込み
        self.get_logger().info(f"Loading model: {model_path}")
        self.model = create_crush_model(
            model_name=model_name,
            in_channels=3,  # diff_rgb only
            dropout=0.0,  # inference時はdropout不要
            pretrained=False,
            freeze_backbone=False,
            device=self.device,
        )

        state_dict = torch.load(model_path, map_location=self.device)
        self.model.load_state_dict(state_dict)
        self.model.eval()
        self.get_logger().info("Model loaded successfully")

        # cv_bridge
        self.bridge = CvBridge()

        # QoS設定
        qos_img = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscriber: diff_rgb (32FC3)
        self.sub_diff = self.create_subscription(
            Image, "/tactile/diff_rgb", self._on_diff_rgb, qos_img
        )

        # Publishers
        self.pub_is_crush = self.create_publisher(Bool, "/tactile/is_crush", 10)
        self.pub_crush_prob = self.create_publisher(Float32, "/tactile/crush_probability", 10)
        self.pub_crush_state = self.create_publisher(String, "/tactile/crush_state", 10)

        self.get_logger().info("Crush detector node started")
        self.get_logger().info(f"  Subscribing: /tactile/diff_rgb")
        self.get_logger().info(f"  Publishing: /tactile/is_crush, /tactile/crush_probability, /tactile/crush_state")
        self.get_logger().info(f"  Threshold: {self.threshold}")

    def _preprocess(self, diff_img: np.ndarray) -> torch.Tensor:
        """
        diff_rgb画像を前処理してモデル入力用のテンソルに変換

        Args:
            diff_img: (H, W, 3) float32 [0, 1]

        Returns:
            tensor: (1, 3, image_size, image_size)
        """
        # リサイズ
        if diff_img.shape[:2] != (self.image_size, self.image_size):
            diff_img = cv2.resize(diff_img, (self.image_size, self.image_size))

        # (H, W, C) -> (C, H, W)
        diff_img = np.transpose(diff_img, (2, 0, 1))

        # numpy -> torch tensor
        tensor = torch.from_numpy(diff_img).float()

        # batch次元追加
        tensor = tensor.unsqueeze(0)

        return tensor.to(self.device)

    def _on_diff_rgb(self, msg: Image):
        """diff_rgb画像を受信してcrush検出を実行"""
        try:
            # 32FC3 -> numpy
            diff_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # float32チェック
            if diff_img.dtype != np.float32:
                diff_img = diff_img.astype(np.float32)

            # 前処理
            input_tensor = self._preprocess(diff_img)

            # 推論
            with torch.no_grad():
                logits = self.model(input_tensor)
                prob = torch.sigmoid(logits).item()

            is_crush = prob >= self.threshold

            # Publish結果
            bool_msg = Bool()
            bool_msg.data = is_crush
            self.pub_is_crush.publish(bool_msg)

            prob_msg = Float32()
            prob_msg.data = prob
            self.pub_crush_prob.publish(prob_msg)

            state_msg = String()
            state_msg.data = "CRUSH" if is_crush else "SAFE"
            self.pub_crush_state.publish(state_msg)

        except Exception as e:
            self.get_logger().error(f"Error in crush detection: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = CrushDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
