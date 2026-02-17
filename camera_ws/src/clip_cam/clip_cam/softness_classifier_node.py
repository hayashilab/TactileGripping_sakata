"""
OpenCLIPを使って物体の柔らかさ/硬さを判定するROS2ノード

Subscribe: /camera/camera/color/image_raw (sensor_msgs/Image)
Publish: /softness_classification (std_msgs/String)
Publish: /softness_classification/image (sensor_msgs/Image) - 可視化用
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
import torch
import open_clip
from PIL import Image as PILImage
import numpy as np


class SoftnessClassifierNode(Node):
    def __init__(self):
        super().__init__('softness_classifier')

        # パラメータ宣言
        self.declare_parameter('model_name', 'ViT-B-32')
        self.declare_parameter('pretrained', 'openai')
        self.declare_parameter('inference_rate', 5.0)  # Hz（30Hzの入力を間引く）
        self.declare_parameter('device', 'cuda' if torch.cuda.is_available() else 'cpu')
        self.declare_parameter('publish_visualization', True)  # 可視化画像をpublishするか

        # パラメータ取得
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        pretrained = self.get_parameter('pretrained').get_parameter_value().string_value
        self.inference_rate = self.get_parameter('inference_rate').get_parameter_value().double_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.publish_visualization = self.get_parameter('publish_visualization').get_parameter_value().bool_value

        self.get_logger().info(f'Using device: {self.device}')
        self.get_logger().info(f'Loading model: {model_name} ({pretrained})')

        # OpenCLIPモデル読み込み
        self.model, _, self.preprocess = open_clip.create_model_and_transforms(
            model_name, pretrained=pretrained
        )
        self.model = self.model.to(self.device)
        self.model.eval()

        self.tokenizer = open_clip.get_tokenizer(model_name)

        # テキストプロンプト（soft/hard判定用）
        # soft: 柔らかい、壊れやすい、つぶれやすい、繊細なもの（紙コップ、お菓子など含む）
        # hard: 硬い、頑丈な、壊れにくいもの
        self.text_prompts = [
            "a photo of something soft and squishy, like a sponge, foam, plush, sweets, or a ripe fruit",
            "a photo of something hard and stiff, like metal, glass, plastic, ceramic, or stone",
        ]
        self.labels = ["soft", "hard"]

        # テキスト特徴量を事前計算
        text_tokens = self.tokenizer(self.text_prompts).to(self.device)
        with torch.no_grad():
            self.text_features = self.model.encode_text(text_tokens)
            self.text_features /= self.text_features.norm(dim=-1, keepdim=True)

        self.get_logger().info('Model loaded successfully')

        # CV Bridge
        self.bridge = CvBridge()

        # Publisher
        self.pub_classification = self.create_publisher(String, '/softness_classification', 10)
        self.pub_visualization = self.create_publisher(Image, '/softness_classification/image', 10)

        # Subscriber
        self.sub_image = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        # 推論レート制御用
        self.last_inference_time = self.get_clock().now()
        self.inference_interval = 1.0 / self.inference_rate

        # 最新の分類結果（可視化用）
        self.last_label = ""
        self.last_confidence = 0.0

        self.get_logger().info(f'Inference rate: {self.inference_rate} Hz')
        self.get_logger().info(f'Visualization: {self.publish_visualization}')
        self.get_logger().info('SoftnessClassifierNode started')

    def image_callback(self, msg: Image):
        """画像コールバック：推論レートに応じて間引き処理"""
        current_time = self.get_clock().now()
        elapsed = (current_time - self.last_inference_time).nanoseconds / 1e9

        if elapsed < self.inference_interval:
            return  # スキップ

        self.last_inference_time = current_time

        try:
            # ROS Image -> OpenCV (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # OpenCV (BGR) -> PIL Image (RGB)
            rgb_image = cv_image[:, :, ::-1]  # BGR to RGB
            pil_image = PILImage.fromarray(rgb_image)

            # 推論
            label, confidence = self.classify(pil_image)
            self.last_label = label
            self.last_confidence = confidence

            # 結果をpublish
            result_msg = String()
            result_msg.data = f'{label}:{confidence:.3f}'
            self.pub_classification.publish(result_msg)

            # 可視化画像をpublish
            if self.publish_visualization:
                vis_image = self.draw_result(cv_image, label, confidence)
                vis_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding='bgr8')
                vis_msg.header = msg.header
                self.pub_visualization.publish(vis_msg)

            self.get_logger().info(f'Classification: {label} (confidence: {confidence:.3f})')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def draw_result(self, cv_image: np.ndarray, label: str, confidence: float) -> np.ndarray:
        """分類結果を画像にオーバーレイ"""
        vis_image = cv_image.copy()
        h, w = vis_image.shape[:2]

        # 背景の半透明ボックス
        overlay = vis_image.copy()
        box_h = 80
        cv2.rectangle(overlay, (0, 0), (w, box_h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, vis_image, 0.4, 0, vis_image)

        # ラベルに応じた色（soft=緑, hard=赤）
        if label == "soft":
            color = (0, 255, 0)  # 緑 (BGR)
        else:
            color = (0, 0, 255)  # 赤 (BGR)

        # テキスト描画
        text = f'{label.upper()}: {confidence:.1%}'
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1.5
        thickness = 3

        # テキストサイズ取得
        (text_w, text_h), baseline = cv2.getTextSize(text, font, font_scale, thickness)
        text_x = (w - text_w) // 2
        text_y = (box_h + text_h) // 2

        # テキスト描画（影付き）
        cv2.putText(vis_image, text, (text_x + 2, text_y + 2), font, font_scale, (0, 0, 0), thickness + 1)
        cv2.putText(vis_image, text, (text_x, text_y), font, font_scale, color, thickness)

        # 信頼度バー
        bar_y = box_h - 15
        bar_w = int((w - 40) * confidence)
        cv2.rectangle(vis_image, (20, bar_y), (20 + bar_w, bar_y + 10), color, -1)
        cv2.rectangle(vis_image, (20, bar_y), (w - 20, bar_y + 10), (255, 255, 255), 1)

        return vis_image

    def classify(self, pil_image: PILImage.Image) -> tuple[str, float]:
        """OpenCLIPでzero-shot分類"""
        # 前処理
        image_tensor = self.preprocess(pil_image).unsqueeze(0).to(self.device)

        with torch.no_grad():
            # 画像特徴量
            image_features = self.model.encode_image(image_tensor)
            image_features /= image_features.norm(dim=-1, keepdim=True)

            # 類似度計算
            similarity = (100.0 * image_features @ self.text_features.T).softmax(dim=-1)
            probs = similarity[0].cpu().numpy()

        # 最も高い確率のラベルを返す
        idx = int(np.argmax(probs))
        return self.labels[idx], float(probs[idx])


def main(args=None):
    rclpy.init(args=args)

    node = SoftnessClassifierNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
