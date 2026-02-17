"""
OpenCLIPを使って1枚の画像から柔らかさ/硬さを判定するノード

1枚撮影 → 判定 → 結果表示 → 終了

Usage:
  ros2 run clip_cam single_shot_classifier
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import torch
import open_clip
from PIL import Image as PILImage
import numpy as np


class SingleShotClassifierNode(Node):
    def __init__(self):
        super().__init__('single_shot_classifier')

        # パラメータ宣言
        self.declare_parameter('model_name', 'ViT-B-32')
        self.declare_parameter('pretrained', 'openai')
        self.declare_parameter('device', 'cuda' if torch.cuda.is_available() else 'cpu')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('save_image', True)  # 結果画像を保存するか
        self.declare_parameter('output_path', '/tmp/softness_result.png')

        # パラメータ取得
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        pretrained = self.get_parameter('pretrained').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.save_image = self.get_parameter('save_image').get_parameter_value().bool_value
        self.output_path = self.get_parameter('output_path').get_parameter_value().string_value

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
        self.text_prompts = [
            "a photo of something soft and squishy, like a sponge, foam, plush, or a ripe fruit",
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

        # 画像受信フラグ
        self.image_received = False
        self.should_shutdown = False

        # Subscriber（1回だけ受信）
        self.get_logger().info(f'Waiting for image from: {image_topic}')
        self.sub_image = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

    def cleanup(self):
        """リソースのクリーンアップ"""
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        # モデルのメモリ解放
        if hasattr(self, 'model'):
            del self.model
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

        self.get_logger().info('Cleanup completed')

    def image_callback(self, msg: Image):
        """画像コールバック：1回だけ処理"""
        if self.image_received:
            return

        self.image_received = True
        self.get_logger().info('Image received! Processing...')

        try:
            # ROS Image -> OpenCV (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # OpenCV (BGR) -> PIL Image (RGB)
            rgb_image = cv_image[:, :, ::-1]  # BGR to RGB
            pil_image = PILImage.fromarray(rgb_image)

            # 推論
            label, confidence, all_probs = self.classify(pil_image)

            # 結果表示
            self.get_logger().info('=' * 50)
            self.get_logger().info('  Classification Result')
            self.get_logger().info('=' * 50)
            self.get_logger().info(f'  Result: {label.upper()}')
            self.get_logger().info(f'  Confidence: {confidence:.1%}')
            self.get_logger().info('-' * 50)
            self.get_logger().info(f'  soft: {all_probs[0]:.1%}')
            self.get_logger().info(f'  hard: {all_probs[1]:.1%}')
            self.get_logger().info('=' * 50)

            # 結果画像を生成
            vis_image = self.draw_result(cv_image, label, confidence, all_probs)

            # 結果画像を保存
            if self.save_image:
                try:
                    cv2.imwrite(self.output_path, vis_image)
                    self.get_logger().info(f'Result image saved to: {self.output_path}')
                except Exception as e:
                    self.get_logger().error(f'Failed to save image: {e}')

            # 結果をウィンドウに表示
            try:
                cv2.imshow('Softness Classification Result', vis_image)
                self.get_logger().info('Press any key to close...')
                cv2.waitKey(0)
            except Exception as e:
                self.get_logger().warn(f'Could not display image window: {e}')
            finally:
                cv2.destroyAllWindows()

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

        # 終了フラグを立てる
        self.should_shutdown = True

    def draw_result(self, cv_image: np.ndarray, label: str, confidence: float, all_probs: np.ndarray) -> np.ndarray:
        """分類結果を画像にオーバーレイ"""
        vis_image = cv_image.copy()
        h, w = vis_image.shape[:2]

        # 背景の半透明ボックス
        overlay = vis_image.copy()
        box_h = 120
        cv2.rectangle(overlay, (0, 0), (w, box_h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, vis_image, 0.4, 0, vis_image)

        # ラベルに応じた色（soft=緑, hard=赤）
        if label == "soft":
            color = (0, 255, 0)  # 緑 (BGR)
        else:
            color = (0, 0, 255)  # 赤 (BGR)

        # メイン結果テキスト
        text = f'{label.upper()}: {confidence:.1%}'
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1.5
        thickness = 3

        (text_w, text_h), baseline = cv2.getTextSize(text, font, font_scale, thickness)
        text_x = (w - text_w) // 2
        text_y = 45

        cv2.putText(vis_image, text, (text_x + 2, text_y + 2), font, font_scale, (0, 0, 0), thickness + 1)
        cv2.putText(vis_image, text, (text_x, text_y), font, font_scale, color, thickness)

        # 両方の確率を表示
        small_font_scale = 0.7
        small_thickness = 2
        y_offset = 75

        # soft
        soft_text = f'soft: {all_probs[0]:.1%}'
        cv2.putText(vis_image, soft_text, (20, y_offset), font, small_font_scale, (0, 255, 0), small_thickness)

        # hard
        hard_text = f'hard: {all_probs[1]:.1%}'
        cv2.putText(vis_image, hard_text, (w - 150, y_offset), font, small_font_scale, (0, 0, 255), small_thickness)

        # 信頼度バー
        bar_y = 95
        bar_w = int((w - 40) * confidence)
        cv2.rectangle(vis_image, (20, bar_y), (20 + bar_w, bar_y + 15), color, -1)
        cv2.rectangle(vis_image, (20, bar_y), (w - 20, bar_y + 15), (255, 255, 255), 1)

        return vis_image

    def classify(self, pil_image: PILImage.Image) -> tuple[str, float, np.ndarray]:
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
        return self.labels[idx], float(probs[idx]), probs


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = SingleShotClassifierNode()

        # should_shutdownがTrueになるまでスピン
        while rclpy.ok() and not node.should_shutdown:
            rclpy.spin_once(node, timeout_sec=0.1)

        node.get_logger().info('Shutting down...')

    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Interrupted by user')
    except Exception as e:
        if node:
            node.get_logger().error(f'Unexpected error: {e}')
        else:
            print(f'Error during initialization: {e}')
    finally:
        # クリーンアップ
        if node:
            try:
                node.cleanup()
                node.destroy_node()
            except Exception:
                pass

        # ROS2シャットダウン
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
