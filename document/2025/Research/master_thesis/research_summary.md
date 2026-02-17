# 触覚センサを用いたクラッシュ検出によるグリッパー制御

## 概要

本研究では、GelSight触覚センサの画像を用いてクラッシュ（つぶれ）を検出し、その検出結果に基づいてグリッパーの制御を行うシステムを構築した。

## システム構成

### 1. データ収集・学習パイプライン

- **データセット**: `dataset_crush-diff/` - diff_rgb画像を用いたクラッシュ検出用データセット
- **学習スクリプト**: `train_crush_detector.py`
- **モデル**: ResNet18ベースの2値分類モデル（Safe vs Crush）
- **学習済み重み**: `runs/crush_detector/crush_resnet18_20260125_013100/best.pt`

#### モデル設定
| パラメータ | 値 |
|-----------|-----|
| モデル | ResNet18 |
| 入力チャンネル | 3 (diff_rgbのみ) |
| 画像サイズ | 128x128 |
| Pretrained | ImageNet |
| Dropout | 0.3 |

### 2. ROS2ノード構成

```
┌─────────────────────┐     /tactile/diff_rgb     ┌─────────────────────┐
│   stream_ros2       │ ──────────────────────────▶│  crush_detector_node│
│ (カメラストリーム)    │                           │  (クラッシュ検出)    │
└─────────────────────┘                           └──────────┬──────────┘
                                                             │
                                                  /tactile/crush_state
                                                             │
                                                             ▼
                                                  ┌─────────────────────┐
                                                  │  gripper_control    │
                                                  │  (グリッパー制御)    │
                                                  └─────────────────────┘
```

### 3. 実装したコンポーネント

#### crush_detector_node.py
触覚センサのdiff_rgb画像からクラッシュを検出するROS2ノード

**Subscribe:**
- `/tactile/diff_rgb` (sensor_msgs/Image, 32FC3)

**Publish:**
- `/tactile/is_crush` (std_msgs/Bool) - クラッシュ判定
- `/tactile/crush_probability` (std_msgs/Float32) - クラッシュ確率 (0.0-1.0)
- `/tactile/crush_state` (std_msgs/String) - "CRUSH" / "SAFE"

**パラメータ:**
- `model_path`: 学習済み重みファイルパス
- `threshold`: 判定閾値（デフォルト: 0.5）
- `image_size`: 入力画像サイズ（デフォルト: 128）

#### gripper_control.py への追加機能

**新規サービス:**
- `/gripper/close_until_crush` (std_srvs/srv/Trigger)
  - CRUSHが検出されるまでグリッパーを閉じ続ける
  - 安全上限（max_abs_pulses）に達したら停止

**実装詳細:**
- `MultiThreadedExecutor`を使用して並列処理を実現
- `ReentrantCallbackGroup`によりサービス処理中もサブスクライバーコールバックを処理可能に
- `tighten_until()`関数: 外部フラグ（stop_check）がTrueになるまでモーターを締め続ける

#### motor_control.py への追加機能

```python
def tighten_until(self, stop_check, rpm, step, max_abs_pulses, sleep_margin_s, cmd_timeout):
    """
    stop_check() が True になるまで、step 分だけ相対移動を繰り返す。

    Returns:
        1: stop_check() により停止
        0: 安全上限で停止
        負値: 通信エラー等
    """
```

## 使用方法

### 1. 環境準備

```bash
# camera_wsでビルド
cd /home/hayashi/worksp/camera_ws
colcon build --packages-select grasp_everything
source install/setup.bash

# gripper_wsでビルド
cd /home/hayashi/worksp/gripper_ws
colcon build --packages-select gripper_control
source install/setup.bash
```

### 2. システム起動

**ターミナル1: カメラストリーム**
```bash
ros2 run grasp_everything stream_ros2
```

**ターミナル2: クラッシュ検出ノード**
```bash
cd /home/hayashi/worksp/camera_ws
uv run python src/grasp_everything/grasp_everything/crush_detector_node.py --ros-args \
    -p model_path:=runs/crush_detector/crush_resnet18_20260125_013100/best.pt \
    -p threshold:=0.1
```

**ターミナル3: グリッパー制御**
```bash
ros2 run gripper_control gripper_control
```

### 3. クラッシュ検出付きグリッパー制御の実行

```bash
ros2 service call /gripper/close_until_crush std_srvs/srv/Trigger
```

### 4. モニタリング

```bash
# クラッシュ状態の確認
ros2 topic echo /tactile/crush_state

# クラッシュ確率の確認
ros2 topic echo /tactile/crush_probability
```

## 技術的課題と解決策

### 課題1: ROS2環境とuv仮想環境の統合
- **問題**: ROS2のrclpyとPyTorchが異なる環境にインストールされている
- **解決**: uvの仮想環境からROS2のPYTHONPATHを継承して実行

### 課題2: サービス処理中のサブスクライバーコールバック
- **問題**: サービスコールバック内でブロッキング処理中、サブスクライバーのコールバックが呼ばれない
- **解決**: `MultiThreadedExecutor` + `ReentrantCallbackGroup`の使用

### 課題3: モーター制御パラメータの調整
- **問題**: setZeroでモーターが動かない
- **解決**: step値とrpm値の調整（step=-50, rpm=10で閉じる方向に動作）

## ファイル構成

```
camera_ws/
├── src/grasp_everything/grasp_everything/
│   ├── crush_detector_node.py      # クラッシュ検出ROS2ノード
│   ├── get_stream_from_url_ros2.py # カメラストリームノード
│   ├── train_crush_detector.py     # モデル学習スクリプト
│   └── models/
│       ├── crush_models.py         # モデル定義（ResNet18等）
│       └── crush_dataset.py        # データセットクラス
├── runs/crush_detector/
│   └── crush_resnet18_20260125_013100/
│       ├── best.pt                 # 学習済み重み
│       └── config.json             # 学習設定
└── launch/
    └── crush_detector.launch.py    # 起動用launchファイル

gripper_ws/
└── src/gripper_control/gripper_control/
    ├── gripper_control.py          # グリッパー制御ノード
    └── motor_control.py            # モーター低レベル制御
```

## 今後の課題

1. クラッシュ検出の精度向上（データ拡張、モデルチューニング）
2. リアルタイム性の向上（推論速度の最適化）
3. 複数の物体に対するロバスト性の検証
4. 把持力制御との統合
