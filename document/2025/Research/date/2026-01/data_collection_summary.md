# データ収集パイプラインまとめ

日付: 2026-01-22

## 目的
- カメラ（RealSense）とGelSight（触覚）を組み合わせたデータ収集の統合。
- OpenCLIPでの物体のsoft/hard判定や、触覚データからの滑り・潰れ推定のためのデータ収集ワークフロー整備。

## 実施したこと（要点）
- ROS2パッケージ `grasp_everything` にデータ収集用ノード・CLI・テストを追加。
- 触覚ノード（GelSight）`get_stream_from_url_ros2.py` を拡張して、出力ディレクトリを動的に設定できるようにし、出力ファイル名を統一（`gelsight_raw.avi` / `gelsight_diff.avi` / `contact_log.csv`）。
- データ収集オーケストレータ `data_collector_node.py` を作成（サービスベースで trial 開始/停止・RGB撮影などを実行）。
- 対話的CLI `data_collection_cli.py` を作成（手順に沿って試行を進められる）。
- センサーのみでのフロー検証用 `data_collection_test.py` を作成（モーターを動かさず、指で押し込む実験のため）。
- `setup.py` にコンソールエントリーポイントを追加し、ノードを `ros2 run` で起動できるようにした。
- Launchファイル `launch/data_collection.launch.py` を作成して RealSense と CLI をまとめて起動可能にした。

## 主要ファイル（新規/更新）
- `camera_ws/src/clip_cam/` ... OpenCLIPを使った分類ノード（既存）
- `camera_ws/src/grasp_everything/grasp_everything/get_stream_from_url_ros2.py` — GelSightノード（出力先パラメータの動的変更、出力ファイル名統一）
- `camera_ws/src/grasp_everything/grasp_everything/data_collector_node.py` — サービスベースの統合収集ノード
- `camera_ws/src/grasp_everything/grasp_everything/data_collection_cli.py` — 対話的CLIでの収集フロー
- `camera_ws/src/grasp_everything/grasp_everything/data_collection_test.py` — センサーのみの手動テストスクリプト
- `camera_ws/src/grasp_everything/launch/data_collection.launch.py` — データ収集用launch
- `camera_ws/src/grasp_everything/setup.py` — entry_pointsに `data_collector`, `data_collection_cli`, `data_collection_test` を追加

（具体的なファイルパスはワークスペース内に配置済み）

## 保存されるデータ構成（統一後）
```
data_raw/{object_id}/trial_{NNNN}/
├── realsense_rgb_pre.png      # RealSenseの事前RGB（CLI/ノードが保存）
├── gelsight_raw.avi          # GelSightノードが trial_dir に保存
├── gelsight_diff.avi         # GelSightノードが trial_dir に保存
├── contact_log.csv           # GelSightノードの接触ログ
├── meta.json                 # trialメタデータ（スクリプトが保存）
└── labels.json               # ラベル（手入力/テンプレート）
```

## テスト（実行手順）
1. ビルド

```bash
cd ~/worksp/camera_ws
colcon build --packages-select grasp_everything
source install/setup.bash
```

2. GelSightノードを起動（別ターミナル）

```bash
ros2 run grasp_everything stream_ros2
```

3. センサーテスト実行（モーターは動かさない）

```bash
ros2 run grasp_everything data_collection_test --ros-args -p object_id:=TEST -p data_dir:=data_raw
```

4. 対話的収集CLI

```bash
ros2 run grasp_everything data_collection_cli --ros-args -p object_id:=S1 -p condition:=A -p data_dir:=data_raw
```

## 変更点（短評）
- GelSightの出力を `dataset_dir` パラメータで動的に切り替えられるようにしたため、各trialのディレクトリに直接ビデオを保存できる。
- `contact_log` の記録フィールドはグリッパーのエンコーダ値（`gripper_encoder`）を保存するように統一済み。
- モーター制御を行わないセンサーテストが分かれているため、安全に把持前後の挙動を確認できる。

## 次の推奨作業
- 収集済みデータに対する `Dataset` クラスと学習スクリプト（training loop）を追加。
- 自動ラベリング補助（GUI/簡易Webフォーム）を作成してラベル付け効率を上げる。
- 収集中の監視ダッシュボード（ビデオプレビュー + 接触エネルギーグラフ）。

---
メモ: もし別の保存命名規則（timestamp付き等）が必要なら指定してください。必要なら `get_stream_from_url_ros2.py` のファイル名フォーマットを調整します。
