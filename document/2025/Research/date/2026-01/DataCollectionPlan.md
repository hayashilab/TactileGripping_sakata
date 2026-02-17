# データ取得統合方針（Gripper制御 + RealSense撮影 + GelSight録画）

## 目的
- **1 trial をワンコマンドで再現可能**にし、10物体×条件×複数回の収集を短時間で回す。
- RealSense（把持前RGB 1枚）と GelSight（raw/diff をAVIで録画）と **グリッパー制御**を同期させ、学習用の **trial単位データ**を自動生成する。
- 収集時は **AVI保存**し、後段の前処理で **PNG化→npy化→学習**へつなぐ。

---

## 前提・要件
- ROS2（ament_python/ament_cmake混在可）
- RealSense: `realsense2_camera`（例：`640x480x30`、colorのみ）
- GelSight: 既存のストリームノード（raw/diff生成・CONTACT検出・AVI録画機能がある前提）
- Gripper: 既存の `gripper_control`（または同等）ノードから **サービス／アクションで制御可能**であること
- 収集の外乱（pull等）は **操作者の手動**を基本とし、タイミングはスクリプトがガイドする

---

## 全体アーキテクチャ（推奨）
「収集オーケストレータ」1つに集約し、各デバイスは **サービス化**して呼び出す。

### 構成案
- **(A) DataCollector Orchestrator（新規）**
  - trialの状態遷移（FSM）を保持し、各ノードのサービスを順に呼ぶ
  - CLI / GUI / 自動バッチの入口を提供
- **(B) Device Nodes（既存/最小改修）**
  - RealSense: トピック購読 + 「撮影サービス」
  - GelSight: 「録画開始/停止」「dataset_dir切替」「CONTACT状態」提供
  - Gripper: 「開く/閉じる」「停止」「相対移動（Δstep）」「状態取得（position）」提供

> 重要: 「ファイル保存（書き込み）」は可能な限り **オーケストレータ側でメタ情報を一括管理**し、  
> raw/diff AVI は GelSight ノードが保存、RGB画像はオーケストレータが保存、という分担が扱いやすい。

---

## Trial FSM（状態遷移）
1 trial を **決め打ちのフェーズ**で扱うと再現性が上がります。

### フェーズ定義（例）
1. **INIT**: trialディレクトリ作成、パラメータ確定（object_id/condition 等）
2. **CAPTURE_PRE_RGB**: 把持前RGBを1枚保存
3. **START_RECORD**: GelSight録画開始（raw/diff AVI）
4. **CLOSE_UNTIL_CONTACT**: グリッパーを閉じ、CONTACT検出まで待機
5. **TIGHTEN_DELTA**: Δstepで追加締め込み（weak/mid/strong）
6. **HOLD**: 一定時間保持（例：2〜3秒）
7. **DISTURB**: 外乱フェーズ（手動）をガイド（例：2秒）
8. **RELEASE**: グリッパー開放（または指定位置へ戻す）
9. **STOP_RECORD**: GelSight録画停止
10. **FINALIZE**: meta.json / labels.json（暫定）出力、欠損チェック、次trialへ

### 代表的な待機条件
- CONTACT待機: GelSightの contact_state が `CONTACT` になった時点
- RELEASE検出（任意）: contact_state が `NON_CONTACT` に戻るまで待機（指標として有用）
- タイムアウト: 各フェーズに上限時間を設ける（例：CONTACT待機 5秒）

---

## ROS2インタフェース方針（サービス化）
「統合スクリプト」側から確実に呼べるよう、**サービス/アクション**を明確化します。

### RealSense（撮影）
- 入力: `/camera/.../color/image_raw`（sensor_msgs/Image）
- 推奨サービス（オーケストレータが提供しても良い）  
  - `CaptureRgb.srv`（例）
    - request: `path`, `encoding`（任意）
    - response: `ok`, `saved_path`, `stamp`
- 実装上は、最新フレームをバッファしておき、サービス呼び出し時に保存するのが簡潔。

### GelSight（録画・状態）
- 推奨サービス
  - `StartRecording.srv`（trial_dir, filenames, fpsなど）
  - `StopRecording.srv`
  - `SetDatasetDir.srv`（もしくはパラメータ更新）
- 推奨トピック
  - `/tactile/contact_state`（String/Int32など）: `INITIALIZING/CONTACT/NON_CONTACT`
  - `/tactile/raw` `/tactile/diff`（Image）: 任意（可視化/デバッグ）

### Gripper（制御）
- 推奨サービス/アクション（例）
  - `Open.srv` / `Close.srv`（または `SetPose`）
  - `Stop.srv`
  - `MoveRelative.srv`（Δstep）
  - `GetState.srv`（position_step, is_moving 等）
- 「閉じる」は **速度一定でCONTACTまで**をオーケストレータが管理（CONTACT検出で Stop → Δstep）すると分かりやすい。

---

## 収集データのディレクトリ構造（raw）
収集時点では **AVI中心**。後処理でPNG/NPYへ展開。

```
data_raw/
  {object_id}/
    trial_{####}/
      realsense_rgb_pre.png
      gelsight_raw.avi
      gelsight_diff.avi
      motor.csv              # グリッパーのcommand/positionログ（推奨）
      meta.json              # 収集条件・タイムスタンプ・検出フレーム等
      labels.json            # 収集中は暫定でも可（後で更新）
```

### meta.json（例）
- 収集パラメータは **必ず保存**（再現性の担保）
```json
{
  "object_id": "S1",
  "condition": "A",
  "delta_step_level": "weak",
  "delta_step_value": 40,
  "disturbance": "pull",
  "realsense_profile": "640x480x30",
  "gelsight_fps": 30,
  "contact_start_frame": 66,
  "t_contact": 1700000000.123,
  "t_release": 1700000003.456,
  "notes": ""
}
```

---

## 同期・タイミング設計
- 時刻は原則 **ROS time（rclpy clock）**で統一し、`meta.json` に記録する。
- GelSightの `contact_start_frame` は、録画開始後のフレーム番号として保持（前処理で K枚抽出の基準に使う）。
- RGB撮影は「閉じ始める前」に固定（物体外観の安定性が高い）。

---

## 外乱（disturbance）の入れ方（スクリプト側のガイド）
外乱は手動で十分ですが、**タイミングと強さの再現性**が重要です。

- pull（推奨）
  - HOLD開始から 1秒後に開始、1〜2秒間、下方向に引っ張る
  - 可能なら「一定距離（例：1〜2cm）」を目安にする
- shake（任意）
  - 左右に2往復
- twist（任意）
  - 軽く捻る（角度は小さく）

> スクリプトは `DISTURB` フェーズでカウントダウン表示（例：3,2,1→開始→2秒→終了）を出すと収集が安定します。

---

## ラベリング運用
- 収集中は最低限（slip/crush/success）だけ入力して `labels.json` を保存。
- 曖昧な trial は `exclude=true` を付けて残す（後から見直し可能）。

```json
{
  "slip": 1,
  "crush": 0,
  "success": 0,
  "exclude": false,
  "note": "pull中に明確に滑った"
}
```

---

## 実装ロードマップ（最短で動かす順）
### Step 0: センサーのみテスト（既に実施中の方向性）
- モータを動かさず、CONTACT→押し込み→外乱→RELEASEの手順で
  - GelSight録画開始/停止
  - contact_state の正しさ
  - trialディレクトリとメタ出力
を検証する。

### Step 1: RealSense撮影統合（モータなし）
- trial開始時に `realsense_rgb_pre.png` を保存できることを確認。

### Step 2: Gripper統合（CONTACTまで閉じる）
- `CLOSE_UNTIL_CONTACT` → `Stop` → `Δstep` を実装
- motor.csv ログの生成（command と position_step を時刻付きで）

### Step 3: 条件テーブル化（A/B/C/D をパラメータ化）
- `condition` から `delta_step_value` と `disturbance` を自動決定
- `object_id` と trial番号の自動採番

### Step 4: バッチ収集（10物体×条件×回数）
- 物体ごとに連続収集（交換回数を減らす）
- 収集後に欠損チェック（AVI/RGB/meta/labels が揃っているか）

---

## 前処理への接続（AVI → PNG → NPY）
収集はAVIで完結させ、学習前処理で以下を行う。
- `gelsight_raw.avi` → `frames_raw/*.png`
- `gelsight_diff.avi` → `frames_diff/*.png`
- `contact_start_frame` を基準に K枚抽出して `tactile_raw.npy` / `tactile_diff.npy`
- `realsense_rgb_pre.png` → OpenCLIPで `clip_emb.npy`

---

## 期待する「統合スクリプト」の最終的な使い方（例）
- 対話型
```
ros2 run grasp_everything data_collection_cli --ros-args -p object_id:=S1 -p condition:=A
```
- 自動バッチ
```
ros2 run grasp_everything data_collection_batch --ros-args -p plan_yaml:=collect_plan.yaml
```

---

## リスクと対策（最低限）
- **CONTACTが出ない**: タイムアウト→安全にOpen→記録停止→trialをexclude
- **保存失敗**: 各フェーズで例外捕捉し、`FINALIZE` で欠損検出
- **FPS低下**: 記録はAVIにしてI/Oを抑える、可視化は任意（デバッグ時のみ）
- **ラベル偏り**: 条件ごとの狙い（slip/crush）を保つよう運用する

