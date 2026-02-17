# プロジェクト方針・実装メモ（ROS2 + OpenCLIP + GelSight）

本書は、以下の一連の作業（`clip_cam` の OpenCLIP 推論ノード作成、RealSense 起動、可視化、単発推論、ならびに `grasp_everything` 側のデータ収集〜前処理〜学習モデル雛形）で発生した決定事項・手順・注意点を、**会話ログではなく実行可能な手順書**として整理したもの。

---

## 1. スコープ

- **clip_cam**
  - `/camera/camera/color/image_raw`（640×480@30Hz）を購読し、OpenCLIP で「柔らかい／硬い」をゼロショット分類
  - 結果（テキスト／画像オーバレイ）の publish
  - uv 仮想環境（`.venv`）利用
  - launch 統合（RealSense + classifier）

- **grasp_everything**
  - GelSight 画像のストリーミング・録画（AVI保存）
  - trial 単位のデータ収集フロー（サービス化・CLI）
  - AVI → PNG 展開、PNG → 学習用 NPY、RGB → OpenCLIP 埋め込み
  - 学習モデル（状態推定：slip/crush）雛形

---

## 2. 前提環境（重要）

- ROS2: **Humble**
- Python: **3.10**
- OpenCLIP は uv 仮想環境（`.venv`）で管理

### 2.1 NumPy 互換性（cv_bridge / OpenCV）

ROS2 Humble の `cv_bridge` は、多くの環境で **NumPy 1.x 前提のバイナリ**として提供される。
uv 仮想環境で NumPy 2.x が入ると、以下のような互換性エラーが出ることがある。

- `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x ...`
- `AttributeError: _ARRAY_API not found`（`cv_bridge` 起因）

そのため、`clip_cam` の uv 環境では以下を **明示的に固定**する。

```bash
uv add 'numpy<2' opencv-python
```

---

## 3. clip_cam（OpenCLIP soft/hard 分類ノード）

### 3.1 パッケージ構造（概略）

```text
clip_cam/
├── .venv/                         # uv仮想環境
├── pyproject.toml
├── uv.lock
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── clip_cam
├── launch/
│   ├── softness_classifier.launch.py
│   ├── single_shot.launch.py
│   └── realsense_softness.launch.py
├── scripts/
│   └── run_softness_classifier.sh
└── clip_cam/
    ├── __init__.py
    ├── softness_classifier_node.py
    └── single_shot_classifier_node.py
```

### 3.2 依存関係（uv）

最低限の例（環境により追加が必要）：

```bash
uv add open_clip_torch torch torchvision 'numpy<2' opencv-python
```

### 3.3 Topics / I/O

- Subscribe
  - `/camera/camera/color/image_raw`（`sensor_msgs/msg/Image`）

- Publish（例）
  - `/softness_classification`（`std_msgs/msg/String` など）
  - `/softness_classification/image`（結果をオーバレイした `sensor_msgs/msg/Image`）

### 3.4 主要パラメータ（例）

- `model_name`: `ViT-B-32`
- `pretrained`: `openai`
- `inference_rate`: `5.0`（入力 30Hz を間引く。必要に応じて 30.0 へ）
- `device`: `cuda` / `cpu`

### 3.5 プロンプト（紙コップ等を soft 側に寄せる）

ゼロショット分類はプロンプトが結果に強く影響する。以下のように「壊れやすさ／つぶれやすさ」を含む文言を採用。

- soft 側
  - `a photo of a soft, fragile, crushable, or delicate object`
- hard 側
  - `a photo of a hard, solid, rigid, or durable object`

---

## 4. 実行方法（uv 環境と ROS2 の橋渡し）

### 4.1 典型的な落とし穴：`ros2 run` で torch が見つからない

`ros2 run` が **システム Python** を実行するため、uv 仮想環境の `torch` が見えずに以下が起きる。

- `ModuleNotFoundError: No module named 'torch'`

#### 対応方針

- **ラッパースクリプト**で `.venv` を有効化してから実行
- **launch 側**で `bash -c` を使い、`.venv` を source してから起動

### 4.2 ラッパースクリプト（例）

`scripts/run_softness_classifier.sh` の役割：
- `.venv` を activate
- 必要ならワークスペースの `install/setup.bash` を source
- Python モジュールを起動

### 4.3 launch ファイルの注意点（`source` はシェル組み込み）

`source` は実行ファイルではなく **bash のビルトイン**のため、launch の `ExecuteProcess` で直接 `source` を実行すると失敗する。

- 典型エラー：`FileNotFoundError: [Errno 2] No such file or directory: 'source'`

**必ず** `bash -c` を介して実行する。

---

## 5. RealSense + classifier を 1 本の launch に統合

手動で別ターミナル起動していた以下を統合する。

- RealSense（例）
  - `ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=false rgb_camera.profile:=640x480x30`
- classifier
  - `ros2 launch clip_cam softness_classifier.launch.py inference_rate:=30.0`

統合後：

```bash
ros2 launch clip_cam realsense_softness.launch.py
```

---

## 6. 可視化

分類結果の可視化は、以下いずれかで行う。

- `rqt_image_view`
  - トピック：`/softness_classification/image`
- RViz2
  - Image ディスプレイで同トピックを選択

テキストのみなら：

```bash
ros2 topic echo /softness_classification
```

---

## 7. 単発推論（1枚だけ撮影して soft/hard 判定）

目的：
- トピックから 1 フレームだけ取得
- 推論してログ出力
- オーバレイ画像を保存／表示（任意）
- 終了処理（例外・Ctrl+C）を確実に行う

launch：

```bash
ros2 launch clip_cam single_shot.launch.py
```

終了処理は `try/except/finally` と `cleanup()` を用意し、OpenCV window / CUDA cache を適切に解放する。

---

## 8. データ収集（AVI保存 → 前処理で PNG/Numpy 化）

### 8.1 目的

- trial 単位で
  - GelSight raw/diff を AVI 保存
  - RealSense の把持前 RGB を1枚保存
  - meta（イベント・条件）と labels（slip/crush）を保存

### 8.2 推奨ディレクトリ構造

#### 収集直後（raw）

```text
data_raw/
├── S1/
│   ├── trial_0001/
│   │   ├── realsense_rgb_pre.png
│   │   ├── gelsight_raw.avi
│   │   ├── gelsight_diff.avi
│   │   ├── meta.json
│   │   └── labels.json
│   └── ...
├── ...
└── H5/
```

#### 前処理後（processed）

```text
data_processed/
├── S1/
│   ├── trial_0001/
│   │   ├── clip_emb.npy
│   │   ├── tactile_raw.npy
│   │   ├── tactile_diff.npy
│   │   ├── meta.json
│   │   └── labels.json
│   └── ...
└── ...
```

### 8.3 1 trial の基本フロー（モータ有り想定）

```text
(1) 物体配置
(2) 録画開始
(3) 閉じ→CONTACT検出→停止
(4) 追加締め込み（weak/mid/strong）
(5) 保持
(6) 外乱（pull 等：条件に応じる）
(7) 開放
(8) 保存・ラベル記録
```

### 8.4 外乱（disturbance）の入れ方（手動）

**pull（推奨）**
- 保持開始後、1秒程度待ってから
- 物体を **下方向にゆっくり引く**（1–2秒）
- 目的：滑る/滑らないの境界条件を作る

補助バリエーション：
- 左右に軽く揺らす
- 軸回りに少しひねる

### 8.5 モータを動かさないセンサのみテスト（指で押す）

目的：
- `CONTACT -> 押し込み -> 保持 -> 外乱 -> release` を **指操作のみ**で再現
- 「step の代わり」の押し込みタイミングをイベントとして記録

推奨フロー：

```text
[Enter] 録画開始
指で触れる → CONTACT検出
押し込む → [Enter]（押し込みイベント）
保持 → [Enter]
外乱（横引きなど）→ [Enter]
指を離す → RELEASE検出
保存
```

---

## 9. 前処理（AVI → PNG → NPY / CLIP埋め込み）

### 9.1 フレーム展開（AVI → PNG連番）

- `gelsight_raw.avi` → `frames_raw/*.png`
- `gelsight_diff.avi` → `frames_diff/*.png`

### 9.2 触覚スタック生成（PNG → `tactile_{raw|diff}.npy`）

- `contact_start_frame` を基準に K 枚抽出
- 例：`K=3`, `frame_interval=5`（30FPSなら約0.33秒の窓）
- 学習用に `64×64` などへリサイズ

### 9.3 OpenCLIP 埋め込み生成（RGB → `clip_emb.npy`）

- `realsense_rgb_pre.png` から `(512,)` の埋め込みを事前計算
- 学習時は原則 freeze（学習しない）

---

## 10. 学習モデル（状態推定：slip/crush）

### 10.1 入力

- `clip_emb`: `(B, 512)`
- `tactile_raw`: `(B, K, 3, 64, 64)`
- `tactile_diff`: `(B, K, 3, 64, 64)`

### 10.2 構造（Late Fusion）

- `TactileEncoder`（raw tower）: `tactile_raw` → `z_raw (B,128)`
- `TactileEncoder`（diff tower）: `tactile_diff` → `z_diff (B,128)`
- `FusionHead`: `concat([clip_emb, z_raw, z_diff])` → `p_slip (B,1)`, `p_crush (B,1)`

### 10.3 出力

- `p_slip`: 0–1（滑り確率）
- `p_crush`: 0–1（過変形確率）

制御側の例：

```text
if p_crush > TH_CRUSH:  STOP_TIGHTEN
elif p_slip > TH_SLIP:  TIGHTEN
else:                   HOLD
```

---

## 11. データ収集方針（短時間で学習を成立させる）

- 物体の多様性を優先（同数 trial でも「物体数↑」が汎化に効く）
- ラベルを意図的に作る
  - slip を増やす：`weak × pull`
  - crush を増やす：`soft × strong × none`（破壊しない範囲で）

10物体での例（各16 trial）：
- soft 5 + hard 5
- 条件 A/B/C/D（各4回）
  - A: weak + pull
  - B: mid + pull
  - C: mid + none
  - D: strong + none

---

## 12. 代表的なトラブルと対処

### 12.1 `source install/setup.bash` で realsense の local_setup が見つからない

`install/` に参照が残っているが、実ファイルがない（不完全ビルド／削除）可能性。

- 対処例
  - `colcon build --packages-select realsense2_camera`
  - `rm -rf install/ build/ && colcon build`
  - 不要なら `install/realsense2_camera` を削除

### 12.2 launch で `.venv` が効かず `torch` が見えない

- `SetEnvironmentVariable` だけでは不十分なケースがある
- `bash -c` 経由で `.venv/bin/activate` を source してから実行する

### 12.3 `cv2` がない

uv に `opencv-python` を追加する。

```bash
uv add opencv-python
```

---

## 13. 次の TODO（提案）

- data collection のサービスAPI仕様を確定（start/stop/capture/set_output_dir 等）
- labels の入力手順（GUI/CLI）を固定し、曖昧 trial を `exclude: true` で管理
- 収集→前処理→学習の end-to-end を 10 trial 程度でスモークテスト

