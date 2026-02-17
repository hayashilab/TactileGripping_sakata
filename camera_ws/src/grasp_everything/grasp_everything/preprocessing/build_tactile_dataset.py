#!/usr/bin/env python3
"""
触覚データセット構築スクリプト

data_raw/の動画からフレームを抽出し、npz形式で保存する。
各npzにはraw画像、diff画像、ラベル情報を含む。

使用例:
    python -m grasp_everything.preprocessing.build_tactile_dataset \
        --data-dir data_raw \
        --out-dir dataset_tactile \
        --size 128 --stride 5
"""

import argparse
import csv
import json
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from tqdm import tqdm


def parse_args():
    parser = argparse.ArgumentParser(description="触覚データセット構築")
    parser.add_argument("--data-dir", type=Path, default=Path("data_raw"),
                        help="入力ディレクトリ (default: data_raw)")
    parser.add_argument("--out-dir", type=Path, default=Path("dataset_tactile"),
                        help="出力ディレクトリ (default: dataset_tactile)")
    parser.add_argument("--size", type=int, default=128,
                        help="出力画像サイズ (default: 128)")
    parser.add_argument("--stride", type=int, default=5,
                        help="フレーム間隔 (default: 5)")
    parser.add_argument("--offset", type=int, default=10,
                        help="接触後オフセットフレーム (default: 10)")
    parser.add_argument("--max-frames", type=int, default=50,
                        help="トライアル当たり最大フレーム数 (default: 50)")
    parser.add_argument("--require-labels", action="store_true",
                        help="ラベルがないトライアルをスキップ")
    return parser.parse_args()


def load_meta(trial_dir: Path) -> dict:
    """meta.jsonを読み込む"""
    meta_path = trial_dir / "meta.json"
    if not meta_path.exists():
        return {}
    with open(meta_path, "r") as f:
        return json.load(f)


def load_labels(trial_dir: Path) -> dict:
    """labels.jsonを読み込む"""
    label_path = trial_dir / "labels.json"
    if not label_path.exists():
        return {}
    with open(label_path, "r") as f:
        data = json.load(f)

    result = {}
    for key in ["slip", "crush", "success"]:
        val = data.get(key)
        if isinstance(val, bool):
            result[key] = int(val)
        elif isinstance(val, (int, float)):
            result[key] = 1 if val else 0
        else:
            result[key] = None

    # exclude フラグ
    result["exclude"] = data.get("exclude", False)
    return result


def get_contact_start_frame(meta: dict) -> int:
    """接触開始フレームを取得"""
    return meta.get("contact_start_frame", 0)


def get_release_frame(meta: dict, total_frames: int) -> int:
    """リリースフレームを取得"""
    return meta.get("release_frame", total_frames)


def extract_frames_from_video(
    video_path: Path,
    start_frame: int,
    end_frame: int,
    stride: int,
    max_frames: int,
    target_size: int,
) -> list:
    """動画からフレームを抽出"""
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        return []

    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    end_frame = min(end_frame, total)

    frames = []
    frame_indices = []

    for idx in range(start_frame, end_frame, stride):
        if len(frames) >= max_frames:
            break

        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ret, frame = cap.read()
        if not ret:
            break

        # BGR -> RGB
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # リサイズ
        frame = cv2.resize(frame, (target_size, target_size))

        frames.append(frame)
        frame_indices.append(idx)

    cap.release()
    return list(zip(frame_indices, frames))


def process_trial(
    trial_dir: Path,
    out_dir: Path,
    size: int,
    stride: int,
    offset: int,
    max_frames: int,
    require_labels: bool,
) -> list:
    """1トライアルを処理"""
    object_id = trial_dir.parent.name
    trial_id = trial_dir.name

    raw_video = trial_dir / "gelsight_raw.avi"
    diff_video = trial_dir / "gelsight_diff.avi"

    if not raw_video.exists() or not diff_video.exists():
        return []

    meta = load_meta(trial_dir)
    labels = load_labels(trial_dir)

    # 除外フラグ
    if labels.get("exclude", False):
        return []

    # ラベル必須チェック
    if require_labels:
        if any(labels.get(k) is None for k in ["slip", "crush", "success"]):
            return []

    # フレーム範囲
    cap = cv2.VideoCapture(str(raw_video))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    cap.release()

    contact_start = get_contact_start_frame(meta)
    release_frame = get_release_frame(meta, total_frames)

    start_frame = contact_start + offset
    end_frame = release_frame

    if start_frame >= end_frame:
        return []

    # フレーム抽出
    raw_frames = extract_frames_from_video(
        raw_video, start_frame, end_frame, stride, max_frames, size
    )
    diff_frames = extract_frames_from_video(
        diff_video, start_frame, end_frame, stride, max_frames, size
    )

    if len(raw_frames) != len(diff_frames):
        # フレーム数が一致しない場合は短い方に合わせる
        min_len = min(len(raw_frames), len(diff_frames))
        raw_frames = raw_frames[:min_len]
        diff_frames = diff_frames[:min_len]

    if not raw_frames:
        return []

    # npz保存
    frames_dir = out_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    results = []
    for (frame_idx, raw_img), (_, diff_img) in zip(raw_frames, diff_frames):
        filename = f"{object_id}__{trial_id}__f{frame_idx:06d}.npz"
        npz_path = frames_dir / filename

        np.savez_compressed(
            npz_path,
            frame_raw=raw_img.astype(np.uint8),
            frame_diff=diff_img.astype(np.uint8),
            object_id=object_id,
            trial_id=trial_id,
            frame_idx=frame_idx,
            slip=labels.get("slip", -1),
            crush=labels.get("crush", -1),
            success=labels.get("success", -1),
        )

        results.append({
            "object_id": object_id,
            "trial_id": trial_id,
            "frame_idx": frame_idx,
            "npz_path": str(npz_path.relative_to(out_dir)),
            "slip": labels.get("slip", -1),
            "crush": labels.get("crush", -1),
            "success": labels.get("success", -1),
        })

    return results


def main():
    args = parse_args()

    data_dir = args.data_dir
    out_dir = args.out_dir

    if not data_dir.exists():
        print(f"Error: {data_dir} does not exist", file=sys.stderr)
        sys.exit(1)

    out_dir.mkdir(parents=True, exist_ok=True)

    # トライアルを収集
    trials = []
    for obj_dir in sorted(data_dir.iterdir()):
        if not obj_dir.is_dir():
            continue
        for trial_dir in sorted(obj_dir.iterdir()):
            if trial_dir.is_dir() and trial_dir.name.startswith("trial_"):
                trials.append(trial_dir)

    print(f"Found {len(trials)} trials")

    # 処理
    all_results = []
    stats = {
        "total_trials": len(trials),
        "processed_trials": 0,
        "total_frames": 0,
        "per_object": {},
    }

    for trial_dir in tqdm(trials, desc="Processing trials"):
        results = process_trial(
            trial_dir,
            out_dir,
            args.size,
            args.stride,
            args.offset,
            args.max_frames,
            args.require_labels,
        )

        if results:
            all_results.extend(results)
            stats["processed_trials"] += 1
            stats["total_frames"] += len(results)

            obj_id = results[0]["object_id"]
            if obj_id not in stats["per_object"]:
                stats["per_object"][obj_id] = {"trials": 0, "frames": 0}
            stats["per_object"][obj_id]["trials"] += 1
            stats["per_object"][obj_id]["frames"] += len(results)

    # index.csv 保存
    index_path = out_dir / "index.csv"
    with open(index_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "object_id", "trial_id", "frame_idx", "npz_path",
            "slip", "crush", "success"
        ])
        writer.writeheader()
        writer.writerows(all_results)

    # summary.json 保存
    summary = {
        "created_at": datetime.now().isoformat(),
        "args": vars(args),
        "stats": stats,
        "label_distribution": {
            "slip": {"0": 0, "1": 0, "-1": 0},
            "crush": {"0": 0, "1": 0, "-1": 0},
            "success": {"0": 0, "1": 0, "-1": 0},
        },
    }

    for r in all_results:
        for key in ["slip", "crush", "success"]:
            val = str(r[key])
            summary["label_distribution"][key][val] += 1

    # Path を文字列に変換
    summary["args"]["data_dir"] = str(summary["args"]["data_dir"])
    summary["args"]["out_dir"] = str(summary["args"]["out_dir"])

    with open(out_dir / "summary.json", "w") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    print(f"\nDone!")
    print(f"  Processed: {stats['processed_trials']}/{stats['total_trials']} trials")
    print(f"  Total frames: {stats['total_frames']}")
    print(f"  Output: {out_dir}")

    # ラベル分布表示
    print("\nLabel distribution:")
    for key in ["slip", "crush", "success"]:
        dist = summary["label_distribution"][key]
        print(f"  {key}: 0={dist['0']}, 1={dist['1']}, unknown={dist['-1']}")


if __name__ == "__main__":
    main()
