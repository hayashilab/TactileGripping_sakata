#!/usr/bin/env python3
"""
つぶれ検出用データセット構築

crush_frameを境界として、各フレームに0（safe）または1（crush）のラベルを付ける。

注意: contact_log.csvのtimestampを使って、CONTACTフレーム番号を
      実際の動画フレーム番号に変換する。

使用例:
    python -m grasp_everything.preprocessing.build_crush_dataset \
        --data-dir data_crush \
        --out-dir dataset_crush \
        --size 128
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
    parser = argparse.ArgumentParser(description="つぶれ検出用データセット構築")
    parser.add_argument("--data-dir", type=Path, default=Path("data_crush"),
                        help="入力ディレクトリ")
    parser.add_argument("--out-dir", type=Path, default=Path("dataset_crush"),
                        help="出力ディレクトリ")
    parser.add_argument("--size", type=int, default=128,
                        help="出力画像サイズ")
    parser.add_argument("--stride", type=int, default=3,
                        help="フレーム間隔")
    parser.add_argument("--margin-frames", type=int, default=15,
                        help="crush_frame前後の除外フレーム数（動画フレーム単位、曖昧な境界を除外）")
    parser.add_argument("--fps", type=float, default=30.0,
                        help="動画のFPS（タイムスタンプ→フレーム変換用）")
    return parser.parse_args()


def load_meta(trial_dir: Path) -> dict:
    meta_path = trial_dir / "meta.json"
    if not meta_path.exists():
        return {}
    with open(meta_path) as f:
        return json.load(f)


def load_labels(trial_dir: Path) -> dict:
    label_path = trial_dir / "labels.json"
    if not label_path.exists():
        return {}
    with open(label_path) as f:
        return json.load(f)


def get_actual_crush_frame(trial_dir: Path, contact_crush_frame: int, fps: float) -> int:
    """
    contact_log.csvを使って、CONTACTフレーム番号を実際の動画フレーム番号に変換

    Args:
        trial_dir: トライアルディレクトリ
        contact_crush_frame: CONTACTフレーム番号（meta.jsonのcrush_frame）
        fps: 動画のFPS

    Returns:
        実際の動画フレーム番号
    """
    contact_log = trial_dir / "contact_log.csv"
    if not contact_log.exists():
        return None

    with open(contact_log) as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame_num = int(row["frame"])
            if frame_num == contact_crush_frame:
                timestamp = float(row["timestamp"])
                # タイムスタンプから動画フレーム番号を計算
                actual_frame = int(timestamp * fps)
                return actual_frame

    return None


def extract_frames(
    video_path: Path,
    size: int,
    stride: int,
) -> list:
    """動画からフレームを抽出"""
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        return []

    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frames = []

    for idx in range(0, total, stride):
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = cv2.resize(frame, (size, size))
        frames.append((idx, frame))

    cap.release()
    return frames


def process_trial(
    trial_dir: Path,
    out_dir: Path,
    size: int,
    stride: int,
    margin_frames: int,
    fps: float,
) -> list:
    """1トライアルを処理"""
    object_id = trial_dir.parent.name
    trial_id = trial_dir.name

    diff_video = trial_dir / "gelsight_diff.avi"

    if not diff_video.exists():
        return []

    meta = load_meta(trial_dir)
    labels = load_labels(trial_dir)

    if labels.get("exclude", False):
        return []

    # CONTACTフレーム番号を取得
    contact_crush_frame = labels.get("crush_frame")

    # 実際の動画フレーム番号に変換
    actual_crush_frame = None
    if contact_crush_frame is not None:
        actual_crush_frame = get_actual_crush_frame(trial_dir, contact_crush_frame, fps)
        if actual_crush_frame is None:
            # contact_logが見つからない場合はスキップ
            print(f"Warning: Cannot find timestamp for crush_frame in {trial_dir}")
            return []

    # フレーム抽出
    diff_frames = extract_frames(diff_video, size, stride)

    if not diff_frames:
        return []

    # 出力ディレクトリ
    frames_dir = out_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    results = []

    for frame_idx, diff_img in diff_frames:
        # ラベル決定（actual_crush_frameを使用）
        if actual_crush_frame is None:
            # つぶれなかった → 全フレーム safe
            label = 0
        elif frame_idx < actual_crush_frame - margin_frames:
            # crush_frame より前 → safe
            label = 0
        elif frame_idx > actual_crush_frame + margin_frames:
            # crush_frame より後 → crush
            label = 1
        else:
            # 境界付近 → スキップ
            continue

        filename = f"{object_id}__{trial_id}__f{frame_idx:06d}.npz"
        npz_path = frames_dir / filename

        np.savez_compressed(
            npz_path,
            frame_diff=diff_img.astype(np.uint8),
            object_id=object_id,
            trial_id=trial_id,
            frame_idx=frame_idx,
            crush=label,
            crush_frame=actual_crush_frame if actual_crush_frame else -1,
        )

        results.append({
            "object_id": object_id,
            "trial_id": trial_id,
            "frame_idx": frame_idx,
            "npz_path": str(npz_path.relative_to(out_dir)),
            "crush": label,
        })

    return results


def main():
    args = parse_args()

    if not args.data_dir.exists():
        print(f"Error: {args.data_dir} does not exist")
        sys.exit(1)

    args.out_dir.mkdir(parents=True, exist_ok=True)

    # トライアル収集
    trials = []
    for obj_dir in sorted(args.data_dir.iterdir()):
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
        "safe_frames": 0,
        "crush_frames": 0,
    }

    for trial_dir in tqdm(trials, desc="Processing"):
        results = process_trial(
            trial_dir, args.out_dir, args.size, args.stride, args.margin_frames, args.fps
        )

        if results:
            all_results.extend(results)
            stats["processed_trials"] += 1
            stats["total_frames"] += len(results)
            stats["safe_frames"] += sum(1 for r in results if r["crush"] == 0)
            stats["crush_frames"] += sum(1 for r in results if r["crush"] == 1)

    # index.csv
    with open(args.out_dir / "index.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "object_id", "trial_id", "frame_idx", "npz_path", "crush"
        ])
        writer.writeheader()
        writer.writerows(all_results)

    # summary.json
    summary = {
        "created_at": datetime.now().isoformat(),
        "args": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
        "stats": stats,
    }
    with open(args.out_dir / "summary.json", "w") as f:
        json.dump(summary, f, indent=2)

    print(f"\nDone!")
    print(f"  Processed: {stats['processed_trials']}/{stats['total_trials']} trials")
    print(f"  Total frames: {stats['total_frames']}")
    print(f"  Safe: {stats['safe_frames']}, Crush: {stats['crush_frames']}")
    print(f"  Output: {args.out_dir}")


if __name__ == "__main__":
    main()
