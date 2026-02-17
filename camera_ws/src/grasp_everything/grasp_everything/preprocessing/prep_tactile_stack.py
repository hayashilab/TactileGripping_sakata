"""
contact_start基準で触覚画像をK枚抽出してnpy化するスクリプト

Usage:
    python prep_tactile_stack.py --data_dir data_raw --output_dir data_processed
    python prep_tactile_stack.py --data_dir data_raw --output_dir data_processed --num_frames 5
"""

import argparse
import json
import cv2
import numpy as np
from pathlib import Path
from tqdm import tqdm


def load_frames(frames_dir: Path, frame_indices: list, target_size: tuple = (64, 64)) -> np.ndarray:
    """
    指定フレームを読み込んでスタック
    
    Args:
        frames_dir: PNG連番ディレクトリ
        frame_indices: 読み込むフレーム番号のリスト
        target_size: リサイズ後のサイズ (H, W)
    
    Returns:
        (K, 3, H, W) の numpy array (float32, 0-1正規化)
    """
    frames = []
    
    for idx in frame_indices:
        frame_path = frames_dir / f"{idx:06d}.png"
        
        if not frame_path.exists():
            # フレームが存在しない場合は黒画像
            frame = np.zeros((target_size[0], target_size[1], 3), dtype=np.uint8)
        else:
            frame = cv2.imread(str(frame_path))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = cv2.resize(frame, (target_size[1], target_size[0]))
        
        # (H, W, C) -> (C, H, W), 0-1正規化
        frame = frame.transpose(2, 0, 1).astype(np.float32) / 255.0
        frames.append(frame)
    
    return np.stack(frames, axis=0)  # (K, 3, H, W)


def get_frame_indices(contact_start: int, num_frames: int, frame_interval: int = 5) -> list:
    """
    contact_start基準でフレームインデックスを生成
    
    例: contact_start=66, num_frames=3, interval=5
        → [66, 71, 76]
    """
    return [contact_start + i * frame_interval for i in range(num_frames)]


def process_trial(
    trial_dir: Path,
    output_dir: Path,
    num_frames: int = 3,
    frame_interval: int = 5,
    target_size: tuple = (64, 64),
    verbose: bool = False,
):
    """
    1つのtrialを処理
    """
    # meta.json読み込み
    meta_path = trial_dir / "meta.json"
    if not meta_path.exists():
        if verbose:
            print(f"  Skip (no meta.json): {trial_dir}")
        return False
    
    with open(meta_path) as f:
        meta = json.load(f)
    
    contact_start = meta.get("contact_start_frame")
    if contact_start is None:
        if verbose:
            print(f"  Skip (no contact_start_frame): {trial_dir}")
        return False
    
    # フレームディレクトリ確認
    frames_raw_dir = trial_dir / "frames_raw"
    frames_diff_dir = trial_dir / "frames_diff"
    
    if not frames_raw_dir.exists() or not frames_diff_dir.exists():
        if verbose:
            print(f"  Skip (no frames dir): {trial_dir}")
        return False
    
    # 出力ディレクトリ作成
    rel_path = trial_dir.relative_to(trial_dir.parent.parent)
    out_trial_dir = output_dir / rel_path
    out_trial_dir.mkdir(parents=True, exist_ok=True)
    
    # フレームインデックス計算
    frame_indices = get_frame_indices(contact_start, num_frames, frame_interval)
    
    # 触覚画像スタック
    tactile_raw = load_frames(frames_raw_dir, frame_indices, target_size)
    tactile_diff = load_frames(frames_diff_dir, frame_indices, target_size)
    
    # 保存
    np.save(out_trial_dir / "tactile_raw.npy", tactile_raw)
    np.save(out_trial_dir / "tactile_diff.npy", tactile_diff)
    
    # labels.jsonをコピー
    labels_path = trial_dir / "labels.json"
    if labels_path.exists():
        import shutil
        shutil.copy(labels_path, out_trial_dir / "labels.json")
    
    # meta.jsonをコピー（処理情報を追加）
    meta["preprocessing"] = {
        "num_frames": num_frames,
        "frame_interval": frame_interval,
        "frame_indices": frame_indices,
        "target_size": list(target_size),
    }
    with open(out_trial_dir / "meta.json", "w") as f:
        json.dump(meta, f, indent=2)
    
    if verbose:
        print(f"  Processed: {trial_dir.name} -> frames {frame_indices}")
    
    return True


def main():
    parser = argparse.ArgumentParser(description="Prepare tactile stacks from frames")
    parser.add_argument("--data_dir", type=str, default="data_raw", help="Input data directory")
    parser.add_argument("--output_dir", type=str, default="data_processed", help="Output directory")
    parser.add_argument("--num_frames", "-K", type=int, default=3, help="Number of frames to stack")
    parser.add_argument("--frame_interval", type=int, default=5, help="Interval between frames")
    parser.add_argument("--target_size", type=int, nargs=2, default=[64, 64], help="Target image size (H W)")
    parser.add_argument("--verbose", "-v", action="store_true", help="Verbose output")
    args = parser.parse_args()
    
    data_dir = Path(args.data_dir)
    output_dir = Path(args.output_dir)
    target_size = tuple(args.target_size)
    
    # 全trial処理
    trial_dirs = sorted(data_dir.glob("*/trial_*"))
    print(f"Found {len(trial_dirs)} trials")
    print(f"Settings: K={args.num_frames}, interval={args.frame_interval}, size={target_size}")
    
    success_count = 0
    for trial_dir in tqdm(trial_dirs, desc="Processing trials"):
        success = process_trial(
            trial_dir,
            output_dir,
            num_frames=args.num_frames,
            frame_interval=args.frame_interval,
            target_size=target_size,
            verbose=args.verbose,
        )
        if success:
            success_count += 1
    
    print(f"Done! Processed {success_count}/{len(trial_dirs)} trials")


if __name__ == "__main__":
    main()
