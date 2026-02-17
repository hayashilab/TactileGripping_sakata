"""
AVIからPNG連番に展開するスクリプト

Usage:
    python prep_extract_frames.py --data_dir data_raw
    python prep_extract_frames.py --data_dir data_raw --trial S1/trial_0001
"""

import argparse
import cv2
from pathlib import Path
from tqdm import tqdm


def extract_frames_from_avi(avi_path: Path, output_dir: Path, verbose: bool = False):
    """
    AVIファイルからPNG連番を抽出
    
    Args:
        avi_path: 入力AVIファイルパス
        output_dir: 出力ディレクトリ（frames_raw/ or frames_diff/）
        verbose: 詳細出力
    """
    if not avi_path.exists():
        if verbose:
            print(f"  Skip (not found): {avi_path}")
        return 0
    
    # 出力ディレクトリ作成
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # 既に展開済みならスキップ
    existing_frames = list(output_dir.glob("*.png"))
    if len(existing_frames) > 0:
        if verbose:
            print(f"  Skip (already extracted): {output_dir} ({len(existing_frames)} frames)")
        return len(existing_frames)
    
    # 動画読み込み
    cap = cv2.VideoCapture(str(avi_path))
    if not cap.isOpened():
        print(f"  Error: Cannot open {avi_path}")
        return 0
    
    frame_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # PNG保存（6桁ゼロパディング）
        output_path = output_dir / f"{frame_count:06d}.png"
        cv2.imwrite(str(output_path), frame)
        frame_count += 1
    
    cap.release()
    
    if verbose:
        print(f"  Extracted: {avi_path.name} -> {frame_count} frames")
    
    return frame_count


def process_trial(trial_dir: Path, verbose: bool = False):
    """
    1つのtrialディレクトリを処理
    """
    if verbose:
        print(f"Processing: {trial_dir}")
    
    # raw
    raw_avi = trial_dir / "gelsight_raw.avi"
    raw_frames_dir = trial_dir / "frames_raw"
    extract_frames_from_avi(raw_avi, raw_frames_dir, verbose)
    
    # diff
    diff_avi = trial_dir / "gelsight_diff.avi"
    diff_frames_dir = trial_dir / "frames_diff"
    extract_frames_from_avi(diff_avi, diff_frames_dir, verbose)


def main():
    parser = argparse.ArgumentParser(description="Extract frames from AVI files")
    parser.add_argument("--data_dir", type=str, default="data_raw", help="Data directory")
    parser.add_argument("--trial", type=str, default=None, help="Specific trial (e.g., S1/trial_0001)")
    parser.add_argument("--verbose", "-v", action="store_true", help="Verbose output")
    args = parser.parse_args()
    
    data_dir = Path(args.data_dir)
    
    if args.trial:
        # 特定のtrialのみ処理
        trial_dir = data_dir / args.trial
        if trial_dir.exists():
            process_trial(trial_dir, verbose=True)
        else:
            print(f"Trial not found: {trial_dir}")
    else:
        # 全trial処理
        trial_dirs = sorted(data_dir.glob("*/trial_*"))
        print(f"Found {len(trial_dirs)} trials")
        
        for trial_dir in tqdm(trial_dirs, desc="Extracting frames"):
            process_trial(trial_dir, verbose=args.verbose)
    
    print("Done!")


if __name__ == "__main__":
    main()
