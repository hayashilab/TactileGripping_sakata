"""
RealSense RGB画像からOpenCLIP埋め込みを事前計算するスクリプト

Usage:
    python prep_clip_emb.py --data_dir data_raw --output_dir data_processed
"""

import argparse
import json
import numpy as np
import torch
from pathlib import Path
from PIL import Image
from tqdm import tqdm

import open_clip


def load_clip_model(model_name: str = "ViT-B-32", pretrained: str = "openai", device: str = "cuda"):
    """
    OpenCLIPモデルを読み込み
    """
    model, _, preprocess = open_clip.create_model_and_transforms(model_name, pretrained=pretrained)
    model = model.to(device)
    model.eval()
    return model, preprocess


def compute_clip_embedding(model, preprocess, image_path: Path, device: str = "cuda") -> np.ndarray:
    """
    1枚の画像からCLIP埋め込みを計算
    
    Returns:
        (512,) or (768,) depending on model
    """
    image = Image.open(image_path).convert("RGB")
    image_tensor = preprocess(image).unsqueeze(0).to(device)
    
    with torch.no_grad():
        features = model.encode_image(image_tensor)
        features = features / features.norm(dim=-1, keepdim=True)
    
    return features[0].cpu().numpy()


def process_trial(
    trial_dir: Path,
    output_dir: Path,
    model,
    preprocess,
    device: str,
    verbose: bool = False,
):
    """
    1つのtrialを処理
    """
    # RealSense画像確認
    rgb_path = trial_dir / "realsense_rgb_pre.png"
    if not rgb_path.exists():
        if verbose:
            print(f"  Skip (no RGB): {trial_dir}")
        return False
    
    # 出力ディレクトリ
    rel_path = trial_dir.relative_to(trial_dir.parent.parent)
    out_trial_dir = output_dir / rel_path
    out_trial_dir.mkdir(parents=True, exist_ok=True)
    
    # 既に計算済みならスキップ
    out_path = out_trial_dir / "clip_emb.npy"
    if out_path.exists():
        if verbose:
            print(f"  Skip (already exists): {out_path}")
        return True
    
    # CLIP埋め込み計算
    clip_emb = compute_clip_embedding(model, preprocess, rgb_path, device)
    
    # 保存
    np.save(out_path, clip_emb)
    
    if verbose:
        print(f"  Computed: {trial_dir.name} -> clip_emb.npy (dim={clip_emb.shape[0]})")
    
    return True


def main():
    parser = argparse.ArgumentParser(description="Compute CLIP embeddings for RealSense images")
    parser.add_argument("--data_dir", type=str, default="data_raw", help="Input data directory")
    parser.add_argument("--output_dir", type=str, default="data_processed", help="Output directory")
    parser.add_argument("--model_name", type=str, default="ViT-B-32", help="CLIP model name")
    parser.add_argument("--pretrained", type=str, default="openai", help="Pretrained weights")
    parser.add_argument("--device", type=str, default="cuda", help="Device (cuda or cpu)")
    parser.add_argument("--verbose", "-v", action="store_true", help="Verbose output")
    args = parser.parse_args()
    
    data_dir = Path(args.data_dir)
    output_dir = Path(args.output_dir)
    device = args.device if torch.cuda.is_available() else "cpu"
    
    print(f"Loading CLIP model: {args.model_name} ({args.pretrained})")
    print(f"Device: {device}")
    model, preprocess = load_clip_model(args.model_name, args.pretrained, device)
    
    # 全trial処理
    trial_dirs = sorted(data_dir.glob("*/trial_*"))
    print(f"Found {len(trial_dirs)} trials")
    
    success_count = 0
    for trial_dir in tqdm(trial_dirs, desc="Computing CLIP embeddings"):
        success = process_trial(
            trial_dir,
            output_dir,
            model,
            preprocess,
            device,
            verbose=args.verbose,
        )
        if success:
            success_count += 1
    
    print(f"Done! Processed {success_count}/{len(trial_dirs)} trials")


if __name__ == "__main__":
    main()
