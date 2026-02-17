#!/usr/bin/env python3
"""
build_frames_after_contact.py
=============================
AVIから「contact_start 以降の全フレーム」を 1枚ずつ npz にして index.csv を作る。

入力:
  data_raw/<object_id>/trial_XXXX/
    - gelsight_diff.avi (優先) / gelsight_raw.avi (fallback)
    - meta.json (任意: contact_start_frame)
    - contact_log.csv (任意: timestamp,state,...)
    - labels.json (任意/必須はオプションで制御)

出力:
  <out_dir>/
    - frames_npz/<object_id>__<trial_id>__fXXXXXX.npz  (frame: uint8 [H,W,C])
    - index.csv
    - summary.json (failed_trials など)

例:
  uv run python build_frames_after_contact.py \
    --data-dir /home/hayashi/worksp/camera_ws/data_raw \
    --out-dir  /home/hayashi/worksp/camera_ws/dataset_frames_npz \
    --use-contact-start --offset 0 --size 224 --stride 1 --require-labels
"""

import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, Any, Optional, Tuple, List

import cv2
import numpy as np


# ----------------------------
# Utility
# ----------------------------
def read_json(p: Path) -> Dict[str, Any]:
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except Exception:
        return {}


def write_json(p: Path, obj: Dict[str, Any]):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(obj, indent=2, ensure_ascii=False), encoding="utf-8")


def choose_video_candidates(trial_dir: Path) -> List[Path]:
    """優先順で返す（diff -> raw）"""
    out = []
    p = trial_dir / "gelsight_diff.avi"
    if p.exists():
        out.append(p)
    p = trial_dir / "gelsight_raw.avi"
    if p.exists():
        out.append(p)
    return out


def get_contact_start_frame(meta: Dict[str, Any]) -> Optional[int]:
    v = meta.get("contact_start_frame", None)
    if isinstance(v, int):
        return int(v)
    frames = meta.get("frames", None)
    if isinstance(frames, dict) and isinstance(frames.get("contact_start_frame"), int):
        return int(frames["contact_start_frame"])
    return None


def parse_contact_time_from_contact_log(csv_path: Path) -> Optional[float]:
    """
    contact_log.csv の最初の CONTACT 行の timestamp(秒) を返す。
    典型: timestamp,state,energy,gripper_step
    """
    if not csv_path.exists():
        return None
    try:
        with csv_path.open("r", newline="", encoding="utf-8") as f:
            reader = csv.reader(f)
            header = next(reader, None)
            if not header:
                return None

            idx_ts = None
            idx_state = None
            for i, h in enumerate(header):
                hl = h.strip().lower()
                if hl == "timestamp":
                    idx_ts = i
                elif hl == "state":
                    idx_state = i

            if idx_ts is None:
                idx_ts = 0
            if idx_state is None:
                idx_state = 1 if len(header) > 1 else None
            if idx_state is None:
                return None

            for row in reader:
                if idx_state < len(row) and row[idx_state].strip() == "CONTACT":
                    if idx_ts < len(row):
                        try:
                            return float(row[idx_ts])
                        except Exception:
                            return None
    except Exception:
        return None
    return None


def safe_resize(frame_bgr: np.ndarray, size: int) -> np.ndarray:
    return cv2.resize(frame_bgr, (size, size), interpolation=cv2.INTER_AREA)


def iter_trials(data_dir: Path) -> List[Path]:
    trials = []
    for obj_dir in sorted([p for p in data_dir.iterdir() if p.is_dir()]):
        for tr in sorted([p for p in obj_dir.iterdir() if p.is_dir() and p.name.startswith("trial_")]):
            trials.append(tr)
    return trials


# ----------------------------
# Core: robust start frame
# ----------------------------
def compute_start_frame_robust(
    cap: cv2.VideoCapture,
    meta_start: Optional[int],
    contact_time_s: Optional[float],
    fallback_start: int,
) -> Tuple[int, Dict[str, Any]]:
    """
    start_frame を頑健に決める:
      1) meta_start があればそれ
      2) contact_time_s があれば time*fps
      3) fallback_start
    ただし total_frames が分かるなら clamp する（0..total-1）。
    """
    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps <= 1e-6:
        fps = 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)

    attempts = []
    if meta_start is not None:
        attempts.append(("meta_start", int(meta_start)))
    if contact_time_s is not None:
        est = int(round(float(contact_time_s) * float(fps)))
        attempts.append(("contact_log_time", est))
    attempts.append(("fallback_start", int(fallback_start)))

    chosen = None
    reason = None

    for name, st in attempts:
        st = max(0, int(st))
        if total > 0:
            # total-1 を超える場合は末尾に寄せる
            st = min(st, total - 1)
        # 「このstartから1枚でも読める」ことを軽く検証
        cap.set(cv2.CAP_PROP_POS_FRAMES, st)
        ok, _ = cap.read()
        if ok:
            chosen = st
            reason = name
            break

    if chosen is None:
        # どれも読めない場合は、可能なら末尾から
        if total > 0:
            chosen = max(0, total - 1)
            reason = "clamp_end"
        else:
            chosen = 0
            reason = "zero_fallback"

    info = {
        "video_fps": float(fps),
        "total_frames": int(total),
        "used_start_frame": int(chosen),
        "start_reason": str(reason),
        "attempts": attempts,
    }
    return chosen, info


def open_video_robust(video_path: Path) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video: {video_path}")
    return cap


def save_frame_npz(out_npz: Path, frame_bgr: np.ndarray, size: int, payload: Dict[str, Any]):
    """
    frame_bgr: BGR uint8
    保存: frame: uint8 [H,W,C] RGB
    """
    fr = safe_resize(frame_bgr, size)
    fr = cv2.cvtColor(fr, cv2.COLOR_BGR2RGB)
    fr = fr.astype(np.uint8)

    out_npz.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out_npz, frame=fr, **payload)


# ----------------------------
# Main
# ----------------------------
def main():
    ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--data-dir", default="data_raw")
    ap.add_argument("--out-dir", default="dataset_frames_npz")
    ap.add_argument("--size", type=int, default=224)
    ap.add_argument("--use-contact-start", action="store_true")
    ap.add_argument("--offset", type=int, default=0, help="contact_start_frame に足すオフセット")
    ap.add_argument("--fallback-start", type=int, default=0)
    ap.add_argument("--stride", type=int, default=1, help="NならN枚に1枚を保存（冗長性削減）")
    ap.add_argument("--max-frames", type=int, default=-1, help="contact_start以降に最大何枚保存するか。-1は無制限")
    ap.add_argument("--skip-existing", action="store_true")
    ap.add_argument("--require-labels", action="store_true")
    ap.add_argument("--status-ok-only", action="store_true")
    args = ap.parse_args()

    data_dir = Path(args.data_dir)
    out_dir = Path(args.out_dir)
    frames_dir = out_dir / "frames_npz"
    frames_dir.mkdir(parents=True, exist_ok=True)

    trials = iter_trials(data_dir)
    if not trials:
        print(f"No trials found: {data_dir}")
        return

    index_rows = []
    stats = defaultdict(Counter)
    failed_trials = []
    skipped = 0
    processed_frames = 0
    processed_trials = 0

    for tr in trials:
        obj_id = tr.parent.name
        trial_id = tr.name

        # labels
        labels_path = tr / "labels.json"
        if args.require_labels and (not labels_path.exists()):
            skipped += 1
            continue
        labels = read_json(labels_path) if labels_path.exists() else {}

        # 必要ならラベル形式をここで確定（あなたの既存コードに合わせる）
        if labels_path.exists():
            try:
                slip = int(labels.get("slip", 0))
                crush = int(labels.get("crush", 0))
                success = int(labels.get("success", 0))
            except Exception:
                skipped += 1
                continue
        else:
            slip, crush, success = -1, -1, -1

        meta = read_json(tr / "meta.json")
        if args.status_ok_only:
            st = meta.get("status", None)
            if st is not None and st != "OK":
                skipped += 1
                continue

        # contact_start 推定に使う情報
        contact_start = get_contact_start_frame(meta) if args.use_contact_start else None
        meta_start = (contact_start + int(args.offset)) if contact_start is not None else None
        contact_time_s = parse_contact_time_from_contact_log(tr / "contact_log.csv") if args.use_contact_start else None

        candidates = choose_video_candidates(tr)
        if not candidates:
            skipped += 1
            continue

        # まず video を開けるものを探す（diff->raw）
        cap = None
        vid_used = None
        last_err = None
        for vid in candidates:
            try:
                cap = open_video_robust(vid)
                vid_used = vid
                break
            except Exception as e:
                last_err = str(e)
                cap = None
                vid_used = None

        if cap is None or vid_used is None:
            failed_trials.append({
                "object_id": obj_id,
                "trial_id": trial_id,
                "error": last_err or "cannot_open_any_video",
                "candidates": [v.name for v in candidates],
            })
            skipped += 1
            continue

        try:
            # start frame を robust に決定
            start_frame, info = compute_start_frame_robust(
                cap=cap,
                meta_start=meta_start,
                contact_time_s=contact_time_s,
                fallback_start=int(args.fallback_start),
            )

            # start_frame から順に読み進めて、strideで保存
            total = info["total_frames"]  # 0 の可能性あり（環境によっては取得失敗）
            fps = info["video_fps"]

            # いったん start_frame にシークし直す（compute_start_frame_robust で read 済みのため）
            cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)

            saved = 0
            read_idx = start_frame
            # compute_start_frame_robust 内で1フレーム read している可能性があるので、
            # ここは POS をセットした後に読み直す構造でOK（重複保存は stride/idx で回避可能）
            while True:
                ok, frame = cap.read()
                if not ok or frame is None:
                    break

                # stride適用
                if (read_idx - start_frame) % int(args.stride) == 0:
                    out_npz = frames_dir / f"{obj_id}__{trial_id}__f{read_idx:06d}.npz"
                    if (not args.skip_existing) or (not out_npz.exists()):
                        payload = {
                            "object_id": obj_id,
                            "trial_id": trial_id,
                            "frame_idx": int(read_idx),
                            "video": str(vid_used.name),
                            "video_fps": float(fps),
                            "total_frames": int(total),
                            "contact_time_s": (None if contact_time_s is None else float(contact_time_s)),
                            "meta_contact_start_frame": (None if contact_start is None else int(contact_start)),
                            "used_start_frame": int(info["used_start_frame"]),
                            "start_reason": str(info["start_reason"]),
                            "attempts": info["attempts"],
                            "labels": {
                                "condition": labels.get("condition", ""),
                                "slip": slip,
                                "crush": crush,
                                "success": success,
                            },
                        }
                        save_frame_npz(out_npz, frame, int(args.size), payload)

                    index_rows.append({
                        "object_id": obj_id,
                        "trial_id": trial_id,
                        "frame_idx": int(read_idx),
                        "relpath_npz": str(out_npz.relative_to(out_dir)),
                        "video": str(vid_used.name),
                        "video_fps": float(fps),
                        "used_start_frame": int(info["used_start_frame"]),
                        "start_reason": str(info["start_reason"]),
                        "condition": labels.get("condition", ""),
                        "slip": slip,
                        "crush": crush,
                        "success": success,
                    })

                    processed_frames += 1
                    saved += 1

                    if args.max_frames > 0 and saved >= int(args.max_frames):
                        break

                read_idx += 1

            # stats
            if slip in (0, 1):
                stats["slip"][slip] += 1
            if crush in (0, 1):
                stats["crush"][crush] += 1
            if success in (0, 1):
                stats["success"][success] += 1
            stats["object_id"][obj_id] += 1
            processed_trials += 1

        except Exception as e:
            failed_trials.append({
                "object_id": obj_id,
                "trial_id": trial_id,
                "error": str(e),
                "meta_start": (None if meta_start is None else int(meta_start)),
                "contact_time_s": contact_time_s,
                "candidates": [v.name for v in candidates],
            })
            skipped += 1
        finally:
            cap.release()

    # index.csv
    out_dir.mkdir(parents=True, exist_ok=True)
    index_path = out_dir / "index.csv"
    with index_path.open("w", newline="", encoding="utf-8") as f:
        fieldnames = [
            "object_id", "trial_id", "frame_idx", "relpath_npz",
            "video", "video_fps", "used_start_frame", "start_reason",
            "condition", "slip", "crush", "success"
        ]
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in index_rows:
            w.writerow(r)

    summary = {
        "data_dir": str(data_dir),
        "out_dir": str(out_dir),
        "processed_trials": int(processed_trials),
        "processed_frames": int(processed_frames),
        "skipped": int(skipped),
        "failed_trials": failed_trials,
        "size": int(args.size),
        "use_contact_start": bool(args.use_contact_start),
        "offset": int(args.offset),
        "fallback_start": int(args.fallback_start),
        "stride": int(args.stride),
        "max_frames": int(args.max_frames),
        "label_stats": {k: dict(v) for k, v in stats.items()},
    }
    write_json(out_dir / "summary.json", summary)

    print(f"Done. processed_trials={processed_trials} processed_frames={processed_frames} skipped={skipped} failed={len(failed_trials)}")
    print(f"index: {index_path}")
    print(f"summary: {out_dir/'summary.json'}")


if __name__ == "__main__":
    main()
