#!/usr/bin/env python3
# build_frames_after_contact_dual_profiled_min.py

import argparse, csv, json
from pathlib import Path
from typing import Dict, Any, Optional, Tuple, List
from collections import Counter, defaultdict

import cv2
import numpy as np


PROFILES: Dict[str, Dict[str, Any]] = {
    "fast":     dict(warmup_frames=0,  contact_check_frames=0,  min_contact_energy=0.0,
                     tail_flat_frames=0,  tail_flat_energy=0.0,  dup_threshold=0.80,
                     pad_last=True, delete_trial_on_fail_quality=False),
    "balanced": dict(warmup_frames=10, contact_check_frames=20, min_contact_energy=0.02,
                     tail_flat_frames=30, tail_flat_energy=0.002, dup_threshold=0.30,
                     pad_last=True, delete_trial_on_fail_quality=True),
    "strict":   dict(warmup_frames=15, contact_check_frames=30, min_contact_energy=0.03,
                     tail_flat_frames=20, tail_flat_energy=0.003, dup_threshold=0.20,
                     pad_last=True, delete_trial_on_fail_quality=True),
}


def rjson(p: Path) -> Dict[str, Any]:
    try: return json.loads(p.read_text(encoding="utf-8"))
    except Exception: return {}


def wjson(p: Path, obj: Dict[str, Any]):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(obj, indent=2, ensure_ascii=False), encoding="utf-8")


def contact_start_from_meta(meta: Dict[str, Any]) -> Optional[int]:
    v = meta.get("contact_start_frame")
    if isinstance(v, int): return int(v)
    fr = meta.get("frames")
    if isinstance(fr, dict) and isinstance(fr.get("contact_start_frame"), int):
        return int(fr["contact_start_frame"])
    return None


def contact_time_from_log(csv_path: Path) -> Optional[float]:
    if not csv_path.exists(): return None
    try:
        with csv_path.open("r", newline="", encoding="utf-8") as f:
            rd = csv.reader(f)
            hdr = next(rd, None)
            if not hdr: return None
            idx_ts = next((i for i, h in enumerate(hdr) if h.strip().lower() == "timestamp"), 0)
            idx_st = next((i for i, h in enumerate(hdr) if h.strip().lower() == "state"), 1 if len(hdr) > 1 else None)
            if idx_st is None: return None
            for row in rd:
                if idx_st < len(row) and row[idx_st].strip() == "CONTACT":
                    try: return float(row[idx_ts])
                    except Exception: return None
    except Exception:
        return None
    return None


def it_trials(data_dir: Path):
    for obj in sorted([p for p in data_dir.iterdir() if p.is_dir()]):
        for tr in sorted([p for p in obj.iterdir() if p.is_dir() and p.name.startswith("trial_")]):
            yield tr


def open_cap(p: Path) -> Optional[cv2.VideoCapture]:
    if not p.exists(): return None
    cap = cv2.VideoCapture(str(p))
    if not cap.isOpened():
        cap.release()
        return None
    return cap


def cap_info(cap: cv2.VideoCapture) -> Tuple[float, int]:
    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps <= 1e-6: fps = 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    return float(fps), int(total)


def robust_start(cap_ref: cv2.VideoCapture, meta_start: Optional[int], contact_time_s: Optional[float], fallback: int):
    fps, total = cap_info(cap_ref)
    attempts = []
    if meta_start is not None: attempts.append(("meta_start", int(meta_start)))
    if contact_time_s is not None: attempts.append(("contact_log_time", int(round(float(contact_time_s) * fps))))
    attempts.append(("fallback_start", int(fallback)))

    chosen, reason = None, None
    for name, st in attempts:
        st = max(0, int(st))
        if total > 0: st = min(st, total - 1)
        cap_ref.set(cv2.CAP_PROP_POS_FRAMES, st)
        ok, _ = cap_ref.read()
        if ok:
            chosen, reason = st, name
            break
    if chosen is None:
        chosen = max(0, total - 1) if total > 0 else 0
        reason = "clamp_end" if total > 0 else "zero_fallback"
    return int(chosen), dict(ref_video_fps=fps, ref_total_frames=total, used_start_frame=chosen, start_reason=reason, attempts=attempts)


def seek(cap: cv2.VideoCapture, idx: int):
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(max(0, idx)))


def read_next(cap: Optional[cv2.VideoCapture], last: Optional[np.ndarray], pad_last: bool):
    if cap is None: return None, last, False
    ok, fr = cap.read()
    if ok and fr is not None: return fr, fr, True
    if pad_last and last is not None: return last.copy(), last, True
    return None, last, False


def to_rgb_uint8(fr_bgr: np.ndarray, size: int) -> np.ndarray:
    fr = cv2.resize(fr_bgr, (size, size), interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(fr, cv2.COLOR_BGR2RGB).astype(np.uint8)


def diff_energy(rgb: np.ndarray) -> float:
    return float(np.mean(np.abs(rgb.astype(np.float32))) / 255.0)


def ahash8x8(rgb: np.ndarray) -> int:
    s = cv2.resize(rgb, (8, 8), interpolation=cv2.INTER_AREA).astype(np.float32)
    g = np.mean(s, axis=2)
    m = float(np.mean(g))
    bits = (g > m).astype(np.uint8).flatten()
    h = 0
    for b in bits:
        h = (h << 1) | int(b)
    return int(h)


def make_label(labels: Dict[str, Any], slip: int, crush: int, success: int, mode: str):
    if mode == "risk":
        if slip in (0, 1) and crush in (0, 1) and success in (0, 1):
            return int((slip == 1) or (crush == 1) or (success == 0))
        return -1
    return {"slip": slip, "crush": crush, "success": success, "condition": labels.get("condition", "")}


def delete_files(paths: List[Path]):
    for p in paths:
        try:
            if p.exists(): p.unlink()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default="data_raw")
    ap.add_argument("--out-dir", default="dataset_frames_npz_v3")
    ap.add_argument("--profile", choices=list(PROFILES.keys()), default="balanced")
    ap.add_argument("--config", default="", help="JSON override for profile internals")

    ap.add_argument("--size", type=int, default=160)
    ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--max-frames-per-trial", type=int, default=300)

    ap.add_argument("--label-mode", choices=["risk", "multi"], default="risk")
    ap.add_argument("--use-contact-start", action="store_true")
    ap.add_argument("--offset", type=int, default=0)
    ap.add_argument("--fallback-start", type=int, default=0)

    ap.add_argument("--require-labels", action="store_true")
    ap.add_argument("--status-ok-only", action="store_true")
    ap.add_argument("--skip-existing", action="store_true")

    ap.add_argument("--raw-name", default="gelsight_raw.avi")
    ap.add_argument("--diff-name", default="gelsight_diff.avi")
    args = ap.parse_args()

    internal = dict(PROFILES[args.profile])
    if args.config:
        cfg = rjson(Path(args.config))
        for k in list(cfg.keys()):
            if k not in internal: cfg.pop(k, None)
        internal.update(cfg)

    warmup = int(internal["warmup_frames"])
    checkK = int(internal["contact_check_frames"])
    minE = float(internal["min_contact_energy"])
    flatL = int(internal["tail_flat_frames"])
    flatE = float(internal["tail_flat_energy"])
    dup_th = float(internal["dup_threshold"])
    pad_last = bool(internal["pad_last"])
    rollback_on_quality = bool(internal["delete_trial_on_fail_quality"])

    data_dir = Path(args.data_dir)
    out_dir = Path(args.out_dir)
    frames_dir = out_dir / "frames_npz"
    frames_dir.mkdir(parents=True, exist_ok=True)

    index_rows: List[Dict[str, Any]] = []
    stats = defaultdict(Counter)
    failed_trials = []
    skipped_trials = processed_trials = processed_frames = 0

    for tr in it_trials(data_dir):
        obj_id, trial_id = tr.parent.name, tr.name

        labels_path = tr / "labels.json"
        if args.require_labels and not labels_path.exists():
            skipped_trials += 1
            continue
        labels = rjson(labels_path) if labels_path.exists() else {}
        if labels_path.exists():
            try:
                slip = int(labels.get("slip", 0))
                crush = int(labels.get("crush", 0))
                success = int(labels.get("success", 0))
            except Exception:
                skipped_trials += 1
                continue
        else:
            slip = crush = success = -1

        meta = rjson(tr / "meta.json")
        if args.status_ok_only:
            st = meta.get("status", None)
            if st is not None and st != "OK":
                skipped_trials += 1
                continue

        cstart = contact_start_from_meta(meta) if args.use_contact_start else None
        meta_start = (cstart + args.offset) if cstart is not None else None
        ctime = contact_time_from_log(tr / "contact_log.csv") if args.use_contact_start else None

        raw_path = tr / args.raw_name
        diff_path = tr / args.diff_name
        cap_raw = open_cap(raw_path)
        cap_diff = open_cap(diff_path)
        if cap_raw is None and cap_diff is None:
            failed_trials.append({"object_id": obj_id, "trial_id": trial_id, "error": "cannot_open_raw_or_diff"})
            skipped_trials += 1
            continue

        cap_ref = cap_diff if cap_diff is not None else cap_raw
        saved_files: List[Path] = []
        idx0 = len(index_rows)

        try:
            start, sinfo = robust_start(cap_ref, meta_start, ctime, args.fallback_start)
            start += warmup
            if cap_raw is not None: seek(cap_raw, start)
            if cap_diff is not None: seek(cap_diff, start)

            raw_fps, raw_total = cap_info(cap_raw) if cap_raw is not None else (None, None)
            diff_fps, diff_total = cap_info(cap_diff) if cap_diff is not None else (None, None)

            # contact plausibility (diff energy)
            if checkK > 0 and cap_diff is not None and minE > 0.0:
                pos_raw = cap_raw.get(cv2.CAP_PROP_POS_FRAMES) if cap_raw is not None else None
                pos_diff = cap_diff.get(cv2.CAP_PROP_POS_FRAMES)

                lr = ld = None
                es = []
                for _ in range(checkK):
                    fr_r, lr, ok_r = read_next(cap_raw, lr, pad_last)
                    fr_d, ld, ok_d = read_next(cap_diff, ld, pad_last)
                    if not ok_r and not ok_d: break
                    if fr_d is None: continue
                    es.append(diff_energy(to_rgb_uint8(fr_d, args.size)))

                if cap_raw is not None and pos_raw is not None: seek(cap_raw, int(pos_raw))
                if cap_diff is not None: seek(cap_diff, int(pos_diff))

                if len(es) > 0 and float(np.mean(es)) < minE:
                    skipped_trials += 1
                    continue

            lr = ld = None
            read_idx = start
            saved = 0
            flat_run = 0
            prev_h = None
            dup = tot = 0

            while True:
                fr_r, lr, ok_r = read_next(cap_raw, lr, pad_last)
                fr_d, ld, ok_d = read_next(cap_diff, ld, pad_last)
                if not ok_r and not ok_d: break

                if fr_d is not None:
                    d_rgb = to_rgb_uint8(fr_d, args.size)
                    e = diff_energy(d_rgb)
                else:
                    d_rgb = None
                    e = 0.0

                if flatL > 0 and flatE > 0.0:
                    flat_run = (flat_run + 1) if (e < flatE) else 0
                    if flat_run >= flatL: break

                if d_rgb is not None:
                    h = ahash8x8(d_rgb)
                elif fr_r is not None:
                    h = ahash8x8(to_rgb_uint8(fr_r, args.size))
                else:
                    h = None

                if h is not None:
                    tot += 1
                    if prev_h is not None and h == prev_h: dup += 1
                    prev_h = h

                if (read_idx - start) % args.stride == 0:
                    out_npz = frames_dir / f"{obj_id}__{trial_id}__f{read_idx:06d}.npz"
                    if not (args.skip_existing and out_npz.exists()):
                        x_raw = to_rgb_uint8(fr_r, args.size) if fr_r is not None else np.zeros((args.size, args.size, 3), np.uint8)
                        x_diff = d_rgb if d_rgb is not None else np.zeros((args.size, args.size, 3), np.uint8)
                        lbl = make_label(labels, slip, crush, success, args.label_mode)

                        payload = dict(
                            object_id=obj_id, trial_id=trial_id, frame_idx=int(read_idx),
                            raw_video=(raw_path.name if cap_raw is not None else ""),
                            diff_video=(diff_path.name if cap_diff is not None else ""),
                            raw_fps=(float(raw_fps) if raw_fps is not None else -1.0),
                            diff_fps=(float(diff_fps) if diff_fps is not None else -1.0),
                            raw_total_frames=(int(raw_total) if raw_total is not None else -1),
                            diff_total_frames=(int(diff_total) if diff_total is not None else -1),
                            diff_energy=float(e),
                            contact_time_s=(None if ctime is None else float(ctime)),
                            meta_contact_start_frame=(None if cstart is None else int(cstart)),
                            used_start_frame=int(sinfo["used_start_frame"]),
                            start_reason=str(sinfo["start_reason"]),
                            attempts=sinfo["attempts"],
                            profile=args.profile, label_mode=args.label_mode, label=lbl,
                        )

                        np.savez_compressed(out_npz, frame_raw=x_raw, frame_diff=x_diff, **payload)
                        saved_files.append(out_npz)

                        row = dict(
                            object_id=obj_id, trial_id=trial_id, frame_idx=int(read_idx),
                            relpath_npz=str(out_npz.relative_to(out_dir)),
                            raw_video=payload["raw_video"], diff_video=payload["diff_video"],
                            used_start_frame=payload["used_start_frame"], start_reason=payload["start_reason"],
                        )
                        if args.label_mode == "risk":
                            row["risk"] = int(lbl) if isinstance(lbl, int) else -1
                        else:
                            row.update(condition=labels.get("condition", ""), slip=slip, crush=crush, success=success)
                        index_rows.append(row)

                    processed_frames += 1
                    saved += 1
                    if args.max_frames_per_trial > 0 and saved >= args.max_frames_per_trial:
                        break

                read_idx += 1

            dup_ratio = float(dup / max(1, tot)) if tot > 0 else 0.0
            if dup_ratio > dup_th and rollback_on_quality:
                delete_files(saved_files)
                del index_rows[idx0:]
                skipped_trials += 1
                continue
            elif dup_ratio > dup_th:
                failed_trials.append({"object_id": obj_id, "trial_id": trial_id, "error": f"dup_ratio({dup_ratio:.3f})"})

            if args.label_mode == "risk":
                if slip in (0, 1) and crush in (0, 1) and success in (0, 1):
                    stats["risk"][int((slip == 1) or (crush == 1) or (success == 0))] += 1
            else:
                if slip in (0, 1): stats["slip"][slip] += 1
                if crush in (0, 1): stats["crush"][crush] += 1
                if success in (0, 1): stats["success"][success] += 1
            stats["object_id"][obj_id] += 1
            processed_trials += 1

        except Exception as e:
            delete_files(saved_files)
            del index_rows[idx0:]
            failed_trials.append({"object_id": obj_id, "trial_id": trial_id, "error": str(e)})
            skipped_trials += 1

        finally:
            if cap_raw is not None: cap_raw.release()
            if cap_diff is not None: cap_diff.release()

    index_path = out_dir / "index.csv"
    out_dir.mkdir(parents=True, exist_ok=True)
    with index_path.open("w", newline="", encoding="utf-8") as f:
        if args.label_mode == "risk":
            fields = ["object_id","trial_id","frame_idx","relpath_npz","raw_video","diff_video","used_start_frame","start_reason","risk"]
        else:
            fields = ["object_id","trial_id","frame_idx","relpath_npz","raw_video","diff_video","used_start_frame","start_reason","condition","slip","crush","success"]
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in index_rows: w.writerow(r)

    summary = dict(
        data_dir=str(data_dir), out_dir=str(out_dir), profile=args.profile,
        processed_trials=int(processed_trials), processed_frames=int(processed_frames),
        skipped_trials=int(skipped_trials), failed_trials=failed_trials,
        size=int(args.size), stride=int(args.stride), max_frames_per_trial=int(args.max_frames_per_trial),
        use_contact_start=bool(args.use_contact_start), offset=int(args.offset), fallback_start=int(args.fallback_start),
        label_mode=args.label_mode, internal_knobs=dict(warmup_frames=warmup, contact_check_frames=checkK, min_contact_energy=minE,
                                                       tail_flat_frames=flatL, tail_flat_energy=flatE, dup_threshold=dup_th,
                                                       pad_last=pad_last, delete_trial_on_fail_quality=rollback_on_quality),
        label_stats={k: dict(v) for k, v in stats.items()},
    )
    wjson(out_dir / "summary.json", summary)

    print(f"Done. processed_trials={processed_trials} processed_frames={processed_frames} skipped_trials={skipped_trials} failed={len(failed_trials)}")
    print(f"index: {index_path}")
    print(f"summary: {out_dir/'summary.json'}")


if __name__ == "__main__":
    main()
