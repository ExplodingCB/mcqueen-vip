"""Label tier A frames by projecting the surveyed track through the logged pose.

    python -m mcq_training.project_labels --track tracks/purdue \\
        --frames-dir training/frames/kart_2026-11-02_1410/2026-11-02_1410 \\
        --camera training/configs/camera_default.yaml --preview 20

Reads ``poses.csv`` next to the frames (written by ``extract_mcap.py``), writes
one PNG per frame under ``labels/<source>/<video_id>/`` and appends rows with
``method=projection, status=auto`` to ``labels/index.csv``. ``--preview N``
also writes overlays for the first N frames under ``labels/preview/`` so a
misaligned camera or a stale survey is visible before anything trains.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image

from mcq_sim.track import Track
from mcq_training import DATA
from mcq_training.camera import CameraModel
from mcq_training.dataset import LABEL_COLUMNS, append_rows, read_rows
from mcq_training.labels import load_label, overlay, render_label, save_label


def label_frames(
    track: Track,
    cam: CameraModel,
    frames_root: Path,
    frames_dir: Path,
    labels_root: Path,
    band_m: float = 0.15,
    yaw_sigma: float = np.radians(0.5),
    vehicle_mask: np.ndarray | None = None,
    range_m: float | None = None,
    require_fixed: bool = True,
    preview: int = 0,
    note: str = "",
) -> dict:
    poses = read_rows(frames_dir / "poses.csv")
    if not poses:
        raise SystemExit(f"no poses.csv in {frames_dir}; run extract_mcap.py first")
    stats = {"frames": len(poses), "labeled": 0, "skipped": 0}
    rows = []
    date = time.strftime("%Y-%m-%d")
    cams: dict[tuple[int, int], CameraModel] = {}
    for i, p in enumerate(poses):
        if require_fixed and int(float(p.get("gnss_status", 2))) != 2:
            stats["skipped"] += 1
            continue
        frame_path = frames_root / p["frame"]
        with Image.open(frame_path) as im:
            size = im.size
            rgb = np.asarray(im.convert("RGB")) if i < preview else None
        c = cams.get(size)
        if c is None:
            c = cams[size] = cam if size == (cam.width, cam.height) else cam.scaled(*size)
        mask = vehicle_mask
        if mask is not None and mask.shape != (c.height, c.width):
            mask = np.asarray(Image.fromarray(mask.astype(np.uint8) * 255).resize(size, Image.NEAREST)) > 0
        pose = (float(p["x"]), float(p["y"]), float(p["yaw"]))
        label = render_label(track, pose, c, band_m, yaw_sigma, mask, range_m)
        rel = Path(p["frame"]).with_suffix(".png")
        out = labels_root / rel
        out.parent.mkdir(parents=True, exist_ok=True)
        save_label(out, label)
        parts = Path(p["frame"]).parts
        rows.append(
            {
                "frame": p["frame"],
                "label": str(rel),
                "source": parts[0] if len(parts) > 2 else "",
                "video_id": parts[1] if len(parts) > 2 else "",
                "method": "projection",
                "status": "auto",
                "reviewer": "",
                "date": date,
                "note": note or f"band={band_m} yaw_sigma_deg={np.degrees(yaw_sigma):.2f} track={track.track_id}",
            }
        )
        if rgb is not None:
            pdir = labels_root / "preview" / rel.parent
            pdir.mkdir(parents=True, exist_ok=True)
            Image.fromarray(overlay(rgb, label)).save(pdir / rel.with_suffix(".jpg").name, quality=85)
        stats["labeled"] += 1
    append_rows(labels_root / "index.csv", LABEL_COLUMNS, rows)
    return stats


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--track", required=True, type=Path, help="track directory (track.csv + track.yaml)")
    ap.add_argument("--frames-dir", required=True, type=Path, help="directory holding the frames and poses.csv")
    ap.add_argument("--frames-root", type=Path, default=None, help="root the index paths are relative to")
    ap.add_argument("--camera", required=True, type=Path, help="camera yaml (intrinsics and mount pose)")
    ap.add_argument("--labels-root", type=Path, default=DATA / "labels")
    ap.add_argument("--band", type=float, default=0.15, help="m of ignore either side of each edge")
    ap.add_argument("--yaw-sigma-deg", type=float, default=0.5, help="heading uncertainty widening the band")
    ap.add_argument("--vehicle-mask", type=Path, default=None, help="PNG, nonzero where the kart's body is")
    ap.add_argument("--range", type=float, default=None, help="label only this far ahead (default: whole track)")
    ap.add_argument("--allow-float", action="store_true")
    ap.add_argument("--preview", type=int, default=0)
    args = ap.parse_args(argv)

    frames_root = args.frames_root or args.frames_dir.parents[1]
    mask = load_label(args.vehicle_mask) > 0 if args.vehicle_mask else None
    stats = label_frames(
        Track.load(args.track),
        CameraModel.load(args.camera),
        frames_root,
        args.frames_dir,
        args.labels_root,
        args.band,
        np.radians(args.yaw_sigma_deg),
        mask,
        args.range,
        not args.allow_float,
        args.preview,
    )
    print(f"{stats['labeled']} labeled, {stats['skipped']} skipped of {stats['frames']} -> {args.labels_root}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
