#!/usr/bin/env python3
"""Sample frames from downloaded videos, crop, resize, drop near-duplicates, and
write an index that carries the source metadata (camera hint, license) forward.

    python extract_frames.py                       # every downloaded video in manifest.jsonl
    python extract_frames.py --videos a.mp4 b.mp4  # ad hoc files (camera hint via --camera)
    python extract_frames.py --fps 2 --skip-start 20 --skip-end 20 --hash-distance 6

Output: frames/<source>/<video_id>/<t_ms>.jpg and frames/index.csv
"""
from __future__ import annotations

import argparse
import csv
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import imagehash
from PIL import Image

HERE = Path(__file__).resolve().parent


def ffmpeg_exe() -> str:
    exe = shutil.which("ffmpeg")
    if exe:
        return exe
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except ImportError:
        sys.exit("ffmpeg not found; install ffmpeg or pip install imageio-ffmpeg")


def probe_duration(video: Path) -> float | None:
    """Duration in seconds via ffmpeg's stderr banner (works without ffprobe)."""
    out = subprocess.run([ffmpeg_exe(), "-i", str(video)], capture_output=True, text=True)
    for line in out.stderr.splitlines():
        line = line.strip()
        if line.startswith("Duration:"):
            hms = line.split()[1].rstrip(",")
            h, m, s = hms.split(":")
            return int(h) * 3600 + int(m) * 60 + float(s)
    return None


def crop_filter(crop: list[float]) -> str | None:
    top, bottom, left, right = crop
    if not any(crop):
        return None
    w = f"iw*{1 - left - right:.4f}"
    h = f"ih*{1 - top - bottom:.4f}"
    return f"crop={w}:{h}:iw*{left:.4f}:ih*{top:.4f}"


def sample(video: Path, out_dir: Path, fps: float, skip_start: float, skip_end: float,
           crop: list[float], long_edge: int) -> list[tuple[Path, int]]:
    """Run ffmpeg once; return (path, t_ms) for each sampled frame in order."""
    duration = probe_duration(video)
    filters = [f"fps={fps}"]
    cf = crop_filter(crop)
    if cf:
        filters.append(cf)
    filters.append(f"scale='if(gt(iw,ih),{long_edge},-2)':'if(gt(iw,ih),-2,{long_edge})'")
    cmd = [ffmpeg_exe(), "-hide_banner", "-loglevel", "error", "-ss", str(skip_start)]
    if duration is not None and duration > skip_start + skip_end:
        cmd += ["-t", str(duration - skip_start - skip_end)]
    cmd += ["-i", str(video), "-vf", ",".join(filters), "-q:v", "3", str(out_dir / "%06d.jpg")]
    subprocess.run(cmd, check=True)
    frames = sorted(out_dir.glob("*.jpg"))
    return [(p, int(round((skip_start + i / fps) * 1000))) for i, p in enumerate(frames)]


def dedupe(frames: list[tuple[Path, int]], distance: int) -> list[tuple[Path, int, str]]:
    """Keep a frame when its dHash differs from every kept hash in the recent window."""
    kept: list[tuple[Path, int, str]] = []
    recent: list[imagehash.ImageHash] = []
    for path, t_ms in frames:
        with Image.open(path) as im:
            h = imagehash.dhash(im, hash_size=8)
        if any(h - r <= distance for r in recent):
            continue
        kept.append((path, t_ms, str(h)))
        recent.append(h)
        if len(recent) > 50:
            recent.pop(0)
    return kept


def jobs_from_manifest(manifest: Path) -> list[dict]:
    jobs = []
    if not manifest.exists():
        return jobs
    for line in manifest.read_text().splitlines():
        if not line.strip():
            continue
        r = json.loads(line)
        if r.get("status") == "downloaded" and r.get("path"):
            jobs.append({"video": HERE / r["path"], "video_id": r["id"], "source": r.get("source", "manual"),
                         "camera": r.get("camera", "unknown"), "license": r.get("license") or "standard",
                         "crop": r.get("crop") or [0, 0, 0, 0], "track": r.get("track") or ""})
    return jobs


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--manifest", type=Path, default=HERE / "manifest.jsonl")
    ap.add_argument("--videos", nargs="*", type=Path, help="explicit video files instead of the manifest")
    ap.add_argument("--source", default="manual", help="source id for --videos")
    ap.add_argument("--camera", default="unknown", help="camera hint for --videos")
    ap.add_argument("--crop", nargs=4, type=float, metavar=("TOP", "BOTTOM", "LEFT", "RIGHT"), default=None)
    ap.add_argument("--out", type=Path, default=HERE / "frames")
    ap.add_argument("--fps", type=float, default=2.0)
    ap.add_argument("--skip-start", type=float, default=20.0)
    ap.add_argument("--skip-end", type=float, default=20.0)
    ap.add_argument("--long-edge", type=int, default=1280)
    ap.add_argument("--hash-distance", type=int, default=6, help="max Hamming distance to count as a duplicate")
    ap.add_argument("--overwrite", action="store_true")
    args = ap.parse_args()

    if args.videos:
        jobs = [{"video": v, "video_id": v.stem, "source": args.source, "camera": args.camera, "license": "own",
                 "crop": args.crop or [0, 0, 0, 0], "track": ""} for v in args.videos]
    else:
        jobs = jobs_from_manifest(args.manifest)
        if args.crop:
            for j in jobs:
                j["crop"] = args.crop
    if not jobs:
        print("nothing to do (no downloaded videos in manifest and no --videos given)")
        return 0

    index_path = args.out / "index.csv"
    args.out.mkdir(parents=True, exist_ok=True)
    write_header = not index_path.exists()
    total_sampled = total_kept = 0
    with index_path.open("a", newline="") as f:
        w = csv.writer(f)
        if write_header:
            w.writerow(["frame", "source", "video_id", "t_ms", "camera", "license", "track", "dhash"])
        for j in jobs:
            dest = args.out / j["source"] / j["video_id"]
            if dest.exists() and not args.overwrite:
                print(f"skip {j['video_id']} (exists)")
                continue
            if not Path(j["video"]).exists():
                print(f"missing {j['video']}")
                continue
            with tempfile.TemporaryDirectory() as tmp:
                frames = sample(Path(j["video"]), Path(tmp), args.fps, args.skip_start, args.skip_end, j["crop"], args.long_edge)
                kept = dedupe(frames, args.hash_distance)
                dest.mkdir(parents=True, exist_ok=True)
                for path, t_ms, h in kept:
                    target = dest / f"{t_ms:09d}.jpg"
                    shutil.move(str(path), target)
                    w.writerow([str(target.relative_to(args.out)), j["source"], j["video_id"], t_ms, j["camera"], j["license"], j["track"], h])
            total_sampled += len(frames)
            total_kept += len(kept)
            print(f"{j['video_id']}: {len(frames)} sampled, {len(kept)} kept")
    print(f"done: {total_sampled} sampled, {total_kept} kept, index at {index_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
