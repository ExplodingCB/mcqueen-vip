"""How does a driving model's pavement mask do on real kart footage?

    PYTHONPATH=training/perception python training/perception/real_eval.py \\
        --weights runs/drive-v3/model.pt

Runs the model over the real onboard frames in ``training/data/frames`` that the
auto-labeler (``training/data/autolabel.py``) produced a usable pavement mask
for, and scores the model's mask against it, per video and overall. The labels
are pseudo-labels from SegFormer, not human-reviewed masks: they are good enough
to show where a model trained only on the simulator falls apart, not good
enough to certify one. Frames from other tracks and helmet cameras say nothing
about the racing line, which only means something on the kart's own mount, so
that output is not scored here.

A fixed guess is scored alongside as a floor: "the lower part of the picture is
road". A model that does not beat it has learned nothing about real pavement.

Writes a JSON report and, for the dashboard, contact sheets of typical and of
worst frames (magenta: both say pavement, red: model says pavement and the
label does not, blue: the label does and the model does not).
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
DATA = ROOT / "training" / "data"
sys.path.insert(0, str(DATA))

from autolabel import center_crop_16x9  # noqa: E402

from model import DriveNet  # noqa: E402

SIZE = (240, 135)  # what the model reads
PASS_IOU = 0.8  # for the gallery's green edge only: a mean this high is a mask worth looking at


def load_frames(limit_per_video: int | None, frames_root: Path, masks_root: Path):
    labels = [r for r in csv.DictReader((masks_root / "labels.csv").open()) if r["kept"] == "1"]
    by_video: dict[str, list[dict]] = {}
    for r in labels:
        by_video.setdefault(Path(r["frame"]).parent.name, []).append(r)
    rows = []
    for video, group in by_video.items():
        if limit_per_video:
            group = group[:: max(1, len(group) // limit_per_video)][:limit_per_video]
        rows += [(video, r) for r in group]
    images, truth = [], []
    for _, r in rows:
        img = center_crop_16x9(Image.open(frames_root / r["frame"]).convert("RGB")).resize(SIZE, Image.BILINEAR)
        images.append(np.asarray(img))
        truth.append(np.asarray(Image.open(masks_root / r["mask"]).resize(SIZE, Image.NEAREST)) > 127)
    return [v for v, _ in rows], np.stack(images), np.stack(truth), [r["frame"] for _, r in rows]


def iou(pred: np.ndarray, truth: np.ndarray) -> np.ndarray:
    inter, union = (pred & truth).sum((1, 2)), (pred | truth).sum((1, 2))
    return np.where(union > 0, inter / np.maximum(union, 1), 1.0)


def predict(weights: Path, images: np.ndarray, device) -> np.ndarray:
    checkpoint = torch.load(weights, map_location=device, weights_only=True)
    net = DriveNet(fine=bool(checkpoint.get("fine", False))).to(device).eval()
    net.load_state_dict(checkpoint["state"])
    out = []
    with torch.no_grad():
        for i in range(0, len(images), 64):
            x = torch.from_numpy(images[i : i + 64]).to(device).permute(0, 3, 1, 2).float()
            out.append((net.forward_all(x)[0] > 0).cpu().numpy())
    return np.concatenate(out)


def sheet(images, truth, pred, scores, order, title, columns=4, rows=3, tile=(320, 180)) -> Image.Image:
    canvas = Image.new("RGB", (columns * tile[0], rows * tile[1]), (20, 20, 22))
    for slot, i in enumerate(order[: columns * rows]):
        over = images[i].astype(np.float32)
        both, only_pred, only_truth = pred[i] & truth[i], pred[i] & ~truth[i], ~pred[i] & truth[i]
        for mask, color in ((both, (255, 43, 214)), (only_pred, (255, 40, 40)), (only_truth, (40, 100, 255))):
            over[mask] = 0.55 * over[mask] + 0.45 * np.array(color, dtype=np.float32)
        tile_img = Image.fromarray(over.astype(np.uint8)).resize(tile, Image.BILINEAR)
        ImageDraw.Draw(tile_img).text((6, 4), f"IoU {scores[i]:.2f}", fill=(255, 255, 255))
        canvas.paste(tile_img, ((slot % columns) * tile[0], (slot // columns) * tile[1]))
    ImageDraw.Draw(canvas).text((6, canvas.height - 14), title, fill=(255, 255, 255))
    return canvas


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--weights", required=True)
    parser.add_argument("--per-video", type=int, default=150)
    parser.add_argument("--frames", type=Path, default=DATA / "frames", help="folder holding the frames and index.csv")
    parser.add_argument("--masks", type=Path, default=DATA / "masks", help="folder holding labels.csv and the masks")
    parser.add_argument("--tag", default="real footage", help="name for the report and the dashboard sheets")
    parser.add_argument("--plots", type=Path, default=HERE / "runs" / "plots", help="where dashboard sheets go")
    args = parser.parse_args()

    weights = Path(args.weights) if Path(args.weights).is_absolute() else HERE / args.weights
    device = torch.device(
        "cuda" if torch.cuda.is_available() else "mps" if torch.backends.mps.is_available() else "cpu"
    )
    videos, images, truth, frames = load_frames(args.per_video, args.frames, args.masks)
    if not len(images):
        sys.exit("no labeled real frames: run training/data/extract_frames.py and autolabel.py first")
    pred = predict(weights, images, device)
    scores = iou(pred, truth)
    blind = np.zeros_like(truth)
    blind[:, int(SIZE[1] * 0.55) :] = True  # "the lower part of the picture is road"
    blind_scores = iou(blind, truth)

    per_video = {}
    for v in dict.fromkeys(videos):
        idx = [i for i, name in enumerate(videos) if name == v]
        per_video[v] = {
            "frames": len(idx),
            "iou": float(scores[idx].mean()),
            "blind_iou": float(blind_scores[idx].mean()),
        }
    report = {
        "weights": str(weights),
        "frames": len(scores),
        "videos": len(per_video),
        "iou_mean": float(scores.mean()),
        "iou_median": float(np.median(scores)),
        "iou_p10": float(np.percentile(scores, 10)),
        "share_above_0.8": float((scores >= 0.8).mean()),
        "blind_baseline_iou": float(blind_scores.mean()),
        "per_video": per_video,
        "note": "scored against SegFormer pseudo-labels, not reviewed masks",
    }
    name = weights.parent.name
    out = ROOT / "logs" / "real-eval"
    out.mkdir(parents=True, exist_ok=True)
    (out / f"{name}-{time.strftime('%Y%m%d-%H%M%S')}.json").write_text(json.dumps(report, indent=2) + "\n")

    args.plots.mkdir(parents=True, exist_ok=True)
    stamp = int(time.time())
    median = np.argsort(np.abs(scores - np.median(scores)))
    for kind, order, detail in (
        ("typical", median, "frames near the median"),
        ("worst", np.argsort(scores), "the lowest-scoring frames"),
    ):
        png = args.plots / f"{args.tag.replace(' ', '-')}-{name}-{kind}-{stamp}.png"
        sheet(images, truth, pred, scores, order, f"{name}: {detail}").save(png)
        png.with_suffix(".json").write_text(
            json.dumps(
                {
                    "title": f"{args.tag} · {name} · {kind}",
                    "passed": bool(report["iou_mean"] >= PASS_IOU),
                    "detail": f"{detail}; mean mask IoU {report['iou_mean']:.2f} over {report['frames']} frames from "
                    f"{report['videos']} videos (blind baseline {report['blind_baseline_iou']:.2f})",
                    "made": time.time(),
                }
            )
            + "\n"
        )
    print(
        f"{name}: mean mask IoU {report['iou_mean']:.3f} (median {report['iou_median']:.3f}, "
        f"p10 {report['iou_p10']:.3f}) over {report['frames']} real frames from {report['videos']} videos; "
        f"blind baseline {report['blind_baseline_iou']:.3f}"
    )
    for v, r in sorted(per_video.items(), key=lambda kv: kv[1]["iou"]):
        print(f"  {v:>14} {r['frames']:4d} frames  IoU {r['iou']:.3f}  (blind {r['blind_iou']:.3f})")


if __name__ == "__main__":
    main()
