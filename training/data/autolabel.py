#!/usr/bin/env python3
"""Pseudo-label extracted frames with a pavement mask, and keep only the masks
that look trustworthy. This is step 2 of docs/08 section 5, done with a
pretrained scene parser instead of Grounded SAM 2: SegFormer trained on ADE20K
already knows "road" from a low camera, runs on MPS, and needs no prompts.

    python autolabel.py                         # every frame in frames/index.csv
    python autolabel.py --limit 200 --preview   # quick look: also writes overlays

Output: masks/<frame path>.png (255 = pavement) beside a labels.csv that records,
per frame, the labeler, a confidence score, whether the mask passed the checks,
and why not. Nothing here is reviewed; reviewed=0 in every row until a person
looks (docs/08 section 5 step 3).

A mask is kept when the pavement it shows could be a track seen from the kart:
the pavement is one region that reaches the lower part of the frame, covers a
plausible share of the image, and the parser was confident about it.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F
from PIL import Image

HERE = Path(__file__).resolve().parent
MODEL = "nvidia/segformer-b5-finetuned-ade-640-640"
PAVEMENT = ("road", "runway", "sidewalk", "path")  # ADE20K classes that mean drivable hard surface here
LABEL_SIZE = (480, 270)  # the kart camera's resolution; masks are stored at this size


def device():
    if torch.backends.mps.is_available():
        return torch.device("mps")
    return torch.device("cuda" if torch.cuda.is_available() else "cpu")


def center_crop_16x9(im: Image.Image) -> Image.Image:
    w, h = im.size
    if w * 9 > h * 16:
        nw = h * 16 // 9
        return im.crop(((w - nw) // 2, 0, (w - nw) // 2 + nw, h))
    nh = w * 9 // 16
    return im.crop((0, (h - nh) // 2, w, (h - nh) // 2 + nh))


def largest_bottom_component(mask: np.ndarray) -> np.ndarray:
    """The largest connected pavement region in the lower part of the frame.

    Not the bottom-middle pixel: helmet and chassis cameras often show the kart's
    own bodywork there, with the track around it.
    """
    from scipy import ndimage

    labels, n = ndimage.label(mask)
    if n == 0:
        return mask
    h, w = mask.shape
    seed = labels[int(h * 0.6) :]
    ids, counts = np.unique(seed[seed > 0], return_counts=True)
    if len(ids) == 0:
        return np.zeros_like(mask)
    return labels == ids[np.argmax(counts)]


def judge(mask: np.ndarray, confidence: np.ndarray) -> tuple[bool, str]:
    h, w = mask.shape
    share = mask.mean()
    if share < 0.06:
        return False, "too_little_pavement"
    if share > 0.75:
        return False, "too_much_pavement"
    if mask[int(h * 0.6) :].mean() < 0.12:
        return False, "pavement_not_near_kart"
    if mask[: int(h * 0.25)].mean() > 0.2:
        return False, "pavement_in_sky"
    # mean confidence along the pavement boundary band is what decides edge quality
    if float(confidence[mask].mean()) < 0.8:
        return False, "low_confidence"
    return True, ""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", type=Path, default=HERE / "frames")
    ap.add_argument("--out", type=Path, default=HERE / "masks")
    ap.add_argument("--model", default=MODEL)
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--limit", type=int, default=None)
    ap.add_argument("--per-video", type=int, default=None, help="at most this many frames from each video, evenly spread")
    ap.add_argument("--preview", action="store_true", help="also write overlay JPEGs to masks/_preview")
    ap.add_argument("--overwrite", action="store_true")
    args = ap.parse_args()

    from transformers import AutoImageProcessor, SegformerForSemanticSegmentation

    index = args.frames / "index.csv"
    if not index.exists():
        sys.exit(f"{index} not found; run extract_frames.py first")
    rows = list(csv.DictReader(index.open()))
    labels_path = args.out / "labels.csv"
    done = set()
    if labels_path.exists() and not args.overwrite:
        done = {r["frame"] for r in csv.DictReader(labels_path.open())}
    rows = [r for r in rows if r["frame"] not in done]
    if args.per_video:
        # an even spread of frames from each video, so one long upload cannot dominate
        by_video: dict[str, list[dict]] = {}
        for r in rows:
            by_video.setdefault(r["video_id"], []).append(r)
        rows = []
        for group in by_video.values():
            step = max(1, len(group) // args.per_video)
            rows.extend(group[::step][: args.per_video])
    rows = rows[: args.limit]
    if not rows:
        print("nothing to label")
        return 0

    dev = device()
    processor = AutoImageProcessor.from_pretrained(args.model)
    net = SegformerForSemanticSegmentation.from_pretrained(args.model).to(dev).eval()
    names = {v.lower(): int(k) for k, v in net.config.id2label.items()}
    pavement_ids = [names[c] for c in PAVEMENT if c in names]
    print(f"{args.model} on {dev}; pavement classes {pavement_ids}; {len(rows)} frames", flush=True)

    args.out.mkdir(parents=True, exist_ok=True)
    if args.preview:
        (args.out / "_preview").mkdir(exist_ok=True)
    new_file = not labels_path.exists() or args.overwrite
    kept = 0
    with labels_path.open("w" if args.overwrite else "a", newline="") as f:
        w = csv.writer(f)
        if new_file:
            w.writerow(["frame", "mask", "labeler", "pavement_share", "confidence", "kept", "reason", "reviewed"])
        for i in range(0, len(rows), args.batch):
            chunk = rows[i : i + args.batch]
            images = [center_crop_16x9(Image.open(args.frames / r["frame"]).convert("RGB")) for r in chunk]
            inputs = processor(images=images, return_tensors="pt").to(dev)
            with torch.no_grad():
                logits = net(**inputs).logits
            # interpolate class probabilities to the label size, then pool the pavement classes
            probs = F.interpolate(logits, size=LABEL_SIZE[::-1], mode="bilinear", align_corners=False).softmax(1)
            pave = probs[:, pavement_ids].sum(1).cpu().numpy()
            for r, im, p in zip(chunk, images, pave, strict=True):
                raw = p > 0.5
                mask = largest_bottom_component(raw)
                confidence = np.maximum(p, 1 - p)
                ok, reason = judge(mask, confidence) if mask.any() else (False, "no_pavement")
                rel = Path(r["frame"]).with_suffix(".png")
                target = args.out / rel
                target.parent.mkdir(parents=True, exist_ok=True)
                Image.fromarray((mask * 255).astype(np.uint8)).save(target)
                w.writerow(
                    [
                        r["frame"],
                        str(rel),
                        args.model,
                        f"{mask.mean():.3f}",
                        f"{float(confidence[mask].mean()) if mask.any() else 0:.3f}",
                        int(ok),
                        reason,
                        0,
                    ]
                )
                kept += ok
                if args.preview:
                    small = np.asarray(im.resize(LABEL_SIZE), dtype=np.float32)
                    tint = np.array([255, 0, 255] if ok else [255, 140, 0], dtype=np.float32)
                    small[mask] = 0.55 * small[mask] + 0.45 * tint
                    name = str(rel).replace("/", "__").replace(".png", ".jpg")
                    Image.fromarray(small.astype(np.uint8)).save(args.out / "_preview" / name, quality=80)
            print(f"{min(i + args.batch, len(rows))}/{len(rows)} labeled, {kept} kept", flush=True)
    print(f"done: {kept} of {len(rows)} masks kept, labels at {labels_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
