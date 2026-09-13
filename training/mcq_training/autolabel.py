"""Tier B and C labels: text-prompted segmentation of extracted frames.

    python -m mcq_training.autolabel --sources q_kart_onboard,q_kart_chassis_cam --limit 500
    python -m mcq_training.autolabel --backend dummy --limit 5      # pipeline check, no model

The backend is SAM 3 (Meta, November 2025; ``facebook/sam3`` on Hugging Face,
gated, SAM License) through its official ``sam3`` package: one text prompt
returns every instance of the concept with a score. Prompts per class and the
score threshold live in ``configs/autolabel.yaml``. Classes are painted in the
listed order so a kart mask overrides the pavement under it. A frame with no
pavement above threshold gets ``status=auto_empty`` and stays out of the
dataset until someone looks at it.

Every label written here is ``status=auto``: the review step in CVAT or Label
Studio promotes it to ``reviewed`` or ``rejected`` in ``labels/index.csv``.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

from mcq_training import DATA, TRAINING
from mcq_training.dataset import LABEL_COLUMNS, append_rows, read_rows
from mcq_training.labels import BACKGROUND, CLASS_NAMES, save_label

DEFAULT_PROMPTS = {
    "prompts": {
        "pavement": ["asphalt race track surface", "paved track"],
        "kart": ["go-kart", "racing kart"],
    },
    "min_score": 0.3,
    "paint_order": ["pavement", "kart"],
}


def to_bool_masks(masks) -> np.ndarray:
    """(n, H, W) booleans from whatever the backend returns: tensors or arrays,
    with or without a channel axis, logits or probabilities."""
    if hasattr(masks, "detach"):
        masks = masks.detach().cpu().numpy()
    m = np.asarray(masks)
    if m.ndim == 2:
        m = m[None]
    if m.ndim == 4:
        m = m[:, 0]
    if m.dtype == bool:
        return m
    return m > (0.5 if m.min() >= 0.0 and m.max() <= 1.0 else 0.0)


class DummyBackend:
    """Deterministic stand-in: the lower 55 % of the frame is pavement and a
    box in the middle is a kart. Exercises the writing and indexing path."""

    def segment(self, image: Image.Image, prompt: str) -> list[tuple[np.ndarray, float]]:
        w, h = image.size
        m = np.zeros((h, w), dtype=bool)
        if "kart" in prompt:
            m[int(0.5 * h) : int(0.7 * h), int(0.4 * w) : int(0.6 * w)] = True
            return [(m, 0.9)]
        m[int(0.45 * h) :] = True
        return [(m, 0.95)]


class Sam3Backend:
    """SAM 3 image model through the ``sam3`` package (pip install from
    github.com/facebookresearch/sam3; weights from facebook/sam3 after
    ``hf auth login``)."""

    def __init__(self, device: str = "cuda"):
        from sam3.model.sam3_image_processor import Sam3Processor
        from sam3.model_builder import build_sam3_image_model

        self.model = build_sam3_image_model()
        if device:
            self.model = self.model.to(device)
        self.processor = Sam3Processor(self.model)
        self._state = None
        self._image_id = None

    def segment(self, image: Image.Image, prompt: str) -> list[tuple[np.ndarray, float]]:
        if self._image_id != id(image):
            self._state = self.processor.set_image(image)
            self._image_id = id(image)
        out = self.processor.set_text_prompt(state=self._state, prompt=prompt)
        masks = to_bool_masks(out["masks"])
        scores = out["scores"]
        if hasattr(scores, "detach"):
            scores = scores.detach().cpu().numpy()
        return [(m, float(s)) for m, s in zip(masks, np.asarray(scores).ravel(), strict=False)]


BACKENDS = {"dummy": DummyBackend, "sam3": Sam3Backend}


def label_image(backend, image: Image.Image, config: dict) -> tuple[np.ndarray, dict]:
    """Class-index label for one frame plus per-class best scores."""
    w, h = image.size
    label = np.full((h, w), BACKGROUND, dtype=np.uint8)
    best: dict[str, float] = {}
    min_score = float(config.get("min_score", 0.3))
    for cls in config.get("paint_order", list(config["prompts"])):
        cid = CLASS_NAMES.index(cls)
        for prompt in config["prompts"].get(cls, []):
            for mask, score in backend.segment(image, prompt):
                if score < min_score or mask.shape != (h, w):
                    continue
                label[mask] = cid
                best[cls] = max(best.get(cls, 0.0), float(score))
    return label, best


def run(
    backend,
    frames_root: Path,
    labels_root: Path,
    rows: list[dict],
    config: dict,
    overwrite: bool = False,
) -> dict:
    stats = {"frames": len(rows), "labeled": 0, "empty": 0, "skipped": 0}
    out_rows = []
    date = time.strftime("%Y-%m-%d")
    method = config.get("method", type(backend).__name__.replace("Backend", "").lower())
    for r in rows:
        rel = Path(r["frame"]).with_suffix(".png")
        out = labels_root / rel
        if out.exists() and not overwrite:
            stats["skipped"] += 1
            continue
        with Image.open(frames_root / r["frame"]) as im:
            image = im.convert("RGB")
        label, best = label_image(backend, image, config)
        out.parent.mkdir(parents=True, exist_ok=True)
        save_label(out, label)
        empty = "pavement" not in best
        stats["empty" if empty else "labeled"] += 1
        out_rows.append(
            {
                "frame": r["frame"],
                "label": str(rel),
                "source": r.get("source", ""),
                "video_id": r.get("video_id", ""),
                "method": method,
                "status": "auto_empty" if empty else "auto",
                "reviewer": "",
                "date": date,
                "note": " ".join(f"{k}={v:.2f}" for k, v in sorted(best.items())),
            }
        )
    append_rows(labels_root / "index.csv", LABEL_COLUMNS, out_rows)
    return stats


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames-index", type=Path, default=DATA / "frames" / "index.csv")
    ap.add_argument("--frames-root", type=Path, default=None, help="default: directory of the index")
    ap.add_argument("--labels-root", type=Path, default=DATA / "labels")
    ap.add_argument("--prompts", type=Path, default=TRAINING / "configs" / "autolabel.yaml")
    ap.add_argument("--backend", choices=sorted(BACKENDS), default="sam3")
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--sources", default=None, help="comma-separated source ids (default: all)")
    ap.add_argument("--limit", type=int, default=None)
    ap.add_argument("--overwrite", action="store_true")
    args = ap.parse_args(argv)

    config = dict(DEFAULT_PROMPTS)
    if args.prompts.exists():
        with open(args.prompts) as f:
            config.update(yaml.safe_load(f) or {})
    rows = read_rows(args.frames_index)
    if args.sources:
        wanted = set(args.sources.split(","))
        rows = [r for r in rows if r.get("source") in wanted]
    if args.limit:
        rows = rows[: args.limit]
    backend = BACKENDS[args.backend](args.device) if args.backend == "sam3" else BACKENDS[args.backend]()
    stats = run(backend, args.frames_root or args.frames_index.parent, args.labels_root, rows, config, args.overwrite)
    print(
        f"{stats['labeled']} labeled, {stats['empty']} without pavement, {stats['skipped']} existing "
        f"of {stats['frames']} -> {args.labels_root}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
