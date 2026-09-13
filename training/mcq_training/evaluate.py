"""Evaluate a checkpoint on a split: IoU per class and per tier, and the edge
error in metres after inverse perspective mapping on the frames whose camera
is known (tier A), checked against the gates in the config.

    python -m mcq_training.evaluate --checkpoint runs/seg_v1_a/best.pt --split datasets/seg_v1/test.csv
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader

from mcq_training import DATA, TRAINING
from mcq_training.camera import CameraModel
from mcq_training.data import SegDataset
from mcq_training.dataset import read_split
from mcq_training.labels import NUM_CLASSES
from mcq_training.metrics import BoundaryAccumulator, confusion_matrix, error_at_range, summarize_confusion
from mcq_training.models import build_model
from mcq_training.train import input_size, pick_device


def load_checkpoint(path: Path, device: torch.device) -> tuple[torch.nn.Module, dict]:
    state = torch.load(path, map_location=device, weights_only=False)
    config = state["config"]
    mcfg = config.get("model", {})
    model = build_model(mcfg.get("name", "lraspp_mobilenet_v3_large"), NUM_CLASSES, False)
    model.load_state_dict(state["model"])
    model.to(device).eval()
    return model, config


def resolve_camera(config: dict, override: Path | None) -> CameraModel | None:
    path = override or (config.get("eval") or {}).get("camera")
    if not path:
        return None
    path = Path(path)
    if not path.is_absolute() and not path.exists():
        path = TRAINING / path
    return CameraModel.load(path) if path.exists() else None


@torch.no_grad()
def evaluate(
    checkpoint: Path,
    split: Path,
    frames_root: Path,
    labels_root: Path,
    device: torch.device,
    camera: Path | None = None,
    boundary_tiers: tuple[str, ...] = ("A",),
    limit: int | None = None,
    batch: int = 8,
) -> dict:
    model, config = load_checkpoint(checkpoint, device)
    size = input_size(config)
    samples = read_split(split)
    if limit:
        samples = samples[:limit]
    if not samples:
        raise SystemExit(f"{split} is empty")
    ds = SegDataset(samples, frames_root, labels_root, size, None)
    loader = DataLoader(ds, batch, shuffle=False, num_workers=0)
    cam = resolve_camera(config, camera)
    cam = cam.scaled(*size) if cam else None
    cms: dict[str, np.ndarray] = {"all": np.zeros((NUM_CLASSES, NUM_CLASSES), dtype=np.int64)}
    boundary = BoundaryAccumulator(cam) if cam else None
    for x, y, idx in loader:
        pred = model(x.to(device)).argmax(1).cpu().numpy()
        gt = y.numpy()
        for p, g, i in zip(pred, gt, idx.tolist(), strict=True):
            cm = confusion_matrix(p, g)
            tier = samples[i].tier
            cms["all"] += cm
            cms.setdefault(tier, np.zeros_like(cm))
            cms[tier] += cm
            if boundary is not None and tier in boundary_tiers:
                boundary.add(p.astype(np.uint8), g.astype(np.uint8))
    result = {
        "checkpoint": str(checkpoint),
        "split": str(split),
        "frames": len(samples),
        "input": {"width": size[0], "height": size[1]},
        "pixels": {k: summarize_confusion(v) for k, v in cms.items()},
    }
    if boundary is not None and boundary.frames:
        result["boundary"] = boundary.summary()
    gates = (config.get("eval") or {}).get("gates") or {}
    if gates:
        basis = "A" if "A" in cms else "all"
        pav = result["pixels"][basis]["iou"].get("pavement")
        checks = {"basis": basis}
        if "pavement_iou" in gates:
            checks["pavement_iou"] = {
                "value": pav,
                "min": gates["pavement_iou"],
                "pass": pav is not None and pav >= gates["pavement_iou"],
            }
        if "boundary_error_15m" in gates:
            e = error_at_range(result["boundary"], 15.0) if "boundary" in result else None
            checks["boundary_error_15m"] = {
                "value": e,
                "max": gates["boundary_error_15m"],
                "pass": e is not None and e <= gates["boundary_error_15m"],
            }
        checks["pass"] = all(v["pass"] for k, v in checks.items() if isinstance(v, dict))
        result["gates"] = checks
    return result


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--checkpoint", type=Path, required=True)
    ap.add_argument("--split", type=Path, required=True, help="a split csv from build_dataset")
    ap.add_argument("--frames-root", type=Path, default=DATA / "frames")
    ap.add_argument("--labels-root", type=Path, default=DATA / "labels")
    ap.add_argument("--camera", type=Path, default=None, help="override the camera yaml named in the run config")
    ap.add_argument(
        "--boundary-tiers", default="A", help="tiers whose frames get the edge metric (their camera must match)"
    )
    ap.add_argument("--out", type=Path, default=None, help="metrics json (default: next to the checkpoint)")
    ap.add_argument("--device", default=None)
    ap.add_argument("--limit", type=int, default=None)
    args = ap.parse_args(argv)
    result = evaluate(
        args.checkpoint,
        args.split,
        args.frames_root,
        args.labels_root,
        pick_device(args.device),
        args.camera,
        tuple(args.boundary_tiers.split(",")),
        args.limit,
    )
    out = args.out or args.checkpoint.parent / f"eval_{args.split.stem}.json"
    with open(out, "w") as f:
        json.dump(result, f, indent=2)
    for tier, s in result["pixels"].items():
        print(
            f"{tier:4s} miou {s['miou']:.3f} " + " ".join(f"{k}={v:.3f}" for k, v in s["iou"].items() if v is not None)
        )
    if "boundary" in result:
        b = result["boundary"]
        print(
            "edge error m: "
            + " ".join(
                f"{n}={'-' if v is None else f'{v:.2f}'}" for n, v in zip(b["bins"], b["mean_abs_error_m"], strict=True)
            )
        )
    if "gates" in result:
        print(
            "gates:",
            "PASS" if result["gates"]["pass"] else "FAIL",
            json.dumps({k: v for k, v in result["gates"].items() if k != "pass"}),
        )
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
