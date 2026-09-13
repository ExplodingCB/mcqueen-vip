"""Train a segmentation model on a built dataset.

    python -m mcq_training.train --config configs/seg_small.yaml --dataset datasets/seg_v1 --out runs/seg_v1_a

Plain PyTorch: cross-entropy with the ignore index, AdamW, polynomial decay,
mixed precision on a GPU, tier-weighted sampling, validation mIoU every epoch,
``best.pt`` by the metric named in the config and ``last.pt`` always. The run
directory records the config, the dataset manifest and the git hash so the
model can be traced back to its data.
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from pathlib import Path

import numpy as np
import torch
import yaml
from torch import nn
from torch.utils.data import DataLoader, WeightedRandomSampler

from mcq_training import DATA, REPO, TRAINING
from mcq_training.data import Augment, AugmentParams, SegDataset
from mcq_training.dataset import git_hash, read_split
from mcq_training.labels import CLASS_NAMES, IGNORE, NUM_CLASSES
from mcq_training.metrics import confusion_matrix, summarize_confusion
from mcq_training.models import build_model, count_parameters


def load_config(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f) or {}


def input_size(config: dict) -> tuple[int, int]:
    inp = config.get("input", {})
    return int(inp.get("width", 640)), int(inp.get("height", 400))


def pick_device(name: str | None) -> torch.device:
    if name:
        return torch.device(name)
    return torch.device("cuda" if torch.cuda.is_available() else "cpu")


@torch.no_grad()
def evaluate_loader(model: nn.Module, loader: DataLoader, device: torch.device) -> np.ndarray:
    model.eval()
    cm = np.zeros((NUM_CLASSES, NUM_CLASSES), dtype=np.int64)
    for x, y, _ in loader:
        logits = model(x.to(device, non_blocking=True))
        pred = logits.argmax(1).cpu().numpy()
        cm += confusion_matrix(pred, y.numpy())
    return cm


def selection_value(summary: dict, key: str) -> float:
    if key == "miou":
        return summary["miou"]
    if key.endswith("_iou"):
        v = summary["iou"].get(key[:-4])
        return -1.0 if v is None else v
    raise KeyError(f"unknown selection metric {key!r}")


def train(
    config: dict,
    dataset_dir: Path,
    out_dir: Path,
    frames_root: Path,
    labels_root: Path,
    device: torch.device,
    epochs: int | None = None,
    limit: int | None = None,
    seed: int = 0,
    init: Path | None = None,
) -> dict:
    torch.manual_seed(seed)
    size = input_size(config)
    epochs = epochs or int(config.get("epochs", 40))
    batch = int(config.get("batch_size", 16))
    workers = int(config.get("workers", 4))
    amp = bool(config.get("amp", True)) and device.type == "cuda"

    train_samples = read_split(dataset_dir / "train.csv")
    val_samples = read_split(dataset_dir / "val.csv")
    if limit:
        train_samples, val_samples = train_samples[:limit], val_samples[: max(limit // 4, 1)]
    if not train_samples or not val_samples:
        raise SystemExit(f"{dataset_dir}: need non-empty train.csv and val.csv")
    augment = Augment(AugmentParams.from_dict(config.get("augment")), seed=seed)
    ds_train = SegDataset(train_samples, frames_root, labels_root, size, augment)
    ds_val = SegDataset(val_samples, frames_root, labels_root, size, None)
    sampler = None
    if (config.get("sampling") or {}).get("tier_weighted", True):
        sampler = WeightedRandomSampler(ds_train.weights, num_samples=len(ds_train), replacement=True)
    dl_train = DataLoader(
        ds_train, batch, shuffle=sampler is None, sampler=sampler, num_workers=workers, drop_last=len(ds_train) > batch
    )
    dl_val = DataLoader(ds_val, batch, shuffle=False, num_workers=workers)

    mcfg = config.get("model", {})
    model = build_model(
        mcfg.get("name", "lraspp_mobilenet_v3_large"), NUM_CLASSES, mcfg.get("pretrained_backbone", False)
    )
    if init is not None:
        state = torch.load(init, map_location="cpu", weights_only=False)
        missing, unexpected = model.load_state_dict(state["model"], strict=False)
        print(
            f"initialized from {init} (epoch {state.get('epoch')}); "
            f"missing {len(missing)}, unexpected {len(unexpected)}"
        )
    model.to(device)
    weights = (config.get("loss") or {}).get("class_weights")
    criterion = nn.CrossEntropyLoss(
        weight=torch.tensor(weights, dtype=torch.float32, device=device) if weights else None, ignore_index=IGNORE
    )
    lr = float(config.get("lr", 1e-3))
    opt = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=float(config.get("weight_decay", 1e-4)))
    total_iters = max(epochs * len(dl_train), 1)
    sched = torch.optim.lr_scheduler.LambdaLR(opt, lambda it: (1.0 - min(it, total_iters - 1) / total_iters) ** 0.9)
    scaler = torch.amp.GradScaler(enabled=amp)
    select = config.get("select", "pavement_iou")

    out_dir.mkdir(parents=True, exist_ok=True)
    with open(out_dir / "config.yaml", "w") as f:
        yaml.safe_dump(config, f, sort_keys=False)
    manifest_path = dataset_dir / "manifest.json"
    if manifest_path.exists():
        shutil.copy(manifest_path, out_dir / "dataset_manifest.json")
    history, best_value = [], -np.inf
    record = {
        "git": git_hash(REPO),
        "dataset": str(dataset_dir),
        "model": mcfg,
        "parameters": count_parameters(model),
        "input": {"width": size[0], "height": size[1]},
        "classes": list(CLASS_NAMES),
        "device": str(device),
        "init": str(init) if init else None,
        "train_frames": len(ds_train),
        "val_frames": len(ds_val),
        "history": history,
    }
    print(
        f"{record['model']} {record['parameters'] / 1e6:.2f} M params on {device}, "
        f"{len(ds_train)} train / {len(ds_val)} val"
    )
    for epoch in range(1, epochs + 1):
        model.train()
        t0, losses = time.time(), []
        for x, y, _ in dl_train:
            x, y = x.to(device, non_blocking=True), y.to(device, non_blocking=True)
            with torch.autocast(device_type=device.type, enabled=amp):
                loss = criterion(model(x), y)
            opt.zero_grad(set_to_none=True)
            scaler.scale(loss).backward()
            scaler.step(opt)
            scaler.update()
            sched.step()
            losses.append(loss.item())
        cm = evaluate_loader(model, dl_val, device)
        summary = summarize_confusion(cm)
        value = selection_value(summary, select)
        entry = {
            "epoch": epoch,
            "train_loss": float(np.mean(losses)) if losses else None,
            "val": summary,
            "lr": float(opt.param_groups[0]["lr"]),
            "seconds": time.time() - t0,
        }
        history.append(entry)
        state = {"model": model.state_dict(), "config": config, "epoch": epoch, "val": summary, "classes": CLASS_NAMES}
        torch.save(state, out_dir / "last.pt")
        if value > best_value:
            best_value = value
            record["best"] = {"epoch": epoch, select: value, "val": summary}
            torch.save(state, out_dir / "best.pt")
        with open(out_dir / "metrics.json", "w") as f:
            json.dump(record, f, indent=2)
        iou = " ".join(f"{k}={v:.3f}" for k, v in summary["iou"].items() if v is not None)
        print(
            f"epoch {epoch:3d} loss {entry['train_loss']:.4f} val miou {summary['miou']:.3f} [{iou}] "
            f"{entry['seconds']:.0f}s"
        )
    return record


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", type=Path, default=TRAINING / "configs" / "seg_small.yaml")
    ap.add_argument("--dataset", type=Path, required=True, help="directory with train.csv and val.csv")
    ap.add_argument("--out", type=Path, required=True, help="run directory")
    ap.add_argument("--frames-root", type=Path, default=DATA / "frames")
    ap.add_argument("--labels-root", type=Path, default=DATA / "labels")
    ap.add_argument("--device", default=None)
    ap.add_argument("--epochs", type=int, default=None)
    ap.add_argument("--limit", type=int, default=None, help="use only the first N training frames (smoke runs)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument(
        "--init", type=Path, default=None, help="checkpoint to start from (pretrain on tier C, fine-tune on A and B)"
    )
    args = ap.parse_args(argv)
    record = train(
        load_config(args.config),
        args.dataset,
        args.out,
        args.frames_root,
        args.labels_root,
        pick_device(args.device),
        args.epochs,
        args.limit,
        args.seed,
        args.init,
    )
    best = record.get("best", {})
    print(f"best epoch {best.get('epoch')}: {json.dumps(best.get('val', {}).get('iou'))} -> {args.out / 'best.pt'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
