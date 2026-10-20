"""Train the pavement segmenter on synthetic frames, optionally mixed with
auto-labeled frames from real onboard video.

    PYTHONPATH=training/perception:src/mcq_sim python training/perception/train.py --run v0
    # fine-tune a synthetic model on video frames (training/data autolabel.py output)
    ... train.py --run v2 --init runs/v1/model.pt --real training/data/masks/labels.csv --epochs 6

Synthetic frames are generated on the fly per epoch shard from procedurally drawn
tracks (synth.py), so nothing large is stored and every epoch sees new layouts.
The Purdue track and the synthetic oval are never drawn; evaluate.py scores on
them. Real frames are split by video, so validation videos are never trained on;
their IoU is against pseudo-labels, not reviewed masks.
"""

from __future__ import annotations

import argparse
import csv
import json
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F
from PIL import Image

import synth
from model import PavementNet

HERE = Path(__file__).resolve().parent


def device():
    if torch.backends.mps.is_available():
        return torch.device("mps")
    return torch.device("cuda" if torch.cuda.is_available() else "cpu")


def _shard(args):
    n_tracks, frames, seed = args
    rgb, mask, _ = synth.generate(n_tracks, frames, seed)
    return rgb, mask


def make_dataset(n_tracks: int, frames: int, seed: int, workers: int):
    per = max(1, n_tracks // workers)
    jobs = [(per, frames, seed * 1000 + i) for i in range(workers)]
    with ProcessPoolExecutor(workers) as pool:
        shards = list(pool.map(_shard, jobs))
    return np.concatenate([s[0] for s in shards]), np.concatenate([s[1] for s in shards])


def load_real(labels_csv: Path, val_fraction: float, seed: int, per_video: int = 300, hold: tuple = ()):
    """Kept auto-labeled frames at the training resolution, split by video.

    At most ``per_video`` frames come from any one video, spread evenly through
    it, so one long upload cannot outweigh ten short ones from other tracks.

    ``hold`` names videos (their ids) that go to validation whatever the random split
    says, so a chosen test video is never trained on. The frames are expected in the
    folder named like the masks one with "masks" replaced by "frames" (``masks`` and
    ``frames``, or ``test_masks`` and ``test_frames``).

    Returns ((rgb, mask) train, (rgb, mask) validation). Frames are center-cropped
    to 16:9 the way autolabel.py cropped them before labeling.
    """
    from autolabel import center_crop_16x9  # training/data, on the path below

    root = labels_csv.parent
    frames_root = root.parent / root.name.replace("masks", "frames")
    by_video: dict[str, list[dict]] = {}
    for r in csv.DictReader(labels_csv.open()):
        if r["kept"] == "1":
            by_video.setdefault(str(Path(r["frame"]).parent), []).append(r)
    rows = []
    for group in by_video.values():
        keep = np.linspace(0, len(group) - 1, min(per_video, len(group))).round().astype(int)
        rows.extend(group[i] for i in np.unique(keep))
    videos = sorted(by_video)
    rng = np.random.default_rng(seed)
    n_val = max(1, int(round(len(videos) * val_fraction))) if len(videos) > 1 else 0
    val_videos = set(rng.choice(videos, n_val, replace=False)) if n_val else set()
    val_videos |= {v for v in videos if Path(v).name in set(hold)}
    size = (synth.HALF_WIDTH, synth.HALF_HEIGHT)
    split = {True: ([], []), False: ([], [])}
    for r in rows:
        rgb = np.asarray(
            center_crop_16x9(Image.open(frames_root / r["frame"]).convert("RGB")).resize(size, Image.BILINEAR)
        )
        mask = np.asarray(Image.open(root / r["mask"]).resize(size, Image.NEAREST)) > 127
        bucket = split[str(Path(r["frame"]).parent) in val_videos]
        bucket[0].append(rgb)
        bucket[1].append(mask)

    def stack(b):
        if not b[0]:
            return np.zeros((0, size[1], size[0], 3), np.uint8), np.zeros((0, size[1], size[0]), bool)
        return np.stack(b[0]), np.stack(b[1])

    return stack(split[False]), stack(split[True]), sorted(str(v) for v in val_videos)


def augment_real(x, y, gen):
    """Scale-and-crop, brightness, contrast and color jitter on a real batch (N,3,H,W)."""
    n, _, h, w = x.shape
    scale = float(gen.uniform(1.0, 1.35))
    if scale > 1.01:
        hh, ww = int(h * scale), int(w * scale)
        x = F.interpolate(x, size=(hh, ww), mode="bilinear", align_corners=False)
        y = F.interpolate(y[:, None], size=(hh, ww), mode="nearest")[:, 0]
        top, left = int(gen.integers(0, hh - h + 1)), int(gen.integers(0, ww - w + 1))
        x, y = x[..., top : top + h, left : left + w], y[..., top : top + h, left : left + w]
    gain = torch.from_numpy(gen.uniform(0.85, 1.15, (n, 3, 1, 1))).to(x)
    bright = torch.from_numpy(gen.uniform(0.7, 1.3, (n, 1, 1, 1))).to(x)
    mean = x.mean((1, 2, 3), keepdim=True)
    contrast = torch.from_numpy(gen.uniform(0.75, 1.25, (n, 1, 1, 1))).to(x)
    x = ((x - mean) * contrast + mean) * bright * gain
    return x.clamp(0, 255), y


def loss_fn(logits, target):
    bce = F.binary_cross_entropy_with_logits(logits, target)
    p = torch.sigmoid(logits)
    dice = 1 - (2 * (p * target).sum((1, 2)) + 1) / (p.sum((1, 2)) + target.sum((1, 2)) + 1)
    return bce + dice.mean()


@torch.no_grad()
def score(net, rgb, mask, dev, batch=64):
    net.eval()
    ious = []
    for i in range(0, len(rgb), batch):
        x = torch.from_numpy(rgb[i : i + batch]).to(dev).permute(0, 3, 1, 2).float()
        pred = (net(x) > 0).cpu().numpy()
        truth = mask[i : i + batch]
        inter = (pred & truth).sum((1, 2))
        union = (pred | truth).sum((1, 2))
        ious.extend(np.where(union > 0, inter / np.maximum(union, 1), 1.0))
    return float(np.mean(ious))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", default="v0")
    parser.add_argument("--epochs", type=int, default=12)
    parser.add_argument("--tracks", type=int, default=160, help="random tracks per epoch shard set")
    parser.add_argument("--frames", type=int, default=60, help="frames per track")
    parser.add_argument("--batch", type=int, default=32)
    parser.add_argument("--lr", type=float, default=2e-3)
    parser.add_argument("--width", type=int, default=24)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--init", type=Path, help="start from these weights (fine-tuning)")
    parser.add_argument("--real", type=Path, help="labels.csv from training/data/autolabel.py")
    parser.add_argument("--real-share", type=float, default=0.5, help="fraction of each batch that is real frames")
    parser.add_argument("--real-val", type=float, default=0.2, help="fraction of real videos held out")
    parser.add_argument("--real-per-video", type=int, default=300, help="cap on frames used from one video")
    args = parser.parse_args()

    torch.manual_seed(args.seed)
    dev = device()
    net = PavementNet(args.width).to(dev)
    if args.init:
        checkpoint = torch.load(args.init, map_location="cpu", weights_only=True)
        if checkpoint["width"] != args.width:
            raise SystemExit(f"--init has width {checkpoint['width']}, --width is {args.width}")
        net.load_state_dict(checkpoint["state"])
    params = sum(p.numel() for p in net.parameters())
    out = HERE / "runs" / args.run
    out.mkdir(parents=True, exist_ok=True)
    print(f"device {dev}, {params / 1e6:.2f} M parameters, run {out}", flush=True)
    metrics = (out / "metrics.jsonl").open("w", buffering=1)  # read live by watch.py

    def record(**row):
        metrics.write(json.dumps(row) + "\n")

    record(
        type="start",
        args={k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
        parameters=params,
        device=str(dev),
        time=time.time(),
    )

    started = time.perf_counter()
    val_rgb, val_mask = make_dataset(args.workers, 25, seed=9000 + args.seed, workers=args.workers)  # unseen tracks
    print(f"validation set {len(val_rgb)} frames from {args.workers} unseen tracks", flush=True)
    real_rgb = real_val = None
    real_per_batch = 0
    if args.real:
        import sys

        sys.path.insert(0, str(HERE.parent / "data"))
        (real_rgb, real_mask), real_val, val_videos = load_real(
            args.real, args.real_val, args.seed, args.real_per_video
        )
        real_per_batch = int(round(args.batch * args.real_share)) if len(real_rgb) else 0
        print(
            f"real frames: {len(real_rgb)} train, {len(real_val[0])} validation from held-out videos {val_videos}",
            flush=True,
        )
        record(type="real", train=len(real_rgb), val=len(real_val[0]), val_videos=val_videos)
    synth_per_batch = args.batch - real_per_batch

    steps_per_epoch = args.tracks * args.frames // synth_per_batch
    optimizer = torch.optim.AdamW(net.parameters(), lr=args.lr, weight_decay=1e-4)
    schedule = torch.optim.lr_scheduler.OneCycleLR(optimizer, args.lr, total_steps=args.epochs * steps_per_epoch)
    history = []
    for epoch in range(args.epochs):
        rgb, mask = make_dataset(args.tracks, args.frames, seed=args.seed * 100 + epoch + 1, workers=args.workers)
        order = np.random.default_rng(epoch).permutation(len(rgb))
        net.train()
        flips = np.random.default_rng(1000 + epoch)
        real_order = np.random.default_rng(2000 + epoch)
        running = 0.0
        for step in range(steps_per_epoch):
            idx = order[step * synth_per_batch : (step + 1) * synth_per_batch]
            x = torch.from_numpy(rgb[idx]).to(dev).permute(0, 3, 1, 2).float()
            y = torch.from_numpy(mask[idx]).to(dev).float()
            if real_per_batch:
                ridx = real_order.choice(len(real_rgb), real_per_batch, replace=len(real_rgb) < real_per_batch)
                rx = torch.from_numpy(real_rgb[ridx]).to(dev).permute(0, 3, 1, 2).float()
                ry = torch.from_numpy(real_mask[ridx]).to(dev).float()
                rx, ry = augment_real(rx, ry, real_order)
                x, y = torch.cat([x, rx]), torch.cat([y, ry])
            if flips.random() < 0.5:  # mirror: a mirrored track is a valid track
                x, y = x.flip(-1), y.flip(-1)
            loss = loss_fn(net(x), y)
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            optimizer.step()
            schedule.step()
            running += float(loss.detach())
            if step % 10 == 0:
                record(
                    type="step",
                    epoch=epoch + 1,
                    step=epoch * steps_per_epoch + step,
                    loss=float(loss.detach()),
                    lr=schedule.get_last_lr()[0],
                    t=time.perf_counter() - started,
                )
        val_iou = score(net, val_rgb, val_mask, dev)
        real_iou = score(net, *real_val, dev) if real_val is not None and len(real_val[0]) else None
        history.append(
            {"epoch": epoch + 1, "train_loss": running / steps_per_epoch, "val_iou": val_iou, "real_val_iou": real_iou}
        )
        record(
            type="epoch",
            epoch=epoch + 1,
            step=(epoch + 1) * steps_per_epoch,
            train_loss=running / steps_per_epoch,
            val_iou=val_iou,
            real_val_iou=real_iou,
            t=time.perf_counter() - started,
        )
        real_text = f" real val IoU {real_iou:.4f}" if real_iou is not None else ""
        print(
            f"epoch {epoch + 1}/{args.epochs} loss {running / steps_per_epoch:.4f} val IoU {val_iou:.4f}{real_text} "
            f"({time.perf_counter() - started:.0f} s)",
            flush=True,
        )
        torch.save(
            {"state": net.state_dict(), "width": args.width, "history": history, "init": str(args.init or "")},
            out / "model.pt",
        )
    (out / "history.json").write_text(
        json.dumps({"args": vars(args), "history": history}, indent=2, default=str) + "\n"
    )
    latest = HERE / "runs" / "latest"
    if latest.is_symlink() or latest.exists():
        latest.unlink()
    latest.symlink_to(args.run)
    record(type="done", t=time.perf_counter() - started)
    print(f"saved {out / 'model.pt'}")


if __name__ == "__main__":
    main()
