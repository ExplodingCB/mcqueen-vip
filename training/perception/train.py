"""Train the pavement segmenter on synthetic frames from random tracks.

    PYTHONPATH=training/perception:src/mcq_sim python training/perception/train.py --run v0

Frames are generated on the fly per epoch shard from procedurally drawn tracks
(synth.py), so nothing large is stored and every epoch sees new layouts. The
Purdue track and the synthetic oval are never drawn; evaluate.py scores on them.
"""

from __future__ import annotations

import argparse
import json
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F

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
    args = parser.parse_args()

    torch.manual_seed(args.seed)
    dev = device()
    net = PavementNet(args.width).to(dev)
    params = sum(p.numel() for p in net.parameters())
    out = HERE / "runs" / args.run
    out.mkdir(parents=True, exist_ok=True)
    print(f"device {dev}, {params / 1e6:.2f} M parameters, run {out}", flush=True)
    metrics = (out / "metrics.jsonl").open("w", buffering=1)  # read live by watch.py

    def record(**row):
        metrics.write(json.dumps(row) + "\n")

    record(type="start", args=vars(args), parameters=params, device=str(dev), time=time.time())

    started = time.perf_counter()
    val_rgb, val_mask = make_dataset(args.workers, 25, seed=9000 + args.seed, workers=args.workers)  # unseen tracks
    print(f"validation set {len(val_rgb)} frames from {args.workers} unseen tracks", flush=True)

    steps_per_epoch = args.tracks * args.frames // args.batch
    optimizer = torch.optim.AdamW(net.parameters(), lr=args.lr, weight_decay=1e-4)
    schedule = torch.optim.lr_scheduler.OneCycleLR(optimizer, args.lr, total_steps=args.epochs * steps_per_epoch)
    history = []
    for epoch in range(args.epochs):
        rgb, mask = make_dataset(args.tracks, args.frames, seed=args.seed * 100 + epoch + 1, workers=args.workers)
        order = np.random.default_rng(epoch).permutation(len(rgb))
        net.train()
        flips = np.random.default_rng(1000 + epoch)
        running = 0.0
        for step in range(steps_per_epoch):
            idx = order[step * args.batch : (step + 1) * args.batch]
            x = torch.from_numpy(rgb[idx]).to(dev).permute(0, 3, 1, 2).float()
            y = torch.from_numpy(mask[idx]).to(dev).float()
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
        history.append({"epoch": epoch + 1, "train_loss": running / steps_per_epoch, "val_iou": val_iou})
        record(
            type="epoch",
            epoch=epoch + 1,
            step=(epoch + 1) * steps_per_epoch,
            train_loss=running / steps_per_epoch,
            val_iou=val_iou,
            t=time.perf_counter() - started,
        )
        print(
            f"epoch {epoch + 1}/{args.epochs} loss {running / steps_per_epoch:.4f} val IoU {val_iou:.4f} "
            f"({time.perf_counter() - started:.0f} s)",
            flush=True,
        )
        torch.save({"state": net.state_dict(), "width": args.width, "history": history}, out / "model.pt")
    (out / "history.json").write_text(json.dumps({"args": vars(args), "history": history}, indent=2) + "\n")
    latest = HERE / "runs" / "latest"
    if latest.is_symlink() or latest.exists():
        latest.unlink()
    latest.symlink_to(args.run)
    record(type="done", t=time.perf_counter() - started)
    print(f"saved {out / 'model.pt'}")


if __name__ == "__main__":
    main()
