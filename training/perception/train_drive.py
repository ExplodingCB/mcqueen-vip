"""Train the driving model: camera in, racing line and target speed out, with
the pavement mask kept as a second output for the stack's safety check.

    PYTHONPATH=training/perception:src/mcq_sim python training/perception/train_drive.py --run drive-v1 \\
        --init training/perception/runs/v1/model.pt

Frames and labels come from drive_data.py: the Purdue track with its orthophoto
draped over the ground, poses around the expert's racing line, the camera mount
and the image appearance jittered. Validation frames use seeds never trained
on; they are the same track, so they measure how well the model reads the line
from new viewpoints and looks, not whether it generalizes to another track.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F

from model import DriveNet

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def device():
    if torch.backends.mps.is_available():
        return torch.device("mps")
    return torch.device("cuda" if torch.cuda.is_available() else "cpu")


def _shard(job):
    track_dir, n, seed, corner_weight, expert_json = job
    # workers inherit the trainer's priority; run it under `nice` to keep the machine responsive
    import drive_data

    from mcq_sim.track import Track

    track = Track.load(track_dir)
    if expert_json:  # a searched expert line (laptime_opt.py) in place of the minimum-curvature one
        import laptime_opt

        expert = laptime_opt.expert_from_theta(json.load(open(expert_json))["theta"])
    else:
        expert = drive_data.Expert.for_track(track)
    expert.corner_weight = corner_weight
    return drive_data.generate(track, expert, n, seed)


def make_dataset(track_dir: str, n: int, seed: int, workers: int, corner_weight: float = 0.0, expert_json=None):
    per = int(np.ceil(n / workers))
    jobs = [(track_dir, per, seed * 1000 + i, corner_weight, expert_json) for i in range(workers)]
    with ProcessPoolExecutor(workers) as pool:
        shards = list(pool.map(_shard, jobs))
    return tuple(np.concatenate([s[i] for s in shards]) for i in range(4))


def mask_loss(logits, target):
    bce = F.binary_cross_entropy_with_logits(logits, target)
    p = torch.sigmoid(logits)
    dice = 1 - (2 * (p * target).sum((1, 2)) + 1) / (p.sum((1, 2)) + target.sum((1, 2)) + 1)
    return bce + dice.mean()


def drive_loss(points, speeds, true_points, true_speeds):
    """Smooth L1 in metres and m/s; near waypoints count more, since they are
    what the controller steers by first."""
    k = points.shape[1]
    weight = torch.linspace(1.5, 0.5, k, device=points.device)
    path = (F.smooth_l1_loss(points, true_points, reduction="none", beta=0.5).sum(-1) * weight).mean()
    speed = (F.smooth_l1_loss(speeds, true_speeds, reduction="none", beta=0.5) * weight).mean()
    return path, speed


@torch.no_grad()
def evaluate(net, data, dev, batch=64):
    net.eval()
    rgb, mask, points, speeds = data
    err_near, err_far, err_speed, ious = [], [], [], []
    for i in range(0, len(rgb), batch):
        x = torch.from_numpy(rgb[i : i + batch]).to(dev).permute(0, 3, 1, 2).float()
        m, p, v = net.forward_all(x)
        p, v, m = p.cpu().numpy(), v.cpu().numpy(), (m > 0).cpu().numpy()
        dist = np.linalg.norm(p - points[i : i + batch], axis=-1)
        err_near.append(dist[:, :5].mean(1))  # 2 to 10 m ahead
        err_far.append(dist[:, 5:].mean(1))  # 12 to 24 m ahead
        err_speed.append(np.abs(v - speeds[i : i + batch]).mean(1))
        t = mask[i : i + batch]
        inter, union = (m & t).sum((1, 2)), (m | t).sum((1, 2))
        ious.append(np.where(union > 0, inter / np.maximum(union, 1), 1.0))
    return {
        "line_error_near_m": float(np.concatenate(err_near).mean()),
        "line_error_far_m": float(np.concatenate(err_far).mean()),
        "speed_error_mps": float(np.concatenate(err_speed).mean()),
        "mask_iou": float(np.concatenate(ious).mean()),
    }


@torch.no_grad()
def real_mask_iou(net, rgb, mask, dev, batch=64):
    """Mean pavement IoU on held-out real frames, against the auto-labels."""
    net.eval()
    ious = []
    for i in range(0, len(rgb), batch):
        x = torch.from_numpy(rgb[i : i + batch]).to(dev).permute(0, 3, 1, 2).float()
        pred = (net.forward_all(x)[0] > 0).cpu().numpy()
        truth = mask[i : i + batch]
        inter, union = (pred & truth).sum((1, 2)), (pred | truth).sum((1, 2))
        ious.extend(np.where(union > 0, inter / np.maximum(union, 1), 1.0))
    return float(np.mean(ious))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", default="drive-v1")
    parser.add_argument("--track", default=str(ROOT / "tracks/purdue_gp"))
    parser.add_argument("--init", type=Path, help="segmenter or driver weights to start from")
    parser.add_argument("--frames", type=int, default=24000)
    parser.add_argument("--val-frames", type=int, default=1200)
    parser.add_argument("--epochs", type=int, default=10)
    parser.add_argument("--batch", type=int, default=32)
    parser.add_argument("--lr", type=float, default=1.5e-3)
    parser.add_argument("--mask-weight", type=float, default=0.5)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--dagger", type=Path, nargs="*", default=[], help="npz files from dagger.py to train on too")
    parser.add_argument("--dagger-repeat", type=int, default=2, help="times each DAgger frame appears per epoch")
    parser.add_argument("--corner-weight", type=float, default=0.0, help="draw corner poses up to 1+w times as often")
    parser.add_argument("--expert-json", type=Path, help="best.json from laptime_opt.py: label with that expert's line")
    parser.add_argument("--fine", action="store_true", help="driving head also reads quarter-resolution features")
    parser.add_argument(
        "--real", type=Path, nargs="+", help="labels.csv files from autolabel.py: real footage for the mask"
    )
    parser.add_argument(
        "--real-hold", nargs="*", default=[], help="video ids that are never trained on, only validated"
    )
    parser.add_argument(
        "--real-share", type=float, default=0.4, help="real frames per batch, as a share of the batch size"
    )
    parser.add_argument("--real-val", type=float, default=0.2, help="fraction of the first set's videos held out")
    parser.add_argument("--real-per-video", type=int, default=120, help="cap on frames used from one real video")
    parser.add_argument("--real-mask-weight", type=float, default=1.0)
    args = parser.parse_args()

    torch.manual_seed(args.seed)
    dev = device()
    net = DriveNet(fine=args.fine).to(dev)
    if args.init:
        state = torch.load(args.init, map_location="cpu", weights_only=True)["state"]
        own = net.state_dict()
        # a segmenter has no driving head yet, and a coarse driver's head does not fit a fine one
        state = {k: v for k, v in state.items() if k in own and own[k].shape == v.shape}
        net.load_state_dict(state, strict=False)
        print(f"initialized {len(state)} of {len(own)} tensors from {args.init}", flush=True)
    out = HERE / "runs" / args.run
    out.mkdir(parents=True, exist_ok=True)
    metrics = (out / "metrics.jsonl").open("w", buffering=1)

    def record(**row):
        metrics.write(json.dumps(row) + "\n")

    record(type="start", args={k: str(v) for k, v in vars(args).items()}, device=str(dev), time=time.time())
    started = time.perf_counter()
    print(f"device {dev}; generating {args.frames} training and {args.val_frames} validation frames", flush=True)
    train = make_dataset(
        args.track,
        args.frames,
        seed=1 + args.seed,
        workers=args.workers,
        corner_weight=args.corner_weight,
        expert_json=args.expert_json,
    )
    val = make_dataset(
        args.track, args.val_frames, seed=9000 + args.seed, workers=args.workers, expert_json=args.expert_json
    )
    repeats = np.arange(len(train[0]))  # which stored frame each epoch slot reads
    if args.dagger:
        extra = [np.load(f) for f in args.dagger]
        first = len(train[0])
        train = tuple(
            np.concatenate([train[i], *[e[k] for e in extra]])
            for i, k in enumerate(("rgb", "mask", "points", "speeds"))
        )
        # DAgger frames appear ``dagger_repeat`` times per epoch, by index, not by copying them
        repeats = np.concatenate([repeats, *[np.arange(first, len(train[0]))] * args.dagger_repeat])
        print(f"added {len(train[0]) - first} DAgger frames x {args.dagger_repeat}", flush=True)
    # The whole training set lives in GPU memory as uint8, so a step gathers its batch on the GPU
    # instead of indexing numpy and copying across every time.
    gpu = tuple(torch.from_numpy(a).to(dev) for a in train)
    print(f"{sum(t.numel() * t.element_size() for t in gpu) / 1e9:.1f} GB of training data on {dev}", flush=True)
    real = real_val = None
    n_real = 0
    if args.real:
        sys.path.insert(0, str(ROOT / "training" / "data"))
        from train import augment_real, load_real

        parts, vals, held_out = [], [], []
        for k, labels in enumerate(args.real):
            # the first set also holds out a random share of its videos; later sets only the named ones
            train_part, val_part, held = load_real(
                labels, args.real_val if k == 0 else 0.0, args.seed, args.real_per_video, tuple(args.real_hold)
            )
            parts.append(train_part)
            vals.append(val_part)
            held_out += held
        r_rgb = np.concatenate([a[0] for a in parts])
        r_mask = np.concatenate([a[1] for a in parts])
        real_val = (np.concatenate([a[0] for a in vals]), np.concatenate([a[1] for a in vals]))
        real = (torch.from_numpy(r_rgb).to(dev), torch.from_numpy(r_mask).to(dev))
        n_real = max(1, round(args.batch * args.real_share))
        print(
            f"real footage: {len(r_rgb)} train frames, {len(real_val[0])} validation frames from held-out videos "
            f"{held_out}",
            flush=True,
        )
        record(type="real", train=len(r_rgb), val=len(real_val[0]), val_videos=held_out)
        real_rng = np.random.default_rng(args.seed + 77)
    steps = len(repeats) // args.batch
    optimizer = torch.optim.AdamW(net.parameters(), lr=args.lr, weight_decay=1e-4)
    schedule = torch.optim.lr_scheduler.OneCycleLR(optimizer, args.lr, total_steps=args.epochs * steps)
    history = []
    for epoch in range(args.epochs):
        net.train()
        order = repeats[np.random.default_rng(epoch).permutation(len(repeats))]
        flips = np.random.default_rng(100 + epoch)
        totals = np.zeros(3)
        for step in range(steps):
            idx = torch.from_numpy(order[step * args.batch : (step + 1) * args.batch]).to(dev)
            x = gpu[0][idx].permute(0, 3, 1, 2).float()
            m = gpu[1][idx].float()
            p = gpu[2][idx]
            v = gpu[3][idx]
            if flips.random() < 0.5:  # a mirrored frame is a mirrored line on a mirrored track
                x, m = x.flip(-1), m.flip(-1)
                p = p * torch.tensor([1.0, -1.0], device=dev)
            if real is None:
                logits, points, speeds = net.forward_all(x)
                lm = mask_loss(logits, m)
                lp, lv = drive_loss(points, speeds, p, v)
                loss = lp + lv + args.mask_weight * lm
            else:
                # Real frames add to the batch and teach the mask only: they have no racing line.
                ridx = torch.randint(0, len(real[0]), (n_real,), device=dev)
                rx, ry = augment_real(real[0][ridx].permute(0, 3, 1, 2).float(), real[1][ridx].float(), real_rng)
                if flips.random() < 0.5:
                    rx, ry = rx.flip(-1), ry.flip(-1)
                logits, points, speeds = net.forward_all(torch.cat([x, rx]))
                nb = x.shape[0]
                lm = mask_loss(logits[:nb], m)
                lp, lv = drive_loss(points[:nb], speeds[:nb], p, v)
                loss = lp + lv + args.mask_weight * lm + args.real_mask_weight * mask_loss(logits[nb:], ry)
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            optimizer.step()
            schedule.step()
            totals += [float(lp.detach()), float(lv.detach()), float(lm.detach())]
            if step % 20 == 0:
                record(
                    type="step",
                    epoch=epoch + 1,
                    step=epoch * steps + step,
                    loss=float(loss.detach()),
                    path=float(lp.detach()),
                    speed=float(lv.detach()),
                    mask=float(lm.detach()),
                    t=time.perf_counter() - started,
                )
        scores = evaluate(net, val, dev)
        if real_val is not None and len(real_val[0]):
            scores["real_val_iou"] = real_mask_iou(net, *real_val, dev)
        row = {
            "epoch": epoch + 1,
            **dict(zip(("path_loss", "speed_loss", "mask_loss"), (totals / steps).tolist(), strict=True)),
            **scores,
        }
        history.append(row)
        record(type="epoch", **row, t=time.perf_counter() - started)
        print(
            f"epoch {epoch + 1}/{args.epochs} line error near {scores['line_error_near_m']:.2f} m, "
            f"far {scores['line_error_far_m']:.2f} m, speed {scores['speed_error_mps']:.2f} m/s, "
            f"mask IoU {scores['mask_iou']:.3f}"
            + (f", real-footage IoU {scores['real_val_iou']:.3f}" if "real_val_iou" in scores else "")
            + f" ({time.perf_counter() - started:.0f} s)",
            flush=True,
        )
        torch.save(
            {
                "state": net.state_dict(),
                "kind": "drive",
                "fine": args.fine,
                "history": history,
                "init": str(args.init or ""),
            },
            out / "model.pt",
        )
    (out / "history.json").write_text(
        json.dumps({"args": vars(args), "history": history}, indent=2, default=str) + "\n"
    )
    record(type="done", t=time.perf_counter() - started)
    print(f"saved {out / 'model.pt'}")


if __name__ == "__main__":
    main()
