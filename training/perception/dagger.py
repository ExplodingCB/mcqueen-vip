"""DAgger data: frames from the states the driving model actually reaches.

    PYTHONPATH=training/perception:src/mcq_sim python training/perception/dagger.py \\
        --weights runs/drive-v2/model.pt --rollouts 160 --out runs/dagger-v2.npz

A model trained only on poses sampled around the expert's line never sees the
states it gets into once it drives: a little off the line, heading slightly
wrong, at speed, on a corner exit. Those errors compound until it leaves the
pavement. Here the model drives the full simulated stack from the camera alone
(its line filtered to the perceived corridor, as in the closed loop), with
random steering bumps so it wanders off its line. At each camera frame the
expert labels what it should have done from where the kart really is: the
racing line ahead of the true pose and the speed along it. Train on those
labels (``train_drive.py --dagger``) and the model learns to recover.

The expert's labels are for training only. At run time the model sees pixels.
"""

from __future__ import annotations

import argparse
import json
import os
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def rollout(job):
    weights, cap, seed, bump, stride, race, sloppy, expert_json = job
    os.environ["MCQ_DRIVER_WEIGHTS"] = str(weights)
    import drive_data

    from mcq_sim.environment import Simulator
    from mcq_sim.params import RACE_LIMITS, load_params
    from mcq_sim.track import Track

    track = Track.load(ROOT / "tracks/purdue_gp")
    if expert_json:  # label with a searched expert line (laptime_opt.py) instead of the minimum-curvature one
        import laptime_opt

        expert = laptime_opt.expert_from_theta(json.load(open(expert_json))["theta"])
    else:
        expert = drive_data.Expert.for_track(track)
    tuning = load_params().override(RACE_LIMITS) if race else None
    sim = Simulator(track, policy="stack", perception="driver:create", speed_cap=cap, seed=seed, tuning=tuning)
    rng = np.random.default_rng(seed)
    if sloppy:
        # each rollout drives a differently imperfect kart: every actuator and the grip scaled
        # between the nominal value and a clearly worse one (mcq_sim.params.DEGRADED_KART)
        from dataclasses import replace

        from mcq_sim.params import DEGRADED_KART

        nominal = sim.vehicle_params
        mix = {
            k: getattr(nominal, k) + float(rng.uniform(0.0, 1.2)) * (v - getattr(nominal, k))
            for k, v in DEGRADED_KART.items()
        }
        sim.vehicle_params = replace(nominal, **mix)
        sim.reset()
        from mcq_sim.actuators import sample

        sample(rng).install(sim.stack)
    stack, perception = sim.stack, sim.perception

    # Steering bumps: a random offset held for a second or so, every few seconds.
    state = {"until": -1.0, "offset": 0.0, "next": float(rng.uniform(3.0, 8.0))}
    tick = stack.tick

    def bumped(s):
        steer, throttle, brake = tick(s)
        if s.t >= state["next"]:
            state.update(until=s.t + float(rng.uniform(0.4, 1.2)), offset=float(rng.normal(0.0, bump)))
            state["next"] = s.t + float(rng.uniform(3.0, 8.0))
        if s.t < state["until"]:
            steer = float(np.clip(steer + state["offset"], -0.45, 0.45))
        return steer, throttle, brake

    stack.tick = bumped

    frames, calls = [], [0]
    observe = perception.observe

    def recording(s):
        mask = observe(s)
        calls[0] += 1
        if calls[0] % stride == 0:
            points, speeds = expert.labels(s.x, s.y, s.yaw)
            frames.append((perception.last_rgb.copy(), perception.last_truth.copy(), points, speeds))
        return mask

    perception.observe = recording
    while not sim.reason and not stack.stop_reason and sim.kart.state.t < 240 and sim.progress < track.length:
        sim.step()
    if not frames:
        return None
    rgb = np.stack([drive_data.appearance_jitter(f[0], rng) for f in frames])
    half = np.stack([drive_data.half(r) for r in rgb])
    mask = np.stack([drive_data.half(f[1][..., None].astype(np.float32))[..., 0] >= 0.5 for f in frames])
    points = np.stack([f[2] for f in frames])
    speeds = np.stack([f[3] for f in frames])
    return half, mask, points, speeds, (seed, cap, sim.progress, sim.reason or stack.stop_reason or "lap")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--weights", required=True, help="driver checkpoint that drives the rollouts")
    parser.add_argument("--rollouts", type=int, default=96)
    parser.add_argument("--caps", nargs="+", type=float, default=[4.0, 6.0, 8.0])
    parser.add_argument("--bump", type=float, default=0.07, help="steering bump size, radians (1 sigma)")
    parser.add_argument("--stride", type=int, default=6, help="keep one camera frame in this many (20 Hz in)")
    parser.add_argument("--race-limits", action="store_true")
    parser.add_argument("--expert-json", type=Path, help="best.json from laptime_opt.py: label with that expert's line")
    parser.add_argument("--sloppy-kart", action="store_true", help="randomize grip and actuator quality per rollout")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()

    weights = Path(args.weights) if Path(args.weights).is_absolute() else HERE / args.weights
    jobs = [
        (
            weights.resolve(),
            args.caps[i % len(args.caps)],
            1000 * (args.seed + 1) + i,
            args.bump,
            args.stride,
            args.race_limits,
            args.sloppy_kart,
            args.expert_json,
        )
        for i in range(args.rollouts)
    ]
    started = time.perf_counter()
    parts, notes = [], []
    with ProcessPoolExecutor(args.workers) as pool:
        for i, result in enumerate(pool.map(rollout, jobs)):
            if result is None:
                continue
            parts.append(result[:4])
            notes.append(result[4])
            if (i + 1) % 8 == 0 or i + 1 == len(jobs):
                total = sum(len(p[0]) for p in parts)
                print(
                    f"{i + 1}/{len(jobs)} rollouts, {total} frames, {time.perf_counter() - started:.0f} s", flush=True
                )
    rgb, mask, points, speeds = (np.concatenate([p[k] for p in parts]) for k in range(4))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.out, rgb=rgb, mask=mask, points=points, speeds=speeds)
    reasons: dict[str, int] = {}
    for _, _, _, why in notes:
        reasons[why] = reasons.get(why, 0) + 1
    progress = np.array([n[2] for n in notes])
    print(
        f"saved {len(rgb)} frames to {args.out}; rollouts ended: {reasons}; median progress {np.median(progress):.0f} m"
    )


if __name__ == "__main__":
    main()
