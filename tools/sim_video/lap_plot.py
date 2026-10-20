"""Top-down plot of a simulated drive: the track, the expert racing line, the
lines the driving model chose, the corridor the stack perceived, the path it
drove coloured by speed, and the cycles where the stack vetoed the model.

    PYTHONPATH=training/perception:src/mcq_sim python tools/sim_video/lap_plot.py \\
        --weights training/perception/runs/drive-v2/model.pt --cap 8 --out shot.png

By default the whole lap is drawn; ``--follow`` zooms on where the run ended or
stopped, which is the picture to look at when a lap fails. ``--oracle-line``
drives the expert's own labels through the model's path instead of a network:
the best any model trained on those labels could do.
"""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--weights", help="driver checkpoint; omit with --oracle-line")
    parser.add_argument("--oracle-line", action="store_true")
    parser.add_argument("--cap", type=float, default=6.0)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--seconds", type=float, default=240.0)
    parser.add_argument("--race-limits", action="store_true")
    parser.add_argument("--follow", action="store_true", help="zoom on where the run ended")
    parser.add_argument("--title", default="")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    if args.weights:
        os.environ["MCQ_DRIVER_WEIGHTS"] = str(Path(args.weights).resolve())
    import sys

    sys.path.insert(0, str(ROOT / "training/perception"))
    import race_eval

    from mcq_sim.environment import Simulator
    from mcq_sim.params import RACE_LIMITS, load_params
    from mcq_sim.track import Track

    track = Track.load(ROOT / "tracks/purdue_gp")
    tuning = load_params().override(RACE_LIMITS) if args.race_limits else None
    if args.oracle_line:
        sim = race_eval.oracle_line_sim(track, args.cap, args.seed, tuning)
    else:
        sim = Simulator(
            track, policy="stack", perception="driver:create", speed_cap=args.cap, seed=args.seed, tuning=tuning
        )
    stack = sim.stack
    vetoes, trail, lines = [], [], []
    original = stack._veto

    def veto(reason):
        s = sim.kart.state
        vetoes.append((s.x, s.y))
        return original(reason)

    stack._veto = veto
    while not sim.reason and sim.kart.state.t < args.seconds and sim.progress < track.length:
        sim.step()
        s = sim.kart.state
        trail.append((s.x, s.y, s.v))
        if stack.model_line is not None and (not lines or lines[-1] is not stack.model_line):
            lines.append(stack.model_line)
    s = sim.kart.state
    report = sim.report()

    fig, ax = plt.subplots(figsize=(9, 9))
    left, right = track.edges()
    ax.fill(np.concatenate([left[:, 0], right[::-1, 0]]), np.concatenate([left[:, 1], right[::-1, 1]]), color="#d7d7d2")
    for edge in (left, right):
        ax.plot(edge[:, 0], edge[:, 1], "k-", lw=0.8)
    reference = stack.reference
    rs = np.arange(0, reference.length, 0.5)
    rx, ry, _ = reference.cartesian(rs, np.zeros_like(rs))
    ax.plot(rx, ry, color="#666", lw=1, label="minimum-curvature reference")
    step = max(1, len(lines) // 60)
    for line in lines[::step]:
        ax.plot(line[:, 0], line[:, 1], color="#f0a030", lw=0.6, alpha=0.5)
    arr = np.array(trail)
    scatter = ax.scatter(arr[:, 0], arr[:, 1], c=arr[:, 2], s=5, cmap="viridis", vmin=0, vmax=max(arr[:, 2].max(), 1))
    fig.colorbar(scatter, label="speed (m/s)", shrink=0.6)
    if vetoes:
        v = np.array(vetoes)
        ax.plot(v[:, 0], v[:, 1], "rx", ms=5, label=f"vetoed cycles ({len(v)})")
    ax.plot(s.x, s.y, "ro", ms=8, label="end")
    ax.plot([], [], color="#f0a030", label="model line (sampled)")
    ax.set_aspect("equal")
    if args.follow:
        ax.set_xlim(s.x - 25, s.x + 25)
        ax.set_ylim(s.y - 25, s.y + 25)
    ax.legend(loc="lower left", fontsize=8)
    verdict = "lap complete" if report["passed"] else f"stopped: {report['stop_reason']}"
    ax.set_title(
        f"{args.title or 'drive'}  cap {args.cap:g} m/s  t={s.t:.1f} s  {verdict}  "
        f"clearance {report['minimum_body_clearance_m']:.2f} m  vetoes {stack.model_vetoes}",
        fontsize=9,
    )
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=90, bbox_inches="tight")
    sidecar = {
        "title": args.title or "drive",
        "weights": args.weights or ("oracle line" if args.oracle_line else ""),
        "cap": args.cap,
        "seed": args.seed,
        "race_limits": args.race_limits,
        "passed": bool(report["passed"]),
        "verdict": verdict,
        "progress_m": float(report["progress_m"]),
        "time_s": float(report["sim_time_s"]),
        "clearance_m": float(report["minimum_body_clearance_m"]),
        "vetoes": int(stack.model_vetoes),
        "made": time.time(),
    }
    Path(args.out).with_suffix(".json").write_text(json.dumps(sidecar) + "\n")
    print(f"{verdict}: {report['progress_m']:.0f} m, {report['sim_time_s']:.1f} s -> {args.out}")


if __name__ == "__main__":
    main()
