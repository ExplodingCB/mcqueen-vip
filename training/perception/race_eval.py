"""Race candidate driving models against each other, and against the expert,
on full simulated laps of the Purdue track.

    PYTHONPATH=training/perception:src/mcq_sim python training/perception/race_eval.py \\
        --weights runs/drive-v1/model.pt runs/drive-v2/model.pt --caps 4 6 8 10 --seeds 0 1 2

Each candidate drives the full stack (sensors, estimator, controllers) from the
draped camera alone, at each speed cap and seed. A cap counts only if every
seed completes the lap with no boundary violation. A candidate's result is its
best lap time at the highest cap it clears; ties go to more clearance. The
expert (the map-driven stack on the racing line) runs the same caps as the bar:
the model is trained to copy it, so it cannot beat it, only match it.

Writes a leaderboard JSON beside each run and one summary for the race.
"""

from __future__ import annotations

import argparse
import json
import os
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
KEYS = (
    "passed",
    "stop_reason",
    "sim_time_s",
    "progress_m",
    "max_speed_mps",
    "minimum_body_clearance_m",
    "boundary_violations",
    "model_plans",
    "model_vetoes",
    "model_veto_reasons",
)


class _OracleLine:
    """Stands in for a driving model with perfect output: the true pavement mask
    and the expert's own labels at the true pose. Driven through exactly the
    path a trained model's output takes (corridor check, speed profile,
    controllers), it is the fastest any model trained on these labels can go."""

    def __init__(self, sim, expert):
        self.sim, self.expert = sim, expert

    def predict_drive(self, rgb):
        s = self.sim.kart.state
        _, truth = self.sim.camera.render(s)
        points, speeds = self.expert.labels(s.x, s.y, s.yaw)
        return truth.astype(float), points, speeds

    def predict(self, rgb):
        return self.predict_drive(rgb)[0]


def oracle_line_sim(track, cap, seed, tuning=None, expert=None, trusted=None):
    import drive_data

    from mcq_sim.environment import Simulator

    sim = Simulator(
        track,
        policy="stack",
        perception="camera-demo",
        camera_appearance="aerial",
        speed_cap=cap,
        seed=seed,
        tuning=tuning,
    )
    sim.perception.segmenter = _OracleLine(sim, expert or drive_data.Expert.for_track(track))
    if trusted:
        sim.stack.corridor.trusted_range = trusted
    return sim


def _lap(job):
    name, weights, cap, seed, seconds, race = job
    if weights:
        os.environ["MCQ_DRIVER_WEIGHTS"] = str(weights)
    from mcq_sim.environment import Simulator, evaluate
    from mcq_sim.params import RACE_LIMITS, load_params
    from mcq_sim.track import Track

    track = Track.load(ROOT / "tracks/purdue_gp")
    tuning = load_params().override(RACE_LIMITS) if race else None
    if name == "oracle-line":
        sim = oracle_line_sim(track, cap, seed, tuning)
    else:
        perception = "driver:create" if weights else "map"
        sim = Simulator(track, policy="stack", perception=perception, speed_cap=cap, seed=seed, tuning=tuning)
    started = time.perf_counter()
    report = evaluate(sim, seconds)
    row = {k: report.get(k) for k in KEYS}
    row.update(name=name, cap=cap, seed=seed, wall_s=round(time.perf_counter() - started, 1))
    return row


def summarize(rows, caps):
    """Per candidate: the highest cap where every seed passed, and the lap there."""
    out = {}
    for name in dict.fromkeys(r["name"] for r in rows):
        mine = [r for r in rows if r["name"] == name]
        cleared = []
        for cap in caps:
            at = [r for r in mine if r["cap"] == cap]
            if at and all(r["passed"] and not r["boundary_violations"] for r in at):
                cleared.append(
                    {
                        "cap": cap,
                        "lap_s": max(r["sim_time_s"] for r in at),
                        "min_clearance_m": min(r["minimum_body_clearance_m"] for r in at),
                    }
                )
        best = max(cleared, key=lambda c: (c["cap"], -c["lap_s"], c["min_clearance_m"])) if cleared else None
        furthest = max(mine, key=lambda r: r["progress_m"] or 0)
        out[name] = {"best": best, "cleared_caps": [c["cap"] for c in cleared], "furthest_m": furthest["progress_m"]}
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--weights", nargs="*", default=[], help="driver checkpoints (relative to training/perception)")
    parser.add_argument("--caps", nargs="+", type=float, default=[4, 6, 8])
    parser.add_argument("--seeds", nargs="+", type=int, default=[0, 1, 2])
    parser.add_argument("--no-expert", action="store_true")
    parser.add_argument("--race-limits", action="store_true", help="use mcq_sim.params.RACE_LIMITS")
    parser.add_argument("--oracle-line", action="store_true", help="also race perfect labels through the model's path")
    parser.add_argument("--seconds", type=float, default=240)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--out", type=Path, default=ROOT / "logs" / "race")
    args = parser.parse_args()

    candidates = [] if args.no_expert else [("expert", None)]
    if args.oracle_line:
        candidates.append(("oracle-line", None))
    for w in args.weights:
        path = Path(w) if Path(w).is_absolute() else HERE / w
        candidates.append((path.parent.name if path.name == "model.pt" else path.stem, path.resolve()))
    jobs = [(n, w, c, s, args.seconds, args.race_limits) for n, w in candidates for c in args.caps for s in args.seeds]
    print(f"{len(jobs)} laps: {[n for n, _ in candidates]} x caps {args.caps} x seeds {args.seeds}", flush=True)
    rows = []
    with ProcessPoolExecutor(args.workers) as pool:
        for row in pool.map(_lap, jobs):
            rows.append(row)
            print(
                f"  {row['name']:>16} cap {row['cap']:>4} seed {row['seed']}: "
                + (
                    f"lap {row['sim_time_s']:.1f} s, clearance {row['minimum_body_clearance_m']:.2f} m"
                    if row["passed"]
                    else f"FAIL {row['stop_reason']} at {row['progress_m']:.0f} m"
                )
                + (
                    f", vetoes {row['model_vetoes']}/{row['model_plans'] + row['model_vetoes']}"
                    if row["model_plans"]
                    else ""
                ),
                flush=True,
            )
    summary = summarize(rows, args.caps)
    args.out.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    (args.out / f"race-{stamp}.json").write_text(json.dumps({"rows": rows, "summary": summary}, indent=2) + "\n")
    print("\nleaderboard (highest cap every seed clears):")
    ranked = sorted(summary.items(), key=lambda kv: (kv[1]["best"] or {"cap": 0, "lap_s": 1e9})["cap"], reverse=True)
    for name, s in ranked:
        b = s["best"]
        line = (
            f"cap {b['cap']} m/s, lap {b['lap_s']:.1f} s, clearance {b['min_clearance_m']:.2f} m"
            if b
            else "no clean cap"
        )
        print(f"  {name:>16}: {line}   (cleared {s['cleared_caps']}, furthest {s['furthest_m']:.0f} m)")


if __name__ == "__main__":
    main()
