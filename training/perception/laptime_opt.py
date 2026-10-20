"""Search for the fastest lap the kart can drive, by simulating thousands of laps.

    PYTHONPATH=training/perception:src/mcq_sim python training/perception/laptime_opt.py \\
        --run laptime-v1 --generations 120

Imitating an expert can never beat the expert. This is the other half: treat the
lap as an optimization problem, score each candidate by its simulated lap time
through the whole stack (sensors, estimator, controllers, the kart's dynamics),
and let CMA-ES evolve the answer. What is searched:

  * the racing line: 40 lateral offsets around the track, joined smoothly,
  * the speed profile along it: the cornering, acceleration and braking limits
    that set how hard the kart is asked to drive.

A candidate that leaves the pavement, or comes within 0.25 m of an edge, is
penalized, so the optimum is a line the kart actually follows with room to
spare. It is scored on pavement 0.25 m narrower on each side than the track
file, and again with a sloppier kart (less grip, weaker brakes, and a steering servo with
backlash, gain and offset error), taking the slower lap, so the winner tolerates the file being off by
that much and the actuators being worse than the simulator assumes.

This is the privileged, offline part: it reads the map and the true pose, the
way a TrackMania speedrunner replays a track. Nothing here runs on the kart.
Its product is a faster expert; ``train_drive.py`` and ``dagger.py`` then teach
the camera model to reproduce it from pixels alone. The numbers are for the
simulator's placeholder tyres and drivetrain: the method transfers to the real
kart, the lap time does not, until the kart's grip and power are measured.
"""

from __future__ import annotations

import argparse
import functools
import json
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]

N_KNOTS = 40
NARROW_M = 0.25  # score on pavement this much narrower on each side
CLEARANCE_GOAL_M = 0.25  # body-to-edge clearance below this is penalized, on the narrowed pavement
SIGHT_M = 16.0  # how far ahead the corridor is known; the camera's trusted range, give or take


# Each candidate drives every world and the slowest counts: the nominal kart, and
# a sloppier one (less grip, laggier steering, weaker brakes: mcq_sim.params.
# DEGRADED_KART), since the real kart's actuators are unmeasured.
def worlds():
    """(seed, kart overrides, servo, on the real pavement?): the nominal kart on the narrowed pavement, a
    sloppier kart and servo on the narrowed pavement, and the nominal kart on the pavement as the track file
    has it. A line that only works where the corridor is tight (because the safety filter rescues it) fails
    the last one."""
    from mcq_sim.actuators import DEGRADED
    from mcq_sim.params import DEGRADED_KART

    return ((0, {}, None, False), (1, DEGRADED_KART, DEGRADED, False), (2, {}, None, True))


@functools.lru_cache(maxsize=1)
def real_track():
    from mcq_sim.track import Track

    return Track.load(ROOT / "tracks/purdue_gp")


CORRECTION_PENALTY_S = 0.03  # per cycle the safety filter had to pull the line back inside the corridor


# physical parameters appended after the offsets: (name, start, low, high, step)
SPEED_PARAMS = [("a_lat", 3.0, 2.5, 6.0, 0.4), ("a_long", 2.0, 1.5, 4.5, 0.3), ("a_brake", 3.0, 2.0, 4.5, 0.3)]
OFFSET_STEP_M = 0.5
MAX_OFFSET_M = 2.0
FAIL_BASE = 1000.0


@functools.lru_cache(maxsize=1)
def fitness_track():
    from mcq_sim.track import Track

    track = Track.load(ROOT / "tracks/purdue_gp")
    out = Track(
        track.x,
        track.y,
        np.maximum(track.w_right - NARROW_M, 1.8),
        np.maximum(track.w_left - NARROW_M, 1.8),
        closed=True,
        track_id=track.track_id + "-narrowed",
    )
    out.meta = track.meta
    return out


@functools.lru_cache(maxsize=1)
def base_line():
    """The minimum-curvature line on the narrowed pavement, the search's origin."""
    from mcq_sim.reference import smooth_reference

    return smooth_reference(fitness_track(), 1.3)


def line_from_offsets(offsets: np.ndarray, base=None, keep: float = 0.8 + CLEARANCE_GOAL_M):
    """The base line moved sideways by a smooth periodic function of s through
    ``offsets`` (metres, positive to the left), clipped to keep the body inside
    the pavement with the clearance goal."""
    from scipy.interpolate import CubicSpline

    from mcq_sim.track import Track

    base = base or base_line()
    knots = np.linspace(0.0, base.length, len(offsets), endpoint=False)
    spline = CubicSpline(np.append(knots, base.length), np.append(offsets, offsets[0]), bc_type="periodic")
    s = np.arange(0.0, base.length, 0.5)
    left, right = base.width_at(s)
    d = np.clip(spline(s), -(right - keep), left - keep)
    bx, by, heading = base.cartesian(s, np.zeros_like(s))
    nx, ny = -np.sin(heading), np.cos(heading)
    return Track(
        bx + d * nx,
        by + d * ny,
        right + d,
        left - d,
        closed=True,
        track_id="optimized-line",
        meta=dict(base.meta),
    )


def split(theta: np.ndarray):
    return theta[:N_KNOTS], theta[N_KNOTS:]


def expert_from_theta(theta: np.ndarray):
    import drive_data

    offsets, speed = split(np.asarray(theta, dtype=float))
    line = line_from_offsets(offsets)
    a_lat, a_long, a_brake = (float(v) for v in speed)
    return drive_data.Expert.from_line(fitness_track(), line, a_lat, a_long, a_brake)


def drive(
    theta,
    seed: int,
    kart: dict | None = None,
    servo: dict | None = None,
    steer_loop: bool = True,
    loop_gains: tuple | None = None,
    seconds: float = 150.0,
    track=None,
    sight: float = SIGHT_M,
    race=True,
):
    """One simulated lap with the optimized expert's labels standing in for the
    model, through the model's own path in the stack. Returns a result dict.
    ``track`` defaults to the narrowed fitness pavement."""
    from dataclasses import replace

    from mcq_sim.environment import Simulator, evaluate
    from mcq_sim.params import RACE_LIMITS, load_params

    track = track or fitness_track()
    expert = expert_from_theta(theta)
    tuning = load_params().override(RACE_LIMITS) if race else None
    sim = Simulator(
        track,
        policy="stack",
        perception="camera-demo",
        camera_appearance="flat",
        speed_cap=25.0,
        seed=seed,
        tuning=tuning,
    )
    sim.vehicle_params = replace(sim.vehicle_params, **(kart or {}))
    sim.reset()
    perception, stack = sim.perception, sim.stack
    stack.steer_loop = steer_loop
    if loop_gains:
        stack.steer_kp, stack.steer_ki, stack.steer_i_max = loop_gains
    if servo:
        from mcq_sim.actuators import ServoImperfection

        ServoImperfection(**servo, seed=seed).install(stack)

    def observe(s):  # no camera: the labels are the model's output
        points, speeds = expert.labels(s.x, s.y, s.yaw)
        perception.last_drive = (points.astype(float), speeds.astype(float))
        perception.ious.append(1.0)
        perception.inference_ms.append(0.0)

    def corridor(_mask, _x, _y, _yaw, _t):  # the pavement ahead, as the camera would report it
        s = sim.kart.state
        return track.bounds_ahead(s.x, s.y, s.yaw, sight, 1.0)

    perception.observe = observe
    stack.corridor.update = corridor
    report = evaluate(sim, seconds)
    return {
        "passed": bool(report["passed"]) and not report["boundary_violations"],
        "lap_s": report["sim_time_s"],
        "progress_m": report["progress_m"],
        "clearance_m": report["minimum_body_clearance_m"],
        "max_speed": report["max_speed_mps"],
        "corrections": int(report.get("model_corrections") or 0),
        "stop": report["stop_reason"],
    }


def score(result: dict, track_length: float) -> float:
    if not result["passed"]:
        return FAIL_BASE + (track_length - result["progress_m"])
    shortfall = max(0.0, CLEARANCE_GOAL_M - result["clearance_m"])
    return result["lap_s"] + 60.0 * shortfall + CORRECTION_PENALTY_S * result.get("corrections", 0)


def evaluate_candidate(theta):
    """Score in every world; the worst counts, so a line that only works on a lucky run or on grip
    the real kart may not have loses."""
    results = [
        drive(theta, seed, kart, servo, track=real_track() if real else None) for seed, kart, servo, real in worlds()
    ]
    length = fitness_track().length
    return max(score(r, length) for r in results), results


def initial_theta():
    return np.concatenate([np.zeros(N_KNOTS), [p[1] for p in SPEED_PARAMS]])


def main():
    import cma

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", default="laptime-v1")
    parser.add_argument("--generations", type=int, default=120)
    parser.add_argument("--population", type=int, default=16)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--minutes", type=float, default=None, help="stop after this long")
    parser.add_argument("--resume", type=Path, help="start from a best.json")
    parser.add_argument("--sigma", type=float, default=1.0)
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args()

    out = HERE / "runs" / args.run
    out.mkdir(parents=True, exist_ok=True)
    metrics = (out / "metrics.jsonl").open("w", buffering=1)

    def record(**row):
        metrics.write(json.dumps(row) + "\n")

    x0 = np.array(json.loads(args.resume.read_text())["theta"]) if args.resume else initial_theta()
    low = np.concatenate([np.full(N_KNOTS, -MAX_OFFSET_M), [p[2] for p in SPEED_PARAMS]])
    high = np.concatenate([np.full(N_KNOTS, MAX_OFFSET_M), [p[3] for p in SPEED_PARAMS]])
    stds = np.concatenate([np.full(N_KNOTS, OFFSET_STEP_M), [p[4] for p in SPEED_PARAMS]])
    es = cma.CMAEvolutionStrategy(
        x0.tolist(),
        args.sigma,
        {
            "popsize": args.population,
            "bounds": [low.tolist(), high.tolist()],
            "CMA_stds": stds.tolist(),
            "seed": args.seed,
            "verbose": -9,
        },
    )
    record(
        type="start",
        args={
            "epochs": args.generations,
            "batch": 1,
            "population": args.population,
            "narrow_m": NARROW_M,
            "sight_m": SIGHT_M,
            "worlds": ["nominal kart, narrowed", "degraded kart, narrowed", "nominal kart, real pavement"],
            "run": args.run,
        },
        device="cma-es",
        time=time.time(),
    )
    print(
        f"{args.run}: {N_KNOTS + len(SPEED_PARAMS)} parameters, population {args.population}, "
        f"scored on pavement {NARROW_M} m narrower, worst of three worlds",
        flush=True,
    )

    best_score, best = np.inf, None
    started = time.perf_counter()
    with ProcessPoolExecutor(args.workers) as pool:
        for generation in range(1, args.generations + 1):
            candidates = es.ask()
            outcomes = list(pool.map(evaluate_candidate, candidates))
            scores = [o[0] for o in outcomes]
            es.tell(candidates, scores)
            i = int(np.argmin(scores))
            finished = float(np.mean([s < FAIL_BASE for s in scores]))
            if scores[i] < best_score:
                best_score, best = (
                    scores[i],
                    {
                        "theta": [float(v) for v in candidates[i]],
                        "score": scores[i],
                        "results": outcomes[i][1],
                        "generation": generation,
                    },
                )
                (out / "best.json").write_text(json.dumps(best, indent=2) + "\n")
            elapsed = time.perf_counter() - started
            both_finish = all(r["passed"] for r in best["results"])
            worst_lap = max(r["lap_s"] for r in best["results"]) if both_finish else None
            record(
                type="epoch",
                epoch=generation,
                best_lap_s=worst_lap,
                gen_best_score=scores[i],
                finished_fraction=finished,
                sigma=float(es.sigma),
                best_clearance_m=min(r["clearance_m"] for r in best["results"]),
                t=elapsed,
            )
            print(
                f"gen {generation:3d}: best score {best_score:7.2f} (this gen {scores[i]:7.2f}), "
                f"{finished:4.0%} of candidates finish, sigma {es.sigma:.2f}, {elapsed / 60:.1f} min",
                flush=True,
            )
            if args.minutes and elapsed > args.minutes * 60:
                break
    record(type="done", t=time.perf_counter() - started)
    print(f"saved {out / 'best.json'}: {json.dumps(best['results'])}")


if __name__ == "__main__":
    main()
