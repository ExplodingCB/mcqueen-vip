"""How much of what is wrong between the simulator and the real kart can the
driving model take?

    PYTHONPATH=training/perception:src/mcq_sim python training/perception/robustness.py \\
        --weights runs/drive-v2/model.pt --cap 6

Each case drives the same lap with the model, from the camera alone, with one
thing changed. None of the changes is visible to the model as a label: it has
to cope with them the way it would on the real track.

  track      the pavement is wider or narrower than the track file says, or its
             edges wander. The model never reads the track file; the stack
             drives what the camera sees. This checks that, and checks the model
             is not leaning on having memorized the one track it trained on.
  camera     the real mount is not the assumed one: pitch, height and field of
             view are off by amounts a careless install produces. The model and
             the corridor tracker keep the nominal calibration, as they would on
             a kart that was not recalibrated.
  image      the picture itself is worse: darker, brighter, noisy, blurred.
  gnss       the position feed is worse than the sim's RTK: five times the noise,
             a ten second outage, float only for the whole lap.
  actuator   the kart does not do what it is told as well as the sim's does. The
             steering servo does not land where commanded: a dead zone, a gain
             error, an offset, jitter, a slow slew. Brakes and throttle are
             weak, grip is low. The model outputs a line and a speed, and the
             controllers close the loop on measured state; this measures how
             much sloppiness that absorbs.

A case passes if the lap completes with no boundary violation. The nominal lap is
the reference; a case that fails where nominal passes is a measured weakness.
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

# name, group, what changes
CASES = [
    ("nominal", "reference", {}),
    ("pavement +0.5 m each side", "track", {"widen": 0.5}),
    ("pavement -0.5 m each side", "track", {"widen": -0.5}),
    ("edges wander +-0.4 m", "track", {"wander": 0.4}),
    ("camera pitch +3 deg", "camera", {"pitch": 3.0}),
    ("camera pitch -3 deg", "camera", {"pitch": -3.0}),
    ("camera 10 cm higher", "camera", {"height": 0.1}),
    ("camera 10 cm lower", "camera", {"height": -0.1}),
    ("camera fov +8 deg", "camera", {"fov": 8.0}),
    ("camera fov -8 deg", "camera", {"fov": -8.0}),
    ("image dark (x0.55)", "image", {"gain": 0.55}),
    ("image bright (x1.5)", "image", {"gain": 1.5}),
    ("image noisy (sigma 10)", "image", {"noise": 10.0}),
    ("image blurred", "image", {"blur": 2}),
    ("servo dead zone 2 deg", "actuator", {"servo": {"deadband": 0.035}}),
    ("servo gain -12%", "actuator", {"servo": {"gain": 0.88}}),
    ("servo gain +12%", "actuator", {"servo": {"gain": 1.12}}),
    ("servo offset +2 deg", "actuator", {"servo": {"bias": 0.035}}),
    ("servo jitter 0.6 deg", "actuator", {"servo": {"jitter": 0.01}}),
    ("servo slow (1.2 rad/s)", "actuator", {"kart": {"steer_rate_max": 1.2}}),
    ("brakes 40% weaker", "actuator", {"kart": {"brake_decel_max": 3.6}}),
    ("throttle response 2.5x slower", "actuator", {"kart": {"throttle_tau": 0.30}}),
    ("grip 0.75", "actuator", {"kart": {"friction": 0.75}}),
    ("all servo errors at once", "actuator", {"servo": "degraded"}),
    ("servo + brakes + grip, milder", "actuator", {"servo": "degraded", "kart": "degraded"}),
    ("gnss 5x noise", "gnss", {"gnss_sigma": 0.1}),
    ("gnss 10 s outage", "gnss", {"outage": True}),
    ("gnss float only", "gnss", {"float_only": True}),
]


def _perturbed_track(track, widen=0.0, wander=0.0, seed=0):
    from mcq_sim.track import Track

    wl, wr = track.w_left + widen, track.w_right + widen
    if wander:
        rng = np.random.default_rng(seed)
        n = len(track.x)
        # smooth noise: a few random knots, interpolated, so edges drift over tens of metres
        knots = rng.normal(0.0, wander, (2, 12))
        grid = np.linspace(0, 1, 12)
        at = np.linspace(0, 1, n)
        wl = wl + np.interp(at, grid, knots[0])
        wr = wr + np.interp(at, grid, knots[1])
    wl, wr = np.maximum(wl, 1.8), np.maximum(wr, 1.8)
    out = Track(track.x, track.y, wr, wl, closed=track.closed, track_id=track.track_id + "-perturbed")
    out.meta = track.meta
    return out


class _DegradedSegmenter:
    """The model behind a worse picture."""

    def __init__(self, inner, gain=1.0, noise=0.0, blur=0, seed=0):
        self.inner, self.gain, self.noise, self.blur = inner, gain, noise, blur
        self.rng = np.random.default_rng(seed)

    def degrade(self, rgb):
        img = rgb.astype(np.float32) * self.gain
        if self.blur:
            from scipy.ndimage import uniform_filter

            img = uniform_filter(img, size=(2 * self.blur + 1, 2 * self.blur + 1, 1))
        if self.noise:
            img = img + self.rng.normal(0.0, self.noise, img.shape)
        return np.clip(img, 0, 255).astype(np.uint8)

    def predict_drive(self, rgb):
        return self.inner.predict_drive(self.degrade(rgb))

    def predict(self, rgb):
        return self.predict_drive(rgb)[0]


def run_case(job):
    name, group, change, weights, cap, seed, race, seconds = job
    os.environ["MCQ_DRIVER_WEIGHTS"] = str(weights)
    from mcq_sim.camera import CameraParams, TrackCamera
    from mcq_sim.environment import Simulator, evaluate
    from mcq_sim.params import RACE_LIMITS, load_params
    from mcq_sim.track import Track

    track = Track.load(ROOT / "tracks/purdue_gp")
    if "widen" in change or "wander" in change:
        track = _perturbed_track(track, change.get("widen", 0.0), change.get("wander", 0.0), seed)
    overrides = dict(RACE_LIMITS) if race else {}
    if "gnss_sigma" in change:
        overrides["sensors.gnss.sigma_fixed"] = change["gnss_sigma"]
    if change.get("outage"):
        overrides["sensors.gnss.schedule"] = [[0.0, "FIXED"], [20.0, "NONE"], [30.0, "FIXED"]]
    if change.get("float_only"):
        overrides["sensors.gnss.schedule"] = [[0.0, "FLOAT"]]
    sim = Simulator(
        track,
        policy="stack",
        perception="driver:create",
        speed_cap=cap,
        seed=seed,
        tuning=load_params().override(overrides),
    )
    if "kart" in change:
        from dataclasses import replace

        from mcq_sim.params import DEGRADED_KART

        kart = DEGRADED_KART if change["kart"] == "degraded" else change["kart"]
        sim.vehicle_params = replace(sim.vehicle_params, **kart)
        sim.reset()
    servo = change.get("servo")
    if servo:
        from mcq_sim.actuators import DEGRADED, ServoImperfection

        ServoImperfection(**(DEGRADED if servo == "degraded" else servo), seed=seed).install(sim.stack)
    if group == "camera":
        nominal = sim.camera.params
        true_mount = CameraParams(
            width=nominal.width,
            height=nominal.height,
            horizontal_fov_deg=nominal.horizontal_fov_deg + change.get("fov", 0.0),
            height_m=nominal.height_m + change.get("height", 0.0),
            pitch_down_deg=nominal.pitch_down_deg + change.get("pitch", 0.0),
            forward_m=nominal.forward_m,
            far_m=nominal.far_m,
        )
        # What the camera really sees; the stack's corridor tracker keeps the nominal geometry.
        sim.perception.camera = TrackCamera(track, true_mount, appearance=sim.camera.appearance)
    if group == "image":
        sim.perception.segmenter = _DegradedSegmenter(
            sim.perception.segmenter, change.get("gain", 1.0), change.get("noise", 0.0), change.get("blur", 0), seed
        )
    started = time.perf_counter()
    r = evaluate(sim, seconds)
    return {
        "case": name,
        "group": group,
        "passed": bool(r["passed"]) and not r["boundary_violations"],
        "stop_reason": r["stop_reason"],
        "progress_m": r["progress_m"],
        "lap_s": r["sim_time_s"] if r["passed"] else None,
        "clearance_m": r["minimum_body_clearance_m"],
        "model_vetoes": r["model_vetoes"],
        "model_corrections": r.get("model_corrections"),
        "estimator_max_err_m": (r.get("estimator") or {}).get("position_error_max_m"),
        "wall_s": round(time.perf_counter() - started, 1),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--weights", required=True)
    parser.add_argument("--cap", type=float, default=6.0)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--race-limits", action="store_true")
    parser.add_argument("--seconds", type=float, default=240.0)
    parser.add_argument("--groups", nargs="*", help="only these groups (track camera image actuator gnss)")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--out", type=Path, default=ROOT / "logs" / "robustness")
    args = parser.parse_args()
    weights = Path(args.weights) if Path(args.weights).is_absolute() else HERE / args.weights
    cases = [c for c in CASES if not args.groups or c[1] in args.groups or c[1] == "reference"]
    jobs = [(n, g, ch, weights.resolve(), args.cap, args.seed, args.race_limits, args.seconds) for n, g, ch in cases]
    rows = []
    with ProcessPoolExecutor(args.workers) as pool:
        for row in pool.map(run_case, jobs):
            rows.append(row)
            verdict = (
                f"lap {row['lap_s']:.1f} s, clearance {row['clearance_m']:.2f} m"
                if row["passed"]
                else (f"FAIL {row['stop_reason']} at {row['progress_m']:.0f} m")
            )
            print(f"  {row['group']:>9}  {row['case']:<28} {verdict}", flush=True)
    passed = sum(r["passed"] for r in rows)
    print(f"\n{passed} of {len(rows)} cases complete the lap")
    args.out.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    (args.out / f"{weights.parent.name}-{stamp}.json").write_text(
        json.dumps({"cap": args.cap, "rows": rows}, indent=2) + "\n"
    )


if __name__ == "__main__":
    main()
