"""Open-loop recorded-command replay. No fitting and no repeated pose resets."""

from __future__ import annotations

import csv
import hashlib
from dataclasses import asdict
from pathlib import Path

import numpy as np

from mcq_sim.dynamics import DynamicKart, DynamicState
from mcq_sim.environment import heading_error, load_kart

REQUIRED = ("t", "x", "y", "yaw", "v", "steer", "yaw_rate", "v_lateral", "steer_cmd", "throttle_cmd", "brake_cmd")


def validate_recording(path, config=None):
    """Commands on row i apply during (t[i-1], t[i]]. All units SI.

    The recording must be a continuous, synchronized session in the track ENU
    frame. Pose is rear axle; v_lateral is at CG. The first row initializes the
    model. Actuator history before it is unknown, and is reported as such.
    """
    path = Path(path)
    with path.open() as f:
        reader = csv.DictReader(f)
        if not set(REQUIRED) <= set(reader.fieldnames or []):
            raise ValueError(f"recording requires columns {', '.join(REQUIRED)}")
        rows = list(reader)
    values = {k: np.array([float(r[k]) for r in rows]) for k in REQUIRED}
    if len(rows) < 3 or not all(np.isfinite(v).all() for v in values.values()):
        raise ValueError("recording needs at least three finite rows")
    intervals = np.diff(values["t"])
    if np.any(intervals <= 0) or np.any(intervals > 0.2 + 1e-9):
        raise ValueError("timestamps must increase with no gap larger than 0.2 s")
    if np.any(values["v"] < 0):
        raise ValueError("reverse motion is not supported")
    for key in ("throttle_cmd", "brake_cmd"):
        if np.any(values[key] < 0) or np.any(values[key] > 1):
            raise ValueError(f"{key} must be in [0, 1]")
    params, _, provenance = load_kart(config)
    initial = {k: float(values[k][0]) for k in ("x", "y", "yaw", "v", "steer", "yaw_rate", "v_lateral")}
    for key in ("throttle", "brake"):
        if key in rows[0]:
            value = float(rows[0][key])
            if not np.isfinite(value) or not 0 <= value <= 1:
                raise ValueError(f"initial {key} must be finite and in [0, 1]")
            initial[key] = value
    sim = DynamicKart(params, DynamicState(**initial))
    predicted = {k: [] for k in ("x", "y", "yaw", "v", "yaw_rate")}
    for i, interval in enumerate(intervals, 1):
        # Replay piecewise-constant measured commands, never controller output.
        count = max(1, int(np.ceil(interval / 0.01 - 1e-9)))
        sim.dt = float(interval / count)
        for _ in range(count):
            sim.step(values["steer_cmd"][i], values["throttle_cmd"][i], values["brake_cmd"][i])
        for key in predicted:
            predicted[key].append(getattr(sim.state, key))
    errors = {k: np.array(v) - values[k][1:] for k, v in predicted.items()}
    position = np.hypot(errors["x"], errors["y"])
    yaw = heading_error(predicted["yaw"], values["yaw"][1:])

    def rmse(data):
        return float(np.sqrt(np.mean(np.square(data))))

    return {
        "kind": "continuous_open_loop_replay",
        "file": path.name,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "samples": len(rows),
        "duration_s": float(values["t"][-1] - values["t"][0]),
        "position_rmse_m": rmse(position),
        "position_p95_m": float(np.percentile(position, 95)),
        "speed_rmse_mps": rmse(errors["v"]),
        "yaw_rmse_deg": float(np.degrees(rmse(yaw))),
        "yaw_rate_rmse_radps": rmse(errors["yaw_rate"]),
        "speed_normalized_rmse": rmse(errors["v"]) / max(float(np.max(np.abs(values["v"]))), 1.0),
        "accuracy_validated": False,
        "interpretation": "Errors for this recording only. Data provenance and held-out coverage need human review.",
        "initialization": "One measured state reset; zero pending command history; optional initial pedal feedback.",
        "parameter_provenance": provenance,
        "vehicle_parameters": asdict(params),
    }
