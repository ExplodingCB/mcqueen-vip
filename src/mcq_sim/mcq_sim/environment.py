"""Fixed-step experiments with separate privileged baseline and camera policies."""

from __future__ import annotations

import importlib
import math
import sys
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np
import yaml

from mcq_sim.camera import CameraParams, DemoSegmenter, TrackCamera, checked_mask, mask_iou
from mcq_sim.dynamics import DynamicKart, DynamicsParams, DynamicState
from mcq_sim.track import Track, wrap_angle


def default_kart_config():
    source = Path(__file__).resolve().parent.parent / "config/kart_provisional.yaml"
    if source.exists():
        return source
    installed = Path(sys.prefix) / "share/mcq_sim/config/kart_provisional.yaml"
    if installed.exists():
        return installed
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("mcq_sim")) / "config/kart_provisional.yaml"


def load_kart(path=None):
    raw = yaml.safe_load(Path(path or default_kart_config()).read_text())
    return DynamicsParams(**raw["dynamics"]), CameraParams(**raw["camera"]), raw["provenance"]


def load_segmenter(spec):
    if spec == "camera-demo":
        return DemoSegmenter()
    module, factory = spec.split(":", 1)
    model = getattr(importlib.import_module(module), factory)()
    if not callable(getattr(model, "predict", None)):
        raise ValueError("model factory must return an object with predict(rgb)")
    return model


class Simulator:
    """Physics at 100 Hz; control/perception at 20 Hz of simulated time.

    Timing is lockstep for reproducibility. Inference wall time is reported;
    this is not a real-time deadline emulator. Truth is kept out of predict().
    """

    control_dt = 0.05

    def __init__(self, track: Track, config=None, policy="reference", speed_cap=4.0, seed=0):
        self.track = track
        self.vehicle_params, self.camera_params, self.provenance = load_kart(config)
        self.camera = TrackCamera(track, self.camera_params)
        self.policy_name = policy
        self.segmenter = None if policy in ("reference", "manual") else load_segmenter(policy)
        if not math.isfinite(speed_cap) or not 0 < speed_cap <= 12:
            raise ValueError("speed cap must be in (0, 12] m/s")
        self.speed_cap = speed_cap
        self.seed = seed
        self.reset()

    def reset(self):
        x, y, yaw = self.track.cartesian(0)
        self.kart = DynamicKart(self.vehicle_params, DynamicState(x=float(x[0]), y=float(y[0]), yaw=float(yaw[0])))
        self.rng = np.random.default_rng(self.seed)
        self.progress = 0.0
        self.previous_s = 0.0
        self.min_clearance = math.inf
        self.max_speed = 0.0
        self.violations = 0
        self.reason = None
        self.command = (0.0, 0.0, 0.0)
        self.log = []
        self.ious = []
        self.inference_ms = []
        self.last_prediction = None
        self.last_rgb = None
        self.last_truth = None
        self.last_frame_time = None
        if self.segmenter is not None and callable(getattr(self.segmenter, "reset", None)):
            self.segmenter.reset(self.seed)

    def _pedals(self, target):
        s, p = self.kart.state, self.vehicle_params
        desired = 1.8 * (target - s.v)
        if desired < -0.2:
            return 0.0, min(1.0, -desired / p.brake_decel_max)
        return float(
            np.clip((desired + p.rolling_decel + p.drag * s.v**2) / max(0.2, p.accel_max * (1 - s.v / p.v_max)), 0, 1)
        ), 0.0

    def _reference_command(self):
        s = self.kart.state
        along = float(self.track.frenet(s.x, s.y)[0][0])
        lookahead = max(2.0, 0.5 * s.v)
        # A user-drawn coordinate reference can sit off the asphalt midpoint.
        # This baseline centers the kart in the available pavement corridor;
        # it must not shift the physical edges to center the source line.
        left, right = self.track.width_at(along + lookahead)
        clearance = self.vehicle_params.half_width + 0.5
        lower, upper = clearance - float(right[0]), float(left[0]) - clearance
        if lower > upper:
            self.reason = "reference_corridor_too_narrow"
            return 0.0, 0.0, 1.0
        target_d = 0.5 * (float(left[0]) - float(right[0]))
        tx, ty, _ = self.track.cartesian(along + lookahead, target_d)
        dx, dy = float(tx[0]) - s.x, float(ty[0]) - s.y
        lateral = -math.sin(s.yaw) * dx + math.cos(s.yaw) * dy
        curvature = 2 * lateral / max(dx * dx + dy * dy, 0.01)
        preview_s = along + np.linspace(0, max(6, s.v * 2), 30)
        kmax = float(np.max(np.abs(self.track.curvature_at(preview_s))))
        target = min(self.speed_cap, math.sqrt(1.8 / max(kmax, 0.005)))
        steer = math.atan((self.vehicle_params.wheelbase + 0.002 * s.v**2) * curvature)
        return (steer, *self._pedals(target))

    def _camera_command(self):
        self.last_rgb, self.last_truth = self.camera.render(self.kart.state)
        self.last_frame_time = self.kart.state.t
        started = time.perf_counter()
        prediction = checked_mask(self.segmenter.predict(self.last_rgb.copy()), self.last_truth.shape)
        self.inference_ms.append((time.perf_counter() - started) * 1000)
        self.last_prediction = prediction
        self.ious.append(mask_iou(prediction, self.last_truth))
        # Ground-plane inverse projection. Only the predicted mask and speed are
        # used here. No centerline, true pose, true mask or off-camera road access.
        camera, s = self.camera, self.kart.state
        col = camera.params.width // 2
        lookahead = max(2.0, s.v * 0.55)
        rows = np.flatnonzero(camera.valid[:, col])
        row = int(rows[np.argmin(np.abs(camera.forward[rows, col] - lookahead))])
        indices = np.flatnonzero(prediction[row] & camera.valid[row])
        if len(indices) < 6:
            self.reason = "perception_no_drivable_path"
            return 0.0, 0.0, 1.0
        groups = np.split(indices, np.where(np.diff(indices) > 1)[0] + 1)
        groups = [g for g in groups if len(g) >= 6]
        if not groups:
            self.reason = "perception_no_drivable_path"
            return 0.0, 0.0, 1.0
        group = min(groups, key=lambda g: min(abs(g[0] - col), abs(g[-1] - col)) if not g[0] <= col <= g[-1] else 0)
        mid = int(round((int(group[0]) + int(group[-1])) / 2))
        forward, left = float(camera.forward[row, mid]), float(camera.left[row, mid])
        curve = 2 * left / max(forward**2 + left**2, 0.01)
        visible_width = abs(float(camera.left[row, group[0]] - camera.left[row, group[-1]]))
        if visible_width < 2 * self.vehicle_params.half_width + 0.4:
            self.reason = "perception_corridor_too_narrow"
            return 0.0, 0.0, 1.0
        target = min(self.speed_cap, math.sqrt(1.5 / max(abs(curve), 0.01)))
        return math.atan(self.vehicle_params.wheelbase * curve), *self._pedals(target)

    def step(self, manual=None):
        if self.reason:
            return
        try:
            if self.policy_name == "manual":
                self.command = tuple(manual or (0, 0, 1))
            elif self.policy_name == "reference":
                self.command = self._reference_command()
            else:
                self.command = self._camera_command()
        except Exception as exc:  # noqa: BLE001 - terminate on any third-party policy exception
            self.reason = f"model_error: {exc}"
            self.command = (0.0, 0.0, 1.0)
        if self.reason:
            return  # terminate the episode; never keep the last valid throttle
        for _ in range(round(self.control_dt / self.kart.dt)):
            s = self.kart.step(*self.command)
            points = self.kart.footprint()
            clearance = float(self.track.distance_to_edge(points[:, 0], points[:, 1]).min())
            self.min_clearance = min(self.min_clearance, clearance)
            self.max_speed = max(self.max_speed, s.v)
            current_s = float(self.track.frenet(s.x, s.y)[0][0])
            delta = (current_s - self.previous_s + self.track.length / 2) % self.track.length - self.track.length / 2
            if abs(delta) > max(2, s.v * self.kart.dt * 3):
                self.reason = "track_projection_discontinuity"
            else:
                self.progress += delta
            self.previous_s = current_s
            if clearance < 0:
                self.violations += 1
                self.reason = "body_outside_track"
            self.log.append(
                {
                    **asdict(s),
                    "steer_cmd": self.command[0],
                    "throttle_cmd": self.command[1],
                    "brake_cmd": self.command[2],
                    "clearance_m": clearance,
                    "progress_m": self.progress,
                }
            )
            if self.reason:
                break

    def frame(self):
        return self.camera.render(self.kart.state)

    def report(self, laps=1):
        completed = max(0, int(self.progress / self.track.length))
        return {
            "track_id": self.track.track_id,
            "policy": self.policy_name,
            "seed": self.seed,
            "physics": "planar_dynamic_bicycle",
            "accuracy_validated": False,
            "geometry_status": self.track.meta.get("geometry_status", "unspecified"),
            "parameter_provenance": self.provenance,
            "completed_laps": completed,
            "requested_laps": laps,
            "passed": completed >= laps and not self.reason and not self.violations,
            "stop_reason": self.reason,
            "sim_time_s": self.kart.state.t,
            "progress_m": self.progress,
            "max_speed_mps": self.max_speed,
            "minimum_body_clearance_m": self.min_clearance if math.isfinite(self.min_clearance) else None,
            "boundary_violations": self.violations,
            "synthetic_pavement_iou": float(np.mean(self.ious)) if self.ious else None,
            "inference_p95_ms": float(np.percentile(self.inference_ms, 95)) if self.inference_ms else None,
            "perception_frames": len(self.ious),
            "timing": "lockstep; measured wall inference time is not injected as latency",
            "vehicle_parameters": asdict(self.vehicle_params),
            "camera_parameters": asdict(self.camera_params),
        }


def evaluate(sim, seconds=180.0, laps=1):
    if not math.isfinite(seconds) or seconds <= 0 or laps < 1:
        raise ValueError("positive duration and at least one lap required")
    while sim.kart.state.t < seconds - 1e-9 and not sim.reason and sim.progress < laps * sim.track.length:
        sim.step()
    if not sim.reason and sim.progress < laps * sim.track.length:
        sim.reason = "time_limit"
    return sim.report(laps)


def heading_error(predicted, measured):
    return np.asarray(wrap_angle(np.asarray(predicted) - np.asarray(measured)))
