"""Camera frames labeled with what an expert driver would do: the racing line
ahead of the kart and the speed to drive it at, plus the pavement mask.

The expert is privileged: it knows the map. Its line is the minimum-curvature
line through the pavement (mcq_sim.reference, the same line the map-driven
stack follows), and its speed is the fastest the stack's own limits allow on
that line: lateral acceleration in the corners, braking into them and
acceleration out of them, up to the kart's top speed. The model sees only the
camera and learns to say where that line is and how fast to go.

Labels are defined for any pose, not only poses on the line: the waypoints are
the racing line itself, in the kart's frame, from the point on the line nearest
the kart. A kart that is off the line therefore sees waypoints that lead back
onto it, which is the recovery behavior a model trained only on perfect laps
never learns. Poses are sampled around the line with lateral and heading
error for that reason.

Frames come from the simulator camera with the track's orthophoto draped over
the ground (mcq_sim.camera, appearance "aerial"), with the camera mount and the
image appearance jittered so the model does not depend on one exact mount or
one photograph's colors.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np

from mcq_sim import ccontrol
from mcq_sim.camera import CameraParams, TrackCamera
from mcq_sim.params import load_params
from mcq_sim.reference import smooth_reference
from mcq_sim.track import Track

WAYPOINT_SPACING_M = 2.0
N_WAYPOINTS = 12  # 2 to 24 m ahead along the line, about what the camera resolves
HALF_WIDTH = 0.8  # the kart's half width, as the stack uses it
LINE_MARGIN = 0.5  # beyond the half width, as the stack's reference keeps
TOP_SPEED = 23.0  # m/s, the kart model's top speed (52 mph)


@dataclass
class Expert:
    """The racing line of one track and the speed along it, on a fine grid."""

    track: Track
    line: Track
    s: np.ndarray
    x: np.ndarray
    y: np.ndarray
    v: np.ndarray
    kappa: np.ndarray
    corner_weight: float = 0.0  # poses in corners drawn up to (1 + this) times as often as on straights

    @classmethod
    def for_track(cls, track: Track, params=None) -> Expert:
        p = (params or load_params()).planner
        line = smooth_reference(track, HALF_WIDTH + LINE_MARGIN)
        s = np.arange(0.0, line.length, 0.25)
        x, y, _ = line.cartesian(s, np.zeros_like(s))
        kappa = line.curvature_at(s) if hasattr(line, "curvature_at") else _curvature(x, y)
        # Race pace on a closed loop: profile three laps and keep the middle one,
        # so braking for turn one is felt at the end of the lap before it.
        sp = ccontrol.SpeedProfileParams(
            float(p["a_lat_max"]), float(p["a_long_max"]), float(p["a_brake_max"]), TOP_SPEED, float(p["v_min"])
        )
        s3 = np.concatenate([s, s + line.length, s + 2 * line.length])
        k3 = np.tile(kappa, 3)
        v3 = ccontrol.speed_profile(sp, s3, k3, None, TOP_SPEED, TOP_SPEED)
        v = v3[len(s) : 2 * len(s)]
        return cls(track, line, s, x, y, v, np.abs(kappa))

    @classmethod
    def from_line(
        cls,
        track: Track,
        line: Track,
        a_lat: float,
        a_long: float,
        a_brake: float,
        top_speed: float = TOP_SPEED,
        v_min: float = 0.5,
    ) -> Expert:
        """An expert on a given line, with its own cornering and braking limits.
        ``for_track`` is this with the minimum-curvature line and the stack's
        default limits; the lap-time optimizer (laptime_opt.py) calls it with
        lines and limits it searched for."""
        s = np.arange(0.0, line.length, 0.25)
        x, y, _ = line.cartesian(s, np.zeros_like(s))
        kappa = _curvature(x, y)
        sp = ccontrol.SpeedProfileParams(a_lat, a_long, a_brake, top_speed, v_min)
        s3 = np.concatenate([s, s + line.length, s + 2 * line.length])
        v3 = ccontrol.speed_profile(sp, s3, np.tile(kappa, 3), None, top_speed, top_speed)
        return cls(track, line, s, x, y, v3[len(s) : 2 * len(s)], np.abs(kappa))

    def labels(self, x: float, y: float, yaw: float):
        """Waypoints (N, 2) in the kart frame and the target speed at each, from
        the point on the line nearest the kart."""
        s0 = float(self.line.frenet(np.array([x]), np.array([y]))[0][0])
        ahead = s0 + WAYPOINT_SPACING_M * np.arange(1, N_WAYPOINTS + 1)
        wrapped = np.mod(ahead, self.line.length)
        wx = np.interp(wrapped, self.s, self.x, period=self.line.length)
        wy = np.interp(wrapped, self.s, self.y, period=self.line.length)
        c, sn = math.cos(yaw), math.sin(yaw)
        dx, dy = wx - x, wy - y
        points = np.column_stack([c * dx + sn * dy, -sn * dx + c * dy])
        speeds = np.interp(wrapped, self.s, self.v, period=self.line.length)
        return points.astype(np.float32), speeds.astype(np.float32)

    def sample_pose(self, rng: np.random.Generator):
        """A pose near the racing line: lateral error up to what the pavement
        allows with the kart's width, heading error mostly small, sometimes not."""
        if self.corner_weight > 0:
            weight = 1.0 + self.corner_weight * self.kappa / max(float(self.kappa.max()), 1e-9)
            i = int(rng.choice(len(self.s), p=weight / weight.sum()))
            s = float(self.s[i] + rng.uniform(0, self.s[1] - self.s[0]))
        else:
            s = float(rng.uniform(0, self.line.length))
        wl, wr = self.line.width_at(np.array([s]))
        room_left, room_right = float(wl[0]) - HALF_WIDTH - 0.1, float(wr[0]) - HALF_WIDTH - 0.1
        d = float(np.clip(rng.normal(0.0, 0.7), -max(room_right, 0.0), max(room_left, 0.0)))
        px, py, heading = self.line.cartesian(np.array([s]), np.array([d]))
        spread = math.radians(6.0) if rng.random() < 0.8 else math.radians(18.0)
        yaw = float(heading[0]) + float(rng.normal(0.0, spread))
        return float(px[0]), float(py[0]), yaw


def _curvature(x, y):
    dx, dy = np.gradient(x), np.gradient(y)
    ddx, ddy = np.gradient(dx), np.gradient(dy)
    return (dx * ddy - dy * ddx) / np.maximum((dx * dx + dy * dy) ** 1.5, 1e-9)


def jittered_camera(rng: np.random.Generator) -> CameraParams:
    """The nominal mount with a few centimetres and degrees of play."""
    nominal = CameraParams()
    return CameraParams(
        width=nominal.width,
        height=nominal.height,
        horizontal_fov_deg=float(rng.uniform(92, 108)),
        height_m=float(rng.uniform(0.6, 0.8)),
        pitch_down_deg=float(rng.uniform(5.0, 11.0)),
        forward_m=nominal.forward_m,
        far_m=nominal.far_m,
    )


def appearance_jitter(rgb: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    """Exposure, white balance, contrast, haze and sensor noise on one frame."""
    img = rgb.astype(np.float32)
    img *= rng.uniform(0.65, 1.35)
    img *= rng.uniform(0.88, 1.12, size=3)[None, None, :]
    mean = img.mean()
    img = (img - mean) * rng.uniform(0.75, 1.25) + mean
    if rng.random() < 0.3:
        img = img * 0.8 + 0.2 * np.array([200, 205, 210], dtype=np.float32)
    img += rng.normal(0, rng.uniform(1, 6), img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def half(rgb: np.ndarray) -> np.ndarray:
    """480 x 270 to the 240 x 135 the model reads, as the segmenter does."""
    h, w = rgb.shape[:2]
    return rgb.reshape(h // 2, 2, w // 2, 2, -1).mean((1, 3)).astype(rgb.dtype)


def generate(track: Track, expert: Expert, n_frames: int, seed: int, jitter=True):
    """(rgb uint8 N x 135 x 240 x 3, mask bool N x 135 x 240, waypoints N x K x 2, speeds N x K)."""
    rng = np.random.default_rng(seed)
    cameras = {}
    rgbs, masks, points, speeds = [], [], [], []
    for _ in range(n_frames):
        params = jittered_camera(rng) if jitter else CameraParams()
        key = (round(params.horizontal_fov_deg), round(params.height_m, 2), round(params.pitch_down_deg, 1))
        if key not in cameras:  # building a camera is the expensive part; reuse near-identical mounts
            cameras[key] = TrackCamera(track, params, appearance="aerial")
            if len(cameras) > 64:
                cameras.pop(next(iter(cameras)))
        camera = cameras[key]
        x, y, yaw = expert.sample_pose(rng)
        rgb, mask = camera.render(SimpleNamespace(x=x, y=y, yaw=yaw))
        if jitter:
            rgb = appearance_jitter(rgb, rng)
        wp, v = expert.labels(x, y, yaw)
        rgbs.append(half(rgb))
        masks.append(half(mask[..., None].astype(np.float32))[..., 0] >= 0.5)
        points.append(wp)
        speeds.append(v)
    return np.stack(rgbs), np.stack(masks), np.stack(points), np.stack(speeds)
