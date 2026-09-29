"""Synthetic camera frames with pavement labels, for the first training runs.

The simulator's own camera draws flat grey asphalt beside flat green grass, which
a color threshold separates perfectly. A network trained on that learns to be a
color threshold. This generator keeps the simulator's exact ground-plane
geometry, so labels are exact, and randomizes everything about appearance that a
real camera would not hold constant: surface colors and patchiness, shadows,
edge paint, exposure, white balance, haze, sky, camera height, pitch and field of
view, sensor noise and blur.

It runs over many procedurally generated closed tracks, so the network cannot
memorize one layout. The Purdue track and the synthetic oval are never used for
training; they are the held-out set.

What this does and does not show. A model that segments these frames has learned
geometry-consistent pavement under varied lighting and surfaces. It has not seen
a real photograph, and the acceptance test in docs/08-training-data.md section 7
is held-out real footage. Treat synthetic IoU as a check that the pipeline and
the model can learn the task, and as pretraining, never as the result.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from mcq_sim.camera import CameraParams, TrackCamera
from mcq_sim.track import Track

HALF_WIDTH, HALF_HEIGHT = 240, 135  # training resolution: half of the 480 x 270 the kart's camera delivers


# ---------------------------------------------------------------------- tracks


def random_track(rng: np.random.Generator, track_id: str = "random") -> Track:
    """A closed track with varied curvature and width, no tighter than a kart can turn.

    A radial perturbation of a circle gives smooth, non-self-intersecting loops.
    Width varies slowly along the lap. Layouts that need a tighter radius than
    2.5 m at the centerline are rejected and redrawn.
    """
    for _ in range(200):
        radius = rng.uniform(25.0, 60.0)
        n_points = 720
        theta = np.linspace(0, 2 * np.pi, n_points, endpoint=False)
        r = np.ones(n_points)
        for k in range(2, 7):
            r += rng.uniform(0.0, 0.32 / k) * np.cos(k * theta + rng.uniform(0, 2 * np.pi))
        stretch = rng.uniform(0.6, 1.6)
        x = radius * r * np.cos(theta) * stretch
        y = radius * r * np.sin(theta)
        width = rng.uniform(3.6, 6.5) * (1.0 + 0.18 * np.sin(rng.integers(1, 4) * theta + rng.uniform(0, 6.28)))
        track = Track(x, y, width / 2 * rng.uniform(0.8, 1.2), width / 2, closed=True, track_id=track_id)
        # Uniform spacing keeps the second-difference curvature honest.
        if np.abs(track.kappa).max() < 1 / 2.5 and 150 < track.length < 450:
            return track
    raise RuntimeError("could not draw a drivable random track")


# ------------------------------------------------------------------ appearance


def _smooth_noise(x, y, rng, octaves=3, base=0.05):
    """World-anchored low-frequency noise in roughly [-1, 1]."""
    out = np.zeros_like(x)
    for o in range(octaves):
        freq = base * 2.3**o
        a, b, c, d = rng.uniform(0, 6.28, 4)
        ang = rng.uniform(0, 6.28)
        u = x * math.cos(ang) + y * math.sin(ang)
        v = -x * math.sin(ang) + y * math.cos(ang)
        out += np.sin(u * freq * 6.28 + a) * np.sin(v * freq * 6.28 * 1.3 + b) / (o + 1)
    return out / 1.6


@dataclass
class Look:
    """Everything about a frame's appearance, drawn once per frame."""

    asphalt: np.ndarray
    asphalt_tint: np.ndarray
    grass: np.ndarray
    grass_texture: float
    asphalt_texture: float
    patchiness: float
    paint: np.ndarray | None  # colour of an edge line, or None
    paint_width: float
    shadow: float
    shadow_scale: float
    gain: float
    gamma: float
    balance: np.ndarray
    haze: float
    haze_color: np.ndarray
    sky_top: np.ndarray
    sky_bottom: np.ndarray
    noise: float
    blur: bool
    vignette: float
    camera: CameraParams = field(default_factory=CameraParams)

    @classmethod
    def draw(cls, rng: np.random.Generator) -> Look:
        grass_palettes = [(68, 89, 52), (95, 110, 55), (120, 112, 72), (105, 88, 68), (58, 78, 48), (135, 125, 90)]
        base = np.array(grass_palettes[rng.integers(len(grass_palettes))], dtype=float)
        paint = None
        if rng.random() < 0.4:
            paint = np.array([230, 230, 225.0] if rng.random() < 0.7 else [225, 190, 60.0])
        return cls(
            asphalt=np.full(3, rng.uniform(42, 125)),
            asphalt_tint=rng.normal(0, 4, 3),
            grass=np.clip(base + rng.normal(0, 8, 3), 20, 200),
            grass_texture=rng.uniform(4, 20),
            asphalt_texture=rng.uniform(2, 10),
            patchiness=rng.uniform(0, 18),
            paint=paint,
            paint_width=rng.uniform(0.08, 0.25),
            shadow=rng.uniform(0, 0.55) if rng.random() < 0.6 else 0.0,
            shadow_scale=rng.uniform(0.02, 0.09),
            gain=rng.uniform(0.55, 1.45),
            gamma=rng.uniform(0.8, 1.25),
            balance=rng.normal(1.0, 0.07, 3),
            haze=rng.uniform(0.05, 0.4),
            haze_color=np.array([145, 169, 177.0]) + rng.normal(0, 14, 3),
            sky_top=np.array([90, 130, 180.0]) + rng.normal(0, 25, 3),
            sky_bottom=np.array([170, 190, 205.0]) + rng.normal(0, 20, 3),
            noise=rng.uniform(0, 7),
            blur=rng.random() < 0.3,
            vignette=rng.uniform(0, 0.35),
            camera=CameraParams(
                width=HALF_WIDTH,
                height=HALF_HEIGHT,
                horizontal_fov_deg=100.0 + rng.uniform(-8, 8),
                height_m=0.7 + rng.uniform(-0.12, 0.18),
                pitch_down_deg=8.0 + rng.uniform(-4, 4),
                forward_m=0.8 + rng.uniform(-0.2, 0.2),
                far_m=90.0,
            ),
        )


def render(track: Track, x: float, y: float, yaw: float, look: Look, rng: np.random.Generator):
    """One frame and its pavement mask at 240 x 135. Returns (rgb uint8, mask bool)."""
    cam = TrackCamera(track, look.camera)
    p = look.camera
    h, w = p.height, p.width
    rgb = np.empty((h, w, 3), dtype=float)
    ramp = np.linspace(0, 1, h)[:, None, None]
    rgb[:] = look.sky_top[None, None, :] * (1 - ramp) + look.sky_bottom[None, None, :] * ramp
    mask = np.zeros((h, w), dtype=bool)

    f, l = cam.forward[cam.valid], cam.left[cam.valid]
    c, s = math.cos(yaw), math.sin(yaw)
    wx, wy = x + c * f - s * l, y + s * f + c * l
    edge = track.distance_to_edge(wx, wy)
    asphalt = edge >= 0

    tex = np.sin(wx * 17.7 + wy * 31.1) * np.sin(wy * 13.3 - wx * 21.5)
    fine = rng.normal(0, 1, wx.shape)
    patch = _smooth_noise(wx, wy, rng, octaves=3, base=0.07)
    grey = look.asphalt[None, :] + look.asphalt_tint[None, :]
    asphalt_px = grey + (look.asphalt_texture * (0.6 * tex + 0.4 * fine) + look.patchiness * patch)[:, None]
    grass_px = (
        look.grass[None, :] + (look.grass_texture * (0.5 * tex + 0.5 * fine) + 0.6 * look.patchiness * patch)[:, None]
    )
    pixels = np.where(asphalt[:, None], asphalt_px, grass_px)
    if look.paint is not None:
        line = (np.abs(edge) < look.paint_width) | (np.abs(edge - 0.05) < look.paint_width * 0.5)
        pixels = np.where(line[:, None], look.paint[None, :], pixels)
    if look.shadow > 0:
        shade = _smooth_noise(wx, wy, rng, octaves=2, base=look.shadow_scale)
        pixels = pixels * (1 - look.shadow * np.clip(shade * 3.0, 0, 1))[:, None]
    haze = np.clip(f / p.far_m, 0, 1)[:, None] * look.haze
    pixels = pixels * (1 - haze) + look.haze_color[None, :] * haze
    rgb[cam.valid] = pixels
    mask[cam.valid] = asphalt

    # Camera and sensor: white balance, exposure, gamma, vignette, blur, noise.
    rgb *= look.balance[None, None, :]
    rgb = 255.0 * np.clip(rgb * look.gain / 255.0, 0, 1) ** look.gamma
    if look.vignette > 0:
        yy, xx = np.mgrid[0:h, 0:w]
        rr = np.hypot((xx - w / 2) / (w / 2), (yy - h / 2) / (h / 2))
        rgb *= (1 - look.vignette * np.clip(rr - 0.4, 0, 1) ** 2)[:, :, None]
    if look.blur:
        k = np.array([0.25, 0.5, 0.25])
        rgb = np.apply_along_axis(lambda v: np.convolve(np.pad(v, 1, mode="edge"), k, mode="valid"), 1, rgb)
    if look.noise > 0:
        rgb += rng.normal(0, look.noise, rgb.shape)
    return np.clip(rgb, 0, 255).astype(np.uint8), mask


# ------------------------------------------------------------------------ poses


def sample_pose(track: Track, rng: np.random.Generator):
    """A kart pose: mostly on the pavement, some near the edges and some on the grass."""
    s = rng.uniform(0, track.length)
    left, right = (float(v[0]) for v in track.width_at(np.array([s])))
    kind = rng.random()
    if kind < 0.7:
        d = rng.uniform(-right + 0.8, left - 0.8)
    elif kind < 0.9:
        side = rng.choice([-1, 1])
        d = side * (left if side > 0 else right) + rng.uniform(-1.2, 1.2) * -side
    else:
        side = rng.choice([-1, 1])
        d = side * ((left if side > 0 else right) + rng.uniform(0.3, 3.0))
    heading = float(track.heading_at(np.array([s]))[0])
    yaw = heading + rng.normal(0, 0.2 if rng.random() < 0.85 else 0.7)
    px, py, _ = track.cartesian(np.array([s]), np.array([d]))
    return float(px[0]), float(py[0]), float(yaw)


def generate(n_tracks: int, frames_per_track: int, seed: int):
    """Frames and masks from ``n_tracks`` random tracks. Returns (rgb, mask, track_index)."""
    rng = np.random.default_rng(seed)
    rgbs, masks, owners = [], [], []
    for t in range(n_tracks):
        track = random_track(rng, f"random_{seed}_{t}")
        for _ in range(frames_per_track):
            x, y, yaw = sample_pose(track, rng)
            rgb, mask = render(track, x, y, yaw, Look.draw(rng), rng)
            rgbs.append(rgb)
            masks.append(mask)
            owners.append(t)
    return np.stack(rgbs), np.stack(masks), np.asarray(owners)


def generate_on(track: Track, n_frames: int, seed: int):
    """Frames from one given track, the held-out evaluation."""
    rng = np.random.default_rng(seed)
    rgbs, masks = [], []
    for _ in range(n_frames):
        x, y, yaw = sample_pose(track, rng)
        rgb, mask = render(track, x, y, yaw, Look.draw(rng), rng)
        rgbs.append(rgb)
        masks.append(mask)
    return np.stack(rgbs), np.stack(masks)
