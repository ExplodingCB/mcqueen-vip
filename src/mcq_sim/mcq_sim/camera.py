"""Deterministic flat-ground pinhole RGB and pavement labels.

Two appearances. ``flat`` paints procedural grey asphalt and green grass; the
color-threshold demo is exact on it, so it serves interface tests. ``aerial``
drapes the track's georeferenced orthophoto over the ground plane, so the kart
sees the real surface: the actual asphalt, patches, paint, curbs, run-off and
grass of the track, as photographed from above. Neither is photorealistic
acceptance data: the ground is flat, nothing has height, and the photograph's
lighting is the day it was taken. The RGB and label image use identical ray
geometry. Class 0 = other, 1 = road (the track file's pavement edges).
"""

from __future__ import annotations

import functools
import math
from dataclasses import dataclass

import numpy as np

from mcq_sim.track import Track


@dataclass
class CameraParams:
    width: int = 480
    height: int = 270
    horizontal_fov_deg: float = 100.0
    height_m: float = 0.7
    pitch_down_deg: float = 8.0
    forward_m: float = 0.8
    far_m: float = 90.0

    def __post_init__(self):
        if not (32 <= self.width <= 1920 and 24 <= self.height <= 1080):
            raise ValueError("camera resolution outside supported range")
        if not all(math.isfinite(float(v)) for v in vars(self).values()):
            raise ValueError("camera parameters must be finite")
        if not (10 < self.horizontal_fov_deg < 170 and 0 < self.height_m < 5 and 0 <= self.pitch_down_deg < 60):
            raise ValueError("invalid camera geometry")
        if self.far_m <= 1:
            raise ValueError("far_m must exceed 1 m")


APPEARANCES = ("flat", "aerial")


@functools.lru_cache(maxsize=4)
def _aerial_pyramid(path: str) -> list[np.ndarray]:
    """The orthophoto and its successive 2x2 means, loaded once per file:
    cameras with different mounts on the same track share it."""
    from PIL import Image

    image = np.asarray(Image.open(path).convert("RGB"), dtype=np.float32)
    levels = [image]
    while min(levels[-1].shape[:2]) > 32 and len(levels) < 8:
        a = levels[-1]
        h, w = a.shape[0] // 2 * 2, a.shape[1] // 2 * 2
        levels.append(0.25 * (a[0:h:2, 0:w:2] + a[1:h:2, 0:w:2] + a[0:h:2, 1:w:2] + a[1:h:2, 1:w:2]))
    for level in levels:
        level.flags.writeable = False
    return levels


class TrackCamera:
    def __init__(self, track: Track, params: CameraParams | None = None, appearance: str = "flat"):
        if appearance not in APPEARANCES:
            raise ValueError(f"appearance must be one of {APPEARANCES}")
        if appearance == "aerial" and not track.meta.get("aerial_path"):
            raise ValueError(f"track {track.track_id} has no aerial image to drape")
        self.track, self.params, self.appearance = track, params or CameraParams(), appearance
        self.skip_truth = False  # True for a live demo: skip the scoring mask (see render)
        p = self.params
        self.focal = p.width / (2 * math.tan(math.radians(p.horizontal_fov_deg) / 2))
        u, v = np.meshgrid(np.arange(p.width) + 0.5, np.arange(p.height) + 0.5)
        vertical = (p.height / 2 - v) / self.focal
        pitch = math.radians(p.pitch_down_deg)
        ray_x = math.cos(pitch) + vertical * math.sin(pitch)
        ray_z = -math.sin(pitch) + vertical * math.cos(pitch)
        scale = p.height_m / np.maximum(-ray_z, 1e-9)
        self.forward = ray_x * scale + p.forward_m
        self.left = (p.width / 2 - u) / self.focal * scale
        self.valid = (ray_z < -1e-6) & (self.forward > 0) & (self.forward < p.far_m)
        if appearance == "aerial":
            self._load_aerial()

    def _load_aerial(self):
        """The orthophoto as a mip pyramid, and which level each pixel reads.

        A pixel far ahead covers many photo pixels; sampling the full-size photo
        there aliases into shimmer as the kart moves. Each camera pixel reads
        the level whose texel is closest to its own ground footprint."""
        levels = _aerial_pyramid(self.track.meta["aerial_path"])
        xmin, ymin, xmax, ymax = (float(v) for v in self.track.meta["aerial"]["bounds_enu"])
        self.aerial_bounds = (xmin, ymin, xmax, ymax)
        self.aerial_res = (xmax - xmin) / levels[0].shape[1]
        self.aerial_levels = levels
        # ground footprint of each pixel: the larger of its along- and across-track extent
        along = np.abs(np.gradient(self.forward, axis=0))
        across = np.abs(np.gradient(self.left, axis=1))
        footprint = np.maximum(along, across)
        level = np.log2(np.maximum(footprint / self.aerial_res, 1.0))
        self.aerial_level = np.clip(np.round(level), 0, len(levels) - 1).astype(int)

    def _sample_aerial(self, x, y, level):
        """Bilinear photo color at map points, NaN outside the photo."""
        xmin, ymin, xmax, ymax = self.aerial_bounds
        out = np.full((len(x), 3), np.nan, dtype=np.float32)
        for k in np.unique(level):
            pick = level == k
            img = self.aerial_levels[k]
            res = (xmax - xmin) / img.shape[1]
            u = (x[pick] - xmin) / res - 0.5
            v = (ymax - y[pick]) / res - 0.5
            inside = (u >= 0) & (v >= 0) & (u <= img.shape[1] - 1) & (v <= img.shape[0] - 1)
            u0 = np.clip(np.floor(u).astype(int), 0, img.shape[1] - 2)
            v0 = np.clip(np.floor(v).astype(int), 0, img.shape[0] - 2)
            fu, fv = (u - u0)[:, None], (v - v0)[:, None]
            color = (
                img[v0, u0] * (1 - fu) * (1 - fv)
                + img[v0, u0 + 1] * fu * (1 - fv)
                + img[v0 + 1, u0] * (1 - fu) * fv
                + img[v0 + 1, u0 + 1] * fu * fv
            )
            color[~inside] = np.nan
            out[pick] = color
        return out

    def render(self, state):
        p = self.params
        rgb = np.empty((p.height, p.width, 3), dtype=np.uint8)
        sky = np.linspace(0, 1, p.height)[:, None, None]
        rgb[:] = np.array([110, 155, 184])[None, None, :] + sky * np.array([38, 28, 12])[None, None, :]
        road = np.zeros((p.height, p.width), dtype=bool)
        f, l = self.forward[self.valid], self.left[self.valid]
        c, sn = math.cos(state.yaw), math.sin(state.yaw)
        x = state.x + c * f - sn * l
        y = state.y + sn * f + c * l
        if self.skip_truth and self.appearance == "aerial":
            # A live demo needs the picture, not the scoring mask. Where the photo covers the ground the
            # edge distance is never read, so it is computed only for the pixels the photo does not cover
            # (the procedural fallback) and the returned road mask is left empty.
            photo = self._sample_aerial(x, y, self.aerial_level[self.valid])
            covered = np.isfinite(photo[:, 0])
            asphalt = np.zeros(len(x), dtype=bool)
            if not covered.all():
                asphalt[~covered] = self.track.distance_to_edge(x[~covered], y[~covered]) >= 0
        else:
            photo = None
            asphalt = self.track.distance_to_edge(x, y) >= 0
        # World-anchored procedural material: motion is consistent across frames.
        texture = np.sin(x * 17.7 + y * 31.1) * np.sin(y * 13.3 - x * 21.5)
        grass = np.column_stack((68 + 9 * texture, 89 + 12 * texture, 52 + 6 * texture))
        grey = 65 + 5 * texture
        pixels = np.where(asphalt[:, None], np.column_stack((grey, grey + 1, grey + 2)), grass)
        if self.appearance == "aerial":
            if photo is None:
                photo = self._sample_aerial(x, y, self.aerial_level[self.valid])
            known = np.isfinite(photo[:, 0])
            # The photo is 15 cm a texel, far coarser than a near pixel; a fine
            # world-anchored grain, fading with distance, keeps near ground from
            # reading as a smooth smear and makes motion visible.
            grain = np.sin(x * 211.0 + y * 97.0) * np.sin(y * 173.0 - x * 131.0)
            fade = np.clip(1.0 - f / 15.0, 0.0, 1.0)
            photo *= (1.0 + 0.07 * grain * fade)[:, None]
            pixels[known] = photo[known]  # outside the photo the procedural grass stands in
        haze = np.clip(f / p.far_m, 0, 1)[:, None] * 0.22
        pixels = pixels * (1 - haze) + np.array([145, 169, 177]) * haze
        rgb[self.valid] = np.clip(pixels, 0, 255).astype(np.uint8)
        if not (self.skip_truth and self.appearance == "aerial"):
            road[self.valid] = asphalt
        return rgb, road


def checked_mask(value, shape):
    mask = np.asarray(value)
    if mask.shape != shape or not np.isfinite(mask).all():
        raise ValueError(f"model must return a finite pavement mask with shape {shape}")
    if np.any(mask < 0) or np.any(mask > 1):
        raise ValueError("mask values must be probabilities or binary labels in [0, 1]")
    return mask >= 0.5


class DemoSegmenter:
    """Synthetic color threshold for plumbing checks; NOT a trained model."""

    def predict(self, rgb):
        image = rgb.astype(float)
        return (np.abs(image[..., 1] - image[..., 0]) < 8) & (image[..., 0] < 110)


def mask_iou(prediction, truth):
    union = np.count_nonzero(prediction | truth)
    return float(np.count_nonzero(prediction & truth) / union) if union else 1.0
