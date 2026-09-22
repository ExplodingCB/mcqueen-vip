"""Deterministic flat-ground pinhole RGB and pavement labels for interface tests.

These procedural images are not photorealistic perception acceptance data.
The RGB and label image use identical ray geometry. Class 0 = other, 1 = road.
"""

from __future__ import annotations

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


class TrackCamera:
    def __init__(self, track: Track, params: CameraParams | None = None):
        self.track, self.params = track, params or CameraParams()
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
        edge = self.track.distance_to_edge(x, y)
        asphalt = edge >= 0
        # World-anchored procedural material: motion is consistent across frames.
        texture = np.sin(x * 17.7 + y * 31.1) * np.sin(y * 13.3 - x * 21.5)
        grass = np.column_stack((68 + 9 * texture, 89 + 12 * texture, 52 + 6 * texture))
        grey = 65 + 5 * texture
        pixels = np.where(asphalt[:, None], np.column_stack((grey, grey + 1, grey + 2)), grass)
        haze = np.clip(f / p.far_m, 0, 1)[:, None] * 0.22
        pixels = pixels * (1 - haze) + np.array([145, 169, 177]) * haze
        rgb[self.valid] = np.clip(pixels, 0, 255).astype(np.uint8)
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
