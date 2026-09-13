"""Pinhole camera between ``base_link`` and the rectified image.

Frames follow REP 103: ``base_link`` is x forward, y left, z up at the rear
axle on the ground. The camera body frame is aligned with ``base_link`` when
yaw, pitch and roll are zero; pitch is positive when the camera looks down.
The optical frame is z forward, x right, y down, which is what the intrinsics
apply to. Images are assumed rectified, so there is no distortion term: the
perception node rectifies before inference and training frames from the kart
are rectified at extraction.

The same numbers live in the URDF (``camera_front``) and the camera_info
topic on the kart; ``training/configs/camera_default.yaml`` is the placeholder
until the camera is mounted and calibrated.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
import yaml

# Body (x forward, y left, z up) to optical (x right, y down, z forward).
R_BODY_TO_OPTICAL = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])


def rot_x(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def map_to_base(points: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    """Map-frame (n, 2) points into ``base_link`` given the base_link pose in map."""
    points = np.asarray(points, dtype=float)
    c, s = np.cos(yaw), np.sin(yaw)
    dx, dy = points[:, 0] - x, points[:, 1] - y
    return np.column_stack([c * dx + s * dy, -s * dx + c * dy])


@dataclass
class CameraModel:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    x: float = 0.0  # camera position in base_link, m
    y: float = 0.0
    z: float = 0.75
    yaw: float = 0.0  # rad
    pitch: float = 0.0  # rad, positive looking down
    roll: float = 0.0  # rad

    # ------------------------------------------------------------- builders
    @classmethod
    def from_fov(cls, width: int, height: int, hfov_deg: float, **pose) -> CameraModel:
        """Square pixels, principal point at the image center, horizontal field of view given."""
        fx = 0.5 * width / np.tan(np.radians(hfov_deg) / 2.0)
        return cls(width, height, fx, fx, width / 2.0, height / 2.0, **pose)

    @classmethod
    def from_dict(cls, d: dict) -> CameraModel:
        d = dict(d)
        for key in ("yaw", "pitch", "roll"):
            if f"{key}_deg" in d:
                d[key] = float(np.radians(d.pop(f"{key}_deg")))
        if "hfov_deg" in d:
            hfov = d.pop("hfov_deg")
            for k in ("fx", "fy", "cx", "cy"):
                d.pop(k, None)
            return cls.from_fov(int(d.pop("width")), int(d.pop("height")), float(hfov), **d)
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})

    @classmethod
    def load(cls, path: str | Path) -> CameraModel:
        with open(path) as f:
            return cls.from_dict(yaml.safe_load(f))

    def save(self, path: str | Path) -> None:
        plain = {k: (int(v) if k in ("width", "height") else float(v)) for k, v in asdict(self).items()}
        with open(path, "w") as f:
            yaml.safe_dump(plain, f, sort_keys=False)

    def scaled(self, width: int, height: int) -> CameraModel:
        """The same camera after the image is resized to width x height."""
        sx, sy = width / self.width, height / self.height
        return CameraModel(
            width,
            height,
            self.fx * sx,
            self.fy * sy,
            self.cx * sx,
            self.cy * sy,
            self.x,
            self.y,
            self.z,
            self.yaw,
            self.pitch,
            self.roll,
        )

    # ------------------------------------------------------------ transforms
    @property
    def rotation(self) -> np.ndarray:
        """Camera body axes expressed in ``base_link`` (R_base_from_body)."""
        return rot_z(self.yaw) @ rot_y(self.pitch) @ rot_x(self.roll)

    @property
    def position(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z], dtype=float)

    @property
    def intrinsics(self) -> np.ndarray:
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])

    def base_to_optical(self, pts: np.ndarray) -> np.ndarray:
        pts = np.atleast_2d(np.asarray(pts, dtype=float))
        body = (pts - self.position) @ self.rotation  # R^T applied row-wise
        return body @ R_BODY_TO_OPTICAL.T

    def optical_to_base(self, pts: np.ndarray) -> np.ndarray:
        pts = np.atleast_2d(np.asarray(pts, dtype=float))
        body = pts @ R_BODY_TO_OPTICAL  # R_BODY_TO_OPTICAL is orthonormal: inverse is the transpose
        return body @ self.rotation.T + self.position

    def project_optical(self, pts_opt: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(u, v) and depth for optical-frame points. Depth <= 0 is behind the camera;
        the caller clips before calling this or masks with the returned depth."""
        z = pts_opt[:, 2]
        safe = np.where(np.abs(z) < 1e-9, 1e-9, z)
        u = self.fx * pts_opt[:, 0] / safe + self.cx
        v = self.fy * pts_opt[:, 1] / safe + self.cy
        return np.column_stack([u, v]), z

    def project(self, pts_base: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """``base_link`` (n, 3) points to pixel coordinates and depth."""
        return self.project_optical(self.base_to_optical(pts_base))

    def rays(self, uv: np.ndarray) -> np.ndarray:
        """Unit-free ray directions in ``base_link`` for pixel coordinates (n, 2)."""
        uv = np.atleast_2d(np.asarray(uv, dtype=float))
        d_opt = np.column_stack([(uv[:, 0] - self.cx) / self.fx, (uv[:, 1] - self.cy) / self.fy, np.ones(len(uv))])
        return (d_opt @ R_BODY_TO_OPTICAL) @ self.rotation.T

    def ground_from_pixels(self, uv: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Inverse perspective mapping onto the ground plane z = 0 of ``base_link``.
        Returns (n, 3) points and a validity mask (ray hits the ground ahead of the camera)."""
        d = self.rays(uv)
        dz = d[:, 2]
        valid = dz < -1e-9
        t = np.where(valid, -self.z / np.where(valid, dz, -1.0), 0.0)
        pts = self.position + t[:, None] * d
        pts[~valid] = np.nan
        return pts, valid

    def horizon_row(self) -> float:
        """Image row of the ground horizon below the principal column."""
        fwd = self.rotation @ np.array([1.0, 0.0, 0.0])
        fwd[2] = 0.0
        n = np.linalg.norm(fwd)
        if n < 1e-9:  # camera pointing straight down or up
            return -np.inf if self.pitch > 0 else np.inf
        far = self.position + 1e7 * fwd / n
        uv, _ = self.project(far[None, :])
        return float(uv[0, 1])
