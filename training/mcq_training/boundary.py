"""Track edges from a segmentation mask: the reference implementation of what
``mcq_perception`` does in C++ after the network (docs/02-architecture.md
section 10), used here to evaluate models in metres rather than pixels.

Per image row from the bottom up to the horizon, the leftmost and rightmost
pavement pixels of the main pavement blob are the left and right edges. Each
edge pixel is mapped onto the ground plane with the camera model, giving edge
polylines in ``base_link``. A row whose pavement touches the image border has
not observed that edge, so its confidence is 0 and the metric skips it.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import ndimage

from mcq_training.camera import CameraModel
from mcq_training.labels import PAVEMENT


@dataclass
class Bounds:
    rows: np.ndarray  # image rows, bottom first
    left_uv: np.ndarray  # (n, 2)
    right_uv: np.ndarray
    left: np.ndarray  # (n, 3) base_link, z = 0
    right: np.ndarray
    left_confidence: np.ndarray  # 1 where the edge was observed, 0 at the image border
    right_confidence: np.ndarray

    def __len__(self) -> int:
        return len(self.rows)


def main_pavement(mask: np.ndarray) -> np.ndarray:
    """The largest 8-connected pavement component; drops islands and noise."""
    lab, n = ndimage.label(mask, structure=np.ones((3, 3), dtype=int))
    if n <= 1:
        return mask.astype(bool)
    sizes = ndimage.sum(mask, lab, index=np.arange(1, n + 1))
    return lab == (1 + int(np.argmax(sizes)))


def extract_bounds(label: np.ndarray, cam: CameraModel, row_step: int = 1, min_width_px: int = 2) -> Bounds:
    """Edge polylines from a label or prediction map (class indices, see labels.py)."""
    if label.shape != (cam.height, cam.width):
        raise ValueError(f"label {label.shape} does not match camera {(cam.height, cam.width)}")
    pav = main_pavement(label == PAVEMENT)
    top = max(int(np.floor(cam.horizon_row())) + 1, 0)
    rows, lu, ru = [], [], []
    for v in range(cam.height - 1, top - 1, -row_step):
        cols = np.flatnonzero(pav[v])
        if len(cols) < min_width_px:
            continue
        rows.append(v)
        lu.append(cols[0])
        ru.append(cols[-1])
    rows = np.asarray(rows, dtype=int)
    if len(rows) == 0:
        empty = np.zeros((0, 3))
        return Bounds(rows, np.zeros((0, 2)), np.zeros((0, 2)), empty, empty, np.zeros(0), np.zeros(0))
    lu, ru = np.asarray(lu, dtype=float), np.asarray(ru, dtype=float)
    left_uv = np.column_stack([lu, rows.astype(float)])
    right_uv = np.column_stack([ru, rows.astype(float)])
    left, ok_l = cam.ground_from_pixels(left_uv)
    right, ok_r = cam.ground_from_pixels(right_uv)
    left_conf = ((lu > 0) & ok_l).astype(float)
    right_conf = ((ru < cam.width - 1) & ok_r).astype(float)
    return Bounds(rows, left_uv, right_uv, left, right, left_conf, right_conf)
