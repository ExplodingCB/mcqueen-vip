"""Turning what the camera sees into the edges the planner drives between.

The architecture puts a segmentation model between the camera and the planner:
the model says which pixels are pavement, and this module turns that mask into
left and right edge polylines in the kart's frame (x forward, y left), which the
planner's BOUNDARY mode consumes. Three sources feed it the same way so that
their results can be compared:

  oracle       the true edges, standing in for perception that works perfectly.
               Separates the planner's own limits from the model's.
  camera-demo  the color threshold, which exercises the interface only.
  module:factory  any model with ``predict(rgb)``, including a trained one.

The camera renders from the true pose, as a camera would see the world. What the
model returns is all the driving stack learns about the boundaries; it never
sees the map. Scoring reads the true mask separately.
"""

from __future__ import annotations

import time

import numpy as np

from mcq_sim.camera import TrackCamera, checked_mask, mask_iou
from mcq_sim.track import Track

MIN_ROWS = 4  # fewer edge samples than this is not a drivable path
NEAR_M, FAR_M, SPACING_M = 2.0, 25.0, 1.0


def edges_from_mask(mask: np.ndarray, camera: TrackCamera, near=NEAR_M, far=FAR_M, spacing=SPACING_M):
    """Left and right edge polylines in the base_link frame from a pavement mask.

    For each forward distance along the camera's center column, take the
    contiguous run of pavement that contains the center (or the nearest one) and
    read its two ends through the ground-plane projection. An end at the image
    border is kept: the true edge is at least that far out, so the corridor it
    describes is a safe underestimate. Returns None when too few rows have a
    path, which the caller treats as no drivable path.
    """
    p = camera.params
    col = p.width // 2
    rows = np.flatnonzero(camera.valid[:, col])
    if len(rows) == 0:
        return None
    ahead = camera.forward[rows, col]
    left, right, seen = [], [], []
    for target in np.arange(near, far + 1e-9, spacing):
        row = int(rows[np.argmin(np.abs(ahead - target))])
        if abs(float(camera.forward[row, col]) - target) > 0.6 * max(spacing, 0.25 * target):
            continue  # the pixel rows are too coarse this far out to hit the target
        indices = np.flatnonzero(mask[row] & camera.valid[row])
        if len(indices) < 4:
            continue
        groups = np.split(indices, np.where(np.diff(indices) > 1)[0] + 1)
        groups = [g for g in groups if len(g) >= 4]
        if not groups:
            continue
        group = min(groups, key=lambda g: 0 if g[0] <= col <= g[-1] else min(abs(g[0] - col), abs(g[-1] - col)))
        a, b = int(group[0]), int(group[-1])  # a is the leftmost column, so the larger `left`
        left.append((float(camera.forward[row, a]), float(camera.left[row, a])))
        right.append((float(camera.forward[row, b]), float(camera.left[row, b])))
        seen.append(target)
    if len(left) < MIN_ROWS:
        return None
    left, right = np.asarray(left), np.asarray(right)
    # The planner needs to see a little behind the kart, where the camera cannot.
    return _extend_back(left), _extend_back(right)


def _extend_back(edge: np.ndarray, back=2.0):
    """Hold the first lateral position back to ``back`` metres behind the kart.

    Extrapolating along the first segment is wrong where the edge fans out
    close to the camera: a steep first segment carries the line across the
    kart. Holding it is the conservative choice, since the corridor behind the
    kart is at least as wide as it was where the camera last saw it.
    """
    if edge[0, 0] <= -back:
        return edge
    return np.vstack([[-back, edge[0, 1]], edge])


class OraclePerception:
    """The true edges from the true pose, in place of a model."""

    name = "oracle"

    def __init__(self, track: Track, range_m: float = 25.0):
        self.track, self.range_m = track, range_m
        self.ious: list[float] = []
        self.inference_ms: list[float] = []

    def reset(self, seed=0):
        self.ious, self.inference_ms = [], []

    def __call__(self, state):
        return self.track.bounds_ahead(state.x, state.y, state.yaw, self.range_m, 1.0)


class CameraPerception:
    """Camera frame, model, mask, edges. Records mask IoU against the true mask
    and the inference wall time; neither is visible to the driving stack."""

    def __init__(self, camera: TrackCamera, segmenter, name="camera"):
        self.camera, self.segmenter, self.name = camera, segmenter, name
        self.ious: list[float] = []
        self.inference_ms: list[float] = []
        self.last_rgb = self.last_truth = self.last_prediction = None

    def reset(self, seed=0):
        self.ious, self.inference_ms = [], []
        if callable(getattr(self.segmenter, "reset", None)):
            self.segmenter.reset(seed)

    def __call__(self, state):
        rgb, truth = self.camera.render(state)
        started = time.perf_counter()
        prediction = checked_mask(self.segmenter.predict(rgb.copy()), truth.shape)
        self.inference_ms.append((time.perf_counter() - started) * 1000)
        self.ious.append(mask_iou(prediction, truth))
        self.last_rgb, self.last_truth, self.last_prediction = rgb, truth, prediction
        return edges_from_mask(prediction, self.camera)
