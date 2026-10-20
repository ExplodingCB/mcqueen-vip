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

import math
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


class CorridorTracker:
    """Left and right edges from a pavement mask by marching along the corridor
    on the ground plane, with what the camera saw in the last few seconds.

    ``edges_from_mask`` reads each image row on its own, which fails in two
    places the Purdue lap has. Beside and just ahead of the kart the corridor
    runs off the sides of the image, so the edge there can only be read as "at
    least this far out", and the planner, which needs the corridor around the
    kart, is left with a strip too narrow to steer through. In a hairpin one
    image row crosses both legs, so the row's pavement run is not the corridor.

    Here the mask is resampled onto a top-down grid around the kart. Cells the
    camera cannot see now are filled from recent frames, each placed with the
    estimated pose at its time (never the truth), so the pavement beside and
    behind the kart is the pavement it drove past. The edges are then found by
    walking the corridor: from the kart, step along the corridor's direction,
    cast a ray to each side until it leaves the pavement, re-center, repeat. A
    ray that runs into ground nobody has seen stops at the last known pavement,
    the conservative reading. The result is the paired, along-corridor edge
    polylines the planner's BOUNDARY mode takes, from a few metres behind the
    kart to about 25 m ahead.
    """

    def __init__(
        self,
        camera: TrackCamera,
        resolution=0.1,
        back=5.0,
        ahead=FAR_M,
        half_width=16.0,
        step=0.5,
        memory_s=3.0,
        keyframe_s=0.25,
        first_view=3.0,
        trusted_range=12.0,
    ):
        self.camera, self.resolution, self.back, self.ahead = camera, resolution, back, ahead
        self.step, self.memory_s, self.keyframe_s, self.first_view = step, memory_s, keyframe_s, first_view
        self.trusted_range = trusted_range
        self.xs = np.arange(-back - 1.0, ahead + 2.0, resolution)
        self.ys = np.arange(-half_width, half_width + 1e-9, resolution)
        self.gx, self.gy = np.meshgrid(self.xs, self.ys, indexing="ij")
        self.reset()

    def reset(self, seed=0):
        self.frames: list[tuple[float, float, float, float, np.ndarray]] = []  # t, x, y, yaw, mask

    # ------------------------------------------------------------ geometry
    def _pixels(self, fx: np.ndarray, ly: np.ndarray):
        """Pixel row and column of ground points in the camera's base_link
        frame, and whether the camera sees them. The inverse of the
        per-pixel ground projection in ``TrackCamera``."""
        p, cam = self.camera.params, self.camera
        pitch = math.radians(p.pitch_down_deg)
        ahead = fx - p.forward_m
        denominator = ahead * math.cos(pitch) + p.height_m * math.sin(pitch)
        vertical = (ahead * math.sin(pitch) - p.height_m * math.cos(pitch)) / np.where(
            denominator > 1e-6, denominator, np.nan
        )
        scale = p.height_m / (math.sin(pitch) - vertical * math.cos(pitch))
        v = p.height / 2 - vertical * cam.focal
        u = p.width / 2 - ly * cam.focal / scale
        with np.errstate(invalid="ignore"):
            seen = (denominator > 1e-6) & (u >= 0) & (u < p.width) & (v >= 0) & (v < p.height) & (fx < p.far_m)
        row = np.clip(np.nan_to_num(v), 0, p.height - 1).astype(int)
        col = np.clip(np.nan_to_num(u), 0, p.width - 1).astype(int)
        seen &= cam.valid[row, col]
        return row, col, seen

    def _grid(self, x: float, y: float, yaw: float, newest):
        """Pavement (1), not pavement (0) or unknown (-1) on the grid around the
        pose. Each cell takes the frame that saw it from closest: a pixel row
        far ahead covers half a metre of ground and is where a model is least
        sure, so a near view taken a second ago beats a distant one taken now.
        Views beyond ``trusted_range`` are not used at all."""
        grid = np.full(self.gx.shape, -1, dtype=np.int8)
        best = np.full(self.gx.shape, self.trusted_range)
        c, sn = math.cos(yaw), math.sin(yaw)
        wx, wy = x + c * self.gx - sn * self.gy, y + sn * self.gx + c * self.gy
        for _t, fx0, fy0, fyaw, mask in [newest, *reversed(self.frames)]:
            cf, sf = math.cos(fyaw), math.sin(fyaw)
            dx, dy = wx - fx0, wy - fy0
            fx, ly = cf * dx + sf * dy, -sf * dx + cf * dy
            distance = np.hypot(fx, ly)
            closer = distance < best
            if not closer.any():
                continue
            row, col, seen = self._pixels(fx[closer], ly[closer])
            take = np.zeros_like(closer)
            take[closer] = seen
            grid[take] = mask[row[seen], col[seen]]
            best[take] = distance[take]
        return grid

    def _lookup(self, grid, px, py):
        i = np.round((px - self.xs[0]) / self.resolution).astype(int)
        j = np.round((py - self.ys[0]) / self.resolution).astype(int)
        inside = (i >= 0) & (i < grid.shape[0]) & (j >= 0) & (j < grid.shape[1])
        out = np.full(np.shape(px), -1, dtype=np.int8)
        out[inside] = grid[i[inside], j[inside]]
        return out

    def _ray(self, grid, px, py, nx, ny, reach=15.0):
        """Distance along (nx, ny) to the last pavement cell before the first
        cell that is not known pavement, and whether that cell was unseen
        rather than seen not to be pavement."""
        d = np.arange(0.0, reach, 0.5 * self.resolution)
        values = self._lookup(grid, px + d * nx, py + d * ny)
        stop = np.flatnonzero(values != 1)
        if not len(stop):
            return float(d[-1]), True
        return (float(d[stop[0] - 1]) if stop[0] > 0 else 0.0), bool(values[stop[0]] < 0)

    def _march(self, grid, x0, heading, length):
        """Walk ``length`` metres of corridor from (x0, 0); returns (left, right)
        point lists in order of travel."""
        px, py, h = x0, 0.0, heading
        left, right, centers, widths = [], [], [], []
        travelled = 0.0
        while travelled <= length:
            if self._lookup(grid, np.array([px]), np.array([py]))[0] != 1:
                break
            nx, ny = -math.sin(h), math.cos(h)
            (dl, unseen_l), (dr, unseen_r) = self._ray(grid, px, py, nx, ny), self._ray(grid, px, py, -nx, -ny)
            # Where unseen ground, not grass, pinches the corridor, the view has
            # run out (around a hairpin, past the image edge): stop there rather
            # than hand the planner a corridor that closes for no real reason.
            if (unseen_l or unseen_r) and widths and dl + dr < 0.7 * float(np.median(widths)):
                break
            widths.append(dl + dr)
            left.append((px + dl * nx, py + dl * ny))
            right.append((px - dr * nx, py - dr * ny))
            cx, cy = px + 0.5 * (dl - dr) * nx, py + 0.5 * (dl - dr) * ny
            if centers:
                bx, by = centers[max(0, len(centers) - 3)]
                if math.hypot(cx - bx, cy - by) > 1e-3:
                    turn = math.atan2(
                        math.sin(math.atan2(cy - by, cx - bx) - h), math.cos(math.atan2(cy - by, cx - bx) - h)
                    )
                    h += float(np.clip(turn, -0.35, 0.35))
            centers.append((cx, cy))
            # step from a point pulled halfway back toward the center line
            px = 0.5 * (px + cx) + self.step * math.cos(h)
            py = 0.5 * (py + cy) + self.step * math.sin(h)
            travelled += self.step
        return self._smooth(left, right)

    @staticmethod
    def _smooth(left, right, window=5):
        """Low-pass the corridor: average its center line over ``window`` steps
        and take the narrowest half-width nearby, then rebuild the edges.

        Raw ray ends jitter by a cell or two with mask noise, and in a curve
        the rays fan, so edge points can bunch or step backwards. The planner
        reads curvature from the reference built on these edges, and a few
        centimetres of zigzag a metre apart exceed the steering limit. The
        minimum keeps the corridor no wider than any ray actually measured.
        """
        if len(left) < 3:
            return left, right
        lft, rgt = np.asarray(left), np.asarray(right)
        center, half = 0.5 * (lft + rgt), 0.5 * np.hypot(*(lft - rgt).T)
        k = min(window, len(center) if len(center) % 2 else len(center) - 1)
        pad = k // 2
        kernel = np.ones(k) / k
        padded = np.vstack([np.repeat(center[:1], pad, 0), center, np.repeat(center[-1:], pad, 0)])
        smooth = np.column_stack([np.convolve(padded[:, i], kernel, mode="valid") for i in range(2)])
        narrow = np.array([half[max(0, i - pad) : i + pad + 1].min() for i in range(len(half))])
        direction = np.gradient(smooth, axis=0)
        direction /= np.maximum(np.hypot(*direction.T), 1e-9)[:, None]
        normal = np.column_stack([-direction[:, 1], direction[:, 0]])
        return list(smooth + narrow[:, None] * normal), list(smooth - narrow[:, None] * normal)

    # ------------------------------------------------------------ the frame
    def update(self, mask, x: float, y: float, yaw: float, t: float):
        """Edges for the pavement ``mask`` seen at estimated pose (x, y, yaw) at
        time t. Returns (left, right) polylines in base_link, or None when the
        corridor ahead is too short to drive."""
        if mask is None:
            return None
        mask = np.asarray(mask, dtype=bool)
        # Every frame is looked at first; one every keyframe_s is remembered.
        newest = (t, x, y, yaw, mask)
        if not self.frames or t - self.frames[-1][0] >= self.keyframe_s - 1e-9:
            self.frames.append(newest)
        self.frames = [f for f in self.frames if f[0] >= t - self.memory_s]
        grid = self._grid(x, y, yaw, newest)
        # From the kart when the ground under it is remembered; otherwise (the
        # first frames) from where the camera's view is wide enough to see the
        # corridor, held back over the kart as edges_from_mask does.
        start = 0.0 if self._lookup(grid, np.array([0.0]), np.array([0.0]))[0] == 1 else self.first_view
        ahead_left, ahead_right = self._march(grid, start, 0.0, self.ahead)
        if len(ahead_left) * self.step < MIN_ROWS * SPACING_M:
            return None
        if start > 0.0:  # nothing remembered behind: hold the first edges back
            return _extend_back(np.asarray(ahead_left)), _extend_back(np.asarray(ahead_right))
        behind_left, behind_right = self._march(grid, start, math.pi, self.back)
        # behind the kart the march faces backwards, so its left is our right
        return np.asarray(behind_right[:0:-1] + ahead_left), np.asarray(behind_left[:0:-1] + ahead_right)


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


def checked_drive(points, speeds):
    """A driving model's output as finite arrays: waypoints (K, 2), speeds (K,)."""
    points, speeds = np.asarray(points, dtype=float), np.asarray(speeds, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or speeds.shape != (len(points),) or len(points) < 2:
        raise ValueError("a driving model must return waypoints (K, 2) and speeds (K,)")
    if not (np.isfinite(points).all() and np.isfinite(speeds).all()):
        raise ValueError("driving model output must be finite")
    return points, speeds


class CameraPerception:
    """Camera frame, model, mask, edges. Records mask IoU against the true mask
    and the inference wall time; neither is visible to the driving stack."""

    def __init__(self, camera: TrackCamera, segmenter, name="camera"):
        self.camera, self.segmenter, self.name = camera, segmenter, name
        self.ious: list[float] = []
        self.inference_ms: list[float] = []
        self.last_rgb = self.last_truth = self.last_prediction = None
        self.last_drive = None  # (waypoints K x 2 in base_link, speeds K) from a driving model

    def reset(self, seed=0):
        self.ious, self.inference_ms = [], []
        self.last_drive = None
        if callable(getattr(self.segmenter, "reset", None)):
            self.segmenter.reset(seed)

    def __call__(self, state):
        return edges_from_mask(self.observe(state), self.camera)

    def observe(self, state):
        """The predicted pavement mask for the frame at ``state``, for
        ``CorridorTracker``; scoring is recorded the same way."""
        rgb, truth = self.camera.render(state)
        started = time.perf_counter()
        if callable(getattr(self.segmenter, "predict_drive", None)):
            # a driving model: the line it wants to take comes with the mask
            probability, points, speeds = self.segmenter.predict_drive(rgb.copy())
            self.last_drive = checked_drive(points, speeds)
        else:
            probability = self.segmenter.predict(rgb.copy())
        prediction = checked_mask(probability, truth.shape)
        self.inference_ms.append((time.perf_counter() - started) * 1000)
        self.ious.append(mask_iou(prediction, truth))
        self.last_rgb, self.last_truth, self.last_prediction = rgb, truth, prediction
        return prediction
