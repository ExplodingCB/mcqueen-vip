"""Track model: centerline with half-widths, arc length, Frenet conversion, geofence.

The on-disk format is the TUM global_racetrajectory_optimization one so the
raceline optimizer reads the file unchanged:

    # x_m, y_m, w_tr_right_m, w_tr_left_m

A companion track.yaml carries the datum, the start/finish line, the id and the
survey date. Coordinates are in the local ``map`` frame, x east, y north.
Positive lateral offset (d) is to the left of the direction of travel.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml
from scipy.interpolate import CubicSpline
from scipy.spatial import cKDTree

CSV_HEADER = "# x_m, y_m, w_tr_right_m, w_tr_left_m"


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


@dataclass
class Track:
    x: np.ndarray
    y: np.ndarray
    w_right: np.ndarray
    w_left: np.ndarray
    closed: bool = True
    track_id: str = "unnamed"
    meta: dict = field(default_factory=dict)

    # Derived in __post_init__.
    s: np.ndarray = field(init=False, repr=False)
    psi: np.ndarray = field(init=False, repr=False)
    kappa: np.ndarray = field(init=False, repr=False)
    length: float = field(init=False)
    _tree: cKDTree = field(init=False, repr=False)

    def __post_init__(self):
        self.x = np.asarray(self.x, dtype=float)
        self.y = np.asarray(self.y, dtype=float)
        self.w_right = np.asarray(self.w_right, dtype=float)
        self.w_left = np.asarray(self.w_left, dtype=float)
        # Drop repeated points so arc length is strictly increasing.
        seg = np.hypot(np.diff(self.x), np.diff(self.y))
        keep = np.concatenate([[True], seg > 1e-6])
        if self.closed and np.hypot(self.x[-1] - self.x[0], self.y[-1] - self.y[0]) <= 1e-6:
            keep[-1] = False
        self.x, self.y = self.x[keep], self.y[keep]
        self.w_right, self.w_left = self.w_right[keep], self.w_left[keep]
        n = len(self.x)
        if n < 3:
            raise ValueError("a track needs at least three distinct points")
        seg = np.hypot(np.diff(self.x), np.diff(self.y))
        self.s = np.concatenate([[0.0], np.cumsum(seg)])
        if self.closed:
            closing = float(np.hypot(self.x[0] - self.x[-1], self.y[0] - self.y[-1]))
            self.length = float(self.s[-1] + closing)
            knots = np.append(self.s, self.length)
            self._sx = CubicSpline(knots, np.append(self.x, self.x[0]), bc_type="periodic")
            self._sy = CubicSpline(knots, np.append(self.y, self.y[0]), bc_type="periodic")
        else:
            self.length = float(self.s[-1])
            self._sx = CubicSpline(self.s, self.x, bc_type="natural")
            self._sy = CubicSpline(self.s, self.y, bc_type="natural")
        self.psi = self.heading_at(self.s)
        self.kappa = self.curvature_at(self.s)
        self._tree = cKDTree(np.column_stack([self.x, self.y]))

    def _wrap_s(self, s):
        s = np.atleast_1d(np.asarray(s, dtype=float))
        return np.mod(s, self.length) if self.closed else np.clip(s, 0.0, self.length)

    # ------------------------------------------------------------------ I/O
    @classmethod
    def from_csv(cls, path: str | Path, closed: bool = True, track_id: str | None = None) -> Track:
        data = np.loadtxt(path, delimiter=",", comments="#", ndmin=2)
        if data.shape[1] != 4:
            raise ValueError(f"{path}: expected 4 columns, got {data.shape[1]}")
        return cls(
            data[:, 0],
            data[:, 1],
            data[:, 2],
            data[:, 3],
            closed=closed,
            track_id=track_id or Path(path).stem,
        )

    def to_csv(self, path: str | Path) -> None:
        rows = np.column_stack([self.x, self.y, self.w_right, self.w_left])
        np.savetxt(path, rows, delimiter=",", fmt="%.4f", header=CSV_HEADER[2:], comments="# ")

    @classmethod
    def load(cls, directory: str | Path) -> Track:
        directory = Path(directory)
        meta = {}
        yaml_path = directory / "track.yaml"
        if yaml_path.exists():
            with open(yaml_path) as f:
                meta = yaml.safe_load(f) or {}
        track = cls.from_csv(
            directory / "track.csv",
            closed=bool(meta.get("closed", True)),
            track_id=meta.get("track_id", directory.name),
        )
        track.meta = meta
        if meta.get("aerial", {}).get("file"):
            track.meta["aerial_path"] = str((directory / meta["aerial"]["file"]).resolve())
        return track

    def save(self, directory: str | Path) -> None:
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        self.to_csv(directory / "track.csv")
        meta = dict(self.meta)
        meta.setdefault("track_id", self.track_id)
        meta.setdefault("closed", self.closed)
        meta.setdefault("datum", {"latitude": 0.0, "longitude": 0.0, "height": 0.0})
        meta.setdefault("start_finish", {"a": [float(self.x[0]), float(self.y[0])], "b": [0.0, 0.0]})
        with open(directory / "track.yaml", "w") as f:
            yaml.safe_dump(meta, f, sort_keys=False)

    # ------------------------------------------------------------ builders
    @classmethod
    def synthetic_oval(
        cls,
        straight: float = 60.0,
        radius: float = 15.0,
        width: float = 5.0,
        spacing: float = 1.0,
        track_id: str = "synthetic_oval",
    ) -> Track:
        """Counter-clockwise oval: two straights joined by semicircles, start on the
        lower straight heading +x."""
        arc = np.pi * radius
        n_straight = max(int(round(straight / spacing)), 2)
        n_arc = max(int(round(arc / spacing)), 4)
        xs, ys = [], []
        # Lower straight, y = -radius, x from -straight/2 to +straight/2.
        for i in range(n_straight):
            xs.append(-straight / 2 + straight * i / n_straight)
            ys.append(-radius)
        # Right semicircle centered at (straight/2, 0), from angle -pi/2 to pi/2.
        for i in range(n_arc):
            a = -np.pi / 2 + np.pi * i / n_arc
            xs.append(straight / 2 + radius * np.cos(a))
            ys.append(radius * np.sin(a))
        # Upper straight, y = +radius, x decreasing.
        for i in range(n_straight):
            xs.append(straight / 2 - straight * i / n_straight)
            ys.append(radius)
        # Left semicircle centered at (-straight/2, 0), from pi/2 to 3pi/2.
        for i in range(n_arc):
            a = np.pi / 2 + np.pi * i / n_arc
            xs.append(-straight / 2 + radius * np.cos(a))
            ys.append(radius * np.sin(a))
        n = len(xs)
        half = np.full(n, width / 2)
        track = cls(np.array(xs), np.array(ys), half, half, closed=True, track_id=track_id)
        track.meta = {
            "track_id": track_id,
            "closed": True,
            "datum": {"latitude": 0.0, "longitude": 0.0, "height": 0.0},
            "start_finish": {"a": [float(xs[0]), float(ys[0]) - width], "b": [float(xs[0]), float(ys[0]) + width]},
            "survey_date": "synthetic",
            "description": f"oval, straights {straight} m, radius {radius} m, width {width} m",
        }
        return track

    @classmethod
    def from_bounds(cls, left: np.ndarray, right: np.ndarray, n: int = 40, track_id: str = "bounds") -> Track:
        """Build an open local track from perceived left and right edge polylines
        (map frame, each (k, 2), ordered forward). The centerline is the midpoint
        of the two edges resampled by arc length."""
        left = np.asarray(left, dtype=float)
        right = np.asarray(right, dtype=float)
        if len(left) < 2 or len(right) < 2:
            raise ValueError("need at least two points per edge")

        def resample(poly):
            seg = np.hypot(*np.diff(poly, axis=0).T)
            s = np.concatenate([[0.0], np.cumsum(seg)])
            u = np.linspace(0.0, s[-1], n)
            return np.column_stack([np.interp(u, s, poly[:, 0]), np.interp(u, s, poly[:, 1])])

        lp, rp = resample(left), resample(right)
        center = 0.5 * (lp + rp)
        half = 0.5 * np.hypot(*(lp - rp).T)
        return cls(center[:, 0], center[:, 1], half, half, closed=False, track_id=track_id)

    def shifted(self, dx: float, dy: float) -> Track:
        return Track(
            self.x + dx,
            self.y + dy,
            self.w_right,
            self.w_left,
            closed=self.closed,
            track_id=self.track_id,
            meta=dict(self.meta),
        )

    # ------------------------------------------------------------- geometry
    def _next(self, i: np.ndarray) -> np.ndarray:
        n = len(self.x)
        if self.closed:
            return (i + 1) % n
        return np.minimum(i + 1, n - 1)

    def _prev(self, i: np.ndarray) -> np.ndarray:
        n = len(self.x)
        if self.closed:
            return (i - 1) % n
        return np.maximum(i - 1, 0)

    def frenet(self, x, y):
        """Cartesian (map) to Frenet (s, d). Vectorized; d positive left."""
        x = np.atleast_1d(np.asarray(x, dtype=float))
        y = np.atleast_1d(np.asarray(y, dtype=float))
        pts = np.column_stack([x, y])
        _, i = self._tree.query(pts)
        i = np.asarray(i)
        best_s = np.zeros(len(x))
        best_d = np.zeros(len(x))
        best_dist = np.full(len(x), np.inf)
        for a, b in ((self._prev(i), i), (i, self._next(i))):
            ax, ay = self.x[a], self.y[a]
            tx, ty = self.x[b] - ax, self.y[b] - ay
            seg_len = np.hypot(tx, ty)
            valid = seg_len > 1e-9
            tx = np.where(valid, tx / np.maximum(seg_len, 1e-9), 0.0)
            ty = np.where(valid, ty / np.maximum(seg_len, 1e-9), 0.0)
            px, py = x - ax, y - ay
            t = np.clip(px * tx + py * ty, 0.0, seg_len)
            d = tx * py - ty * px
            qx, qy = ax + t * tx, ay + t * ty
            dist = np.hypot(x - qx, y - qy)
            better = valid & (dist < best_dist)
            s_here = self.s[a] + t
            if self.closed:
                s_here = np.mod(s_here, self.length)
            best_s = np.where(better, s_here, best_s)
            best_d = np.where(better, d, best_d)
            best_dist = np.where(better, dist, best_dist)
        return best_s, best_d

    def _interp_along(self, s: np.ndarray, values: np.ndarray, angular: bool = False):
        if self.closed:
            s = np.mod(s, self.length)
            sp = np.concatenate([self.s, [self.length]])
            vp = np.concatenate([values, [values[0]]])
        else:
            s = np.clip(s, 0.0, self.length)
            sp, vp = self.s, values
        if angular:
            c = np.interp(s, sp, np.cos(vp))
            sn = np.interp(s, sp, np.sin(vp))
            return np.arctan2(sn, c)
        return np.interp(s, sp, vp)

    def cartesian(self, s, d=0.0):
        """Frenet (s, d) to map (x, y) and reference heading psi at s."""
        s = self._wrap_s(s)
        d = np.broadcast_to(np.asarray(d, dtype=float), s.shape)
        cx, cy = self._sx(s), self._sy(s)
        psi = np.arctan2(self._sy(s, 1), self._sx(s, 1))
        return cx - d * np.sin(psi), cy + d * np.cos(psi), psi

    def heading_at(self, s):
        s = self._wrap_s(s)
        return np.arctan2(self._sy(s, 1), self._sx(s, 1))

    def curvature_at(self, s):
        s = self._wrap_s(s)
        dx, dy = self._sx(s, 1), self._sy(s, 1)
        ddx, ddy = self._sx(s, 2), self._sy(s, 2)
        return (dx * ddy - dy * ddx) / np.maximum(np.hypot(dx, dy) ** 3, 1e-9)

    def width_at(self, s):
        s = np.atleast_1d(np.asarray(s, dtype=float))
        return self._interp_along(s, self.w_left), self._interp_along(s, self.w_right)

    def edges(self):
        """Left and right edge polylines as (n, 2) arrays."""
        nx, ny = -np.sin(self.psi), np.cos(self.psi)
        left = np.column_stack([self.x + self.w_left * nx, self.y + self.w_left * ny])
        right = np.column_stack([self.x - self.w_right * nx, self.y - self.w_right * ny])
        return left, right

    def distance_to_edge(self, x, y):
        """Signed distance to the nearest edge, positive inside the track."""
        s, d = self.frenet(x, y)
        wl, wr = self.width_at(s)
        return np.minimum(wl - d, wr + d)

    def inside(self, x, y, margin: float = 0.0):
        return self.distance_to_edge(x, y) >= -margin

    def bounds_ahead(
        self,
        x: float,
        y: float,
        yaw: float,
        range_m: float,
        spacing: float = 1.0,
        back: float = 5.0,
    ):
        """Perceived-edge stand-in: the left and right edges from ``back`` metres
        behind the kart to ``range_m`` ahead, in the base_link frame (x forward,
        y left). The part behind stands in for the boundary map the track server
        accumulates as the kart drives."""
        s0, _ = self.frenet(x, y)
        s = s0[0] + np.arange(-back, range_m + 1e-9, spacing)
        if not self.closed:
            s = s[(s >= 0.0) & (s <= self.length)]
        wl, wr = self.width_at(s)
        lx, ly, _ = self.cartesian(s, wl)
        rx, ry, _ = self.cartesian(s, -wr)
        c, sn = np.cos(yaw), np.sin(yaw)

        def to_base(px, py):
            dx, dy = px - x, py - y
            return np.column_stack([c * dx + sn * dy, -sn * dx + c * dy])

        return to_base(lx, ly), to_base(rx, ry)


def base_to_map(points: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    px = x + c * points[:, 0] - s * points[:, 1]
    py = y + s * points[:, 0] + c * points[:, 1]
    return np.column_stack([px, py])


@dataclass
class Raceline:
    """A TUM optimizer output: s, x, y, heading, curvature, speed, acceleration.

    File columns (semicolon separated, '#' comments):
    s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2
    """

    s: np.ndarray
    x: np.ndarray
    y: np.ndarray
    psi: np.ndarray
    kappa: np.ndarray
    v: np.ndarray
    a: np.ndarray

    @classmethod
    def from_csv(cls, path: str | Path) -> Raceline:
        data = np.loadtxt(path, delimiter=";", comments="#", ndmin=2)
        if data.shape[1] < 7:
            raise ValueError(f"{path}: expected 7 columns")
        psi = data[:, 3]
        return cls(data[:, 0], data[:, 1], data[:, 2], psi, data[:, 4], data[:, 5], data[:, 6])

    def as_track(self, w_left: np.ndarray, w_right: np.ndarray) -> Track:
        return Track(self.x, self.y, w_right, w_left, closed=True, track_id="raceline")
