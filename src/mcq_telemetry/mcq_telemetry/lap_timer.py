"""Finish-segment timing, independent of ROS and the simulator's Frenet wrap."""

from __future__ import annotations

import csv
import math
from pathlib import Path

import yaml


def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def subtract(a, b):
    return a[0] - b[0], a[1] - b[1]


class LapTimer:
    """Forward finite-segment crossings with distance debounce and timestamp interpolation.

    Starting within start_tolerance of the line starts a standing lap at the
    first sample. Starting elsewhere arms timing at the first forward crossing.
    Gaps/teleports invalidate the partial lap, retaining completed results.
    """

    def __init__(
        self,
        a,
        b,
        direction,
        min_lap_distance,
        start_tolerance=0.25,
        max_gap=0.5,
        max_step=5.0,
        distance_resolution=0.5,
    ):
        self.a, self.b = tuple(a), tuple(b)
        self.line = subtract(self.b, self.a)
        self.length = math.hypot(*self.line)
        alignment = cross(self.line, direction)
        if self.length <= 0 or abs(alignment) < 1e-9:
            raise ValueError("finish line must be nonzero and cross the direction of travel")
        if min_lap_distance <= 0 or start_tolerance < 0 or max_gap <= 0 or max_step <= 0 or distance_resolution <= 0:
            raise ValueError("invalid lap timer limits")
        self.sign = 1 if alignment > 0 else -1
        self.min_distance = min_lap_distance
        self.start_tolerance = start_tolerance
        self.max_gap, self.max_step = max_gap, max_step
        self.distance_resolution = distance_resolution
        self.distance_anchor = None
        self.previous = None
        self.started = None
        self.distance = 0.0
        self.completed_laps = 0
        self.last_lap = math.nan
        self.best_lap = math.nan
        self.initial = True

    @classmethod
    def from_track(cls, directory, **kwargs):
        directory = Path(directory)
        meta = yaml.safe_load((directory / "track.yaml").read_text())
        if not meta.get("closed"):
            raise ValueError("lap timing requires a closed track")
        a, b = meta["start_finish"]["a"], meta["start_finish"]["b"]
        with (directory / "track.csv").open() as stream:
            points = [
                tuple(map(float, row[:2])) for row in csv.reader(line for line in stream if not line.startswith("#"))
            ]
        if len(points) < 3 or not all(math.isfinite(v) for p in [*points, a, b] for v in p):
            raise ValueError("track needs finite finish coordinates and at least three centerline points")
        segments = list(zip(points, points[1:] + points[:1], strict=True))
        # Find the centerline segment actually intersecting the finish segment.
        line = subtract(b, a)
        direction = None
        for p, q in segments:
            tangent = subtract(q, p)
            denominator = cross(tangent, line)
            if abs(denominator) < 1e-9:
                continue
            delta = subtract(a, p)
            u, v = cross(delta, line) / denominator, cross(delta, tangent) / denominator
            if 0 <= u <= 1 and 0 <= v <= 1:
                direction = tangent
                break
        if direction is None:
            raise ValueError("finish segment does not cross the centerline")
        length = sum(math.dist(p, q) for p, q in segments)
        timer = cls(a, b, direction, min_lap_distance=length * 0.5, **kwargs)
        timer.centerline = points
        return timer, str(meta["track_id"])

    def side(self, point):
        return self.sign * cross(self.line, subtract(point, self.a)) / self.length

    def on_segment(self, point):
        delta = subtract(point, self.a)
        along = sum(x * y for x, y in zip(delta, self.line, strict=True)) / self.length**2
        return 0 <= along <= 1

    def invalidate(self, allow_standing_start=False):
        self.previous = None
        self.started = None
        self.distance = 0.0
        self.distance_anchor = None
        self.initial = allow_standing_start

    def update(self, stamp, x, y):
        if not all(math.isfinite(v) for v in (stamp, x, y)):
            self.invalidate()
            return
        point = (x, y)
        if self.previous is not None:
            old_time, old = self.previous
            if stamp <= old_time or stamp - old_time > self.max_gap or math.dist(old, point) > self.max_step:
                self.invalidate()
        if self.previous is None:
            if self.initial and abs(self.side(point)) <= self.start_tolerance and self.on_segment(point):
                self.started = stamp
            self.initial = False
            self.previous = stamp, point
            self.distance_anchor = point
            return
        old_time, old = self.previous
        # Do not turn stationary GNSS jitter into accumulated lap distance.
        # Keep the anchor until motion exceeds the spatial resolution; this
        # also preserves real slow motion without a per-sample speed cutoff.
        step = math.dist(self.distance_anchor, point)
        if step >= self.distance_resolution:
            self.distance += step
            self.distance_anchor = point
        before, after = self.side(old), self.side(point)
        if before < 0 <= after:
            fraction = -before / (after - before)
            crossing = tuple(p + fraction * (q - p) for p, q in zip(old, point, strict=True))
            if self.on_segment(crossing) and (self.started is None or self.distance >= self.min_distance):
                when = old_time + fraction * (stamp - old_time)
                if self.started is not None:
                    self.last_lap = when - self.started
                    self.best_lap = min(self.best_lap, self.last_lap) if self.completed_laps else self.last_lap
                    self.completed_laps += 1
                self.started = when
                self.distance = 0.0
                self.distance_anchor = crossing
        self.previous = stamp, point

    def elapsed(self, now):
        return max(0.0, now - self.started) if self.started is not None else math.nan
