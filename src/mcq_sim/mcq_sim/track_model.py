"""Validate the TrackModel wire representation without opening track files."""

import numpy as np

from mcq_sim.track import Track


def track_from_model(msg):
    if msg.raceline or msg.raceline_kappa or msg.raceline_v:
        raise ValueError("raceline models are unsupported until surveyed-boundary validation is implemented")
    if msg.header.frame_id != "map":
        raise ValueError("TrackModel must be in map")
    x = np.array([p.x for p in msg.centerline], dtype=float)
    y = np.array([p.y for p in msg.centerline], dtype=float)
    left = np.asarray(msg.width_left, dtype=float)
    right = np.asarray(msg.width_right, dtype=float)
    if len(x) < 3 or len(left) != len(x) or len(right) != len(x):
        raise ValueError("TrackModel centerline and widths must share at least three rows")
    if not all(np.isfinite(a).all() for a in (x, y, left, right)) or np.any(left <= 0) or np.any(right <= 0):
        raise ValueError("TrackModel needs finite coordinates and positive widths")
    return Track(x, y, right, left, closed=msg.closed, track_id=msg.track_id)


def model_signature(msg):
    # Periodic publications refresh availability without rebuilding identical splines.
    return (
        msg.header.frame_id,
        msg.closed,
        msg.track_id,
        tuple((p.x, p.y) for p in msg.centerline),
        tuple(msg.width_left),
        tuple(msg.width_right),
        tuple((p.x, p.y) for p in msg.raceline),
        tuple(msg.raceline_kappa),
        tuple(msg.raceline_v),
    )
