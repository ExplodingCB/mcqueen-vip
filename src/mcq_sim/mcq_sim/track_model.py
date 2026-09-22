"""Validate the TrackModel wire representation without opening track files."""

import numpy as np

from mcq_sim.track import Raceline, Track


def track_from_model(msg):
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


def reference_from_model(msg, track):
    """Return optional raceline with widths measured back to surveyed boundaries."""
    if not msg.raceline:
        if msg.raceline_kappa or msg.raceline_v:
            raise ValueError("raceline arrays must share an index")
        return track, None
    n = len(msg.raceline)
    if n < 3 or len(msg.raceline_kappa) != n or len(msg.raceline_v) != n:
        raise ValueError("raceline arrays must share at least three points")
    x = np.array([p.x for p in msg.raceline])
    y = np.array([p.y for p in msg.raceline])
    kappa = np.array(msg.raceline_kappa)
    speed = np.array(msg.raceline_v)
    if not all(np.isfinite(a).all() for a in (x, y, kappa, speed)) or np.any(speed < 0):
        raise ValueError("raceline geometry must be finite and speeds nonnegative")
    s, d = track.frenet(x, y)
    left, right = track.width_at(s)
    if np.any(left - d <= 0) or np.any(right + d <= 0):
        raise ValueError("raceline must stay between surveyed edges")
    reference = Track(x, y, right + d, left - d, closed=track.closed, track_id=track.track_id)
    if len(reference.x) != n:
        raise ValueError("raceline cannot contain duplicate points")
    raceline = Raceline(reference.s, x, y, reference.psi, kappa, speed, np.zeros(n))
    return reference, raceline
