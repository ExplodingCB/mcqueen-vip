"""A smooth drivable reference line through a track's corridor.

The Purdue centerline is a hand-drawn KML trace. It is right about where the
pavement is and wrong about how to drive it: it kinks, and at one spot it hugs
the outside edge while its curvature touches the steering limit. A sampling
planner that offsets from that line inherits the kinks, and the kart, which can
turn no tighter than about 2.9 m, finds no feasible path where the pavement is
perfectly drivable.

This finds the line a driver would take instead: the path through the corridor
with the least total squared curvature, kept a set distance from both edges.
Positions are linear in the lateral offsets, so the problem is a bounded linear
least-squares one and needs nothing beyond scipy. It is the minimum-curvature
formulation the TUM optimizer uses (docs/10-glossary.md), without its vehicle
model or speed profile. `tools/raceline` (issue #5) replaces it when that lands.

The result is a Track whose edges are the original ones, re-measured from the
new line, so it drops into the planner wherever the centerline did.
"""

from __future__ import annotations

import numpy as np
import scipy.sparse as sp
from scipy.optimize import lsq_linear

from mcq_sim.track import Track


def _second_difference(n: int) -> np.ndarray:
    """Cyclic second difference operator on n samples."""
    eye = sp.identity(n, format="csr")
    shift = sp.csr_matrix((np.ones(n), (np.arange(n), (np.arange(n) + 1) % n)), shape=(n, n))
    return (shift - 2.0 * eye + shift.T).tocsr()


def smooth_reference(track: Track, keep: float, spacing: float = 0.5) -> Track:
    """Minimum-curvature line at least ``keep`` metres from each edge.

    Where the corridor is narrower than 2 * keep the line runs down its middle.
    """
    if not track.closed:
        raise ValueError("smooth_reference needs a closed track")
    n = max(int(round(track.length / spacing)), 16)
    s = np.linspace(0.0, track.length, n, endpoint=False)
    cx, cy, _ = track.cartesian(s)
    heading = track.heading_at(s)
    nx, ny = -np.sin(heading), np.cos(heading)
    left, right = track.width_at(s)

    lower, upper = -right + keep, left - keep
    narrow = upper < lower
    middle = 0.5 * (left - right)
    lower, upper = np.where(narrow, middle, lower), np.where(narrow, middle, upper)

    d2 = _second_difference(n)
    a = sp.vstack([d2 @ sp.diags(nx), d2 @ sp.diags(ny)]).tocsr()
    b = -np.concatenate([d2 @ cx, d2 @ cy])
    d = lsq_linear(a, b, bounds=(lower, np.maximum(upper, lower + 1e-9)), method="trf", lsmr_tol="auto").x

    x, y = cx + d * nx, cy + d * ny
    # Re-measure the edges from the new line: to first order the left edge is
    # that much further away where the line moved right, and the reverse.
    meta = dict(track.meta)
    meta.update({"reference": "minimum_curvature", "reference_keep_m": keep, "reference_source": track.track_id})
    return Track(
        x,
        y,
        w_right=right + d,
        w_left=left - d,
        closed=True,
        track_id=f"{track.track_id}_min_curvature",
        meta=meta,
    )


def smooth_local_reference(track: Track, keep: float, sigmas=(4.0, 3.0, 2.0, 1.0)) -> Track:
    """A drivable line through an open corridor, such as the one built from the
    edges a camera perceived this cycle. Cheap enough for every planning cycle.

    The corridor's midline is low-passed along its arc length, which is what a
    pure-pursuit lookahead does implicitly and what shortens a hairpin's radius
    problem: the midline through a tight corner can be a curve no kart can turn,
    while the smoothed one is not. The widest smoothing that still keeps the line
    ``keep`` metres inside both edges is used. A bounded least-squares solve was
    tried here first and did worse: it left zigzags where a bound bound.
    """
    if track.closed:
        raise ValueError("smooth_local_reference needs an open track")
    n = len(track.x)
    if n < 5:
        return track
    spacing = float(np.mean(np.diff(track.s)))
    heading = track.heading_at(track.s)
    nx, ny = -np.sin(heading), np.cos(heading)

    def low_pass(values, sigma):
        half = max(int(3 * sigma / spacing), 1)
        kernel = np.exp(-0.5 * ((np.arange(-half, half + 1) * spacing) / sigma) ** 2)
        padded = np.concatenate([np.full(half, values[0]), values, np.full(half, values[-1])])
        return np.convolve(padded, kernel / kernel.sum(), mode="valid")

    for sigma in sigmas:
        x, y = low_pass(track.x, sigma), low_pass(track.y, sigma)
        d = (x - track.x) * nx + (y - track.y) * ny  # left of the midline is positive
        if np.all(track.w_left - d >= keep) and np.all(track.w_right + d >= keep):
            break
    return Track(x, y, w_right=track.w_right + d, w_left=track.w_left - d, closed=False, track_id=track.track_id)
