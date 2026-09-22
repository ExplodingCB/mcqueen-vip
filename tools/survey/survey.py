#!/usr/bin/env python3
"""Turn edge drives into a track model file (docs/02-architecture.md section 6).

Input: two drives, one along the left edge and one along the right edge, in the
direction of travel, as x,y in the local map frame or as lat,lon(,height) from
the RTK receiver. Output: tracks/<id>/track.csv and track.yaml in the TUM format
that mcq_track, mcq_sim and the raceline optimizer read.

    python tools/survey/survey.py --left left.csv --right right.csv \\
        --out tracks/purdue --track-id purdue --datum 40.4237 -86.9212 190

Steps: drop standstill points, close each loop where the drive overlaps its own
start, resample by arc length, smooth, pair the edges, build the centerline as
the midpoint and the half-widths as the edge distances, and resample the result
at the requested spacing. MCAP input (an EgoState or NavSatFix topic) goes
through the same pipeline once the mcap packages are installed.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src" / "mcq_sim"))

from mcq_sim.geodesy import geodetic_to_enu  # noqa: E402
from mcq_sim.track import Track  # noqa: E402


# ----------------------------------------------------------------- pipeline
@dataclass
class SurveyParams:
    spacing: float = 1.0  # m between output points
    min_step: float = 0.05  # m; closer consecutive samples are standstill and dropped
    smooth_m: float = 3.0  # m window of the moving average on each edge
    close_tol: float = 3.0  # m; the drive is closed when it comes back this close to its start


def clean_drive(xy: np.ndarray, min_step: float) -> np.ndarray:
    xy = np.asarray(xy, float)
    if len(xy) < 3:
        raise ValueError("a drive needs at least three points")
    keep = [0]
    for i in range(1, len(xy)):
        if np.hypot(*(xy[i] - xy[keep[-1]])) >= min_step:
            keep.append(i)
    return xy[keep]


def close_loop(xy: np.ndarray, tol: float) -> np.ndarray:
    """Cut the drive where it first returns to its start after covering more
    than half its length, so an overlap at the finish does not double up."""
    seg = np.hypot(*np.diff(xy, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    far = s > 0.5 * s[-1]
    dist = np.hypot(*(xy - xy[0]).T)
    candidates = np.where(far & (dist < tol))[0]
    if len(candidates) == 0:
        raise ValueError(f"drive does not return to its start within {tol} m; is it a closed lap?")
    # Within the first run of near-start samples, cut at the one closest to the start.
    run_end = candidates[0]
    while run_end + 1 < len(xy) and far[run_end + 1] and dist[run_end + 1] < tol:
        run_end += 1
    cut = candidates[0] + int(np.argmin(dist[candidates[0] : run_end + 1]))
    return xy[:cut]


def resample_closed(xy: np.ndarray, spacing: float) -> np.ndarray:
    closed = np.vstack([xy, xy[:1]])
    seg = np.hypot(*np.diff(closed, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(int(round(s[-1] / spacing)), 8)
    u = np.linspace(0.0, s[-1], n, endpoint=False)
    return np.column_stack([np.interp(u, s, closed[:, 0]), np.interp(u, s, closed[:, 1])])


def smooth_closed(xy: np.ndarray, window_pts: int) -> np.ndarray:
    if window_pts <= 1:
        return xy
    k = np.ones(window_pts) / window_pts
    pad = window_pts // 2
    out = np.empty_like(xy)
    for j in range(2):
        col = np.concatenate([xy[-pad:, j], xy[:, j], xy[:pad, j]])
        out[:, j] = np.convolve(col, k, mode="same")[pad : pad + len(xy)]
    return out


def prepare_edge(xy: np.ndarray, p: SurveyParams) -> np.ndarray:
    xy = clean_drive(xy, p.min_step)
    xy = close_loop(xy, p.close_tol)
    fine = min(p.spacing, 0.5)
    xy = resample_closed(xy, fine)
    return smooth_closed(xy, max(int(round(p.smooth_m / fine)), 1))


def signed_area(xy: np.ndarray) -> float:
    x, y = xy[:, 0], xy[:, 1]
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def build_track(left: np.ndarray, right: np.ndarray, p: SurveyParams | None = None, track_id: str = "survey") -> Track:
    p = p or SurveyParams()
    left_e = prepare_edge(left, p)
    right_e = prepare_edge(right, p)

    # First centerline: midpoints between each left point and its nearest right point.
    _, idx = cKDTree(right_e).query(left_e)
    center = 0.5 * (left_e + right_e[idx])
    center = resample_closed(center, p.spacing)
    center = smooth_closed(center, max(int(round(p.smooth_m / p.spacing)), 1))

    # The left edge must be on the left of the direction of travel.
    tangent = np.roll(center, -1, axis=0) - np.roll(center, 1, axis=0)
    normal = np.column_stack([-tangent[:, 1], tangent[:, 0]]) / np.maximum(np.hypot(*tangent.T), 1e-9)[:, None]
    _, il = cKDTree(left_e).query(center)
    side = np.sum((left_e[il] - center) * normal, axis=1)
    if np.median(side) < 0:
        raise ValueError("the 'left' drive lies to the right of the direction of travel; swap the inputs")

    w_left, _ = cKDTree(left_e).query(center)
    w_right, _ = cKDTree(right_e).query(center)
    track = Track(center[:, 0], center[:, 1], w_right, w_left, closed=True, track_id=track_id)
    return track


# ---------------------------------------------------------------------- I/O
def read_drive_csv(path: Path, datum=None) -> np.ndarray:
    """x,y in metres, or lat,lon(,height) converted to ENU about the datum.
    The header names decide which."""
    with open(path) as f:
        header = f.readline().strip().lstrip("#").lower().replace(" ", "")
    names = header.split(",")
    data = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    if names[:2] == ["x", "y"] or names[:2] == ["x_m", "y_m"]:
        return data[:, :2]
    if names[:2] in (["lat", "lon"], ["latitude", "longitude"]):
        if datum is None:
            raise ValueError("lat,lon input needs --datum lat lon height")
        h = data[:, 2] if data.shape[1] > 2 else np.full(len(data), datum[2])
        e, n, _ = geodetic_to_enu(data[:, 0], data[:, 1], h, datum)
        return np.column_stack([e, n])
    raise ValueError(f"{path}: header must start with x,y or lat,lon; got {header!r}")


def read_drive_mcap(path: Path, topic: str, datum=None) -> np.ndarray:
    """Positions from an MCAP log: mcq_msgs/EgoState (pose in map) or
    sensor_msgs/NavSatFix (converted with the datum)."""
    try:
        from mcap.reader import make_reader
        from mcap_ros2.decoder import DecoderFactory
    except ImportError as exc:  # pragma: no cover - depends on optional packages
        raise SystemExit("pip install mcap mcap-ros2-support to read MCAP logs") from exc
    pts = []
    with open(path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        messages = [d for _s, _c, _m, d in reader.iter_decoded_messages(topics=[topic])]
    for msg in messages:
        if hasattr(msg, "latitude"):
            if datum is None:
                raise ValueError("NavSatFix input needs --datum lat lon height")
            e, n, _ = geodetic_to_enu(msg.latitude, msg.longitude, msg.altitude, datum)
            pts.append((float(e), float(n)))
        else:
            pts.append((msg.pose.position.x, msg.pose.position.y))
    return np.asarray(pts, float)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--left", required=True, help="left edge drive: csv, or mcap with --topic")
    parser.add_argument("--right", required=True, help="right edge drive: csv, or mcap with --topic")
    parser.add_argument("--topic", default="/ego_state", help="topic to read from mcap inputs")
    parser.add_argument("--out", required=True, help="output track directory")
    parser.add_argument("--track-id", default=None)
    parser.add_argument("--datum", type=float, nargs=3, metavar=("LAT", "LON", "H"), help="map frame origin")
    parser.add_argument("--spacing", type=float, default=1.0)
    parser.add_argument("--smooth", type=float, default=3.0, help="moving average window, m")
    parser.add_argument("--survey-date", default="")
    args = parser.parse_args(argv)

    def load(path):
        path = Path(path)
        if path.suffix == ".mcap":
            return read_drive_mcap(path, args.topic, args.datum)
        return read_drive_csv(path, args.datum)

    params = SurveyParams(spacing=args.spacing, smooth_m=args.smooth)
    track_id = args.track_id or Path(args.out).name
    track = build_track(load(args.left), load(args.right), params, track_id=track_id)
    track.meta = {
        "track_id": track_id,
        "closed": True,
        "datum": {
            "latitude": args.datum[0] if args.datum else 0.0,
            "longitude": args.datum[1] if args.datum else 0.0,
            "height": args.datum[2] if args.datum else 0.0,
        },
        "start_finish": {"a": [float(track.x[0]), float(track.y[0])], "b": [float(track.x[0]), float(track.y[0])]},
        "survey_date": args.survey_date,
        "source": {"left": str(args.left), "right": str(args.right)},
    }
    track.save(args.out)
    wl, wr = track.w_left, track.w_right
    print(
        f"wrote {args.out}: {len(track.x)} points, {track.length:.1f} m, "
        f"width {np.min(wl + wr):.2f} to {np.max(wl + wr):.2f} m"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
