"""Build the user KML centerline with pavement widths from the pinned image trace."""

from __future__ import annotations

import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import map_coordinates, median_filter
from scipy.optimize import minimize_scalar

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/mcq_sim"))
sys.path.insert(0, str(ROOT / "tools/survey"))

from survey import geodetic_to_enu  # noqa: E402

from mcq_sim.track import Track  # noqa: E402


def user_centerline(x, y, start_xy):
    points = np.column_stack((x, y))
    if np.sum(points[:, 0] * np.roll(points[:, 1], -1) - points[:, 1] * np.roll(points[:, 0], -1)) < 0:
        points = points[::-1]
    # Long straight gaps need intermediate collinear knots, otherwise a periodic
    # cubic can bow several metres away from the supplied straight segment.
    knots = []
    for a, b in zip(points, np.roll(points, -1, axis=0), strict=True):
        length = np.linalg.norm(b - a)
        count = int(np.ceil(length / 3)) if length > 12 else 1
        knots.extend(a + (b - a) * f for f in np.linspace(0, 1, count, endpoint=False))
    knots = np.asarray(knots)
    sparse = Track(knots[:, 0], knots[:, 1], np.full(len(knots), 2.6), np.full(len(knots), 2.6))
    start_s = float(sparse.frenet(*start_xy)[0][0])

    def start_distance(s):
        px, py, _ = sparse.cartesian(s)
        return float((px[0] - start_xy[0]) ** 2 + (py[0] - start_xy[1]) ** 2)

    start_s = minimize_scalar(start_distance, bounds=(start_s - 3, start_s + 3), method="bounded").x % sparse.length
    # Keep every supplied knot exactly, plus dense samples, and rotate to the
    # existing checkered line. This preserves the user's edits after resampling.
    samples = np.unique(np.r_[np.arange(0, sparse.length, 0.35), sparse.s, start_s])
    samples = samples[np.argsort((samples - start_s) % sparse.length)]
    px, py, _ = sparse.cartesian(samples)
    return px, py


def transfer_pavement_widths(reference, x, y):
    """Intersect new centerline normals with the image-derived boundary ribbons.

    The user line can be off the middle of the asphalt. Keep asymmetric widths
    instead of shifting/widening the road to make that line look centered.
    """
    candidate = Track(x, y, np.full(len(x), 2.6), np.full(len(x), 2.6))
    centers = np.column_stack((x, y))
    normals = np.column_stack((-np.sin(candidate.psi), np.cos(candidate.psi)))
    output = []
    for edge, sign in zip(reference.edges(), (1, -1), strict=True):
        a = edge
        segment = np.roll(edge, -1, axis=0) - edge
        q = a[None, :, :] - centers[:, None, :]
        n = normals[:, None, :]
        denominator = n[..., 0] * segment[None, :, 1] - n[..., 1] * segment[None, :, 0]
        valid = abs(denominator) > 1e-10
        denominator = np.where(valid, denominator, 1)
        distance = (q[..., 0] * segment[None, :, 1] - q[..., 1] * segment[None, :, 0]) / denominator
        along = (q[..., 0] * n[..., 1] - q[..., 1] * n[..., 0]) / denominator
        valid &= (along >= 0) & (along <= 1) & (distance * sign > 0) & (distance * sign < 8)
        width = np.min(np.where(valid, distance * sign, np.inf), axis=1)
        if not np.isfinite(width).all() or width.min() < 0.2:
            raise ValueError("user centerline leaves the image-derived pavement; inspect before rebuilding")
        output.append(width)
    return output[1], output[0]  # right, left


def build(directory=ROOT / "tracks/purdue_gp"):
    source = json.loads((directory / "aerial_source.json").read_text())
    image_meta = source["export"]
    extent = image_meta["extent"]
    datum = (40.43775, -86.94475, 0.0)

    def pixel_to_enu(px, py):
        mx = extent["xmin"] + np.asarray(px) / image_meta["width"] * (extent["xmax"] - extent["xmin"])
        my = extent["ymax"] - np.asarray(py) / image_meta["height"] * (extent["ymax"] - extent["ymin"])
        lon = np.degrees(mx / 6378137.0)
        lat = np.degrees(2 * np.arctan(np.exp(my / 6378137.0)) - np.pi / 2)
        return geodetic_to_enu(lat, lon, np.zeros_like(lon), datum)[:2]

    pixels = np.loadtxt(directory / "centerline_pixels.csv", delimiter=",")
    raw_user = ET.parse(directory / "user_track.kml").find(".//{*}coordinates").text
    user_ll = np.array([[float(v) for v in p.split(",")] for p in raw_user.split()])
    if np.array_equal(user_ll[0], user_ll[-1]):
        user_ll = user_ll[:-1]
    if len(user_ll) < 3 or not np.isfinite(user_ll).all():
        raise ValueError("KML must contain at least three finite coordinate points")
    ux, uy, _ = geodetic_to_enu(user_ll[:, 1], user_ll[:, 0], user_ll[:, 2], datum)

    def area(x, y):
        return float(np.sum(x * np.roll(y, -1) - y * np.roll(x, -1)))

    east, north = pixel_to_enu(*pixels.T)
    if area(east, north) < 0:
        # User explicitly confirmed counterclockwise travel. Positive signed
        # area in ENU is CCW; keep the same visible start/finish point.
        pixels = np.vstack((pixels[:1], pixels[:0:-1]))
        east, north = pixel_to_enu(*pixels.T)
    # Manual center picks follow the main course, not the pit-lane shortcut.
    sparse = Track(east, north, np.full(len(east), 2.6), np.full(len(east), 2.6))
    sample_s = np.linspace(0, sparse.length, int(np.ceil(sparse.length / 0.5)), endpoint=False)
    x, y, _ = sparse.cartesian(sample_s)
    # Estimate paint-line locations perpendicular to the manually traced road.
    # Occluded/unreliable sections fall back to 2.6 m per side. These widths are
    # image estimates, not surveys; preserve the measured pixel trace separately.
    gray = np.asarray(Image.open(directory / "aerial.png").convert("L"), dtype=float)
    direction = np.roll(pixels, -1, axis=0) - np.roll(pixels, 1, axis=0)
    normals = np.column_stack((direction[:, 1], -direction[:, 0]))
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    offsets = np.arange(10, 29, 0.5)
    widths = []
    fallback_counts = []
    for sign in (-1, 1):
        profiles = []
        for shift in (0, -3, 3):
            points = pixels[:, None, :] + sign * normals[:, None, :] * (offsets + shift)[None, :, None]
            profiles.append(map_coordinates(gray, [points[..., 1], points[..., 0]], order=1, mode="nearest"))
        score = profiles[0] - (profiles[1] + profiles[2]) / 2 - 0.6 * abs(offsets - 18)[None, :]
        index = score.argmax(axis=1)
        valid = score[np.arange(len(pixels)), index] > 12
        chosen = offsets[index]
        edge_px = pixels + sign * normals * chosen[:, None]
        ex, ey = pixel_to_enu(*edge_px.T)
        half = np.hypot(ex - east, ey - north)
        half = np.where(valid, np.clip(half, 1.9, 3.5), 2.6)
        half = median_filter(half, size=5, mode="wrap")
        widths.append(sparse._interp_along(sample_s, half))
        fallback_counts.append(int((~valid).sum()))
    pavement = Track(x, y, widths[0], widths[1])
    x, y = user_centerline(ux, uy, (x[0], y[0]))
    right, left = transfer_pavement_widths(pavement, x, y)
    track = Track(x, y, right, left, track_id="purdue_gp_kml_provisional")
    ring = ET.parse(directory / "region.kml").find(".//{*}coordinates").text
    boundary = np.array([[float(v) for v in point.split(",")] for point in ring.split()])
    bx, by, _ = geodetic_to_enu(boundary[:, 1], boundary[:, 0], boundary[:, 2], datum)
    ax, ay = pixel_to_enu([0, image_meta["width"]], [image_meta["height"], 0])
    track.meta = {
        "track_id": track.track_id,
        "closed": True,
        "datum": dict(zip(("latitude", "longitude", "height"), datum, strict=True)),
        "survey_date": None,
        "geometry_status": "user_revised_KML_centerline",
        "width_status": "asymmetric_distances_to_image_estimated_pavement",
        "elevation_status": "flat_unmeasured",
        "direction": "counterclockwise",
        "direction_status": "user_confirmed_2026-09-22",
        "start_finish_status": "traced_at_visible_checkered_line",
        "accuracy_validated": False,
        "region_enu": np.column_stack((bx, by)).tolist(),
        "user_points_enu": np.column_stack((ux, uy)).tolist(),
        "source": {
            "url": source["service"],
            "retrieved": source["retrieved"],
            "capture_year": source["capture_year"],
            "raster_id": source["raster_id"],
            "attribution": "Indiana Geographic Information Office",
            "license": "CC0",
            "aerial_sha256": hashlib.sha256((directory / "aerial.png").read_bytes()).hexdigest(),
            "trace_sha256": hashlib.sha256((directory / "centerline_pixels.csv").read_bytes()).hexdigest(),
            "centerline_file": "user_track.kml",
            "centerline_sha256": hashlib.sha256((directory / "user_track.kml").read_bytes()).hexdigest(),
        },
        "aerial": {"file": "aerial.png", "bounds_enu": [float(ax[0]), float(ay[0]), float(ax[1]), float(ay[1])]},
        "width_fallback_control_points_right_left": fallback_counts,
        "description": "Revised user KML centerline; pavement edges estimated from the Indiana orthophoto.",
    }
    # Track.save fills a generic start line. Supply the actual traced cross-section.
    left, right = track.width_at(0)
    lx, ly, _ = track.cartesian(0, left)
    rx, ry, _ = track.cartesian(0, -right)
    track.meta["start_finish"] = {"a": [float(lx[0]), float(ly[0])], "b": [float(rx[0]), float(ry[0])]}
    _, user_offsets = track.frenet(ux, uy)
    track.meta["user_point_comparison"] = {
        "count": len(ux),
        "median_centerline_offset_m": float(np.median(np.abs(user_offsets))),
        "max_centerline_offset_m": float(np.max(np.abs(user_offsets))),
        "interpretation": "Source waypoint interpolation check; not survey residuals.",
        "kml_sha256": hashlib.sha256((directory / "user_track.kml").read_bytes()).hexdigest(),
    }
    track.save(directory)
    print(
        f"{len(track.x)} points; {track.length:.2f} m; width {min(track.w_left + track.w_right):.2f} "
        f"to {max(track.w_left + track.w_right):.2f} m; image trace, not survey"
    )
    return track


if __name__ == "__main__":
    build()
