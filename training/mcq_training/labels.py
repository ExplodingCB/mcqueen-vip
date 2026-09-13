"""Label conventions, and tier A labels at zero human cost: the surveyed track
projected into the kart's own camera through the logged pose.

A label is a single-channel PNG the size of the frame with one class index per
pixel: 0 background, 1 pavement, 2 kart, 255 ignore. The ignore value covers
the band around each edge where the survey and the pose cannot vouch for the
pixel, the kart's own bodywork, and anything a reviewer struck out. The loss
and the metrics skip it.

Projection: the two track edges are closed rings in the map frame (or an open
strip for a local track). They are moved into ``base_link`` with the pose,
clipped against the camera's near plane in the optical frame, projected with
the pinhole model and rasterized. For a closed track the pavement is the
exclusive-or of the two projected rings, which is exact under the homography
between the ground plane and the image. The ignore band is the difference
between the rings pushed outward and pulled inward by a margin that grows with
distance to absorb heading error.
"""

from __future__ import annotations

import numpy as np
from PIL import Image, ImageDraw

from mcq_sim.track import Track
from mcq_training.camera import CameraModel, map_to_base

CLASS_NAMES = ("background", "pavement", "kart")
BACKGROUND, PAVEMENT, KART = 0, 1, 2
IGNORE = 255
NUM_CLASSES = len(CLASS_NAMES)
PALETTE = {BACKGROUND: (0, 0, 0), PAVEMENT: (96, 96, 96), KART: (220, 40, 40), IGNORE: (255, 200, 0)}


# ------------------------------------------------------------------- label I/O
def save_label(path, label: np.ndarray) -> None:
    Image.fromarray(np.asarray(label, dtype=np.uint8), mode="L").save(path)


def load_label(path) -> np.ndarray:
    with Image.open(path) as im:
        return np.asarray(im.convert("L"), dtype=np.uint8)


def colorize(label: np.ndarray) -> np.ndarray:
    out = np.zeros(label.shape + (3,), dtype=np.uint8)
    for value, color in PALETTE.items():
        out[label == value] = color
    return out


def overlay(image: np.ndarray, label: np.ndarray, alpha: float = 0.45) -> np.ndarray:
    """Blend the label colors over an RGB frame for review previews."""
    color = colorize(label).astype(float)
    base = np.asarray(image, dtype=float)
    mask = (label != BACKGROUND)[..., None]
    out = np.where(mask, (1 - alpha) * base + alpha * color, base)
    return np.clip(out, 0, 255).astype(np.uint8)


# ------------------------------------------------------------------ geometry
def offset_edges(track: Track, offset: np.ndarray | float) -> tuple[np.ndarray, np.ndarray]:
    """Left and right edge polylines in map with the half-widths changed by
    ``offset`` (scalar or one value per centerline point; positive widens)."""
    offset = np.broadcast_to(np.asarray(offset, dtype=float), track.x.shape)
    nx, ny = -np.sin(track.psi), np.cos(track.psi)
    wl = np.maximum(track.w_left + offset, 0.02)
    wr = np.maximum(track.w_right + offset, 0.02)
    left = np.column_stack([track.x + wl * nx, track.y + wl * ny])
    right = np.column_stack([track.x - wr * nx, track.y - wr * ny])
    return left, right


def clip_polygon_plane(poly: np.ndarray, normal: np.ndarray, d: float) -> np.ndarray:
    """Sutherland-Hodgman clip of a closed polygon (n, k) to the half space
    normal . p >= d."""
    poly = np.asarray(poly, dtype=float)
    if len(poly) == 0:
        return poly
    out = []
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        fa, fb = float(normal @ a) - d, float(normal @ b) - d
        if fa >= 0:
            out.append(a)
        if (fa >= 0) != (fb >= 0):
            t = fa / (fa - fb)
            out.append(a + t * (b - a))
    return np.asarray(out, dtype=float).reshape(-1, poly.shape[1])


def clip_polygon_rect(poly: np.ndarray, xmin: float, ymin: float, xmax: float, ymax: float) -> np.ndarray:
    for normal, d in (((1.0, 0.0), xmin), ((-1.0, 0.0), -xmax), ((0.0, 1.0), ymin), ((0.0, -1.0), -ymax)):
        poly = clip_polygon_plane(poly, np.array(normal), d)
        if len(poly) < 3:
            return poly[:0]
    return poly


def project_polygon(poly_base: np.ndarray, cam: CameraModel, z_near: float = 0.05) -> np.ndarray:
    """Closed ``base_link`` polygon (n, 3) to a closed pixel polygon, clipped to
    the near plane and to a rectangle a few images wide around the frame."""
    opt = cam.base_to_optical(poly_base)
    opt = clip_polygon_plane(opt, np.array([0.0, 0.0, 1.0]), z_near)
    if len(opt) < 3:
        return np.zeros((0, 2))
    uv, _ = cam.project_optical(opt)
    w, h = cam.width, cam.height
    return clip_polygon_rect(uv, -2.0 * w, -2.0 * h, 3.0 * w, 3.0 * h)


def rasterize(poly_uv: np.ndarray, width: int, height: int) -> np.ndarray:
    if len(poly_uv) < 3:
        return np.zeros((height, width), dtype=bool)
    im = Image.new("L", (width, height), 0)
    ImageDraw.Draw(im).polygon([(float(u), float(v)) for u, v in poly_uv], fill=1)
    return np.asarray(im, dtype=bool)


def _window(track: Track, s0: float, back: float, ahead: float) -> np.ndarray:
    """Indices of centerline points from ``back`` behind s0 to ``ahead`` in front."""
    if track.closed:
        rel = np.mod(track.s - s0 + back, track.length) - back
        idx = np.where((rel >= -back) & (rel <= ahead))[0]
        return idx[np.argsort(rel[idx])]
    return np.where((track.s >= s0 - back) & (track.s <= s0 + ahead))[0]


def pavement_mask(
    track: Track,
    pose: tuple[float, float, float],
    cam: CameraModel,
    offset: np.ndarray | float = 0.0,
    range_m: float | None = None,
    back: float = 5.0,
    z_near: float = 0.05,
) -> np.ndarray:
    """Boolean pavement mask for the kart at ``pose`` = (x, y, yaw) in map.
    Closed tracks with ``range_m`` None use the full rings; otherwise a strip
    from ``back`` behind the kart to ``range_m`` ahead."""
    x, y, yaw = pose
    left, right = offset_edges(track, offset)

    def to_base3(pts):
        b = map_to_base(pts, x, y, yaw)
        return np.column_stack([b, np.zeros(len(b))])

    if track.closed and range_m is None:
        ml = rasterize(project_polygon(to_base3(left), cam, z_near), cam.width, cam.height)
        mr = rasterize(project_polygon(to_base3(right), cam, z_near), cam.width, cam.height)
        return ml ^ mr
    s0 = float(track.frenet(x, y)[0][0])
    idx = _window(track, s0, back, range_m if range_m is not None else track.length)
    if len(idx) < 2:
        return np.zeros((cam.height, cam.width), dtype=bool)
    strip = np.vstack([left[idx], right[idx][::-1]])
    return rasterize(project_polygon(to_base3(strip), cam, z_near), cam.width, cam.height)


def render_label(
    track: Track,
    pose: tuple[float, float, float],
    cam: CameraModel,
    band_m: float = 0.15,
    yaw_sigma: float = np.radians(0.5),
    vehicle_mask: np.ndarray | None = None,
    range_m: float | None = None,
) -> np.ndarray:
    """Projected label: pavement inside the edges pulled in by the band,
    ignore between that and the edges pushed out by the band, background
    elsewhere. The band grows with distance from the kart by tan(yaw_sigma)."""
    x, y, yaw = pose
    dist = np.hypot(track.x - x, track.y - y)
    band = band_m + dist * np.tan(yaw_sigma)
    outer = pavement_mask(track, pose, cam, +band, range_m)
    inner = pavement_mask(track, pose, cam, -band, range_m)
    label = np.full((cam.height, cam.width), BACKGROUND, dtype=np.uint8)
    label[outer] = IGNORE
    label[inner & outer] = PAVEMENT
    if vehicle_mask is not None:
        label[np.asarray(vehicle_mask, dtype=bool)] = IGNORE
    return label
