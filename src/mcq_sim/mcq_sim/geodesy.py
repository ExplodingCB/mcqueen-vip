"""WGS84 to local tangent plane and back.

The map frame is an east-north-up plane anchored at a per-track datum
(docs/02-architecture.md section 3), so anything that speaks to a GNSS
receiver converts here: the survey tool reads recorded fixes into map
coordinates, and the simulator's GNSS model writes map coordinates back out as
latitude and longitude. One implementation so the two cannot disagree.

Both directions go through ECEF and are exact to numerical precision. The
obvious shortcut for the inverse, scaling north and east by the radii of
curvature at the datum, warps by 3 cm at 500 m from the datum at this latitude,
because a displacement in east has to be scaled by the cosine of the point's
latitude and not the datum's. That is larger than an RTK fix and a third of the
Phase 1 pose budget, so it is not worth the six lines it saves.
"""

from __future__ import annotations

import numpy as np

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def geodetic_to_ecef(lat_deg, lon_deg, h):
    lat, lon = np.radians(lat_deg), np.radians(lon_deg)
    n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * np.sin(lat) ** 2)
    x = (n + h) * np.cos(lat) * np.cos(lon)
    y = (n + h) * np.cos(lat) * np.sin(lon)
    z = (n * (1.0 - WGS84_E2) + h) * np.sin(lat)
    return x, y, z


def geodetic_to_enu(lat_deg, lon_deg, h, datum):
    """East, north, up in metres relative to datum = (lat, lon, height)."""
    lat0, lon0, h0 = datum
    x0, y0, z0 = geodetic_to_ecef(lat0, lon0, h0)
    x, y, z = geodetic_to_ecef(np.asarray(lat_deg, float), np.asarray(lon_deg, float), np.asarray(h, float))
    dx, dy, dz = x - x0, y - y0, z - z0
    la, lo = np.radians(lat0), np.radians(lon0)
    east = -np.sin(lo) * dx + np.cos(lo) * dy
    north = -np.sin(la) * np.cos(lo) * dx - np.sin(la) * np.sin(lo) * dy + np.cos(la) * dz
    up = np.cos(la) * np.cos(lo) * dx + np.cos(la) * np.sin(lo) * dy + np.sin(la) * dz
    return east, north, up


def ecef_to_geodetic(x, y, z):
    """Latitude and longitude in degrees, height in metres. Iterates on the
    latitude, which converges to double precision in a few passes away from the
    poles; the kart is not going to either pole."""
    x, y, z = np.asarray(x, float), np.asarray(y, float), np.asarray(z, float)
    p = np.hypot(x, y)
    lon = np.arctan2(y, x)
    lat = np.arctan2(z, p * (1.0 - WGS84_E2))
    for _ in range(5):
        n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * np.sin(lat) ** 2)
        h = p / np.cos(lat) - n
        lat = np.arctan2(z, p * (1.0 - WGS84_E2 * n / (n + h)))
    n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * np.sin(lat) ** 2)
    h = p / np.cos(lat) - n
    return np.degrees(lat), np.degrees(lon), h


def enu_to_geodetic(east, north, up, datum):
    """Latitude and longitude in degrees and height in metres from a local
    east-north-up offset about datum = (lat, lon, height). Inverse of
    `geodetic_to_enu` to numerical precision."""
    lat0, lon0, h0 = datum
    east = np.asarray(east, float)
    north = np.asarray(north, float)
    up = np.asarray(up, float)
    la, lo = np.radians(lat0), np.radians(lon0)
    # Transpose of the ECEF-to-ENU rotation.
    dx = -np.sin(lo) * east - np.sin(la) * np.cos(lo) * north + np.cos(la) * np.cos(lo) * up
    dy = np.cos(lo) * east - np.sin(la) * np.sin(lo) * north + np.cos(la) * np.sin(lo) * up
    dz = np.cos(la) * north + np.sin(la) * up
    x0, y0, z0 = geodetic_to_ecef(lat0, lon0, h0)
    return ecef_to_geodetic(x0 + dx, y0 + dy, z0 + dz)
