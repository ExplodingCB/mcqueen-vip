"""Synthetic survey: drive both edges of the oval with RTK-like noise and
uneven sampling, then recover the track."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import survey  # noqa: E402

from mcq_sim.track import Track  # noqa: E402


def make_drive(edge: np.ndarray, rng, noise=0.02, overlap_pts=15, standstill_pts=20):
    """A lap along an edge: uneven spacing, RTK noise, a standstill at the
    start and an overlap past the finish."""
    seg = np.hypot(*np.diff(np.vstack([edge, edge[:1]]), axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    steps = rng.uniform(0.2, 1.4, int(2 * s[-1]) + 10)  # metres between samples, more than a lap
    u = np.cumsum(steps)
    u = u[u < s[-1]]
    closed = np.vstack([edge, edge[:1]])
    pts = np.column_stack([np.interp(u, s, closed[:, 0]), np.interp(u, s, closed[:, 1])])
    pts = np.vstack([pts, pts[:overlap_pts]])
    pts = np.vstack([np.repeat(pts[:1], standstill_pts, axis=0), pts])
    return pts + rng.normal(0.0, noise, pts.shape)


@pytest.fixture(scope="module")
def oval():
    return Track.synthetic_oval()


def test_enu_conversion_scale():
    datum = (40.4237, -86.9212, 190.0)
    e, n, u = survey.geodetic_to_enu(40.4247, -86.9212, 190.0, datum)
    assert n == pytest.approx(111.0, rel=0.01) and abs(e) < 0.01 and abs(u) < 0.01
    e, n, u = survey.geodetic_to_enu(40.4237, -86.9202, 190.0, datum)
    assert e == pytest.approx(84.87, rel=0.01) and abs(n) < 0.01
    e, n, u = survey.geodetic_to_enu(40.4237, -86.9212, 200.0, datum)
    assert u == pytest.approx(10.0, abs=1e-3)


def test_recovers_oval_from_noisy_edge_drives(oval, tmp_path):
    rng = np.random.default_rng(7)
    left, right = oval.edges()
    left_drive = make_drive(left, rng)
    right_drive = make_drive(right, rng)
    track = survey.build_track(left_drive, right_drive, track_id="recovered")

    assert track.closed
    assert track.length == pytest.approx(oval.length, rel=0.01)
    _, d = oval.frenet(track.x, track.y)
    assert np.abs(d).max() < 0.10
    assert np.abs(track.w_left - 2.5).max() < 0.10
    assert np.abs(track.w_right - 2.5).max() < 0.10
    assert np.abs(track.kappa).max() < 1 / 15.0 * 1.3

    # Round trip through the CLI with csv inputs in the map frame.
    for name, drive in (("left", left_drive), ("right", right_drive)):
        np.savetxt(tmp_path / f"{name}.csv", drive, delimiter=",", header="x,y", comments="")
    out = tmp_path / "recovered"
    assert (
        survey.main(["--left", str(tmp_path / "left.csv"), "--right", str(tmp_path / "right.csv"), "--out", str(out)])
        == 0
    )
    loaded = Track.load(out)
    assert loaded.track_id == "recovered" and abs(loaded.length - oval.length) < 3.0


def test_lat_lon_input_uses_datum(oval, tmp_path):
    rng = np.random.default_rng(3)
    left, right = oval.edges()
    datum = (40.4237, -86.9212, 190.0)
    # Invert ENU to lat/lon with the flat-earth scale from the conversion itself.
    m_per_deg_lat = survey.geodetic_to_enu(datum[0] + 1e-3, datum[1], datum[2], datum)[1] / 1e-3
    m_per_deg_lon = survey.geodetic_to_enu(datum[0], datum[1] + 1e-3, datum[2], datum)[0] / 1e-3
    for name, edge in (("left", left), ("right", right)):
        drive = make_drive(edge, rng, noise=0.0)
        rows = np.column_stack([datum[0] + drive[:, 1] / m_per_deg_lat, datum[1] + drive[:, 0] / m_per_deg_lon])
        np.savetxt(tmp_path / f"{name}.csv", rows, delimiter=",", header="lat,lon", comments="", fmt="%.9f")
    with pytest.raises(ValueError):
        survey.read_drive_csv(tmp_path / "left.csv")
    xy = survey.read_drive_csv(tmp_path / "left.csv", datum)
    assert np.abs(xy[:, 0]).max() < 50.0 and np.abs(xy[:, 1]).max() < 20.0
    out = tmp_path / "geo"
    survey.main(
        [
            "--left",
            str(tmp_path / "left.csv"),
            "--right",
            str(tmp_path / "right.csv"),
            "--out",
            str(out),
            "--datum",
            *map(str, datum),
        ]
    )
    loaded = Track.load(out)
    assert loaded.meta["datum"]["latitude"] == datum[0]
    assert abs(loaded.length - oval.length) < 3.0


def test_swapped_edges_are_rejected(oval):
    rng = np.random.default_rng(1)
    left, right = oval.edges()
    with pytest.raises(ValueError, match="swap"):
        survey.build_track(make_drive(right, rng), make_drive(left, rng))


def test_open_drive_is_rejected(oval):
    left, _ = oval.edges()
    with pytest.raises(ValueError, match="closed lap"):
        survey.build_track(left[: len(left) // 2], left[: len(left) // 2])
