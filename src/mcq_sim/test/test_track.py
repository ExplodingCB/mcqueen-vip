import numpy as np
import pytest

from mcq_sim.track import Raceline, Track, base_to_map


def test_oval_geometry(oval):
    expected = 2 * 60.0 + 2 * np.pi * 15.0
    assert oval.length == pytest.approx(expected, rel=0.01)
    assert oval.closed
    # Straights are flat, arcs have curvature 1/R (left turns, positive).
    s = np.array([10.0, 30.0])
    assert np.allclose(oval.curvature_at(s), 0.0, atol=2e-3)
    s_arc = 60.0 + np.pi * 15.0 / 2
    assert oval.curvature_at(s_arc)[0] == pytest.approx(1 / 15.0, rel=0.03)


def test_frenet_roundtrip(oval):
    rng = np.random.default_rng(1)
    s = rng.uniform(0, oval.length, 200)
    d = rng.uniform(-2.0, 2.0, 200)
    x, y, _ = oval.cartesian(s, d)
    s2, d2 = oval.frenet(x, y)
    ds = np.abs((s2 - s + oval.length / 2) % oval.length - oval.length / 2)
    assert ds.max() < 0.1  # chord projection versus spline arc length
    assert np.abs(d2 - d).max() < 0.03


def test_frenet_sign_convention(oval):
    # Positive d is left of the direction of travel: on the lower straight the
    # kart heads +x, so a point at larger y is to the left.
    s, d = oval.frenet(0.0, -15.0 + 1.0)
    assert d[0] == pytest.approx(1.0, abs=1e-6)
    s, d = oval.frenet(0.0, -15.0 - 1.0)
    assert d[0] == pytest.approx(-1.0, abs=1e-6)


def test_edges_and_geofence(oval):
    inside = oval.distance_to_edge(0.0, -15.0)[0]
    assert inside == pytest.approx(2.5, abs=1e-6)
    assert oval.inside(0.0, -15.0 + 2.4)
    assert not oval.inside(0.0, -15.0 + 2.6)
    assert oval.inside(0.0, -15.0 + 2.9, margin=0.5)
    assert not oval.inside(0.0, -15.0 + 3.1, margin=0.5)
    left, right = oval.edges()
    assert np.allclose(oval.distance_to_edge(left[:, 0], left[:, 1]), 0.0, atol=0.02)
    assert np.allclose(oval.distance_to_edge(right[:, 0], right[:, 1]), 0.0, atol=0.02)


def test_csv_and_directory_roundtrip(oval, tmp_path):
    oval.save(tmp_path / "t")
    header = (tmp_path / "t" / "track.csv").read_text().splitlines()[0]
    assert header.startswith("# x_m, y_m, w_tr_right_m, w_tr_left_m")
    loaded = Track.load(tmp_path / "t")
    assert loaded.track_id == "synthetic_oval"
    assert loaded.length == pytest.approx(oval.length, abs=1e-3)
    assert np.allclose(loaded.x, oval.x, atol=1e-3)
    assert loaded.meta["datum"]["latitude"] == 0.0


def test_bounds_ahead_and_local_track(oval):
    s0 = 55.0
    x, y, psi = (float(v[0]) for v in oval.cartesian(s0, 0.0))
    left, right = oval.bounds_ahead(x, y, psi, 25.0, 1.0, back=5.0)
    # In base_link the kart is at the origin: edges are about +-2.5 m in y at x = 0.
    i0 = np.argmin(np.abs(left[:, 0]))
    assert left[i0, 1] == pytest.approx(2.5, abs=0.05)
    assert right[i0, 1] == pytest.approx(-2.5, abs=0.05)
    assert left[:, 0].min() < -4.0 and left[:, 0].max() > 15.0  # window curves away
    local = Track.from_bounds(base_to_map(left, x, y, psi), base_to_map(right, x, y, psi))
    assert not local.closed
    _, d_center = oval.frenet(local.x, local.y)
    assert np.abs(d_center).max() < 0.05
    assert local.heading_at(5.0)[0] == pytest.approx(psi, abs=0.02)


def test_shifted(oval):
    moved = oval.shifted(0.0, 2.0)
    assert moved.distance_to_edge(0.0, -15.0)[0] == pytest.approx(0.5, abs=1e-3)


def test_raceline_csv(tmp_path):
    path = tmp_path / "raceline.csv"
    rows = ["# s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2"]
    for i in range(10):
        rows.append(f"{i:.1f}; {i:.1f}; 0.0; 0.0; 0.0; 5.0; 0.0")
    path.write_text("\n".join(rows))
    rl = Raceline.from_csv(path)
    assert len(rl.s) == 10 and rl.v[3] == 5.0


def test_rejects_bad_inputs(tmp_path):
    with pytest.raises(ValueError):
        Track(np.array([0.0, 1.0]), np.array([0.0, 0.0]), np.ones(2), np.ones(2))
    bad = tmp_path / "bad.csv"
    bad.write_text("0,0,1\n1,0,1\n2,0,1\n")
    with pytest.raises(ValueError):
        Track.from_csv(bad)
