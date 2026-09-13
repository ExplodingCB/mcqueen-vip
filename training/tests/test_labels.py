import numpy as np

from mcq_sim.track import Track
from mcq_training import labels as L


def _pose_on_oval(oval, s, d=0.0):
    x, y, psi = oval.cartesian(s, d)
    return float(x[0]), float(y[0]), float(psi[0])


def test_projected_label_matches_analytic_edges(oval, cam):
    pose = _pose_on_oval(oval, 10.0)  # lower straight, heading +x, on the centerline
    label = L.render_label(oval, pose, cam, band_m=0.15, yaw_sigma=0.0)
    assert label.shape == (cam.height, cam.width)
    # Nothing above the horizon is pavement or ignore.
    top = int(np.floor(cam.horizon_row())) - 1
    assert np.all(label[:top] == L.BACKGROUND)
    # On the row 6 m ahead the pavement spans the projected +-2.5 m edges of the 5 m wide oval
    # (the bottom row only sees about +-1 m either side, all pavement).
    v = int(round(cam.project(np.array([[6.0, 0.0, 0.0]]))[0][0, 1]))
    ground, ok = cam.ground_from_pixels(np.array([[cam.cx, v]]))
    assert ok[0]
    x0 = ground[0, 0]
    assert np.all(label[cam.height - 1] == L.PAVEMENT)
    (u_left, _), (u_right, _) = cam.project(np.array([[x0, 2.5, 0.0], [x0, -2.5, 0.0]]))[0]
    (u_left_in, _), (u_right_in, _) = cam.project(np.array([[x0, 2.5 - 0.15, 0.0], [x0, -2.5 + 0.15, 0.0]]))[0]
    (u_left_out, _), (u_right_out, _) = cam.project(np.array([[x0, 2.5 + 0.15, 0.0], [x0, -2.5 - 0.15, 0.0]]))[0]
    row = label[v]
    assert row[int(cam.cx)] == L.PAVEMENT
    assert row[int(u_left_in) + 3] == L.PAVEMENT and row[int(u_right_in) - 3] == L.PAVEMENT
    assert row[int(u_left)] == L.IGNORE and row[int(u_right)] == L.IGNORE
    assert row[int(u_left_out) - 3] == L.BACKGROUND and row[int(u_right_out) + 3] == L.BACKGROUND
    frac = np.mean(label == L.PAVEMENT)
    assert 0.05 < frac < 0.6


def test_lateral_shift_moves_pavement(oval, cam):
    centre = L.render_label(oval, _pose_on_oval(oval, 10.0, 0.0), cam, yaw_sigma=0.0)
    left = L.render_label(oval, _pose_on_oval(oval, 10.0, 1.0), cam, yaw_sigma=0.0)
    v = int(round(cam.project(np.array([[6.0, 0.0, 0.0]]))[0][0, 1]))
    mean_c = np.mean(np.where(centre[v] == L.PAVEMENT)[0])
    mean_l = np.mean(np.where(left[v] == L.PAVEMENT)[0])
    assert mean_l > mean_c + 20  # kart moved left, so the track slides right in the image


def test_ring_and_strip_agree_near_the_kart(oval, cam):
    pose = _pose_on_oval(oval, 40.0)
    ring = L.pavement_mask(oval, pose, cam)
    strip = L.pavement_mask(oval, pose, cam, range_m=30.0)
    # The ring is the exclusive-or of two polygons and the strip is one polygon, so
    # they can differ by a pixel along shared edges; nowhere else, and never below 10 m.
    assert np.sum(strip & ~ring) < 1e-3 * np.sum(ring)
    uv, _ = cam.project(np.array([[10.0, 0.0, 0.0]]))
    v10 = int(uv[0, 1])
    assert np.sum(ring[v10:] != strip[v10:]) <= 2 * (cam.height - v10)


def test_open_track_from_bounds(cam):
    left = np.column_stack([np.linspace(-5, 40, 46), np.full(46, 2.0)])
    right = np.column_stack([np.linspace(-5, 40, 46), np.full(46, -2.0)])
    track = Track.from_bounds(left, right, n=46)
    label = L.render_label(track, (0.0, 0.0, 0.0), cam, yaw_sigma=0.0, range_m=30.0)
    assert np.mean(label == L.PAVEMENT) > 0.05
    v = cam.height - 1
    assert label[v, int(cam.cx)] == L.PAVEMENT


def test_yaw_band_grows_with_distance(oval, cam):
    pose = _pose_on_oval(oval, 10.0)
    tight = L.render_label(oval, pose, cam, band_m=0.15, yaw_sigma=0.0)
    loose = L.render_label(oval, pose, cam, band_m=0.15, yaw_sigma=np.radians(1.0))
    assert np.sum(loose == L.IGNORE) > np.sum(tight == L.IGNORE)
    assert np.sum(loose == L.PAVEMENT) < np.sum(tight == L.PAVEMENT)


def test_vehicle_mask_and_png_roundtrip(oval, cam, tmp_path):
    hood = np.zeros((cam.height, cam.width), dtype=bool)
    hood[-40:] = True
    label = L.render_label(oval, _pose_on_oval(oval, 10.0), cam, vehicle_mask=hood)
    assert np.all(label[-40:] == L.IGNORE)
    L.save_label(tmp_path / "l.png", label)
    assert np.array_equal(L.load_label(tmp_path / "l.png"), label)
    rgb = L.overlay(np.zeros((cam.height, cam.width, 3), dtype=np.uint8), label)
    assert rgb.shape == (cam.height, cam.width, 3)


def test_clip_polygon():
    square = np.array([[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]])
    half = L.clip_polygon_plane(square, np.array([1.0, 0.0]), 0.0)
    assert len(half) == 4 and half[:, 0].min() >= 0.0
    assert len(L.clip_polygon_plane(square, np.array([1.0, 0.0]), 2.0)) == 0
    assert len(L.clip_polygon_rect(square * 10, -1, -1, 1, 1)) == 4
