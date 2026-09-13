import numpy as np
import pytest

from mcq_training.camera import CameraModel, map_to_base


def test_optical_axes_and_horizon(cam):
    # A ground point straight ahead lands on the principal column, below the horizon;
    # a near one (depression angle above the 10 deg pitch) lands below the principal row.
    uv, depth = cam.project(np.array([[10.0, 0.0, 0.0], [2.0, 0.0, 0.0]]))
    assert np.all(depth > 0)
    assert np.all(np.abs(uv[:, 0] - cam.cx) < 1e-6)
    assert cam.horizon_row() < uv[0, 1] < cam.cy < uv[1, 1]
    # Left in base_link is left in the image (smaller u).
    uv_l, _ = cam.project(np.array([[10.0, 1.0, 0.0]]))
    assert uv_l[0, 0] < cam.cx
    # The horizon sits above the principal row for a camera pitched down.
    assert cam.horizon_row() < cam.cy
    assert cam.horizon_row() == pytest.approx(cam.cy - cam.fy * np.tan(cam.pitch), abs=1e-3)


def test_ground_roundtrip(cam):
    rng = np.random.default_rng(0)
    pts = np.column_stack([rng.uniform(2.0, 40.0, 200), rng.uniform(-8.0, 8.0, 200), np.zeros(200)])
    uv, depth = cam.project(pts)
    assert np.all(depth > 0)
    back, valid = cam.ground_from_pixels(uv)
    assert valid.all()
    assert np.abs(back - pts).max() < 1e-6
    # Pixels above the horizon do not hit the ground.
    _, valid = cam.ground_from_pixels(np.array([[cam.cx, cam.horizon_row() - 5.0]]))
    assert not valid[0]


def test_frame_transforms_are_inverse(cam):
    rng = np.random.default_rng(1)
    pts = rng.normal(size=(50, 3)) * 10
    assert np.allclose(cam.optical_to_base(cam.base_to_optical(pts)), pts)


def test_scaled_keeps_rays(cam):
    small = cam.scaled(640, 400)
    uv, _ = cam.project(np.array([[12.0, -1.5, 0.0]]))
    uv_s, _ = small.project(np.array([[12.0, -1.5, 0.0]]))
    assert np.allclose(uv_s, uv * 0.5)


def test_yaml_roundtrip_and_fov(tmp_path):
    cam = CameraModel.from_dict({"width": 1280, "height": 800, "hfov_deg": 100.0, "z": 0.7, "pitch_deg": 8.0})
    assert cam.pitch == pytest.approx(np.radians(8.0))
    assert cam.fx == pytest.approx(0.5 * 1280 / np.tan(np.radians(50.0)))
    cam.save(tmp_path / "cam.yaml")
    again = CameraModel.load(tmp_path / "cam.yaml")
    assert again == cam


def test_map_to_base():
    # Kart at (10, 5) heading +y: a map point 1 m further along +y is 1 m ahead.
    out = map_to_base(np.array([[10.0, 6.0], [9.0, 5.0]]), 10.0, 5.0, np.pi / 2)
    assert np.allclose(out, [[1.0, 0.0], [0.0, 1.0]])
