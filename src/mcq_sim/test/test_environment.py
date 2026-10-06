import xml.etree.ElementTree as ET
from dataclasses import asdict
from pathlib import Path

import numpy as np
import pytest

from mcq_sim.camera import checked_mask
from mcq_sim.environment import Simulator, evaluate, load_kart
from mcq_sim.track import Track

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def purdue():
    return Track.load(ROOT / "tracks/purdue_gp")


def test_supplied_dimensions_preserved_without_conflating_track_and_body_width():
    dynamics, _, provenance = load_kart()
    assert dynamics.wheelbase == 1.4
    assert provenance["rear_track_width_max_m"] == pytest.approx(55 * 0.0254)
    assert dynamics.half_width * 2 != provenance["rear_track_width_max_m"]
    assert provenance["accuracy_validated"] is False


def test_track_is_a_closed_mapped_circuit_separate_from_region(purdue):
    assert len(purdue.x) > 800
    assert 428 < purdue.length < 438
    assert np.max(np.hypot(np.diff(purdue.x), np.diff(purdue.y))) < 0.6
    assert np.all(purdue.w_left > 0)
    region = np.asarray(purdue.meta["region_enu"])
    # Convex user region, verify the complete centerline lies inside each edge.
    points = np.column_stack((purdue.x, purdue.y))
    for a, b in zip(region[:-1], region[1:], strict=True):
        d = b - a
        cross = d[0] * (points[:, 1] - a[1]) - d[1] * (points[:, 0] - a[0])
        assert np.all(cross >= 0)
    raw = ET.parse(ROOT / "tracks/purdue_gp/region.kml").find(".//{*}coordinates").text.split()
    assert len(raw) == 5
    assert raw[0].startswith("-86.94563287420567,40.43674585340416")
    assert purdue.meta["accuracy_validated"] is False
    assert len(purdue.meta["user_points_enu"]) == 63
    points = np.asarray(purdue.meta["user_points_enu"])
    # The refined KML now defines actual simulation geometry, not just UI dots.
    distances, _ = purdue._tree.query(points)
    assert distances.max() < 0.0001  # CSV precision only; every source knot is retained
    assert purdue.meta["geometry_status"] == "user_revised_KML_centerline"
    assert purdue.meta["direction"] == "counterclockwise"
    assert np.sum(purdue.x * np.roll(purdue.y, -1) - purdue.y * np.roll(purdue.x, -1)) > 0
    # At the checkered line, CCW starts southeast along the main straight.
    assert np.cos(purdue.psi[0]) > 0 and np.sin(purdue.psi[0]) < 0


def test_reset_reproduces_physics_and_camera(purdue):
    sim = Simulator(purdue)

    def run():
        for _ in range(10):
            sim.step()
        rgb, mask = sim.frame()
        return asdict(sim.kart.state), rgb, mask

    state, rgb, mask = run()
    sim.reset()
    state2, rgb2, mask2 = run()
    assert state == state2
    assert np.array_equal(rgb, rgb2)
    assert np.array_equal(mask, mask2)
    assert rgb.dtype == np.uint8 and rgb.shape == (270, 480, 3)
    assert mask.any() and not mask.all()


def test_camera_policy_receives_only_rgb_and_invalid_mask_terminates(purdue):
    sim = Simulator(purdue, policy="camera-demo")

    class BadModel:
        def predict(self, rgb):
            assert isinstance(rgb, np.ndarray) and rgb.shape == (270, 480, 3)
            return np.full((270, 480), np.nan)

    sim.segmenter = BadModel()
    sim.step()
    assert sim.reason.startswith("model_error")
    assert sim.command == (0, 0, 1)
    assert sim.kart.state.t == 0


def test_empty_prediction_terminates_without_throttle(purdue):
    sim = Simulator(purdue, policy="camera-demo")
    sim.segmenter.predict = lambda rgb: np.zeros(rgb.shape[:2], dtype=bool)
    sim.step()
    assert sim.reason == "perception_no_drivable_path"
    assert sim.command[1:] == (0, 1)


def test_body_outside_track_detected_while_reference_point_is_inside(purdue):
    sim = Simulator(purdue, policy="manual")
    left_width = float(purdue.width_at(30)[0][0])
    x, y, yaw = purdue.cartesian(30, left_width - 0.4)
    sim.kart.state.x, sim.kart.state.y, sim.kart.state.yaw = float(x[0]), float(y[0]), float(yaw[0])
    assert purdue.distance_to_edge(x, y)[0] > 0
    sim.previous_s = 30
    sim.step((0, 0, 1))
    assert sim.reason == "body_outside_track"
    assert sim.violations == 1


def test_reference_completes_purdue_lap_without_body_violation(purdue):
    sim = Simulator(purdue)
    report = evaluate(sim, seconds=180)
    assert report["passed"], report
    assert report["minimum_body_clearance_m"] > 0.5
    assert report["accuracy_validated"] is False


def test_time_limit_is_not_success(purdue):
    sim = Simulator(purdue)
    result = evaluate(sim, seconds=0.1)
    assert result["stop_reason"] == "time_limit"
    assert not result["passed"]


@pytest.mark.parametrize("mask", [np.ones((3, 3)), np.full((270, 480), 255), np.full((270, 480), -1)])
def test_wrong_model_output_contract_rejected(mask):
    with pytest.raises(ValueError):
        checked_mask(mask, (270, 480))
