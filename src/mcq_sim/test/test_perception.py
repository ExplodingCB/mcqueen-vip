"""From a pavement mask to edges the planner can drive between."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from mcq_sim.camera import CameraParams, DemoSegmenter, TrackCamera
from mcq_sim.environment import Simulator, evaluate
from mcq_sim.perception import CameraPerception, OraclePerception, edges_from_mask
from mcq_sim.reference import smooth_local_reference
from mcq_sim.track import Track, base_to_map

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture(scope="module")
def purdue():
    return Track.load(ROOT / "tracks/purdue_gp")


def pose_on(track, s, d=0.0, yaw_offset=0.0):
    x, y, yaw = track.cartesian(np.array([s]), np.array([d]))
    return SimpleNamespace(x=float(x[0]), y=float(y[0]), yaw=float(track.heading_at(np.array([s]))[0]) + yaw_offset)


def test_true_mask_gives_edges_inside_the_corridor_and_on_it_where_visible(purdue):
    camera = TrackCamera(purdue, CameraParams())
    accurate = 0
    poses = (20.0, 90.0, 210.0, 330.0)
    for s in poses:
        state = pose_on(purdue, s)
        _, truth = camera.render(state)
        edges = edges_from_mask(truth, camera)
        assert edges is not None
        left, right = edges
        assert np.all(left[:, 1] > right[:, 1] - 1e-6)  # left is left
        medians = []
        for polyline in (left, right):
            world = base_to_map(polyline[polyline[:, 0] > 2.0], state.x, state.y, state.yaw)
            distance = purdue.distance_to_edge(world[:, 0], world[:, 1])
            # Where the true edge is out of frame the mask's visible limit is
            # reported instead, which is inside the corridor: safe, never outside.
            assert distance.min() > -0.3, (s, distance.min())
            medians.append(np.median(np.abs(distance)))
        accurate += max(medians) < 0.25
    assert accurate >= len(poses) - 1


def test_a_mask_with_no_pavement_is_no_path(purdue):
    camera = TrackCamera(purdue, CameraParams())
    assert edges_from_mask(np.zeros((270, 480), dtype=bool), camera) is None


def test_the_edges_reach_behind_the_camera_for_the_planner(purdue):
    camera = TrackCamera(purdue, CameraParams())
    _, truth = camera.render(pose_on(purdue, 50.0))
    left, _ = edges_from_mask(truth, camera)
    assert left[0, 0] < 0.0


def test_camera_perception_scores_the_mask_but_returns_only_edges(purdue):
    camera = TrackCamera(purdue, CameraParams())
    perceive = CameraPerception(camera, DemoSegmenter())
    result = perceive(pose_on(purdue, 60.0))
    assert isinstance(result, tuple) and len(result) == 2
    assert perceive.ious and perceive.ious[-1] > 0.95  # the demo is exact on the flat simulator camera
    assert perceive.inference_ms


def test_oracle_edges_are_the_tracks_own(purdue):
    state = pose_on(purdue, 100.0)
    left, right = OraclePerception(purdue, 25.0)(state)
    assert left.shape == right.shape and left[:, 0].max() > 20.0


def test_local_reference_is_smoother_than_the_midline_and_inside_the_corridor(purdue):
    # The hairpin about 150 m along the lap, seen from 15 m before it.
    state = pose_on(purdue, 137.0)
    left, right = purdue.bounds_ahead(state.x, state.y, state.yaw, 25.0, 1.0)
    mid = Track.from_bounds(
        base_to_map(left, state.x, state.y, state.yaw), base_to_map(right, state.x, state.y, state.yaw)
    )
    keep = 1.3
    smooth = smooth_local_reference(mid, keep)
    near = smooth.s < 22
    assert np.abs(smooth.kappa[near]).max() < np.abs(mid.kappa[near]).max()
    assert np.all(smooth.w_left >= keep - 1e-6) and np.all(smooth.w_right >= keep - 1e-6)


def test_the_full_stack_completes_the_lap_on_true_edges(purdue):
    sim = Simulator(purdue, policy="stack", perception="oracle")
    report = evaluate(sim, 240.0, 1)
    assert report["passed"], (report["stop_reason"], report["stack_stop_reason"])
    assert report["perception"] == "oracle"
    # A nominal body clearance: the planner's margin, not the reference driver's.
    assert report["minimum_body_clearance_m"] > 0.15


def test_camera_demo_perception_drives_through_the_same_interface(purdue):
    sim = Simulator(purdue, policy="stack", perception="camera-demo")
    for _ in range(200):
        sim.step()
    report = sim.report()
    assert report["perception_frames"] > 100
    assert report["synthetic_pavement_iou"] > 0.95
    assert sim.kart.state.v > 1.0
