"""The synthetic generator: exact labels, real variety, a clean held-out split."""

from __future__ import annotations

import numpy as np
import pytest

import synth
from mcq_sim.camera import CameraParams, TrackCamera


def test_random_tracks_are_closed_drivable_and_different():
    rng = np.random.default_rng(0)
    tracks = [synth.random_track(rng, f"t{i}") for i in range(5)]
    for track in tracks:
        assert track.closed and 150 < track.length < 450
        assert np.abs(track.kappa).max() < 1 / 2.5  # no tighter than the kart can turn
        assert np.all(track.w_left > 1.5) and np.all(track.w_right > 1.0)
    assert len({round(t.length) for t in tracks}) > 1


def test_frames_and_masks_have_the_training_shape_and_both_classes():
    rgb, mask, owner = synth.generate(n_tracks=2, frames_per_track=6, seed=3)
    assert rgb.shape == (12, synth.HALF_HEIGHT, synth.HALF_WIDTH, 3) and rgb.dtype == np.uint8
    assert mask.shape == (12, synth.HALF_HEIGHT, synth.HALF_WIDTH) and mask.dtype == bool
    assert set(owner) == {0, 1}
    # Some frames sit on the grass, so a few masks are empty; most have pavement.
    assert (mask.any(axis=(1, 2))).mean() > 0.6
    assert 0.02 < mask.mean() < 0.7


def test_the_label_is_the_same_geometry_the_simulator_camera_uses():
    rng = np.random.default_rng(1)
    track = synth.random_track(rng)
    x, y, yaw = synth.sample_pose(track, rng)
    look = synth.Look.draw(rng)
    _, mask = synth.render(track, x, y, yaw, look, rng)
    from types import SimpleNamespace

    _, expected = TrackCamera(track, look.camera).render(SimpleNamespace(x=x, y=y, yaw=yaw))
    assert np.array_equal(mask, expected)


def test_appearance_is_randomized_beyond_a_color_threshold():
    rng = np.random.default_rng(2)
    track = synth.random_track(rng)
    x, y, yaw = synth.sample_pose(track, rng)
    means = []
    for _ in range(12):
        rgb, mask = synth.render(track, x, y, yaw, synth.Look.draw(rng), rng)
        if mask.any() and (~mask).any():
            means.append(rgb[mask].mean(axis=0))
    means = np.asarray(means)
    assert means.std(axis=0).max() > 10  # pavement is not one color


def test_generation_is_reproducible():
    a = synth.generate(1, 3, seed=7)[0]
    b = synth.generate(1, 3, seed=7)[0]
    assert np.array_equal(a, b)
    assert not np.array_equal(a, synth.generate(1, 3, seed=8)[0])


def test_the_model_maps_a_frame_to_one_logit_per_pixel():
    torch = pytest.importorskip("torch")
    from model import PavementNet

    net = PavementNet(width=8).eval()
    out = net(torch.zeros(2, 3, synth.HALF_HEIGHT, synth.HALF_WIDTH))
    assert out.shape == (2, synth.HALF_HEIGHT, synth.HALF_WIDTH)
    assert CameraParams().width == 2 * synth.HALF_WIDTH  # half of the deployed camera
