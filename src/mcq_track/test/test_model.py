from types import SimpleNamespace

import numpy as np
import pytest

from mcq_sim.planner import FrenetPlanner, PlannerParams
from mcq_sim.track import Track
from mcq_sim.track_model import model_signature, reference_from_model, track_from_model


def model():
    track = Track.synthetic_oval()
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        centerline=[SimpleNamespace(x=x, y=y) for x, y in zip(track.x, track.y, strict=True)],
        width_left=list(track.w_left),
        width_right=list(track.w_right),
        closed=True,
        track_id="test",
        raceline=[],
        raceline_kappa=[],
        raceline_v=[],
    )


def test_model_reconstruction_and_change_detection():
    msg = model()
    restored = track_from_model(msg)
    original = Track.synthetic_oval()
    s = np.linspace(0, original.length, 500)
    assert np.allclose(restored.cartesian(s), original.cartesian(s))
    signature = model_signature(msg)
    msg.header.stamp = 10
    assert signature == model_signature(msg)
    msg.centerline[0].y += 2
    assert signature != model_signature(msg)


@pytest.mark.parametrize("fault", ["frame", "width_count", "negative", "nan"])
def test_model_rejects_bad_geometry(fault):
    msg = model()
    if fault == "frame":
        msg.header.frame_id = "base_link"
    elif fault == "width_count":
        msg.width_left.pop()
    elif fault == "negative":
        msg.width_right[0] = -1
    else:
        msg.centerline[0].x = float("nan")
    with pytest.raises(ValueError):
        track_from_model(msg)


def test_optional_raceline_reference():
    msg = model()
    track = track_from_model(msg)
    assert reference_from_model(msg, track) == (track, None)
    x, y, _ = track.cartesian(track.s, 0.2)
    msg.raceline = [SimpleNamespace(x=a, y=b) for a, b in zip(x, y, strict=True)]
    msg.raceline_kappa = list(track.kappa)
    msg.raceline_v = [4.0] * len(x)
    reference, raceline = reference_from_model(msg, track)
    assert np.allclose(reference.x, x)
    assert np.all(reference.w_left < track.w_left)
    assert np.all(raceline.v == 4)
    msg.raceline_v[0] = -1
    with pytest.raises(ValueError):
        reference_from_model(msg, track)


def test_raceline_speed_closes_at_actual_track_length():
    msg = model()
    track = track_from_model(msg)
    msg.raceline = msg.centerline
    msg.raceline_kappa = list(track.kappa)
    msg.raceline_v = [4.0] * len(track.x)
    msg.raceline_v[0] = 2.0
    reference, raceline = reference_from_model(msg, track)
    planner = FrenetPlanner(reference, PlannerParams(), raceline=raceline)
    seam_midpoint = 0.5 * (reference.s[-1] + reference.length)
    assert planner._raceline_speed(reference.s[-1]) == pytest.approx(4.0)
    assert planner._raceline_speed(seam_midpoint) == pytest.approx(3.0)
    assert planner._raceline_speed(reference.length) == pytest.approx(2.0)
