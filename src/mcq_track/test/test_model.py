from types import SimpleNamespace

import numpy as np
import pytest

from mcq_sim.track import Track
from mcq_sim.track_model import model_signature, track_from_model


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


@pytest.mark.parametrize("fault", ["reordered_sparse", "open_endpoint", "curvature_only", "speed_only"])
def test_nonempty_raceline_is_rejected_until_boundary_validation_exists(fault):
    msg = model()
    if fault == "reordered_sparse":
        n = len(msg.centerline)
        msg.raceline = [msg.centerline[i] for i in [0, n // 2, n // 4, 3 * n // 4]]
        msg.raceline_kappa = [0.0] * 4
        msg.raceline_v = [2.0] * 4
    elif fault == "open_endpoint":
        msg.closed = False
        msg.centerline = [SimpleNamespace(x=x, y=0.0) for x in [0.0, 5.0, 10.0]]
        msg.width_left = msg.width_right = [1.0] * 3
        msg.raceline = [SimpleNamespace(x=x, y=0.0) for x in [-5.0, -2.0, 0.0]]
        msg.raceline_kappa = [0.0] * 3
        msg.raceline_v = [2.0] * 3
    elif fault == "curvature_only":
        msg.raceline_kappa = [0.0]
    else:
        msg.raceline_v = [2.0]
    with pytest.raises(ValueError, match="raceline models are unsupported"):
        track_from_model(msg)
