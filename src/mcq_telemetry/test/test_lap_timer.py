import math
from pathlib import Path

import pytest
from mcq_telemetry.lap_timer import LapTimer

from mcq_sim.harness import run_closed_loop
from mcq_sim.params import load_params
from mcq_sim.track import Track

TRACK = Path(__file__).resolve().parents[3] / "tracks/synthetic_oval"


def timer():
    return LapTimer((0, -1), (0, 1), (1, 0), min_lap_distance=3, max_gap=2, max_step=10)


def test_standing_lap_interpolates_and_debounces():
    lap = timer()
    for t, x, y in [(0, 0, 0), (1, 2, 0), (2, 2, 2), (3, -2, 2), (4, -1, 0), (5, 1, 0)]:
        lap.update(t, x, y)
    assert lap.completed_laps == 1
    assert lap.last_lap == pytest.approx(4.5)
    assert lap.best_lap == lap.last_lap
    for t, x in [(5.1, -0.1), (5.2, 0.1), (5.3, -0.1), (5.4, 0.1)]:
        lap.update(t, x, 0)
    assert lap.completed_laps == 1


def test_flying_start_reverse_and_outside_segment():
    lap = timer()
    lap.update(0, 2, 0)
    lap.update(1, -2, 0)  # reverse
    assert lap.started is None
    lap.update(2, -2, 2)
    lap.update(3, 2, 2)  # crosses infinite line, outside finish segment
    assert lap.started is None
    lap.update(4, -1, 0)
    lap.update(5, 1, 0)  # flying start, not a completed lap
    assert lap.started == 4.5
    assert lap.completed_laps == 0


@pytest.mark.parametrize("sample", [(4, 1, 0), (-1, 1, 0), (1, 100, 0), (1, math.nan, 0)])
def test_discontinuities_invalidate_partial_lap(sample):
    lap = timer()
    lap.update(0, 0, 0)
    lap.update(*sample)
    assert lap.started is None
    assert lap.completed_laps == 0
    assert math.isnan(lap.elapsed(5))


def test_track_load_and_reversed_endpoints(tmp_path):
    import yaml

    metadata = yaml.safe_load((TRACK / "track.yaml").read_text())
    metadata["start_finish"]["a"], metadata["start_finish"]["b"] = (
        metadata["start_finish"]["b"],
        metadata["start_finish"]["a"],
    )
    (tmp_path / "track.yaml").write_text(yaml.safe_dump(metadata))
    (tmp_path / "track.csv").write_text((TRACK / "track.csv").read_text())
    original, track_id = LapTimer.from_track(TRACK)
    reversed_line, _ = LapTimer.from_track(tmp_path)
    assert track_id == "synthetic_oval"
    assert original.side((-29, -15)) == reversed_line.side((-29, -15))


@pytest.mark.parametrize("mode", ["FOLLOW", "BOUNDARY"])
def test_timer_agrees_with_closed_loop_harness(mode):
    result = run_closed_loop(Track.load(TRACK), load_params(), laps=2, mode=mode)
    assert result.passed, result.summary()
    lap, _ = LapTimer.from_track(TRACK)
    measured = []
    for t, x, y in zip(result.log["t"], result.log["x"], result.log["y"], strict=True):
        previous = lap.completed_laps
        lap.update(float(t), float(x), float(y))
        if lap.completed_laps != previous:
            measured.append(lap.last_lap)
    assert lap.completed_laps == result.completed_laps
    # Harness uses sample boundaries and polyline s; timer interpolates the
    # surveyed segment crossing. Allow two 100 Hz samples of quantization.
    assert measured == pytest.approx(result.lap_times, abs=0.02)


def test_stationary_jitter_never_accumulates_a_lap():
    lap = timer()
    lap.update(0, 0, 0)
    for i in range(1, 2001):
        lap.update(i * 0.01, 0.05 if i % 2 else -0.05, 0)
    assert lap.completed_laps == 0
    assert lap.distance == 0
