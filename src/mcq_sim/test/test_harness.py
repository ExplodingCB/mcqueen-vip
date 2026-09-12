"""Phase 0 exit test, software half: the full loop follows a synthetic oval from
the track file alone, and the Jetson-side checks stop it when the map is wrong."""

import numpy as np
import pytest

from mcq_sim.harness import run_closed_loop


@pytest.mark.parametrize("mode,cap", [("FOLLOW", 5.0), ("BOUNDARY", 4.0)])
def test_oval_laps_with_zero_interventions(oval, params, mode, cap):
    params.planner["v_cap"] = cap
    params.planner["boundary_v_cap"] = cap
    result = run_closed_loop(oval, params, laps=2, mode=mode)
    assert result.passed, result.summary()
    assert result.completed_laps == 2
    assert result.violations == 0
    assert result.max_abs_d < 1.0
    nominal = oval.length / cap
    for lap in result.lap_times:
        assert nominal * 0.95 < lap < nominal * 1.15
    # The second lap runs at the cap almost throughout.
    log = result.log
    second = log["t"] > result.lap_times[0]
    assert np.mean(log["v"][second]) > 0.95 * cap
    assert log["stop"].max() == 0.0


def test_geofence_fires_on_grossly_wrong_map(oval, params):
    result = run_closed_loop(oval, params, laps=1, believed_track=oval.shifted(0.0, 8.0))
    assert result.stop_reason == "geofence"
    assert result.completed_laps == 0
    assert result.log["v"][-1] < 0.05


def test_perception_disagreement_fires_on_shifted_map(oval, params):
    # Fault injection 15: the survey is off by more than the threshold but the
    # kart is still inside the believed boundaries, so only the comparison of
    # perceived edges against the map can catch it.
    result = run_closed_loop(oval, params, laps=1, believed_track=oval.shifted(0.0, 1.2))
    assert result.stop_reason == "perception_disagreement"
    assert result.sim_time < 5.0
    assert result.log["v"][-1] < 0.05


def test_csv_log(oval, params, tmp_path):
    params.harness["timeout_s"] = 2.0
    result = run_closed_loop(oval, params, laps=1)
    out = tmp_path / "run.csv"
    result.write_csv(out)
    header = out.read_text().splitlines()[0].split(",")
    assert header[:5] == ["t", "x", "y", "yaw", "v"]
    assert len(out.read_text().splitlines()) == len(result.log["t"]) + 1
