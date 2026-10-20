"""The full software stack driving the dynamic kart on the Purdue track.

Sensors, the C estimator, the Frenet planner and the C controllers close one
loop here, and the only route from the truth to any of them is the sensor
models. The scoring columns compare the estimate with the truth at the same
instant, which is easy to get wrong by one tick: at 4 m/s a 10 ms slip reads as
a 4 cm bias, and this file exists partly because that happened.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from mcq_sim.environment import Simulator, evaluate
from mcq_sim.params import load_params
from mcq_sim.reference import smooth_reference
from mcq_sim.stack import TELEMETRY_COLUMNS
from mcq_sim.track import Track

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture(scope="module")
def purdue():
    return Track.load(ROOT / "tracks/purdue_gp")


@pytest.fixture(scope="module")
def lap(purdue):
    sim = Simulator(purdue, policy="stack", speed_cap=4.0)
    report = evaluate(sim, 240.0, 1)
    return sim, report


def test_the_full_stack_completes_a_lap_inside_the_boundaries(lap):
    _, report = lap
    print(
        f"\nlap {report['sim_time_s']:.1f} s, clearance {report['minimum_body_clearance_m']:.2f} m, "
        f"estimator {report['estimator']}"
    )
    assert report["passed"], report["stop_reason"]
    assert report["completed_laps"] == 1
    assert report["boundary_violations"] == 0
    assert report["stack_stop_reason"] is None
    assert report["accuracy_validated"] is False


def test_the_estimator_holds_the_phase_1_budget_while_driving(lap):
    _, report = lap
    error = report["estimator"]
    # 0.10 m RMS is the Phase 1 exit number. The lever-arm bug this file guards
    # against measured 0.27 m here and then lost the fix entirely.
    assert error["position_rms_m"] < 0.05
    assert error["position_max_m"] < 0.15
    assert error["yaw_rms_deg"] < 2.0


def test_the_stack_is_driven_by_the_estimate_not_the_truth(lap):
    sim, _ = lap
    # If the controllers read the truth the estimate could only be decoration.
    # Its error is small but not zero, and it is the estimate that is logged
    # as the pose the planner used.
    rows = [r for r in sim.log if r["est_pos_error_m"] is not None]
    assert rows
    assert max(r["est_pos_error_m"] for r in rows) > 0.001
    assert all(set(TELEMETRY_COLUMNS) <= set(r) for r in sim.log)


def test_the_kart_holds_the_brake_until_the_first_fix(purdue):
    sim = Simulator(purdue, policy="stack")
    for _ in range(3):  # 0.15 s, past the 80 ms fix latency
        sim.step()
    assert sim.kart.state.t == pytest.approx(0.15)
    # No fix, no estimate: the first ticks brake, then it moves off.
    assert sim.log[0]["brake_cmd"] == 1.0 and sim.log[0]["throttle_cmd"] == 0.0
    assert sim.log[0]["est_x"] is None
    assert sim.log[-1]["est_x"] is not None


def test_a_ten_second_gnss_outage_is_driven_through(purdue):
    schedule = [[0.0, "FIXED"], [30.0, "NONE"], [40.0, "FIXED"]]
    tuning = load_params().override({"sensors.gnss.schedule": schedule})
    sim = Simulator(purdue, policy="stack", tuning=tuning)
    report = evaluate(sim, 240.0, 1)
    # It drifts, and the covariance gate would stop the kart if it drifted far
    # enough to matter. Here it dead-reckons inside the geofence margin.
    assert report["passed"], (report["stop_reason"], report["stack_stop_reason"])
    assert report["estimator"]["position_max_m"] < float(tuning.safety["geofence_margin"])
    during = [r for r in sim.log if 31.0 < r["t"] < 40.0]
    assert all(r["gnss_status"] == 0 for r in during)


def test_losing_the_fix_for_good_ends_in_a_stop_not_a_wander(purdue):
    tuning = load_params().override({"sensors.gnss.schedule": [[0.0, "FIXED"], [20.0, "NONE"]]})
    sim = Simulator(purdue, policy="stack", tuning=tuning)
    report = evaluate(sim, 240.0, 1)
    assert not report["passed"]
    assert report["stack_stop_reason"] in ("localization_covariance", "geofence")
    assert report["boundary_violations"] == 0
    assert sim.kart.state.v < 0.1


def test_the_csv_export_has_one_set_of_columns(lap):
    sim, _ = lap
    assert len({tuple(r) for r in sim.log}) == 1


def test_speed_cap_is_read_every_tick(purdue):
    sim = Simulator(purdue, policy="stack", speed_cap=2.0)
    for _ in range(300):
        sim.step()
    slow = sim.kart.state.v
    sim.speed_cap = 6.0
    for _ in range(300):
        sim.step()
    assert slow < 2.5
    assert sim.kart.state.v > slow + 1.0


def test_reset_reproduces_the_same_run(purdue):
    sim = Simulator(purdue, policy="stack", seed=3)
    for _ in range(100):
        sim.step()
    first = (sim.kart.state.x, sim.kart.state.y, sim.stack.estimate.x)
    sim.reset()
    for _ in range(100):
        sim.step()
    assert first == (sim.kart.state.x, sim.kart.state.y, sim.stack.estimate.x)


# ------------------------------------------------------------------ reference


def test_the_reference_line_is_smoother_than_the_kml_and_stays_off_the_edges(purdue):
    keep = 1.3
    ref = smooth_reference(purdue, keep)
    assert ref.closed
    assert np.abs(ref.kappa).max() < math.tan(0.45) / 1.4  # what the steering lock can follow
    assert np.abs(ref.kappa).max() < 0.7 * np.abs(purdue.kappa).max()
    assert purdue.distance_to_edge(ref.x, ref.y).min() >= keep - 1e-3
    # The corridor is the same one, measured from the new line.
    assert np.all(ref.w_left > 0) and np.all(ref.w_right > 0)
    assert math.isclose(ref.length, purdue.length, rel_tol=0.05)
    # Direction of travel is preserved.
    assert np.sum(ref.x * np.roll(ref.y, -1) - ref.y * np.roll(ref.x, -1)) > 0
