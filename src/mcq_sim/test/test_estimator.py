"""The state estimator against the simulator's sensors, scored on the truth.

This is where the Phase 1 number lives: fused pose within 0.10 m RMS
(docs/05-roadmap.md). The C core's own tests pin its structure on hand-built
inputs; here the real sensor models feed it, with 20 Hz fixes arriving 80 ms
late, a gyro bias that walks, quantized wheel speeds, and a scheduled outage.

The kart drives itself on the truth, and the estimator watches. Scoring an
estimator inside its own control loop measures the pair, and when the number
comes out wrong you cannot tell which half to fix.

Read the numbers these tests print as a lower bound, not as a prediction. The
filter is being scored against the same models its noise parameters were written
for, and the simulator is a kinematic bicycle with no tyre slip, so course over
ground equals heading exactly and the heading estimate comes out far better than
it will on the kart. That is what `course_slip_sigma` is there for. The real
number needs real logs, which is what the replay suite (#7) is for.
"""

from __future__ import annotations

import numpy as np
import pytest

from mcq_sim.cekf import Ekf
from mcq_sim.harness import run_closed_loop
from mcq_sim.params import load_params
from mcq_sim.sensors import GNSS_NONE, SensorSuite
from mcq_sim.track import Track

LAPS = 5
WHEELBASE = 1.05


@pytest.fixture(scope="module")
def truth():
    """Five laps of the oval, driven on the truth. Returns per-tick arrays with
    the two quantities the IMU needs differenced out of the log."""
    track = Track.synthetic_oval()
    params = load_params()
    result = run_closed_loop(track, params, laps=LAPS, seed=0)
    assert result.passed, result.summary()
    log = {k: np.asarray(v, dtype=float) for k, v in result.log.items()}
    dt = float(params.harness["dt"])
    yaw = np.unwrap(log["yaw"])
    log["yaw_rate"] = np.gradient(yaw, dt)
    log["a_long"] = np.gradient(log["v"], dt)
    log["dt"] = np.full(len(yaw), dt)
    return log


def run_estimator(truth, sensor_overrides=None, seed=1, params=None):
    """Feed the sensor models to the filter tick by tick. Returns the per-tick
    estimate, the filter, and the sensor suite that produced the measurements
    (which holds the true biases the filter was supposed to find)."""
    sensors = {"gnss": {}, "imu": {}, "wheel": {}, "steering": {}}
    for group, values in (sensor_overrides or {}).items():
        sensors[group] = values
    suite = SensorSuite.from_params(sensors, seed=seed)
    ekf = Ekf(**(params or {}))

    est = {"t": [], "x": [], "y": [], "yaw": [], "v": [], "sigma": [], "status": []}
    n = len(truth["t"])
    status = None
    for i in range(n):
        t = float(truth["t"][i])
        x, y = float(truth["x"][i]), float(truth["y"][i])
        yaw, v = float(truth["yaw"][i]), float(truth["v"][i])
        yaw_rate, a_long = float(truth["yaw_rate"][i]), float(truth["a_long"][i])

        for sample in suite.gnss.step(t, x, y, v * np.cos(yaw), v * np.sin(yaw)):
            status = sample.status
            if sample.status != GNSS_NONE:
                ekf.gnss_position(sample.t, sample.x, sample.y, sample.sigma_pos)
                ekf.gnss_velocity(sample.t, sample.vx, sample.vy, sample.sigma_vel)
        for sample in suite.imu.step(t, a_long, v * yaw_rate, yaw_rate):
            ekf.predict(sample.t, sample.ax, sample.ay, sample.gz)
        wheels = suite.wheels.read(v, float(truth["throttle"][i]))
        ekf.wheel_speed(t, 0.5 * (wheels[0] + wheels[1]), 0.05)

        out = ekf.output()
        est["t"].append(t)
        est["x"].append(out.x)
        est["y"].append(out.y)
        est["yaw"].append(out.yaw)
        est["v"].append(out.v)
        est["sigma"].append(out.pos_sigma)
        est["status"].append(status)
    return {k: np.asarray(v) for k, v in est.items()}, ekf, suite


def position_error(est, truth):
    return np.hypot(est["x"] - truth["x"], est["y"] - truth["y"])


def test_five_laps_inside_the_phase_1_budget(truth):
    est, ekf, suite = run_estimator(truth)
    error = position_error(est, truth)
    # Skip the first two seconds: heading is unobservable until the kart moves
    # above course_min_speed, and the filter says so through its covariance.
    settled = truth["t"] > 2.0
    rms = float(np.sqrt(np.mean(error[settled] ** 2)))
    print(
        f"\n{LAPS} laps, {truth['t'][-1]:.0f} s: position RMS {rms * 100:.1f} cm, "
        f"max {error[settled].max() * 100:.1f} cm, counters {ekf.counters}"
    )
    assert rms < 0.10  # the Phase 1 exit number
    assert error[settled].max() < 0.5
    # Every fix arrives late, so every one of them should have replayed.
    assert ekf.counters["rewinds"] > 1000
    assert ekf.counters["late_drops"] == 0
    # A handful of rejected fixes is a gate doing its job; hundreds would mean
    # the filter and the sensors disagree about reality.
    assert ekf.counters["rejections"] < 0.01 * ekf.counters["updates"]


def test_yaw_and_speed_track_the_truth(truth):
    est, _, _ = run_estimator(truth)
    settled = truth["t"] > 2.0
    yaw_error = np.abs(np.arctan2(np.sin(est["yaw"] - truth["yaw"]), np.cos(est["yaw"] - truth["yaw"])))
    speed_error = np.abs(est["v"] - truth["v"])
    print(
        f"\nyaw RMS {np.degrees(np.sqrt(np.mean(yaw_error[settled] ** 2))):.2f} deg, "
        f"speed RMS {np.sqrt(np.mean(speed_error[settled] ** 2)):.3f} m/s"
    )
    # Course over ground is the only heading source, so this is as good as the
    # slip angle allows, which is what course_slip_sigma is about.
    assert np.sqrt(np.mean(yaw_error[settled] ** 2)) < np.radians(5.0)
    assert np.sqrt(np.mean(speed_error[settled] ** 2)) < 0.2


def test_the_gyro_bias_is_found(truth):
    est, ekf, suite = run_estimator(truth)
    del est
    out = ekf.output()
    # The model gives the run a power-on bias and lets it walk from there; the
    # filter has to end up near wherever it walked to, since a bias it has not
    # found is the drift it will show during the next outage.
    actual = float(suite.imu.gyro_bias[2])
    print(f"\ngyro bias: actual {actual * 1e3:+.2f} mrad/s, estimated {out.gyro_bias * 1e3:+.2f} mrad/s")
    assert abs(out.gyro_bias - actual) < 0.004


def test_a_ten_second_outage_is_survived_and_admitted(truth):
    # Lose the fix entirely for ten seconds, the budget in docs/02 section 9.
    start, end = 30.0, 40.0
    schedule = [[0.0, "FIXED"], [start, "NONE"], [end, "FIXED"]]
    est, ekf, suite = run_estimator(truth, sensor_overrides={"gnss": {"schedule": schedule}})
    error = position_error(est, truth)
    t = truth["t"]

    before = (t > 20.0) & (t < start)
    during = (t > start) & (t < end)
    after = (t > end + 5.0) & (t < end + 20.0)

    print(
        f"\noutage: error before {error[before].max() * 100:.1f} cm, "
        f"during {error[during].max() * 100:.1f} cm, after {error[after].max() * 100:.1f} cm; "
        f"sigma during {est['sigma'][during].max():.2f} m"
    )
    assert error[before].max() < 0.5
    # It drifts, and it has to admit it: the covariance gate is what stops the
    # kart, and it can only fire if sigma grows.
    assert est["sigma"][during].max() > 5.0 * est["sigma"][before].max()
    # The number that matters is the geofence margin: once the filter's doubt
    # exceeds it, a covariance gate set at the margin fires and the kart stops
    # rather than driving on a guess (docs/02-architecture.md section 6).
    margin = float(load_params().safety["geofence_margin"])
    assert est["sigma"][during].max() > margin
    # And it comes back rather than staying lost.
    assert error[after].max() < 0.5


def test_the_filter_reports_a_covariance_the_geofence_can_use(truth):
    est, ekf, suite = run_estimator(truth)
    cov = ekf.covariance6()
    assert cov[0] > 0.0 and cov[7] > 0.0 and cov[35] > 0.0
    # Unestimated rows must not read as certainty.
    assert cov[14] > 1e3 and cov[21] > 1e3 and cov[28] > 1e3
    # The reported sigma should be in the same league as the actual error, or the
    # gate is either useless or permanently tripped.
    error = position_error(est, truth)
    settled = truth["t"] > 2.0
    assert est["sigma"][settled].mean() < 0.5
    assert est["sigma"][settled].mean() > 0.2 * error[settled].mean()


def test_yaw_rate_consistency_holds_while_driving(truth):
    est, ekf, suite = run_estimator(truth)
    del est
    # At the end of the run the kart is on the oval with the steering settled, so
    # the gyro and the steering angle should agree through the bicycle model.
    steer = float(truth["steer"][-1])
    assert ekf.yaw_rate_consistent(steer, WHEELBASE, 0.2)
    # A steering angle that never produced this yaw rate should not pass.
    assert not ekf.yaw_rate_consistent(steer - 0.3, WHEELBASE, 0.05)
