"""The sensor models: rate, latency, noise, fix status and reproducibility.

These are the inputs the state estimator will be written against, so the
properties pinned here are the ones it has to cope with: fixes that arrive late
and carry their own measurement time, a fix status that degrades, an IMU bias
that walks, and wheel speeds that disagree with each other under throttle.
"""

import numpy as np
import pytest

from mcq_sim.params import load_params
from mcq_sim.sensors import (
    GNSS_FIXED,
    GNSS_FLOAT,
    GNSS_NONE,
    GnssParams,
    SensorSuite,
    SteeringParams,
)

DT = 0.01  # the simulator's own period


def drive(suite: SensorSuite, duration: float, v: float = 6.0, yaw_rate: float = 0.2, throttle: float = 0.5):
    """Run the models along a fictional constant-speed arc, collecting what a
    node would have published. Returns (gnss, imu, wheels, steering) where the
    gnss entries are (arrival_time, sample)."""
    gnss, imu, wheels, steering = [], [], [], []
    n = int(round(duration / DT))
    for k in range(n):
        t = k * DT
        yaw = yaw_rate * t
        x, y = v * t * np.cos(yaw), v * t * np.sin(yaw)
        for sample in suite.gnss.step(t, x, y, v * np.cos(yaw), v * np.sin(yaw)):
            gnss.append((t, sample))
        imu.extend(suite.imu.step(t, 0.0, v * yaw_rate, yaw_rate))
        wheels.append(suite.wheels.read(v, throttle))
        steering.append(suite.steering.read(0.1))
    return gnss, imu, wheels, steering


def test_the_yaml_defaults_load_into_the_models():
    params = load_params()
    assert set(params.sensors) >= {"gnss", "imu", "wheel", "steering"}
    suite = SensorSuite.from_params(params.sensors, seed=1)
    assert suite.gnss.params.rate_hz == 20.0
    assert suite.imu.params.rate_hz == 200.0
    assert suite.gnss.status_at(0.0) == GNSS_FIXED


def test_status_constants_match_the_message():
    # The module mirrors mcq_msgs/EgoState so it can be imported without ROS.
    ego = pytest.importorskip("mcq_msgs.msg").EgoState
    assert (GNSS_NONE, GNSS_FLOAT, GNSS_FIXED) == (ego.GNSS_NONE, ego.GNSS_FLOAT, ego.GNSS_FIXED)


def test_gnss_rate_and_latency():
    suite = SensorSuite.from_params(seed=0)
    latency = suite.gnss.params.latency
    gnss, _, _, _ = drive(suite, 10.0)
    # 20 Hz for 10 s, less the handful still in flight behind the latency.
    assert 190 <= len(gnss) <= 200
    ages = np.array([arrival - sample.t for arrival, sample in gnss])
    # Every fix arrives one latency after it was valid, within a sim tick.
    assert np.abs(ages - latency).max() <= DT + 1e-9
    stamps = np.array([sample.t for _, sample in gnss])
    assert np.allclose(np.diff(stamps), 1.0 / suite.gnss.params.rate_hz)


def test_gnss_noise_follows_the_fix_status():
    fixed = SensorSuite.from_params({"gnss": {"schedule": [[0.0, "FIXED"]]}}, seed=2)
    float_fix = SensorSuite.from_params({"gnss": {"schedule": [[0.0, "FLOAT"]]}}, seed=2)
    errors = {}
    for name, suite in (("FIXED", fixed), ("FLOAT", float_fix)):
        gnss, _, _, _ = drive(suite, 120.0, v=0.0, yaw_rate=0.0)
        # v = 0, so the truth stays at the origin and the sample is the error.
        errors[name] = np.array([[s.x, s.y] for _, s in gnss])
    assert errors["FIXED"].std() == pytest.approx(GnssParams().sigma_fixed, rel=0.25)
    assert errors["FLOAT"].std() == pytest.approx(GnssParams().sigma_float, rel=0.25)
    assert errors["FLOAT"].std() > 5 * errors["FIXED"].std()


def test_scheduled_outage_degrades_and_recovers():
    schedule = [[0.0, "FIXED"], [4.0, "FLOAT"], [6.0, "NONE"], [16.0, "FIXED"]]
    suite = SensorSuite.from_params({"gnss": {"schedule": schedule}}, seed=3)
    gnss, _, _, _ = drive(suite, 20.0)

    # The sequence of statuses, in order, with repeats collapsed.
    seen = [s.status for _, s in gnss]
    collapsed = [seen[0]] + [b for a, b in zip(seen, seen[1:], strict=False) if a != b]
    assert collapsed == [GNSS_FIXED, GNSS_FLOAT, GNSS_NONE, GNSS_FIXED]

    # Each transition happens at the scheduled measurement time, not at arrival.
    # The recovery is the first fixed sample after the outage began, since the
    # schedule also starts fixed.
    for t_step, status in ((4.0, GNSS_FLOAT), (6.0, GNSS_NONE), (16.0, GNSS_FIXED)):
        first = min(s.t for _, s in gnss if s.status == status and s.t >= t_step - 1.0)
        assert first == pytest.approx(t_step, abs=1.0 / suite.gnss.params.rate_hz + 1e-9)

    # A lost fix still produces messages, and they say so rather than going
    # quiet: the estimator has to gate on the status, not on silence.
    none_samples = [s for _, s in gnss if s.status == GNSS_NONE]
    assert len(none_samples) > 100
    assert all(not s.has_fix for s in none_samples)
    assert all(s.sigma_pos > 1.0 for s in none_samples)


def test_a_run_is_reproducible_from_the_seed():
    a, _, _, _ = drive(SensorSuite.from_params(seed=7), 5.0)
    b, _, _, _ = drive(SensorSuite.from_params(seed=7), 5.0)
    c, _, _, _ = drive(SensorSuite.from_params(seed=8), 5.0)
    assert [s.x for _, s in a] == [s.x for _, s in b]
    assert [s.x for _, s in a] != [s.x for _, s in c]


def test_each_sensor_draws_from_its_own_stream():
    # Sensors spawn separate generators off the seed, so reading one does not
    # shift another. Without that, adding a sensor changes every old number.
    suite = SensorSuite.from_params(seed=11)
    only_gnss = [suite.gnss.step(k * DT, 0.0, 0.0, 0.0, 0.0) for k in range(500)]
    flat = [s.x for batch in only_gnss for s in batch]

    both = SensorSuite.from_params(seed=11)
    mixed = []
    for k in range(500):
        both.imu.step(k * DT, 0.0, 0.0, 0.0)
        both.wheels.read(5.0, 0.5)
        mixed.extend(s.x for s in both.gnss.step(k * DT, 0.0, 0.0, 0.0, 0.0))
    assert flat == mixed


def test_imu_reports_the_motion_plus_a_bias_that_walks():
    suite = SensorSuite.from_params(seed=5)
    v, yaw_rate = 6.0, 0.2
    _, imu, _, _ = drive(suite, 30.0, v=v, yaw_rate=yaw_rate)
    assert len(imu) == pytest.approx(30.0 * suite.imu.params.rate_hz, rel=0.02)
    assert np.allclose(np.diff([s.t for s in imu]), 1.0 / suite.imu.params.rate_hz)

    gz = np.array([s.gz for s in imu])
    az = np.array([s.az for s in imu])
    ay = np.array([s.ay for s in imu])
    # The signal is there under the noise: yaw rate on z, gravity on z accel,
    # centripetal acceleration on y.
    assert gz.mean() == pytest.approx(yaw_rate, abs=0.05)
    assert az.mean() == pytest.approx(suite.imu.params.gravity, abs=0.3)
    assert ay.mean() == pytest.approx(v * yaw_rate, abs=0.3)

    # The bias moved, and not by a physically absurd amount: a MEMS gyro that
    # walked a radian per second in half a minute would be broken, not drifting.
    assert not np.allclose(suite.imu.gyro_bias, 0.0)
    assert np.abs(suite.imu.gyro_bias).max() < 0.1


def test_wheels_disagree_under_throttle_and_are_quantized():
    suite = SensorSuite.from_params(seed=6)
    resolution = suite.wheels.params.resolution
    reads = np.array([suite.wheels.read(8.0, throttle=1.0) for _ in range(400)])
    front, rear = reads[:, :2].mean(), reads[:, 2:].mean()
    assert front == pytest.approx(8.0, abs=0.02)
    # The rears are driven, so they read high; that difference is the slip flag
    # the estimator raises (docs/02-architecture.md section 9).
    assert rear > front + 0.2
    assert np.allclose(reads / resolution, np.round(reads / resolution))
    assert (reads >= 0.0).all()

    coasting = np.array([suite.wheels.read(8.0, throttle=0.0) for _ in range(400)])
    assert coasting[:, 2:].mean() == pytest.approx(coasting[:, :2].mean(), abs=0.02)


def test_steering_is_quantized_to_the_encoder():
    suite = SensorSuite.from_params(seed=9)
    resolution = SteeringParams().resolution
    reads = np.array([suite.steering.read(0.12) for _ in range(200)])
    assert reads.mean() == pytest.approx(0.12, abs=0.005)
    assert np.allclose(reads / resolution, np.round(reads / resolution))


def test_a_bad_status_name_is_rejected_at_load():
    with pytest.raises(ValueError, match="unknown GNSS status"):
        SensorSuite.from_params({"gnss": {"schedule": [[0.0, "RTK"]]}}, seed=0).gnss.status_at(0.0)
