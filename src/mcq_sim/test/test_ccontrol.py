"""The C control core through ctypes, compared with independent Python math."""

import numpy as np
import pytest

from mcq_sim import ccontrol as cc


@pytest.fixture(scope="module")
def bike():
    return cc.BicycleParams(1.05, 0.002, 0.45)


def test_bicycle_roundtrip(bike):
    for v in (0.0, 5.0, 10.0):
        for k in np.linspace(-0.2, 0.2, 9):
            steer = cc.steer_from_curvature(bike, k, v)
            expected = np.arctan(k * (1.05 + 0.002 * v * v))
            assert steer == pytest.approx(expected, abs=1e-6)
            assert cc.curvature_from_steer(bike, steer, v) == pytest.approx(k, abs=1e-5)


def test_pure_pursuit_geometry(bike):
    pp = cc.PurePursuit(0.5, 1.5, 6.0, bike)
    xs = np.arange(0.0, 50.0)
    ys = np.zeros_like(xs)
    r = pp(xs, ys, False, 10.0, 1.0, 0.0, 4.0)
    assert r.lookahead == pytest.approx(2.0)
    # Independent pure pursuit: target is the first point at least L ahead.
    d = np.hypot(xs - 10.0, ys - 1.0)
    target = int(np.argmax(d[10:] >= 2.0)) + 10
    alpha = np.arctan2(ys[target] - 1.0, xs[target] - 10.0)
    kappa = 2 * np.sin(alpha) / d[target]
    assert r.target_index == target
    assert r.curvature == pytest.approx(kappa, abs=1e-6)
    assert r.steer < 0.0
    assert r.lateral_error == pytest.approx(-1.0, abs=1e-6)


def test_speed_profile_matches_python():
    p = cc.SpeedProfileParams(3.0, 2.0, 3.0, 10.0, 1.0)
    s = np.arange(0.0, 60.0)
    kappa = np.where((s >= 30) & (s < 40), 0.2, 0.0)
    v = cc.speed_profile(p, s, kappa, None, 2.0, 0.0)

    ref = np.minimum(10.0, np.where(kappa > 1e-6, np.sqrt(3.0 / np.maximum(kappa, 1e-6)), 10.0))
    ref = np.maximum(ref, 1.0)
    ref[0] = min(ref[0], 2.0)
    for i in range(1, len(s)):
        ref[i] = min(ref[i], np.sqrt(ref[i - 1] ** 2 + 2 * 2.0 * (s[i] - s[i - 1])))
    ref[-1] = 0.0
    for i in range(len(s) - 2, -1, -1):
        ref[i] = min(ref[i], np.sqrt(ref[i + 1] ** 2 + 2 * 3.0 * (s[i + 1] - s[i])))
    assert np.allclose(v, ref, atol=1e-4)


def test_long_control_and_pedals():
    pid = cc.pid_params(0.8, 0.2, 1.0, 2.5, -4.0, 0.01)
    lc = cc.LongControl(cc.LongParams(pid, 0.5, -1.5, 2.5, -4.0), cc.ActuatorMap(0.33, 0.17, 0.05, 0.3))
    assert lc.update(False, False, 3.0, 5.0, 0.0) == 0.0
    a = lc.update(True, False, 3.0, 5.0, 0.5)
    assert a == pytest.approx(0.8 * 2.0 + 0.2 * 2.0 * 0.01 + 0.5, abs=1e-5)
    t, b = lc.pedals(1.0)
    assert t == pytest.approx(1.3 * 0.33, abs=1e-6) and b == 0.0
    t, b = lc.pedals(-2.0)
    assert t == 0.0 and b == pytest.approx(1.7 * 0.17, abs=1e-6)


def test_limits():
    lim = cc.LateralLimits(4.0, 5.0, 0.45, 3.0, 1.0)
    assert cc.limit_curvature(lim, 1.0, 0.04, 10.0, 0.01) == pytest.approx(0.04, abs=1e-6)
    assert cc.rate_limit(1.0, 0.0, 3.0, 0.01) == pytest.approx(0.03, abs=1e-6)


def test_gain_schedule_table():
    sched = cc.GainSchedule.table([0.0, 10.0], [1.0, 3.0])
    assert cc.lib().mcq_gain_lookup(sched, 5.0) == pytest.approx(2.0)
