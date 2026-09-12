import numpy as np
import pytest

from mcq_sim.vehicle import KartSim, VehicleParams, VehicleState


def test_full_throttle_accelerates_then_saturates():
    sim = KartSim(VehicleParams(gnss_noise=0.0), VehicleState())
    for _ in range(3000):
        sim.step(0.0, 1.0, 0.0)
    v = sim.state.v
    assert 5.0 < v < 12.0
    for _ in range(3000):
        sim.step(0.0, 1.0, 0.0)
    assert sim.state.v > v - 1e-3  # still not decelerating
    assert abs(sim.state.y) < 1e-6


def test_brake_stops_without_reversing():
    sim = KartSim(VehicleParams(gnss_noise=0.0), VehicleState(v=8.0))
    for _ in range(1000):
        sim.step(0.0, 0.0, 1.0)
    assert sim.state.v == 0.0


def test_steering_actuator_lag_and_rate():
    p = VehicleParams(gnss_noise=0.0)
    sim = KartSim(p, VehicleState(v=3.0))
    sim.step(0.4, 0.0, 0.0)
    assert sim.state.steer == pytest.approx(p.steer_rate_max * sim.dt, rel=1e-3)
    for _ in range(200):
        sim.step(0.4, 0.0, 0.0)
    assert sim.state.steer == pytest.approx(0.4, abs=1e-3)


def test_steady_state_turn_radius():
    p = VehicleParams(gnss_noise=0.0, understeer=0.0, rolling_decel=0.0, drag=0.0)
    sim = KartSim(p, VehicleState(v=4.0, steer=0.1))
    radius = p.wheelbase / np.tan(0.1)
    xs, ys = [], []
    for _ in range(int(2 * np.pi * radius / 4.0 / sim.dt)):
        sim.step(0.1, 0.0, 0.0)
        xs.append(sim.state.x)
        ys.append(sim.state.y)
    xs, ys = np.array(xs), np.array(ys)
    cx, cy = xs.mean(), ys.mean()
    r = np.hypot(xs - cx, ys - cy)
    assert r.mean() == pytest.approx(radius, rel=0.02)
    assert r.std() < 0.05
