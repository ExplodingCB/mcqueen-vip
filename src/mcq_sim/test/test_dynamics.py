import math
from dataclasses import asdict

import numpy as np
import pytest

from mcq_sim.dynamics import DynamicKart, DynamicsParams, DynamicState


def test_force_free_motion_matches_analytic_solution():
    sim = DynamicKart(DynamicsParams(rolling_decel=0, drag=0), DynamicState(v=5, yaw=0.3))
    for _ in range(200):
        sim.step(0, 0, 0)
    assert sim.state.x == pytest.approx(10 * math.cos(0.3), abs=1e-10)
    assert sim.state.y == pytest.approx(10 * math.sin(0.3), abs=1e-10)
    assert sim.state.v == 5


def test_command_transport_delay_and_steering_rate():
    sim = DynamicKart(DynamicsParams(command_delay=0.04, steer_rate_max=1))
    for _ in range(4):
        sim.step(0.4, 1, 0)
        assert sim.state.steer == 0
        assert sim.state.throttle == 0
    sim.step(0.4, 1, 0)
    assert sim.state.steer == pytest.approx(0.01)
    assert sim.state.throttle > 0


def stopping_distance(friction):
    sim = DynamicKart(DynamicsParams(friction=friction), DynamicState(v=8))
    for _ in range(1000):
        sim.step(0, 0, 1)
    assert sim.state.v == 0
    return sim.state.x


def test_low_grip_increases_stopping_distance_and_never_reverses():
    assert stopping_distance(0.3) > 1.8 * stopping_distance(0.9)


def test_left_right_symmetry_and_combined_grip():
    states = []
    for sign in (-1, 1):
        sim = DynamicKart(DynamicsParams(friction=0.5), DynamicState(v=7))
        for _ in range(400):
            sim.step(sign * 0.15, 0.5, 0.1)
            s = sim.state
            # Add back the external rolling/aero drag to isolate tire forces.
            tire_ax = s.a_long + sim.params.rolling_decel * min(s.v / 0.1, 1) + sim.params.drag * s.v**2
            assert math.hypot(tire_ax, s.a_lat) <= 0.5 * 9.81 + 0.05
            assert np.isfinite(list(asdict(s).values())).all()
        states.append(asdict(sim.state))
    for field in ("x", "v"):
        assert states[0][field] == pytest.approx(states[1][field], abs=1e-8)
    for field in ("y", "yaw", "v_lateral", "yaw_rate"):
        assert states[0][field] == pytest.approx(-states[1][field], abs=1e-8)


def test_fixed_step_convergence():
    states = []
    for dt in (0.01, 0.005):
        sim = DynamicKart(state=DynamicState(v=3), dt=dt)
        for _ in range(round(3 / dt)):
            sim.step(0.08, 0.6, 0)
        states.append(sim.state)
    assert states[0].x == pytest.approx(states[1].x, abs=1e-7)
    assert states[0].y == pytest.approx(states[1].y, abs=1e-7)


@pytest.mark.parametrize("kwargs", [{"mass": 0}, {"friction": -1}, {"cg_from_rear": 4}, {"steer_tau": float("nan")}])
def test_invalid_parameters_rejected(kwargs):
    with pytest.raises(ValueError):
        DynamicsParams(**kwargs)


def test_nonfinite_commands_rejected_before_state_mutation():
    sim = DynamicKart()
    before = asdict(sim.state)
    with pytest.raises(ValueError):
        sim.step(float("nan"), 0, 0)
    assert asdict(sim.state) == before
