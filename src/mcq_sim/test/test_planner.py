import numpy as np
import pytest

from mcq_sim.planner import FrenetPlanner, PlannerParams, quintic_offsets


def test_quintic_boundary_conditions():
    u = np.linspace(0, 1, 101)
    d = quintic_offsets(0.5, 0.2, np.array([-1.0, 1.5]), u, 10.0)
    assert d.shape == (2, 101)
    assert np.allclose(d[:, 0], 0.5)
    assert np.allclose(d[:, -1], [-1.0, 1.5])
    slope0 = (d[:, 1] - d[:, 0]) / (u[1] * 10.0)
    assert np.allclose(slope0, 0.2, atol=0.02)
    slope1 = (d[:, -1] - d[:, -2]) / (u[1] * 10.0)
    assert np.allclose(slope1, 0.0, atol=0.02)


def test_follow_on_reference_stays_centered(oval, params):
    planner = FrenetPlanner(oval, PlannerParams.from_dict(params.planner))
    x, y, psi = (float(v[0]) for v in oval.cartesian(20.0, 0.0))
    traj = planner.plan(x, y, psi, 5.0, 0.0)
    assert traj.feasible and not traj.stop_requested
    assert traj.reference == "CENTERLINE"
    assert len(traj.t) == 51 and traj.t[-1] == pytest.approx(2.5)
    assert np.abs(traj.d).max() < 0.05  # straight ahead: no reason to leave the centerline
    assert np.allclose(traj.v, 5.0, atol=0.05)
    # Points are inside the track with the kart width to spare.
    assert oval.distance_to_edge(traj.x, traj.y).min() > 0.9


def test_candidates_respect_boundaries_and_obstacle(oval, params):
    pp = PlannerParams.from_dict(params.planner)
    planner = FrenetPlanner(oval, pp)
    x, y, psi = (float(v[0]) for v in oval.cartesian(10.0, 0.0))
    # Obstacle on the centerline 12 m ahead: the plan must go around it.
    ox, oy, _ = (float(v[0]) for v in oval.cartesian(22.0, 0.0))
    traj = planner.plan(x, y, psi, 5.0, 0.0, obstacles=[(ox, oy, 0.5)])
    assert traj.feasible
    assert np.hypot(traj.x - ox, traj.y - oy).min() > 0.5 + pp.kart_half_width - 1e-6
    assert oval.distance_to_edge(traj.x, traj.y).min() >= pp.kart_half_width + pp.margin - 0.05


def test_stop_request_brakes_to_zero(oval, params):
    planner = FrenetPlanner(oval, PlannerParams.from_dict(params.planner))
    x, y, psi = (float(v[0]) for v in oval.cartesian(20.0, 0.0))
    traj = planner.plan(x, y, psi, 5.0, 0.0, stop_requested=True)
    assert traj.stop_requested
    assert traj.v[0] == pytest.approx(5.0, abs=0.1)
    assert traj.v[-1] < 0.6  # v_min floor while still moving; ramp reaches the floor
    assert np.all(np.diff(traj.v) <= 1e-6)


def test_blocked_track_requests_stop(oval, params):
    planner = FrenetPlanner(oval, PlannerParams.from_dict(params.planner))
    x, y, psi = (float(v[0]) for v in oval.cartesian(20.0, 0.0))
    # A wall of obstacles across the track 10 m ahead.
    wall = [(*(float(v[0]) for v in oval.cartesian(30.0, d)[:2]), 0.6) for d in np.linspace(-2.5, 2.5, 6)]
    traj = planner.plan(x, y, psi, 5.0, 0.0, obstacles=wall)
    assert not traj.feasible and traj.stop_requested


def test_boundary_mode_uses_perceived_edges(oval, params):
    pp = PlannerParams.from_dict(params.planner)
    planner = FrenetPlanner(oval, pp, mode="BOUNDARY")
    x, y, psi = (float(v[0]) for v in oval.cartesian(70.0, 0.3))
    bounds = oval.bounds_ahead(x, y, psi, pp.boundary_range, 1.0)
    traj = planner.plan(x, y, psi, 4.0, 0.0, bounds=bounds)
    assert traj.feasible and traj.reference == "BOUNDARY"
    assert traj.v.max() <= pp.boundary_v_cap + 1e-6
    assert oval.distance_to_edge(traj.x, traj.y).min() > 0.8
    with pytest.raises(ValueError):
        planner.plan(x, y, psi, 4.0, 0.0)
