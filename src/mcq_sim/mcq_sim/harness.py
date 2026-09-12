"""Closed-loop harness: kart model, planner, C controllers, Jetson-side checks.

This is the software half of the Phase 0 exit test (docs/05-roadmap.md): the
full loop follows a track from a track file alone, with the planner at 20 Hz,
the controller at 100 Hz, and the safety checks from docs/04-safety.md
section 5 turning any problem into a stop request.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from mcq_sim import ccontrol
from mcq_sim.params import Params
from mcq_sim.planner import FrenetPlanner, PlannerParams, Trajectory
from mcq_sim.track import Raceline, Track, base_to_map
from mcq_sim.vehicle import KartSim, VehicleParams, VehicleState


@dataclass
class RunResult:
    lap_times: list[float] = field(default_factory=list)
    laps_requested: int = 0
    max_abs_d: float = 0.0
    min_edge_distance: float = float("inf")
    violations: int = 0  # ticks with the kart body outside the true track
    stop_reason: str | None = None
    sim_time: float = 0.0
    log: dict[str, np.ndarray] = field(default_factory=dict)

    @property
    def completed_laps(self) -> int:
        return len(self.lap_times)

    @property
    def passed(self) -> bool:
        return self.completed_laps >= self.laps_requested and self.violations == 0 and self.stop_reason is None

    def summary(self) -> str:
        laps = ", ".join(f"{t:.2f}" for t in self.lap_times) or "none"
        return (
            f"laps {self.completed_laps}/{self.laps_requested} [{laps}] s; "
            f"max |d| {self.max_abs_d:.2f} m; min edge distance {self.min_edge_distance:.2f} m; "
            f"violations {self.violations}; stop {self.stop_reason or 'none'}; "
            f"sim time {self.sim_time:.1f} s; {'PASS' if self.passed else 'FAIL'}"
        )

    def write_csv(self, path) -> None:
        keys = list(self.log)
        rows = np.column_stack([self.log[k] for k in keys])
        np.savetxt(path, rows, delimiter=",", header=",".join(keys), comments="", fmt="%.5f")


class Controller:
    """The controller node's logic: pure pursuit plus longitudinal PID through
    the software limit stage, all in the C core."""

    def __init__(self, params: Params, vehicle: VehicleParams, dt: float):
        lat, lon = params.lateral, params.longitudinal
        self.dt = dt
        self.bike = ccontrol.BicycleParams(vehicle.wheelbase, vehicle.understeer, vehicle.steer_max)
        self.pp = ccontrol.PurePursuit(lat["k_v"], lat["l_min"], lat["l_max"], self.bike)
        self.limits = ccontrol.LateralLimits(
            lat["a_lat_max"], lat["jerk_lat_max"], vehicle.steer_max, lat["steer_rate_max"], 1.0
        )
        pid = ccontrol.pid_params(lon["k_p"], lon["k_i"], lon["k_f"], lon["accel_max"], lon["decel_max"], dt)
        long_params = ccontrol.LongParams(
            pid, lon["stopping_speed"], lon["stopping_decel"], lon["accel_max"], lon["decel_max"]
        )
        actuator = ccontrol.ActuatorMap(
            lon["throttle_per_accel"], lon["brake_per_decel"], lon["deadband"], vehicle.rolling_decel
        )
        self.long = ccontrol.LongControl(long_params, actuator)
        self.prev_kappa = 0.0
        self.prev_steer = 0.0
        self.lateral_error = 0.0

    def step(self, traj: Trajectory, x, y, yaw, v, now: float, enabled: bool = True):
        res = self.pp(traj.x, traj.y, False, x, y, yaw, v)
        self.lateral_error = res.lateral_error
        kappa = ccontrol.limit_curvature(self.limits, res.curvature, self.prev_kappa, v, self.dt)
        self.prev_kappa = kappa
        steer = ccontrol.steer_from_curvature(self.bike, kappa, v)
        steer = ccontrol.rate_limit(steer, self.prev_steer, self.limits.steer_rate_max, self.dt)
        self.prev_steer = steer
        v_t, a_t = traj.at_time(now - traj.stamp)
        accel = self.long.update(enabled, traj.stop_requested, v, v_t, a_t)
        throttle, brake = self.long.pedals(accel)
        return steer, throttle, brake, accel


def run_closed_loop(
    track: Track,
    params: Params,
    laps: int = 3,
    mode: str = "FOLLOW",
    raceline: Raceline | None = None,
    believed_track: Track | None = None,
    obstacles=(),
    seed: int = 0,
    start: VehicleState | None = None,
    perception_threshold: float = 1.0,
    perception_window: float = 1.0,
) -> RunResult:
    """Run the loop on ``track`` (the world). ``believed_track`` is what the
    software thinks the track is (defaults to the world); giving a shifted copy
    reproduces fault-injection item 15."""
    believed = believed_track or track
    vehicle = VehicleParams.from_dict(params.vehicle)
    pp = PlannerParams.from_dict(params.planner)
    pp.kappa_max = float(np.tan(vehicle.steer_max) / vehicle.wheelbase)
    dt = float(params.harness["dt"])
    planner_period = float(params.harness["planner_period"])
    timeout = float(params.harness.get("timeout_s", 600))
    geofence_margin = float(params.safety["geofence_margin"])
    traj_max_age = float(params.safety["trajectory_max_age"])

    if start is None:
        x0, y0, psi0 = track.cartesian(0.0, 0.0)
        start = VehicleState(x=float(x0[0]), y=float(y0[0]), yaw=float(psi0[0]))
    sim = KartSim(vehicle, start, dt=dt, seed=seed)
    planner = FrenetPlanner(believed, pp, mode=mode, raceline=raceline)
    ctrl = Controller(params, vehicle, dt)
    result = RunResult(laps_requested=laps)

    cols = [
        "t",
        "x",
        "y",
        "yaw",
        "v",
        "steer_cmd",
        "steer",
        "throttle",
        "brake",
        "accel_cmd",
        "s",
        "d",
        "edge",
        "lat_err",
        "stop",
    ]
    log = {k: [] for k in cols}

    traj: Trajectory | None = None
    next_plan = 0.0
    stop_requested = False
    disagreement_since: float | None = None
    prev_s = None
    lap_start = 0.0
    left_start = False
    t = 0.0

    while t < timeout:
        st = sim.state
        mx, my, myaw = sim.measured_pose()
        v = st.v

        # ---- Jetson-side checks (docs/04-safety.md section 5) on the believed map.
        edge_believed = float(believed.distance_to_edge(mx, my)[0])
        if edge_believed < -geofence_margin and result.stop_reason is None:
            result.stop_reason = "geofence"
            stop_requested = True
        if mode == "FOLLOW" and result.stop_reason is None:
            # Perception disagreement: the true edges seen ahead should lie on the
            # believed edges. A sustained gap larger than the threshold is a stop.
            left, right = track.bounds_ahead(st.x, st.y, st.yaw, 10.0, 2.0)
            pts = np.vstack([base_to_map(left, st.x, st.y, st.yaw), base_to_map(right, st.x, st.y, st.yaw)])
            gap = np.abs(believed.distance_to_edge(pts[:, 0], pts[:, 1])).max()
            if gap > perception_threshold:
                disagreement_since = t if disagreement_since is None else disagreement_since
                if t - disagreement_since >= perception_window:
                    result.stop_reason = "perception_disagreement"
                    stop_requested = True
            else:
                disagreement_since = None

        # ---- Planner at 20 Hz.
        if t >= next_plan - 1e-9:
            bounds = track.bounds_ahead(mx, my, myaw, pp.boundary_range, 1.0) if mode == "BOUNDARY" else None
            traj = planner.plan(mx, my, myaw, v, t, stop_requested=stop_requested, obstacles=obstacles, bounds=bounds)
            if not traj.feasible and result.stop_reason is None:
                result.stop_reason = "no_feasible_path"
                stop_requested = True
            next_plan += planner_period
        if t - traj.stamp > traj_max_age and result.stop_reason is None:
            result.stop_reason = "trajectory_age"
            stop_requested = True

        # ---- Controller at 100 Hz.
        steer, throttle, brake, accel = ctrl.step(traj, mx, my, myaw, v, t)
        sim.step(steer, throttle, brake)

        # ---- Bookkeeping on the true track.
        s_true, d_true = track.frenet(st.x, st.y)
        s_true, d_true = float(s_true[0]), float(d_true[0])
        edge_true = float(track.distance_to_edge(st.x, st.y)[0])
        result.max_abs_d = max(result.max_abs_d, abs(d_true))
        result.min_edge_distance = min(result.min_edge_distance, edge_true)
        if edge_true < pp.kart_half_width:
            result.violations += 1
        if prev_s is not None and track.closed:
            if s_true > 0.1 * track.length:
                left_start = True
            if left_start and prev_s > 0.8 * track.length and s_true < 0.2 * track.length:
                result.lap_times.append(t - lap_start)
                lap_start = t
                left_start = False
        prev_s = s_true

        for k, val in zip(
            cols,
            (
                t,
                st.x,
                st.y,
                st.yaw,
                st.v,
                steer,
                st.steer,
                throttle,
                brake,
                accel,
                s_true,
                d_true,
                edge_true,
                ctrl.lateral_error,
                float(stop_requested),
            ),
            strict=True,
        ):
            log[k].append(val)

        t += dt
        if result.completed_laps >= laps:
            break
        if stop_requested and st.v < 0.05 and t > 1.0:
            break

    result.sim_time = t
    result.log = {k: np.asarray(v_) for k, v_ in log.items()}
    return result
