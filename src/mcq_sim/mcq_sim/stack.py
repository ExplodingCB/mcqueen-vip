"""The full software stack in one loop: sensors, estimator, planner, controller.

Until this module the repository had two simulators that did not meet. The
kinematic harness drove the planner and the C controllers on the true pose, and
the dynamic Purdue environment drove a privileged baseline or a camera demo. The
estimator was scored in a third arrangement that watched a kart it did not
control. Here they close into one loop on the dynamic kart:

  truth --> sensors --> C EKF --> planner (20 Hz) --> C controllers (100 Hz) --> kart

Nothing after the sensor models sees the truth. The planner and controllers take
the estimator's pose and speed, the stop conditions run on the believed map with
the estimator's covariance, and the truth stays with the caller for scoring.

The estimator is started the way a crew would start it: the kart is staged on
the start line facing along the track, so the filter takes its position from the
first GNSS fix and its heading from the track with a loose sigma, and course over
ground tightens it as soon as the kart moves. Until the first fix arrives the
stack holds the brake.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from mcq_sim import ccontrol
from mcq_sim.cekf import Ekf
from mcq_sim.dynamics import DynamicsParams, DynamicState
from mcq_sim.harness import Controller
from mcq_sim.params import Params, load_params
from mcq_sim.perception import CorridorTracker
from mcq_sim.planner import FrenetPlanner, PlannerParams, Trajectory
from mcq_sim.reference import smooth_reference
from mcq_sim.sensors import GNSS_NONE, SensorSuite
from mcq_sim.track import Track, base_to_map
from mcq_sim.vehicle import VehicleParams

START_YAW_SIGMA = 0.1  # rad, how well the kart is aimed when staged on the line
START_POS_SIGMA_FLOOR = 0.05  # m


TELEMETRY_COLUMNS = (
    "est_x",
    "est_y",
    "est_yaw",
    "est_v",
    "est_pos_error_m",
    "est_yaw_error_rad",
    "est_pos_sigma_m",
    "gnss_status",
)


@dataclass
class StackEstimate:
    x: float
    y: float
    yaw: float
    v: float
    pos_sigma: float
    yaw_sigma: float


STACK_MARGIN = 0.3  # m of planner edge margin: the body sweeps wider than the rear axle path in a corner
REFERENCE_SLACK = 0.5  # m, kept from the edges beyond the kart's half width
_reference_cache: dict[tuple, Track] = {}


def path_s_est(points: np.ndarray) -> np.ndarray:
    """Arc length along a polyline sampled finely, from its first point."""
    return np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(points, axis=0).T))])


def reference_for(track: Track, half_width: float) -> Track:
    """The smooth line the planner offsets from, built once per track and kart
    width because the solve takes a few seconds."""
    key = (track.track_id, round(track.length, 3), len(track.x), half_width)
    if key not in _reference_cache:
        _reference_cache[key] = smooth_reference(track, half_width + REFERENCE_SLACK)
    return _reference_cache[key]


class DrivingStack:
    dt = 0.01  # control tick, the kart's physics step

    def __init__(
        self,
        track: Track,
        kart: DynamicsParams,
        params: Params | None = None,
        speed_cap: float = 4.0,
        seed: int = 0,
        believed_track: Track | None = None,
        perceive=None,
    ):
        self.track = track
        self.believed = believed_track or track
        self.kart = kart
        self.params = params or load_params()
        self.seed = seed
        # The controller's bicycle model. The dynamic kart has no understeer
        # parameter of its own, so this one comes from the tuning YAML.
        vehicle = VehicleParams.from_dict(self.params.vehicle)
        vehicle.wheelbase, vehicle.steer_max = kart.wheelbase, kart.steer_max
        vehicle.rolling_decel = kart.rolling_decel
        self.vehicle = vehicle
        self.speed_cap = speed_cap
        # With a perception source the planner drives between the edges it is
        # given each cycle (BOUNDARY mode) and the map is not consulted.
        self.perceive = perceive
        self.reset()

    # ------------------------------------------------------------------ setup
    def reset(self) -> None:
        p = self.params
        planner_params = PlannerParams.from_dict(p.planner)
        planner_params.kart_half_width = self.kart.half_width
        planner_params.smooth_boundary_reference = True
        planner_params.margin = max(planner_params.margin, STACK_MARGIN)
        planner_params.kappa_max = math.tan(self.kart.steer_max) / self.kart.wheelbase
        self.reference = reference_for(self.believed, self.kart.half_width)
        self.planner = FrenetPlanner(self.reference, planner_params, mode="BOUNDARY" if self.perceive else "FOLLOW")
        if self.perceive is not None and callable(getattr(self.perceive, "reset", None)):
            self.perceive.reset(self.seed)
        # A camera sees only ahead; what it saw is remembered on the estimated pose.
        self.corridor = CorridorTracker(self.perceive.camera) if hasattr(self.perceive, "observe") else None
        self.controller = Controller(p, self.vehicle, self.dt)
        self.suite = SensorSuite.from_params(p.sensors, seed=self.seed)
        self.ekf = Ekf()
        self.geofence_margin = float(p.safety["geofence_margin"])
        self.trajectory_max_age = float(p.safety["trajectory_max_age"])
        self.planner_period = float(p.harness["planner_period"])
        self.trajectory: Trajectory | None = None
        self.held_plans = 0  # infeasible replans that kept the previous trajectory
        self.perceived = None  # the last perceived corridor, map frame
        self.model_plans = 0  # cycles driven on the driving model's own line
        self.model_vetoes = 0  # cycles its line was rejected and the planner drove
        self.model_veto_reasons: dict[str, int] = {}
        self.steer_loop = True  # close a loop on the steering encoder around the servo
        self.steer_kp, self.steer_ki, self.steer_i_max = 1.5, 6.0, 0.12
        self.steer_i = 0.0
        self.steer_measured = 0.0
        self.model_corrections = 0  # cycles the model's line was pulled back inside the corridor
        self.model_max_correction = 1.5  # m, a line farther outside than this is refused, not corrected
        self.model_hold_s = 0.15  # s, how long a vetoed cycle may keep the previous model line
        self.model_line = None  # the model's last line, map frame, for the viewer
        self.next_plan = 0.0
        self.stop_reason: str | None = None
        self.initialized = False
        self.gnss_status = GNSS_NONE
        self.command = (0.0, 0.0, 1.0)
        self.estimate = StackEstimate(0.0, 0.0, 0.0, 0.0, math.inf, math.inf)
        self.accel_cmd = 0.0
        self.yaw_rate_consistent = True
        self._last_yaw_rate: float | None = None

    @property
    def stopping(self) -> bool:
        return self.stop_reason is not None

    # ---------------------------------------------------------------- sensing
    def _sense(self, s: DynamicState) -> None:
        b = self.kart.cg_from_rear
        # GNSS is localized to the rear axle, so its velocity is the rear axle's.
        rear_vy = s.v_lateral - b * s.yaw_rate
        c, sn = math.cos(s.yaw), math.sin(s.yaw)
        vx, vy = s.v * c - rear_vy * sn, s.v * sn + rear_vy * c

        for fix in self.suite.gnss.step(s.t, s.x, s.y, vx, vy):
            self.gnss_status = fix.status
            if fix.status == GNSS_NONE:
                continue
            if not self.initialized:
                yaw0 = self.track.cartesian(0.0)[2]
                self.ekf.set_pose(
                    fix.t, fix.x, fix.y, float(yaw0[0]), max(fix.sigma_pos, START_POS_SIGMA_FLOOR), START_YAW_SIGMA
                )
                # set_pose leaves velocity unknown; the same fix carries it.
                self.ekf.gnss_velocity(fix.t, fix.vx, fix.vy, fix.sigma_vel)
                self.initialized = True
                continue
            self.ekf.gnss_position(fix.t, fix.x, fix.y, fix.sigma_pos)
            self.ekf.gnss_velocity(fix.t, fix.vx, fix.vy, fix.sigma_vel)
        # The IMU is mounted at the rear axle, the estimator's base_link, so the
        # CG acceleration the dynamics gives is moved back along the chassis.
        # Skipping this is not small: 0.6 m of lever arm times a steering
        # transition's 1 to 2 rad/s^2 of yaw acceleration is a metre per second
        # squared of lateral error, enough to push the velocity off the fix.
        yaw_accel = (s.yaw_rate - self._last_yaw_rate) / self.dt if self._last_yaw_rate is not None else 0.0
        self._last_yaw_rate = s.yaw_rate
        a_long = s.a_long + b * s.yaw_rate**2
        a_lat = s.a_lat - b * yaw_accel
        for imu in self.suite.imu.step(s.t, a_long, a_lat, s.yaw_rate):
            self.ekf.predict(imu.t, imu.ax, imu.ay, imu.gz)
        wheels = self.suite.wheels.read(s.v, s.throttle)
        steer = self.suite.steering.read(s.steer)
        self.steer_measured = steer
        if self.initialized:
            self.ekf.wheel_speed(s.t, 0.5 * (wheels[0] + wheels[1]), 0.05)
            # A disagreement is raised, not filtered away (docs/02 section 9).
            out = self.ekf.output()
            if out.v > 1.0:
                self.yaw_rate_consistent = self.ekf.yaw_rate_consistent(steer, self.kart.wheelbase, 0.5)
        out = self.ekf.output()
        self.estimate = StackEstimate(out.x, out.y, out.yaw, max(out.v, 0.0), out.pos_sigma, out.yaw_sigma)

    # -------------------------------------------------------------- the tick
    def _replan(self, s: DynamicState, e: StackEstimate, t: float) -> Trajectory | None:
        bounds = None
        if self.perceive is not None:
            # the camera sees the world; only its output comes back
            if self.corridor is not None:
                bounds = self.corridor.update(self.perceive.observe(s), e.x, e.y, e.yaw, t)
                # in the map frame on the estimated pose, for the viewer only
                self.perceived = None if bounds is None else [base_to_map(b, e.x, e.y, e.yaw) for b in bounds]
                drive = getattr(self.perceive, "last_drive", None)
                if drive is not None and bounds is not None:
                    trajectory = self._model_trajectory(drive, bounds, e, t)
                    if trajectory is not None:
                        return trajectory
                    self.model_vetoes += 1  # outside the corridor or unsteerable
                    previous = self.trajectory
                    if (
                        previous is not None
                        and previous.reference == "MODEL"
                        and previous.feasible
                        and t - previous.stamp <= self.model_hold_s
                    ):
                        return previous  # one bad frame: keep driving the last good line
                    # The planner drives, and it must not speed up: it knows
                    # nothing of the corner the model was slowing for.
                    self.planner.p.boundary_v_cap = min(self.speed_cap, max(e.v, self.planner.p.v_min))
            else:
                bounds = self.perceive(s)
            if bounds is None:
                self.stop_reason = self.stop_reason or "perception_no_drivable_path"
                return self.trajectory
        return self.planner.plan(e.x, e.y, e.yaw, e.v, t, stop_requested=self.stopping, bounds=bounds)

    def _steering_loop(self, desired: float, v: float, model_path: bool = True) -> float:
        """Close a loop on the steering encoder around the servo.

        The controllers work out the wheel angle the path needs and trust the
        servo to deliver it. A real servo delivers some other angle: a fraction
        of it, plus an offset, ignoring small moves (dead zone and backlash).
        With the measured wheel angle (the column encoder, docs/03 section 5) the
        error is known, so the command is nudged by it, proportionally for a
        quick correction and with an integral to remove what is left. Steady
        gain, offset and dead-zone errors are absorbed by feedback here instead
        of reaching the path as a drift the controller must unwind. Held at rest,
        where there is no motion to learn the servo from. Used on the driving
        model's path, where it was tuned; the older planner-driven paths keep
        the open-loop steering they were tuned with."""
        if not self.steer_loop or not model_path or v < 0.5 or self.stopping:
            self.steer_i *= 0.9
            return desired
        error = desired - self.steer_measured
        limit = self.steer_i_max
        i_next = float(np.clip(self.steer_i + self.steer_ki * error * self.dt, -limit, limit))
        command = desired + self.steer_kp * error + i_next
        if abs(command) <= self.kart.steer_max:  # anti-windup: do not integrate into the lock
            self.steer_i = i_next
        return float(np.clip(command, -self.kart.steer_max, self.kart.steer_max))

    def _veto(self, reason: str) -> None:
        self.model_veto_reasons[reason] = self.model_veto_reasons.get(reason, 0) + 1
        return None

    def _model_trajectory(self, drive, bounds, e: StackEstimate, t: float) -> Trajectory | None:
        """The driving model's line and speeds as a trajectory the controllers
        track, or None when it fails the stack's checks: the kart body, with the
        planner's margin, must stay inside the perceived corridor over the
        stretch it could reach before stopping (where the camera has seen),
        speed is limited to what can stop within sight, and the line must be
        within the steering limit. The model chooses where to drive and how fast;
        the stack only refuses lines that would leave the pavement."""
        points, speeds = drive
        p = self.planner.p
        path = np.vstack([[0.0, 0.0], points])
        v_want = np.concatenate([[speeds[0]], speeds])
        # resample at 0.5 m and smooth lightly so curvature is not dominated by
        # the model's per-waypoint noise
        seg = np.hypot(*np.diff(path, axis=0).T)
        if not np.all(np.isfinite(seg)) or seg.sum() < 4.0:
            return self._veto("too_short")
        along = np.concatenate([[0.0], np.cumsum(seg)])
        s_fine = np.arange(0.0, along[-1], 0.5)
        fine = np.column_stack([np.interp(s_fine, along, path[:, 0]), np.interp(s_fine, along, path[:, 1])])
        kernel = np.ones(5) / 5
        padded = np.vstack([np.repeat(fine[:1], 2, 0), fine, np.repeat(fine[-1:], 2, 0)])
        fine = np.column_stack([np.convolve(padded[:, i], kernel, mode="valid") for i in range(2)])
        fine[0] = 0.0
        v_fine = np.interp(s_fine, along, v_want)

        # The corridor check covers the stretch the kart could reach before it
        # could stop, and only where the camera has actually seen the pavement:
        # a point past the far end of the perceived corridor is unknown, not
        # outside it. Unknown ground is handled by speed instead (below).
        reach = max(8.0, e.v * e.v / (2.0 * p.a_brake_max) + e.v * 0.5)
        corridor = Track.from_bounds(bounds[0], bounds[1])
        cs, cd = corridor.frenet(fine[:, 0], fine[:, 1])
        seen = cs < corridor.length - 0.5  # frenet clamps points past the end to it
        # first fine sample not in the corridor: how far the kart can see its line
        sight = float(s_fine[np.argmax(~seen)]) if not seen.all() else float(s_fine[-1])
        check = (s_fine <= reach) & seen
        wl, wr = corridor.width_at(cs)
        # The planner's own margin: the body is longer than it is wide and the
        # controller tracks a curve slightly wide.
        keep = self.kart.half_width + p.margin
        room_left, room_right = wl - keep, wr - keep
        # Safety filter: the model decides where to drive, and its line is pulled
        # back inside the corridor the camera perceives where it strays. A line
        # that is grossly outside (or a corridor too narrow to drive) is refused.
        centered = 0.5 * (room_left - room_right)
        clamped = np.where(room_left + room_right > 0.2, np.clip(cd, -room_right, room_left), centered)
        clamped = np.where(seen, clamped, cd)
        overshoot = np.abs(cd - clamped)[check]
        if overshoot.size and overshoot.max() > self.model_max_correction:
            return self._veto("outside_corridor")
        if overshoot.size and overshoot.max() > 1e-3:
            self.model_corrections += 1
            cx, cy, _ = corridor.cartesian(cs, clamped)
            fine = np.where(seen[:, None], np.column_stack([cx, cy]), fine)
            padded = np.vstack([np.repeat(fine[:1], 2, 0), fine, np.repeat(fine[-1:], 2, 0)])
            fine = np.column_stack([np.convolve(padded[:, i], kernel, mode="valid") for i in range(2)])
            fine[0] = 0.0
        if sight < 4.0:
            return self._veto("corridor_too_short")
        xy = base_to_map(fine, e.x, e.y, e.yaw)
        dx, dy = np.gradient(xy[:, 0]), np.gradient(xy[:, 1])
        yaw = np.unwrap(np.arctan2(dy, dx))
        ds = np.maximum(np.hypot(dx, dy), 1e-6)
        kappa = np.gradient(yaw) / ds
        if np.abs(kappa[2:-2]).max(initial=0.0) > 1.05 * p.kappa_max:
            return self._veto("over_steering_limit")
        path_s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1])))])
        v_ref = np.minimum(v_fine, self.speed_cap)
        # Drive within sight: the kart must be able to stop on pavement it has
        # seen, so speed at every point is capped by braking to a stop at the
        # end of the visible line.
        v_ref = np.minimum(v_ref, np.sqrt(2.0 * p.a_brake_max * np.maximum(sight - path_s_est(fine), 0.0) + p.v_min**2))
        sp = ccontrol.SpeedProfileParams(p.a_lat_max, p.a_long_max, p.a_brake_max, self.speed_cap, p.v_min)
        v = ccontrol.speed_profile(sp, path_s, kappa, v_ref, max(e.v, 0.0), float(v_ref[-1]))
        v_avg = np.maximum(0.5 * (v[1:] + v[:-1]), p.v_min)
        t_k = np.concatenate([[0.0], np.cumsum(np.diff(path_s) / v_avg)])
        a = np.gradient(v) / np.maximum(np.gradient(t_k), 1e-6)
        self.model_plans += 1
        self.model_line = xy
        zeros = np.zeros_like(path_s)
        return Trajectory(t_k, xy[:, 0], xy[:, 1], yaw, kappa, v, a, path_s, zeros, "MODEL", False, t)

    def tick(self, s: DynamicState) -> tuple[float, float, float]:
        """One 100 Hz control cycle from the true state, which only the sensor
        models read. Returns steer, throttle, brake."""
        self._sense(s)
        if not self.initialized:
            self.command = (0.0, 0.0, 1.0)
            return self.command
        e, t = self.estimate, s.t

        # Jetson-side checks (docs/04-safety.md section 5), on the believed map
        # and the estimator's own doubt.
        if self.stop_reason is None:
            edge = float(self.believed.distance_to_edge(e.x, e.y)[0]) if self.perceive is None else 0.0
            if edge < -self.geofence_margin:
                self.stop_reason = "geofence"
            elif e.pos_sigma > self.geofence_margin:
                self.stop_reason = "localization_covariance"
            elif not self.yaw_rate_consistent:
                self.stop_reason = "yaw_rate_inconsistent"

        self.planner.p.v_cap = self.speed_cap
        self.planner.p.boundary_v_cap = self.speed_cap
        if self.trajectory is None or t >= self.next_plan - 1e-9:
            previous = self.trajectory
            self.trajectory = self._replan(s, e, t)
            if self.trajectory is None:  # nothing perceived yet: hold
                self.command = (0.0, 0.0, 1.0)
                return self.command
            if not self.trajectory.feasible and self.stop_reason is None:
                # One infeasible replan keeps the last feasible trajectory while it
                # is younger than the trajectory-age limit (docs/04 section 5);
                # a corridor that stays closed for that long stops the kart.
                if previous is not None and previous.feasible and t - previous.stamp <= self.trajectory_max_age:
                    self.trajectory = previous
                    self.held_plans += 1
                else:
                    self.stop_reason = "no_feasible_path"
            self.next_plan = t + self.planner_period
        if t - self.trajectory.stamp > self.trajectory_max_age and self.stop_reason is None:
            self.stop_reason = "trajectory_age"

        steer, throttle, brake, self.accel_cmd = self.controller.step(self.trajectory, e.x, e.y, e.yaw, e.v, t)
        steer = self._steering_loop(float(steer), e.v, self.trajectory.reference == "MODEL")
        self.command = (float(steer), float(throttle), float(brake))
        return self.command

    def telemetry(self, s: DynamicState) -> dict[str, float]:
        """Estimate against truth, for the log. Scoring only; nothing reads it."""
        e = self.estimate
        if not self.initialized:  # same columns, empty until the first fix
            return dict.fromkeys(TELEMETRY_COLUMNS)
        yaw_error = math.atan2(math.sin(e.yaw - s.yaw), math.cos(e.yaw - s.yaw))
        return {
            "est_x": e.x,
            "est_y": e.y,
            "est_yaw": e.yaw,
            "est_v": e.v,
            "est_pos_error_m": math.hypot(e.x - s.x, e.y - s.y),
            "est_yaw_error_rad": yaw_error,
            "est_pos_sigma_m": e.pos_sigma,
            "gnss_status": float(self.gnss_status),
        }


def estimate_error_summary(log: list[dict]) -> dict[str, float | None]:
    """RMS and maximum of the estimator's error over a Simulator log."""
    rows = [r for r in log if r.get("est_pos_error_m") is not None and r["t"] > 2.0]
    if not rows:
        return {"position_rms_m": None, "position_max_m": None, "yaw_rms_deg": None}
    pos = np.array([r["est_pos_error_m"] for r in rows])
    yaw = np.array([r["est_yaw_error_rad"] for r in rows])
    return {
        "position_rms_m": float(np.sqrt(np.mean(pos**2))),
        "position_max_m": float(pos.max()),
        "yaw_rms_deg": float(np.degrees(np.sqrt(np.mean(yaw**2)))),
    }
