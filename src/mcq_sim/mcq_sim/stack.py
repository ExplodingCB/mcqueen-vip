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

from mcq_sim.cekf import Ekf
from mcq_sim.dynamics import DynamicsParams, DynamicState
from mcq_sim.harness import Controller
from mcq_sim.params import Params, load_params
from mcq_sim.planner import FrenetPlanner, PlannerParams, Trajectory
from mcq_sim.reference import smooth_reference
from mcq_sim.sensors import GNSS_NONE, SensorSuite
from mcq_sim.track import Track
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


REFERENCE_SLACK = 0.5  # m, kept from the edges beyond the kart's half width
_reference_cache: dict[tuple, Track] = {}


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
        self.reset()

    # ------------------------------------------------------------------ setup
    def reset(self) -> None:
        p = self.params
        planner_params = PlannerParams.from_dict(p.planner)
        planner_params.kart_half_width = self.kart.half_width
        planner_params.kappa_max = math.tan(self.kart.steer_max) / self.kart.wheelbase
        self.reference = reference_for(self.believed, self.kart.half_width)
        self.planner = FrenetPlanner(self.reference, planner_params)
        self.controller = Controller(p, self.vehicle, self.dt)
        self.suite = SensorSuite.from_params(p.sensors, seed=self.seed)
        self.ekf = Ekf()
        self.geofence_margin = float(p.safety["geofence_margin"])
        self.trajectory_max_age = float(p.safety["trajectory_max_age"])
        self.planner_period = float(p.harness["planner_period"])
        self.trajectory: Trajectory | None = None
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
        if self.initialized:
            self.ekf.wheel_speed(s.t, 0.5 * (wheels[0] + wheels[1]), 0.05)
            # A disagreement is raised, not filtered away (docs/02 section 9).
            out = self.ekf.output()
            if out.v > 1.0:
                self.yaw_rate_consistent = self.ekf.yaw_rate_consistent(steer, self.kart.wheelbase, 0.5)
        out = self.ekf.output()
        self.estimate = StackEstimate(out.x, out.y, out.yaw, max(out.v, 0.0), out.pos_sigma, out.yaw_sigma)

    # -------------------------------------------------------------- the tick
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
            edge = float(self.believed.distance_to_edge(e.x, e.y)[0])
            if edge < -self.geofence_margin:
                self.stop_reason = "geofence"
            elif e.pos_sigma > self.geofence_margin:
                self.stop_reason = "localization_covariance"
            elif not self.yaw_rate_consistent:
                self.stop_reason = "yaw_rate_inconsistent"

        self.planner.p.v_cap = self.speed_cap
        if self.trajectory is None or t >= self.next_plan - 1e-9:
            self.trajectory = self.planner.plan(e.x, e.y, e.yaw, e.v, t, stop_requested=self.stopping)
            if not self.trajectory.feasible and self.stop_reason is None:
                self.stop_reason = "no_feasible_path"
            self.next_plan = t + self.planner_period
        if t - self.trajectory.stamp > self.trajectory_max_age and self.stop_reason is None:
            self.stop_reason = "trajectory_age"

        steer, throttle, brake, self.accel_cmd = self.controller.step(self.trajectory, e.x, e.y, e.yaw, e.v, t)
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
