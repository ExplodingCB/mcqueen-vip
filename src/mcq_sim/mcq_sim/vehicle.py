"""Kart model for the simulator.

A rear-axle kinematic bicycle with an understeer term, a rate-limited
first-order steering actuator, throttle and brake maps and aerodynamic plus
rolling drag. The Pacejka lateral tire model and the motor and brake response
identified in Phase 1 slot in here without changing the interface.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from mcq_sim.track import wrap_angle


@dataclass
class VehicleParams:
    wheelbase: float = 1.05
    understeer: float = 0.002
    steer_max: float = 0.45
    steer_rate_max: float = 3.14
    steer_tau: float = 0.08
    accel_max: float = 3.0
    v_max: float = 12.0
    brake_decel_max: float = 6.0
    rolling_decel: float = 0.3
    drag: float = 0.004
    gnss_noise: float = 0.02

    @classmethod
    def from_dict(cls, d: dict) -> VehicleParams:
        return cls(**{k: float(v) for k, v in d.items() if k in cls.__dataclass_fields__})


@dataclass
class VehicleState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0
    steer: float = 0.0  # actual front-wheel angle
    yaw_rate: float = 0.0
    a_long: float = 0.0
    t: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0


@dataclass
class KartSim:
    params: VehicleParams
    state: VehicleState = field(default_factory=VehicleState)
    dt: float = 0.01
    seed: int = 0

    def __post_init__(self):
        self.rng = np.random.default_rng(self.seed)

    def curvature(self, steer: float, v: float) -> float:
        p = self.params
        return float(np.tan(steer) / (p.wheelbase + p.understeer * v * v))

    def step(self, steer_cmd: float, throttle: float, brake: float) -> VehicleState:
        p, s, dt = self.params, self.state, self.dt
        steer_cmd = float(np.clip(steer_cmd, -p.steer_max, p.steer_max))
        throttle = float(np.clip(throttle, 0.0, 1.0))
        brake = float(np.clip(brake, 0.0, 1.0))

        # Steering actuator: first-order lag toward the command, rate limited.
        desired_rate = (steer_cmd - s.steer) / max(p.steer_tau, 1e-3)
        rate = float(np.clip(desired_rate, -p.steer_rate_max, p.steer_rate_max))
        s.steer = float(np.clip(s.steer + rate * dt, -p.steer_max, p.steer_max))

        # Longitudinal: motor torque falls off toward v_max, brake and drag oppose.
        drive = throttle * p.accel_max * max(0.0, 1.0 - s.v / p.v_max)
        resist = brake * p.brake_decel_max + p.rolling_decel + p.drag * s.v * s.v
        a = drive - resist
        if s.v <= 0.0 and a < 0.0:
            a = 0.0
        v_new = max(0.0, s.v + a * dt)
        s.a_long = (v_new - s.v) / dt

        # Kinematic bicycle at the rear axle with the understeer correction.
        kappa = self.curvature(s.steer, s.v)
        s.yaw_rate = s.v * kappa
        s.x += s.v * np.cos(s.yaw) * dt
        s.y += s.v * np.sin(s.yaw) * dt
        s.yaw = float(wrap_angle(s.yaw + s.yaw_rate * dt))
        s.v = v_new
        s.t += dt
        s.throttle, s.brake = throttle, brake
        return s

    def measured_pose(self):
        """Pose as the localization node would report it: GNSS noise on position."""
        s = self.state
        n = self.rng.normal(0.0, self.params.gnss_noise, 2) if self.params.gnss_noise > 0 else (0, 0)
        return s.x + n[0], s.y + n[1], s.yaw
