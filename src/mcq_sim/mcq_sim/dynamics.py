"""Uncalibrated planar kart dynamics, rear-axle map pose and CG body velocity.

Axle tire slip, saturating lateral forces, a shared rear friction circle,
longitudinal load transfer, rear drive/brake and delayed actuators. Low-speed
relaxation avoids the dynamic bicycle singularity at standstill. This model
does not resolve individual wheels, rigid-axle scrub, chassis flex or terrain.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from mcq_sim.track import wrap_angle
from mcq_sim.vehicle import VehicleState


@dataclass
class DynamicsParams:
    wheelbase: float = 1.4
    cg_from_rear: float = 0.45
    mass: float = 170.0
    yaw_inertia: float = 35.0
    cg_height: float = 0.28
    cornering_front: float = 9000.0
    cornering_rear: float = 10000.0
    friction: float = 0.9
    steer_max: float = 0.45
    steer_rate_max: float = 3.14
    steer_tau: float = 0.08
    command_delay: float = 0.04
    throttle_tau: float = 0.12
    brake_tau: float = 0.08
    accel_max: float = 3.0
    v_max: float = 12.0
    brake_decel_max: float = 6.0
    rolling_decel: float = 0.3
    drag: float = 0.004
    half_width: float = 0.8  # provisional overall envelope, not rear track width
    front_extent: float = 1.65  # from rear axle
    rear_extent: float = 0.35

    def __post_init__(self):
        for key, value in vars(self).items():
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{key} must be finite and nonnegative")
        for key in ("wheelbase", "mass", "yaw_inertia", "v_max", "friction", "half_width"):
            if getattr(self, key) <= 0:
                raise ValueError(f"{key} must be positive")
        if not 0 < self.cg_from_rear < self.wheelbase:
            raise ValueError("CG must be between the axles")


@dataclass
class DynamicState(VehicleState):
    v_lateral: float = 0.0  # CG, positive left
    a_lat: float = 0.0
    slip_front: float = 0.0
    slip_rear: float = 0.0


class DynamicKart:
    def __init__(self, params: DynamicsParams | None = None, state: DynamicState | None = None, dt=0.01):
        if not math.isfinite(dt) or not 0 < dt <= 0.05:
            raise ValueError("dt must be in (0, 0.05] seconds")
        self.params = params or DynamicsParams()
        self.state = state or DynamicState()
        self.dt = dt
        self._commands = deque()
        self._active = (0.0, 0.0, 0.0)

    def step(self, steer: float, throttle: float, brake: float) -> DynamicState:
        if not all(math.isfinite(v) for v in (steer, throttle, brake)):
            raise ValueError("commands must be finite")
        p, s = self.params, self.state
        command = (
            float(np.clip(steer, -p.steer_max, p.steer_max)),
            float(np.clip(throttle, 0, 1)),
            float(np.clip(brake, 0, 1)),
        )
        self._commands.append((s.t + p.command_delay, command))
        # Timestamp subtraction can produce 0.0100000000000002. Do not
        # accidentally change integration rate when replaying those intervals.
        n = max(1, int(math.ceil(self.dt / 0.0025 - 1e-9)))
        h = self.dt / n
        for _ in range(n):
            while self._commands and self._commands[0][0] <= s.t + 1e-10:
                _, self._active = self._commands.popleft()
            self._integrate(h)
            s.t += h
        return s

    def _integrate(self, h):
        p, s = self.params, self.state
        steer, throttle, brake = self._active
        rate = np.clip((steer - s.steer) / max(p.steer_tau, h), -p.steer_rate_max, p.steer_rate_max)
        s.steer += float(rate) * h
        s.throttle += (throttle - s.throttle) * (1 - math.exp(-h / max(p.throttle_tau, 1e-6)))
        s.brake += (brake - s.brake) * (1 - math.exp(-h / max(p.brake_tau, 1e-6)))
        a, b = p.wheelbase - p.cg_from_rear, p.cg_from_rear
        weight = p.mass * 9.81
        # Use the preceding physical longitudinal acceleration for load transfer.
        front_load = float(
            np.clip((weight * b - p.mass * s.a_long * p.cg_height) / p.wheelbase, 0.05 * weight, 0.95 * weight)
        )
        rear_load = weight - front_load
        front_limit, rear_limit = p.friction * front_load, p.friction * rear_load
        drive = s.throttle * p.accel_max * max(0, 1 - s.v / p.v_max)
        fx_rear = float(np.clip(p.mass * (drive - s.brake * p.brake_decel_max), -rear_limit, rear_limit))
        s.slip_front = math.atan2(s.v_lateral + a * s.yaw_rate, max(s.v, 0.5)) - s.steer
        s.slip_rear = math.atan2(s.v_lateral - b * s.yaw_rate, max(s.v, 0.5))
        fy_front = -front_limit * math.tanh(p.cornering_front * s.slip_front / front_limit)
        remaining = math.sqrt(max(0, rear_limit**2 - fx_rear**2))
        fy_rear = -remaining * math.tanh(p.cornering_rear * s.slip_rear / max(remaining, 1e-6))
        ax = (fx_rear - fy_front * math.sin(s.steer)) / p.mass
        ax -= p.rolling_decel * min(s.v / 0.1, 1) + p.drag * s.v**2
        ay = (fy_front * math.cos(s.steer) + fy_rear) / p.mass
        r_dot = (a * fy_front * math.cos(s.steer) - b * fy_rear) / p.yaw_inertia
        blend = float(np.clip((s.v - 0.5) / 1.0, 0, 1))
        kinematic_r = s.v * math.tan(s.steer) / p.wheelbase
        vy_dot = blend * (ay - s.yaw_rate * s.v) + (1 - blend) * (b * kinematic_r - s.v_lateral) / 0.05
        r_dot = blend * r_dot + (1 - blend) * (kinematic_r - s.yaw_rate) / 0.05
        vx_dot = ax + blend * s.yaw_rate * s.v_lateral
        new_v = max(0.0, s.v + vx_dot * h)
        s.a_long = ax if new_v > 0 else 0.0
        s.a_lat = blend * ay + (1 - blend) * s.v * kinematic_r
        old_v = s.v
        s.v = new_v
        s.v_lateral += vy_dot * h
        s.yaw_rate += r_dot * h
        if new_v < 0.02 and drive <= s.brake * p.brake_decel_max:
            s.v_lateral = s.yaw_rate = 0.0
        yaw_mid = s.yaw + 0.5 * s.yaw_rate * h
        # Transform CG lateral velocity back to the rear-axle reference point.
        rear_vy = s.v_lateral - b * s.yaw_rate
        v_mid = 0.5 * (old_v + new_v)
        s.x += (v_mid * math.cos(yaw_mid) - rear_vy * math.sin(yaw_mid)) * h
        s.y += (v_mid * math.sin(yaw_mid) + rear_vy * math.cos(yaw_mid)) * h
        s.yaw = float(wrap_angle(s.yaw + s.yaw_rate * h))

    def footprint(self):
        """Sample the entire perimeter at <= 0.15 m spacing for edge checking."""
        p, s = self.params, self.state
        x = np.linspace(-p.rear_extent, p.front_extent, 16)
        y = np.linspace(-p.half_width, p.half_width, 12)
        local = np.vstack(
            (
                np.column_stack((x, np.full_like(x, p.half_width))),
                np.column_stack((x, np.full_like(x, -p.half_width))),
                np.column_stack((np.full_like(y, p.front_extent), y)),
                np.column_stack((np.full_like(y, -p.rear_extent), y)),
            )
        )
        c, sn = math.cos(s.yaw), math.sin(s.yaw)
        return local @ np.array([[c, sn], [-sn, c]]) + [s.x, s.y]
