"""Frenet sampling local planner (docs/02-architecture.md section 7.2).

Every cycle: read the ego (s, d, v) on the reference, generate quintic lateral
offset candidates over the horizon, reject the ones that leave the boundaries
or hit an obstacle, give each survivor a reachable speed profile (C
mcq_speed_profile), score by time plus deviation and curvature-change
penalties, and publish the best as a time-sampled trajectory.

FOLLOW mode uses the surveyed track (or a raceline) as the reference; BOUNDARY
mode builds the reference from perceived edges every cycle and uses a lower
speed cap. Same planner, same output, different reference.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from mcq_sim import ccontrol
from mcq_sim.track import Raceline, Track, base_to_map, wrap_angle


@dataclass
class PlannerParams:
    horizon_s: float = 2.5
    dt: float = 0.05
    n_lateral: int = 9
    kart_half_width: float = 0.7
    margin: float = 0.2
    w_ref: float = 0.5
    w_jerk: float = 2.0
    a_lat_max: float = 3.0
    a_long_max: float = 2.0
    a_brake_max: float = 3.0
    v_cap: float = 5.0
    v_min: float = 0.5
    boundary_v_cap: float = 4.0
    boundary_range: float = 25.0
    min_path_length: float = 6.0
    kappa_max: float = 0.4  # from the steering limit: tan(steer_max) / wheelbase
    maneuver_fractions: tuple = (0.5, 1.0)  # fraction of the path over which d reaches d_end

    @classmethod
    def from_dict(cls, d: dict) -> PlannerParams:
        fields = cls.__dataclass_fields__
        out = {}
        for k, v in d.items():
            if k in fields:
                default = fields[k].default
                out[k] = tuple(v) if isinstance(default, tuple) else type(default)(v)
        return cls(**out)


@dataclass
class Trajectory:
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    kappa: np.ndarray
    v: np.ndarray
    a: np.ndarray
    s: np.ndarray
    d: np.ndarray
    reference: str  # RACELINE, CENTERLINE or BOUNDARY
    stop_requested: bool
    stamp: float
    feasible: bool = True

    def at_time(self, tau: float) -> tuple[float, float]:
        """Target speed and acceleration tau seconds after the stamp."""
        tau = float(np.clip(tau, self.t[0], self.t[-1]))
        return float(np.interp(tau, self.t, self.v)), float(np.interp(tau, self.t, self.a))


def quintic_offsets(d0: float, dd0: float, d_end: np.ndarray, u: np.ndarray, length: float, fraction: float = 1.0):
    """Lateral offset d(u) for u in [0, 1] along a path of the given length, from
    (d0, slope dd0 per metre, zero curvature) to (d_end, flat, zero curvature),
    reaching d_end at u = fraction and holding it afterwards.
    Returns an array of shape (len(d_end), len(u))."""
    b = dd0 * length * fraction  # slope per unit of the maneuver
    d_end = np.asarray(d_end, dtype=float)[:, None]
    u = np.minimum(np.asarray(u, dtype=float)[None, :] / fraction, 1.0)
    # Quintic with d(0)=d0, d'(0)=b, d''(0)=0, d(1)=d_end, d'(1)=0, d''(1)=0.
    c3 = 10.0 * (d_end - d0) - 6.0 * b
    c4 = -15.0 * (d_end - d0) + 8.0 * b
    c5 = 6.0 * (d_end - d0) - 3.0 * b
    return d0 + b * u + c3 * u**3 + c4 * u**4 + c5 * u**5


class FrenetPlanner:
    def __init__(
        self,
        track: Track,
        params: PlannerParams,
        mode: str = "FOLLOW",
        raceline: Raceline | None = None,
    ):
        if mode not in ("FOLLOW", "BOUNDARY"):
            raise ValueError(mode)
        self.track = track
        self.p = params
        self.mode = mode
        self.raceline = raceline
        self.last: Trajectory | None = None

    # ------------------------------------------------------------ helpers
    def _reference(self, x, y, yaw, bounds):
        if self.mode == "BOUNDARY":
            if bounds is None:
                raise ValueError("BOUNDARY mode needs perceived bounds")
            left, right = bounds
            ref = Track.from_bounds(base_to_map(left, x, y, yaw), base_to_map(right, x, y, yaw))
            return ref, self.p.boundary_v_cap, "BOUNDARY"
        if self.raceline is not None:
            return self.track, self.p.v_cap, "RACELINE"
        return self.track, self.p.v_cap, "CENTERLINE"

    def _raceline_speed(self, s):
        if self.raceline is None:
            return None
        rl = self.raceline
        return np.interp(np.mod(s, rl.s[-1]), rl.s, rl.v)

    # --------------------------------------------------------------- plan
    def plan(
        self,
        x: float,
        y: float,
        yaw: float,
        v: float,
        stamp: float,
        stop_requested: bool = False,
        obstacles=(),
        bounds=None,
    ) -> Trajectory:
        p = self.p
        ref, v_cap, reference = self._reference(x, y, yaw, bounds)

        s0, d0 = ref.frenet(x, y)
        s0, d0 = float(s0[0]), float(d0[0])
        psi0 = float(ref.heading_at(s0)[0])
        heading_err = float(wrap_angle(yaw - psi0))
        dd0 = float(np.clip(np.tan(np.clip(heading_err, -1.2, 1.2)), -1.5, 1.5))

        n = int(round(p.horizon_s / p.dt)) + 1
        length = max(v * p.horizon_s + 0.5 * p.a_long_max * p.horizon_s**2, p.min_path_length)
        if not ref.closed:
            length = min(length, max(ref.length - s0, 1.0))
        u = np.linspace(0.0, 1.0, n)
        s_k = s0 + u * length
        wl, wr = ref.width_at(s_k)
        keep = p.kart_half_width + p.margin

        lo, hi = -wr[-1] + keep, wl[-1] - keep
        if hi <= lo:
            d_end = np.array([0.5 * (wl[-1] - wr[-1])])
        else:
            d_end = np.linspace(lo, hi, p.n_lateral)
            # Always include the reference itself as a candidate.
            if not np.any(np.isclose(d_end, 0.0, atol=1e-6)) and lo < 0.0 < hi:
                d_end = np.sort(np.append(d_end, 0.0))
        d_k = np.vstack([quintic_offsets(d0, dd0, d_end, u, length, f) for f in p.maneuver_fractions])
        m = d_k.shape[0]

        # Cartesian samples, heading and curvature from the polyline itself.
        xs, ys, _ = ref.cartesian(np.tile(s_k, m), d_k.ravel())
        xs, ys = xs.reshape(m, n), ys.reshape(m, n)
        dx, dy = np.gradient(xs, axis=1), np.gradient(ys, axis=1)
        ds = np.hypot(dx, dy)
        yaw_k = np.arctan2(dy, dx)
        dpsi = wrap_angle(np.gradient(np.unwrap(yaw_k, axis=1), axis=1))
        kappa_k = dpsi / np.maximum(ds, 1e-6)
        path_s = np.concatenate([np.zeros((m, 1)), np.cumsum(np.hypot(np.diff(xs), np.diff(ys)), axis=1)], axis=1)

        # Feasibility: inside the boundaries with the kart width, steerable, clear.
        tol = 1e-6
        inside = (wl[None, :] - d_k >= keep - tol) & (wr[None, :] + d_k >= keep - tol)
        violation = np.maximum(keep - (wl[None, :] - d_k), keep - (wr[None, :] + d_k)).max(axis=1)
        feasible = inside.all(axis=1) & (np.abs(kappa_k[:, 1:-1]).max(axis=1) <= p.kappa_max)
        for ox, oy, orad in obstacles:
            clear = np.hypot(xs - ox, ys - oy) > orad + p.kart_half_width
            feasible &= clear.all(axis=1)

        # Speed profile and cost per candidate. A stop request replaces the
        # profile with a braking ramp from the current speed along the path.
        sp = ccontrol.SpeedProfileParams(p.a_lat_max, p.a_long_max, p.a_brake_max, v_cap, p.v_min)
        v_end = v_cap
        v_k = np.empty((m, n))
        for i in range(m):
            if stop_requested:
                v_ref = np.sqrt(np.maximum(v * v - 2.0 * p.a_brake_max * path_s[i], 0.0))
                v_ref = np.where(path_s[i] > 0, np.maximum(v_ref, 0.0), v)
                sp.a_long_max, sp.v_cap, v_end = 0.0, max(v, p.v_min), 0.0
            else:
                v_ref = self._raceline_speed(s_k)
            v_k[i] = ccontrol.speed_profile(sp, path_s[i], kappa_k[i], v_ref, max(v, 0.0), v_end)
        v_avg = np.maximum(0.5 * (v_k[:, 1:] + v_k[:, :-1]), p.v_min)
        t_k = np.concatenate([np.zeros((m, 1)), np.cumsum(np.diff(path_s, axis=1) / v_avg, axis=1)], axis=1)
        cost = t_k[:, -1] + p.w_ref * np.mean(d_k**2, axis=1) + p.w_jerk * np.sum(np.diff(kappa_k, axis=1) ** 2, axis=1)

        if feasible.any():
            best = int(np.argmin(np.where(feasible, cost, np.inf)))
            ok = True
        else:
            # Nothing is clean: take the least-violating candidate and ask for a stop.
            best = int(np.argmin(violation))
            ok = False
            stop_requested = True
            ramp = np.sqrt(np.maximum(v * v - 2.0 * p.a_brake_max * path_s[best], 0.0))
            v_k[best] = ccontrol.speed_profile(
                ccontrol.SpeedProfileParams(p.a_lat_max, 0.0, p.a_brake_max, max(v, p.v_min), p.v_min),
                path_s[best],
                kappa_k[best],
                ramp,
                max(v, 0.0),
                0.0,
            )
            t_k[best] = np.concatenate(
                [[0.0], np.cumsum(np.diff(path_s[best]) / np.maximum(0.5 * (v_k[best, 1:] + v_k[best, :-1]), p.v_min))]
            )

        # Resample the winner on the time grid.
        t_out = np.arange(0.0, p.horizon_s + 1e-9, p.dt)
        tk = t_k[best]
        if tk[-1] < t_out[-1]:
            # Slow path: pad by holding the last point.
            tk = np.append(tk, t_out[-1] + 1.0)
            pad = lambda a: np.append(a, a[-1])  # noqa: E731
            cols = [pad(c) for c in (xs[best], ys[best], kappa_k[best], v_k[best], s_k, d_k[best])]
            yaw_src = pad(yaw_k[best])
        else:
            cols = [xs[best], ys[best], kappa_k[best], v_k[best], s_k, d_k[best]]
            yaw_src = yaw_k[best]
        x_t, y_t, kappa_t, v_t, s_t, d_t = (np.interp(t_out, tk, c) for c in cols)
        yaw_t = np.arctan2(np.interp(t_out, tk, np.sin(yaw_src)), np.interp(t_out, tk, np.cos(yaw_src)))
        a_t = np.gradient(v_t, t_out) if len(t_out) > 1 else np.zeros_like(v_t)
        traj = Trajectory(t_out, x_t, y_t, yaw_t, kappa_t, v_t, a_t, s_t, d_t, reference, stop_requested, stamp, ok)
        self.last = traj
        return traj
