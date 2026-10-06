"""Sensor models: what the kart's sensors report, not what is true.

Until this module existed the simulator published the fused answer itself: the
true pose with 2 cm of white noise on position, the true yaw, the true speed,
and an RTK status hard-coded to fixed. Every controller and planner result so
far therefore assumed localization that no receiver can deliver. These models
produce what the Jetson actually receives:

  GNSS   20 Hz, late by the receiver and USB delay, noise that depends on the
         fix status, and a status that can be scheduled to degrade from fixed
         to float to none and back.
  IMU    200 Hz, white noise on top of a bias that walks, so a filter that
         does not estimate bias drifts away during a GNSS outage as it would
         on the kart.
  Wheels and steering  quantized by their encoders, with the rear wheels
         reading high under throttle so the slip check has something to see.

The simulator keeps the truth to itself on `/ego_truth`, which is what the
state estimator is scored against (docs/05-roadmap.md, Phase 1: 0.10 m RMS).

No ROS in this module on purpose. The models run in the pure-Python harness and
in the node, and their tests need neither.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np

# Values of the GNSS_* constants in mcq_msgs/EgoState.msg. The message package
# needs ROS to import, this module must not, so the values are mirrored here
# and test_sensors.py asserts they still agree when mcq_msgs is importable.
GNSS_NONE = 0
GNSS_FLOAT = 1
GNSS_FIXED = 2

STATUS_BY_NAME = {"NONE": GNSS_NONE, "FLOAT": GNSS_FLOAT, "FIXED": GNSS_FIXED}
STATUS_NAME = {v: k for k, v in STATUS_BY_NAME.items()}

GRAVITY = 9.80665


def _status_code(status) -> int:
    """A status from the YAML, given as a name or as a code."""
    if isinstance(status, str):
        try:
            return STATUS_BY_NAME[status.upper()]
        except KeyError:
            raise ValueError(f"unknown GNSS status '{status}'; use NONE, FLOAT or FIXED") from None
    code = int(status)
    if code not in STATUS_NAME:
        raise ValueError(f"unknown GNSS status code {code}")
    return code


def _fields_from(cls, d: dict):
    """Keep the keys that are fields of cls, coerced to float. Sequence-valued
    fields (the GNSS schedule) are passed through untouched."""
    out = {}
    for key, value in (d or {}).items():
        if key not in cls.__dataclass_fields__:
            continue
        out[key] = value if isinstance(value, (list, tuple)) else float(value)
    return out


# ----------------------------------------------------------------------- GNSS


@dataclass
class GnssParams:
    rate_hz: float = 20.0
    # Time between the fix being valid and the Jetson having it. A ZED-F9P over
    # USB with an NTRIP correction stream is tens of milliseconds; the estimator
    # has to apply a measurement that is already this old, which is why the
    # samples carry their own measurement time.
    latency: float = 0.08
    sigma_fixed: float = 0.02  # m, 1 sigma horizontal with RTK fixed
    sigma_float: float = 0.45  # m, RTK float
    sigma_none: float = 25.0  # m, no fix: the position carries no information
    vel_sigma_fixed: float = 0.03  # m/s per axis
    vel_sigma_float: float = 0.20
    vel_sigma_none: float = 5.0
    # Piecewise-constant fix status as [[t_seconds, status], ...], applied from
    # each time onward. Scripted rather than random so a run is reproducible.
    schedule: list = field(default_factory=lambda: [[0.0, "FIXED"]])

    @classmethod
    def from_dict(cls, d: dict) -> GnssParams:
        return cls(**_fields_from(cls, d))

    def steps(self) -> list[tuple[float, int]]:
        steps = sorted((float(t), _status_code(s)) for t, s in self.schedule)
        if not steps:
            return [(0.0, GNSS_FIXED)]
        return steps


@dataclass
class GnssSample:
    t: float  # when the fix was valid, not when it arrived
    x: float
    y: float
    vx: float
    vy: float
    status: int
    sigma_pos: float
    sigma_vel: float

    @property
    def has_fix(self) -> bool:
        return self.status != GNSS_NONE


class GnssModel:
    """Fixes at a fixed rate, delayed by the latency, noise by fix status."""

    def __init__(self, params: GnssParams, rng: np.random.Generator):
        self.params = params
        self.rng = rng
        self._steps = params.steps()
        self._next_t: float | None = None
        self._queue: deque[tuple[float, GnssSample]] = deque()

    def status_at(self, t: float) -> int:
        status = self._steps[0][1]
        for t_step, code in self._steps:
            if t + 1e-12 < t_step:
                break
            status = code
        return status

    def sigmas(self, status: int) -> tuple[float, float]:
        p = self.params
        if status == GNSS_FIXED:
            return p.sigma_fixed, p.vel_sigma_fixed
        if status == GNSS_FLOAT:
            return p.sigma_float, p.vel_sigma_float
        return p.sigma_none, p.vel_sigma_none

    def step(self, t: float, x: float, y: float, vx: float, vy: float) -> list[GnssSample]:
        """Advance to time t and return the fixes that have arrived by then.

        A fix measured at t is released at t + latency, so the caller gets a
        sample whose own timestamp is in the past. That is the point.
        """
        period = 1.0 / self.params.rate_hz
        if self._next_t is None:
            self._next_t = t
        while self._next_t <= t + 1e-12:
            t_meas = self._next_t
            status = self.status_at(t_meas)
            sigma_pos, sigma_vel = self.sigmas(status)
            noise = self.rng.normal(0.0, sigma_pos, 2)
            vnoise = self.rng.normal(0.0, sigma_vel, 2)
            sample = GnssSample(
                t=t_meas,
                x=float(x + noise[0]),
                y=float(y + noise[1]),
                vx=float(vx + vnoise[0]),
                vy=float(vy + vnoise[1]),
                status=status,
                sigma_pos=sigma_pos,
                sigma_vel=sigma_vel,
            )
            self._queue.append((t_meas + self.params.latency, sample))
            self._next_t += period
        out = []
        while self._queue and self._queue[0][0] <= t + 1e-12:
            out.append(self._queue.popleft()[1])
        return out


# ------------------------------------------------------------------------ IMU


@dataclass
class ImuParams:
    rate_hz: float = 200.0
    accel_sigma: float = 0.05  # m/s^2, white noise per axis
    gyro_sigma: float = 0.002  # rad/s, white noise per axis
    # Bias random walk, in units per root second. A MEMS part left alone drifts;
    # an estimator that does not track this cannot dead-reckon through a GNSS
    # outage, which is the case Phase 1 has to survive.
    accel_bias_walk: float = 0.01
    gyro_bias_walk: float = 0.0005
    accel_bias0: float = 0.05  # m/s^2, 1 sigma of the bias at power-on
    gyro_bias0: float = 0.005  # rad/s
    gravity: float = GRAVITY

    @classmethod
    def from_dict(cls, d: dict) -> ImuParams:
        return cls(**_fields_from(cls, d))


@dataclass
class ImuSample:
    t: float
    ax: float  # body frame, x forward (REP 103)
    ay: float  # y left
    az: float  # z up, carrying gravity
    gx: float
    gy: float
    gz: float  # yaw rate


class ImuModel:
    """Planar kart: real signal on x and y acceleration and z gyro, noise and
    bias on everything, gravity on z."""

    def __init__(self, params: ImuParams, rng: np.random.Generator):
        self.params = params
        self.rng = rng
        self.accel_bias = rng.normal(0.0, params.accel_bias0, 3)
        self.gyro_bias = rng.normal(0.0, params.gyro_bias0, 3)
        self._next_t: float | None = None

    def step(self, t: float, a_long: float, a_lat: float, yaw_rate: float) -> list[ImuSample]:
        p = self.params
        period = 1.0 / p.rate_hz
        if self._next_t is None:
            self._next_t = t
        out = []
        while self._next_t <= t + 1e-12:
            root_dt = np.sqrt(period)
            self.accel_bias += self.rng.normal(0.0, p.accel_bias_walk * root_dt, 3)
            self.gyro_bias += self.rng.normal(0.0, p.gyro_bias_walk * root_dt, 3)
            accel = np.array([a_long, a_lat, p.gravity]) + self.accel_bias
            accel += self.rng.normal(0.0, p.accel_sigma, 3)
            gyro = np.array([0.0, 0.0, yaw_rate]) + self.gyro_bias
            gyro += self.rng.normal(0.0, p.gyro_sigma, 3)
            out.append(
                ImuSample(
                    t=self._next_t,
                    ax=float(accel[0]),
                    ay=float(accel[1]),
                    az=float(accel[2]),
                    gx=float(gyro[0]),
                    gy=float(gyro[1]),
                    gz=float(gyro[2]),
                )
            )
            self._next_t += period
        return out


# ------------------------------------------------------- wheels and steering


@dataclass
class WheelParams:
    sigma: float = 0.05  # m/s of white noise per wheel
    # Speed resolution of the gateway's hall-sensor measurement. The real number
    # follows from the tooth count and the measurement window and is measured in
    # Phase 1; until then it is a stand-in that is coarse enough to be visible.
    resolution: float = 0.05  # m/s
    rear_slip: float = 0.04  # fraction of speed added to the rears at full throttle

    @classmethod
    def from_dict(cls, d: dict) -> WheelParams:
        return cls(**_fields_from(cls, d))


class WheelModel:
    def __init__(self, params: WheelParams, rng: np.random.Generator):
        self.params = params
        self.rng = rng

    def read(self, v: float, throttle: float = 0.0) -> list[float]:
        """Front left, front right, rear left, rear right in m/s. The fronts are
        unpowered and read the truth; the rears read high under throttle."""
        p = self.params
        slip = 1.0 + p.rear_slip * float(np.clip(throttle, 0.0, 1.0))
        true = np.array([v, v, v * slip, v * slip])
        noisy = true + self.rng.normal(0.0, p.sigma, 4)
        if p.resolution > 0:
            noisy = np.round(noisy / p.resolution) * p.resolution
        return [float(max(0.0, w)) for w in noisy]


@dataclass
class SteeringParams:
    sigma: float = 0.002  # rad
    # A 14-bit absolute encoder on the column (docs/03-hardware.md section 5).
    bits: float = 14.0
    bias: float = 0.0  # rad, mounting offset left in on purpose so calibration has something to find

    @classmethod
    def from_dict(cls, d: dict) -> SteeringParams:
        return cls(**_fields_from(cls, d))

    @property
    def resolution(self) -> float:
        return 2.0 * np.pi / (2.0 ** int(self.bits))


class SteeringModel:
    def __init__(self, params: SteeringParams, rng: np.random.Generator):
        self.params = params
        self.rng = rng

    def read(self, angle: float) -> float:
        p = self.params
        measured = angle + p.bias + self.rng.normal(0.0, p.sigma)
        return float(np.round(measured / p.resolution) * p.resolution)


# ---------------------------------------------------------------------- suite


@dataclass
class SensorSuite:
    """Every sensor on one seed, so a run is reproducible.

    Each model draws from its own generator spawned off the seed. Adding a
    sensor therefore cannot shift the numbers another sensor produces, which
    keeps old runs comparable.
    """

    gnss: GnssModel
    imu: ImuModel
    wheels: WheelModel
    steering: SteeringModel

    @classmethod
    def from_params(cls, sensors: dict | None = None, seed: int = 0) -> SensorSuite:
        sensors = sensors or {}
        streams = np.random.SeedSequence(seed).spawn(4)
        return cls(
            gnss=GnssModel(GnssParams.from_dict(sensors.get("gnss", {})), np.random.default_rng(streams[0])),
            imu=ImuModel(ImuParams.from_dict(sensors.get("imu", {})), np.random.default_rng(streams[1])),
            wheels=WheelModel(WheelParams.from_dict(sensors.get("wheel", {})), np.random.default_rng(streams[2])),
            steering=SteeringModel(
                SteeringParams.from_dict(sensors.get("steering", {})), np.random.default_rng(streams[3])
            ),
        )
