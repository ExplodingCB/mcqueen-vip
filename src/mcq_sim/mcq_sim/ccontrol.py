"""ctypes bindings to the C control core in src/mcq_control/core.

The shared library is looked up in MCQ_CONTROL_CORE_LIB, then in the core's
build directory; if it is missing and cmake is available it is built there.
Struct layouts mirror the headers under src/mcq_control/core/include/mcq.
"""

from __future__ import annotations

import ctypes as C
import os
import subprocess
import sys
from functools import lru_cache
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
CORE_DIR = REPO_ROOT / "src" / "mcq_control" / "core"
BUILD_DIR = CORE_DIR / "build"
LIB_NAME = "libmcq_control_core.dylib" if sys.platform == "darwin" else "libmcq_control_core.so"

MAX_BREAKPOINTS = 8


class GainSchedule(C.Structure):
    _fields_ = [("n", C.c_int), ("x", C.c_float * MAX_BREAKPOINTS), ("y", C.c_float * MAX_BREAKPOINTS)]

    @classmethod
    def constant(cls, gain: float) -> GainSchedule:
        s = cls()
        s.n = 1
        s.x[0] = 0.0
        s.y[0] = gain
        return s

    @classmethod
    def table(cls, xs, ys) -> GainSchedule:
        if len(xs) > MAX_BREAKPOINTS:
            raise ValueError("too many breakpoints")
        s = cls()
        s.n = len(xs)
        for i, (x, y) in enumerate(zip(xs, ys, strict=True)):
            s.x[i] = x
            s.y[i] = y
        return s


class PidParams(C.Structure):
    _fields_ = [
        ("k_p", GainSchedule),
        ("k_i", GainSchedule),
        ("k_f", C.c_float),
        ("pos_limit", C.c_float),
        ("neg_limit", C.c_float),
        ("i_rate", C.c_float),
        ("i_unwind", C.c_float),
    ]


class Pid(C.Structure):
    _fields_ = [("params", PidParams), ("p", C.c_float), ("i", C.c_float), ("f", C.c_float), ("control", C.c_float)]


class BicycleParams(C.Structure):
    _fields_ = [("wheelbase", C.c_float), ("understeer", C.c_float), ("steer_max", C.c_float)]


class KinematicState(C.Structure):
    _fields_ = [("x", C.c_float), ("y", C.c_float), ("yaw", C.c_float), ("v", C.c_float)]


class PurePursuitParams(C.Structure):
    _fields_ = [("k_v", C.c_float), ("l_min", C.c_float), ("l_max", C.c_float)]


class CPath(C.Structure):
    _fields_ = [("x", C.POINTER(C.c_float)), ("y", C.POINTER(C.c_float)), ("n", C.c_int), ("closed", C.c_bool)]


class PurePursuitResult(C.Structure):
    _fields_ = [
        ("steer", C.c_float),
        ("curvature", C.c_float),
        ("lookahead", C.c_float),
        ("nearest_index", C.c_int),
        ("target_index", C.c_int),
        ("lateral_error", C.c_float),
    ]


class LateralLimits(C.Structure):
    _fields_ = [
        ("a_lat_max", C.c_float),
        ("jerk_lat_max", C.c_float),
        ("steer_max", C.c_float),
        ("steer_rate_max", C.c_float),
        ("v_min", C.c_float),
    ]


class LongParams(C.Structure):
    _fields_ = [
        ("pid", PidParams),
        ("stopping_speed", C.c_float),
        ("stopping_decel", C.c_float),
        ("accel_max", C.c_float),
        ("decel_max", C.c_float),
    ]


class ActuatorMap(C.Structure):
    _fields_ = [
        ("throttle_per_accel", C.c_float),
        ("brake_per_decel", C.c_float),
        ("deadband", C.c_float),
        ("rolling_decel", C.c_float),
    ]


class Long(C.Structure):
    _fields_ = [("params", LongParams), ("pid", Pid), ("state", C.c_int), ("accel", C.c_float)]


class SpeedProfileParams(C.Structure):
    _fields_ = [
        ("a_lat_max", C.c_float),
        ("a_long_max", C.c_float),
        ("a_brake_max", C.c_float),
        ("v_cap", C.c_float),
        ("v_min", C.c_float),
    ]


def _find_or_build() -> Path:
    env = os.environ.get("MCQ_CONTROL_CORE_LIB")
    if env and Path(env).exists():
        return Path(env)
    try:  # installed by colcon next to the mcq_control package
        from ament_index_python.packages import get_package_prefix

        installed = Path(get_package_prefix("mcq_control")) / "lib" / LIB_NAME
        if installed.exists():
            return installed
    except Exception:  # noqa: BLE001 - no ROS, or package not installed
        pass
    lib = BUILD_DIR / LIB_NAME
    if lib.exists():
        return lib
    subprocess.run(
        [
            "cmake",
            "-S",
            str(CORE_DIR),
            "-B",
            str(BUILD_DIR),
            "-DCMAKE_BUILD_TYPE=Release",
            "-DMCQ_CORE_BUILD_TESTS=OFF",
        ],
        check=True,
        capture_output=True,
    )
    subprocess.run(["cmake", "--build", str(BUILD_DIR)], check=True, capture_output=True)
    return lib


@lru_cache(maxsize=1)
def lib() -> C.CDLL:
    L = C.CDLL(str(_find_or_build()))
    f = C.c_float
    L.mcq_gain_lookup.argtypes = [C.POINTER(GainSchedule), f]
    L.mcq_gain_lookup.restype = f
    L.mcq_pid_init.argtypes = [C.POINTER(Pid), C.POINTER(PidParams)]
    L.mcq_pid_reset.argtypes = [C.POINTER(Pid)]
    L.mcq_pid_update.argtypes = [C.POINTER(Pid), f, f, f, C.c_bool]
    L.mcq_pid_update.restype = f
    L.mcq_curvature_from_steer.argtypes = [C.POINTER(BicycleParams), f, f]
    L.mcq_curvature_from_steer.restype = f
    L.mcq_steer_from_curvature.argtypes = [C.POINTER(BicycleParams), f, f]
    L.mcq_steer_from_curvature.restype = f
    L.mcq_kinematic_step.argtypes = [C.POINTER(BicycleParams), C.POINTER(KinematicState), f, f, f]
    L.mcq_pure_pursuit.argtypes = [
        C.POINTER(PurePursuitParams),
        C.POINTER(BicycleParams),
        C.POINTER(CPath),
        f,
        f,
        f,
        f,
        C.POINTER(PurePursuitResult),
    ]
    L.mcq_pure_pursuit.restype = C.c_bool
    L.mcq_limit_curvature.argtypes = [C.POINTER(LateralLimits), f, f, f, f]
    L.mcq_limit_curvature.restype = f
    L.mcq_rate_limit.argtypes = [f, f, f, f]
    L.mcq_rate_limit.restype = f
    L.mcq_long_init.argtypes = [C.POINTER(Long), C.POINTER(LongParams)]
    L.mcq_long_update.argtypes = [C.POINTER(Long), C.c_bool, C.c_bool, f, f, f]
    L.mcq_long_update.restype = f
    L.mcq_actuator_map.argtypes = [C.POINTER(ActuatorMap), f, C.POINTER(f), C.POINTER(f)]
    L.mcq_speed_profile.argtypes = [
        C.POINTER(SpeedProfileParams),
        C.POINTER(f),
        C.POINTER(f),
        C.POINTER(f),
        C.c_int,
        f,
        f,
        C.POINTER(f),
    ]
    return L


# --------------------------------------------------------------- wrappers
def _f32(a) -> np.ndarray:
    return np.ascontiguousarray(np.asarray(a, dtype=np.float32))


def _ptr(a: np.ndarray):
    return a.ctypes.data_as(C.POINTER(C.c_float))


class PurePursuit:
    """Pure pursuit through the C implementation."""

    def __init__(self, k_v: float, l_min: float, l_max: float, bike: BicycleParams):
        self.params = PurePursuitParams(k_v, l_min, l_max)
        self.bike = bike

    def __call__(self, path_x, path_y, closed: bool, x: float, y: float, yaw: float, v: float):
        px, py = _f32(path_x), _f32(path_y)
        path = CPath(_ptr(px), _ptr(py), len(px), closed)
        res = PurePursuitResult()
        ok = lib().mcq_pure_pursuit(self.params, self.bike, path, x, y, yaw, v, res)
        if not ok:
            raise ValueError("path too short")
        return res


class LongControl:
    def __init__(self, params: LongParams, actuator: ActuatorMap):
        self.state = Long()
        lib().mcq_long_init(self.state, params)
        self.actuator = actuator

    def update(self, enabled: bool, stop: bool, v: float, v_target: float, a_target: float) -> float:
        return lib().mcq_long_update(self.state, enabled, stop, v, v_target, a_target)

    def pedals(self, accel: float) -> tuple[float, float]:
        t, b = C.c_float(), C.c_float()
        lib().mcq_actuator_map(self.actuator, accel, t, b)
        return t.value, b.value


def speed_profile(params: SpeedProfileParams, s, kappa, v_ref, v0: float, v_end: float) -> np.ndarray:
    s32, k32 = _f32(s), _f32(kappa)
    n = len(s32)
    out = np.zeros(n, dtype=np.float32)
    vref_ptr = _ptr(_f32(v_ref)) if v_ref is not None else None
    lib().mcq_speed_profile(params, _ptr(s32), _ptr(k32), vref_ptr, n, v0, v_end, _ptr(out))
    return out.astype(float)


def limit_curvature(lim: LateralLimits, kappa: float, prev: float, v: float, dt: float) -> float:
    return lib().mcq_limit_curvature(lim, kappa, prev, v, dt)


def rate_limit(target: float, prev: float, rate: float, dt: float) -> float:
    return lib().mcq_rate_limit(target, prev, rate, dt)


def steer_from_curvature(bike: BicycleParams, kappa: float, v: float) -> float:
    return lib().mcq_steer_from_curvature(bike, kappa, v)


def curvature_from_steer(bike: BicycleParams, steer: float, v: float) -> float:
    return lib().mcq_curvature_from_steer(bike, steer, v)


def pid_params(k_p, k_i, k_f, pos_limit, neg_limit, i_rate, i_unwind=0.0) -> PidParams:
    p = PidParams()
    p.k_p = GainSchedule.constant(k_p) if np.isscalar(k_p) else GainSchedule.table(*k_p)
    p.k_i = GainSchedule.constant(k_i) if np.isscalar(k_i) else GainSchedule.table(*k_i)
    p.k_f, p.pos_limit, p.neg_limit, p.i_rate, p.i_unwind = k_f, pos_limit, neg_limit, i_rate, i_unwind
    return p
