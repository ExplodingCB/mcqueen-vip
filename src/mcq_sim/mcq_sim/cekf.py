"""ctypes bindings to the state estimator core in src/mcq_localization/core.

The same arrangement as `ccontrol.py`: the shared library is looked up in
MCQ_LOCALIZATION_CORE_LIB, then in the colcon install space, then in the core's
build directory, and built there with cmake if it is missing. Struct layouts
mirror `src/mcq_localization/core/include/mcq/ekf.h`.

This exists so the filter that will run on the kart is the filter the simulator
scores. A Python reimplementation would be easier to write and would prove
nothing about the C.
"""

from __future__ import annotations

import ctypes as C
import os
import subprocess
from functools import lru_cache
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
CORE_DIR = REPO_ROOT / "src" / "mcq_localization" / "core"
BUILD_DIR = CORE_DIR / "build"
LIB_NAME = "libmcq_localization_core.so"

N = 8  # state elements, MCQ_EKF_N
HISTORY = 64  # MCQ_EKF_HISTORY


class EkfParams(C.Structure):
    _fields_ = [
        ("accel_sigma", C.c_double),
        ("gyro_sigma", C.c_double),
        ("accel_bias_walk", C.c_double),
        ("gyro_bias_walk", C.c_double),
        ("course_min_speed", C.c_double),
        ("course_slip_sigma", C.c_double),
        ("wheel_sigma_min", C.c_double),
        ("max_predict_dt", C.c_double),
    ]


class EkfStep(C.Structure):
    _fields_ = [
        ("t", C.c_double),
        ("x", C.c_double * N),
        ("p", C.c_double * (N * N)),
        ("ax", C.c_double),
        ("ay", C.c_double),
        ("gz", C.c_double),
        ("dt", C.c_double),
    ]


class EkfState(C.Structure):
    _fields_ = [
        ("params", EkfParams),
        ("x", C.c_double * N),
        ("p", C.c_double * (N * N)),
        ("t", C.c_double),
        ("initialized", C.c_bool),
        ("history", EkfStep * HISTORY),
        ("history_head", C.c_int),
        ("history_count", C.c_int),
        ("last_ax", C.c_double),
        ("last_ay", C.c_double),
        ("last_gz", C.c_double),
        ("predictions", C.c_ulong),
        ("updates", C.c_ulong),
        ("rewinds", C.c_ulong),
        ("late_drops", C.c_ulong),
        ("rejections", C.c_ulong),
        ("last_innovation_ratio", C.c_double),
    ]


class EkfOutput(C.Structure):
    _fields_ = [
        ("x", C.c_double),
        ("y", C.c_double),
        ("yaw", C.c_double),
        ("vx", C.c_double),
        ("vy", C.c_double),
        ("v", C.c_double),
        ("v_lat", C.c_double),
        ("yaw_rate", C.c_double),
        ("a_long", C.c_double),
        ("a_lat", C.c_double),
        ("pos_sigma", C.c_double),
        ("yaw_sigma", C.c_double),
        ("gyro_bias", C.c_double),
        ("ax_bias", C.c_double),
        ("ay_bias", C.c_double),
    ]


def _find_or_build() -> Path:
    env = os.environ.get("MCQ_LOCALIZATION_CORE_LIB")
    if env and Path(env).exists():
        return Path(env)
    try:  # installed by colcon next to the mcq_localization package
        from ament_index_python.packages import get_package_prefix

        installed = Path(get_package_prefix("mcq_localization")) / "lib" / LIB_NAME
        if installed.exists():
            return installed
    except Exception:  # noqa: BLE001 - no ROS, or package not installed
        pass
    lib_path = BUILD_DIR / LIB_NAME
    if lib_path.exists():
        return lib_path
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
    return lib_path


@lru_cache(maxsize=1)
def lib() -> C.CDLL:
    handle = C.CDLL(str(_find_or_build()))
    d = C.c_double
    state = C.POINTER(EkfState)
    handle.mcq_ekf_default_params.restype = EkfParams
    handle.mcq_ekf_init.argtypes = [state, C.POINTER(EkfParams)]
    handle.mcq_ekf_set_pose.argtypes = [state, d, d, d, d, d, d]
    handle.mcq_ekf_predict.argtypes = [state, d, d, d, d]
    handle.mcq_ekf_predict.restype = C.c_bool
    handle.mcq_ekf_gnss_position.argtypes = [state, d, d, d, d]
    handle.mcq_ekf_gnss_position.restype = C.c_bool
    handle.mcq_ekf_gnss_velocity.argtypes = [state, d, d, d, d]
    handle.mcq_ekf_gnss_velocity.restype = C.c_bool
    handle.mcq_ekf_wheel_speed.argtypes = [state, d, d, d]
    handle.mcq_ekf_wheel_speed.restype = C.c_bool
    handle.mcq_ekf_yaw_rate_consistent.argtypes = [state, d, d, d]
    handle.mcq_ekf_yaw_rate_consistent.restype = C.c_bool
    handle.mcq_ekf_output.argtypes = [state, C.POINTER(EkfOutput)]
    handle.mcq_ekf_covariance6.argtypes = [state, C.POINTER(C.c_double)]
    return handle


class Ekf:
    """The C filter with a Python face. Methods return what the C returns, so a
    rejected measurement reads as False here too."""

    def __init__(self, **overrides: float):
        self._lib = lib()
        self.params = self._lib.mcq_ekf_default_params()
        for key, value in overrides.items():
            if not hasattr(self.params, key):
                raise AttributeError(f"unknown estimator parameter '{key}'")
            setattr(self.params, key, float(value))
        self.state = EkfState()
        self._lib.mcq_ekf_init(C.byref(self.state), C.byref(self.params))

    def set_pose(self, t: float, x: float, y: float, yaw: float, pos_sigma: float, yaw_sigma: float) -> None:
        self._lib.mcq_ekf_set_pose(C.byref(self.state), t, x, y, yaw, pos_sigma, yaw_sigma)

    def predict(self, t: float, ax: float, ay: float, gz: float) -> bool:
        return bool(self._lib.mcq_ekf_predict(C.byref(self.state), t, ax, ay, gz))

    def gnss_position(self, t: float, x: float, y: float, sigma: float) -> bool:
        return bool(self._lib.mcq_ekf_gnss_position(C.byref(self.state), t, x, y, sigma))

    def gnss_velocity(self, t: float, vx: float, vy: float, sigma: float) -> bool:
        return bool(self._lib.mcq_ekf_gnss_velocity(C.byref(self.state), t, vx, vy, sigma))

    def wheel_speed(self, t: float, speed: float, sigma: float) -> bool:
        return bool(self._lib.mcq_ekf_wheel_speed(C.byref(self.state), t, speed, sigma))

    def yaw_rate_consistent(self, steering_angle: float, wheelbase: float, tolerance: float) -> bool:
        return bool(self._lib.mcq_ekf_yaw_rate_consistent(C.byref(self.state), steering_angle, wheelbase, tolerance))

    def output(self) -> EkfOutput:
        out = EkfOutput()
        self._lib.mcq_ekf_output(C.byref(self.state), C.byref(out))
        return out

    def covariance6(self) -> list[float]:
        buf = (C.c_double * 36)()
        self._lib.mcq_ekf_covariance6(C.byref(self.state), buf)
        return list(buf)

    @property
    def counters(self) -> dict[str, int]:
        s = self.state
        return {
            "predictions": s.predictions,
            "updates": s.updates,
            "rewinds": s.rewinds,
            "late_drops": s.late_drops,
            "rejections": s.rejections,
        }
