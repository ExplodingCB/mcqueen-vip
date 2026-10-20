"""Parameter loading from the YAML in config/."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


def _default_config() -> Path:
    source = Path(__file__).resolve().parent.parent / "config" / "sim_default.yaml"
    if source.exists():
        return source
    try:  # installed by colcon: the yaml is in the package share directory
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("mcq_sim")) / "config" / "sim_default.yaml"
    except Exception:  # noqa: BLE001 - no ROS; the source path is the best answer
        return source


DEFAULT_CONFIG = _default_config()


@dataclass
class Params:
    vehicle: dict[str, float] = field(default_factory=dict)
    sensors: dict[str, dict] = field(default_factory=dict)
    planner: dict[str, float] = field(default_factory=dict)
    lateral: dict[str, float] = field(default_factory=dict)
    longitudinal: dict[str, float] = field(default_factory=dict)
    safety: dict[str, float] = field(default_factory=dict)
    harness: dict[str, float] = field(default_factory=dict)

    def override(self, dotted: dict[str, Any]) -> Params:
        """Apply overrides given as {"section.key": value}, or
        {"section.group.key": value} for the nested sections."""
        for key, value in dotted.items():
            section, *path = key.split(".")
            target = getattr(self, section)
            for step in path[:-1]:
                target = target.setdefault(step, {})
            target[path[-1]] = value
        return self


def load_params(path: str | Path | None = None) -> Params:
    with open(path or DEFAULT_CONFIG) as f:
        raw = yaml.safe_load(f)
    return Params(**{k: dict(v) for k, v in raw.items()})


# Acceleration limits for the driving-model path, in place of the conservative
# defaults in sim_default.yaml (3 m/s^2 lateral, 2 longitudinal, 3 braking).
# Lateral 6 m/s^2 is two thirds of the sim's 0.9 friction; a perfect line on the
# true corridor laps Purdue in 59 s with them. Placeholders like the rest of the
# kart model until the drive-by-wire phase measures real grip.
RACE_LIMITS = {
    "planner.a_lat_max": 6.0,
    "planner.a_long_max": 4.0,
    "planner.a_brake_max": 5.0,
    "lateral.a_lat_max": 8.0,
    "longitudinal.accel_max": 4.5,
    "longitudinal.decel_max": -5.5,
}


# A sloppier kart than the placeholder model, for testing and training the
# driving stack against actuators that are not as good as the simulator's.
# Values replace DynamicsParams fields (the nominal ones are in
# config/kart_provisional.yaml). Placeholders for the unmeasured real kart.
DEGRADED_KART = {
    "friction": 0.8,  # 0.9 nominal
    "steer_tau": 0.10,  # s (0.08): the servo is only a little slower than the sim's
    "throttle_tau": 0.16,  # s (0.12)
    "brake_tau": 0.12,  # s (0.08)
    "accel_max": 5.0,  # m/s^2 (6.0)
    "brake_decel_max": 4.5,  # m/s^2 (6.0)
}
# The servo error (gain, offset, backlash, jitter) is separate: mcq_sim.actuators.DEGRADED.
