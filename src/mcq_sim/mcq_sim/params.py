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
