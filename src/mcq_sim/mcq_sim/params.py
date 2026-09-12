"""Parameter loading from the YAML in config/."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config" / "sim_default.yaml"


@dataclass
class Params:
    vehicle: dict[str, float] = field(default_factory=dict)
    planner: dict[str, float] = field(default_factory=dict)
    lateral: dict[str, float] = field(default_factory=dict)
    longitudinal: dict[str, float] = field(default_factory=dict)
    safety: dict[str, float] = field(default_factory=dict)
    harness: dict[str, float] = field(default_factory=dict)

    def override(self, dotted: dict[str, Any]) -> Params:
        """Apply overrides given as {"section.key": value}."""
        for key, value in dotted.items():
            section, name = key.split(".", 1)
            getattr(self, section)[name] = value
        return self


def load_params(path: str | Path | None = None) -> Params:
    with open(path or DEFAULT_CONFIG) as f:
        raw = yaml.safe_load(f)
    return Params(**{k: dict(v) for k, v in raw.items()})
