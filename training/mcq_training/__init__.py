"""Perception training pipeline: label projection, auto-labeling, dataset
assembly, training, evaluation and export. See docs/09-training-pipeline.md.

The track model comes from ``mcq_sim.track`` so the survey, the simulator and
the label projector agree on one definition of the pavement; the path below
makes that import work from a checkout without ``PYTHONPATH`` set.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
TRAINING = REPO / "training"
DATA = TRAINING / "data"  # raw/, frames/, labels/, datasets/: never in git

_sim = REPO / "src" / "mcq_sim"
if _sim.is_dir() and str(_sim) not in sys.path:
    sys.path.insert(0, str(_sim))
