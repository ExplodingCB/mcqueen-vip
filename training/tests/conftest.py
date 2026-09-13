import sys
from pathlib import Path

import numpy as np
import pytest

TRAINING = Path(__file__).resolve().parents[1]
if str(TRAINING) not in sys.path:
    sys.path.insert(0, str(TRAINING))

from mcq_sim.track import Track  # noqa: E402
from mcq_training.camera import CameraModel  # noqa: E402


@pytest.fixture(scope="session")
def oval() -> Track:
    return Track.synthetic_oval()


@pytest.fixture(scope="session")
def cam() -> CameraModel:
    # The placeholder kart camera: 1280 x 800, 100 deg horizontal, 0.75 m up, 10 deg down.
    return CameraModel.from_fov(1280, 800, 100.0, x=0.6, y=0.0, z=0.75, pitch=np.radians(10.0))
