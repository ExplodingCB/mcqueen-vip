import pytest

from mcq_sim.params import load_params
from mcq_sim.track import Track


@pytest.fixture(scope="session")
def oval() -> Track:
    return Track.synthetic_oval()


@pytest.fixture
def params():
    return load_params()
