import json
import os
import subprocess
import sys
import urllib.error
import urllib.request
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
PACKAGE_ROOT = ROOT / "src" / "mcq_sim"


@pytest.fixture
def viewer():
    # pytest's `pythonpath` setting puts src/mcq_sim on the path of this process
    # only, so a child process needs to be told separately or it cannot import
    # the package it is being asked to run.
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([str(PACKAGE_ROOT), env.get("PYTHONPATH", "")]).rstrip(os.pathsep)
    process = subprocess.Popen(
        [sys.executable, "-m", "mcq_sim", "view", "--port", "0", "--track", str(ROOT / "tracks/purdue_gp")],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
    )
    try:
        line = process.stdout.readline()
        assert line.startswith("Simulator: http://127.0.0.1:"), line
        yield line.split()[1]
    finally:
        process.terminate()
        process.wait(timeout=10)


def post(url, body, origin=None):
    headers = {"Content-Type": "application/json"}
    if origin:
        headers["Origin"] = origin
    request = urllib.request.Request(url + "/api/control", json.dumps(body).encode(), headers)
    with urllib.request.urlopen(request, timeout=10) as response:
        return json.load(response)


def test_manual_step_uses_commands_and_reset_clears_state(viewer):
    post(viewer, {"action": "mode", "mode": "manual"})
    result = post(viewer, {"action": "step", "manual": [0.1, 1, 0]})
    assert result["state"]["t"] == pytest.approx(0.05)
    assert result["state"]["throttle"] > 0
    assert result["state"]["steer"] > 0
    assert not result["running"]
    result = post(viewer, {"action": "reset"})
    assert result["state"]["t"] == 0 and result["state"]["v"] == 0


def test_physics_slider_updates_report_and_resets(viewer):
    result = post(viewer, {"action": "parameter", "key": "friction", "value": 0.4})
    assert result["report"]["vehicle_parameters"]["friction"] == 0.4
    assert result["state"]["t"] == 0
    assert not result["report"]["accuracy_validated"]


def test_cross_origin_and_invalid_parameters_are_rejected(viewer):
    for body, origin, code in [
        ({"action": "play"}, "https://example.com", 403),
        ({"action": "parameter", "key": "friction", "value": 0}, None, 400),
        ({"action": "step", "manual": [0, float("nan"), 0]}, None, 400),
        ([], None, 400),
    ]:
        with pytest.raises(urllib.error.HTTPError) as error:
            post(viewer, body, origin)
        assert error.value.code == code
