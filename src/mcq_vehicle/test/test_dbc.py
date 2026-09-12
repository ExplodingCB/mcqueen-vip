"""The CAN definition parses, fits classic frames, and the firmware's generated
test vectors are in sync with it."""

import subprocess
import sys
from pathlib import Path

import cantools
import pytest

REPO = Path(__file__).resolve().parents[3]
DBC = REPO / "src" / "mcq_vehicle" / "dbc" / "mcqueen.dbc"


@pytest.fixture(scope="module")
def db():
    return cantools.database.load_file(DBC)


def test_frames_and_layouts(db):
    names = {m.name: m for m in db.messages}
    assert {
        "JETSON_COMMAND",
        "GATEWAY_STATUS",
        "GATEWAY_STEERING",
        "GATEWAY_WHEELS",
        "GATEWAY_MOTOR",
        "GATEWAY_LIMITS",
    } <= set(names)
    for m in db.messages:
        assert m.length == 8
        assert m.frame_id < 0x800  # standard identifiers only
        used = sorted((s.start, s.start + s.length) for s in m.signals)
        for (_, a1), (b0, _) in zip(used, used[1:], strict=False):
            assert a1 <= b0, f"{m.name}: overlapping signals"
        assert used[-1][1] <= 64
    cmd = names["JETSON_COMMAND"]
    assert cmd.frame_id == 0x100 and cmd.senders == ["JETSON"]
    assert cmd.get_signal_by_name("HEARTBEAT").length == 16
    assert cmd.get_signal_by_name("CRC").start == 56
    assert names["GATEWAY_STATUS"].get_signal_by_name("FAULT_FLAGS").length == 16


def test_mode_values_match_msg_constants(db):
    status = db.get_message_by_name("GATEWAY_STATUS")
    choices = status.get_signal_by_name("MODE").choices
    assert {int(k): str(v) for k, v in choices.items()} == {
        0: "INIT",
        1: "RC",
        2: "AUTO",
        3: "URGENT_STOP",
        4: "DRIVETRAIN_OFF",
        5: "FAULT",
    }
    msg = (REPO / "src" / "mcq_msgs" / "msg" / "VehicleState.msg").read_text()
    for value, name in choices.items():
        assert f"MODE_{name} = {int(value)}" in msg


def test_command_roundtrip(db):
    cmd = db.get_message_by_name("JETSON_COMMAND")
    fields = {
        "STEERING_ANGLE": -0.25,
        "THROTTLE": 0.5,
        "BRAKE": 0.125,
        "LAT_ENABLE": 1,
        "LONG_ENABLE": 0,
        "REQUEST_URGENT_STOP": 1,
        "RESERVED": 0,
        "HEARTBEAT": 65535,
        "CRC": 0x5A,
    }
    decoded = cmd.decode(cmd.encode(fields))
    for k, v in fields.items():
        assert decoded[k] == pytest.approx(v, abs=1e-3)


@pytest.mark.parametrize(
    "script",
    [
        REPO / "firmware" / "gateway" / "tools" / "gen_can_vectors.py",
        REPO / "src" / "mcq_vehicle" / "tools" / "gen_can_code.py",
    ],
)
def test_generated_artifacts_up_to_date(script):
    proc = subprocess.run([sys.executable, str(script), "--check"], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr
