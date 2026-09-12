"""End-to-end check of the gateway core over a real SocketCAN interface.

Needs a vcan0 interface (sudo modprobe vcan; sudo ip link add dev vcan0 type
vcan; sudo ip link set up vcan0), python-can, cantools and a built
firmware/gateway/build/gateway_host. Skips otherwise. A Python "Jetson" sends
JETSON_COMMAND frames at 100 Hz and reads GATEWAY_STATUS back, walking the
mode transitions of docs/04-safety.md over the bus.
"""

from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

import pytest

can = pytest.importorskip("can")
cantools = pytest.importorskip("cantools")

REPO = Path(__file__).resolve().parents[3]
BINARY = REPO / "firmware" / "gateway" / "build" / "gateway_host"
DBC = REPO / "src" / "mcq_vehicle" / "dbc" / "mcqueen.dbc"
INTERFACE = "vcan0"

sys.path.insert(0, str(REPO / "firmware" / "gateway" / "tools"))
from gen_can_vectors import crc8_j1850  # noqa: E402

pytestmark = pytest.mark.skipif(
    not Path(f"/sys/class/net/{INTERFACE}").exists() or not BINARY.exists(),
    reason=f"needs {INTERFACE} and a built gateway_host",
)


class Rig:
    def __init__(self):
        self.proc = subprocess.Popen(
            [str(BINARY), "--interface", INTERFACE],
            stdin=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        self.bus = can.Bus(INTERFACE, interface="socketcan")
        self.db = cantools.database.load_file(DBC)
        self.cmd_msg = self.db.get_message_by_name("JETSON_COMMAND")
        self.status_msg = self.db.get_message_by_name("GATEWAY_STATUS")
        self.counter = 100
        self.last_status = None

    def close(self):
        try:
            self.send("quit")
            self.proc.wait(timeout=5)
        finally:
            self.bus.shutdown()

    def send(self, line: str):
        self.proc.stdin.write(line + "\n")
        self.proc.stdin.flush()

    def drain(self):
        while True:
            frame = self.bus.recv(timeout=0)
            if frame is None:
                return
            if frame.arbitration_id == self.status_msg.frame_id:
                self.last_status = self.status_msg.decode(bytes(frame.data))

    def mode(self) -> str | None:
        self.drain()
        return str(self.last_status["MODE"]) if self.last_status else None

    def wait_mode(self, name: str, timeout: float, jetson: bool = False, **cmd) -> str | None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if jetson:
                self.jetson_frame(**cmd)
            time.sleep(0.01)
            if self.mode() == name:
                return name
        return self.mode()

    def jetson_frame(self, steering=0.0, throttle=0.0, brake=0.0, urgent=False, freeze=False):
        if not freeze:
            self.counter = (self.counter + 1) & 0xFFFF
        data = bytearray(
            self.cmd_msg.encode(
                {
                    "STEERING_ANGLE": steering,
                    "THROTTLE": throttle,
                    "BRAKE": brake,
                    "LAT_ENABLE": 1,
                    "LONG_ENABLE": 1,
                    "REQUEST_URGENT_STOP": int(urgent),
                    "RESERVED": 0,
                    "HEARTBEAT": self.counter,
                    "CRC": 0,
                }
            )
        )
        data[7] = crc8_j1850(bytes(data[:7]))
        self.bus.send(can.Message(arbitration_id=self.cmd_msg.frame_id, data=bytes(data), is_extended_id=False))

    def run_jetson(self, seconds: float, **cmd):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.jetson_frame(**cmd)
            time.sleep(0.01)
            self.drain()

    def enter_auto(self):
        assert self.wait_mode("RC", 2.0) == "RC"
        self.run_jetson(1.3)
        self.send("auto on")
        assert self.wait_mode("AUTO", 0.5, jetson=True) == "AUTO"


@pytest.fixture
def rig():
    r = Rig()
    try:
        yield r
    finally:
        r.close()


def test_boots_to_rc_and_reports_limits(rig):
    assert rig.wait_mode("RC", 2.0) == "RC"
    assert rig.last_status["CONTACTOR_CLOSED"] == 1
    assert rig.last_status["RC_LINK_OK"] == 1


def test_auto_entry_then_heartbeat_loss_then_reset(rig):
    rig.enter_auto()
    rig.run_jetson(0.3, throttle=0.2)
    assert rig.mode() == "AUTO"
    assert rig.last_status["HEARTBEAT_ECHO"] == rig.counter
    # Jetson dies: urgent stop within the timeout, then reset at standstill.
    assert rig.wait_mode("URGENT_STOP", 0.3) == "URGENT_STOP"
    rig.send("speed 0")
    time.sleep(0.7)
    rig.send("reset")
    assert rig.wait_mode("RC", 0.5) == "RC"


def test_frozen_counter_is_rejected(rig):
    rig.enter_auto()
    assert rig.wait_mode("URGENT_STOP", 0.5, jetson=True, freeze=True) == "URGENT_STOP"
    assert rig.last_status["FRAMES_REJECTED"] > 0


def test_auto_refused_above_handover_speed(rig):
    assert rig.wait_mode("RC", 2.0) == "RC"
    rig.send("speed 4.0")
    rig.run_jetson(1.3)
    rig.send("auto on")
    assert rig.wait_mode("AUTO", 0.4, jetson=True) == "RC"
    assert int(rig.last_status["FAULT_FLAGS"]) & (1 << 13)  # HANDOVER_REFUSED was set at the attempt


def test_remote_estop_sequence(rig):
    rig.enter_auto()
    rig.send("estop on")
    assert rig.wait_mode("URGENT_STOP", 0.3, jetson=True) == "URGENT_STOP"
    assert rig.last_status["CONTACTOR_CLOSED"] == 0
    assert rig.wait_mode("DRIVETRAIN_OFF", 1.5, jetson=True) == "DRIVETRAIN_OFF"
    rig.send("estop off")
    rig.send("manual-reset")
    assert rig.wait_mode("RC", 0.5) == "RC"
