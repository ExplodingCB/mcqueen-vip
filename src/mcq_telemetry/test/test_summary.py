import math
from types import SimpleNamespace as NS

import pytest
from mcq_telemetry.summary import age_and_valid, fault_names, target_speed


def test_freshness_missing_stale_and_future():
    assert age_and_valid(None, 1, 0.5)[1] is False
    msg = NS(header=NS(stamp=NS(sec=1, nanosec=100_000_000)))
    assert age_and_valid(msg, 1.2, 0.5)[1] is True
    assert age_and_valid(msg, 2, 0.5)[1] is False
    assert age_and_valid(msg, 1, 0.5)[1] is False


def test_target_matches_controller_time_interpolation():
    points = [NS(t=0, v=2), NS(t=1, v=4), NS(t=2, v=0)]
    assert math.isnan(target_speed([], 0))
    assert target_speed(points, -1) == 2
    assert target_speed(points, 0.25) == pytest.approx(2.5)
    assert target_speed(points, 1.5) == 2
    assert target_speed(points, 10) == 0


def test_fault_names_preserve_unknown_bits():
    definitions = {"FAULT_CRC": 4, "FAULT_HEARTBEAT_TIMEOUT": 1}
    assert fault_names(0, definitions) == "NONE"
    assert fault_names(5, definitions) == "CRC, HEARTBEAT_TIMEOUT"
    assert fault_names(12, definitions) == "CRC, UNKNOWN_BITS(0x8)"
