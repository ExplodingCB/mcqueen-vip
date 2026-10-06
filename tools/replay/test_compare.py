import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location("replay_compare", Path(__file__).with_name("compare.py"))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
compare = module.compare
ReplayMismatch = module.ReplayMismatch


def check(expected, actual, tolerances=None):
    compare(expected, actual, topic="/vehicle_command", timestamp_ns=12_345_678_901, tolerances=tolerances or {})


def test_tolerance_boundary_and_named_first_divergence():
    check({"throttle": 0.5}, {"throttle": 0.500001}, {"throttle": {"abs": 0.00001}})
    with pytest.raises(ReplayMismatch, match=r"field=throttle timestamp=12\.345678901 .*expected=0.5 actual=0.6"):
        check({"throttle": 0.5}, {"throttle": 0.6}, {"throttle": {"abs": 0.00001}})


def test_relative_tolerance():
    check({"x": 1000.0}, {"x": 1000.5}, {"x": {"rel": 0.001}})
    with pytest.raises(ReplayMismatch):
        check({"x": 1.0}, {"x": 1.5}, {"x": {"rel": 0.001}})


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_is_never_a_pass(value):
    with pytest.raises(ReplayMismatch):
        check({"steering_angle": value}, {"steering_angle": value})


def test_unlisted_boolean_heartbeat_and_nested_fields_are_exact():
    for field, before, after in [("lat_enable", True, False), ("heartbeat", 1, 2), ("lat_enable", True, 1)]:
        with pytest.raises(ReplayMismatch, match=f"field={field}"):
            check({field: before}, {field: after})
    with pytest.raises(ReplayMismatch, match=r"field=header.stamp.nanosec"):
        check({"header": {"stamp": {"nanosec": 1}}}, {"header": {"stamp": {"nanosec": 2}}})


def test_array_length_and_missing_field_cannot_pass():
    with pytest.raises(ReplayMismatch):
        check({"points": [1, 2]}, {"points": [1]})
    with pytest.raises(ReplayMismatch):
        check({"brake": 0.0}, {})


def test_bad_and_misspelled_tolerances_cannot_hide_regression():
    for value in [-1.0, float("inf"), float("nan")]:
        with pytest.raises(ValueError, match="invalid tolerance"):
            check({"throttle": 0.5}, {"throttle": 0.5}, {"throttle": {"abs": value}})
    with pytest.raises(ValueError, match="unknown tolerance"):
        check({"throttle": 0.5}, {"throttle": 0.5}, {"throtle": {"abs": 0.1}})
