"""Exercise smoke-checker callback ordering without a ROS installation."""

import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest


@pytest.fixture
def checker(monkeypatch):
    class Node:
        def __init__(self, name):
            self.logger = SimpleNamespace(info=lambda *_: None, warning=lambda *_: None, error=lambda *_: None)

        def get_logger(self):
            return self.logger

        def create_subscription(self, *args):
            return None

    rclpy = ModuleType("rclpy")
    node_module = ModuleType("rclpy.node")
    node_module.Node = Node
    qos_module = ModuleType("rclpy.qos")
    qos_module.qos_profile_sensor_data = object()
    messages = ModuleType("mcq_msgs.msg")
    for name in ("EgoState", "Trajectory", "VehicleCommand"):
        setattr(messages, name, object)
    messages.GeofenceState = SimpleNamespace(REASON_POSE_STALE=6)
    messages.VehicleState = SimpleNamespace(MODE_RC=1, MODE_AUTO=2)
    for name, module in {
        "rclpy": rclpy,
        "rclpy.node": node_module,
        "rclpy.qos": qos_module,
        "mcq_msgs": ModuleType("mcq_msgs"),
        "mcq_msgs.msg": messages,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location("graph_checker_test", Path(__file__).with_name("check_graph.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.Checker(SimpleNamespace(distance=120.0, speed=3.0, timeout=90.0))


def stale_pose():
    return SimpleNamespace(violation=True, reason=6, distance_to_edge=3.0)


@pytest.mark.parametrize("mode,expected_failure", [(1, False), (2, True), (5, True)])
def test_stale_pose_before_mode_waits_for_classification(checker, mode, expected_failure):
    checker.on_geofence(stale_pose())
    assert checker.geofence_violations == 0
    assert checker.startup_stale_poses == 0
    assert checker.verdict() is None
    checker.on_vehicle(SimpleNamespace(mode=mode))
    assert checker.geofence_violations == int(expected_failure)
    assert checker.startup_stale_poses == int(not expected_failure)
    assert checker.verdict() == (1 if expected_failure else None)


def test_pending_stale_pose_cannot_pass_with_missing_mode(checker):
    checker.on_geofence(stale_pose())
    checker.progress = 120.0
    checker.v_max = 4.0
    assert checker.verdict() is None
    checker.t0 -= 91.0
    assert checker.verdict() == 1


def test_motion_before_mode_fails_pending_stale_pose(checker):
    checker.on_geofence(stale_pose())
    checker.on_ego(SimpleNamespace(v=1.0, s=0.0))
    assert checker.geofence_violations == 1
    assert checker.verdict() == 1


@pytest.mark.parametrize("reason", [1, 2, 5])
def test_other_violation_reasons_fail_before_mode(checker, reason):
    checker.on_geofence(SimpleNamespace(violation=True, reason=reason, distance_to_edge=-1.0))
    assert checker.geofence_violations == 1
    assert checker.verdict() == 1


def test_stale_pose_after_auto_fails_even_after_returning_to_rc(checker):
    checker.on_vehicle(SimpleNamespace(mode=2))
    checker.on_vehicle(SimpleNamespace(mode=1))
    checker.on_geofence(stale_pose())
    assert checker.geofence_violations == 1
    assert checker.verdict() == 1


def test_stale_pose_during_rc_motion_fails(checker):
    checker.on_vehicle(SimpleNamespace(mode=1))
    checker.on_ego(SimpleNamespace(v=1.0, s=0.0))
    checker.on_geofence(stale_pose())
    assert checker.geofence_violations == 1
    assert checker.verdict() == 1


def test_known_stationary_rc_startup_counts_stale_pose(checker):
    checker.on_vehicle(SimpleNamespace(mode=1))
    checker.on_geofence(stale_pose())
    assert checker.geofence_violations == 0
    assert checker.startup_stale_poses == 1
    assert checker.verdict() is None
