"""Exercise real generated messages and QoS in the Jazzy CI job."""

import math
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("mcq_msgs.msg")

from mcq_telemetry.node import TelemetryNode  # noqa: E402
from rclpy.qos import HistoryPolicy, ReliabilityPolicy  # noqa: E402

from mcq_msgs.msg import (  # noqa: E402
    EgoState,
    GatewayStatus,
    PlannerStatus,
    Trajectory,
    TrajectoryPoint,
    VehicleCommand,
)


class Capture:
    def publish(self, message):
        self.message = message


def test_summary_and_qos_with_real_ros_messages():
    track = Path(__file__).resolve().parents[3] / "tracks/synthetic_oval"
    rclpy.init(args=["--ros-args", "-p", f"track_dir:={track}"])
    node = TelemetryNode()
    try:
        for subscription in node.subscriptions:
            # Parameter events may also be present on some rclpy versions.
            if subscription.topic_name == "/parameter_events":
                continue
            assert subscription.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
            assert subscription.qos_profile.history == HistoryPolicy.KEEP_LAST
            assert subscription.qos_profile.depth == 1
        assert node.pub.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
        capture = Capture()
        node.pub = capture
        node.publish()
        assert not capture.message.ego_valid
        assert capture.message.gateway_mode == "UNKNOWN"
        assert math.isnan(capture.message.speed)
        assert math.isnan(capture.message.speed_cap)
        stamp = node.get_clock().now().to_msg()
        messages = {
            "gateway": GatewayStatus(mode=2, fault_flags=4, fault_latched=5),
            "ego": EgoState(v=3.0, d=-0.2, gnss_status=2),
            "trajectory": Trajectory(points=[TrajectoryPoint(t=0.0, v=4.0), TrajectoryPoint(t=1.0, v=4.0)]),
            "planner": PlannerStatus(submode="BOUNDARY", speed_cap=4.0),
            "command": VehicleCommand(request_urgent_stop=True),
        }
        messages["ego"].pose.position.x = -30.0
        messages["ego"].pose.position.y = -15.0
        for key, message in messages.items():
            message.header.stamp = stamp
            message.header.frame_id = "map"
            node.receive(key, message)
        node.publish()
        summary = capture.message
        assert summary.ego_valid and summary.gateway_valid and summary.trajectory_valid
        assert summary.speed_error == pytest.approx(-1)
        assert summary.gnss_status == "FIXED"
        assert summary.submode == "BOUNDARY" and summary.speed_cap == 4
        assert summary.fault_flags == 4 and summary.fault_latched == 5
        assert summary.active_faults == "CRC"
        assert "HEARTBEAT_TIMEOUT" in summary.latched_faults
        assert summary.urgent_stop
        assert summary.lap_active
        for message in messages.values():
            message.header.stamp.sec -= 10
        node.publish()
        assert not capture.message.ego_valid and not capture.message.lap_active
        assert capture.message.gateway_mode == "UNKNOWN"
        assert math.isnan(capture.message.target_speed)
    finally:
        node.destroy_node()
        rclpy.shutdown()
