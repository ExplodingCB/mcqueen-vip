#!/usr/bin/env python3
"""Graph acceptance: FOLLOW driving, failed reload, real shifted geometry or pose covariance, gateway stop."""

import argparse
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from mcq_msgs.msg import EgoState, GeofenceState, TrackModel, VehicleCommand, VehicleState
from mcq_msgs.srv import LoadTrack
from mcq_sim.track import Track


def prepare(directory):
    # A centered kart on the usual 5 m oval stays inside after a 2 m shift:
    # half-width 2.5 m plus 0.5 m inflation. Use explicit 2 m wide pavement.
    original = Track.synthetic_oval(width=2.0)
    original.save(directory / "original")
    shifted = original.shifted(0, 2)
    shifted.track_id = "shifted_2m"
    shifted.meta["track_id"] = shifted.track_id
    shifted.save(directory / "shifted")
    bad = directory / "invalid"
    bad.mkdir(parents=True, exist_ok=True)
    (bad / "track.yaml").write_text("track_id: invalid\nclosed: true\n")
    (bad / "track.csv").write_text("0,0,1,1\n1,nan,1,1\n2,0,1,1\n")


class Checker(Node):
    def __init__(self, args):
        super().__init__("check_geofence")
        self.args = args
        self.started = time.monotonic()
        self.injected = None
        self.ready = False
        self.invalid_future = None
        self.reload_future = None
        self.invalid_rejected = False
        self.reloaded = False
        self.clean = False
        self.reason_seen = False
        self.stop_seen = False
        self.gateway_seen = False
        self.model_changed = False
        self.v = 0.0
        self.distance = None
        self.failure = None
        self.model_count = 0
        qos = qos_profile_sensor_data
        self.ego_pub = self.create_publisher(EgoState, "ego_state", qos)
        self.create_subscription(EgoState, "ego_truth", self.on_ego, qos)
        self.create_subscription(GeofenceState, "geofence_state", self.on_geofence, qos)
        self.create_subscription(VehicleCommand, "vehicle_command", self.on_command, qos)
        self.create_subscription(VehicleState, "vehicle_state", self.on_vehicle, qos)
        model_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(TrackModel, "track_model", self.on_model, model_qos)
        self.client = self.create_client(LoadTrack, "mcq/load_track")

    def on_model(self, msg):
        self.model_count += 1
        if msg.track_id == "shifted_2m":
            self.model_changed = True

    def on_ego(self, msg):
        self.v = msg.v
        if self.injected is not None and self.args.scenario == "covariance":
            # Both diagonal variances pass 0.25; the principal variance is 0.30.
            msg.covariance[0] = msg.covariance[7] = 0.2
            msg.covariance[1] = msg.covariance[6] = 0.1
        self.ego_pub.publish(msg)

    def on_geofence(self, msg):
        if not msg.violation:
            self.clean = True
        elif self.injected is None and msg.reason != GeofenceState.REASON_POSE_STALE:
            self.failure = f"unexpected pre-injection geofence reason {msg.reason}"
        elif self.injected is not None:
            expected = (
                GeofenceState.REASON_OUTSIDE_TRACK if self.args.scenario == "shift" else GeofenceState.REASON_COVARIANCE
            )
            if msg.reason == expected:
                self.reason_seen = True
                self.distance = msg.distance_to_edge

    def on_command(self, msg):
        if self.reason_seen and msg.request_urgent_stop:
            self.stop_seen = True

    def on_vehicle(self, msg):
        if self.injected is not None and msg.mode == VehicleState.MODE_URGENT_STOP:
            self.gateway_seen = True
        self.ready = msg.mode == VehicleState.MODE_AUTO

    def tick(self):
        elapsed = time.monotonic() - self.started
        if self.failure:
            raise AssertionError(self.failure)
        if self.invalid_future is None and self.client.service_is_ready() and self.clean:
            request = LoadTrack.Request()
            request.path = str(self.args.directory / "invalid")
            self.invalid_future = self.client.call_async(request)
        if self.invalid_future is not None and self.invalid_future.done() and not self.invalid_rejected:
            response = self.invalid_future.result()
            if response.success or response.track_id != "synthetic_oval":
                raise AssertionError("failed reload did not preserve active track")
            self.invalid_rejected = True
        if self.injected is None and self.ready and self.v > 2 and self.clean and self.invalid_rejected:
            self.injected = time.monotonic()
            self.get_logger().info(f"injecting {self.args.scenario} while FOLLOW drives at {self.v:.2f} m/s")
            if self.args.scenario == "shift":
                request = LoadTrack.Request()
                request.path = str(self.args.directory / "shifted")
                self.reload_future = self.client.call_async(request)
            else:
                self.reloaded = True
        if self.reload_future is not None and self.reload_future.done():
            response = self.reload_future.result()
            if not response.success or response.track_id != "shifted_2m":
                raise AssertionError(f"shifted load failed: {response.message}")
            self.reloaded = True
        if self.reason_seen and self.stop_seen and self.gateway_seen and self.reloaded and self.v < 0.05:
            if self.args.scenario == "shift" and (not self.model_changed or self.distance >= 0):
                raise AssertionError("shift did not replace TrackModel and leave the inflated polygon")
            if self.args.scenario == "covariance" and self.distance <= 0:
                raise AssertionError("covariance test must fire while position remains inside")
            if self.model_count < 10:
                raise AssertionError("TrackModel did not publish periodically")
            self.get_logger().info(
                f"PASS {self.args.scenario}: geofence, urgent-stop command, gateway stop, standstill; "
                f"inflated edge distance {self.distance:.3f} m; invalid reload preserved active track"
            )
            return True
        if elapsed > 60 or (self.injected is not None and time.monotonic() - self.injected > 8):
            raise AssertionError(
                f"timeout: v={self.v:.2f}, clean={self.clean}, injected={self.injected is not None}, "
                f"geofence={self.reason_seen}, command={self.stop_seen}, gateway={self.gateway_seen}"
            )
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--prepare", action="store_true")
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--scenario", choices=["shift", "covariance"], default="shift")
    args = parser.parse_args()
    if args.prepare:
        prepare(args.directory)
        return
    rclpy.init()
    checker = Checker(args)
    try:
        while rclpy.ok():
            rclpy.spin_once(checker, timeout_sec=0.02)
            if checker.tick():
                return
        raise RuntimeError("ROS stopped before acceptance")
    except (AssertionError, RuntimeError) as error:
        checker.get_logger().error(str(error))
        sys.exit(1)
    finally:
        checker.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
