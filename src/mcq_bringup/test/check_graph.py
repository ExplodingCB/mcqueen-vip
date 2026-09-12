#!/usr/bin/env python3
"""Graph smoke test for CI: with sim.launch.py running, exit 0 once the kart has
driven a set distance at speed with no stop requested, exit 1 on timeout.

    ros2 run mcq_bringup check_graph.py --distance 120 --speed 3 --timeout 90
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from mcq_msgs.msg import EgoState, GeofenceState, VehicleCommand


class Checker(Node):
    def __init__(self, args):
        super().__init__("check_graph")
        self.args = args
        self.start_s = None
        self.progress = 0.0
        self.last_s = None
        self.v_max = 0.0
        self.commands = 0
        self.stop_requests = 0  # while driving; a startup transient is not an intervention
        self.startup_stop_requests = 0
        self.geofence_violations = 0
        self.moving_since = None
        self.first_ego = None
        self.track_length = None
        qos = qos_profile_sensor_data
        self.create_subscription(EgoState, "ego_state", self.on_ego, qos)
        self.create_subscription(VehicleCommand, "vehicle_command", self.on_cmd, qos)
        self.create_subscription(GeofenceState, "geofence_state", self.on_geofence, qos)
        self.t0 = time.monotonic()
        self.last_report = self.t0

    def on_ego(self, msg):
        if self.first_ego is None:
            self.first_ego = time.monotonic() - self.t0
            self.get_logger().info(f"first ego state after {self.first_ego:.2f} s")
        if self.moving_since is None and msg.v > 0.5:
            self.moving_since = time.monotonic() - self.t0
            self.get_logger().info(f"kart moving after {self.moving_since:.2f} s")
        if self.last_s is not None:
            ds = msg.s - self.last_s
            if ds < -50.0:  # wrapped past the start line
                ds = 0.0
            self.progress += max(ds, 0.0)
        self.last_s = msg.s
        self.v_max = max(self.v_max, msg.v)

    def on_cmd(self, msg):
        self.commands += 1
        if msg.request_urgent_stop:
            if self.moving_since is None:
                self.startup_stop_requests += 1
            else:
                self.stop_requests += 1

    def on_geofence(self, msg):
        if msg.violation:
            self.geofence_violations += 1

    def status(self):
        return (
            f"progress {self.progress:.1f} m, v_max {self.v_max:.2f} m/s, commands {self.commands}, "
            f"stop requests {self.stop_requests} (startup {self.startup_stop_requests}), "
            f"geofence violations {self.geofence_violations}"
        )

    def verdict(self):
        elapsed = time.monotonic() - self.t0
        if elapsed - (self.last_report - self.t0) >= 5.0:
            self.get_logger().info(self.status())
            self.last_report = time.monotonic()
        if self.stop_requests or self.geofence_violations:
            self.get_logger().error("FAIL: " + self.status())
            return 1
        if self.progress >= self.args.distance and self.v_max >= self.args.speed:
            self.get_logger().info("PASS: " + self.status())
            return 0
        if elapsed > self.args.timeout:
            self.get_logger().error("FAIL (timeout): " + self.status())
            return 1
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--distance", type=float, default=120.0, help="metres to cover")
    parser.add_argument("--speed", type=float, default=3.0, help="peak speed to reach, m/s")
    parser.add_argument("--timeout", type=float, default=90.0)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Checker(args)
    code = None
    while code is None and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        code = node.verdict()
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code if code is not None else 1)


if __name__ == "__main__":
    main()
