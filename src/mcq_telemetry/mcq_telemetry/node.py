"""Separate best-effort observer process: cache inputs, publish a 10 Hz summary."""

import math
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from mcq_msgs.msg import EgoState, GatewayStatus, PlannerStatus, Telemetry, Trajectory, VehicleCommand
from mcq_telemetry.lap_timer import LapTimer
from mcq_telemetry.summary import age_and_valid, fault_names, stamp_seconds, target_speed


class TelemetryNode(Node):
    def __init__(self):
        super().__init__("telemetry")
        defaults = {
            "track_dir": "",
            "rate_hz": 10.0,
            "stale_after": 0.5,
            "start_tolerance": 0.25,
            "max_sample_gap": 0.5,
            "max_sample_step": 5.0,
            "distance_resolution": 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def get(name):
            return self.get_parameter(name).value

        self.stale_after = float(get("stale_after"))
        rate = float(get("rate_hz"))
        if self.stale_after <= 0 or rate <= 0:
            raise ValueError("stale_after and rate_hz must be positive")
        self.laps, self.track_id = LapTimer.from_track(
            get("track_dir"),
            start_tolerance=float(get("start_tolerance")),
            max_gap=float(get("max_sample_gap")),
            max_step=float(get("max_sample_step")),
            distance_resolution=float(get("distance_resolution")),
        )
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.fault_definitions = {
            name: getattr(GatewayStatus, name)
            for name in sorted(set(dir(GatewayStatus)) | set(dir(type(GatewayStatus))))
            if name.startswith("FAULT_")
        }
        self.latest = dict.fromkeys(("ego", "trajectory", "gateway", "planner", "command"))
        self.pub = self.create_publisher(Telemetry, "telemetry/summary", qos)
        self.pose_pub = self.create_publisher(PoseStamped, "telemetry/pose", qos)
        self.trail_pub = self.create_publisher(Path, "telemetry/trail", qos)
        self.track_pub = self.create_publisher(Path, "telemetry/track", qos)
        self.trail = deque(maxlen=max(1, int(rate * 30)))
        self.track_path = Path()
        self.track_path.header.frame_id = "map"
        for x, y in self.laps.centerline + self.laps.centerline[:1]:
            point = PoseStamped()
            point.header.frame_id = "map"
            point.pose.position.x = x
            point.pose.position.y = y
            point.pose.orientation.w = 1.0
            self.track_path.poses.append(point)
        for key, topic, message in (
            ("ego", "ego_state", EgoState),
            ("trajectory", "trajectory", Trajectory),
            ("gateway", "gateway_status", GatewayStatus),
            ("planner", "planner_status", PlannerStatus),
            ("command", "vehicle_command", VehicleCommand),
        ):
            self.create_subscription(message, topic, lambda msg, key=key: self.receive(key, msg), qos)
        self.create_timer(1.0 / rate, self.publish)
        self.create_timer(1.0, self.publish_paths)

    def receive(self, key, msg):
        self.latest[key] = msg
        if key == "ego":
            # Constant-time geometry at input rate avoids missing a crossing
            # between the 10 Hz publications. No file I/O or remote calls here.
            now = self.get_clock().now().nanoseconds * 1e-9
            gateway = self.latest["gateway"]
            if not age_and_valid(gateway, now, self.stale_after)[1]:
                if self.laps.previous is not None:
                    self.laps.invalidate()
                return
            if gateway.mode != 2:  # AUTO; do not include time waiting in RC in lap one.
                self.laps.invalidate(allow_standing_start=True)
                return
            if msg.header.frame_id != "map":
                self.laps.invalidate()
                return
            if age_and_valid(msg, now, self.stale_after)[1]:
                self.laps.update(stamp_seconds(msg.header.stamp), msg.pose.position.x, msg.pose.position.y)
            else:
                self.laps.invalidate()

    def publish(self):
        msg = Telemetry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        now = stamp_seconds(msg.header.stamp)
        msg.track_id = self.track_id
        for key, source in self.latest.items():
            age, valid = age_and_valid(source, now, self.stale_after)
            setattr(msg, key + "_age", float(age))
            setattr(msg, key + "_valid", valid)
        ego, traj = self.latest["ego"], self.latest["trajectory"]
        gateway, planner, command = (self.latest[k] for k in ("gateway", "planner", "command"))
        msg.speed = float(ego.v) if msg.ego_valid else math.nan
        msg.lateral_deviation = float(ego.d) if msg.ego_valid else math.nan
        msg.gnss_status = (
            {0: "NONE", 1: "FLOAT", 2: "FIXED"}.get(ego.gnss_status, "UNKNOWN") if msg.ego_valid else "UNKNOWN"
        )
        msg.trajectory_valid = msg.trajectory_valid and len(traj.points) >= 2
        msg.target_speed = float(target_speed(traj.points, msg.trajectory_age)) if msg.trajectory_valid else math.nan
        msg.speed_error = msg.speed - msg.target_speed
        modes = {0: "INIT", 1: "RC", 2: "AUTO", 3: "URGENT_STOP", 4: "DRIVETRAIN_OFF", 5: "FAULT"}
        msg.gateway_mode = modes.get(gateway.mode, "UNKNOWN") if msg.gateway_valid else "UNKNOWN"
        msg.active_faults = msg.latched_faults = "UNKNOWN"
        if msg.gateway_valid:
            msg.active_faults = fault_names(gateway.fault_flags, self.fault_definitions)
            msg.latched_faults = fault_names(gateway.fault_latched, self.fault_definitions)
            msg.fault_flags, msg.fault_latched = gateway.fault_flags, gateway.fault_latched
        msg.submode = planner.submode if msg.planner_valid else "UNKNOWN"
        msg.speed_cap = float(planner.speed_cap) if msg.planner_valid else math.nan
        msg.urgent_stop = bool(command.request_urgent_stop) if msg.command_valid else False
        if not msg.ego_valid and self.laps.previous is not None:
            self.laps.invalidate()
        msg.completed_laps = self.laps.completed_laps
        msg.lap_active = self.laps.started is not None
        msg.current_lap_time = self.laps.elapsed(now)
        msg.last_lap_time, msg.best_lap_time = self.laps.last_lap, self.laps.best_lap
        self.pub.publish(msg)
        if (
            msg.ego_valid
            and ego.header.frame_id == "map"
            and all(math.isfinite(value) for value in (ego.pose.position.x, ego.pose.position.y, ego.pose.position.z))
        ):
            pose = PoseStamped()
            pose.header = ego.header
            pose.pose = ego.pose
            self.pose_pub.publish(pose)
            if not self.trail or pose.header.stamp != self.trail[-1].header.stamp:
                self.trail.append(pose)
        else:
            self.trail.clear()

    def publish_paths(self):
        stamp = self.get_clock().now().to_msg()
        self.track_path.header.stamp = stamp
        for pose in self.track_path.poses:
            pose.header.stamp = stamp
        self.track_pub.publish(self.track_path)
        trail = Path()
        trail.header.frame_id = "map"
        trail.header.stamp = stamp
        trail.poses = list(self.trail)
        self.trail_pub.publish(trail)


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
