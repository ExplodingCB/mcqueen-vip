"""ROS 2 planner prototype consuming the authoritative TrackModel and geofence.

Track files are loaded by mcq_track/track_server. The simulator retains its own
physical-world track so a bad surveyed map cannot move the simulated world.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from mcq_msgs.msg import EgoState, GeofenceState, TrackModel
from mcq_msgs.msg import Trajectory as TrajectoryMsg
from mcq_sim.params import DEFAULT_CONFIG, load_params
from mcq_sim.planner import FrenetPlanner, PlannerParams
from mcq_sim.ros_utils import pose_to_yaw, trajectory_to_msg
from mcq_sim.track_model import model_signature, reference_from_model, track_from_model


class PlannerNode(Node):
    def __init__(self):
        super().__init__("planner_node")
        self.declare_parameter("config", str(DEFAULT_CONFIG))
        self.declare_parameter("mode", "FOLLOW")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("speed_cap", -1.0)  # < 0: use the config value
        self.declare_parameter("wheelbase", 1.05)
        self.declare_parameter("steer_max", 0.45)
        self.declare_parameter("geofence_max_age", 0.2)
        self.declare_parameter("track_max_age", 0.5)

        self.track = None
        self.model_key = None
        self.model_received = None
        params = load_params(self.get_parameter("config").value)
        pp = PlannerParams.from_dict(params.planner)
        cap = float(self.get_parameter("speed_cap").value)
        if cap > 0:
            pp.v_cap = cap
            pp.boundary_v_cap = min(cap, pp.boundary_v_cap)
        pp.kappa_max = float(np.tan(self.get_parameter("steer_max").value) / self.get_parameter("wheelbase").value)
        self.pp = pp
        self.mode = str(self.get_parameter("mode").value)
        self.planner = None
        self.geofence = None

        self.ego: EgoState | None = None
        self.stop_requested = False
        qos = qos_profile_sensor_data
        self.pub_traj = self.create_publisher(TrajectoryMsg, "trajectory", qos)
        self.create_subscription(EgoState, "ego_state", self.on_ego, qos)
        self.create_subscription(GeofenceState, "geofence_state", self.on_geofence, qos)
        model_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(TrackModel, "track_model", self.on_track, model_qos)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self.plan)
        self.get_logger().info(f"waiting for TrackModel, mode {self.mode}, cap {pp.v_cap} m/s")

    def on_ego(self, msg: EgoState):
        self.ego = msg

    def on_track(self, msg: TrackModel):
        key = model_signature(msg)
        if key != self.model_key:
            try:
                track = track_from_model(msg)
                reference, raceline = reference_from_model(msg, track)
            except ValueError as error:
                self.get_logger().error(f"invalid TrackModel: {error}")
                self.planner = None
                self.model_key = None
                self.stop_requested = True
                return
            self.track = track
            self.planner = FrenetPlanner(reference, self.pp, mode=self.mode, raceline=raceline)
            self.model_key = key
            self.get_logger().info(f"planning on '{track.track_id}'")
        self.model_received = self.get_clock().now()

    def on_geofence(self, msg: GeofenceState):
        self.geofence = msg
        if msg.violation and msg.reason != GeofenceState.REASON_POSE_STALE:
            self.stop_requested = True

    def plan(self):
        if self.ego is None or self.planner is None:
            return
        x, y = self.ego.pose.position.x, self.ego.pose.position.y
        yaw = pose_to_yaw(self.ego.pose)
        v = float(self.ego.v)
        stamp = self.get_clock().now().to_msg()

        now = self.get_clock().now()
        geofence_age = (
            (now.nanoseconds - (self.geofence.header.stamp.sec * 10**9 + self.geofence.header.stamp.nanosec)) * 1e-9
            if self.geofence is not None
            else float("inf")
        )
        unavailable = (
            self.geofence is None
            or self.geofence.violation
            or geofence_age < -0.05
            or geofence_age > self.get_parameter("geofence_max_age").value
            or (now - self.model_received).nanoseconds * 1e-9 > self.get_parameter("track_max_age").value
        )

        bounds = None
        if self.mode == "BOUNDARY":
            # Stand-in for perception until mcq_perception publishes TrackBounds.
            bounds = self.track.bounds_ahead(x, y, yaw, self.pp.boundary_range, 1.0)
        traj = self.planner.plan(x, y, yaw, v, 0.0, stop_requested=self.stop_requested or unavailable, bounds=bounds)
        if not traj.feasible and not self.stop_requested:
            self.get_logger().error("no feasible path, requesting stop")
            self.stop_requested = True
        self.pub_traj.publish(trajectory_to_msg(traj, stamp))


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
