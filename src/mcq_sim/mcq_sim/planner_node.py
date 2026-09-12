"""ROS 2 wrapper around the Frenet planner prototype (the future local_planner).

Subscribes EgoState, plans at 20 Hz on the loaded track and publishes
Trajectory. Runs the geofence check from docs/04-safety.md section 5 on the
believed map and turns a violation into a braking trajectory.

    ros2 run mcq_sim planner_node --ros-args -p track_dir:=tracks/synthetic_oval
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node

from mcq_msgs.msg import EgoState, GeofenceState
from mcq_msgs.msg import Trajectory as TrajectoryMsg
from mcq_sim.params import DEFAULT_CONFIG, load_params
from mcq_sim.planner import FrenetPlanner, PlannerParams
from mcq_sim.ros_utils import pose_to_yaw, trajectory_to_msg
from mcq_sim.track import Track


class PlannerNode(Node):
    def __init__(self):
        super().__init__("planner_node")
        self.declare_parameter("track_dir", "")
        self.declare_parameter("config", str(DEFAULT_CONFIG))
        self.declare_parameter("mode", "FOLLOW")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("speed_cap", -1.0)  # < 0: use the config value
        self.declare_parameter("wheelbase", 1.05)
        self.declare_parameter("steer_max", 0.45)
        self.declare_parameter("geofence_margin", 0.5)

        track_dir = self.get_parameter("track_dir").value
        self.track = Track.load(track_dir) if track_dir else Track.synthetic_oval()
        params = load_params(self.get_parameter("config").value)
        pp = PlannerParams.from_dict(params.planner)
        cap = float(self.get_parameter("speed_cap").value)
        if cap > 0:
            pp.v_cap = cap
            pp.boundary_v_cap = min(cap, pp.boundary_v_cap)
        pp.kappa_max = float(np.tan(self.get_parameter("steer_max").value) / self.get_parameter("wheelbase").value)
        self.pp = pp
        self.mode = str(self.get_parameter("mode").value)
        self.planner = FrenetPlanner(self.track, pp, mode=self.mode)
        self.margin = float(self.get_parameter("geofence_margin").value)

        self.ego: EgoState | None = None
        self.stop_requested = False
        self.pub_traj = self.create_publisher(TrajectoryMsg, "trajectory", 10)
        self.pub_geofence = self.create_publisher(GeofenceState, "geofence_state", 10)
        self.create_subscription(EgoState, "ego_state", self.on_ego, 10)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self.plan)
        self.get_logger().info(f"planning in {self.mode} mode on '{self.track.track_id}', cap {pp.v_cap} m/s")

    def on_ego(self, msg: EgoState):
        self.ego = msg

    def plan(self):
        if self.ego is None:
            return
        x, y = self.ego.pose.position.x, self.ego.pose.position.y
        yaw = pose_to_yaw(self.ego.pose)
        v = float(self.ego.v)
        stamp = self.get_clock().now().to_msg()

        edge = float(self.track.distance_to_edge(x, y)[0])
        gf = GeofenceState()
        gf.header.stamp = stamp
        gf.header.frame_id = "map"
        gf.distance_to_edge = edge
        gf.margin = self.margin
        if edge < -self.margin:
            gf.violation = True
            gf.reason = GeofenceState.REASON_OUTSIDE_TRACK
            if not self.stop_requested:
                self.get_logger().error(f"geofence violation: {edge:.2f} m outside the edge, requesting stop")
            self.stop_requested = True
        self.pub_geofence.publish(gf)

        bounds = None
        if self.mode == "BOUNDARY":
            # Stand-in for perception until mcq_perception publishes TrackBounds.
            bounds = self.track.bounds_ahead(x, y, yaw, self.pp.boundary_range, 1.0)
        traj = self.planner.plan(x, y, yaw, v, 0.0, stop_requested=self.stop_requested, bounds=bounds)
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
