"""ROS 2 wrapper around the kart model.

Publishes what the kart would publish (VehicleState and GatewayStatus from the
gateway bridge, EgoState from localization) and consumes VehicleCommand. The
gateway is emulated just enough for the graph to behave: a command older than
the heartbeat timeout or carrying request_urgent_stop puts the emulated
gateway in URGENT_STOP with full brake.

    ros2 run mcq_sim sim_node --ros-args -p track_dir:=tracks/synthetic_oval
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from mcq_msgs.msg import EgoState, GatewayStatus, VehicleCommand, VehicleState
from mcq_sim.params import DEFAULT_CONFIG, load_params
from mcq_sim.ros_utils import yaw_to_pose
from mcq_sim.track import Track
from mcq_sim.vehicle import KartSim, VehicleParams
from mcq_sim.vehicle import VehicleState as KartState

HEARTBEAT_TIMEOUT_S = 0.05


class SimNode(Node):
    def __init__(self):
        super().__init__("sim_node")
        self.declare_parameter("track_dir", "")
        self.declare_parameter("config", str(DEFAULT_CONFIG))
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("start_s", 0.0)
        self.declare_parameter("seed", 0)

        track_dir = self.get_parameter("track_dir").value
        self.track = Track.load(track_dir) if track_dir else Track.synthetic_oval()
        params = load_params(self.get_parameter("config").value)
        self.dt = 1.0 / float(self.get_parameter("rate_hz").value)
        x0, y0, psi0 = self.track.cartesian(float(self.get_parameter("start_s").value), 0.0)
        start = KartState(x=float(x0[0]), y=float(y0[0]), yaw=float(psi0[0]))
        self.sim = KartSim(
            VehicleParams.from_dict(params.vehicle),
            start,
            dt=self.dt,
            seed=int(self.get_parameter("seed").value),
        )

        self.cmd: VehicleCommand | None = None
        self.cmd_time = None
        self.mode = VehicleState.MODE_AUTO
        self.faults = 0
        self.urgent_brake = 0.0

        # Best-effort, keep-last on the high-rate topics so no slow subscriber
        # can back-pressure the loop; see docs/02-architecture.md section 4.
        qos = qos_profile_sensor_data
        self.pub_state = self.create_publisher(VehicleState, "vehicle_state", qos)
        self.pub_gateway = self.create_publisher(GatewayStatus, "gateway_status", qos)
        self.pub_ego = self.create_publisher(EgoState, "ego_state", qos)
        self.create_subscription(VehicleCommand, "vehicle_command", self.on_command, qos)
        self.create_timer(self.dt, self.step)
        self.get_logger().info(
            f"simulating on '{self.track.track_id}' ({self.track.length:.1f} m) at {1 / self.dt:.0f} Hz"
        )

    def on_command(self, msg: VehicleCommand):
        self.cmd = msg
        self.cmd_time = self.get_clock().now()

    def emulated_gateway(self):
        """Returns (steer, throttle, brake) after the gateway's own logic."""
        now = self.get_clock().now()
        fresh = self.cmd is not None and (now - self.cmd_time).nanoseconds * 1e-9 <= HEARTBEAT_TIMEOUT_S
        self.faults = 0
        if not fresh:
            self.faults |= GatewayStatus.FAULT_HEARTBEAT_TIMEOUT
        elif self.cmd.request_urgent_stop:
            self.faults |= GatewayStatus.FAULT_JETSON_REQUEST
        if self.faults:
            self.mode = VehicleState.MODE_URGENT_STOP
            self.urgent_brake = min(1.0, self.urgent_brake + self.dt / 0.3)  # 300 ms ramp
            steer = self.sim.state.steer if self.sim.state.v > 3.0 else 0.0
            return steer, 0.0, self.urgent_brake
        self.urgent_brake = 0.0
        self.mode = VehicleState.MODE_AUTO
        cmd = self.cmd
        steer = cmd.steering_angle if cmd.lat_enable else 0.0
        throttle = cmd.throttle if cmd.long_enable else 0.0
        brake = cmd.brake if cmd.long_enable else 0.0
        return steer, throttle, brake

    def step(self):
        steer, throttle, brake = self.emulated_gateway()
        st = self.sim.step(steer, throttle, brake)
        stamp = self.get_clock().now().to_msg()

        vs = VehicleState()
        vs.header.stamp = stamp
        vs.header.frame_id = "base_link"
        vs.steering_angle = float(st.steer)
        vs.wheel_speed = [float(st.v)] * 4
        vs.brake_pressure = float(st.brake)
        vs.mode = self.mode
        vs.fault_flags = self.faults
        vs.heartbeat_echo = self.cmd.heartbeat if self.cmd is not None else 0
        self.pub_state.publish(vs)

        gs = GatewayStatus()
        gs.header = vs.header
        gs.mode = self.mode
        gs.fault_flags = self.faults
        gs.heartbeat_echo = vs.heartbeat_echo
        gs.rc_link_ok = True
        gs.contactor_closed = True
        gs.tx_auto_switch = True
        gs.speed = float(st.v)
        self.pub_gateway.publish(gs)

        mx, my, myaw = self.sim.measured_pose()
        s, d = self.track.frenet(mx, my)
        ego = EgoState()
        ego.header.stamp = stamp
        ego.header.frame_id = "map"
        ego.pose = yaw_to_pose(mx, my, myaw)
        ego.v = float(st.v)
        ego.yaw_rate = float(st.yaw_rate)
        ego.a_long = float(st.a_long)
        ego.a_lat = float(st.v * st.yaw_rate)
        ego.gnss_status = EgoState.GNSS_FIXED
        ego.s = float(s[0])
        ego.d = float(d[0])
        self.pub_ego.publish(ego)


def main(args=None):
    rclpy.init(args=args)
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
