"""ROS 2 wrapper around the kart model.

Publishes what the kart's hardware publishes and consumes VehicleCommand:

    /vehicle_state, /gateway_status   what the gateway bridge reports, with the
                                      wheel speeds and steering angle as their
                                      encoders measure them, not as they are
    /gnss/fix, /gnss/fix_velocity     20 Hz, late, noise by fix status
    /imu/data_raw                     200 Hz, bias that walks
    /ego_truth                        the true state, for scoring only
    /ego_state                        the same truth, republished while
                                      `publish_ego_state` is true, standing in
                                      for mcq_localization until it exists

Nothing on the kart will ever publish /ego_truth, and the estimator is scored
against it (docs/05-roadmap.md, Phase 1: 0.10 m RMS). Turn `publish_ego_state`
off in the launch file the day the state estimator lands.

The gateway is emulated just enough for the graph to behave: a command older
than the heartbeat timeout or carrying request_urgent_stop puts the emulated
gateway in URGENT_STOP with full brake.

    ros2 run mcq_sim sim_node --ros-args -p track_dir:=tracks/synthetic_oval
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

from mcq_msgs.msg import EgoState, GatewayStatus, VehicleCommand, VehicleState
from mcq_sim.geodesy import enu_to_geodetic
from mcq_sim.params import DEFAULT_CONFIG, load_params
from mcq_sim.ros_utils import yaw_to_pose
from mcq_sim.sensors import GNSS_FIXED, GNSS_FLOAT, SensorSuite
from mcq_sim.track import Track
from mcq_sim.vehicle import KartSim, VehicleParams
from mcq_sim.vehicle import VehicleState as KartState

HEARTBEAT_TIMEOUT_S = 0.05
HEARTBEAT_CONTINUOUS_S = 1.0  # the real gateway's requirement before AUTO entry

# How our fix status reaches a NavSatFix. The message has no RTK float or fixed,
# so the distinction the estimator needs travels in the covariance and the
# status only says how the fix was obtained. Check this against what
# `ublox_dgnss` actually publishes when the real receiver arrives in Phase 1,
# because the estimator gates on it.
NAVSAT_STATUS = {
    GNSS_FIXED: NavSatStatus.STATUS_GBAS_FIX,
    GNSS_FLOAT: NavSatStatus.STATUS_FIX,
}


class SimNode(Node):
    def __init__(self):
        super().__init__("sim_node")
        self.declare_parameter("track_dir", "")
        self.declare_parameter("config", str(DEFAULT_CONFIG))
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("start_s", 0.0)
        self.declare_parameter("seed", 0)
        # Seconds after the heartbeat has been continuous for a second before
        # the emulated operator flips the switch to AUTO.
        self.declare_parameter("handover_delay", 2.0)
        # Republish the truth as /ego_state, standing in for the state
        # estimator. Set false once mcq_localization publishes it.
        self.declare_parameter("publish_ego_state", True)

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

        seed = int(self.get_parameter("seed").value)
        self.sensors = SensorSuite.from_params(params.sensors, seed=seed)
        datum = (params.sensors.get("datum") or self.track.meta.get("datum")) or {}
        self.datum = (
            float(datum.get("latitude", 0.0)),
            float(datum.get("longitude", 0.0)),
            float(datum.get("height", 0.0)),
        )
        self.t0 = None  # ROS time of the first step, the sensor clock's origin

        self.cmd: VehicleCommand | None = None
        self.cmd_time = None
        self.mode = VehicleState.MODE_RC
        self.faults = 0
        self.urgent_brake = 0.0
        self.fresh_since = None
        self.handover_delay = float(self.get_parameter("handover_delay").value)

        # Best-effort, keep-last on the high-rate topics so no slow subscriber
        # can back-pressure the loop; see docs/02-architecture.md section 4.
        qos = qos_profile_sensor_data
        self.pub_state = self.create_publisher(VehicleState, "vehicle_state", qos)
        self.pub_gateway = self.create_publisher(GatewayStatus, "gateway_status", qos)
        self.pub_truth = self.create_publisher(EgoState, "ego_truth", qos)
        self.publish_ego_state = bool(self.get_parameter("publish_ego_state").value)
        self.pub_ego = self.create_publisher(EgoState, "ego_state", qos) if self.publish_ego_state else None
        self.pub_fix = self.create_publisher(NavSatFix, "gnss/fix", qos)
        self.pub_fix_vel = self.create_publisher(TwistWithCovarianceStamped, "gnss/fix_velocity", qos)
        self.pub_imu = self.create_publisher(Imu, "imu/data_raw", qos)
        self.create_subscription(VehicleCommand, "vehicle_command", self.on_command, qos)
        self.create_timer(self.dt, self.step)
        self.get_logger().info(
            f"simulating on '{self.track.track_id}' ({self.track.length:.1f} m) at {1 / self.dt:.0f} Hz; "
            f"GNSS {self.sensors.gnss.params.rate_hz:.0f} Hz at {self.sensors.gnss.params.latency * 1e3:.0f} ms, "
            f"IMU {self.sensors.imu.params.rate_hz:.0f} Hz, seed {seed}"
            + ("; /ego_state is the truth until mcq_localization exists" if self.publish_ego_state else "")
        )

    def on_command(self, msg: VehicleCommand):
        self.cmd = msg
        self.cmd_time = self.get_clock().now()

    def set_mode(self, mode: int):
        if mode != self.mode:
            names = {v: k[5:] for k, v in vars(VehicleState).items() if k.startswith("MODE_")}
            self.get_logger().info(f"gateway {names.get(self.mode, self.mode)} -> {names.get(mode, mode)}")
            self.mode = mode

    def emulated_gateway(self):
        """Returns (steer, throttle, brake) after the gateway's own logic: RC
        until the heartbeat has been continuous and the operator hands over,
        AUTO while commands stay fresh, URGENT_STOP on timeout or request."""
        now = self.get_clock().now()
        fresh = self.cmd is not None and (now - self.cmd_time).nanoseconds * 1e-9 <= HEARTBEAT_TIMEOUT_S
        self.faults = 0
        if not fresh:
            self.fresh_since = None
        elif self.fresh_since is None:
            self.fresh_since = now
        continuous = (now - self.fresh_since).nanoseconds * 1e-9 if self.fresh_since is not None else 0.0

        if self.mode == VehicleState.MODE_AUTO:
            if not fresh:
                self.faults |= GatewayStatus.FAULT_HEARTBEAT_TIMEOUT
            elif self.cmd.request_urgent_stop:
                self.faults |= GatewayStatus.FAULT_JETSON_REQUEST
            if self.faults:
                self.set_mode(VehicleState.MODE_URGENT_STOP)
        elif self.mode == VehicleState.MODE_URGENT_STOP:
            # Standstill, then an operator reset back to RC.
            if self.sim.state.v < 0.05 and not (fresh and self.cmd.request_urgent_stop):
                self.set_mode(VehicleState.MODE_RC)
        if self.mode == VehicleState.MODE_RC:
            if continuous >= HEARTBEAT_CONTINUOUS_S + self.handover_delay and not self.cmd.request_urgent_stop:
                self.set_mode(VehicleState.MODE_AUTO)

        if self.mode == VehicleState.MODE_URGENT_STOP:
            self.urgent_brake = min(1.0, self.urgent_brake + self.dt / 0.3)  # 300 ms ramp
            steer = self.sim.state.steer if self.sim.state.v > 3.0 else 0.0
            return steer, 0.0, self.urgent_brake
        self.urgent_brake = 0.0
        if self.mode != VehicleState.MODE_AUTO:
            return 0.0, 0.0, 0.0  # RC with nobody on the transmitter: the kart sits still
        cmd = self.cmd
        steer = cmd.steering_angle if cmd.lat_enable else 0.0
        throttle = cmd.throttle if cmd.long_enable else 0.0
        brake = cmd.brake if cmd.long_enable else 0.0
        return steer, throttle, brake

    def step(self):
        steer, throttle, brake = self.emulated_gateway()
        st = self.sim.step(steer, throttle, brake)
        now = self.get_clock().now()
        if self.t0 is None:
            self.t0 = now
        t = (now - self.t0).nanoseconds * 1e-9
        stamp = now.to_msg()

        # The gateway reports what its encoders and hall sensors measure, which
        # is not what the simulator knows.
        wheels = self.sensors.wheels.read(st.v, st.throttle)
        measured_steer = self.sensors.steering.read(st.steer)

        vs = VehicleState()
        vs.header.stamp = stamp
        vs.header.frame_id = "base_link"
        vs.steering_angle = measured_steer
        vs.wheel_speed = [float(w) for w in wheels]
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
        # The gateway's speed limit works off the front wheels, so it sees the
        # measurement too, not the truth.
        gs.speed = float(0.5 * (wheels[0] + wheels[1]))
        self.pub_gateway.publish(gs)

        self.publish_sensors(t)
        self.publish_truth(t, stamp)

    def sensor_stamp(self, t: float):
        """A sample stamped when it was measured. GNSS arrives a latency late,
        so its stamp is already in the past when it is published, which is what
        the estimator has to deal with on the kart."""
        return (self.t0 + Duration(seconds=t)).to_msg()

    def publish_sensors(self, t: float):
        st = self.sim.state
        for sample in self.sensors.gnss.step(t, st.x, st.y, st.v * math.cos(st.yaw), st.v * math.sin(st.yaw)):
            self.publish_fix(sample)
        for sample in self.sensors.imu.step(t, st.a_long, st.v * st.yaw_rate, st.yaw_rate):
            self.publish_imu(sample)

    def publish_fix(self, sample):
        lat, lon, height = enu_to_geodetic(sample.x, sample.y, 0.0, self.datum)
        fix = NavSatFix()
        fix.header.stamp = self.sensor_stamp(sample.t)
        fix.header.frame_id = "gnss_antenna"
        fix.status.status = NAVSAT_STATUS.get(sample.status, NavSatStatus.STATUS_NO_FIX)
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = float(lat)
        fix.longitude = float(lon)
        fix.altitude = float(height)
        var = sample.sigma_pos**2
        # Vertical is about twice horizontal on an RTK fix.
        fix.position_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, 4.0 * var]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub_fix.publish(fix)

        vel = TwistWithCovarianceStamped()
        vel.header.stamp = fix.header.stamp
        # The axes are east and north, which is the map frame, not the antenna.
        vel.header.frame_id = "map"
        vel.twist.twist.linear.x = sample.vx
        vel.twist.twist.linear.y = sample.vy
        vel_var = sample.sigma_vel**2
        covariance = [0.0] * 36
        covariance[0] = covariance[7] = covariance[14] = vel_var
        vel.twist.covariance = covariance
        self.pub_fix_vel.publish(vel)

    def publish_imu(self, sample):
        p = self.sensors.imu.params
        msg = Imu()
        msg.header.stamp = self.sensor_stamp(sample.t)
        msg.header.frame_id = "imu_link"
        msg.linear_acceleration.x = sample.ax
        msg.linear_acceleration.y = sample.ay
        msg.linear_acceleration.z = sample.az
        msg.angular_velocity.x = sample.gx
        msg.angular_velocity.y = sample.gy
        msg.angular_velocity.z = sample.gz
        # A raw IMU reports no orientation; -1 in the first element is the
        # sensor_msgs way of saying so.
        msg.orientation_covariance = [-1.0] + [0.0] * 8
        accel_var, gyro_var = p.accel_sigma**2, p.gyro_sigma**2
        msg.linear_acceleration_covariance = [accel_var, 0.0, 0.0, 0.0, accel_var, 0.0, 0.0, 0.0, accel_var]
        msg.angular_velocity_covariance = [gyro_var, 0.0, 0.0, 0.0, gyro_var, 0.0, 0.0, 0.0, gyro_var]
        self.pub_imu.publish(msg)

    def publish_truth(self, t: float, stamp):
        """The true state. The pose is exact; `gnss_status` is not part of the
        pose but a fact about the receiver, so it reports the real status and a
        scripted outage reaches the downstream trust checks even while
        /ego_state is still this same message."""
        st = self.sim.state
        s, d = self.track.frenet(st.x, st.y)
        ego = EgoState()
        ego.header.stamp = stamp
        ego.header.frame_id = "map"
        ego.pose = yaw_to_pose(st.x, st.y, st.yaw)
        ego.v = float(st.v)
        ego.yaw_rate = float(st.yaw_rate)
        ego.a_long = float(st.a_long)
        ego.a_lat = float(st.v * st.yaw_rate)
        ego.gnss_status = self.sensors.gnss.status_at(t)
        ego.s = float(s[0])
        ego.d = float(d[0])
        self.pub_truth.publish(ego)
        if self.pub_ego is not None:
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
