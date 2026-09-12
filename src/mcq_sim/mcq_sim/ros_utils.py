"""Small helpers shared by the rclpy nodes. Importing this module requires ROS 2."""

from __future__ import annotations

import math

from geometry_msgs.msg import Pose

from mcq_msgs.msg import Trajectory as TrajectoryMsg
from mcq_msgs.msg import TrajectoryPoint
from mcq_sim.planner import Trajectory


def yaw_to_pose(x: float, y: float, yaw: float) -> Pose:
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def pose_to_yaw(pose: Pose) -> float:
    q = pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


REFERENCE_CODES = {
    "RACELINE": TrajectoryMsg.REFERENCE_RACELINE,
    "CENTERLINE": TrajectoryMsg.REFERENCE_CENTERLINE,
    "BOUNDARY": TrajectoryMsg.REFERENCE_BOUNDARY,
}


def trajectory_to_msg(traj: Trajectory, stamp, frame_id: str = "map") -> TrajectoryMsg:
    msg = TrajectoryMsg()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.reference = REFERENCE_CODES[traj.reference]
    msg.stop_requested = bool(traj.stop_requested)
    for i in range(len(traj.t)):
        p = TrajectoryPoint()
        p.t = float(traj.t[i])
        p.x = float(traj.x[i])
        p.y = float(traj.y[i])
        p.yaw = float(traj.yaw[i])
        p.kappa = float(traj.kappa[i])
        p.v = float(traj.v[i])
        p.a = float(traj.a[i])
        p.s = float(traj.s[i])
        p.d = float(traj.d[i])
        msg.points.append(p)
    return msg
