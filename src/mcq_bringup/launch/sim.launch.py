"""The Phase 0 graph: simulator, planner and controller on a track file.

ros2 launch mcq_bringup sim.launch.py
ros2 launch mcq_bringup sim.launch.py mode:=BOUNDARY speed_cap:=4.0
ros2 launch mcq_bringup sim.launch.py track:=/path/to/tracks/my_track
ros2 launch mcq_bringup sim.launch.py ego_from_sim:=false   # with a real estimator
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    track = LaunchConfiguration("track")
    mode = LaunchConfiguration("mode")
    speed_cap = LaunchConfiguration("speed_cap")
    record = LaunchConfiguration("record")
    bag = LaunchConfiguration("bag")
    return LaunchDescription(
        [
            DeclareLaunchArgument("record", default_value="false", description="record every topic to MCAP"),
            DeclareLaunchArgument("bag", default_value="sim_session", description="bag directory when recording"),
            # Recording runs at low priority: it must never starve the control loop.
            ExecuteProcess(
                cmd=["nice", "-n", "10", "ros2", "bag", "record", "-s", "mcap", "-o", bag, "-a"],
                output="screen",
                condition=IfCondition(record),
            ),
            DeclareLaunchArgument(
                "track",
                default_value=PathJoinSubstitution([FindPackageShare("mcq_bringup"), "tracks", "synthetic_oval"]),
                description="track directory with track.csv and track.yaml",
            ),
            DeclareLaunchArgument("mode", default_value="FOLLOW", description="FOLLOW or BOUNDARY"),
            DeclareLaunchArgument(
                "geofence_output", default_value="geofence_state", description="track-server verdict topic"
            ),
            DeclareLaunchArgument(
                "ego_from_sim",
                default_value="true",
                description="publish the simulator's truth as /ego_state; false once mcq_localization runs",
            ),
            DeclareLaunchArgument("speed_cap", default_value="5.0", description="planner speed cap, m/s"),
            Node(
                package="mcq_sim",
                executable="sim_node",
                name="sim_node",
                output="screen",
                parameters=[{"track_dir": track, "publish_ego_state": LaunchConfiguration("ego_from_sim")}],
            ),
            Node(
                package="mcq_track",
                executable="track_server",
                name="track_server",
                output="screen",
                remappings=[("geofence_state", LaunchConfiguration("geofence_output"))],
                parameters=[
                    PathJoinSubstitution([FindPackageShare("mcq_track"), "config", "track.yaml"]),
                    {"track_dir": track},
                ],
            ),
            Node(
                package="mcq_sim",
                executable="planner_node",
                name="planner_node",
                output="screen",
                parameters=[{"mode": mode, "speed_cap": speed_cap}],
            ),
            Node(
                package="mcq_control",
                executable="controller_node",
                name="controller",
                output="screen",
                parameters=[PathJoinSubstitution([FindPackageShare("mcq_control"), "config", "controller.yaml"])],
            ),
        ]
    )
