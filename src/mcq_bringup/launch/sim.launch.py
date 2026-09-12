"""The Phase 0 graph: simulator, planner and controller on a track file.

ros2 launch mcq_bringup sim.launch.py
ros2 launch mcq_bringup sim.launch.py mode:=BOUNDARY speed_cap:=4.0
ros2 launch mcq_bringup sim.launch.py track:=/path/to/tracks/my_track
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    track = LaunchConfiguration("track")
    mode = LaunchConfiguration("mode")
    speed_cap = LaunchConfiguration("speed_cap")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "track",
                default_value=PathJoinSubstitution([FindPackageShare("mcq_bringup"), "tracks", "synthetic_oval"]),
                description="track directory with track.csv and track.yaml",
            ),
            DeclareLaunchArgument("mode", default_value="FOLLOW", description="FOLLOW or BOUNDARY"),
            DeclareLaunchArgument("speed_cap", default_value="5.0", description="planner speed cap, m/s"),
            Node(
                package="mcq_sim",
                executable="sim_node",
                name="sim_node",
                output="screen",
                parameters=[{"track_dir": track}],
            ),
            Node(
                package="mcq_sim",
                executable="planner_node",
                name="planner_node",
                output="screen",
                parameters=[{"track_dir": track, "mode": mode, "speed_cap": speed_cap}],
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
