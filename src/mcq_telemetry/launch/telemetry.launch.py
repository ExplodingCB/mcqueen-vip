"""Start this launch and connect every pit client BEFORE entering AUTO."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("mcq_telemetry")
    return LaunchDescription(
        [
            DeclareLaunchArgument("track", description="directory containing track.yaml and track.csv"),
            DeclareLaunchArgument("port", default_value="8765"),
            Node(
                package="mcq_telemetry",
                executable="telemetry_node",
                name="telemetry",
                output="screen",
                prefix="nice -n 10",
                parameters=[
                    PathJoinSubstitution([share, "config", "telemetry.yaml"]),
                    {"track_dir": LaunchConfiguration("track")},
                ],
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                prefix="nice -n 10",
                parameters=[
                    PathJoinSubstitution([share, "config", "bridge.yaml"]),
                    {"port": ParameterValue(LaunchConfiguration("port"), value_type=int)},
                ],
            ),
        ]
    )
