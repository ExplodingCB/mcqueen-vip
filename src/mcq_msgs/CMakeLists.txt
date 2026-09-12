cmake_minimum_required(VERSION 3.16)
project(mcq_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/VehicleState.msg"
  "msg/VehicleCommand.msg"
  "msg/GatewayStatus.msg"
  "msg/EgoState.msg"
  "msg/TrackModel.msg"
  "msg/TrackBounds.msg"
  "msg/GeofenceState.msg"
  "msg/TrajectoryPoint.msg"
  "msg/Trajectory.msg"
  "msg/Detection.msg"
  "msg/Detections.msg"
)

set(srv_files
  "srv/LoadTrack.srv"
  "srv/SetAutoSubmode.srv"
  "srv/SetSpeedCap.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
