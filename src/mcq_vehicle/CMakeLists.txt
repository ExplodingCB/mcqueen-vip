cmake_minimum_required(VERSION 3.16)
project(mcq_vehicle C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mcq_msgs REQUIRED)

# Frame codec generated from the DBC plus the gateway's CRC, so the bridge and
# the firmware share the exact same bytes.
set(firmware_dir ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/gateway)
add_library(mcqueen_can STATIC generated/mcqueen.c ${firmware_dir}/src/crc8.c)
target_include_directories(mcqueen_can PUBLIC generated ${firmware_dir}/include)
set_target_properties(mcqueen_can PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(gateway_bridge_component SHARED src/gateway_bridge.cpp)
target_compile_options(gateway_bridge_component PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(gateway_bridge_component PUBLIC
  mcqueen_can
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${mcq_msgs_TARGETS}
)
rclcpp_components_register_node(gateway_bridge_component
  PLUGIN "mcq_vehicle::GatewayBridge"
  EXECUTABLE gateway_bridge
)

install(TARGETS gateway_bridge_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY dbc DESTINATION share/${PROJECT_NAME})

ament_package()
