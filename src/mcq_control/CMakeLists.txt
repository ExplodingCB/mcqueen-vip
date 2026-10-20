cmake_minimum_required(VERSION 3.16)
project(mcq_control C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mcq_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# The control math, built exactly as it is built standalone (host tests included,
# they run under colcon test as ctest tests).
add_subdirectory(core)

add_library(controller_component SHARED src/controller_node.cpp)
target_compile_options(controller_component PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(controller_component PUBLIC
  mcq_control_core
  rclcpp::rclcpp
  rclcpp_components::component
  ${mcq_msgs_TARGETS}
  ${std_msgs_TARGETS}
)
rclcpp_components_register_node(controller_component
  PLUGIN "mcq_control::ControllerNode"
  EXECUTABLE controller_node
)

install(TARGETS mcq_control_core controller_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY core/include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(mcq_control_core)
ament_package()
