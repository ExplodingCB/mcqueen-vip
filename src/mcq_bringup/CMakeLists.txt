cmake_minimum_required(VERSION 3.16)
project(mcq_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
# Track files live at the repository root so tools outside ROS reach them too;
# install a copy so launch files can find them in the install space.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../tracks DESTINATION share/${PROJECT_NAME})
install(PROGRAMS test/check_graph.py test/graph_smoke.sh DESTINATION lib/${PROJECT_NAME})

ament_package()
