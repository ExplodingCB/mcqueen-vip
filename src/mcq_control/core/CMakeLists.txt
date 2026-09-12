cmake_minimum_required(VERSION 3.16)
project(mcq_control_core C)

# Dependency-free C library holding the control-loop math: PID, bicycle model,
# pure pursuit, curvature and rate limits, the longitudinal state machine and
# the speed profile. The ROS 2 node in the parent package is a thin wrapper
# around this; the Python simulator loads it through ctypes so the same code
# is exercised off the kart. Builds standalone with no ROS installed.

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(MCQ_CORE_BUILD_TESTS "Build host-side unit tests" ON)

add_library(mcq_control_core SHARED
  src/pid.c
  src/bicycle.c
  src/pure_pursuit.c
  src/limits.c
  src/longitudinal.c
  src/speed_profile.c
)
target_include_directories(mcq_control_core PUBLIC include)
target_compile_options(mcq_control_core PRIVATE
  -Wall -Wextra -Wpedantic -Wshadow -Wdouble-promotion -Wconversion -Werror -O2 -fPIC
)
target_compile_definitions(mcq_control_core PRIVATE _USE_MATH_DEFINES)
target_link_libraries(mcq_control_core PUBLIC m)

if(MCQ_CORE_BUILD_TESTS)
  enable_testing()
  foreach(name pid bicycle pure_pursuit limits longitudinal speed_profile)
    add_executable(test_${name} test/test_${name}.c)
    target_link_libraries(test_${name} PRIVATE mcq_control_core)
    target_include_directories(test_${name} PRIVATE test)
    target_compile_options(test_${name} PRIVATE -Wall -Wextra -Werror)
    add_test(NAME ${name} COMMAND test_${name})
  endforeach()
endif()
