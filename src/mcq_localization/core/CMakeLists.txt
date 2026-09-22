cmake_minimum_required(VERSION 3.16)
project(mcq_localization_core C)

# Dependency-free C library holding the state estimator: an error-state EKF over
# position, velocity, heading and the IMU biases, with delayed-measurement
# replay so a GNSS fix is applied at the time it was measured. The ROS 2 node in
# the parent package is a thin wrapper around this; the Python simulator loads
# it through ctypes and scores it against the truth the simulator keeps, which
# is how the 0.10 m Phase 1 number is measured off the kart. Builds standalone
# with no ROS installed.
#
# Doubles here, unlike mcq_control/core: this runs on the Jetson, and a
# covariance in single precision loses symmetry and positive-definiteness.

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(MCQ_CORE_BUILD_TESTS "Build host-side unit tests" ON)

add_library(mcq_localization_core SHARED
  src/linalg.c
  src/ekf.c
)
target_include_directories(mcq_localization_core PUBLIC include)
target_compile_options(mcq_localization_core PRIVATE
  -Wall -Wextra -Wpedantic -Wshadow -Wdouble-promotion -Wconversion -Werror -O2 -fPIC
)
target_link_libraries(mcq_localization_core PUBLIC m)

if(MCQ_CORE_BUILD_TESTS)
  enable_testing()
  foreach(name linalg ekf)
    add_executable(test_${name} test/test_${name}.c)
    target_link_libraries(test_${name} PRIVATE mcq_localization_core)
    target_include_directories(test_${name} PRIVATE test)
    target_compile_options(test_${name} PRIVATE -Wall -Wextra -Werror)
    add_test(NAME ${name} COMMAND test_${name})
  endforeach()
endif()
