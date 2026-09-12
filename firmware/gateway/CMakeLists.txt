cmake_minimum_required(VERSION 3.16)
project(mcq_gateway C)

# Portable core of the safety gateway firmware: state machine, heartbeat,
# limits, CRC and the JETSON_COMMAND codec. The board support layer (CAN
# peripheral, CRSF receiver, encoder and actuator drivers) lives next to it
# once the board is chosen; this library must stay free of hardware headers so
# the host tests keep running.

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(GW_BUILD_TESTS "Build host-side unit tests" ON)

add_library(gateway_core STATIC
  src/crc8.c
  src/command.c
  src/heartbeat.c
  src/limits.c
  src/state_machine.c
)
target_include_directories(gateway_core PUBLIC include)
target_compile_options(gateway_core PRIVATE
  -Wall -Wextra -Wpedantic -Wshadow -Wdouble-promotion -Wconversion -Werror
)
target_link_libraries(gateway_core PUBLIC m)

if(GW_BUILD_TESTS)
  enable_testing()
  foreach(name crc command heartbeat limits state_machine)
    add_executable(test_${name} test/test_${name}.c)
    target_link_libraries(test_${name} PRIVATE gateway_core)
    target_include_directories(test_${name} PRIVATE test)
    target_compile_options(test_${name} PRIVATE -Wall -Wextra -Werror)
    add_test(NAME ${name} COMMAND test_${name})
  endforeach()
endif()
