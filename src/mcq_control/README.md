# mcq_control

Lateral and longitudinal controllers for the 100 Hz loop (docs/02-architecture.md section 8).

`core/` is a dependency-free C library with the math: PID with speed-scheduled gains and anti-windup, the bicycle model with an understeer term, pure pursuit, curvature and rate limits, the off/stopping/pid longitudinal state machine with the actuator map, and the speed profile used by the planner. It builds and tests standalone:

```
cmake -S src/mcq_control/core -B src/mcq_control/core/build -DCMAKE_BUILD_TYPE=Release
cmake --build src/mcq_control/core/build
ctest --test-dir src/mcq_control/core/build --output-on-failure
```

The simulator in `mcq_sim` loads the same shared library through ctypes, so the loop that runs on the oval off the kart is the code that will run on the kart. The ROS 2 node (a composable C++ node that subscribes to `Trajectory`, `EgoState` and `VehicleState`, calls into `core`, and publishes `VehicleCommand`) is the next piece and lives in this package beside `core/`.
