# Work queue

2026-09-22: the state of the tree and the code that needs writing next, split into lanes that one person can own without blocking another. Task ids are stable; when a task is done, mark it here in the same pull request as the code.

## 1. The ten assignments

One owner each, in the order they should be handed out. Task ids in the third column point at the detail in section 4. Sizes are days of focused work by one student who already has the container running; halve nothing for part-time reality, add to it.

| # | Assignment | Tasks | Depends on | Size | Suits |
| --- | --- | --- | --- | --- | --- |
| 1 | Sensor models and ground truth in the simulator | A1 | nothing | 3 to 4 d | Python, numpy, anyone comfortable with noise models |
| 2 | EKF state estimator, plus the URDF and sensor frames it needs | A2, G1 | 1 | 2 to 3 wk | strongest available; C and estimation theory |
| 3 | Track server in C++ with the geofence | B1 | nothing | 1 to 1.5 wk | C++, geometry, KD-trees |
| 4 | Port the local planner to C and wrap it at 20 Hz | C1 | 3 | 1.5 to 2 wk | C and C++, reads Python well |
| 5 | Raceline tooling around the TUM optimizer | D1 | nothing | 3 to 5 d | Python, optimization, self-contained |
| 6 | Lap report tool and the CI gaps it exposed | E1, G4 | nothing | 3 to 4 d | Python and GitHub Actions; good first task |
| 7 | Replay harness against recorded MCAP | E2 | nothing | 1 wk | Python, testing instincts |
| 8 | Gateway board support and the board decision | F1 | board choice | 1.5 to 2 wk | embedded C, electrical sub-team |
| 9 | CRSF receiver parsing with host tests | F2 | nothing | 3 to 5 d | embedded C; testable with no hardware |
| 10 | Telemetry: foxglove bridge, layouts, lap timer | H1 | nothing | 4 to 5 d | Python and ROS 2, wants a visual eye |

Seven of the ten start today. Number 2 waits on number 1, number 4 waits on number 3, and number 8 waits on the board choice between an STM32H7 carrier, a Teensy 4.1 and a comma panda, which is the longest-lead decision on this page.

Held back on purpose for the next wave: the composable-node container (G2, needs 3 and 4 done), the Jetson image and the bench and kart launch files (G3), and the actuator output layer (F3, needs the actuator choices). Perception, MPC, LiDAR and the learned components are Phase 4 and later and nobody should start them while 1 through 4 are open.

## 2. Where we left off

Last commit is `39081d6` (2026-09-13), CI green on `main`, no open pull requests and no open issues. Everything since 2026-09-12 went into making the ROS 2 graph test deterministic in CI, and the outcome is worth stating because it constrains the work below: a participant joining the DDS graph mid-run freezes the rclpy nodes for 200 to 350 ms, the controller's 200 ms freshness check correctly requests an urgent stop, and the only reason CI passes today is process discipline (observers join before the nodes, the emulated gateway holds `RC` until the graph settles). That workaround disappears when the planner stops being Python (task C1).

What runs today: the gateway core with the software half of the fault-injection list, the C control core, the DBC with generated codecs on both sides, the SocketCAN bridge and its host build over `vcan0`, the Python simulator and planner prototype, the ROS 2 graph driving 120 m of the synthetic oval in CI with the session recorded to MCAP, and the survey tool.

One thing to keep in mind while assigning: `sim_node` publishes `EgoState` straight from the simulator's own state with `gnss_status = FIXED`, so every controller and planner number we have so far assumes perfect localization. Lane A is what turns those numbers into real ones.

Four things the documents claim exist and do not:

| Claim | Where | Reality |
| --- | --- | --- |
| `tools/logs/lap_report.py` | README layout table, docs/06 section 4, `pyproject.toml` testpaths | Not in the tree. Commit `b80b1cb` added the dependencies and the testpath but not the tool. pytest ignores a testpath that does not exist, so CI never noticed. |
| `docker/jetson/Dockerfile` | docs/06 sections 1 and 2 | Only `docker/x86` exists |
| `base_link` and sensor frames from a bringup URDF | docs/02 section 3 | No URDF anywhere |
| `clang-tidy` in CI | docs/06 section 3 | Only `clang-format` runs |

Also unimplemented: all three services (`mcq_msgs/srv/LoadTrack`, `SetAutoSubmode`, `SetSpeedCap`) are defined and nothing offers them.

## 3. Phase 0 exit test, what is left

The roadmap's Phase 0 exit test has three parts. The graph against the simulator passes. Recording to MCAP passes; replay does not exist (task E2, and it does not need real logs, the CI bag is a fine first reference). The gateway fault-injection list passes in software on the host; the hardware-in-the-loop half needs the board (lane F).

## 4. Lanes

Eight lanes. A, B, C and F are the critical path to Phase 1 and Phase 2; D, E, G and H are each a week or less and unblock other people.

### Lane A: localization (start now, two people or one strong one)

**A1. Sensor models and ground truth in the simulator.** `sim_node` publishes the fused answer today. Make it publish what the kart's sensors publish instead: `sensor_msgs/NavSatFix` (plus velocity) at 20 Hz with position noise, latency and an RTK status that can be driven `FIXED` to `FLOAT` to `NONE` on command, `sensor_msgs/Imu` at 200 Hz with bias and random walk, and the wheel speeds and steering angle that `VehicleState` already carries. Keep the true state on `/ego_truth` for scoring. Files: `src/mcq_sim/mcq_sim/sim_node.py`, `vehicle.py` (`measured_pose` is the hook), `config/sim_default.yaml`, tests under `src/mcq_sim/test`. Done when the graph still drives the oval with a passthrough node standing in for the estimator, and a scripted 10 s GNSS outage is reproducible from a parameter.

**A2. `mcq_localization/state_estimator`.** Error-state EKF, dependency-free C core with its own cmake host tests in the shape of `src/mcq_control/core`, thin C++ node around it. IMU-driven predict at 200 Hz, GNSS position and velocity update at 20 Hz gated by status, wheel-speed longitudinal update, steering-angle yaw-rate consistency check through the bicycle model. Publishes `EgoState` and the `map` to `base_link` transform, grows covariance on `FLOAT`, dead-reckons in `odom` on `NONE`. Done when, against A1, position RMS against `/ego_truth` is under 0.10 m over five laps with RTK fixed (the Phase 1 exit number), yaw converges from GNSS course above 1 m/s, the 10 s outage ends in a covariance gate rather than a silent divergence, and a tick measures under 1 ms on the workstation.

Decision A2 needs from lane B: `EgoState` carries `s` and `d`, and docs/02 gives Frenet lookup to `track_server`. Either the estimator links the same Frenet library or it leaves the fields to the track server. Settle it in writing before either lane is half done.

### Lane B: track server (start now, one person)

**B1. `mcq_track/track_server` in C++.** Load `track.csv` and `track.yaml`, cumulative arc-length table and KD-tree over the centerline, constant-time `(x,y)` to `(s,d)` and back, publish `TrackModel` at 10 Hz and on change, build the geofence polygon between the edges with a configurable margin, publish `GeofenceState` from `EgoState` position and covariance, and offer `mcq/load_track`. `src/mcq_sim/mcq_sim/track.py` is the tested reference to port from; keep it and cross-check numerically. Done when the C++ output matches the Python `Track` to within 1e-6 on the oval, fault-injection item 15 (shift the track file 2 m, drive `FOLLOW`, geofence must fire) passes in the graph, and `planner_node` consumes `TrackModel` instead of reading the track file itself.

### Lane C: planner port (after B1, one person)

**C1. Port `FrenetPlanner` to C and wrap it in a 20 Hz C++ node.** Same structure as the control core: `src/mcq_planning/core` with host tests, node reads `EgoState`, `TrackModel` and `Detections`, writes `Trajectory`, and offers `mcq/set_auto_submode` and `mcq/set_speed_cap` (the cap can never exceed the gateway's). Keep `src/mcq_sim/mcq_sim/planner.py` as the reference and add a parity test over a sweep of ego states: same chosen offset and speed within tolerance. Done when nothing Python remains in the driving graph, the CI graph test passes without the observers-first ordering, lap time on the oval is within 2 percent of the prototype, and the paragraph in docs/02 about pit tools having to join before `AUTO` can be deleted.

### Lane D: raceline tooling (start now, one person, self-contained)

**D1. `tools/raceline`.** Wrapper around the TUM `global_racetrajectory_optimization` solve: our track file plus a vehicle parameter file and a ggv diagram in, `tracks/<id>/raceline_<date>.csv` out in the format `Raceline.from_csv` already reads. Minimum curvature first, minimum time behind a flag. Pin or vendor the upstream package so CI does not fetch it at test time. Start the ggv at 3 m/s^2 lateral and 2 m/s^2 longitudinal. Done when a raceline for `synthetic_oval` is committed, the simulator in `FOLLOW` on that raceline is faster than on the centerline and never leaves the boundaries, and the tool has a test that runs offline.

### Lane E: logs and replay (start now, one person)

**E1. Write `tools/logs/lap_report.py`.** It is referenced in three places and does not exist. Summarize a session from an MCAP file without ROS installed (laps, lap times, lateral deviation, speed, stop requests, gateway mode transitions), with the message definitions built from the `.msg` files the way `tools/survey` reads logs. Add a CI step that fails when a `pyproject.toml` testpath is missing, since that is why nobody noticed.

**E2. Replay harness.** Feed the sensor topics of a recorded MCAP to one node and compare its outputs against the recorded ones with per-field tolerances. First reference set is the bag CI already records from the sim graph, so this does not wait for real logs. Recommendation: regenerate the reference bag in CI rather than storing it in Git LFS, and keep LFS for the first real track logs. Done when a deliberate gain change in the controller fails the replay and the suite runs in CI. This closes the last part of the Phase 0 exit test that does not need hardware.

### Lane F: gateway board (needs the board decision first, electrical plus one firmware person)

**F1. Board support.** Pick the target (STM32H7 carrier, Teensy 4.1, or a comma panda running our safety mode) and add `firmware/gateway/board/<target>` with the HAL the core needs: CAN transmit and receive, a 100 Hz timer, GPIO for the contactor output and the e-stop and auxiliary-contact inputs, encoder and hall inputs. The core in `firmware/gateway/src` stays dependency-free and untouched. Cross-compile in CI with `arm-none-eabi` in the container. Done when the firmware builds for the target in CI and `firmware/gateway/host/test_gateway_host.py` passes over a USB CAN adapter against the board instead of against the host binary.

**F2. CRSF receiver parsing.** ELRS frames into the state machine as a core module with host tests, including the failsafe frame and link loss, driving the mode switch and RC passthrough. Done when fault-injection items 8, 11 and 12 run on the bench.

**F3. Actuator output layer.** Steering position or torque command, throttle over CAN or an isolated DAC, brake PWM or CAN, contactor with readback, with the existing `limits.c` applied at the output stage. Needs F1 and the actuator choices.

### Lane G: bringup and infrastructure (four small tasks, good for spreading around)

**G1. URDF and static transforms** in `mcq_bringup`: `base_link` at the rear axle on the ground plane, plus `gnss_antenna`, `imu_link`, `camera_front`, `lidar_link` and `steering_column`, with the measured offsets in a kart YAML rather than in the URDF. Lane A needs this. Half a day.

**G2. Composable-node container.** docs/02 section 4 says estimator, track server, planner and controller live in one process with intra-process communication and that nothing in the 100 Hz path serializes through DDS; they are four separate nodes today. Convert them to components, add the container launch, and record the measured per-tick latency in docs/02. After B1 and C1.

**G3. Jetson image and the other launch files.** `docker/jetson/Dockerfile` (referenced, missing) and the arm64 build, plus `bench.launch.py` and `kart.launch.py` skeletons so the bench rig has somewhere to land.

**G4. CI hardening.** Add `clang-tidy` (docs claim it runs), run `colcon test` on every package instead of only `mcq_control`, lint `training/data`, fail on missing pytest testpaths, and add a nightly repeat of the graph test. The DDS timing bug cost most of a day of push-and-see; a nightly run finds the next one without blocking anybody's merge.

### Lane H: telemetry (start now, one person)

**H1. `mcq_telemetry`.** `foxglove_bridge` launch, a 10 Hz aggregation node, a lap timer, and three layouts checked into `src/mcq_telemetry/layouts` (pit, localization debug, control debug). Done when the sim graph can be watched live from a second machine with the layouts in the repo. This has to exist before the first outdoor test, because nobody debugs a kart from a terminal.

## 5. Order and parallelism

Start today with no dependencies: A1, B1, D1, E1, G1, G3, G4, H1. C1 waits on B1. A2 waits on A1. G2 waits on B1 and C1. Lane F waits on the board choice, which is the longest-lead decision on this page.

Critical path to the Phase 1 exit test (a logged RC lap, fused pose within 0.10 m RMS, a track model produced from that log, the fault list at standstill and 3 m/s): A1 then A2 for the pose, F1 and F2 for the on-kart fault list, and the survey tool, which is already done.

Not now, on purpose: perception, the learned components, MPC and LiDAR are Phase 4 and later, and the roadmap cuts them first if time runs out. Nobody should be starting them while lanes A through C are open.

## 6. Process

Sixteen commits have gone straight to `main`, which was fine with one author. docs/06 section 5 requires a branch per change and review by someone outside the author's sub-team, so before the next people start committing: branch protection on `main`, a `CODEOWNERS` file mapping the lanes above to reviewers, and `pre-commit install` in everyone's checkout (the config is already in the repo). One person, one afternoon.
