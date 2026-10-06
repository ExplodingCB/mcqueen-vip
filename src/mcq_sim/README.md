# mcq_sim

Kart model, Frenet planner prototype and the closed-loop harness. Runs on a laptop with numpy and scipy; nothing here runs on the kart. Controllers come from the C library in `src/mcq_control/core` through ctypes (built on first use if cmake is present).

The new Purdue environment adds a visual viewer, dynamic tire/actuator model,
front-camera segmentation interface and measured-log replay. Start with
`python -m mcq_sim view --port 0`. See [the simulator guide](../../docs/10-simulator.md)
and [track provenance](../../tracks/purdue_gp/README.md). This environment is
uncalibrated; passing a simulated lap is not proof of real-kart accuracy.

```
export PYTHONPATH=src/mcq_sim
python -m mcq_sim run --track tracks/synthetic_oval --laps 3            # FOLLOW mode, 5 m/s cap
python -m mcq_sim run --track oval --mode BOUNDARY --laps 2             # reference from perceived edges
python -m mcq_sim run --track oval --speed-cap 8 --out run.csv          # per-tick log
python -m mcq_sim run --track oval --shift 0 1.2                        # fault injection 15
python -m mcq_sim run --track oval --set planner.a_lat_max=4 --set lateral.k_v=0.6
python -m mcq_sim run --track oval --raceline tracks/oval/raceline.csv --config my.yaml --seed 3
python -m mcq_sim make-track --out tracks/my_oval --straight 40 --radius 10
python -m pytest -q src/mcq_sim/test
```

| Module | What it is |
| --- | --- |
| `sensors.py` | What the sensors report rather than what is true: GNSS rate, latency, noise by fix status and a scheduled status, IMU noise and bias walk, wheel and steering quantization |
| `geodesy.py` | Map frame to WGS84 and back, shared with `tools/survey` |
| `track.py` | Track model: TUM csv in and out, spline centerline, Frenet conversion, widths, geofence distance, perceived-edge stand-in, local track from edges |
| `vehicle.py` | Kinematic bicycle with understeer, steering actuator lag and rate limit, throttle and brake maps, drag |
| `planner.py` | Frenet sampling planner: quintic offset candidates, boundary and obstacle checks, reachable speed profile, time and deviation cost, stop ramps, FOLLOW and BOUNDARY modes |
| `ccontrol.py` | ctypes bindings to the C control core |
| `harness.py` | The loop: planner at 20 Hz, controller at 100 Hz, geofence, trajectory age and perception-disagreement checks, lap timing, csv log |
| `sim_node.py` | The ROS 2 node: publishes `VehicleState`, `GatewayStatus`, `/gnss/fix`, `/gnss/fix_velocity`, `/imu/data_raw` and `/ego_truth`, emulates the gateway's mode machine, consumes `VehicleCommand` |
| `planner_node.py` | The ROS 2 node around `planner.py`: `Trajectory` at 20 Hz and `GeofenceState` |
| `ros_utils.py` | Message conversions shared by the two nodes |
| `params.py` | Loads `config/sim_default.yaml` and applies `--set` overrides |
| `config/sim_default.yaml` | Every tunable |

The exit condition of Phase 0 for the software (docs/05-roadmap.md) is `test/test_harness.py`: two laps of the synthetic oval in each mode with zero boundary violations, plus the two map-fault cases.

`sim_node` publishes the sensors, so the ROS 2 graph is what a state estimator will be written against: fixes arrive 80 ms late stamped when they were measured, the fix status follows `sensors.gnss.schedule`, and the IMU bias walks. The truth is on `/ego_truth` and nothing on the kart will ever publish it. While `publish_ego_state` is true the node also republishes that truth as `/ego_state`, which is what keeps the graph driving until `mcq_localization` exists; `ros2 launch mcq_bringup sim.launch.py ego_from_sim:=false` turns it off. Sensor scheduling runs on the ROS clock while the kart model integrates a fixed step, so under a loaded machine the two drift apart by however late the timer is.

The pure-Python harness still feeds its controller `KartSim.measured_pose()`, the true pose with 2 cm of white noise on position and no latency. That is the last place in the repository that assumes localization nobody has to build, and it goes away with the state estimator.
