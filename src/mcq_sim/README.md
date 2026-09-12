# mcq_sim

Kart model, Frenet planner prototype and the closed-loop harness. Runs on a laptop with numpy and scipy; nothing here runs on the kart. Controllers come from the C library in `src/mcq_control/core` through ctypes (built on first use if cmake is present).

```
export PYTHONPATH=src/mcq_sim
python -m mcq_sim run --track tracks/synthetic_oval --laps 3            # FOLLOW mode, 5 m/s cap
python -m mcq_sim run --track oval --mode BOUNDARY --laps 2             # reference from perceived edges
python -m mcq_sim run --track oval --speed-cap 8 --out run.csv          # per-tick log
python -m mcq_sim run --track oval --shift 0 1.2                        # fault injection 15
python -m mcq_sim run --track oval --set planner.a_lat_max=4 --set lateral.k_v=0.6
python -m mcq_sim make-track --out tracks/my_oval --straight 40 --radius 10
python -m pytest -q src/mcq_sim/test
```

| Module | What it is |
| --- | --- |
| `track.py` | Track model: TUM csv in and out, spline centerline, Frenet conversion, widths, geofence distance, perceived-edge stand-in, local track from edges |
| `vehicle.py` | Kinematic bicycle with understeer, steering actuator lag and rate limit, throttle and brake maps, drag |
| `planner.py` | Frenet sampling planner: quintic offset candidates, boundary and obstacle checks, reachable speed profile, time and deviation cost, stop ramps, FOLLOW and BOUNDARY modes |
| `ccontrol.py` | ctypes bindings to the C control core |
| `harness.py` | The loop: planner at 20 Hz, controller at 100 Hz, geofence, trajectory age and perception-disagreement checks, lap timing, csv log |
| `config/sim_default.yaml` | Every tunable |

The exit condition of Phase 0 for the software (docs/05-roadmap.md) is `test/test_harness.py`: two laps of the synthetic oval in each mode with zero boundary violations, plus the two map-fault cases. The ROS 2 node that publishes `VehicleState`, GNSS and IMU messages from this model and consumes `VehicleCommand` comes next; `package.xml` already declares the package so `colcon build` picks it up.
