# McQueen VIP

Autonomous racing kart for the Purdue Grand Prix track at the Northwest Sports Complex in West Lafayette, Indiana.

The first target is a fully autonomous lap in under 60 seconds with no human intervention. The second target is a lap time competitive with the Autonomous Karting Series (AKS) field, which races on the same track every May. The stack must run from a track description alone (centerline plus widths, or perceived boundaries) so it can be tested on any paved loop before it ever sees the Purdue track.

Status: Phase 0 in progress. The design draft is in `docs/`; the code that exists runs without the kart: the gateway firmware core with its fault-injection tests, the C control core, the CAN definition with generated codecs and a SocketCAN bridge, the ROS 2 message package, a simulator that closes the loop on a synthetic oval both as a Python harness and as the ROS 2 graph (simulator, planner, controller nodes), and the survey tool. Read the docs in order, then `## Build and test` below.

## Quick facts

| Item | Value |
| --- | --- |
| Track | 0.2698 mi (434 m), 5 turns, asphalt |
| Human lap record | 24.616 s (2026, Yamaha KT100 kart) |
| Sub-60 s lap | 7.2 m/s average speed |
| Compute | NVIDIA Jetson AGX Orin Developer Kit, 64 GB |
| OS and middleware | JetPack 7.2 (Ubuntu 24.04, CUDA 13), ROS 2 Jazzy |
| Languages | C++ for real-time nodes, Python for tools and training, CUDA/TensorRT for perception |
| Localization | RTK GNSS (u-blox ZED-F9P class, 20 Hz) fused with IMU, wheel speeds and steering angle |
| Perception | One forward camera required; Livox Mid-360 LiDAR optional |
| Safety | Microcontroller gateway between the Jetson and every actuator; RC override; remote and physical e-stop |

## Documents

| File | What it answers |
| --- | --- |
| [docs/01-proposal.md](docs/01-proposal.md) | What we should build, the decisions behind it, and what we will not build |
| [docs/02-architecture.md](docs/02-architecture.md) | The program: processes, rates, frames, messages, planning, control, localization, perception |
| [docs/03-hardware.md](docs/03-hardware.md) | Kart, actuators, sensors, compute, network, power |
| [docs/04-safety.md](docs/04-safety.md) | Safety gateway, e-stop chain, mode state machine, AKS rule mapping |
| [docs/05-roadmap.md](docs/05-roadmap.md) | Phases, milestones, and the test that closes each one |
| [docs/06-dev-setup.md](docs/06-dev-setup.md) | Jetson flashing, ROS 2, Docker, logging, conventions |
| [docs/07-references.md](docs/07-references.md) | Sources checked while writing this draft |
| [docs/08-training-data.md](docs/08-training-data.md) | Where onboard kart footage exists, how much, under what terms, and the ingest pipeline in `training/data/` |
| [docs/09-work-queue.md](docs/09-work-queue.md) | Where the tree stands and the code that needs writing next, in lanes one person can own |

## The stack in one paragraph

A safety gateway microcontroller owns the actuators and only passes commands from the Jetson while a 100 Hz heartbeat is alive, the RC transmitter allows autonomous mode, and no e-stop is asserted. On the Jetson, a localization node fuses RTK GNSS, IMU, wheel speeds and steering angle into a 100 Hz pose in a local track frame. A track model (centerline and half-widths, surveyed with the kart itself or derived live from perceived boundaries) feeds an offline raceline optimizer and an online local planner that keeps the kart inside the boundaries and picks a speed profile. A 100 Hz controller turns the planned trajectory into steering angle, throttle and brake commands using controllers ported from openpilot. Everything is logged to MCAP and replayable in a bicycle-model simulator.

## Repository layout

Directories marked `*` exist and have tests; the rest are planned.

```
mcqueen-vip/
  docs/                    design documents
  src/                     ROS 2 workspace packages
    mcq_msgs/            * message and service definitions
    mcq_bringup/         * launch files and parameter sets per kart and per environment
    mcq_vehicle/         * SocketCAN bridge to the gateway; dbc/mcqueen.dbc is the frame definition
    mcq_localization/      GNSS + IMU + wheel + steering fusion, track frame management
    mcq_track/             track model, Frenet utilities, raceline loading, geofence
    mcq_planning/          local planner, boundary-only planner, speed profile
    mcq_control/         * lateral and longitudinal controllers; core/ is a pure C library
    mcq_perception/        camera and LiDAR boundary extraction, kart detection (TensorRT)
    mcq_sim/             * kart model, Frenet planner prototype, closed-loop harness (Python)
    mcq_telemetry/         pit-side dashboard bridge, RCS black box interface
  firmware/
    gateway/             * safety gateway core: state machine, heartbeat, limits, codec, host tests, SocketCAN host build
  tools/
    raceline/              wrapper around the TUM global race trajectory optimizer
    survey/              * turn recorded RTK edge drives into a track model file
    logs/                  MCAP lap report; message definitions for reading logs without ROS (task E1)
  training/                PyTorch: perception models, learned dynamics, policy experiments (runs off-kart)
    data/                * footage source catalog, YouTube fetch and frame extraction tooling
  docker/                * x86 development container (Jetson image to follow)
  tracks/                * track model files (centerline + widths) and generated racelines
```

## Build and test

Everything below runs on a laptop with cmake, a C compiler and Python 3.10 or newer; ROS 2 is only needed for the message package.

```
pip install numpy scipy pyyaml pytest cantools ruff

# Safety gateway core: fault-injection list on the host
cmake -S firmware/gateway -B firmware/gateway/build && cmake --build firmware/gateway/build
ctest --test-dir firmware/gateway/build --output-on-failure

# Control core (PID, bicycle model, pure pursuit, limits, longitudinal, speed profile)
cmake -S src/mcq_control/core -B src/mcq_control/core/build -DCMAKE_BUILD_TYPE=Release
cmake --build src/mcq_control/core/build
ctest --test-dir src/mcq_control/core/build --output-on-failure

# Simulator, DBC and training tooling tests
python -m pytest -q

# Phase 0 loop: two laps of the synthetic oval from the track file alone
PYTHONPATH=src/mcq_sim python -m mcq_sim run --track tracks/synthetic_oval --laps 2
PYTHONPATH=src/mcq_sim python -m mcq_sim run --track tracks/synthetic_oval --laps 2 --mode BOUNDARY

# Gateway core on a virtual CAN bus, driven from Python (the bench rig without hardware)
sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
./firmware/gateway/build/gateway_host --interface vcan0 &
python -m pytest -q firmware/gateway/host

# ROS 2 graph (inside docker/x86 or any Jazzy install): simulator, planner, controller
rosdep install --from-paths src --ignore-src -y
colcon build && source install/setup.bash
ros2 launch mcq_bringup sim.launch.py                       # add record:=true for an MCAP log
ros2 run mcq_bringup check_graph.py --distance 120          # what CI runs: PASS once 120 m are driven
```

What runs on the kart is C: the gateway core on the microcontroller and the control core inside the Jetson's controller node. Python is for the simulator, the planner prototype, tools and training, and stays off the 100 Hz path.
