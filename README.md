# McQueen VIP

Autonomous racing kart for the Purdue Grand Prix track at the Northwest Sports Complex in West Lafayette, Indiana.

The first target is a fully autonomous lap in under 60 seconds with no human intervention. The second target is a lap time competitive with the Autonomous Karting Series (AKS) field, which races on the same track every May. The stack must run from a track description alone (centerline plus widths, or perceived boundaries) so it can be tested on any paved loop before it ever sees the Purdue track.

Status: proposal. This branch carries the design draft and no code yet. Read the docs in order.

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

## The stack in one paragraph

A safety gateway microcontroller owns the actuators and only passes commands from the Jetson while a 100 Hz heartbeat is alive, the RC transmitter allows autonomous mode, and no e-stop is asserted. On the Jetson, a localization node fuses RTK GNSS, IMU, wheel speeds and steering angle into a 100 Hz pose in a local track frame. A track model (centerline and half-widths, surveyed with the kart itself or derived live from perceived boundaries) feeds an offline raceline optimizer and an online local planner that keeps the kart inside the boundaries and picks a speed profile. A 100 Hz controller turns the planned trajectory into steering angle, throttle and brake commands using controllers ported from openpilot. Everything is logged to MCAP and replayable in a bicycle-model simulator.

## Proposed repository layout

```
mcqueen-vip/
  docs/                    design documents (this draft)
  src/                     ROS 2 workspace packages
    mcq_msgs/              message and service definitions
    mcq_bringup/           launch files and parameter sets per kart and per track
    mcq_vehicle/           CAN interface to the gateway: vehicle state in, actuator commands out
    mcq_localization/      GNSS + IMU + wheel + steering fusion, track frame management
    mcq_track/             track model, Frenet utilities, raceline loading, geofence
    mcq_planning/          local planner, boundary-only planner, speed profile
    mcq_control/           lateral and longitudinal controllers, actuator limits
    mcq_perception/        camera and LiDAR boundary extraction, kart detection (TensorRT)
    mcq_sim/               bicycle-model simulator and replay harness
    mcq_telemetry/         pit-side dashboard bridge, RCS black box interface
  firmware/
    gateway/               safety gateway MCU firmware (watchdog, e-stop, RC passthrough, limits)
  tools/
    raceline/              wrapper around the TUM global race trajectory optimizer
    survey/                turn recorded RTK laps into a track model file
    logs/                  MCAP inspection and export scripts
  training/                PyTorch: perception models, learned dynamics, policy experiments (runs off-kart)
    data/                  footage source catalog, YouTube fetch and frame extraction tooling
  docker/                  x86 and Jetson development containers
  tracks/                  track model files (centerline + widths) and generated racelines
```
