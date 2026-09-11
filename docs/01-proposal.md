# Proposal: what we should build

This document argues for a specific plan. The architecture document describes the program that results from it.

## 1. The problem in numbers

The Purdue Grand Prix track is a 0.2698 mi (434 m) asphalt loop with five turns. The human lap record on it is 24.616 s, set in 2026 by a Yamaha KT100 kart, which is an average of 17.6 m/s. A lap under 60 s needs an average of only 7.2 m/s (16 mph). The kart platforms used in the Autonomous Karting Series are electric sprint karts capped at 15 kW, which reach that speed in a few seconds.

So the sub-60 s goal is not a speed problem. It is a reliability problem: keep the kart inside a 15 ft wide strip of asphalt for five laps with nobody touching it, with localization that does not drift, actuators that do what they were told, and a safety chain that stops the kart when anything disagrees. Once that exists, lap time comes from raising the lateral and longitudinal acceleration limits in one parameter file and from a better raceline, not from new architecture. The roadmap treats 60 s as the milestone that proves the stack, and the 40 s range as the target for the competition season.

The autonomous event on this track is the Autonomous Karting Series national race (held at Purdue May 18 to 22, 2026). Its rulebook, last updated December 2025, is the operative constraint set for anything we build, whether or not we enter in the first year. The rules that shape the software are listed in section 4.

## 2. Decisions

Each decision below has a short rationale. They are meant to be argued with, but they are positions, not options.

### 2.1 JetPack 7.2, ROS 2 Jazzy, C++ for the hot path

JetPack 7.2 (June 2026) moved the whole Orin family to Ubuntu 24.04, kernel 6.8 and CUDA 13. That fixes the ROS 2 choice: Jazzy Jalisco is the Tier 1 distribution for Ubuntu 24.04 on arm64, is supported until May 2029, and is the distribution NVIDIA's Isaac ROS 4.6 (August 2026) targets on Orin with JetPack 7.2. Lyrical Luth (May 2026) is newer and also LTS, but drivers and Isaac ROS have not moved to it; we should not be the ones porting them. Humble on JetPack 6.2.2 (Ubuntu 22.04) is the fallback if JetPack 7.2 misbehaves on our kit.

ROS 2 earns its place for sensor drivers, logging, replay, visualization and the process boundary between subsystems owned by different students. It does not earn a place inside the control loop. The localization, planning and control nodes should live in one C++ process as composable nodes with intra-process communication so the 100 Hz path never crosses DDS. Python stays for tools, offline optimization, training and the simulator, and for first drafts of controllers that are later ported.

### 2.2 A safety gateway that the Jetson cannot bypass

Every actuator (steering, throttle, brake, drivetrain contactor) is wired to a microcontroller, not to the Jetson. The Jetson sends commands over CAN and a heartbeat; the gateway applies angle, rate and torque limits in firmware, arbitrates between RC and autonomous mode, executes urgent stops on its own, and drops to a safe state if the heartbeat, the RC link or the remote e-stop disappears. This is the architecture openpilot uses with its panda board, whose safety code lives in opendbc and is checked to MISRA C rules with 100 % line coverage, and it is what the AKS safety rules require in practice (software-commandable urgent stop, physical-controller remote e-stop, full RC control from the pit).

This is the one part of the system that must be finished and tested before any autonomous driving happens. It is also the part that lets a student team iterate quickly on the Jetson side, because a planner bug becomes an urgent stop instead of a crash.

### 2.3 Map first, boundaries only as the fallback

The stack runs from one track model file: a centerline with left and right half-widths, in the format the TUM global race trajectory optimizer already reads. That file can come from three places, in order of preference: an RTK survey driven with the kart during practice sessions (allowed by the AKS rules, and the only allowed source at competition), the high-resolution scan the team is obtaining, or perceived boundaries accumulated live by the perception node when no survey exists.

The "free-flowing" behavior the team asked for lives in the local planner, which never assumes the raceline is drivable. It samples lateral offsets in a Frenet frame around the reference and keeps only candidates that stay inside the currently believed boundaries, with a speed chosen from the curvature of the candidate. With a surveyed map the reference is the optimized raceline; with boundaries only the reference is the midline of what the camera or LiDAR sees a few tens of meters ahead, and the speed cap is lower. Same planner, same controller, different reference.

### 2.4 Classical stack first, learning behind the same envelope

The baseline is entirely classical and deterministic: EKF localization, offline minimum-curvature then minimum-time raceline, Frenet sampling planner, bicycle-model lateral control, PID longitudinal control. This is the architecture the ForzaETH F1TENTH stack uses and has won with, and it is the one a student team can debug at the track with a laptop.

Learning enters in three places once the baseline laps: a perception model for boundary and kart detection (small segmentation or keypoint network exported to TensorRT), a learned correction to the tire and drivetrain model used by the speed profile and controller, and later a policy trained in simulation that consumes the same boundary observations and emits the same trajectory message as the classical planner. Because every planner output passes through the same limits and the same gateway, a learned planner can be tried on track without touching the safety chain. PyTorch and CUDA are for these components and for offline tooling, not for the control loop.

### 2.5 What we take from openpilot and sunnypilot, and what we do not

openpilot (MIT, Python 3.12) is an operating system for driver assistance on production cars. Its driving model, its car ports and its UI are useless for a kart. Its controls library is not. The concrete pieces to port, all small and dependency-free:

| Source (openpilot master, September 2026) | What it is | Where it goes |
| --- | --- | --- |
| `openpilot/common/pid.py` and `opendbc/car/common/pid.py` | PID with speed-scheduled gains and anti-windup that freezes the integrator when the output clips | `mcq_control`, both loops |
| `opendbc/car/vehicle_model.py` | Dynamic bicycle model; curvature from steering angle and back, with roll compensation | `mcq_control` and `mcq_planning` |
| `openpilot/selfdrive/controls/lib/longcontrol.py` | Longitudinal state machine (off, stopping, pid) and PID on acceleration error with feedforward | `mcq_control` longitudinal |
| `openpilot/selfdrive/controls/lib/latcontrol_angle.py` and `latcontrol_torque.py` | Desired curvature to steering angle (our steer-by-wire case) or to torque with lateral-acceleration feedforward and friction compensation (if the steering motor is torque-controlled) | `mcq_control` lateral |
| `opendbc/car/lateral.py` | Lateral acceleration and jerk limiting helpers, angle and torque rate limiters | `mcq_control` limits and `firmware/gateway` (reimplemented in C) |
| `opendbc/car/interfaces.py` and `opendbc/car/body/` | The CarInterface, CarState, CarController split and a minimal non-car port (the comma body) to copy | `mcq_vehicle` |
| `opendbc/safety/` | The pattern for firmware-side safety hooks (rx checks, tx allowlist, controls_allowed) | `firmware/gateway` |

sunnypilot is an openpilot fork whose additions (model picker, additional driving models, MADS, blind-spot lane changes) are all about production cars and comma hardware. Nothing in it applies to a kart that openpilot does not already provide. Track it for controls changes; do not depend on it.

The larger reuse decisions: the TUM `global_racetrajectory_optimization` package (LGPL-3.0, Python) for the offline raceline, called as a separate tool so the license stays at arm's length from our binaries; the ForzaETH `race_stack` (MIT) as the reference architecture for the Frenet planner and the controller structure, read rather than vendored because its primary branch is ROS 1; `ublox_dgnss` (Apache-2.0) for the GNSS receiver; `livox_ros_driver2` (MIT) if we add the Mid-360.

### 2.6 What we will not build

We will not build a general navigation stack. The previous Purdue effort on this problem (Autonomous Motorsports Purdue, ASSv3, archived April 2024) ran ROS 2 Foxy on a Xavier NX with Nav2's behavior-tree navigator. General-purpose navigation stacks are built for obstacle-rich, low-speed environments and their planners optimize clearance, not lap time. We will not write our own DDS, our own logger, our own visualizer (Foxglove reads MCAP directly), or a custom kart chassis (the AKS rules forbid it anyway).

We will not port openpilot wholesale. Its messaging (cereal, msgq), process manager and hardware layer assume comma devices.

## 3. Targets and how they are measured

| Milestone | Measure | Pass condition |
| --- | --- | --- |
| Drive-by-wire | Steering step response, throttle and brake maps, recorded on the bench and in a parking lot | Documented rise time and steady-state error; gateway limits verified by fault injection |
| Localization | Fused pose versus RTK fix while driving a known loop | Under 0.10 m RMS lateral error at 100 Hz output with RTK fixed; graceful degradation to under 0.5 m over 10 s of GNSS outage |
| Map following | Parking-lot loop of about 100 m surveyed with the kart | 10 consecutive laps at 5 m/s with zero interventions |
| Sub-60 s lap | Purdue track, surveyed raceline | 5 consecutive laps, each under 60 s, zero interventions, kart never leaves the paved surface |
| Boundary-only mode | Same loop with the map withheld from the planner | 5 consecutive laps at reduced speed with zero interventions |
| Competition pace | Purdue track | Best lap under 45 s, 5-lap total under 4 minutes |

## 4. Rules that shape the software

From the AKS rulebook (development version, last updated December 11, 2025):

1. The track is the paved surface, at least 15 ft wide. Individual events may add features. Consequence: boundaries are asphalt edges, not cones, and the perception model must find pavement edges.
2. Race ranking is total time for 5 laps, one kart on track at a time. Practice-session fastest lap sets the running order. Consequence: five consecutive clean laps matter more than one fast lap.
3. All data recorded during official practice with the competing kart may be used during the race: GPS waypoints, centerline, edges, images, point clouds. Data from any other vehicle may not. Consequence: the survey tool must run on the kart, and the map pipeline must turn a practice session into a raceline in minutes.
4. Computation must be entirely onboard. Consequence: no pit-side inference; the pit link carries telemetry and RC only.
5. Teams interface with a Race Control System black box over CAN or USB, sending motor output power, health status, control mode and speed, and receiving track state, allowed control mode and GPS RTCM corrections. A C++/Python library or ROS 2 package will be provided. Consequence: `mcq_telemetry` owns this interface, and the GNSS driver must accept RTCM from it as well as from NTRIP.
6. Urgent stop must be commandable through software and must trigger if the RC controller leaves range. Remote e-stop must be tied to a physical controller with at least 200 m line-of-sight range and must cut drivetrain power after the urgent stop. A push-button physical e-stop must cut all power. Complete remote control (RC driving, mode switching, safety triggers) from the pit is required. Consequence: the gateway design in the safety document.
7. Drivetrain must be electric, rear-wheel only, at most 15 kW measured before the motor controllers; drivetrain battery at most 100 V nominal. Consequence: speed and power caps are parameters in the gateway, and motor power must be logged.
8. Each team is expected to use a Ubiquiti Bullet AC for the track network and may not use personal network devices from the pit.
9. Electronics not rated for outdoor use go in IP65 enclosures with ventilation. Non-internal parts may not be acrylic or brittle 3D prints.

## 5. Risks

| Risk | Why it matters | Mitigation |
| --- | --- | --- |
| RTK fix loss near trees or buildings at the track | The map-first mode depends on centimeter pose | IMU and wheel-speed dead reckoning in the EKF for short outages; geofence with covariance gating drops to urgent stop when the estimate is not trustworthy; boundary-only mode as the long-term fallback |
| Steering actuator too slow or too weak at speed | Lateral control bandwidth caps lap time and can cause oscillation | Characterize before choosing gains; specify the actuator from required steering rate at the tightest turn (see hardware doc) |
| JetPack 7.2 is new on Orin | Driver or CUDA breakage can eat weeks | Keep a JetPack 6.2.2 plus Humble image ready; pin all versions in the Docker files |
| Kart delivery lead time | Nothing on-track happens without it | Simulator and gateway firmware are built against the CAN spec before the kart arrives; a small test vehicle (an RC car or a go-kart chassis borrowed from the Grand Prix team) exercises the same code |
| Team turnover each semester | Knowledge leaves | Everything that is decided goes in `docs/`, every parameter lives in YAML, every drive produces an MCAP log with a README |

## 6. Suggested ownership

| Area | Packages | Skills |
| --- | --- | --- |
| Vehicle and electrical | `firmware/gateway`, `mcq_vehicle`, hardware docs | Embedded C, CAN, power electronics |
| Localization | `mcq_localization`, `tools/survey` | Estimation, GNSS |
| Planning and control | `mcq_track`, `mcq_planning`, `mcq_control`, `tools/raceline`, `mcq_sim` | Vehicle dynamics, optimization, C++ |
| Perception and learning | `mcq_perception`, `training/` | PyTorch, CUDA, TensorRT |
| Infrastructure | `mcq_bringup`, `mcq_telemetry`, `docker/`, CI | Linux, ROS 2, networking |
