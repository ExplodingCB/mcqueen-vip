# References

Sources checked on 2026-09-11 while writing this draft, with what was taken from each. Several were reached through search summaries because the sites were not directly fetchable from the drafting environment; those are marked and should be re-read by whoever owns the relevant area.

## Track and competition

| Source | Used for |
| --- | --- |
| Purdue Grand Prix, Wikipedia (via search summary) https://en.wikipedia.org/wiki/Purdue_Grand_Prix | Track length 0.2698 mi (0.4342 km), 5 turns, asphalt, Northwest Sports Complex; lap record 24.616 s (2026, Yamaha KT100) |
| Autonomous Karting Series rulebook source (main branch, last updated December 11, 2025) https://github.com/autonomous-karting-series/Rulebook | Race procedure, track definition, allowed data, safety, software and network, electrical, mechanical rules quoted in the proposal, hardware and safety documents |
| AKS rulebook rendered site https://rules.autonomouskartingseries.com/ | Same content, rendered |
| AKS GitHub organization https://github.com/autonomous-karting-series | Race Control System server, connector and hardware repositories; GoKartSampleSet kart image dataset |
| 2026 AKS national competition at Purdue (via search summary) https://engineering.purdue.edu/CCE/AboutUs/News/Transportation_Features/lyles-cce-staff-and-students-host-the-2026-annual-autonomous-karting-series-aks-competition | Event dates May 18 to 22, 2026, and categories |
| Autonomous Motorsports Purdue, ASSv3 https://github.com/Autonomous-Motorsports-Purdue/AMP_ASSv3 | Prior Purdue stack: Xavier NX, JetPack 5.0.2, ROS 2 Foxy, ZED camera, Nav2 behavior-tree navigator; archived April 2024 |

## Compute and platform

| Source | Used for |
| --- | --- |
| JetPack 7.2 release coverage (via search summary) https://www.seeedstudio.com/blog/2026/06/12/nvidia-jetpack-7-2-release-agentic-ai-comes-to-jetson-agx-orin/ | JetPack 7.2 brings Ubuntu 24.04, kernel 6.8 and CUDA 13 to the whole Orin family; JetPack 7.2.1 with Jetson Linux 39.2.1 is current |
| JetPack 6.2.2 coverage (via search summary) https://jetsonhacks.com/2026/02/06/jetpack-6-2-2-for-jetson-orin/ | Fallback: Jetson Linux 36.5, kernel 5.15, Ubuntu 22.04 |
| JetPack 7.2 field install notes https://github.com/iuliaferoli/jetson-jp7.2-install | L4T r39.2, CUDA 13.2, PyTorch 2.12.1+cu132 via upstream pre-release wheels, ROS 2 Jazzy installed system-wide, Python 3.12 |
| Jetson AGX Orin Developer Kit specifications (via search summary, NVIDIA technical brief) https://www.nvidia.com/content/dam/en-zz/Solutions/gtcf21/jetson-orin/nvidia-jetson-agx-orin-technical-brief.pdf | 12-core A78AE, 2048 CUDA cores, 64 Tensor Cores, 64 GB LPDDR5 at 204.8 GB/s, 15 to 60 W |
| REP 2000, ROS 2 releases and target platforms https://github.com/ros-infrastructure/rep/blob/master/rep-2000.rst | Jazzy Jalisco (May 2024 to May 2029) Tier 1 on Ubuntu 24.04 arm64; Kilted Kaiju EOL November 2026 |
| Lyrical Luth release timeline (via search summary) https://docs.ros.org/en/jazzy/Releases/lyrical/release-timeline.html | Released May 22, 2026, LTS to May 2031 |
| Isaac ROS 4.6 on Orin with JetPack 7.2 (via search summary) https://nvidia-isaac-ros.github.io/releases/index.html and https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common | Isaac ROS 4.6.0 (August 2026) supports Jetson Orin with JetPack 7.2 and ROS 2 Jazzy; cuVSLAM and nvblox available |

## Sensors and drivers

| Source | Used for |
| --- | --- |
| `ublox_dgnss` ROS 2 driver https://github.com/aussierobots/ublox_dgnss | ZED-F9P, ZED-F9R and ZED-X20P over USB; Humble, Jazzy, Kilted and Rolling; Apache-2.0 |
| u-blox ZED-F9P data sheet (via search summary) https://content.u-blox.com/sites/default/files/ZED-F9P-04B_DataSheet_UBX-21044850.pdf | RTK navigation rate up to 20 Hz |
| Indiana INDOT InCORS https://incors.in.gov/rtk.aspx | Free statewide network RTK corrections over NTRIP (RTCM 3.x MSM4 among others); sign-up by email |
| Livox ROS driver 2 https://github.com/Livox-SDK/livox_ros_driver2 | Mid-360 support under ROS 2; MIT; a Jazzy-compatible fork exists |

## Software to reuse

| Source | Used for |
| --- | --- |
| openpilot https://github.com/commaai/openpilot (files read from master on 2026-09-11: `openpilot/selfdrive/controls/controlsd.py`, `openpilot/selfdrive/controls/lib/latcontrol.py`, `latcontrol_torque.py`, `longcontrol.py`, `openpilot/selfdrive/pandad/pandad.py`, `openpilot/common/pid.py`) | 100 Hz control loop structure; lateral controllers in angle, torque, PID and curvature variants; longitudinal state machine and PID; PID with speed-scheduled gains; MIT license; Python 3.12 |
| opendbc https://github.com/commaai/opendbc (files read: `opendbc/car/interfaces.py`, `opendbc/car/body/*`, `opendbc/car/lateral.py`, `opendbc/car/vehicle_model.py`, `opendbc/safety/modes/body.h`) | CarInterface, CarState, CarController split; the comma body port as a minimal non-car template; lateral acceleration and jerk limit helpers; dynamic bicycle model; firmware safety-mode hook pattern; MIT license |
| openpilot safety model (via search summary) https://blog.comma.ai/understanding-the-openpilot-safety-model/ | Panda sits inline and filters every actuator message in firmware, independent of the planner |
| sunnypilot comparison (via search summary) https://commaguide.com/compare/openpilot-vs-sunnypilot | Fork differences are car and comma-device features (model picker, MADS); nothing kart-relevant beyond openpilot |
| TUM global race trajectory optimization https://github.com/TUMFTM/global_racetrajectory_optimization | Shortest path, minimum curvature, iterative minimum curvature and minimum time methods; centerline-plus-widths CSV input; raceline with velocity profile output; LGPL-3.0 |
| ForzaETH race stack https://github.com/ForzaETH/race_stack | Reference architecture for a winning F1TENTH stack: Cartographer, TUM raceline, Frenet planner, MAP and pure-pursuit controllers; ROS 1 Noetic primary with a ROS 2 branch; MIT |
