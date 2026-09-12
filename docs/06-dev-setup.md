# Development setup and conventions

## 1. Jetson

Flash JetPack 7.2 (Jetson Linux 39.x, Ubuntu 24.04, kernel 6.8, CUDA 13) with NVIDIA SDK Manager from an x86 Ubuntu host. JetPack 7.2 is the first release of the 7 line for AGX Orin (June 2026); if it causes trouble, JetPack 6.2.2 (Ubuntu 22.04, CUDA 12.6) with ROS 2 Humble is the fallback and the packages must stay buildable on both until the choice is settled. Verify the versions on the flashed unit with `apt list --installed | grep nvidia-jetpack` and `nvcc --version` and record them in this file.

After flashing: install an NVMe SSD and move `/home` and the log directory to it; set the maximum power mode (`sudo nvpmodel -m 0`) and lock clocks (`sudo jetson_clocks`) in a systemd unit; add the user to `dialout` for the GNSS receiver; bring up SocketCAN on the header controllers (`ip link set can0 up type can bitrate 1000000`) in a systemd unit; give the LiDAR and the pit link static addresses.

ROS 2 Jazzy comes from the OSRF apt repository (`ros-jazzy-ros-base` plus the packages listed in `docker/jetson/Dockerfile`). Do not build ROS 2 from source on the Jetson. Isaac ROS 4.6 packages, if used, come from NVIDIA's Isaac ROS apt repository or containers for JetPack 7.2 and Jazzy.

PyTorch on the Jetson is for experiments only. NVIDIA's Jetson wheel index had no JetPack 7 builds at the time of writing; community reports run the upstream aarch64 pre-release wheels for CUDA 13.2 on JetPack 7.2 (`pip install --pre torch --extra-index-url https://download.pytorch.org/whl/cu132`), which include the Orin `sm_87` target. Anything that runs on the kart uses TensorRT engines built from ONNX on the Jetson itself.

## 2. Workstation

Ubuntu 24.04 (native or WSL2) with Docker. The `docker/x86/Dockerfile` image is based on `ros:jazzy` and includes colcon, the Python tooling, and CUDA for training if an NVIDIA GPU is present. Everything except perception inference and Jetson-specific drivers runs in this container, including `mcq_sim` and the whole control graph.

Windows and macOS users work inside the container; there is no native support and nobody should spend time creating it.

## 3. Build and test

```
git clone git@github.com:ExplodingCB/mcqueen-vip.git
cd mcqueen-vip
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test && colcon test-result --verbose
```

C and C++ packages use `ament_cmake` with warnings as errors, `clang-format` (the config file at the root) and `clang-tidy` in CI. Python packages use `ruff` for lint and format (configured in the root `pyproject.toml`) and `pytest` for tests. `pre-commit` runs both before every commit. CI on GitHub Actions (`.github/workflows/ci.yml`) has a host job with no ROS (lint, clang-format, the gateway and control core cmake tests, pytest, and the simulator lapping the synthetic oval) and a Jazzy container job that builds the ROS 2 packages. The replay suite against reference MCAP files stored with Git LFS is added when the first logs exist.

The hot-path math is written once in dependency-free C: `firmware/gateway` (state machine, heartbeat, limits, codec) for the microcontroller and `src/mcq_control/core` (controllers, bicycle model, speed profile) for the Jetson. Both build standalone with cmake and carry their own host tests; the ROS 2 nodes are thin wrappers, and the Python simulator loads `mcq_control/core` through ctypes so the same code is exercised off the kart. Board support for the gateway is cross-compiled in the same container; the hardware-in-the-loop job is run manually on the bench rig. The README lists the exact commands.

### Bench without hardware

`firmware/gateway/build/gateway_host` is the gateway core running on a Linux machine against a SocketCAN interface, with fault-injection commands on stdin. On a laptop, `vcan0` stands in for the bus: the ROS 2 graph's `gateway_bridge` node talks to it exactly as it will talk to the board, and `firmware/gateway/host/test_gateway_host.py` walks the mode transitions over the bus from Python. On the bench rig the same binary runs on a USB CAN adapter next to the real actuators until the board firmware takes over.

## 4. Logging

`rosbag2` with the MCAP storage plugin (the Jazzy default) records every topic. Camera topics are recorded as compressed images. Logs are named `YYYY-MM-DD_HHMM_<track>_<kart>_<purpose>.mcap` and live on the Jetson's NVMe until copied to the team's shared storage; they are never committed to git. A `README.md` next to each log records track, weather, parameter set (git hash and YAML hash), what was tested, what broke, and who was operating.

`tools/logs/lap_report.py` summarizes a session (laps, deviation, speed, stop requests, gateway modes) without ROS installed; `tools/survey` turns edge drives from a log into a track directory. Foxglove opens MCAP files directly and connects live through `foxglove_bridge`. Layouts for the pit dashboard, the localization debug view and the control debug view are checked into `src/mcq_telemetry/layouts/`.

## 5. Conventions

Units are SI and angles are radians in every message, parameter and log; conversions happen at the human interface only. Frames follow REP 103 and REP 105. Every tunable is a ROS 2 parameter declared in a package `config/*.yaml`, with kart-specific and track-specific overlays in `mcq_bringup`; nothing is hard-coded and no track coordinates appear in source. Any behavior change on the kart ships with a replay test or a simulator test.

Branches are `feature/<area>-<short-name>` from `main`; every change goes through a pull request reviewed by someone outside the author's sub-team. The `docs/` directory is updated in the same pull request as the code it describes. Decisions that change a design document get a dated entry at the top of that document.

The CAN definition `src/mcq_vehicle/dbc/mcqueen.dbc` is the single source of truth for frame layouts, shared between the gateway firmware and `mcq_vehicle`; changing it changes both sides in the same pull request.
