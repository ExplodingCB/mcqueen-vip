# Hardware

What the software assumes about the kart, and the reasons behind each choice. Part numbers are suggestions to be confirmed by the electrical sub-team; interfaces are requirements.

## 1. Kart platform

The AKS rulebook requires a commercial full-size sprint kart chassis (no scratch-built frames), an all-electric rear-wheel drivetrain at most 15 kW, a drivetrain battery at most 100 V nominal, a minimum weight of 275 lb with everything fitted, and a mechanical brake on both rear wheels. It suggests the Blue Shock Race X3 e-kart for new teams and quotes it at $12,500 to $13,300 plus freight as a full kit (15 kW motor, controller, 96 V Li-ion pack) or $4,700 to $5,500 plus freight as a bare chassis. Tires are fixed by rule: 10 x 4.50-5 front, 11 x 7.10-5 rear, in Hoosier R60B or Vega XH3 or XM3 compounds.

Buy the full kit if the budget allows. Integrating a separate motor and controller is a semester of electrical work that produces nothing the software team can use.

Kart-scale numbers the software needs, to be measured on delivery: wheelbase (about 1.05 m on a sprint kart), track width, mass with all equipment, center of gravity height, steering ratio between column and front wheels (karts are direct, roughly 1:1 with Ackermann geometry), maximum front-wheel steering angle.

## 2. Actuators

| Actuator | Requirement | Interface to the gateway | Feedback |
| --- | --- | --- | --- |
| Steering | Position-controlled motor on the column. At least 180 deg/s at the front wheels under load at standstill, holding torque above the self-aligning torque at the maximum lateral acceleration we intend to reach. Absolute position feedback independent of the motor encoder. | CAN position or torque command, 100 Hz | Column absolute encoder (14-bit magnetic, AS5047 class, or industrial absolute) plus motor driver encoder |
| Throttle | Command to the motor controller's throttle input | CAN if the controller supports it; otherwise an isolated DAC driving the 0 to 5 V input | Motor controller telemetry (current, rpm, temperature, power) |
| Brake | Actuator on the master cylinder capable of locking the rears from 12 m/s. Must reach full brake on gateway command alone within 300 ms. Stroke or pressure feedback. | PWM or CAN, 100 Hz | Stroke sensor or brake-line pressure sensor |
| Drivetrain contactor | Main contactor in the drivetrain battery circuit | Gateway output through a relay; remote e-stop relay in series | Auxiliary contact read back |

Steering actuator candidates: a BLDC servo with planetary reduction on an ODrive or VESC controller in position mode, or an automotive electric power steering column motor driven in torque mode. Whichever is chosen, the software supports both a position interface (default) and a torque interface (the openpilot torque-controller pattern). Size it from the required steering rate, not from the required angle: at 10 m/s a lateral acceleration change of 4 m/s^2 in 0.5 s at a 1.05 m wheelbase corresponds to roughly 2.4 deg of front-wheel angle change, so the actuator rate limit is set by transient corrections and disturbance rejection, not by the turns themselves.

Brake actuator: a ball-screw linear actuator with position feedback on the pedal or master cylinder lever, or a brake-by-wire servo. A design where loss of actuator power leaves the brakes applied is preferred; if that is not practical, the gateway's urgent stop drives the actuator to full brake before opening the contactor.

## 3. Safety gateway

A microcontroller board with two CAN buses (one to the Jetson, one to the actuators and motor controller), a serial port for the RC receiver (CRSF protocol from an ExpressLRS receiver), digital inputs for the remote e-stop relay and the contactor auxiliary contact, and analog or digital inputs for the sensors that the safety logic needs (steering encoder, wheel speed hall sensors, brake feedback, drivetrain current).

Candidates: an STM32H7 board or a Teensy 4.1 with CAN transceivers on a custom carrier, or a comma panda (STM32H7, USB and three CAN buses), which can run a custom safety mode inside the opendbc safety framework. The panda is attractive because its safety framework, its test harness and its USB protocol already exist; the custom board is attractive because it can host the RC receiver, the encoder inputs and the contactor driver directly. Either way the firmware is ours and the behavior is specified in the safety document.

## 4. Remote control and e-stops

| Function | Component | Rule it satisfies |
| --- | --- | --- |
| RC driving and mode switch | ExpressLRS transmitter (RadioMaster Pocket or similar) and diversity receiver on the kart, 2.4 GHz for bring-up, 900 MHz if range at the track is marginal. Failsafe configured to a defined channel state that the gateway interprets as link loss. | Complete remote control from the pit; urgent stop on controller out of range |
| Remote e-stop | Dedicated wireless e-stop with a physical trigger and at least 200 m line-of-sight range (the AKS resource list names the Kar-Tech wireless e-stop). Its receiver relay is in the contactor circuit and its state is read by the gateway. | Remote e-stop tied to a physical controller; initiates urgent stop then cuts drivetrain power |
| Physical e-stop | Push-button mushroom switch that disconnects both battery systems | Physical e-stop cutting all power; kill switch per battery system |

## 5. Sensors

| Sensor | Choice | Rate | Notes |
| --- | --- | --- | --- |
| RTK GNSS | u-blox ZED-F9P (ArduSimple simpleRTK2B or SparkFun GPS-RTK2 carrier), dual-band antenna on a ground plane at the highest point of the kart | 20 Hz RTK fixed | Driver: `ublox_dgnss` (ROS 2 Jazzy, USB). Corrections from InCORS NTRIP during testing, from the RCS black box at competition. Two receivers in moving-base configuration give heading without motion; a ZED-X20P (triple band) converges faster if the budget allows. |
| IMU | 200 to 400 Hz MEMS IMU. An industrial unit (VectorNav VN-100, Xsens MTi-3) if funded; otherwise a BMI088 or ICM-42688 on the gateway board, timestamped there and sent over CAN | 200 Hz minimum | Mount rigidly near the center of gravity; vibration isolation is a calibration problem, not a mounting afterthought |
| Wheel speed | Hall sensors on both front hubs (unpowered wheels give the cleanest speed) and the motor encoder for the rear | 100 Hz from the gateway | Front and rear disagreement flags wheel slip |
| Steering angle | Absolute encoder on the column | 100 Hz from the gateway | Independent of the actuator's own encoder |
| Camera | One forward global-shutter camera: GMSL2 module on a Jetson adapter board, or a USB3 machine-vision camera, 1280 x 800 class at 30 fps, 90 to 110 deg horizontal field of view | 20 to 30 Hz | A Stereolabs ZED X (GMSL2, stereo plus IMU) is the integrated alternative if stereo depth is wanted |
| LiDAR (optional) | Livox Mid-360 (360 deg, 10 Hz, about 40 m range at 10 % reflectivity) | 10 Hz | Driver: `livox_ros_driver2` (MIT; a Jazzy-compatible fork exists). Ethernet, needs a fixed IP on the kart network |

## 6. Compute

NVIDIA Jetson AGX Orin Developer Kit, 64 GB: 12-core Arm Cortex-A78AE, Ampere GPU with 2048 CUDA cores and 64 Tensor Cores, two NVDLA v2 accelerators, 64 GB LPDDR5 at 204.8 GB/s, 15 to 60 W configurable. Run it in the maximum power mode with clocks locked; a 60 W budget is trivial next to a 15 kW drivetrain.

Interfaces we use: the 40-pin header carries two CAN controllers that need external transceivers (a gs_usb USB-CAN adapter is the fallback), USB 3 for the GNSS receiver and a USB camera, 10 GbE for LiDAR and the pit link through a small switch, the CSI connector for a GMSL2 camera adapter, and an M.2 NVMe SSD for logs (the 64 GB eMMC fills in one afternoon of camera recording).

Power: 19 V DC from the auxiliary battery through a DC-DC converter with a hold-up capacitor so contactor transients do not reboot it. Mount in an IP65 enclosure with a fan and filtered vents, on vibration isolators.

## 7. Network

Kart side: a small gigabit switch joining the Jetson, the LiDAR, and a Ubiquiti Bullet AC (the AKS-expected track radio). Pit side: laptop on the AKS-provided ethernet drop or a second Bullet for private testing. The RC and remote e-stop radios are separate from this network on purpose; the safety chain does not depend on Wi-Fi.

Internet for NTRIP during testing: an LTE modem on the kart, or the pit laptop relaying corrections through the Bullet link. At competition the RCS black box supplies RTCM directly.

## 8. Power

Two battery systems, each with its own labeled kill switch, as the rules require. The drivetrain pack (96 V nominal in the BSR kit) feeds only the motor controller through the main contactor and a fuse rated for the full configured power. The auxiliary pack (24 V nominal LiFePO4 suggested; the rule cap is 50 V nominal) feeds the Jetson, gateway, sensors, steering and brake actuators, radios and the RCS black box link. Both packs live in enclosed, ventilated containers with Anderson SB175 external connectors, and the drivetrain current is measured before the controller so motor power can be logged and reported.

## 9. Bench rig

Before the kart exists and every time it is unavailable, a bench rig runs the full software: the gateway board, the steering actuator on a fixture, the brake actuator, the RC receiver and the remote e-stop, wired exactly as on the kart, with the Jetson (or an x86 machine) running the graph against `mcq_sim`. Every fault-injection test in the safety document runs here first.
