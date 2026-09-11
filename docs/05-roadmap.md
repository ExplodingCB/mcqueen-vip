# Roadmap

Six phases across the 2026 to 2027 academic year. Dates are targets, not commitments. Each phase ends with a test that either passes or does not; there is no partial credit, because the next phase builds on the result.

## Phase 0: foundations (September to mid October 2026)

Work: repository structure, Docker images for x86 and Jetson, CI that builds the workspace and runs tests, `mcq_msgs`, the CAN DBC file, `mcq_sim` with a kinematic bicycle model and the standard message interfaces, the gateway firmware state machine on a development board, kart procurement, and a bench rig with whatever actuators have arrived.

Exit test: the full ROS 2 graph runs against `mcq_sim` on a laptop with a pure-pursuit controller following a synthetic oval; the gateway firmware passes the fault-injection list in a hardware-in-the-loop setup on the bench (Jetson or laptop over CAN to the board); a session from the simulator is recorded to MCAP and replayed.

## Phase 1: drive-by-wire and localization (mid October to November 2026)

Work: actuators fitted and characterized (steering step and frequency response, throttle map from command to acceleration, brake map and stopping distance from 8 m/s), RC driving through the gateway, GNSS and IMU mounted and calibrated, InCORS NTRIP account, `state_estimator` with RTK and IMU and wheel speeds, the survey tool.

Exit test: a lap of a parking-lot loop under RC with all sensors logged; fused pose within 0.10 m RMS of the RTK fix; a track model file produced from that log by the survey tool; the fault-injection list passed on the kart at standstill and at 3 m/s.

## Phase 2: map following at low speed (November to December 2026)

Work: `mcq_track`, `local_planner` in `FOLLOW` mode on the centerline, `controller` with pure pursuit and the ported longitudinal PID, geofence and pose-trust checks, telemetry dashboard, speed cap stepping procedure.

Exit test: 10 consecutive laps of a surveyed 100 m loop at 5 m/s with zero interventions, followed by the on-track subset of the fault-injection list at 5 m/s.

## Phase 3: the Purdue track under 60 seconds (January to February 2027)

Work: survey the Grand Prix track with the kart, generate a minimum-curvature raceline and a conservative speed profile, replace pure pursuit with the bicycle-model curvature controller, identify tire and drivetrain parameters from logs, raise the speed cap step by step.

Exit test: five consecutive laps each under 60 s with zero interventions and the kart never leaving the paved surface. This is the headline milestone.

## Phase 4: boundary-only mode and pace (February to March 2027)

Work: camera pipeline with a TensorRT segmentation model trained on our own laps, `TrackBounds` fusion in `mcq_track`, `BOUNDARY` sub-mode in the planner, LiDAR edge extraction if the Mid-360 is fitted, minimum-time raceline with an identified ggv, MPC lateral controller.

Exit tests: five laps in `BOUNDARY` mode at the reduced cap with zero interventions and the map withheld; best lap under 45 s in `FOLLOW` mode.

## Phase 5: competition readiness (April to May 2027)

Work: RCS black box integration once the AKS library is published, inspection documentation (electrical harness and battery descriptions, mechanical BOM), weatherproofing check, the practice-session-to-raceline pipeline timed end to end, reliability laps.

Exit test: a full simulated race day: arrive, survey during a practice session, generate the raceline, run five timed laps, all within the session windows the AKS schedule allows. Entry into the May 2027 AKS national event if the program decides to enter.

## After the first season

Learned dynamics residuals feeding the MPC; a simulation-trained trajectory policy behind the same envelope; dual-antenna heading; multi-kart detection and overtaking for the exposition runs; a second kart.

## What gets cut first if time runs out

LiDAR, MPC and the learned components are the first to go. Boundary-only mode is second. The sub-60 s lap in `FOLLOW` mode with a surveyed map, pure pursuit and RTK is the minimum viable season, and it is achievable with Phase 0 through Phase 3 alone.
