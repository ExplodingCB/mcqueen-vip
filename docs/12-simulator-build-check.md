# Simulator build check, 2026-09-22

The local viewer uses native controls in three regions: Inputs, Map, Kart state. Its map displays the pinned Indiana orthophoto, estimated pavement edges, the user's 63 rough coordinates and the simulated kart. Aerial and point overlays can be toggled. Direction is counterclockwise, explicitly confirmed by the user; a regression assertion checks the ENU signed area and southeast-facing start pose.

The first OSM source was rejected because it followed the pit lane instead of the main straight. The replacement trace produces a 432.70 m centerline and image-estimated widths of 4.66 to 6.05 m. The user's 1.4 m wheelbase is configured; the 55-inch rear track maximum is recorded separately from the assumed body envelope.

Validation commands:

```sh
PATH="$PWD/.venv/bin:$PATH" .venv/bin/python -m pytest src/mcq_sim/test tools/survey
.venv/bin/ruff check .
.venv/bin/ruff format --check .
node --check src/mcq_sim/mcq_sim/web/app.js
git diff --check
.venv/bin/python -m mcq_sim evaluate --out logs/purdue-final-reference
.venv/bin/python -m mcq_sim evaluate --policy camera-demo --out logs/purdue-final-camera-demo
```

The reference run completed one counterclockwise lap in 116.50 simulated seconds at a 4 m/s cap, with zero body-boundary violations and minimum modeled body clearance of 1.008 m. Its record is `logs/purdue-final-reference/report.json`; CSV telemetry is beside it.

The camera demo failed at 19.78 simulated seconds with `body_outside_track`. Its synthetic pavement IoU was 1.0, which exposes a limitation of the simple mask-to-steering controller even with correct synthetic segmentation. This is retained as a failed test result in `logs/purdue-final-camera-demo/report.json`. No trained model was available to evaluate.

The test suite covers dynamics symmetry, tire-force limits, low-grip braking, command delay, timestep convergence, deterministic reset, body-envelope boundary detection, invalid model outputs, open-loop replay, georeferenced geometry, source anchors and HTTP control behavior. Browser inspection checked aerial alignment, running, pausing and plain control layout.

This is an uncalibrated planar simulator. The successful reference lap verifies software behavior on estimated geometry. Physical accuracy, real-camera perception, ROS integration of the new dynamics, and a 99% agreement target remain unverified. No deployment or on-kart test was performed.

## Refined KML revision

The later user KML moves 19 of the 63 points. It now defines the actual simulation centerline, with every source waypoint retained and counterclockwise travel enforced. The original image-derived pavement boundaries are transferred as asymmetric widths. The reference controller centers the kart in that pavement corridor. The new centerline is 435.07 m; previous results above describe the earlier image-centerline build.

The updated reference loop completed a lap with zero boundary violations at both 4 m/s and 8 m/s caps (126.15 s and 111.90 s, respectively). Minimum modeled body clearance was 1.025 m in both runs. Reports are in `logs/purdue-line-v2-reference` and `logs/purdue-line-v2-reference-8mps`. The 14 environment and viewer tests passed. Camera-demo corner performance has not been re-evaluated for this revision.

## Full stack build check, 2026-09-29

The kinematic harness and the dynamic Purdue environment were separate simulators, and the estimator was scored on a third arrangement that watched a kart it did not control. `--policy stack` joins them: sensors, the C estimator, the Frenet planner and the C controllers drive the dynamic kart and read nothing from the truth. Design and usage are in [the simulator guide](10-simulator.md).

Two defects surfaced while joining them, both worth knowing about.

The estimator's library loader only looked for a `.so`, so all six estimator tests failed on macOS the same way the control core had before commit `228bcac`. It now looks for `.dylib` there.

The first integrated run lost the GNSS fix at 20 s and drifted to 1.4 m. The cause was the IMU feed: the dynamics report acceleration at the center of gravity, 0.6 m ahead of the rear axle where the estimator works, and yaw acceleration of 1 to 2 rad/s^2 through a steering transition turns that lever arm into about a metre per second squared of lateral error. Velocity moved off the fix, the gate at a filter sigma of 6 mm rejected every fix afterward, and the estimate ran away while still reporting a healthy covariance. Moving the IMU to the rear axle took position error from 0.27 m RMS to 1.8 cm. A separate scoring mistake, comparing an estimate at time t with the truth after the step to t + 0.01 s, had added a fixed 4 cm at 4 m/s; scoring now happens before the step.

The KML centerline cannot be followed by the planner. Its curvature peaks at 0.55 against a steering lock that allows 0.345, and where the line hugs the outside of the hairpin about 159 m along the lap no candidate path was feasible. The planner now offsets from a minimum-curvature line through the pavement (`reference.py`), whose peak curvature is 0.336 and which stays 1.3 m from both edges. The privileged reference driver was never affected because it ignores the line's shape.

Results, seed 0 unless stated, one Purdue lap, 240 s limit. Reports are not committed; regenerate with the commands in the guide.

| Case | Lap | Min. body clearance | Estimator position RMS / max | Result |
| --- | --- | --- | --- | --- |
| 4 m/s cap | 107.7 s | 0.23 m | 1.8 cm / 3.7 cm | pass |
| 4 m/s cap, seed 1 | 107.6 s | 0.22 m | 1.8 cm / 3.6 cm | pass |
| 6 m/s cap | 76.8 s | 0.25 m | 2.1 cm / 4.4 cm | pass |
| 8 m/s cap | 68.4 s | 0.26 m | 2.4 cm / 5.5 cm | pass |
| 5 s GNSS outage at 30 s | 107.7 s | 0.22 m | 2.0 cm / 12 cm | pass |
| 10 s GNSS outage at 30 s | 107.7 s | 0.23 m | 3.0 cm / 30 cm | pass |
| 20 s RTK float at 30 s | 107.6 s | 0.15 m | 4.0 cm / 17 cm | pass |
| 6 m/s cap, 10 s outage at 30 s | stopped at 40.4 s | n/a | n/a / 57 cm | covariance gate stop |

The last row is the intended behavior: at 6 m/s the outage drifts past the 0.5 m geofence margin, the covariance gate fires and the kart brakes to a stop with no boundary violation. Estimator error is against the truth at the same instant, from 2 s onward. Yaw RMS is 0.5 to 0.9 degrees.

What this does not show. The estimator's noise parameters were written for these sensor models, so the numbers are a lower bound on what a kart will do. Minimum clearance of 0.15 to 0.26 m is much tighter than the reference driver's 1.0 m, because a minimum-curvature line cuts corners by design; it is a real property of this planner and this geometry, not a margin. The camera-demo driver and the perception path are untouched and their limitations above stand. The ROS 2 graph still runs the kinematic kart, and nothing here is calibrated against a real kart.
