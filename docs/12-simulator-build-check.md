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
