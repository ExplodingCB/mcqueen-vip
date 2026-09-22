# Track server

`track_server` loads one surveyed map for the driving graph and offers
`mcq/load_track`. It publishes `track_model` at 10 Hz and immediately on a valid
reload, and `geofence_state` on each ego update and every 100 ms. Failed reloads
preserve the live map. A new verdict is computed before publishing changed geometry.

`track_dir` must contain `track.csv` (x, y, right width, left width) and
`track.yaml` (`track_id`, `closed`, optional `raceline`). A raceline is a relative
or absolute path to seven semicolon-separated columns: s, x, y, heading,
curvature, target speed, target acceleration. The model transmits its geometry,
curvature and speed; the planner reconstructs arc length and heading and computes
acceleration as before. Raceline points must be distinct and inside the surveyed
edges. Without a raceline the planner uses the centerline.

The ROS-independent `mcq_track_core` library exports `mcq_track/track.hpp`.
The future state estimator links this library and builds a `Track` from the
centerline/width arrays in `TrackModel`; it owns `EgoState.s` and `.d`.
The track server never republishes the estimator's state.

Default geofence inflation is 0.5 m. The covariance gate limits the largest
map-plane variance to 0.25 m^2 and rejects malformed covariance. Position testing
uses edge polygons with a real interior hole for a closed loop and end caps for
an open track. Published distance includes the inflation. The Python reference's
`distance_to_edge` uses lateral Frenet clearance instead, so exact parity covers
Frenet conversions, splines, heading, curvature, widths and length, not polygon
distance at corners or open end caps.

Run host checks without ROS:

```sh
cmake -S src/mcq_track/core -B /tmp/mcq-track-build
cmake --build /tmp/mcq-track-build
ctest --test-dir /tmp/mcq-track-build --output-on-failure
PYTHONPATH=src/mcq_sim python -m pytest src/mcq_track/test
```

Parity tests feed both implementations the same CSV and assert maximum absolute
error below 1e-6, including the checked-in oval, variable widths, open tracks,
periodic seams, duplicate centerline rows and nonuniform sampling. Cartesian
queries exactly beyond a coarse polyline vertex can tie two clamped segments
with different signed offsets, so those extra coarse-track sweeps query segment
interiors. The oval sweep includes arbitrary offsets and queries around the seam.

After a ROS build, run `src/mcq_bringup/test/geofence_graph.sh shift` and the same
command with `covariance`. Observers join before the graph. Both start FOLLOW,
reject a malformed reload without changing the active map, inject a fault while
moving, and require the correct verdict, gateway urgent stop and standstill.
The `missing` scenario stops delivering verdicts during AUTO and requires the
controller to request urgent stop after the freshness budget. Every scenario also
asserts that RC startup commands never request urgent stop.
The shift fixture is 2 m wide because a 2 m shift of the usual 5 m oval does not
put a centered pose outside its inflated polygon. These graph checks run in CI;
host geometry tests alone do not establish ROS or driving acceptance.
