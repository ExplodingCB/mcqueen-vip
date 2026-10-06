# Purdue simulator

Two things drive the dynamic kart on the Purdue track. A camera-to-pavement model followed by a separate geometric controller tests perception; the learned model receives RGB only. The full software stack tests everything else: sensor models, the C state estimator, the Frenet planner and the C controllers in one loop. Physics and scoring retain ground truth separately. No trained weights are supplied yet.

## Run

From the repository root:

```sh
uv venv .venv --python 3.12
uv pip install --python .venv/bin/python -e src/mcq_sim pytest cmake
.venv/bin/python -m mcq_sim view --port 0
.venv/bin/python -m mcq_sim evaluate --out logs/purdue-reference
.venv/bin/python -m mcq_sim evaluate --policy camera-demo --out logs/purdue-camera-demo
.venv/bin/python -m mcq_sim evaluate --policy stack --out logs/purdue-stack
.venv/bin/python -m mcq_sim view --policy stack
```

Open the printed localhost URL. Inputs, map and kart state use native controls. Run, pause, step, reset, change sliders, or select manual driving. W/A/S/D and the arrow keys control the kart. Reset after a boundary violation. The viewer advances simulated time on browser ticks, so playback can be slower than real time. Hiding the page pauses playback.

The reference controller uses the true map and is explicitly privileged. Camera demo uses an asphalt color threshold solely to exercise the perception interface. Neither is a trained driving model. `evaluate` returns nonzero on incomplete laps or failures and exports JSON plus CSV. Timing is deterministic lockstep; inference duration is measured but not simulated as additional latency.

## The full stack

`--policy stack` closes the loop the kinematic harness and the estimator tests each closed halfway:

```
truth -> sensors -> C EKF -> planner (20 Hz) -> C controllers (100 Hz) -> dynamic kart
```

Nothing after the sensor models reads the truth. GNSS arrives at 20 Hz and 80 ms late, the IMU runs at 200 Hz with a bias that walks, wheel speed and steering angle are quantized. The planner and controllers take the estimator's pose and speed. Stop conditions run on the estimator's own doubt: the geofence on the believed map, a covariance gate at the geofence margin, the trajectory-age check and the steering/yaw-rate consistency check. The kart holds the brake until the first fix. It starts staged on the line, so the filter takes its position from that fix and its heading from the track with a 0.1 rad sigma; course over ground tightens it once the kart moves. `evaluate` reports the estimator's position and yaw error against the truth beside the lap result, and the telemetry CSV carries `est_*` columns.

Sensor settings and every controller gain live in `src/mcq_sim/config/sim_default.yaml`. Override them per run, with JSON values, for example a ten second GNSS outage:

```sh
.venv/bin/python -m mcq_sim evaluate --policy stack \
  --set 'sensors.gnss.schedule=[[0,"FIXED"],[30,"NONE"],[40,"FIXED"]]'
```

The planner follows a minimum-curvature line through the pavement rather than the KML centerline (`mcq_sim/reference.py`). The KML is a hand-drawn trace with kinks and one point where its curvature exceeds what the steering lock can follow; the smooth line is what a driver would take. It stands in for the raceline tooling in issue #5. The viewer's "Full stack" driver draws the true kart, the estimator's pose and this line.

The IMU is modeled at the rear axle, the estimator's `base_link`. The dynamics report acceleration at the center of gravity 0.6 m ahead of it, and moving it back matters: skipping the lever-arm terms cost a metre per second squared of lateral error through steering transitions and drove position error to 0.27 m RMS before the filter lost the fix. Real hardware will mount the IMU somewhere else, and the sensor frames in the URDF (task G1) are where that offset will be measured.

## Connect a model

Create an importable Python module with a zero-argument factory:

```python
def create():
    return YourSegmenter()


class YourSegmenter:
    def predict(self, rgb):
        # uint8 HWC RGB, 270 x 480. Apply your training normalization.
        # Run your network and resize its pavement probability to 270 x 480.
        return pavement_probability  # bool or finite floats in [0, 1]
```

Run `mcq_sim evaluate --policy your_module:create`. Models may implement `reset(seed)`. Invalid masks, exceptions and missing drivable corridors terminate the episode. Reported IoU concerns procedural images only. Real camera footage remains the perception acceptance set.

## Physics and provenance

The new environment uses 100 Hz steps with internal 400 Hz integration, a dynamic bicycle, axle slip forces, combined rear grip, longitudinal load transfer and delayed actuators. Pose is at the rear axle; lateral velocity is at the center of gravity. Full-perimeter road checks terminate off-track episodes. No barrier impact dynamics, individual wheel rotation, rear-axle scrub, chassis flex, banking or bumps are modeled.

`kart_provisional.yaml` sets the user-reported wheelbase to 1.4 m. The rear track upper bound is 1.397 m; overall width is a separate, provisional 1.6 m envelope. Mass, tire coefficients, response times and camera geometry remain estimates. The existing `run` command and the ROS 2 `sim_node` retain their earlier kinematic model; the estimator tests still use it. The dynamic kart is not yet behind the ROS graph.

## Accuracy work

Collect synchronized steering commands/feedback, pedals, speed, RTK rear-axle pose and IMU data across coastdown, braking, constant-radius turns and complete laps. Measure tire width, mass, CG, steering geometry, camera intrinsics/extrinsics and both pavement edges. Fit on some sessions, validate on separate sessions and conditions.

Use the [replay contract](11-simulator-validation.md) to measure errors. A single "99% accurate" score is undefined until observable quantities, normalization and the operating envelope are fixed. All current reports explicitly retain `accuracy_validated: false`.
