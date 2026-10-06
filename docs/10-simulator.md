# Purdue simulator

The first build tests a camera-to-pavement model followed by a separate geometric controller. The learned model receives RGB only. Physics and scoring retain ground truth separately. No trained weights are supplied yet.

## Run

From the repository root:

```sh
uv venv .venv --python 3.12
uv pip install --python .venv/bin/python -e src/mcq_sim pytest cmake
.venv/bin/python -m mcq_sim view --port 0
.venv/bin/python -m mcq_sim evaluate --out logs/purdue-reference
.venv/bin/python -m mcq_sim evaluate --policy camera-demo --out logs/purdue-camera-demo
```

Open the printed localhost URL. Inputs, map and kart state use native controls. Run, pause, step, reset, change sliders, or select manual driving. W/A/S/D and the arrow keys control the kart. Reset after a boundary violation. The viewer advances simulated time on browser ticks, so playback can be slower than real time. Hiding the page pauses playback.

The reference controller uses the true map and is explicitly privileged. Camera demo uses an asphalt color threshold solely to exercise the perception interface. Neither is a trained driving model. `evaluate` returns nonzero on incomplete laps or failures and exports JSON plus CSV. Timing is deterministic lockstep; inference duration is measured but not simulated as additional latency.

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

`kart_provisional.yaml` sets the user-reported wheelbase to 1.4 m. The rear track upper bound is 1.397 m; overall width is a separate, provisional 1.6 m envelope. Mass, tire coefficients, response times and camera geometry remain estimates. The existing `run` and ROS simulator retain their earlier kinematic model.

## Accuracy work

Collect synchronized steering commands/feedback, pedals, speed, RTK rear-axle pose and IMU data across coastdown, braking, constant-radius turns and complete laps. Measure tire width, mass, CG, steering geometry, camera intrinsics/extrinsics and both pavement edges. Fit on some sessions, validate on separate sessions and conditions.

Use the [replay contract](11-simulator-validation.md) to measure errors. A single "99% accurate" score is undefined until observable quantities, normalization and the operating envelope are fixed. All current reports explicitly retain `accuracy_validated: false`.
