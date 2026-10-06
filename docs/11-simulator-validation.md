# Recorded-data validation

Replay recorded commands through the dynamics model with one initial measured state. The replay never resets the kart to each measured point and never fits parameters to the validation recording.

```sh
.venv/bin/python -m mcq_sim validate measured-session.csv --out logs/validation.json
```

Required CSV columns, all SI:

| Column | Meaning |
| --- | --- |
| `t` | Monotonically increasing seconds, no gap over 0.2 s |
| `x,y,yaw` | Rear-axle pose in the track ENU frame, yaw radians counterclockwise from east |
| `v,v_lateral` | Forward and leftward CG body velocity in m/s |
| `steer,yaw_rate` | Actual front-wheel steering angle and body yaw rate, rad and rad/s |
| `steer_cmd,throttle_cmd,brake_cmd` | Commands held over the interval ending at this row's timestamp |
| `throttle,brake` (optional) | Actual normalized pedal actuator states, used at initialization |

The first row initializes the kart. Row i commands apply over `(t[i-1], t[i]]`, matching the viewer's telemetry export. Localize the GNSS antenna to the rear axle, transform IMU signals into the vehicle frame and synchronize clocks before export. Command history before the first row is unknown; start a capture at a settled standstill or account for this transient when interpreting the result. A synthetic self-replay is a software check, never real-world validation.

The report includes position RMSE and 95th-percentile position error, speed RMSE, yaw RMSE, yaw-rate RMSE, input hash and parameter values. Speed normalized RMSE uses maximum measured speed with a 1 m/s denominator floor. These are separate metrics. They do not establish "99% realism" or prove that a recording was held out from fitting.

Before calibration, agree on numerical error limits and coverage with the team. For example, a 1% speed target must specify the normalization, duration, speed range, grip and payload. Pose error requires a distance tolerance; dividing it by the length of the track can conceal unacceptable excursions. Stationary heading and near-zero acceleration require absolute tolerances. Validate whole maneuvers and independent laps, including failure cases, rather than only mean lap time.

The provisional geometry and planar tire model do not support aggressive kart-limit acceptance. Individual rear-wheel scrub, chassis flex, surface shape, tire temperature and wheel slip are absent. Use residuals from measurements to decide whether a four-wheel model and scanned terrain are required. Keep camera acceptance on held-out real kart recordings, with pavement IoU and boundary error reported separately from the simulator's procedural masks.

Modeling references reviewed on 2026-09-22: [CommonRoad vehicle dynamics](https://commonroad.in.tum.de/docs/commonroad-drivability-checker/sphinx/api_feasibility.html), [TUM vehicle dynamics source](https://github.com/TUMFTM/sim_vehicle_dynamics/blob/master/vehicle_model/vehicledynamics/src/VehicleModel.m). These motivate model structure; neither supplies calibrated parameters for this kart.
