# mcq_telemetry

H1 pit telemetry: a 10 Hz summary, finish-line lap timing, a Foxglove WebSocket
bridge, and four repository-owned layouts. Develop against the simulator before
using this on the kart. The AKS RCS black box interface is Phase 5 and is not
implemented here.

## Before AUTO: required startup order

**Start telemetry and the bridge, connect every Foxglove client, and load the
intended layouts before entering AUTO.** A new DDS participant joining mid-run
has frozen the Jazzy rclpy nodes for **200–350 ms**, exceeding the controller's
200 ms trajectory freshness limit and correctly triggering an urgent stop.
Until [issue #4](https://github.com/ExplodingCB/mcqueen-vip/issues/4) is resolved,
this restriction also applies to opening Foxglove from the pit. Do not reconnect,
switch to a layout that introduces new subscriptions, or start diagnostic ROS
commands during AUTO. Return to RC first. See architecture section 0.

Telemetry and bridge topic subscriptions are explicitly **BEST_EFFORT,
KEEP_LAST, depth 1, VOLATILE**. The bridge forces best-effort even when an
upstream publisher offers reliable delivery; it does not negotiate a reliable
reader. Separate processes run at nice level 10. Input callbacks only cache the
latest message and do constant-time crossing geometry; file loading happens
once at startup. No telemetry service calls, disk writes, or network calls are
on the control path. Bounded queues drop old observations instead of retaining a
backlog. This design still requires the measured comparison below; QoS alone is
not evidence of unchanged timing.

Bridge settings use the upstream
[best-effort topic whitelist and QoS depth parameters](https://github.com/foxglove/foxglove-sdk/blob/main/ros/src/foxglove_bridge/README.md).
The bridge exposes only the listed dashboard topics and observation capability.

## Simulator and a second machine

From the repository root in a Jazzy development environment:

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch mcq_bringup sim.launch.py telemetry:=true handover_delay:=60.0
```

The simulator's handover delay is a setup window, **not a readiness interlock**.
Connect and confirm live data while the gateway indicator is RC. If the window
expires before setup is complete, stop the simulation and restart it with a
longer delay. The real kart requires the operator to keep the gateway in RC
until the observers are ready. Default simulation behavior without telemetry
remains unchanged.

On the second machine, open Foxglove, choose a Foxglove WebSocket connection,
and connect to `ws://<sim-host-LAN-IP>:8765`. Import the JSON files below through
the Layout menu. No ROS installation is needed on that machine. TCP port 8765
must be reachable across the LAN. For Docker, use host networking on Linux or
publish the port with `-p 8765:8765`; give the container `--shm-size=1g` as in CI.
The existing `docker/x86/Dockerfile` includes the bridge.

| Layout | Purpose |
| --- | --- |
| `layouts/pit.json` | Gateway and GNSS indicators; speed/target/cap; lateral deviation; faults; current, last and best lap; completed laps and input freshness |
| `layouts/live-tracking.json` | 3D track centerline, live kart pose and recent trail; mode, GNSS and active faults; speed, deviation, laps and input freshness |
| `layouts/localization-debug.json` | Map coordinates, Frenet progress/deviation, pose covariance, GNSS state and state age |
| `layouts/control-debug.json` | Speed error, target and cap; steering command/feedback; pedals; stops, gateway faults and input ages; raw trajectory |

Load every layout you plan to use before AUTO. The layouts use SI units
and built-in panels, with no custom extension or user script. The original
panel configurations were generated against Foxglove SDK 0.27.0's layout
models; the live-tracking 3D panel follows the current
[3D panel documentation](https://docs.foxglove.dev/docs/visualization/panels/3d). Actual
import/rendering and the second-machine connection remain a manual acceptance
check; schema checks do not substitute for that check.

The live-tracking view uses the local `map` frame in the 3D panel. The telemetry
node publishes `/telemetry/pose` (`geometry_msgs/PoseStamped`) at up to 10 Hz
when the map-frame ego pose is fresh, plus `/telemetry/track` and
`/telemetry/trail` (`nav_msgs/Path`) at 1 Hz. The track path is the closed
centerline from the active `track.csv`; the trail holds at most 30 seconds of
recent valid poses. The 3D view is not a geographic map. When ego data becomes
invalid the pose stops updating and the trail clears; use the `ego_valid` and
Foxglove connection indicators to detect a stale displayed pose.

To compose this package into another bringup, start its launch before AUTO:

```bash
ros2 launch mcq_telemetry telemetry.launch.py track:=/absolute/path/to/track port:=8765
```

Do not also enable `telemetry:=true` in the simulator when running this standalone
launch. Parameters live in `config/telemetry.yaml`; bridge settings are in
`config/bridge.yaml`. Both configurations and all layouts install into the
package share directory.

## Data contract

`/telemetry/summary` (`mcq_msgs/Telemetry`) is published at 10 Hz. Every input has
an age in seconds and a validity flag; missing, future-dated and older-than-0.5 s
inputs are invalid. Invalid floating-point values are NaN and status text is
UNKNOWN. Integer fault flags and the urgent-stop boolean must be read together
with `gateway_valid` and `command_valid`; their default values do not mean
healthy when the source is missing. A disconnected Foxglove session can retain
its last displayed values: its connection/data-source status must also be live.

| Source | Summary |
| --- | --- |
| `/ego_state` | Speed, signed lateral deviation `d`, GNSS state, map pose for lap timing |
| `/trajectory` | Target speed interpolated at current trajectory age, using the controller's endpoint/interpolation rules |
| `/gateway_status` | Gateway mode, active and latched fault bitfields and decoded fault names |
| `/planner_status` | Actual active sub-mode and effective speed cap, published by the simulator planner at 10 Hz |
| `/vehicle_command` | Urgent-stop request |

The simulator reports synthetic GNSS FIXED. Deviation is relative to the active
reference reported by EgoState, not an independently estimated localization
error. Speed error is actual minus target. The applied BOUNDARY cap can be lower
than the requested FOLLOW cap. The future hardware planner must also publish
`PlannerStatus`; the dashboard never substitutes its own desired settings or
polls parameter services. Fault bit definitions remain in `GatewayStatus.msg`.

## Lap semantics

Load `start_finish.a/b` from `track.yaml`. The finite segment must intersect the
closed centerline in `track.csv`. Centerline point order determines forward
travel; reversing the finish endpoints does not reverse lap direction. There
are no hard-coded oval coordinates in the implementation.

The timer processes each received pose, independent of the 10 Hz summary timer,
and interpolates the crossing timestamp between adjacent poses. It counts only
forward crossings inside the segment. A completed lap requires travel of at
least half the centerline length since the last accepted crossing, suppressing
repeated counts from finish-line jitter. Distance is accumulated between anchors
at least 0.5 m apart, so stationary GNSS noise cannot accumulate an entire lap.
This is a lap timer, not a route
compliance or anti-shortcut checker.

Timing runs only with fresh gateway AUTO state and map-frame ego poses. A start
within 0.25 m of the segment starts a standing lap at the first eligible pose;
otherwise the first crossing starts a flying lap without incrementing the
count. Waiting in RC is excluded. Last/best lap are NaN until a lap completes.
A backwards or duplicate timestamp, gap over 0.5 s, nonfinite pose, or step over
5 m invalidates the partial lap without discarding completed laps. The next
forward crossing starts a new lap. Returning to RC permits a new standing
start. Restarting the node resets the session.

## Validation

Host tests, including two-lap comparison with the existing closed-loop harness
in both FOLLOW and BOUNDARY:

```bash
python3 -m pytest src/mcq_telemetry/test -q -rs
```

The harness counts Frenet wraparound; the new timer independently uses the
surveyed segment. Tests require equal completed counts and lap-time agreement
within 20 ms (two 100 Hz samples). Geometry tests also cover jitter, reverse and
out-of-segment crossings, timestamp resets, gaps and teleports. The generated
ROS message/QoS test runs under Jazzy and skips on hosts without ROS.

In a sourced Jazzy workspace, install `python3-websocket` and run:

```bash
src/mcq_telemetry/tools/graph_compare.sh
```

This runs the existing 120 m CI graph twice, once without telemetry and once
with telemetry and a WebSocket client subscribing to the union of all layout
topics. It requires every stream before AUTO, at least 20 s in AUTO, no urgent
stops while driving, a summary stream at 10 Hz within 10%, and full-driving-window rates within 5% of baseline for
`ego_state`, `vehicle_command`, and `trajectory`. The 5% bound is an explicit
measurement tolerance, not a claim of exact rate equality. Inspect the reported
rates and tighten `compare_metrics.py --tolerance` when the host is stable.
Reports are `metrics_telemetry_*.json`, `dashboard_websocket.json`, and the
existing graph smoke logs. CI runs this comparison and retains diagnostics.

Before declaring H1 complete, also perform a full run from a second machine
with each imported layout visible, confirm all requested fields update, and
compare against the baseline. The automated WebSocket client tests traffic,
not Foxglove rendering or the real pit network. No ROS build, graph comparison,
or second-machine UI acceptance was performed in the initial implementation
environment because ROS was absent and Docker daemon access was unavailable.
