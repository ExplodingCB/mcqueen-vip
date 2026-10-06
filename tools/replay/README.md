# Deterministic node replay

`replay.py` feeds MCAP inputs into a real ROS 2 node, captures its outputs, and compares every output field against the recorded reference. It contains no controller or planner equations. Run inside the built Jazzy workspace:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/replay/replay.py --bag /path/to/reference
```

A mismatch exits 1 and names the topic, field, expected and actual values, tolerance, and original output header timestamp. `controller.yaml` permits absolute error of `0.00001` for steering, throttle and brake. Every other field is exact, including heartbeat, header timestamps, enable flags and urgent stop. Missing messages, timeouts, extra outputs, invalid tolerances and nonfinite numbers fail.

## Reference independence

```bash
tools/replay/ci.sh /tmp/phase0_bag ./replay-artifacts
```

CI records the cold-start simulator graph, then rebuilds the controller from the full git commit in `reference/revision`. The frozen `reference/enable-replay.patch` adds transport instrumentation only. The baseline keeps its own node behavior, C core and YAML gains. Candidate sources are never copied into the baseline.

The baseline node processes the recorded input schedule and writes a new reference MCAP. The candidate compares against those recorded baseline outputs. CI then adds `1.0` to the actual controller YAML's `longitudinal.k_p`, launches the candidate again, and requires a throttle or brake divergence with a timestamp. A crash or startup timeout does not count as successful mutation detection. Bags and logs are CI artifacts; the manifest records source MCAP hashes, baseline and candidate commits, and instrumentation hash.

An intentional behavior change needs a reviewed baseline revision update, with a matching instrumentation patch if the new baseline lacks the protocol. Do not regenerate expected outputs using the candidate to make a failing comparison pass. The pinned revision deliberately remains stable across PRs; updating it is a visible source change.

## Ordering contract

MCAP receive timestamps determine event order, preserving stored order for ties. Each input is published reliably and acknowledged after its callback updates node state. Every distinct recorded output header timestamp drives one explicit tick. Replay disables the wall timer; the production timer and best-effort QoS remain the default. Outputs sharing a tick must share its header timestamp.

A live asynchronous bag does not capture the controller's callback order. Direct comparison is available, but exact reproduction of that live output is not guaranteed. CI therefore compares a normalized reference recorded through this deterministic protocol, using the live bag's input data and tick schedule. This checks node behavior; it does not validate real-time scheduling or transport latency. Clock resets require separate sessions.

## Another node

Add a YAML topic map specifying command, node name, parameter file, input topic aliases and output tolerances. Types come from bag metadata. The node must implement the same step protocol, using `ReplayStep` in `src/mcq_control/src/replay_step.hpp` or an equivalent adapter: acknowledge each input's Header on `replay/received` with its relative topic in `frame_id`, and run its ordinary update on each Header received on `replay/tick`. All topics must be relative so the driver can isolate the node in its own namespace. No new bag reader, scheduler, capture path or comparator is needed.
