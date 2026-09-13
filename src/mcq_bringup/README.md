# mcq_bringup

Launch files and parameter sets per kart and per environment.

```
ros2 launch mcq_bringup sim.launch.py                       # simulator, planner, controller on the synthetic oval
ros2 launch mcq_bringup sim.launch.py mode:=BOUNDARY speed_cap:=4.0
ros2 launch mcq_bringup sim.launch.py record:=true bag:=logs/2026-09-12_sim_oval
ros2 run mcq_bringup check_graph.py --distance 120         # PASS once 120 m are driven with no stop request
src/mcq_bringup/test/graph_smoke.sh control                # what CI runs, with rate probes and a top snapshot
```

The simulator's emulated gateway starts in `RC` and hands over to `AUTO` once the heartbeat has been continuous for a second plus `handover_delay` (2 s), the way the real gateway waits for the operator's switch, so the graph has settled before the kart moves. `check_graph.py` counts urgent stop requests only once the kart is moving; a stop request during startup is not an intervention on the kart either, because `AUTO` needs a second of clean heartbeat and the operator's switch. The smoke script starts the checker before the graph: a participant joining mid-run freezes the rclpy nodes for a few hundred milliseconds, which the controller's freshness check treats as the fault it is (docs/02-architecture.md section 4).

Kart launch files (`kart.launch.py` with the gateway bridge, GNSS and IMU drivers and the state estimator) arrive with Phase 1.
