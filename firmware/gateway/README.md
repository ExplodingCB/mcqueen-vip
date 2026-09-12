# Safety gateway firmware

Portable core of the gateway described in [docs/04-safety.md](../../docs/04-safety.md): the mode state machine, the heartbeat check, the per-frame limits, the CRC and the `JETSON_COMMAND` frame codec. Plain C11, no allocation, no hardware headers, so every fault-injection item that is software runs on the host.

```
cmake -S firmware/gateway -B firmware/gateway/build
cmake --build firmware/gateway/build
ctest --test-dir firmware/gateway/build --output-on-failure
```

| File | What it holds |
| --- | --- |
| `include/gateway/config.h` | Limits and timings flashed with the firmware, first guesses until Phase 1 measures them |
| `include/gateway/faults.h` | Fault bit positions, shared with `mcq_msgs/GatewayStatus` and the DBC |
| `src/state_machine.c` | `INIT`, `RC`, `AUTO`, `URGENT_STOP`, `DRIVETRAIN_OFF`, `FAULT` and the outputs in each |
| `src/heartbeat.c` | Counter continuity and timeout |
| `src/limits.c` | Angle, rate, throttle ramp, brake-throttle, speed and power limiters |
| `src/command.c` | Frame unpack, pack, range checks |
| `test/test_state_machine.c` | The fault-injection list from the safety document, run against a simulated Jetson |
| `tools/gen_can_vectors.py` | Regenerates `test/can_vectors.h` from the DBC so the codec cannot drift from it |

`host/gateway_host` is the same core running on a Linux machine against a SocketCAN interface, with a crude vehicle emulation and fault-injection commands on stdin (`auto on`, `rc off`, `estop on`, `reset`, `speed 4`, and so on; see the file header). With `vcan0` up it is the bench rig without hardware: the ROS 2 graph's `gateway_bridge` talks to it exactly as it will talk to the board, and `host/test_gateway_host.py` drives the mode transitions over the bus from Python (CI runs it; it skips without `vcan0`).

```
sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
./firmware/gateway/build/gateway_host --interface vcan0
python -m pytest -q firmware/gateway/host
```

Board support (CAN peripheral, CRSF receiver parsing, encoder and actuator drivers, the main loop calling `gw_step` at a fixed rate) is added next to this core once the board is chosen (STM32H7 or Teensy 4.1 carrier, or a comma panda). Keep this directory free of hardware headers.

Two decisions made in code that the safety document did not spell out: the heartbeat accepts a counter advance of 1 to 5 so a single lost or corrupted frame is dropped rather than ending the session (a frozen or replayed counter is still rejected), and re-entering `AUTO` after any exit requires the transmitter switch to pass through `RC` first, so a switch left in the `AUTO` position never re-arms the kart by itself.
