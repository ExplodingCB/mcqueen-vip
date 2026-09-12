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

Board support (CAN peripheral, CRSF receiver parsing, encoder and actuator drivers, the main loop calling `gw_step` at a fixed rate) is added next to this core once the board is chosen (STM32H7 or Teensy 4.1 carrier, or a comma panda). Keep this directory free of hardware headers.

Two decisions made in code that the safety document did not spell out: the heartbeat accepts a counter advance of 1 to 5 so a single lost or corrupted frame is dropped rather than ending the session (a frozen or replayed counter is still rejected), and re-entering `AUTO` after any exit requires the transmitter switch to pass through `RC` first, so a switch left in the `AUTO` position never re-arms the kart by itself.
