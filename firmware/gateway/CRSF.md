# CRSF receiver input

`crsf.c` decodes receiver bytes into `gw_inputs_t`. `gw_crsf_apply()` supplies RC steering, throttle, brake, AUTO and reset controls before `gw_step()`, which still applies actuator limits and decides mode transitions. It must run every 100 Hz tick, including ticks without serial data.

## Board integration

Initialize one `gw_crsf_t` with `gw_crsf_init()` and the flashed `gw_crsf_config_t`. Failure to initialize is a self-test failure. Read the receiver UART without blocking into a bounded buffer, call `gw_crsf_feed(receiver, bytes, count, now_ms)`, then `gw_crsf_apply(receiver, now_ms, inputs)` followed by `gw_step(gateway, inputs)`. Feed and apply run on the same task. UART interrupt code only queues bytes.

Feed consumes at most 512 bytes per call and returns the count consumed. Keep the unread remainder for later ticks; never drain an unbounded queue inside the control loop. Queue overflow must discard the damaged UART batch and report receiver loss through the board's fault path. Timestamps represent monotonic byte arrival, with unsigned 32-bit wraparound. The board must avoid replaying a backlog as fresh controls.

The parser stores at most 64 bytes. It accepts serial sync `0xC8` and broadcast `0x00`, lengths 2 through 62, CRC-8 DVB-S2, and the first 22 payload bytes of type `0x16`. Extra payload fields are ignored. Unknown frames are consumed whole. Partial frames expire after 20 ms; invalid length or CRC slides the buffer one byte to recover framing. Complete packets behind a stalled prefix retain their arrival time. Valid telemetry payloads are never scanned for embedded control packets.

## Receiver configuration

Defaults use CH1 steering, CH2 throttle, CH3 brake, CH5 AUTO, CH6 reset and CH7 link health. Configuration indices are zero-based, distinct and bounded. Steering maps 172/992/1811 to negative limit/zero/positive limit. Throttle and brake map 172 through 1811 to 0 through 1, with endpoint clamping. AUTO and reset are high above 992.

Hold CH7 high during normal transmission. If the receiver supports configured channel output on failsafe, configure CH7 low on loss. The configured inclusive sentinel range defaults to 0 through 992 and can be changed with its channel index to match the actual receiver. Other failsafe channel values are discarded, so a failsafe RC switch or reset cannot become an operator action. A receiver that stops channel output instead is detected by the 100 ms channel timeout. These deadlines are project safety policy requiring bench validation.

Packed CRSF channels have no universal failsafe flag. Centered steering and zero throttle/brake remain valid controls. Telemetry and corrupt packets do not refresh channel freshness. An explicit failsafe or elapsed timeout remains visible for a gateway tick even if healthy traffic resumes in the same read. Receiver loss invalidates AUTO rearming; rearming requires a healthy RC switch position followed by AUTO. Reset also requires a healthy link and cleared stopping faults.

As checked on 2026-09-22, the [TBS specification](https://github.com/tbs-fpv/tbs-crsf-spec/blob/001b4058231c1d6f2288458500293a0f9de0a3de/crsf.md#0x16-rc-channels-packed-payload) describes cut failsafe as channel-packet silence. The [ExpressLRS wire definitions](https://github.com/ExpressLRS/ExpressLRS/blob/a60b68af521d546fd395674d66e83291d035ae61/src/include/crsf_protocol.h) confirm framing, polynomial, channel packing and endpoints. Its [serial sender](https://github.com/ExpressLRS/ExpressLRS/blob/a60b68af521d546fd395674d66e83291d035ae61/src/src/rx-serial/SerialCRSF.cpp) omits unavailable frames. Receiver PWM failsafe settings are not assumed to affect CRSF serial output; verify the actual receiver firmware on the bench.

## Fault injection coverage

Run `ctest --test-dir firmware/gateway/build --output-on-failure` after the README build commands. `crsf` covers wire decoding, split reads, interleaved packets, resynchronization, corrupt CRC, configured failsafe, timeout and bounded work. `crsf_gateway` runs real packet bytes through the state machine.

| docs/04 section 7 | Host test in `test_crsf_gateway.c` | Bench action |
| --- | --- | --- |
| 8 | `item_8_timeout_in_rc_and_auto`, `item_8_configured_failsafe_in_rc_and_auto` | Power off TX in RC and AUTO; capture the receiver output, zero throttle, braking and RC-link fault |
| 11 | `item_11_auto_refused_at_handover_speed` | Simulate speed at/above 3 m/s; flip AUTO and verify refusal until RC/AUTO cycling below the limit |
| 12 | `item_12_auto_refused_with_stale_heartbeat` | Stop CAN commands for at least 50 ms; flip AUTO and verify refusal, including after heartbeat recovery |

`auto_held_across_loss_requires_real_rc_cycle` verifies recovery cannot rearm a held AUTO switch. Bench and low-speed track runs remain required; host coverage does not establish hardware acceptance.
