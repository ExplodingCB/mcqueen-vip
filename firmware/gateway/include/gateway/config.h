// Limits and timings flashed with the firmware. Readable over CAN
// (GATEWAY_LIMITS frame), never writable from the Jetson.
//
// Every value here is a first guess to be replaced by a measurement during the
// drive-by-wire phase (docs/05-roadmap.md, Phase 1). Units are SI, angles are
// radians, times are milliseconds.
#ifndef GATEWAY_CONFIG_H
#define GATEWAY_CONFIG_H

// Steering, at the front wheels.
#define GW_STEER_ANGLE_MAX_RAD 0.45f   // mechanical range minus margin (about 26 deg)
#define GW_STEER_RATE_MAX_RAD_S 3.14f  // 180 deg/s actuator rating

// Throttle and brake, normalized 0..1.
#define GW_THROTTLE_RAMP_PER_S 2.0f     // 0 to full in 0.5 s
#define GW_RC_THROTTLE_CAP 1.0f         // cap applied to transmitter throttle
#define GW_BRAKE_THROTTLE_CUTOFF 0.10f  // brake above this forces throttle to 0
#define GW_URGENT_BRAKE_RAMP_MS 300u    // ramp to full brake over this time

// Speed and power.
#define GW_SPEED_CAP_M_S 5.0f         // stepped up per docs/04-safety.md section 8
#define GW_SPEED_CAP_MARGIN_M_S 1.0f  // cap plus margin triggers URGENT_STOP
#define GW_POWER_CAP_W 15000.0f       // AKS rule, measured before the motor controller
#define GW_HANDOVER_SPEED_M_S 3.0f    // AUTO entry refused above this
#define GW_STEER_HOLD_SPEED_M_S 3.0f  // URGENT_STOP holds steering above this
#define GW_STANDSTILL_SPEED_M_S 0.2f
#define GW_STANDSTILL_HOLD_MS 500u  // speed must stay below standstill this long

// Heartbeat.
#define GW_HEARTBEAT_TIMEOUT_MS 50u       // no accepted frame for this long: URGENT_STOP
#define GW_HEARTBEAT_CONTINUOUS_MS 1000u  // required before AUTO entry
#define GW_HEARTBEAT_MAX_SKIP 5u          // accepted counter advance is 1..this

// Urgent stop reset.
#define GW_URGENT_RESET_TIMEOUT_MS 30000u  // standstill without operator reset: DRIVETRAIN_OFF

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif  // GATEWAY_CONFIG_H
