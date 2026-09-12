// The gateway mode state machine from docs/04-safety.md section 2.
//
// gw_step() is called once per control tick with everything the firmware has
// read from its inputs (transmitter, e-stop relay, sensors, the latest CAN
// command frame) and fills the actuator outputs. It has no hardware
// dependencies so the whole fault-injection list runs on the host.
//
// Remote e-stop: the wireless e-stop relay sits physically in the contactor
// circuit, so drivetrain power is already gone when the gateway sees it. The
// state machine mirrors that (contactor output opens immediately), runs the
// urgent-stop braking ramp, and moves to DRIVETRAIN_OFF at standstill.
#ifndef GATEWAY_STATE_MACHINE_H
#define GATEWAY_STATE_MACHINE_H

#include <stdbool.h>
#include <stdint.h>

#include "gateway/command.h"
#include "gateway/faults.h"
#include "gateway/heartbeat.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  GW_MODE_INIT = 0,
  GW_MODE_RC = 1,
  GW_MODE_AUTO = 2,
  GW_MODE_URGENT_STOP = 3,
  GW_MODE_DRIVETRAIN_OFF = 4,
  GW_MODE_FAULT = 5,
} gw_mode_t;

typedef struct
{
  uint32_t now_ms;
  bool self_test_ok;      // result of the power-on self test
  bool tx_auto_switch;    // transmitter mode switch in the AUTO position
  bool tx_reset;          // transmitter reset control (level; edge detected inside)
  bool manual_reset;      // physical reset on the kart (level; edge detected inside)
  bool rc_link_ok;        // ELRS receiver not in failsafe
  bool remote_estop;      // wireless e-stop asserted
  bool rcs_auto_allowed;  // race control allows autonomous mode (true outside events)
  float rc_steering;      // transmitter steering, rad at the wheels
  float rc_throttle;      // 0..1
  float rc_brake;         // 0..1
  float speed;            // m/s from the wheel sensors
  float motor_power;      // W before the motor controller
  bool command_received;  // a JETSON_COMMAND frame arrived since the last tick
  uint8_t command_frame[GW_CMD_FRAME_LEN];
} gw_inputs_t;

typedef struct
{
  gw_mode_t mode;
  float steering;         // rad target for the steering actuator
  bool steering_enabled;  // false: actuator released (INIT, DRIVETRAIN_OFF, FAULT)
  float throttle;         // 0..1
  float brake;            // 0..1
  bool contactor_closed;
  gw_faults_t faults;   // active this tick
  gw_faults_t latched;  // every fault since the last operator reset
  uint16_t heartbeat_echo;
  uint8_t frames_rejected;
} gw_outputs_t;

typedef struct
{
  gw_mode_t mode;
  gw_outputs_t out;
  gw_heartbeat_t heartbeat;
  gw_command_t cmd;  // last accepted command
  bool cmd_valid;
  uint32_t last_tick_ms;
  bool auto_switch_armed;  // switch seen in RC since the last AUTO exit
  bool prev_tx_reset;
  bool prev_manual_reset;
  uint32_t urgent_entry_ms;
  float urgent_brake_start;
  bool urgent_from_estop;
  uint32_t standstill_since_ms;
  bool standstill;
  uint32_t standstill_reached_ms;
} gw_t;

void gw_init(gw_t * gw, uint32_t now_ms);
void gw_step(gw_t * gw, const gw_inputs_t * in);
const char * gw_mode_name(gw_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif  // GATEWAY_STATE_MACHINE_H
