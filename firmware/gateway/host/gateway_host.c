// The gateway core on a Linux host, speaking real SocketCAN.
//
// Runs gw_step() at a fixed rate against a CAN interface (vcan0 on a laptop,
// a USB adapter on the bench), emulates just enough vehicle for the speed and
// power limits to mean something, and takes fault-injection commands on stdin.
// With the ROS 2 graph on the other side of the bus this is the bench rig
// without hardware: every software item of docs/04-safety.md section 7 can be
// run here, and the same binary drives a real actuator bench later.
//
//   sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
//   ./gateway_host --interface vcan0
//
// stdin commands, one per line:
//   auto on|off        transmitter mode switch
//   rc on|off          RC link (off = failsafe)
//   estop on|off       remote e-stop
//   reset              operator reset on the transmitter
//   manual-reset       physical reset on the kart
//   selftest ok|fail   result of the power-on self test (applies from INIT)
//   rcs on|off         race control allows AUTO
//   steer <rad>        transmitter steering
//   throttle <0..1>    transmitter throttle
//   brake <0..1>       transmitter brake
//   speed <m/s>|auto   override the emulated speed, or return to emulation
//   power <W>|auto     override the reported motor power
//   quit

#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <math.h>
#include <net/if.h>
#include <poll.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "gateway/config.h"
#include "gateway/state_machine.h"
#include "mcqueen.h"

typedef struct
{
  bool speed_override;
  float speed;
  bool power_override;
  float power;
  float steer;  // emulated actuator position
} vehicle_emulation_t;

static uint32_t now_ms(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static int open_can(const char * interface)
{
  int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    perror("socket");
    return -1;
  }
  struct ifreq ifr;
  memset(&ifr, 0, sizeof(ifr));
  strncpy(ifr.ifr_name, interface, IFNAMSIZ - 1);
  if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    perror(interface);
    close(fd);
    return -1;
  }
  struct sockaddr_can addr;
  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("bind");
    close(fd);
    return -1;
  }
  fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
  return fd;
}

static void send_frame(int fd, uint32_t id, const uint8_t data[8])
{
  struct can_frame f;
  memset(&f, 0, sizeof(f));
  f.can_id = id;
  f.len = 8;
  memcpy(f.data, data, 8);
  if (write(fd, &f, sizeof(f)) != (ssize_t)sizeof(f)) {
    perror("write");
  }
}

static void send_telemetry(
  int fd, const gw_t * gw, const gw_inputs_t * in, const vehicle_emulation_t * veh)
{
  uint8_t data[8];
  struct mcqueen_gateway_status_t st;
  memset(&st, 0, sizeof(st));
  st.mode = (uint8_t)gw->out.mode;
  st.tx_auto_switch = in->tx_auto_switch;
  st.rc_link_ok = in->rc_link_ok;
  st.remote_estop = in->remote_estop;
  st.contactor_closed = gw->out.contactor_closed;
  st.fault_flags = gw->out.faults;
  st.heartbeat_echo = gw->out.heartbeat_echo;
  st.frames_rejected = gw->out.frames_rejected;
  st.speed = (uint16_t)lroundf(fmaxf(in->speed, 0.0f) / 0.01f);
  mcqueen_gateway_status_pack(data, &st, 8);
  send_frame(fd, MCQUEEN_GATEWAY_STATUS_FRAME_ID, data);

  struct mcqueen_gateway_steering_t steer;
  memset(&steer, 0, sizeof(steer));
  steer.steering_angle = (int16_t)lroundf(veh->steer / 0.0001f);
  steer.brake_pressure = (uint16_t)lroundf(gw->out.brake / 0.001f);
  steer.throttle_applied = (uint8_t)lroundf(gw->out.throttle / 0.016f);
  steer.steering_cmd_applied = (int16_t)lroundf(gw->out.steering / 0.0001f);
  mcqueen_gateway_steering_pack(data, &steer, 8);
  send_frame(fd, MCQUEEN_GATEWAY_STEERING_FRAME_ID, data);

  struct mcqueen_gateway_wheels_t wh;
  memset(&wh, 0, sizeof(wh));
  uint16_t ws = (uint16_t)lroundf(fmaxf(in->speed, 0.0f) / 0.01f);
  wh.wheel_speed_fl = wh.wheel_speed_fr = wh.wheel_speed_rl = wh.wheel_speed_rr = ws;
  mcqueen_gateway_wheels_pack(data, &wh, 8);
  send_frame(fd, MCQUEEN_GATEWAY_WHEELS_FRAME_ID, data);

  struct mcqueen_gateway_motor_t mo;
  memset(&mo, 0, sizeof(mo));
  mo.motor_power = (int16_t)lroundf(fminf(fmaxf(in->motor_power, -32768.0f), 32767.0f));
  mo.battery_voltage = (uint16_t)lroundf(96.0f / 0.01f);
  mcqueen_gateway_motor_pack(data, &mo, 8);
  send_frame(fd, MCQUEEN_GATEWAY_MOTOR_FRAME_ID, data);
}

static void send_limits(int fd)
{
  uint8_t data[8];
  struct mcqueen_gateway_limits_t lim;
  memset(&lim, 0, sizeof(lim));
  lim.steer_angle_max = (uint16_t)lroundf(GW_STEER_ANGLE_MAX_RAD / 0.0001f);
  lim.steer_rate_max = (uint16_t)lroundf(GW_STEER_RATE_MAX_RAD_S / 0.001f);
  lim.speed_cap = (uint16_t)lroundf(GW_SPEED_CAP_M_S / 0.01f);
  lim.power_cap = (uint16_t)lroundf(GW_POWER_CAP_W);
  mcqueen_gateway_limits_pack(data, &lim, 8);
  send_frame(fd, MCQUEEN_GATEWAY_LIMITS_FRAME_ID, data);
}

static bool parse_on_off(const char * arg, bool * out)
{
  if (arg == NULL) {
    return false;
  }
  if (strcmp(arg, "on") == 0 || strcmp(arg, "ok") == 0) {
    *out = true;
    return true;
  }
  if (strcmp(arg, "off") == 0 || strcmp(arg, "fail") == 0) {
    *out = false;
    return true;
  }
  return false;
}

// Returns false on "quit".
static bool handle_line(char * line, gw_inputs_t * in, vehicle_emulation_t * veh)
{
  char * cmd = strtok(line, " \t\r\n");
  char * arg = strtok(NULL, " \t\r\n");
  if (cmd == NULL) {
    return true;
  }
  bool flag = false;
  if (strcmp(cmd, "quit") == 0) {
    return false;
  } else if (strcmp(cmd, "auto") == 0 && parse_on_off(arg, &flag)) {
    in->tx_auto_switch = flag;
  } else if (strcmp(cmd, "rc") == 0 && parse_on_off(arg, &flag)) {
    in->rc_link_ok = flag;
  } else if (strcmp(cmd, "estop") == 0 && parse_on_off(arg, &flag)) {
    in->remote_estop = flag;
  } else if (strcmp(cmd, "selftest") == 0 && parse_on_off(arg, &flag)) {
    in->self_test_ok = flag;
  } else if (strcmp(cmd, "rcs") == 0 && parse_on_off(arg, &flag)) {
    in->rcs_auto_allowed = flag;
  } else if (strcmp(cmd, "reset") == 0) {
    in->tx_reset = true;  // held for one tick, cleared by the loop
  } else if (strcmp(cmd, "manual-reset") == 0) {
    in->manual_reset = true;
  } else if (strcmp(cmd, "steer") == 0 && arg) {
    in->rc_steering = strtof(arg, NULL);
  } else if (strcmp(cmd, "throttle") == 0 && arg) {
    in->rc_throttle = strtof(arg, NULL);
  } else if (strcmp(cmd, "brake") == 0 && arg) {
    in->rc_brake = strtof(arg, NULL);
  } else if (strcmp(cmd, "speed") == 0 && arg) {
    veh->speed_override = strcmp(arg, "auto") != 0;
    if (veh->speed_override) {
      veh->speed = strtof(arg, NULL);
    }
  } else if (strcmp(cmd, "power") == 0 && arg) {
    veh->power_override = strcmp(arg, "auto") != 0;
    if (veh->power_override) {
      veh->power = strtof(arg, NULL);
    }
  } else {
    fprintf(stderr, "unknown command: %s\n", cmd);
  }
  return true;
}

static void emulate_vehicle(vehicle_emulation_t * veh, const gw_t * gw, float dt_s)
{
  // First-order steering actuator and a crude longitudinal model, the same
  // shape as mcq_sim's kart so the numbers line up.
  if (gw->out.steering_enabled) {
    veh->steer += (gw->out.steering - veh->steer) * fminf(dt_s / 0.08f, 1.0f);
  }
  if (!veh->speed_override) {
    float accel = gw->out.throttle * 3.0f * fmaxf(0.0f, 1.0f - veh->speed / 12.0f) -
                  gw->out.brake * 6.0f - 0.3f;
    veh->speed = fmaxf(0.0f, veh->speed + accel * dt_s);
  }
  if (!veh->power_override) {
    veh->power = gw->out.throttle * 6000.0f;
  }
}

int main(int argc, char ** argv)
{
  const char * interface = "vcan0";
  double hz = 1000.0;
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--interface") == 0 && i + 1 < argc) {
      interface = argv[++i];
    } else if (strcmp(argv[i], "--hz") == 0 && i + 1 < argc) {
      hz = atof(argv[++i]);
    } else {
      fprintf(stderr, "usage: %s [--interface vcan0] [--hz 1000]\n", argv[0]);
      return 2;
    }
  }
  int fd = open_can(interface);
  if (fd < 0) {
    return 1;
  }

  gw_t gw;
  gw_inputs_t in;
  vehicle_emulation_t veh;
  memset(&in, 0, sizeof(in));
  memset(&veh, 0, sizeof(veh));
  in.self_test_ok = true;
  in.rc_link_ok = true;
  in.rcs_auto_allowed = true;
  gw_init(&gw, now_ms());
  gw_mode_t last_mode = GW_MODE_INIT;
  fprintf(
    stderr, "gateway_host on %s at %.0f Hz; mode %s\n", interface, hz, gw_mode_name(last_mode));

  const uint32_t tick_ns = (uint32_t)(1e9 / hz);
  struct timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);
  uint32_t last_tx_ms = 0;
  uint32_t last_limits_ms = 0;
  char line[128];
  size_t line_len = 0;
  const float dt_s = (float)(1.0 / hz);

  for (;;) {
    // stdin, non-blocking, line at a time.
    struct pollfd pfd = {.fd = STDIN_FILENO, .events = POLLIN};
    while (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
      char c;
      ssize_t r = read(STDIN_FILENO, &c, 1);
      if (r <= 0) {
        pfd.revents = 0;
        break;
      }
      if (c == '\n' || line_len + 1 >= sizeof(line)) {
        line[line_len] = '\0';
        line_len = 0;
        if (!handle_line(line, &in, &veh)) {
          close(fd);
          return 0;
        }
      } else {
        line[line_len++] = c;
      }
    }

    // CAN in: the newest JETSON_COMMAND this tick.
    in.command_received = false;
    struct can_frame f;
    while (read(fd, &f, sizeof(f)) == (ssize_t)sizeof(f)) {
      if ((f.can_id & CAN_SFF_MASK) == MCQUEEN_JETSON_COMMAND_FRAME_ID && f.len == 8) {
        memcpy(in.command_frame, f.data, 8);
        in.command_received = true;
      }
    }

    in.now_ms = now_ms();
    in.speed = veh.speed;
    in.motor_power = veh.power;
    gw_step(&gw, &in);
    in.tx_reset = false;
    in.manual_reset = false;
    emulate_vehicle(&veh, &gw, dt_s);

    if (gw.out.mode != last_mode) {
      fprintf(
        stderr, "[%u ms] %s -> %s (faults 0x%04x)\n", in.now_ms, gw_mode_name(last_mode),
        gw_mode_name(gw.out.mode), gw.out.faults);
      last_mode = gw.out.mode;
    }
    if (in.now_ms - last_tx_ms >= 10u) {
      last_tx_ms = in.now_ms;
      send_telemetry(fd, &gw, &in, &veh);
    }
    if (in.now_ms - last_limits_ms >= 1000u) {
      last_limits_ms = in.now_ms;
      send_limits(fd);
    }

    next.tv_nsec += tick_ns;
    while (next.tv_nsec >= 1000000000L) {
      next.tv_nsec -= 1000000000L;
      next.tv_sec += 1;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
  }
}
