#!/usr/bin/env bash
# Start observers and the pose fault injector before driving to avoid rclpy
# discovery stalls. The simulator publishes truth only; the checker provides ego.
set -eu
scenario=${1:-shift}
fixture=$(mktemp -d)
launch_pid=
checker_pid=
cleanup() {
  if [ -n "$launch_pid" ]; then
    kill -INT -- -"$launch_pid" 2>/dev/null || true
    for _ in $(seq 1 20); do
      if ! kill -0 -- -"$launch_pid" 2>/dev/null; then break; fi
      sleep 0.25
    done
    kill -TERM -- -"$launch_pid" 2>/dev/null || true
    sleep 1
    kill -KILL -- -"$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
  if [ -n "$checker_pid" ]; then kill "$checker_pid" 2>/dev/null || true; fi
  rm -rf "$fixture"
}
trap cleanup EXIT
ros2 run mcq_bringup check_geofence.py --prepare --directory "$fixture"
timeout 70 ros2 run mcq_bringup check_geofence.py --directory "$fixture" --scenario "$scenario" &
checker_pid=$!
sleep 3
setsid ros2 launch mcq_bringup sim.launch.py track:="$fixture/original" ego_from_sim:=false mode:=FOLLOW geofence_output:=geofence_raw \
  > "geofence_${scenario}.log" 2>&1 &
launch_pid=$!
set +e
wait "$checker_pid"
result=$?
set -e
cat "geofence_${scenario}.log"
exit "$result"
