#!/usr/bin/env bash
# Runs the Phase 0 graph and the checker, with diagnostics, so a failure in
# CI tells why. Usage: graph_smoke.sh <label> [extra launch args...]
# Environment: RECORD=true records to $BAG; FASTDDS_PROFILE=<xml> selects a
# Fast DDS profile.
set -u
label=$1; shift
bag=${BAG:-/tmp/phase0_bag}
launch_args=("$@")
if [ "${RECORD:-false}" = "true" ]; then
  launch_args+=("record:=true" "bag:=$bag")
fi
if [ -n "${FASTDDS_PROFILE:-}" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE=$FASTDDS_PROFILE
fi
echo "=== graph smoke test: $label (${launch_args[*]:-no args}) RMW=${RMW_IMPLEMENTATION:-default} profile=${FASTRTPS_DEFAULT_PROFILES_FILE:-none}"

# Nothing from a previous run may still be on the graph.
leftovers=$(ros2 node list 2>/dev/null | wc -l)
if [ "$leftovers" -ne 0 ]; then
  echo "leftover nodes before start:"; ros2 node list
  exit 2
fi

# The launch gets its own process group so the whole tree can be signalled.
setsid ros2 launch mcq_bringup sim.launch.py "${launch_args[@]}" > "launch_$label.log" 2>&1 &
launch_pid=$!
sleep 1
# Diagnostics from the first seconds: are the topics flowing at all?
( timeout 12 ros2 topic hz /ego_state --window 100 > "hz_ego_$label.txt" 2>&1 ) &
( timeout 12 ros2 topic hz /vehicle_command --window 100 > "hz_cmd_$label.txt" 2>&1 ) &
( timeout 12 ros2 topic hz /trajectory --window 20 > "hz_traj_$label.txt" 2>&1 ) &
( sleep 6; top -b -n 1 -o %CPU | head -25 > "top_$label.txt" 2>&1; ros2 topic info -v /ego_state > "info_ego_$label.txt" 2>&1; ros2 node list > "nodes_$label.txt" 2>&1 ) &
sleep 3
timeout 120 ros2 run mcq_bringup check_graph.py --distance 120 --speed 3 --timeout 100
rc=$?

# Stop the launch and everything under it, then wait until the graph is empty.
kill -INT -- -"$launch_pid" 2>/dev/null || kill -INT "$launch_pid" 2>/dev/null || true
for _ in $(seq 1 30); do
  if ! kill -0 "$launch_pid" 2>/dev/null; then break; fi
  sleep 0.5
done
kill -TERM -- -"$launch_pid" 2>/dev/null || true
sleep 2
kill -KILL -- -"$launch_pid" 2>/dev/null || true
pkill -KILL -f "ros2 bag record|mcq_sim/lib|controller_node|check_graph" 2>/dev/null || true
for _ in $(seq 1 40); do
  if [ "$(ros2 node list 2>/dev/null | wc -l)" -eq 0 ]; then break; fi
  sleep 0.5
done
left=$(ros2 node list 2>/dev/null | tr '\n' ' ')
echo "nodes left after shutdown: ${left:-none}"
if [ -n "$left" ]; then ps aux | grep -v grep | grep -E "ros2|mcq|controller" || true; fi

echo "--- launch log"; cat "launch_$label.log"
for f in hz_ego hz_cmd hz_traj top info_ego nodes; do
  echo "--- $f"; cat "${f}_$label.txt" 2>/dev/null | tail -30
done
echo "=== $label result: $rc"
exit $rc
