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
ros2 launch mcq_bringup sim.launch.py "${launch_args[@]}" > "launch_$label.log" 2>&1 &
launch_pid=$!
sleep 4
# Diagnostics run alongside the checker.
( timeout 8 ros2 topic hz /ego_state --window 100 > "hz_ego_$label.txt" 2>&1 ) &
( timeout 8 ros2 topic hz /vehicle_command --window 100 > "hz_cmd_$label.txt" 2>&1 ) &
( timeout 8 ros2 topic hz /trajectory --window 20 > "hz_traj_$label.txt" 2>&1 ) &
( sleep 3; top -b -n 1 -o %CPU | head -25 > "top_$label.txt" 2>&1; ros2 topic info -v /ego_state > "info_ego_$label.txt" 2>&1; ros2 node list > "nodes_$label.txt" 2>&1 ) &
timeout 120 ros2 run mcq_bringup check_graph.py --distance 120 --speed 3 --timeout 100
rc=$?
kill -INT $launch_pid 2>/dev/null || true
sleep 5
echo "--- launch log"; cat "launch_$label.log"
for f in hz_ego hz_cmd hz_traj top info_ego nodes; do
  echo "--- $f"; cat "${f}_$label.txt" 2>/dev/null | tail -30
done
echo "=== $label result: $rc"
exit $rc
