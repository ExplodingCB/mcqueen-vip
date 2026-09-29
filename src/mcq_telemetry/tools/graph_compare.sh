#!/usr/bin/env bash
# Run from the repository root inside a built/sourced Jazzy workspace.
set -euo pipefail
src/mcq_bringup/test/graph_smoke.sh telemetry_baseline handover_delay:=10.0
python3 src/mcq_telemetry/tools/dashboard_probe.py &
probe=$!
trap 'kill -TERM "$probe" 2>/dev/null || true; wait "$probe" 2>/dev/null || true' EXIT
src/mcq_bringup/test/graph_smoke.sh telemetry_dashboard telemetry:=true handover_delay:=10.0
kill -TERM "$probe" 2>/dev/null || true
wait "$probe"
trap - EXIT
python3 src/mcq_telemetry/tools/compare_metrics.py metrics_telemetry_baseline.json metrics_telemetry_dashboard.json
