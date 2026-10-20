#!/bin/bash
# Pre-show check: start the demo, drive one full lap with the demo's driver (drive-v5) through the same API the page uses,
# and report whether it finished, how fast it ran, and whether the machine is quiet enough.
#   tools/sim_video/demo_check.sh            # about 1 to 3 minutes; DEMO_MODEL=drive-v3 to check another driver
cd "$(dirname "$0")/../.." || exit 1
export VECLIB_MAXIMUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=training/perception:src/mcq_sim
PORT=8799
.venv/bin/python -m mcq_sim view --policy stack --perception driver:create --port $PORT > /tmp/mcq-demo-check.log 2>&1 &
SERVER=$!
trap 'kill $SERVER 2>/dev/null' EXIT
for i in $(seq 1 60); do curl -s -o /dev/null localhost:$PORT/api/demo/info && break; sleep 1; done
LOAD=$(uptime | sed 's/.*load averages*: *//' | cut -d' ' -f1 | tr -d ,)
.venv/bin/python - "$PORT" "$LOAD" <<'PY'
import json, sys, time, urllib.request
port, load = sys.argv[1], float(sys.argv[2])
def post(path, body):
    req = urllib.request.Request(f"http://localhost:{port}{path}", json.dumps(body).encode(), {"Content-Type": "application/json"})
    return json.load(urllib.request.urlopen(req, timeout=120))
info = json.load(urllib.request.urlopen(f"http://localhost:{port}/api/demo/info"))
import os
model = os.environ.get("DEMO_MODEL", "drive-v5")  # the driver the demo opens with
assert model in info["models"], model
post("/api/control", {"action": "demo_model", "name": model}); post("/api/control", {"action": "speed", "value": 25})
post("/api/control", {"action": "reset"}); post("/api/control", {"action": "play"})
t0 = time.time(); s = post("/api/demo/tick", {"steps": 12}); first = time.time() - t0; n = 1
while s["running"] and time.time() - t0 < 600:
    s = post("/api/demo/tick", {"steps": 12}); n += 1
wall = time.time() - t0
ratio = s["t"] / wall
print(f"driver {model}, no cap")
print(f"result: {'LAP COMPLETE' if s['finished'] and not s['reason'] else 'STOPPED: ' + str(s['reason'])} in {s['t']:.1f} s of simulated time, closest edge {s['clearance']:.2f} m, "
      f"{s['counters']['corrections']} safety corrections")
print(f"speed: {ratio:.1f}x real time ({wall:.0f} s wall for {s['t']:.0f} s simulated); machine load {load:.1f}")
if ratio < 1.2: print("WARNING: slower than real time. Quit training jobs and other heavy apps before presenting.")
elif load > 6: print("note: the machine is busy; it may feel smoother once other jobs stop.")
print("READY" if s["finished"] and not s["reason"] and ratio >= 1.2 else "NOT READY")
PY
