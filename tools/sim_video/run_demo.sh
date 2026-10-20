#!/bin/bash
# Live demo: a trained model drives the Purdue track in the simulator, with a 3D chase
# camera, the top-down map and the model's own view.
#
#   tools/sim_video/run_demo.sh              # then open http://127.0.0.1:8766/demo
#   tools/sim_video/run_demo.sh my-host.ts.net   # also answer on a tailnet name
#
# For the smoothest demo, stop anything heavy first (training, the lap-time search):
# the simulator runs several times faster than real time on an idle machine and
# crawls on a busy one. The 3D view loads three.js from a CDN, so it needs a network.
cd "$(dirname "$0")/../.." || exit 1
export VECLIB_MAXIMUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=training/perception:src/mcq_sim
exec .venv/bin/python -m mcq_sim view --policy stack --perception driver:create --port 8766 \
  ${1:+--allow-host "$1"}
