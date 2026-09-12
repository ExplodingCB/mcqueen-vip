#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi
export PYTHONPATH="/ws/src/mcq_sim:${PYTHONPATH}"
exec "$@"
