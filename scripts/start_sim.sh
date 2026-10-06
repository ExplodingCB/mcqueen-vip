#!/usr/bin/env bash
# Build and start the complete Phase 0 ROS graph on Ubuntu 24.04.
set -euo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repo_root"

usage() {
  cat <<'EOF'
Usage: scripts/start_sim.sh [options]

Build and launch the simulator, planner, controller, pit telemetry and Foxglove
bridge. Docker is the default; use --native with an installed ROS 2 Jazzy setup.

Options:
  --native                 Use the host ROS 2 Jazzy installation
  --docker                 Use the project's Jazzy Docker image (default)
  --check                  Run the 120 m graph smoke test, then exit
  --no-build               Reuse the existing colcon install space
  --no-telemetry           Omit pit telemetry and Foxglove bridge
  --track DIR              Track directory containing track.csv and track.yaml
  --mode FOLLOW|BOUNDARY   Planner mode (default: FOLLOW)
  --speed-cap MPS          Planner speed cap (default: 5.0)
  --handover-delay SEC     Simulated AUTO handover delay (default: 60)
  --record                 Record all topics to an MCAP bag under logs/
  --help                   Show this help

Stop an interactive run with Ctrl-C. Foxglove connects to ws://localhost:8765.
EOF
}

backend=docker
check=false
build=true
telemetry=true
record=false
track=
mode=FOLLOW
speed_cap=5.0
handover_delay=60

while (($#)); do
  case "$1" in
    --native|--docker) backend=${1#--} ;;
    --check) check=true ;;
    --no-build) build=false ;;
    --no-telemetry) telemetry=false ;;
    --record) record=true ;;
    --track|--mode|--speed-cap|--handover-delay)
      option=$1
      (($# >= 2)) || { echo "Missing value for $option" >&2; exit 2; }
      case "$option" in
        --track) track=$2 ;;
        --mode) mode=$2 ;;
        --speed-cap) speed_cap=$2 ;;
        --handover-delay) handover_delay=$2 ;;
      esac
      shift ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

[[ $mode == FOLLOW || $mode == BOUNDARY ]] || { echo "Mode must be FOLLOW or BOUNDARY" >&2; exit 2; }
number='^[0-9]+([.][0-9]+)?$'
[[ $speed_cap =~ $number && $handover_delay =~ $number ]] || {
  echo "Speed cap and handover delay must be nonnegative numbers" >&2; exit 2;
}

if [[ -n $track ]]; then
  track=$(realpath -e -- "$track")
  [[ -f $track/track.csv && -f $track/track.yaml ]] || {
    echo "Track needs track.csv and track.yaml: $track" >&2; exit 2;
  }
fi

if [[ $backend == docker && ${MCQ_SIM_IN_CONTAINER:-0} != 1 ]]; then
  command -v docker >/dev/null || { echo "Docker is required; install it or use --native." >&2; exit 1; }
  docker_track=$track
  extra_mount=()
  if [[ -n $track ]]; then
    if [[ $track == "$repo_root"/* ]]; then
      docker_track="/ws/${track#"$repo_root"/}"
    else
      docker_track=/mcq-track
      extra_mount=(-v "$track:/mcq-track:ro")
    fi
  fi
  args=(--native --mode "$mode" --speed-cap "$speed_cap" --handover-delay "$handover_delay")
  [[ -z $docker_track ]] || args+=(--track "$docker_track")
  [[ $check == false ]] || args+=(--check)
  [[ $build == true ]] || args+=(--no-build)
  [[ $telemetry == true ]] || args+=(--no-telemetry)
  [[ $record == false ]] || args+=(--record)
  echo "Building the Ubuntu 24.04 / ROS 2 Jazzy development image..."
  docker build -t mcqueen-x86 -f "$repo_root/docker/x86/Dockerfile" "$repo_root"
  tty_args=()
  [[ -t 0 && -t 1 ]] && tty_args=(-it)
  exec docker run --rm --init "${tty_args[@]}" --network host --shm-size=1g \
    --user "$(id -u):$(id -g)" -e HOME=/tmp -e MCQ_SIM_IN_CONTAINER=1 \
    -v "$repo_root:/ws" "${extra_mount[@]}" -w /ws \
    mcqueen-x86 /ws/scripts/start_sim.sh "${args[@]}"
fi

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is missing. Install it or run without --native to use Docker." >&2
  exit 1
fi
# Jazzy's generated setup scripts reference optional unset environment variables.
set +u
source /opt/ros/jazzy/setup.bash
set -u
command -v colcon >/dev/null || { echo "colcon is missing (install python3-colcon-common-extensions)." >&2; exit 1; }

if [[ $build == true ]]; then
  echo "Building the ROS 2 workspace..."
  colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
fi
[[ -f install/setup.bash ]] || { echo "No install/setup.bash; rerun without --no-build." >&2; exit 1; }
set +u
source install/setup.bash
set -u

launch_args=("mode:=$mode" "speed_cap:=$speed_cap" "telemetry:=$telemetry" "handover_delay:=$handover_delay")
[[ -z $track ]] || launch_args+=("track:=$track")
if [[ $record == true ]]; then
  mkdir -p logs
  bag="$repo_root/logs/sim_$(date +%Y-%m-%d_%H%M%S)"
  launch_args+=("record:=true" "bag:=$bag")
  echo "Recording MCAP bag to $bag"
fi

if [[ $check == true ]]; then
  # The existing CI smoke test starts its checker before the graph, avoiding a
  # late DDS participant that can interrupt the controller during a lap.
  mkdir -p logs/sim-check
  cd logs/sim-check
  exec "$repo_root/src/mcq_bringup/test/graph_smoke.sh" manual "${launch_args[@]}"
fi

echo "Starting simulator, planner and controller (telemetry: $telemetry)."
if [[ $telemetry == true ]]; then
  echo "Foxglove: ws://localhost:8765; connect before AUTO (after $handover_delay s)."
fi
exec ros2 launch mcq_bringup sim.launch.py "${launch_args[@]}"
