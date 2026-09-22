#!/usr/bin/env bash
# Run from a built Jazzy workspace, with a cold-start sim MCAP supplied as $1.
# All generated bags, manifests and negative-control logs stay in $2.
set -euo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
source_bag=$(realpath "$1")
artifacts=$(realpath -m "${2:-$root/replay-artifacts}")
mkdir -p "$artifacts"
baseline=$(mktemp -d /tmp/mcq-replay-reference.XXXXXX)
trap 'rm -rf "$baseline"' EXIT
revision=$(tr -d '\n' < "$root/tools/replay/reference/revision")
# This immutable source revision is independent of the candidate. Only the
# checked-in transport instrumentation patch is applied, never candidate core,
# node behavior or parameters. Updating it is an explicit review decision.
git -C "$root" cat-file -e "$revision^{commit}"
git -C "$root" archive "$revision" src/mcq_control src/mcq_msgs | tar -x -C "$baseline"
(cd "$baseline" && git apply "$root/tools/replay/reference/enable-replay.patch")
(
  cd "$baseline"
  source /opt/ros/jazzy/setup.bash
  colcon build --packages-up-to mcq_control --event-handlers console_direct+ \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
) 2>&1 | tee "$artifacts/reference-build.log"
(
  source /opt/ros/jazzy/setup.bash
  source "$baseline/install/local_setup.bash"
  python3 "$root/tools/replay/replay.py" --bag "$source_bag" \
    --parameters "$baseline/src/mcq_control/config/controller.yaml" \
    --capture "$artifacts/reference" --node-log "$artifacts/reference-node.log"
) 2>&1 | tee "$artifacts/reference-capture.log"
python3 - "$root" "$source_bag" "$artifacts" "$revision" <<'PY'
import hashlib
import json
import subprocess
import sys
from pathlib import Path

root, source, artifacts = map(Path, sys.argv[1:4])
manifest = {
    "baseline_revision": sys.argv[4],
    "candidate_revision": subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip(),
    "source_mcap_sha256": {
        str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(source.rglob("*.mcap"))
    },
    "instrumentation_sha256": hashlib.sha256((root / "tools/replay/reference/enable-replay.patch").read_bytes()).hexdigest(),
    "schedule": "receive order; explicit output header timestamps; each input acknowledged before next event",
}
(artifacts / "provenance.json").write_text(json.dumps(manifest, indent=2) + "\n")
PY
(
  source /opt/ros/jazzy/setup.bash
  source "$root/install/local_setup.bash"
  python3 "$root/tools/replay/replay.py" --bag "$artifacts/reference" \
    --node-log "$artifacts/candidate-node.log"
) 2>&1 | tee "$artifacts/candidate-compare.log"
# Prove that a real YAML gain change is rejected by the independent reference.
python3 - "$root/src/mcq_control/config/controller.yaml" "$artifacts/changed-gain.yaml" <<'PY'
import sys
from pathlib import Path
import yaml
config = yaml.safe_load(Path(sys.argv[1]).read_text())
gains = config["controller"]["ros__parameters"]["longitudinal"]
gains["k_p"] = float(gains["k_p"]) + 1.0
Path(sys.argv[2]).write_text(yaml.safe_dump(config))
PY
set +e
(
  source /opt/ros/jazzy/setup.bash
  source "$root/install/local_setup.bash"
  python3 "$root/tools/replay/replay.py" --bag "$artifacts/reference" \
    --parameters "$artifacts/changed-gain.yaml" --node-log "$artifacts/changed-gain-node.log"
) > "$artifacts/changed-gain-compare.log" 2>&1
mutation_status=$?
set -e
cat "$artifacts/changed-gain-compare.log"
if [ "$mutation_status" -ne 1 ] || ! grep -Eq 'field=(throttle|brake) timestamp=[0-9]+\.[0-9]+' "$artifacts/changed-gain-compare.log"; then
  echo "FAIL: altered longitudinal.k_p must cause a field/timestamp replay divergence"
  exit 1
fi
echo "PASS: baseline matched; deliberate YAML gain change rejected with field and timestamp"
