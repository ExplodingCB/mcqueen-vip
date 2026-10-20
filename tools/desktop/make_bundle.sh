#!/bin/bash
# Mac side: pack the data the desktop needs into one file, and (optionally) serve it over Tailscale.
#   bash tools/desktop/make_bundle.sh          # build training/handoff/data-bundle.tar and its checksum
#   bash tools/desktop/make_bundle.sh serve    # then serve it on the tailnet address only, port 8780, until Ctrl-C
# Run `serve` only when the desktop is about to download: it is open to every device on the tailnet while it runs.
set -euo pipefail
cd "$(dirname "$0")/../.."
OUT=training/handoff
mkdir -p "$OUT"
FILES="training/data/frames training/data/masks training/data/test_frames training/data/test_masks \
training/perception/runs/laptime-v2/best.json"
for m in training/perception/runs/drive-v5 training/perception/runs/drive-v6; do [ -f "$m/model.pt" ] && FILES="$FILES $m/model.pt"; done
COPYFILE_DISABLE=1 tar cf "$OUT/data-bundle.tar" $FILES
( cd "$OUT" && shasum -a 256 data-bundle.tar > data-bundle.tar.sha256 )
ls -lh "$OUT/data-bundle.tar" | awk '{print "bundle:", $5}'
if [ "${1:-}" = "serve" ]; then
  IP=$(tailscale ip -4 | head -1)
  echo "serving $OUT on http://$IP:8780 (tailnet only). Ctrl-C to stop."
  cd "$OUT" && exec python3 -m http.server 8780 --bind "$IP"
fi
