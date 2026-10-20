#!/bin/bash
# Download the data bundle from the Mac over Tailscale and unpack it in the repo.
#   bash tools/desktop/pull_bundle.sh            # the Mac must be serving the bundle (see make_bundle.sh)
# The Mac's tailnet name is below; override with MAC=... if it changes.
set -euo pipefail
cd "$(dirname "$0")/../.."
MAC="${MAC:-chases-macbook-air.tail43fd50.ts.net}"
PORT="${PORT:-8780}"
curl -fL --retry 5 -C - -o /tmp/mcq-data-bundle.tar "http://$MAC:$PORT/data-bundle.tar"
curl -fsL "http://$MAC:$PORT/data-bundle.tar.sha256" > /tmp/mcq-data-bundle.sha256
( cd /tmp && echo "$(cut -d' ' -f1 mcq-data-bundle.sha256)  mcq-data-bundle.tar" | sha256sum -c - )
tar xf /tmp/mcq-data-bundle.tar
echo "unpacked: $(du -sh training/data/frames training/data/test_frames 2>/dev/null | tr '\n' ' ')"
rm -f /tmp/mcq-data-bundle.tar
