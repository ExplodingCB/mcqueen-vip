#!/bin/bash
# A short training job whose timings say how much faster the desktop is. Report the output.
#   bash tools/desktop/speedtest.sh
set -euo pipefail
cd "$(dirname "$0")/../.."
export VECLIB_MAXIMUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=training/perception:src/mcq_sim
echo "cpu: $(lscpu | grep 'Model name' | sed 's/.*: *//')  cores: $(nproc)"
START=$(date +%s)
.venv/bin/python training/perception/train_drive.py --run speedtest \
  --init training/perception/runs/drive-v5/model.pt --fine --frames 4000 --val-frames 400 --epochs 2 \
  --workers $(( $(nproc) > 3 ? $(nproc) - 2 : 1 )) \
  --real training/data/masks/labels.csv training/data/test_masks/labels_clean.csv \
  --real-hold t13IbWQsvsE xH4hDjq_n1w fjPo6uzeCyc 2>&1 | grep -E "device|data ready|GB|real footage|epoch|saved"
echo "total wall time: $(( $(date +%s) - START )) s"
echo "for comparison, the MacBook Air: data generation ~8-10 min and ~9-10 min per epoch at 28,000 frames (this test uses 4,000)"
