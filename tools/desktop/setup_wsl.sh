#!/bin/bash
# One-time setup of the desktop (WSL2 Ubuntu) for the heavy runs. Safe to rerun.
#   bash tools/desktop/setup_wsl.sh
# See docs/14-desktop-handoff.md for what this is for and what to do next.
set -euo pipefail
cd "$(dirname "$0")/../.."
sudo apt-get update -y && sudo apt-get install -y build-essential cmake git ffmpeg python3-dev curl
command -v uv >/dev/null || { curl -LsSf https://astral.sh/uv/install.sh | sh; export PATH="$HOME/.local/bin:$PATH"; }
[ -d .venv ] || uv venv --python 3.12 .venv
# the RTX 5080 (Blackwell) needs a PyTorch build for CUDA 12.8 or newer
uv pip install --python .venv/bin/python torch --index-url https://download.pytorch.org/whl/cu128
uv pip install --python .venv/bin/python numpy scipy pyyaml pillow matplotlib cma pytest ruff imagehash
.venv/bin/python - <<'PY'
import torch
print("torch", torch.__version__, "| cuda available:", torch.cuda.is_available())
if torch.cuda.is_available():
    print("gpu:", torch.cuda.get_device_name(0), "|", round(torch.cuda.get_device_properties(0).total_memory / 1e9, 1), "GB")
PY
echo "now: PATH=\"\$PWD/.venv/bin:\$PATH\" PYTHONPATH=training/perception:src/mcq_sim .venv/bin/python -m pytest -q src/mcq_sim/test training/perception/tests training/data/tests"
