"""Build the Hugging Face upload folder for the driving model, and upload it.

    .venv/bin/python tools/hf/upload_model.py --build-only          # build and check, upload nothing
    hf auth login                                                    # once, with a token that can write
    .venv/bin/python tools/hf/upload_model.py                        # a PRIVATE repo ExplodingCB2/mcqueen-driving-model
    .venv/bin/python tools/hf/upload_model.py --repo someone/name --public

The repo is private unless --public is passed: the weights were trained partly on frames from third-party videos,
so making them public is a decision to take on purpose. The folder is assembled in training/handoff/hf-upload
(git-ignored) and the example in it is run once to prove the checkpoints load before anything is uploaded.
"""

from __future__ import annotations

import argparse
import hashlib
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RUNS = ROOT / "training" / "perception" / "runs"
STAGE = ROOT / "training" / "handoff" / "hf-upload"
PRIMARY = "drive-v5"
OTHERS = ["drive-v3", "drive-v4", "drive-v6"]

EXAMPLE = '''"""Run a checkpoint on one image:  python example.py frame.jpg [drive-v5.pt]"""
import sys

import numpy as np
from PIL import Image

from driver import Driver

image = sys.argv[1] if len(sys.argv) > 1 else None
weights = sys.argv[2] if len(sys.argv) > 2 else "drive-v5.pt"
driver = Driver(weights)
frame = (
    np.asarray(Image.open(image).convert("RGB").resize((480, 270)))
    if image
    else np.full((270, 480, 3), 110, dtype=np.uint8)
)
mask, line, speed = driver.predict_drive(frame)
print(f"{weights}: pavement share {float((mask > 0.5).mean()):.2f}")
for k, ((x, y), v) in enumerate(zip(line, speed, strict=True)):
    print(f"  waypoint {k:2d}: {x:5.1f} m ahead, {y:+5.2f} m left, wants {v:4.1f} m/s")
'''


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build() -> Path:
    if STAGE.exists():
        shutil.rmtree(STAGE)
    (STAGE / "checkpoints").mkdir(parents=True)
    shutil.copy(RUNS / PRIMARY / "model.pt", STAGE / f"{PRIMARY}.pt")
    for name in OTHERS:
        shutil.copy(RUNS / name / "model.pt", STAGE / "checkpoints" / f"{name}.pt")
    for src in ("model.py", "driver.py"):
        shutil.copy(ROOT / "training" / "perception" / src, STAGE / src)
    shutil.copy(RUNS / "laptime-v2" / "best.json", STAGE / "laptime-v2-best.json")
    (STAGE / "example.py").write_text(EXAMPLE)
    rows = ["## Checksums (SHA-256)", "", "| File | SHA-256 | Size |", "| --- | --- | --- |"]
    for f in sorted(STAGE.rglob("*.pt")):
        rows.append(f"| `{f.relative_to(STAGE)}` | `{sha(f)}` | {f.stat().st_size / 1e6:.1f} MB |")
    card = (ROOT / "tools" / "hf" / "MODEL_CARD.md").read_text().replace("{{CHECKSUMS}}", "\n".join(rows))
    (STAGE / "README.md").write_text(card)
    return STAGE


def check(stage: Path) -> None:
    """Load every checkpoint and run the shipped example from inside the folder, so a broken upload never leaves."""
    names = [f"{PRIMARY}.pt", *[f"checkpoints/{n}.pt" for n in OTHERS]]
    code = (
        "import sys, numpy as np; from driver import Driver; "
        "m, l, s = Driver(sys.argv[1]).predict_drive(np.zeros((270, 480, 3), np.uint8)); "
        "assert m.shape == (270, 480) and l.shape == (12, 2) and s.shape == (12,); print('ok')"
    )
    for name in names:
        out = subprocess.run([sys.executable, "-c", code, name], cwd=stage, capture_output=True, text=True)
        if out.returncode or "ok" not in out.stdout:
            raise SystemExit(f"{name} does not load:\n{out.stderr[-600:]}")
    out = subprocess.run([sys.executable, "example.py"], cwd=stage, capture_output=True, text=True)
    if out.returncode:
        raise SystemExit("example.py fails:\n" + out.stderr[-600:])
    print(f"all {len(names)} checkpoints load and example.py runs")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--repo", default="ExplodingCB2/mcqueen-driving-model")
    parser.add_argument("--public", action="store_true", help="make the repo public (default: private)")
    parser.add_argument("--build-only", action="store_true")
    args = parser.parse_args()
    stage = build()
    print(f"built {stage}: {sum(f.stat().st_size for f in stage.rglob('*') if f.is_file()) / 1e6:.0f} MB")
    check(stage)
    if args.build_only:
        return
    from huggingface_hub import HfApi

    api = HfApi()
    who = api.whoami()["name"]
    api.create_repo(args.repo, repo_type="model", private=not args.public, exist_ok=True)
    api.upload_folder(
        folder_path=str(stage),
        repo_id=args.repo,
        repo_type="model",
        commit_message="drive-v5 and the checkpoints it came from, with a model card",
    )
    print(f"uploaded as {who}: https://huggingface.co/{args.repo} ({'public' if args.public else 'PRIVATE'})")


if __name__ == "__main__":
    main()
