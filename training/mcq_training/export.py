"""Export a checkpoint to ONNX for the TensorRT build on the Jetson, and check
that the ONNX graph reproduces the PyTorch outputs.

    python -m mcq_training.export --checkpoint runs/seg_v1_a/best.pt
    # on the Jetson (JetPack 7.2, TensorRT 10.16):
    trtexec --onnx=model.onnx --saveEngine=model.engine --fp16

The graph has a fixed 1 x 3 x H x W input because the perception node runs
one frame at a time at one resolution; a sidecar json records the input size,
the normalization and the class order the node must apply.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch

from mcq_training import REPO
from mcq_training.data import MEAN, STD
from mcq_training.dataset import git_hash
from mcq_training.evaluate import load_checkpoint
from mcq_training.labels import CLASS_NAMES
from mcq_training.train import input_size


def export_onnx(checkpoint: Path, out: Path, opset: int = 18, check: bool = True) -> dict:
    model, config = load_checkpoint(checkpoint, torch.device("cpu"))
    w, h = input_size(config)
    dummy = torch.zeros(1, 3, h, w)
    out.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        model,
        (dummy,),
        str(out),
        input_names=["image"],
        output_names=["logits"],
        opset_version=opset,
        dynamo=True,
        external_data=False,
    )
    info = {
        "onnx": str(out),
        "checkpoint": str(checkpoint),
        "input": {"name": "image", "shape": [1, 3, h, w], "layout": "NCHW", "range": "rgb 0..1 then (x - mean) / std"},
        "mean": [float(v) for v in MEAN],
        "std": [float(v) for v in STD],
        "output": {"name": "logits", "shape": [1, len(CLASS_NAMES), h, w]},
        "classes": list(CLASS_NAMES),
        "opset": opset,
        "torch": torch.__version__,
        "git": git_hash(REPO),
        "trtexec": f"trtexec --onnx={out.name} --saveEngine={out.stem}.engine --fp16",
    }
    if check:
        import onnxruntime as ort

        x = torch.rand(1, 3, h, w) * 2 - 1
        with torch.no_grad():
            ref = model(x).numpy()
        sess = ort.InferenceSession(str(out), providers=["CPUExecutionProvider"])
        got = sess.run(["logits"], {"image": x.numpy()})[0]
        info["check"] = {
            "max_abs_diff": float(np.abs(ref - got).max()),
            "argmax_agreement": float(np.mean(ref.argmax(1) == got.argmax(1))),
        }
    with open(out.with_suffix(".json"), "w") as f:
        json.dump(info, f, indent=2)
    return info


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--checkpoint", type=Path, required=True)
    ap.add_argument("--out", type=Path, default=None, help="default: model.onnx next to the checkpoint")
    ap.add_argument("--opset", type=int, default=18)
    ap.add_argument("--no-check", action="store_true", help="skip the onnxruntime comparison")
    args = ap.parse_args(argv)
    out = args.out or args.checkpoint.parent / "model.onnx"
    info = export_onnx(args.checkpoint, out, args.opset, not args.no_check)
    print(f"wrote {out} ({out.stat().st_size / 1e6:.1f} MB), input {info['input']['shape']}")
    if "check" in info:
        c = info["check"]
        print(f"onnxruntime parity: max |diff| {c['max_abs_diff']:.2e}, argmax agreement {c['argmax_agreement']:.4f}")
    print(f"on the Jetson: {info['trtexec']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
