"""The trained model behind the simulator's ``predict(rgb)`` contract.

    python -m mcq_sim evaluate --policy stack --perception segmenter:create

with ``PYTHONPATH=training/perception:src/mcq_sim``. Weights come from
``MCQ_SEGMENTER_WEIGHTS``, else ``runs/latest/model.pt`` beside this file.
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F

from model import PavementNet

HERE = Path(__file__).resolve().parent


class Segmenter:
    def __init__(self, weights: str | Path, device="cpu"):
        checkpoint = torch.load(weights, map_location=device, weights_only=True)
        self.net = PavementNet(checkpoint["width"]).to(device).eval()
        self.net.load_state_dict(checkpoint["state"])
        self.device = device
        self.weights = str(weights)

    @torch.no_grad()
    def predict(self, rgb: np.ndarray) -> np.ndarray:
        """uint8 HWC RGB (270 x 480) in, pavement probability at the same size out."""
        x = torch.from_numpy(np.ascontiguousarray(rgb)).to(self.device).permute(2, 0, 1)[None].float()
        h, w = x.shape[-2:]
        half = F.avg_pool2d(x, 2) if h % 2 == 0 and w % 2 == 0 else F.interpolate(x, size=(h // 2, w // 2), mode="area")
        logits = self.net(half)[None]
        return (
            torch.sigmoid(F.interpolate(logits, size=(h, w), mode="bilinear", align_corners=False))[0, 0].cpu().numpy()
        )


def create():
    path = os.environ.get("MCQ_SEGMENTER_WEIGHTS", HERE / "runs" / "latest" / "model.pt")
    return Segmenter(path)
