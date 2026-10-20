"""The driving model behind the simulator's camera-model contract.

    python -m mcq_sim evaluate --policy stack --perception driver:create

with ``PYTHONPATH=training/perception:src/mcq_sim``. Weights come from
``MCQ_DRIVER_WEIGHTS``, else ``runs/drive-latest/model.pt`` beside this file.

``predict(rgb)`` is the pavement probability, as for a segmenter, so the stack
can build its corridor. ``predict_drive(rgb)`` adds the racing line the model
intends to drive, as waypoints in base_link (x forward, y left), and the speed
it wants at each. The stack drives that line when it stays inside the corridor
and falls back to its own planner when it does not.
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F

from model import DriveNet

HERE = Path(__file__).resolve().parent


class Driver:
    def __init__(self, weights: str | Path, device="cpu"):
        torch.set_num_threads(1)  # many drivers run side by side in rollouts; one thread each
        checkpoint = torch.load(weights, map_location=device, weights_only=True)
        self.net = DriveNet(fine=bool(checkpoint.get("fine", False))).to(device).eval()
        self.net.load_state_dict(checkpoint["state"])
        self.device = device
        self.weights = str(weights)

    @torch.no_grad()
    def predict_drive(self, rgb: np.ndarray):
        """uint8 HWC RGB (270 x 480) in; (pavement probability 270 x 480,
        waypoints K x 2 in metres, speeds K in m/s) out."""
        x = torch.from_numpy(np.ascontiguousarray(rgb)).to(self.device).permute(2, 0, 1)[None].float()
        h, w = x.shape[-2:]
        logits, points, speeds = self.net.forward_all(F.avg_pool2d(x, 2))
        probability = torch.sigmoid(F.interpolate(logits[None], size=(h, w), mode="bilinear", align_corners=False))
        out = probability[0, 0].cpu().numpy(), points[0].cpu().numpy(), speeds[0].cpu().numpy()
        return out

    def predict(self, rgb: np.ndarray) -> np.ndarray:
        return self.predict_drive(rgb)[0]


def create():
    path = os.environ.get("MCQ_DRIVER_WEIGHTS", HERE / "runs" / "drive-latest" / "model.pt")
    return Driver(path)
