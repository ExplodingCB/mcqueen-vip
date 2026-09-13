"""Segmentation metrics in pixels (confusion matrix, IoU) and in metres
(edge error after inverse perspective mapping), matching the acceptance
criteria in docs/08-training-data.md section 7.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from mcq_training.boundary import extract_bounds
from mcq_training.camera import CameraModel
from mcq_training.labels import CLASS_NAMES, IGNORE, NUM_CLASSES


# ------------------------------------------------------------------ pixels
def confusion_matrix(pred: np.ndarray, gt: np.ndarray, n: int = NUM_CLASSES, ignore: int = IGNORE) -> np.ndarray:
    """Rows are ground truth, columns are predictions; ignore pixels are skipped."""
    pred = np.asarray(pred).ravel().astype(np.int64)
    gt = np.asarray(gt).ravel().astype(np.int64)
    keep = gt != ignore
    idx = gt[keep] * n + pred[keep]
    return np.bincount(idx, minlength=n * n).reshape(n, n)


def iou_per_class(cm: np.ndarray) -> np.ndarray:
    """IoU per class; NaN for a class absent from both truth and prediction."""
    tp = np.diag(cm).astype(float)
    union = cm.sum(0) + cm.sum(1) - tp
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(union > 0, tp / union, np.nan)


def mean_iou(cm: np.ndarray) -> float:
    return float(np.nanmean(iou_per_class(cm)))


def pixel_accuracy(cm: np.ndarray) -> float:
    total = cm.sum()
    return float(np.diag(cm).sum() / total) if total else float("nan")


def summarize_confusion(cm: np.ndarray) -> dict:
    per = iou_per_class(cm)
    return {
        "miou": mean_iou(cm),
        "pixel_accuracy": pixel_accuracy(cm),
        "iou": {name: (None if np.isnan(v) else float(v)) for name, v in zip(CLASS_NAMES, per, strict=True)},
        "pixels": int(cm.sum()),
    }


# ------------------------------------------------------------------ metres
@dataclass
class BoundaryAccumulator:
    """Lateral edge error between predicted and true masks, binned by range.

    Both masks go through the same edge extraction and inverse perspective
    mapping, so the number is the model's error in metres on the ground and
    not a property of the extractor. Rows where the truth has an edge and the
    prediction has no pavement at all count as misses.
    """

    cam: CameraModel
    bin_edges: tuple[float, ...] = (0.0, 5.0, 10.0, 15.0, 20.0, 30.0)
    sums: np.ndarray = field(init=False)
    counts: np.ndarray = field(init=False)
    misses: int = 0
    frames: int = 0

    def __post_init__(self):
        n = len(self.bin_edges) - 1
        self.sums = np.zeros((2, n))
        self.counts = np.zeros((2, n), dtype=int)

    def add(self, pred: np.ndarray, gt: np.ndarray) -> None:
        bp = extract_bounds(pred, self.cam)
        bg = extract_bounds(gt, self.cam)
        self.frames += 1
        if len(bg) == 0:
            return
        pred_rows = {int(r): i for i, r in enumerate(bp.rows)}
        for j, row in enumerate(bg.rows):
            i = pred_rows.get(int(row))
            if i is None:
                self.misses += 1
                continue
            for side, (conf_g, conf_p, pts_g, pts_p) in enumerate(
                (
                    (bg.left_confidence[j], bp.left_confidence[i], bg.left[j], bp.left[i]),
                    (bg.right_confidence[j], bp.right_confidence[i], bg.right[j], bp.right[i]),
                )
            ):
                if conf_g < 1.0 or conf_p < 1.0:
                    continue
                r = float(pts_g[0])
                b = int(np.searchsorted(self.bin_edges, r, side="right")) - 1
                if 0 <= b < self.sums.shape[1]:
                    self.sums[side, b] += abs(float(pts_p[1] - pts_g[1]))
                    self.counts[side, b] += 1

    def summary(self) -> dict:
        with np.errstate(invalid="ignore", divide="ignore"):
            per_side = np.where(self.counts > 0, self.sums / np.maximum(self.counts, 1), np.nan)
            both = self.sums.sum(0)
            both_n = self.counts.sum(0)
            combined = np.where(both_n > 0, both / np.maximum(both_n, 1), np.nan)
        bins = [f"{a:g}-{b:g}m" for a, b in zip(self.bin_edges[:-1], self.bin_edges[1:], strict=True)]

        def clean(arr):
            return [None if np.isnan(v) else float(v) for v in arr]

        return {
            "bins": bins,
            "mean_abs_error_m": clean(combined),
            "left_m": clean(per_side[0]),
            "right_m": clean(per_side[1]),
            "samples": [int(v) for v in both_n],
            "missed_rows": int(self.misses),
            "frames": int(self.frames),
        }


def error_at_range(summary: dict, range_m: float) -> float | None:
    """Mean absolute error of the bin containing ``range_m`` (its upper edge inclusive)."""
    for name, value in zip(summary["bins"], summary["mean_abs_error_m"], strict=True):
        lo, hi = (float(v) for v in name[:-1].split("-"))
        if lo < range_m <= hi:
            return value
    return None
