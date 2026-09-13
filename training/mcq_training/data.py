"""PyTorch side of the data: a dataset over a split CSV and the augmentation
that makes helmet and chassis footage look like each other (scale, horizon
shift, flip, color, blur). Pillow and numpy do the work so the same code runs
without torchvision transforms."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch
from PIL import Image, ImageEnhance, ImageFilter, ImageOps
from torch.utils.data import Dataset

from mcq_training.dataset import Sample
from mcq_training.labels import IGNORE, load_label

MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)
FILL = tuple(int(round(255 * m)) for m in MEAN)


@dataclass
class AugmentParams:
    hflip: float = 0.5
    scale: tuple[float, float] = (0.8, 1.25)
    vshift: float = 0.08  # fraction of the height the content may move up or down
    hshift: float = 0.05
    brightness: float = 0.3
    contrast: float = 0.3
    saturation: float = 0.3
    blur: float = 0.15  # probability
    grayscale: float = 0.05  # probability

    @classmethod
    def from_dict(cls, d: dict | None) -> AugmentParams:
        d = dict(d or {})
        if "scale" in d:
            d["scale"] = tuple(float(v) for v in d["scale"])
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})


class Augment:
    def __init__(self, params: AugmentParams | None = None, seed: int | None = None):
        self.p = params or AugmentParams()
        self.rng = np.random.default_rng(seed)

    def __call__(self, image: Image.Image, label: np.ndarray) -> tuple[Image.Image, np.ndarray]:
        p, rng = self.p, self.rng
        w, h = image.size
        s = float(rng.uniform(*p.scale))
        nw, nh = max(int(round(w * s)), 8), max(int(round(h * s)), 8)
        image = image.resize((nw, nh), Image.BILINEAR)
        lab = np.asarray(Image.fromarray(label).resize((nw, nh), Image.NEAREST))
        dx = (w - nw) // 2 + int(round(rng.uniform(-p.hshift, p.hshift) * w))
        dy = (h - nh) // 2 + int(round(rng.uniform(-p.vshift, p.vshift) * h))
        canvas = Image.new("RGB", (w, h), FILL)
        canvas.paste(image, (dx, dy))
        out = np.full((h, w), IGNORE, dtype=np.uint8)
        x0, y0 = max(dx, 0), max(dy, 0)
        x1, y1 = min(dx + nw, w), min(dy + nh, h)
        if x1 > x0 and y1 > y0:
            out[y0:y1, x0:x1] = lab[y0 - dy : y1 - dy, x0 - dx : x1 - dx]
        image = canvas
        if rng.random() < p.hflip:
            image = ImageOps.mirror(image)
            out = np.ascontiguousarray(out[:, ::-1])
        if p.brightness:
            image = ImageEnhance.Brightness(image).enhance(1.0 + rng.uniform(-p.brightness, p.brightness))
        if p.contrast:
            image = ImageEnhance.Contrast(image).enhance(1.0 + rng.uniform(-p.contrast, p.contrast))
        if p.saturation:
            image = ImageEnhance.Color(image).enhance(1.0 + rng.uniform(-p.saturation, p.saturation))
        if rng.random() < p.blur:
            image = image.filter(ImageFilter.GaussianBlur(radius=float(rng.uniform(0.5, 1.5))))
        if rng.random() < p.grayscale:
            image = image.convert("L").convert("RGB")
        return image, out


def image_to_tensor(image: Image.Image) -> torch.Tensor:
    arr = (np.asarray(image, dtype=np.float32) / 255.0 - MEAN) / STD
    return torch.from_numpy(np.ascontiguousarray(arr.transpose(2, 0, 1)))


class SegDataset(Dataset):
    def __init__(
        self,
        samples: list[Sample],
        frames_root: Path,
        labels_root: Path,
        size: tuple[int, int] = (640, 400),
        augment: Augment | None = None,
    ):
        self.samples = samples
        self.frames_root = Path(frames_root)
        self.labels_root = Path(labels_root)
        self.size = size
        self.augment = augment

    def __len__(self) -> int:
        return len(self.samples)

    def load(self, i: int) -> tuple[Image.Image, np.ndarray]:
        s = self.samples[i]
        with Image.open(self.frames_root / s.frame) as im:
            image = im.convert("RGB").resize(self.size, Image.BILINEAR)
        label = Image.fromarray(load_label(self.labels_root / s.label)).resize(self.size, Image.NEAREST)
        return image, np.asarray(label, dtype=np.uint8)

    def __getitem__(self, i: int):
        image, label = self.load(i)
        if self.augment is not None:
            image, label = self.augment(image, label)
        return image_to_tensor(image), torch.from_numpy(label.astype(np.int64)), i

    @property
    def weights(self) -> list[float]:
        return [s.weight for s in self.samples]
