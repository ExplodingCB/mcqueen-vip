"""Segmentation models behind one name. Every entry takes an (n, 3, H, W)
normalized image and returns (n, classes, H, W) logits at the input size, so
training, evaluation and export do not care which one is loaded.

* ``tiny_unet``: a few hundred thousand parameters; CPU tests and a sanity
  baseline, not for the kart.
* ``lraspp_mobilenet_v3_large``: torchvision's Lite R-ASPP on MobileNetV3
  (about 3 M parameters), the first model for the Orin.
* ``deeplabv3_mobilenet_v3_large``: same backbone, heavier head, for comparison.

PIDNet-S and DDRNet-23-slim (Cityscapes real-time networks, MIT) are the next
candidates when accuracy at the edges matters more than the first model can
give; they slot in here with the same contract.
"""

from __future__ import annotations

import torch
from torch import nn
from torch.nn import functional as F

MODEL_NAMES = ("tiny_unet", "lraspp_mobilenet_v3_large", "deeplabv3_mobilenet_v3_large")


class ConvBlock(nn.Sequential):
    def __init__(self, cin: int, cout: int):
        super().__init__(
            nn.Conv2d(cin, cout, 3, padding=1, bias=False),
            nn.BatchNorm2d(cout),
            nn.ReLU(inplace=True),
            nn.Conv2d(cout, cout, 3, padding=1, bias=False),
            nn.BatchNorm2d(cout),
            nn.ReLU(inplace=True),
        )


class TinyUNet(nn.Module):
    def __init__(self, num_classes: int = 3, width: int = 16):
        super().__init__()
        w = width
        self.enc1, self.enc2, self.enc3 = ConvBlock(3, w), ConvBlock(w, 2 * w), ConvBlock(2 * w, 4 * w)
        self.dec2, self.dec1 = ConvBlock(6 * w, 2 * w), ConvBlock(3 * w, w)
        self.head = nn.Conv2d(w, num_classes, 1)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        e1 = self.enc1(x)
        e2 = self.enc2(F.max_pool2d(e1, 2))
        e3 = self.enc3(F.max_pool2d(e2, 2))
        d2 = self.dec2(torch.cat([F.interpolate(e3, size=e2.shape[-2:], mode="bilinear", align_corners=False), e2], 1))
        d1 = self.dec1(torch.cat([F.interpolate(d2, size=e1.shape[-2:], mode="bilinear", align_corners=False), e1], 1))
        return self.head(d1)


class DictOut(nn.Module):
    """torchvision segmentation models return {"out": logits}; unwrap it."""

    def __init__(self, inner: nn.Module):
        super().__init__()
        self.inner = inner

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.inner(x)["out"]


def build_model(name: str, num_classes: int = 3, pretrained_backbone: bool = False) -> nn.Module:
    if name == "tiny_unet":
        return TinyUNet(num_classes)
    if name in ("lraspp_mobilenet_v3_large", "deeplabv3_mobilenet_v3_large"):
        from torchvision.models import MobileNet_V3_Large_Weights
        from torchvision.models import segmentation as seg

        backbone = MobileNet_V3_Large_Weights.IMAGENET1K_V1 if pretrained_backbone else None
        ctor = getattr(seg, name)
        kwargs = {"weights": None, "weights_backbone": backbone, "num_classes": num_classes}
        if name.startswith("deeplabv3"):
            kwargs["aux_loss"] = False
        return DictOut(ctor(**kwargs))
    raise KeyError(f"unknown model {name!r}; choose from {MODEL_NAMES}")


def count_parameters(model: nn.Module) -> int:
    return sum(p.numel() for p in model.parameters())
