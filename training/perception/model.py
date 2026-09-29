"""A small encoder-decoder for the pavement mask.

About 1.6 M parameters, all standard convolutions, so it exports to TensorRT
without surprises and runs far inside the 20 fps budget on the Orin. The input is
the 240 x 135 half-resolution frame; the output is one logit per pixel at the
same size. A dilated bottleneck gives the receptive field to see the whole road
shape at once, which matters more than texture: pavement and grass can be the
same color, and the shape of the corridor still says which is which.
"""

from __future__ import annotations

import torch
import torch.nn as nn
import torch.nn.functional as F


def block(cin, cout, stride=1, dilation=1):
    return nn.Sequential(
        nn.Conv2d(cin, cout, 3, stride, padding=dilation, dilation=dilation, bias=False),
        nn.BatchNorm2d(cout),
        nn.ReLU(inplace=True),
        nn.Conv2d(cout, cout, 3, 1, padding=dilation, dilation=dilation, bias=False),
        nn.BatchNorm2d(cout),
        nn.ReLU(inplace=True),
    )


class PavementNet(nn.Module):
    def __init__(self, width=24):
        super().__init__()
        w = width
        self.e1 = block(3, w, stride=2)  # 1/2
        self.e2 = block(w, 2 * w, stride=2)  # 1/4
        self.e3 = block(2 * w, 4 * w, stride=2)  # 1/8
        self.e4 = block(4 * w, 6 * w, stride=2)  # 1/16
        self.context = nn.Sequential(block(6 * w, 6 * w, dilation=2), block(6 * w, 6 * w, dilation=4))
        self.d3 = block(6 * w + 4 * w, 4 * w)
        self.d2 = block(4 * w + 2 * w, 2 * w)
        self.d1 = block(2 * w + w, w)
        self.head = nn.Conv2d(w, 1, 1)

    @staticmethod
    def _up(x, like):
        return F.interpolate(x, size=like.shape[-2:], mode="bilinear", align_corners=False)

    def forward(self, x):
        size = x.shape[-2:]
        x = x / 255.0 - 0.5
        e1 = self.e1(x)
        e2 = self.e2(e1)
        e3 = self.e3(e2)
        e4 = self.context(self.e4(e3))
        d3 = self.d3(torch.cat([self._up(e4, e3), e3], 1))
        d2 = self.d2(torch.cat([self._up(d3, e2), e2], 1))
        d1 = self.d1(torch.cat([self._up(d2, e1), e1], 1))
        return F.interpolate(self.head(d1), size=size, mode="bilinear", align_corners=False)[:, 0]
