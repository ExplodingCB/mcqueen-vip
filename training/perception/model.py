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


class DriveNet(PavementNet):
    """PavementNet with a driving head: from the same encoder, the racing line
    as ``n_waypoints`` points in the kart's frame (x forward, y left) and the
    target speed at each. The mask head stays, so the stack can still check the
    driving output against the pavement it sees.

    The head reads the whole bottleneck, not a pooled vector, because where the
    road goes is a matter of where in the image it is. Outputs are scaled to
    about unit size: x / 25 m, y / 10 m, speed / 15 m/s.
    """

    SCALE = (25.0, 10.0, 15.0)

    def __init__(self, width=24, n_waypoints=12, grid=(9, 15), fine=False):
        super().__init__(width)
        self.n_waypoints = n_waypoints
        self.squeeze = nn.Sequential(nn.Conv2d(6 * width, 32, 1), nn.BatchNorm2d(32), nn.ReLU(inplace=True))
        self.grid = grid
        # ``fine``: the head also reads the decoder's quarter-resolution features
        # with two pixel-coordinate channels, then strides them down to the grid.
        # Where the kart sits relative to the line is read from where the edges
        # are 5 to 10 m ahead, a matter of a few pixels the 1/16 bottleneck blurs.
        self.fine = fine
        if fine:
            self.fine_path = nn.Sequential(
                nn.Conv2d(2 * width + 2, 64, 3, 2, 1, bias=False),
                nn.BatchNorm2d(64),
                nn.ReLU(inplace=True),
                nn.Conv2d(64, 64, 3, 2, 1, bias=False),
                nn.BatchNorm2d(64),
                nn.ReLU(inplace=True),
                nn.Conv2d(64, 32, 1),
                nn.BatchNorm2d(32),
                nn.ReLU(inplace=True),
            )
        head_in = 64 if fine else 32
        self.drive = nn.Sequential(
            nn.Flatten(),
            nn.Linear(head_in * grid[0] * grid[1], 256),
            nn.ReLU(inplace=True),
            nn.Dropout(0.1),
            nn.Linear(256, 3 * n_waypoints),
        )

    def forward_all(self, x):
        """(mask logits N x H x W, waypoints N x K x 2 in metres, speeds N x K in m/s)."""
        size = x.shape[-2:]
        x = x / 255.0 - 0.5
        e1 = self.e1(x)
        e2 = self.e2(e1)
        e3 = self.e3(e2)
        e4 = self.context(self.e4(e3))
        d3 = self.d3(torch.cat([self._up(e4, e3), e3], 1))
        d2 = self.d2(torch.cat([self._up(d3, e2), e2], 1))
        d1 = self.d1(torch.cat([self._up(d2, e1), e1], 1))
        mask = F.interpolate(self.head(d1), size=size, mode="bilinear", align_corners=False)[:, 0]
        code = F.adaptive_avg_pool2d(self.squeeze(e4), self.grid)
        if self.fine:
            n, _, h, w = d2.shape
            ys = torch.linspace(-1, 1, h, device=d2.device).view(1, 1, h, 1).expand(n, 1, h, w)
            xs = torch.linspace(-1, 1, w, device=d2.device).view(1, 1, 1, w).expand(n, 1, h, w)
            detail = self.fine_path(torch.cat([d2, ys, xs], 1))
            code = torch.cat([code, F.adaptive_avg_pool2d(detail, self.grid)], 1)
        out = self.drive(code).view(-1, self.n_waypoints, 3)
        sx, sy, sv = self.SCALE
        points = torch.stack([out[..., 0] * sx, out[..., 1] * sy], -1)
        speeds = out[..., 2] * sv
        return mask, points, speeds
