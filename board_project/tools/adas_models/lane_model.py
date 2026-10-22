"""Training/export on a development PC; not imported by the ARM application."""
import torch
from torch import nn
from torchvision.models import mobilenet_v2

ROWS, LANES = 96, 4

class LaneNetwork(nn.Module):
    def __init__(self):
        super().__init__()
        backbone = mobilenet_v2(weights=None, width_mult=0.35)
        channels = backbone.features[-2].out_channels
        self.features = nn.Sequential(*list(backbone.features.children())[:-1],
                                      nn.Conv2d(channels, 16, 1), nn.ReLU6())
        self.head = nn.Sequential(nn.Flatten(), nn.Linear(16 * 8 * 16, 128),
                                  nn.ReLU6(), nn.Linear(128, LANES * ROWS * 5 + LANES * 3))

    def forward(self, rgb):
        raw = self.head(self.features(rgb))
        size = LANES * ROWS
        x = raw[:, :size].reshape(-1, LANES, ROWS).sigmoid()
        present = raw[:, size:2*size].reshape(-1, LANES, ROWS).sigmoid()
        paint = raw[:, 2*size:5*size].reshape(-1, LANES, ROWS, 3)
        kind = raw[:, 5*size:].reshape(-1, LANES, 3)
        return x, present, paint, kind
