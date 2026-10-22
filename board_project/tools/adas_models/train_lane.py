#!/usr/bin/env python3
"""Development-PC training for lane_rows_4x96_v1. See docs/model-replacement-roadmap.md."""
import argparse
import json
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--train', required=True, type=Path)
    p.add_argument('--validation', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--epochs', type=int, default=100)
    p.add_argument('--batch', type=int, default=16)
    p.add_argument('--road-roi', type=float, nargs=4, default=[0, .42, 1, .58])
    a = p.parse_args()
    if a.epochs <= 0 or a.batch <= 0:
        p.error('epochs and batch must be positive')
    x0, y0, rw, rh = a.road_roi
    if x0 < 0 or y0 < 0 or rw <= 0 or rh <= 0 or x0+rw > 1 or y0+rh > 1:
        p.error('invalid road ROI')
    import numpy as np
    import torch
    from PIL import Image
    from torch.utils.data import Dataset, DataLoader
    from lane_model import LaneNetwork

    class Samples(Dataset):
        def __init__(self, filename):
            self.base = filename.resolve().parent
            self.rows = [json.loads(line) for line in filename.read_text(encoding='utf-8').splitlines() if line.strip()]
            if not self.rows:
                raise ValueError('dataset is empty')
            self.paths = {(self.base / item['image']).resolve() for item in self.rows}
        def __len__(self):
            return len(self.rows)
        def __getitem__(self, index):
            item = self.rows[index]
            image = Image.open(self.base / item['image']).convert('RGB')
            w, h = image.size
            left, top = int(x0*w) & ~1, int(y0*h) & ~1
            cw, ch = int(rw*w) & ~1, int(rh*h) & ~1
            image = image.crop((left, top, left+cw, top+ch))
            # RGB565 appearance before resize, matching the FPGA transport precision.
            rgb = np.asarray(image).astype(np.uint16)
            rgb[:, :, 0] = (rgb[:, :, 0] >> 3) * 255 // 31
            rgb[:, :, 1] = (rgb[:, :, 1] >> 2) * 255 // 63
            rgb[:, :, 2] = (rgb[:, :, 2] >> 3) * 255 // 31
            image = Image.fromarray(rgb.astype(np.uint8)).resize((512, 256), Image.Resampling.BILINEAR)
            tensor = torch.from_numpy(np.asarray(image).copy()).permute(2, 0, 1).float()/255
            xx = torch.tensor(item['x'], dtype=torch.float32)
            paint = torch.tensor(item['paint'], dtype=torch.long)
            kind = torch.tensor(item['type'], dtype=torch.long)
            if xx.shape != (4, 96) or paint.shape != (4, 96) or kind.shape != (4,):
                raise ValueError('expected x/paint[4,96], type[4]')
            if not torch.isfinite(xx).all() or ((xx != -1) & ((xx < 0) | (xx > 1))).any() or (paint < 0).any() or (paint > 2).any() or (kind < 0).any() or (kind > 2).any():
                raise ValueError('invalid lane labels')
            return tensor, xx, paint, kind

    train, validation = Samples(a.train), Samples(a.validation)
    if train.paths & validation.paths:
        raise ValueError('training and validation images overlap; split whole road sequences first')
    torch.manual_seed(3568)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = LaneNetwork().to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-3, weight_decay=1e-4)
    loaders = [DataLoader(train, batch_size=a.batch, shuffle=True),
               DataLoader(validation, batch_size=a.batch)]
    a.output.parent.mkdir(parents=True, exist_ok=True)
    best = float('inf')
    for epoch in range(a.epochs):
        totals = []
        for training, loader in zip([True, False], loaders):
            model.train(training); total = 0; seen = 0
            for image, truth, paint, kind in loader:
                image, truth, paint, kind = [v.to(device) for v in (image, truth, paint, kind)]
                with torch.set_grad_enabled(training):
                    xx, visible, pp, tt = model(image)
                    mask = truth >= 0
                    location = torch.nn.functional.smooth_l1_loss(xx[mask], truth[mask]) if mask.any() else xx.sum()*0
                    loss = 4*location + torch.nn.functional.binary_cross_entropy(visible, mask.float())
                    loss += torch.nn.functional.cross_entropy(pp.reshape(-1, 3), paint.reshape(-1))
                    loss += torch.nn.functional.cross_entropy(tt.reshape(-1, 3), kind.reshape(-1))
                    if training:
                        optimizer.zero_grad(); loss.backward(); optimizer.step()
                total += float(loss.detach())*image.shape[0]; seen += image.shape[0]
            totals.append(total/seen)
        print(f'epoch={epoch+1} train={totals[0]:.5f} validation={totals[1]:.5f}')
        if totals[1] < best:
            best = totals[1]
            torch.save({'model': model.cpu().state_dict(), 'road_roi': a.road_roi,
                        'contract': 'lane_rows_4x96_v1', 'validation_loss': best}, a.output)
            model.to(device)
    print('Checkpoint saved. Loss does not establish lane accuracy; independent scene evaluation is required.')


if __name__ == '__main__':
    main()
