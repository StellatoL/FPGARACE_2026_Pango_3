#!/usr/bin/env python3
"""Export fixed ADAS tensor contracts. Requires a PC PyTorch/ONNX environment."""
import argparse
import json
import sys
from pathlib import Path

REQUIRED_CLASSES = {'person', 'vehicle', 'crosswalk', 'light_red', 'light_green',
                    'cone', 'barrier', 'debris'}


def export_lane(args):
    import torch
    from lane_model import LaneNetwork
    model = LaneNetwork()
    checkpoint = torch.load(args.weights, map_location='cpu')
    model.load_state_dict(checkpoint['model'])
    model.eval()
    torch.onnx.export(model, torch.zeros(1, 3, 256, 512), str(args.output),
                      input_names=['rgb'], output_names=['lane_x', 'lane_present',
                                                         'lane_paint', 'lane_type'],
                      opset_version=12, do_constant_folding=True)
    return {'output': 'lane_rows_4x96_v1', 'width': 512, 'height': 256,
            'road_roi': checkpoint.get('road_roi', [0, .42, 1, .58])}


def export_scene(args):
    import types
    import torch
    if not args.yolov5_repo:
        raise ValueError('--yolov5-repo must point to a YOLOv5 v7.0 checkout')
    sys.path.insert(0, str(args.yolov5_repo.resolve()))
    from models.experimental import attempt_load
    model = attempt_load(str(args.weights), device=torch.device('cpu'), inplace=True, fuse=True)
    model.eval()
    names = model.names
    names = [names[i] for i in range(len(names))]
    if not REQUIRED_CLASSES <= set(names) or len(set(names)) != len(names) or len(names) > 64:
        raise ValueError('trained scene model lacks mandatory classes or has invalid labels')
    head = model.model[-1]
    if head.nl != 3 or head.na != 3:
        raise ValueError('three YOLOv5 heads with three anchors each are required')
    anchors = (head.anchors.detach().cpu() * head.stride[:, None, None].cpu()).reshape(3, 6).tolist()

    def raw_heads(self, features):
        return tuple(self.m[i](features[i]) for i in range(self.nl))

    head.forward = types.MethodType(raw_heads, head)
    torch.onnx.export(model, torch.zeros(1, 3, 640, 640), str(args.output),
                      input_names=['rgb'], output_names=['p3', 'p4', 'p5'],
                      opset_version=12, do_constant_folding=True)
    return {'output': 'yolov5_raw_nchw_v1', 'width': 640, 'height': 640,
            'classes': names, 'anchors': anchors}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('kind', choices=['scene', 'lane'])
    parser.add_argument('--weights', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--yolov5-repo', type=Path)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    metadata = export_scene(args) if args.kind == 'scene' else export_lane(args)
    metadata['normalization'] = {'mean': [0, 0, 0], 'std': [255, 255, 255]}
    args.output.with_suffix('.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    print('Exported ONNX and tensor metadata; RKNN conversion and real accuracy validation remain required.')


if __name__ == '__main__':
    main()
