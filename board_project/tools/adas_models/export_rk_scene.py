#!/usr/bin/env python3
"""Export the trained eight-class scene model through airockchip/yolov5 --rknpu.

Run with the PyTorch/ONNX interpreter used for conversion. The upstream exporter
is executed unchanged in a temporary workspace; the supplied checkpoint and
checkout are never edited. This contract contains sigmoid NCHW heads, so its
decoder must not apply sigmoid a second time.
"""
import argparse
import ast
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


REQUIRED_CLASSES = {'person', 'vehicle', 'crosswalk', 'light_red', 'light_green',
                    'cone', 'barrier', 'debris'}


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for block in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def shape(tensor):
    return [dimension.dim_value for dimension in tensor.type.tensor_type.shape.dim]


def validate_contract(model, anchors):
    """Reject mislabeled logits, changed head sizes, classes, or anchor values."""
    properties = {item.key: item.value for item in model.metadata_props}
    names = ast.literal_eval(properties.get('names', 'None'))
    if isinstance(names, dict) and set(names) == set(range(len(names))):
        names = [names[i] for i in range(len(names))]
    if (not isinstance(names, list) or len(names) != 8 or
            not all(isinstance(name, str) for name in names) or
            set(names) != REQUIRED_CLASSES):
        raise ValueError('ONNX classes must contain the eight trained ADAS classes exactly once')
    if (len(anchors) != 3 or any(len(row) != 6 for row in anchors) or
            any(not math.isfinite(value) or value <= 0 for row in anchors for value in row)):
        raise ValueError('anchors must contain three groups of six finite positive values')
    if len(model.graph.input) != 1 or shape(model.graph.input[0]) != [1, 3, 640, 640]:
        raise ValueError('ONNX input shape must be fixed NCHW [1,3,640,640]')
    expected = [[1, 39, side, side] for side in (80, 40, 20)]
    if [shape(output) for output in model.graph.output] != expected:
        raise ValueError('ONNX output shape must be [1,39,80,80], [1,39,40,40], [1,39,20,20]')
    producers = {output: node.op_type for node in model.graph.node for output in node.output}
    if any(producers.get(output.name) != 'Sigmoid' for output in model.graph.output):
        raise ValueError('all RK scene outputs must be produced directly by Sigmoid')
    return {'output': 'yolov5_rk_sigmoid_nchw_v1', 'width': 640, 'height': 640,
            'classes': names, 'anchors': anchors,
            'normalization': {'mean': [0, 0, 0], 'std': [255, 255, 255]}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--yolov5-repo', required=True, type=Path,
                        help='airockchip/yolov5 checkout containing export.py --rknpu')
    args = parser.parse_args()
    args.weights = args.weights.resolve()
    args.output = args.output.resolve()
    args.yolov5_repo = args.yolov5_repo.resolve()
    exporter = args.yolov5_repo / 'export.py'
    head_source = args.yolov5_repo / 'models/yolo.py'
    if not args.weights.is_file() or not exporter.is_file() or not head_source.is_file():
        parser.error('weights and airockchip/yolov5 exporter/head source must exist')
    if '--rknpu' not in exporter.read_text(encoding='utf-8'):
        parser.error('selected checkout does not support the official --rknpu entrypoint')
    if 'torch.sigmoid(self.m[i](x[i]))' not in head_source.read_text(encoding='utf-8'):
        parser.error('selected checkout does not implement the RK sigmoid head contract')
    metadata_path = args.output.with_suffix('.json')
    anchors_path = args.output.parent / 'RK_anchors.txt'
    if args.output.suffix != '.onnx':
        parser.error('--output must end in .onnx')
    if any(path.exists() for path in (args.output, metadata_path, anchors_path)):
        parser.error('output ONNX, metadata, or RK_anchors.txt already exists; use a clean output directory')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    source_hash = sha256(args.weights)
    with tempfile.TemporaryDirectory(prefix='.rk-export-', dir=str(args.output.parent)) as directory:
        workspace = Path(directory)
        copied_weights = workspace / 'scene.pt'
        shutil.copyfile(args.weights, copied_weights)
        command = [sys.executable, str(exporter), '--rknpu', '--weights', str(copied_weights),
                   '--include', 'onnx', '--imgsz', '640', '640', '--opset', '12', '--device', 'cpu']
        print('Official export command:', ' '.join(command), flush=True)
        environment = dict(os.environ, YOLOv5_AUTOINSTALL='false', PYTHONDONTWRITEBYTECODE='1')
        subprocess.run(command, cwd=str(workspace), env=environment, check=True)
        import onnx
        exported = copied_weights.with_suffix('.onnx')
        model = onnx.load(str(exported))
        onnx.checker.check_model(model)
        flat_anchors = [float(value) for value in (workspace / 'RK_anchors.txt').read_text().split()]
        if len(flat_anchors) != 18:
            raise ValueError('official exporter did not produce 18 anchor values')
        metadata = validate_contract(model, [flat_anchors[i:i + 6] for i in range(0, 18, 6)])
        if sha256(args.weights) != source_hash or sha256(copied_weights) != source_hash:
            raise RuntimeError('checkpoint hash changed during export')
        metadata['provenance'] = {
            'source_weights': str(args.weights), 'source_weights_sha256': source_hash,
            'exporter': str(exporter), 'exporter_sha256': sha256(exporter),
            'head_source_sha256': sha256(head_source), 'onnx_sha256': sha256(exported),
            'entrypoint': 'export.py --rknpu --weights scene.pt --include onnx --imgsz 640 640 --opset 12 --device cpu',
            'reference': 'https://github.com/airockchip/yolov5/blob/master/README_rkopt.md'}
        metadata['tensors'] = {
            'input': {'name': model.graph.input[0].name, 'shape': shape(model.graph.input[0]), 'layout': 'NCHW'},
            'outputs': [{'name': output.name, 'shape': shape(output), 'layout': 'NCHW',
                         'activation': 'sigmoid'} for output in model.graph.output]}
        shutil.copyfile(exported, args.output)
        shutil.copyfile(workspace / 'RK_anchors.txt', anchors_path)
        metadata_path.write_text(json.dumps(metadata, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(metadata, indent=2))
    print('Exported official RK ONNX and scene metadata. INT8 conversion and labeled accuracy validation remain required.')


if __name__ == '__main__':
    main()
