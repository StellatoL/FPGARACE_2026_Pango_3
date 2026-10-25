#!/usr/bin/env python3
"""Package already-trained INT8 RKNN weights; never treats bundled legacy weights as final."""
import argparse
import hashlib
import json
import math
import shutil
from pathlib import Path

REQUIRED_CLASSES = {'person', 'vehicle', 'crosswalk', 'light_red', 'light_green',
                    'cone', 'barrier', 'debris'}


def prepare(scene, scene_metadata, lane, lane_metadata, output, version, toolkit, runtime):
    sm = json.loads(Path(scene_metadata).read_text(encoding='utf-8'))
    if (lane is None) != (lane_metadata is None):
        raise ValueError('provide both lane and lane-metadata, or neither')
    lm = json.loads(Path(lane_metadata).read_text(encoding='utf-8')) if lane is not None else None
    names = sm.get('classes', [])
    if not REQUIRED_CLASSES <= set(names) or len(set(names)) != len(names) or len(names) > 64:
        raise ValueError('scene labels must contain all mandatory classes without duplicates')
    if sm.get('output') not in ('yolov5_raw_nchw_v1', 'yolov5_rk_sigmoid_nchw_v1') or (lm is not None and lm.get('output') != 'lane_rows_4x96_v1'):
        raise ValueError('unsupported tensor contracts')
    if (sm.get('width'), sm.get('height')) != (640, 640) or (lm is not None and (lm.get('width'), lm.get('height')) != (512, 256)):
        raise ValueError('model dimensions must be scene640x640 and lane512x256')
    anchors = sm.get('anchors', [])
    if len(anchors) != 3 or any(len(row) != 6 or any(not math.isfinite(v) or not 0 < v <= 1280 for v in row) for row in anchors):
        raise ValueError('three valid pixel-unit anchor heads are required')
    roi = lm.get('road_roi', []) if lm is not None else [0, .42, 1, .58]
    if len(roi) != 4 or not all(math.isfinite(v) for v in roi) or min(roi[:2]) < 0 or min(roi[2:]) <= 0 or roi[0]+roi[2] > 1 or roi[1]+roi[3] > 1:
        raise ValueError('invalid road ROI')
    if not version.strip() or not toolkit.strip() or not runtime.strip():
        raise ValueError('version/toolkit/runtime metadata are required')
    sources = [Path(scene).resolve()] + ([Path(lane).resolve()] if lane is not None else [])
    if any(not source.is_file() or not 0 < source.stat().st_size <= 512*1024*1024 for source in sources):
        raise ValueError('nonempty bounded RKNN files are required')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    if any(output.iterdir()):
        raise ValueError('choose an empty output directory; existing model bundles are not overwritten')
    manifest = {'schema': 2, 'target': 'rk3568', 'version': version,
                'toolkit_version': toolkit, 'road_roi': roi, 'lane': None}
    for kind, source, meta in zip(['scene', 'lane'], sources, [sm, lm]):
        destination = output / (kind + '.rknn')
        shutil.copyfile(source, destination)
        item = {'file': destination.name, 'sha256': hashlib.sha256(destination.read_bytes()).hexdigest(),
                'quantization': 'int8', 'runtime_api_prefix': runtime,
                'input': {'width': meta['width'], 'height': meta['height'], 'layout': 'NHWC',
                          'format': 'RGB8', 'mean': [0, 0, 0], 'std': [255, 255, 255]},
                'output': meta['output']}
        if kind == 'scene':
            item.update(classes=names, anchors=anchors)
        manifest[kind] = item
    (output / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    return manifest


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for flag in ['scene', 'scene-metadata', 'output']:
        p.add_argument('--'+flag, required=True, type=Path)
    for flag in ['lane', 'lane-metadata']:
        p.add_argument('--'+flag, type=Path, help='optional paired lane model; absent stays unavailable')
    for flag in ['version', 'toolkit-version', 'runtime-api-prefix']:
        p.add_argument('--'+flag, required=True)
    a = p.parse_args()
    try:
        prepare(a.scene, a.scene_metadata, a.lane, a.lane_metadata, a.output,
                a.version, a.toolkit_version, a.runtime_api_prefix)
    except (OSError, ValueError, TypeError) as exc:
        p.error(str(exc))
    print('Bundle created. ARM validates SHA256, hardware, SDK and tensor contract before enabling perception.')


if __name__ == '__main__':
    main()
