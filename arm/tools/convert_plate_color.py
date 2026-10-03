#!/usr/bin/env python3
"""Convert external plate_recognition_color ONNX to RK3568 FP16 RKNN.

NCHW input 1x3x48x168, BGR; preserve mean=150 and std=49.2.
This optional conversion is not required to use the supplied model binaries.
"""
import argparse
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--onnx', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1] /
                        'build/model-conversion/plate_recognition_color.rknn')
    parser.add_argument('--overwrite', action='store_true')
    args = parser.parse_args()
    if not args.onnx.is_file():
        parser.error('Input ONNX does not exist')
    if args.output.exists() and not args.overwrite:
        parser.error('Output exists; use --overwrite to replace it')
    from rknn.api import RKNN
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rknn = RKNN(verbose=False)
    try:
        def check(result, stage):
            if result != 0:
                raise RuntimeError(f'{stage} failed: {result}')
        check(rknn.config(mean_values=[[150, 150, 150]], std_values=[[49.2, 49.2, 49.2]],
                          target_platform='rk3568'), 'config')
        check(rknn.load_onnx(model=str(args.onnx.resolve()), input_size_list=[[3, 48, 168]]), 'load_onnx')
        check(rknn.build(do_quantization=False), 'build')
        check(rknn.export_rknn(str(args.output.resolve())), 'export_rknn')
        print(f'Exported {args.output} ({args.output.stat().st_size} bytes)')
    finally:
        rknn.release()


if __name__ == '__main__':
    main()
