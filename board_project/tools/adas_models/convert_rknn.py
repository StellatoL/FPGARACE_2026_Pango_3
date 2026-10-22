#!/usr/bin/env python3
"""PC RKNN-Toolkit2 converter. No ARM driver/runtime upgrade is performed."""
import argparse
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--onnx', required=True, type=Path)
    p.add_argument('--calibration', required=True, type=Path,
                   help='Toolkit2 calibration list, representative actual FPGA RGB565 appearance')
    p.add_argument('--output', required=True, type=Path)
    args = p.parse_args()
    if not args.onnx.is_file() or not args.calibration.is_file():
        p.error('ONNX and representative calibration list must exist')
    from rknn.api import RKNN
    model = RKNN(verbose=True)
    try:
        model.config(target_platform='rk3568', mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]])
        for name, action in [('load', lambda: model.load_onnx(model=str(args.onnx))),
                             ('build INT8', lambda: model.build(do_quantization=True, dataset=str(args.calibration))),
                             ('export', lambda: model.export_rknn(str(args.output)))]:
            code = action()
            if code != 0:
                raise RuntimeError(f'{name} failed: {code}')
    finally:
        model.release()
    print('Converted for RK3568. Review compiler operator placement, compare ONNX/RKNN outputs, then benchmark on board.')


if __name__ == '__main__':
    main()
