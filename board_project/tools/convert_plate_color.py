#!/usr/bin/env python3
"""Convert PlateRecognition-main plate_recognition_color.onnx → RKNN (FP16) for RK3568.

Input:  1×3×48×168  NCHW, BGR (normalization baked into RKNN config)
Output: 78×21  (CTC) +  5  (color)

Normalization from PlateRecognition.cpp source:
  data[i] = ((pixel / 255.0) - 0.588) / 0.193
→ mean = 0.588 * 255 = 150  (per channel)
→ std  = 0.193 * 255 = 49.2 (per channel)
"""

import os, sys

ONNX_PATH = "/home/cat/code/FPGA/PlateRecognition-main/PlateRecognition/test/plate_recognition_color.onnx"
RKNN_PATH = "/home/cat/code/FPGA/high_speed_pcie/.codex/model_build/plate_recognition_color.rknn"

os.makedirs(os.path.dirname(RKNN_PATH), exist_ok=True)

from rknn.api import RKNN

rknn = RKNN(verbose=False)

try:
    print("[1/4] Configuring RKNN (FP16, rk3568, mean=150 std=49.2) …")
    ret = rknn.config(
        mean_values=[[150, 150, 150]],
        std_values=[[49.2, 49.2, 49.2]],
        target_platform="rk3568",
    )
    if ret != 0:
        sys.exit(f"rknn.config failed: {ret}")

    print("[2/4] Loading ONNX …")
    ret = rknn.load_onnx(model=ONNX_PATH, input_size_list=[[3, 48, 168]])
    if ret != 0:
        sys.exit(f"rknn.load_onnx failed: {ret}")

    print("[3/4] Building FP16 model …")
    ret = rknn.build(do_quantization=False)
    if ret != 0:
        sys.exit(f"rknn.build failed: {ret}")

    print(f"[4/4] Exporting → {RKNN_PATH}")
    ret = rknn.export_rknn(RKNN_PATH)
    if ret != 0:
        sys.exit(f"rknn.export_rknn failed: {ret}")

    size_mb = os.path.getsize(RKNN_PATH) / 1024 / 1024
    print(f"  SUCCESS  {size_mb:.2f} MB")

finally:
    rknn.release()
