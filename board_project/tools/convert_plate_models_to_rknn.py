#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def build_exportable_pool(torch_module, kernel_size, stride):
    class Pool(torch_module.nn.Module):
        def __init__(self):
            super().__init__()
            channel_stride, height_stride, width_stride = stride
            _, kernel_h, kernel_w = kernel_size
            self.spatial_pool = torch_module.nn.MaxPool2d(
                kernel_size=(kernel_h, kernel_w),
                stride=(height_stride, width_stride),
            )
            self.channel_stride = channel_stride
            self.channel_pool = None
            if channel_stride > 1:
                self.channel_pool = torch_module.nn.MaxPool2d(
                    kernel_size=(1, 1),
                    stride=(1, channel_stride),
                )

        def forward(self, x):
            x = self.spatial_pool(x)
            if self.channel_pool is not None:
                x = x.permute(0, 3, 2, 1)
                x = self.channel_pool(x)
                x = x.permute(0, 3, 2, 1)
            return x

    return Pool()


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    default_lpr_repo = repo_root.parent / "YOLOv5-LPRNet-Licence-Recognition-master"
    default_build_dir = repo_root / ".codex" / "model_build"
    default_models_dir = repo_root / "models"

    parser = argparse.ArgumentParser(
        description="Export YOLOv5-LPRNet weights to ONNX and convert them to RKNN"
    )
    parser.add_argument("--lpr-repo", type=Path, default=default_lpr_repo)
    parser.add_argument("--build-dir", type=Path, default=default_build_dir)
    parser.add_argument("--models-dir", type=Path, default=default_models_dir)
    parser.add_argument("--target-platform", default="rk3568")
    parser.add_argument("--opset", type=int, default=11)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def ensure_import_path(path: Path) -> None:
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)


def load_python_deps(lpr_repo: Path):
    ensure_import_path(lpr_repo)
    from models.experimental import attempt_load  # type: ignore
    from models.LPRNet import CHARS, LPRNet  # type: ignore
    import torch  # type: ignore
    from rknn.api import RKNN  # type: ignore

    return attempt_load, CHARS, LPRNet, torch, RKNN


def export_detector_to_onnx(attempt_load, torch, weights: Path, onnx_path: Path,
                            opset: int, overwrite: bool) -> None:
    if onnx_path.exists() and not overwrite:
        print(f"[skip] detector ONNX exists: {onnx_path}")
        return

    model = attempt_load(str(weights), map_location="cpu")
    model = model.float().eval()
    model.model[-1].export = True

    dummy = torch.zeros((1, 3, 640, 640), dtype=torch.float32)
    onnx_path.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        model,
        dummy,
        str(onnx_path),
        verbose=False,
        opset_version=opset,
        do_constant_folding=True,
        input_names=["images"],
        output_names=["det_p3", "det_p4", "det_p5"],
    )
    print(f"[ok] detector ONNX exported: {onnx_path}")



def build_exportable_lprnet(LPRNet, CHARS, torch, weights: Path):
    model = LPRNet(lpr_max_len=8, phase=False, class_num=len(CHARS), dropout_rate=0)
    state = torch.load(str(weights), map_location="cpu")
    if isinstance(state, dict) and "state_dict" in state:
        state = state["state_dict"]
    model.load_state_dict(state)
    model = model.float().eval()

    model.backbone[3] = build_exportable_pool(torch, (1, 3, 3), (1, 1, 1))
    model.backbone[7] = build_exportable_pool(torch, (1, 3, 3), (2, 1, 2))
    model.backbone[14] = build_exportable_pool(torch, (1, 3, 3), (4, 1, 2))

    sample = torch.zeros((1, 3, 24, 94), dtype=torch.float32)
    logits = model(sample)
    if tuple(logits.shape) != (1, len(CHARS), 18):
        raise RuntimeError(f"unexpected LPRNet export shape: {tuple(logits.shape)}")
    return model



def export_lprnet_to_onnx(LPRNet, CHARS, torch, weights: Path, onnx_path: Path,
                          opset: int, overwrite: bool) -> None:
    if onnx_path.exists() and not overwrite:
        print(f"[skip] recognizer ONNX exists: {onnx_path}")
        return

    model = build_exportable_lprnet(LPRNet, CHARS, torch, weights)
    dummy = torch.zeros((1, 3, 24, 94), dtype=torch.float32)
    onnx_path.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        model,
        dummy,
        str(onnx_path),
        verbose=False,
        opset_version=opset,
        do_constant_folding=True,
        input_names=["images"],
        output_names=["logits"],
    )
    print(f"[ok] recognizer ONNX exported: {onnx_path}")



def convert_onnx_to_rknn(RKNN, onnx_path: Path, rknn_path: Path,
                         target_platform: str, mean_values, std_values,
                         overwrite: bool) -> None:
    if rknn_path.exists() and not overwrite:
        print(f"[skip] RKNN exists: {rknn_path}")
        return

    rknn = RKNN(verbose=False)
    try:
        ret = rknn.config(
            target_platform=target_platform,
            mean_values=[mean_values],
            std_values=[std_values],
        )
        if ret != 0:
            raise RuntimeError(f"rknn.config failed for {onnx_path} with code {ret}")

        ret = rknn.load_onnx(model=str(onnx_path))
        if ret != 0:
            raise RuntimeError(f"rknn.load_onnx failed for {onnx_path} with code {ret}")

        ret = rknn.build(do_quantization=False)
        if ret != 0:
            raise RuntimeError(f"rknn.build failed for {onnx_path} with code {ret}")

        rknn_path.parent.mkdir(parents=True, exist_ok=True)
        ret = rknn.export_rknn(str(rknn_path))
        if ret != 0:
            raise RuntimeError(f"rknn.export_rknn failed for {rknn_path} with code {ret}")

        print(f"[ok] RKNN exported: {rknn_path}")
    finally:
        rknn.release()



def main() -> int:
    args = parse_args()
    lpr_repo = args.lpr_repo.resolve()
    build_dir = args.build_dir.resolve()
    models_dir = args.models_dir.resolve()

    det_weights = lpr_repo / "weights" / "yolov5_best.pt"
    rec_weights = lpr_repo / "weights" / "lprnet_best.pth"
    det_onnx = build_dir / "plate_yolov5.onnx"
    rec_onnx = build_dir / "plate_lprnet.onnx"
    det_rknn = models_dir / "plate_yolov5.rknn"
    rec_rknn = models_dir / "plate_lprnet.rknn"

    missing = [path for path in (det_weights, rec_weights) if not path.exists()]
    if missing:
        raise FileNotFoundError("Missing source weights: " + ", ".join(str(path) for path in missing))

    attempt_load, CHARS, LPRNet, torch, RKNN = load_python_deps(lpr_repo)

    export_detector_to_onnx(attempt_load, torch, det_weights, det_onnx,
                            args.opset, args.overwrite)
    export_lprnet_to_onnx(LPRNet, CHARS, torch, rec_weights, rec_onnx,
                          args.opset, args.overwrite)

    convert_onnx_to_rknn(
        RKNN,
        det_onnx,
        det_rknn,
        args.target_platform,
        mean_values=[0, 0, 0],
        std_values=[255, 255, 255],
        overwrite=args.overwrite,
    )
    convert_onnx_to_rknn(
        RKNN,
        rec_onnx,
        rec_rknn,
        args.target_platform,
        mean_values=[127.5, 127.5, 127.5],
        std_values=[128.0, 128.0, 128.0],
        overwrite=args.overwrite,
    )

    manifest = {
        "detector_weights": str(det_weights),
        "recognizer_weights": str(rec_weights),
        "detector_onnx": str(det_onnx),
        "recognizer_onnx": str(rec_onnx),
        "detector_rknn": str(det_rknn),
        "recognizer_rknn": str(rec_rknn),
        "target_platform": args.target_platform,
        "opset": args.opset,
        "detector_normalization": {"mean": [0, 0, 0], "std": [255, 255, 255]},
        "recognizer_normalization": {"mean": [127.5, 127.5, 127.5], "std": [128.0, 128.0, 128.0]},
    }
    manifest_path = build_dir / "plate_model_manifest.json"
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(f"[ok] manifest written: {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
