#!/usr/bin/env bash
# Native Linux AArch64 build. Never removes the selected output directory.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${BUILD_DIR:-$ROOT_DIR/build/release}"
[[ "$BUILD_DIR" = /* ]] || BUILD_DIR="$ROOT_DIR/$BUILD_DIR"
BUILD_DIR="$(realpath -m "$BUILD_DIR")"
MODE="${1:-build}"
case "$MODE" in
  build|--check|--stage-assets) ;;
  --help|-h)
    printf 'Usage: bash build.sh [--check|--stage-assets]\nBUILD_DIR is relative to arm/ (default: build/release).\n'
    exit 0 ;;
  *) printf 'Unknown option: %s\n' "$MODE" >&2; exit 2 ;;
esac
[[ $# -le 1 ]] || { printf 'Too many arguments\n' >&2; exit 2; }
fail() { printf '[build] ERROR: %s\n' "$*" >&2; exit 1; }
[[ "$BUILD_DIR" != "$ROOT_DIR" && "$BUILD_DIR" != / ]] || fail 'Use a separate build directory'
for asset in adas_arm.pro resources/style.qss third_party/rknn/include/rknn_api.h \
    third_party/rknn/lib/librknnrt.so third_party/ztu/cv.hpp \
    third_party/ztu/nn/structures.hpp third_party/ztu/rk/ez_rknn.hpp \
    third_party/ztu/rk/rknn_dtype.hpp models/traffic_yolov5.rknn \
    models/plate_yolov5.rknn models/plate_lprnet.rknn; do
  [[ -f "$ROOT_DIR/$asset" ]] || fail "Missing asset: $asset"
done
stage_assets() {
  mkdir -p "$BUILD_DIR/models"
  cp "$ROOT_DIR/third_party/rknn/lib/librknnrt.so" "$BUILD_DIR/"
  cp "$ROOT_DIR"/models/*.rknn "$BUILD_DIR/models/"
  printf '[build] Runtime and models staged: %s (no application execution)\n' "$BUILD_DIR"
}
if [[ "$MODE" == --stage-assets ]]; then stage_assets; exit 0; fi
[[ "$(uname -s)" == Linux ]] || fail 'Linux is required'
for tool in g++ make pkg-config; do
  command -v "$tool" >/dev/null || fail "Missing $tool; see README dependency commands"
done
QMAKE_BIN="${QMAKE_BIN:-}"
if [[ -z "$QMAKE_BIN" ]]; then
  QMAKE_BIN="$(command -v qmake || command -v qmake-qt5 || true)"
fi
[[ -n "$QMAKE_BIN" ]] || fail 'Qt 5 qmake is required'
[[ "$("$QMAKE_BIN" -query QT_VERSION)" == 5.* ]] || fail 'Select Qt 5 using QMAKE_BIN'
pkg-config --exists opencv4 || fail 'OpenCV 4 development package is required'
ARCH="$(uname -m)"
printf '[build] Assets/tools checked; host=%s, Qt=%s, OpenCV=%s\n' \
  "$ARCH" "$("$QMAKE_BIN" -query QT_VERSION)" "$(pkg-config --modversion opencv4)"
if [[ "$MODE" == --check ]]; then
  [[ "$ARCH" == aarch64 || "$ARCH" == arm64 ]] || printf '[build] Host checks only; this host cannot link/run the bundled AArch64 runtime.\n'
  exit 0
fi
[[ "$ARCH" == aarch64 || "$ARCH" == arm64 ]] || fail 'Native build requires AArch64 (64-bit ARM)'
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
QMAKE_ARGS=(CONFIG+=release)
[[ "${NO_RGA:-0}" != 1 ]] || QMAKE_ARGS+=(CONFIG+=no_rga)
"$QMAKE_BIN" "$ROOT_DIR/adas_arm.pro" "${QMAKE_ARGS[@]}"
make -j"${JOBS:-$(nproc)}"
stage_assets
printf '[build] Built %s/img_recv_demo\nStart using bash %s/run.sh (same BUILD_DIR).\n' "$BUILD_DIR" "$ROOT_DIR"
