#!/usr/bin/env bash
# CPU-only production-code tests; no Qt, driver, or RKNN execution.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$ROOT_DIR/build/host-tests"
[[ $# -le 1 ]] || { printf 'Usage: bash tests/run_host_tests.sh [video-directory]\n' >&2; exit 2; }
VIDEO_DIR=""
[[ $# == 0 ]] || VIDEO_DIR="$(realpath "$1")"
mkdir -p "$OUT"
exec > "$OUT/run.log" 2>&1
cd "$ROOT_DIR"
CXX="${CXX:-g++}"
read -r -a OPENCV_FLAGS <<< "$(pkg-config --cflags --libs opencv4)"
"$CXX" -std=c++17 -O2 -Isrc tests/adas_engine_test.cpp src/core/adas_engine.cpp -o "$OUT/adas_engine_test"
"$OUT/adas_engine_test"
"$CXX" -std=c++17 -O2 -Isrc tests/plate_text_codec_test.cpp src/common/plate_text_codec.cpp -o "$OUT/plate_text_codec_test"
"$OUT/plate_text_codec_test"
printf 'PASS: plate_text_codec_test (5 comparisons)\n'
"$CXX" -std=c++17 -O2 -Isrc tests/lane_nav_logic_test.cpp src/core/nav_arrow_detector.cpp \
  src/core/lane_line_analyzer.cpp -o "$OUT/lane_nav_logic_test" "${OPENCV_FLAGS[@]}"
"$OUT/lane_nav_logic_test"
if [[ -n "$VIDEO_DIR" ]]; then
  "$CXX" -std=c++17 -O2 -Isrc tests/adas_pipeline_e2e_test.cpp \
    src/core/lane_line_analyzer.cpp src/core/nav_arrow_detector.cpp src/core/adas_engine.cpp \
    -o "$OUT/adas_pipeline_e2e_test" "${OPENCV_FLAGS[@]}"
  "$OUT/adas_pipeline_e2e_test" "$VIDEO_DIR"
else
  printf 'SKIP: video E2E (supply generated video-directory to enable)\n'
fi
