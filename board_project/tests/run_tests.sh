#!/usr/bin/env bash
set -euo pipefail
ADAS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ADAS_TEST_BUILD="${ADAS_TEST_BUILD_DIR:-$(mktemp -d /tmp/adas-tests.XXXXXX)}"
mkdir -p "$ADAS_TEST_BUILD"
cd "$ADAS_ROOT"
ADAS_APP_BUILD="${ADAS_APP_BUILD_DIR:-$ADAS_ROOT/build_adas}"
export LD_LIBRARY_PATH="$ADAS_ROOT/third_party/rknn/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
read -r -a ADAS_CV_FLAGS <<< "$(pkg-config --cflags --libs opencv4)"
read -r -a ADAS_QT_FLAGS <<< "$(pkg-config --cflags --libs Qt5Core)"
ADAS_CXX_FLAGS=(-std=c++17 -O0 -Wall -Wextra -fPIC -pthread -Isrc -Ithird_party)
if [[ "${ADAS_TEST_GUI_ONLY:-0}" != 1 ]]; then
g++ "${ADAS_CXX_FLAGS[@]}" tests/frame_pipeline_test.cpp "${ADAS_QT_FLAGS[@]}" -o "$ADAS_TEST_BUILD/frame_test"
"$ADAS_TEST_BUILD/frame_test"
g++ "${ADAS_CXX_FLAGS[@]}" tests/adas_engine_test.cpp src/core/adas_engine.cpp -o "$ADAS_TEST_BUILD/engine_test"
"$ADAS_TEST_BUILD/engine_test"
g++ "${ADAS_CXX_FLAGS[@]}" tests/road_geometry_test.cpp src/core/road_geometry.cpp src/core/ground_calibration.cpp "${ADAS_CV_FLAGS[@]}" -o "$ADAS_TEST_BUILD/geometry_test"
"$ADAS_TEST_BUILD/geometry_test"
g++ "${ADAS_CXX_FLAGS[@]}" tests/plate_text_codec_test.cpp src/common/plate_text_codec.cpp -o "$ADAS_TEST_BUILD/plate_test"
"$ADAS_TEST_BUILD/plate_test"
/usr/lib/qt5/bin/moc -Isrc src/inference/adas_pipeline.h -o "$ADAS_TEST_BUILD/moc_adas_pipeline.cpp"
g++ "${ADAS_CXX_FLAGS[@]}" tests/neural_pipeline_test.cpp src/inference/adas_pipeline.cpp src/core/ground_calibration.cpp "$ADAS_TEST_BUILD/moc_adas_pipeline.cpp" "${ADAS_QT_FLAGS[@]}" "${ADAS_CV_FLAGS[@]}" -o "$ADAS_TEST_BUILD/pipeline_test"
"$ADAS_TEST_BUILD/pipeline_test"
g++ "${ADAS_CXX_FLAGS[@]}" tests/neural_contract_test.cpp src/inference/neural_contract.cpp "${ADAS_QT_FLAGS[@]}" -o "$ADAS_TEST_BUILD/neural_contract_test"
"$ADAS_TEST_BUILD/neural_contract_test"
g++ "${ADAS_CXX_FLAGS[@]}" tests/neural_schedule_test.cpp -o "$ADAS_TEST_BUILD/neural_schedule_test"
"$ADAS_TEST_BUILD/neural_schedule_test"
python3 tests/model_bundle_test.py
/usr/lib/qt5/bin/moc -Isrc src/common/file_video_source.h -o "$ADAS_TEST_BUILD/moc_file_video_source.cpp"
g++ "${ADAS_CXX_FLAGS[@]}" tests/file_video_source_test.cpp src/common/file_video_source.cpp "$ADAS_TEST_BUILD/moc_file_video_source.cpp" "${ADAS_QT_FLAGS[@]}" "${ADAS_CV_FLAGS[@]}" -o "$ADAS_TEST_BUILD/file_test"
"$ADAS_TEST_BUILD/file_test"
fi
if [[ "${ADAS_TEST_GUI:-1}" == 1 ]]; then
  [[ -x "$ADAS_APP_BUILD/img_recv_demo" ]] || { echo 'Build the app before GUI tests; set ADAS_APP_BUILD_DIR for external output' >&2; exit 1; }
  read -r -a ADAS_GUI_FLAGS <<< "$(pkg-config --cflags --libs Qt5Widgets Qt5Network Qt5Test)"
  /usr/lib/qt5/bin/moc -Isrc tests/adas_ui_test.cpp -o "$ADAS_TEST_BUILD/adas_ui_test.moc"
  # Link only qmake's current object set; historical CPU geometry/replay
  # objects can remain in an incremental output directory after their removal.
  mapfile -t ADAS_OBJECTS < <(python3 - "$ADAS_APP_BUILD" <<'PYOBJECTS'
import pathlib,re,sys
root=pathlib.Path(sys.argv[1])
makefile=(root/'Makefile').read_text().replace('\\\n','')
match=re.search(r'^OBJECTS\s*=([^\n]*)',makefile,re.M)
if not match: raise SystemExit('qmake OBJECTS missing')
for name in match.group(1).split():
    if pathlib.Path(name).name!='main.o': print(root/name)
PYOBJECTS
  )
  [[ "${#ADAS_OBJECTS[@]}" -gt 0 ]] || { echo 'No qmake objects found' >&2; exit 1; }
  g++ "${ADAS_CXX_FLAGS[@]}" -I"$ADAS_TEST_BUILD" tests/adas_ui_test.cpp "${ADAS_OBJECTS[@]}" "${ADAS_GUI_FLAGS[@]}" "${ADAS_CV_FLAGS[@]}" -Lthird_party/rknn/lib -lrknnrt -ldl -o "$ADAS_TEST_BUILD/ui_test"
  QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}" "$ADAS_TEST_BUILD/ui_test"
fi
sha256sum -c .codex/fpga-baseline.sha256
printf 'All requested tests passed. Artifacts: %s\n' "$ADAS_TEST_BUILD"
