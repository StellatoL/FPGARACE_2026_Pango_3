#!/usr/bin/env bash
# Standalone offline acceptance tool; reuse the final application's core objects.
set -euo pipefail
scene_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
scene_build_dir="$scene_root/build_adas"
scene_mode=run
scene_args=()
scene_help() {
  cat <<'HELP'
Offline HDMI single-front diagnostics (production remains FPGA input only).
The complete 1280x720 canvas is the front view; no quadrant/mosaic split is used.
Schema3 executes Vehicle then Traffic for every sampled source frame. Both use
the full canvas, 640 letterbox and black model padding; only Vehicle is tracked.

Usage:
  tools/run_scene_diagnostics.sh --help
  tools/run_scene_diagnostics.sh --print-link --build-dir CLEAN_BUILD
  tools/run_scene_diagnostics.sh --build-only --build-dir CLEAN_BUILD
  tools/run_scene_diagnostics.sh --build-dir CLEAN_BUILD -- --input FILE --output DIR [OPTIONS]

Only the diagnostic translation unit is compiled. The existing clean build must
provide adas_engine.o, ground_calibration.o, road_geometry.o, neural_contract.o,
neural_models.o and rga_preprocessor.o. No UI objects or Qt Widgets are linked.
--print-link prints the exact compile/link command without compiling or inference.

Diagnostic options:
  --input FILE             Original real image or video.
  --output DIR             Results go under natural/ or controlled-drop-person-every-N/.
  --root DIR               Model/project root (default current directory).
  --sample-fps HZ          Video sampling, default 10; never duplicate a source frame.
  --max-frames N           Maximum sampled frames, default 120; 0 means full video.
  --overlay-every N        Save first frame and every Nth sampled frame, default 10.
  --commands LIST          go,force-drive,stop,nav-left,nav-straight,nav-right,
                           force-left,force-right,clear-nav,signal-left,signal-right.
                           Video may use media seconds: 0:go,1.2:nav-left.
  --expect LIST            Coarse presence checks: person,vehicle,light_red,light_green,
                           crosswalk,left-solid,right-dashed,etc.
  --drop-person-every N    Controlled deletion of real person detections every Nth frame.
  --signal-roi X,Y,W,H     All three light evidence ROIs in normalized canvas coordinates;
                           default 0,0,1,.5. Traffic infers the complete front canvas.
  --speed-kmh VALUE        Explicit simulated speed, default 0; never visual speed.
  --calibration FILE       Optional real 1280x720 calibration for measured objects only.
  --intersection          Explicit operator intersection state.
  --overwrite             Replace this mode's previous diagnostic output files.
  --continue-on-inference-error
                           Collect later real frames after a model error. Each failed
                           frame stays Failed and the final process still exits 3.

Output: frames.jsonl, frames.csv, commands.jsonl, summary.json, overlays/*.jpg.
JSON records actual RGA crops/letterbox rectangles and pre/post-NMS counts,
including failed frames, role metadata/hashes, separate errors and role statistics.
Traffic Crosswalk and Signal observations each ingest once without refreshing
Vehicle person/vehicle clocks. A saturated output remains Failed, never empty/Valid.
Photos ingest exactly once; commands only reevaluate the same observation clock.
Scene-level labels are not boxes/IDs/geometry ground truth; no mAP/MOTA is claimed.
HELP
}
while (($#)); do
  case "$1" in
    --help|-h) scene_help; exit 0 ;;
    --build-dir) [[ $# -ge 2 ]] || { echo '--build-dir needs a value' >&2; exit 2; }; scene_build_dir=$2; shift 2 ;;
    --print-link) scene_mode=print; shift ;;
    --build-only) scene_mode=build; shift ;;
    --) shift; scene_args=("$@"); break ;;
    *) echo "unknown wrapper option: $1 (diagnostic options follow --)" >&2; exit 2 ;;
  esac
done
scene_output="$scene_root/.codex/diagnostics/tool"
scene_binary="$scene_output/adas_scene_diagnostic"
scene_objects=()
for scene_name in adas_engine ground_calibration road_geometry neural_contract neural_models rga_preprocessor; do
  scene_objects+=("$scene_build_dir/$scene_name.o")
done
read -r -a scene_cflags <<<"$(pkg-config --cflags Qt5Core opencv4)"
read -r -a scene_qtlibs <<<"$(pkg-config --libs Qt5Core)"
scene_link=("${CXX:-g++}" -std=c++17 -O0 -Wall -Wextra -fPIC -pthread
  "${scene_cflags[@]}" -I"$scene_root/src" -I"$scene_root/third_party/rknn/include"
  "$scene_root/tools/adas_scene_diagnostic.cpp" "${scene_objects[@]}"
  -L"$scene_root/third_party/rknn/lib" -Wl,-rpath,"$scene_root/third_party/rknn/lib"
  -lrknnrt -ldl -lopencv_videoio -lopencv_imgcodecs -lopencv_imgproc -lopencv_core "${scene_qtlibs[@]}" -o "$scene_binary")
if [[ $scene_mode == print ]]; then
  printf '%q ' "${scene_link[@]}"
  printf '\n'
  exit 0
fi
for scene_object in "${scene_objects[@]}"; do
  [[ -f $scene_object ]] || { echo "missing core object: $scene_object; finish the clean production build first" >&2; exit 2; }
done
if [[ $scene_mode == run && ${#scene_args[@]} -eq 0 ]]; then scene_help; exit 2; fi
mkdir -p -- "$scene_output"
scene_relink=false
[[ -x $scene_binary ]] || scene_relink=true
for scene_source in "$scene_root/tools/adas_scene_diagnostic.cpp" "${scene_objects[@]}" \
  "$scene_root/src/core/view_tracker.h" "$scene_root/src/core/tracked_overlays.h" \
  "$scene_root/src/core/tracking_evidence.h" "$scene_root/src/core/manual_controls.h" \
  "$scene_root/src/core/adas_types.h" "$scene_root/src/inference/neural_contract.h" \
  "$scene_root/src/inference/neural_models.h" "$scene_root/src/inference/neural_scene_adapter.h" \
  "$scene_root/tools/run_scene_diagnostics.sh"; do
  [[ $scene_source -nt $scene_binary ]] && scene_relink=true
done
if [[ $scene_relink == true ]]; then "${scene_link[@]}"; fi
if [[ $scene_mode == build ]]; then printf '%s\n' "$scene_binary"; exit 0; fi
exec "$scene_binary" --root "$scene_root" --object-dir "$scene_build_dir" "${scene_args[@]}"
