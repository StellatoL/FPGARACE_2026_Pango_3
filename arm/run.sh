#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${BUILD_DIR:-$ROOT_DIR/build/release}"
[[ "$BUILD_DIR" = /* ]] || BUILD_DIR="$ROOT_DIR/$BUILD_DIR"
BUILD_DIR="$(realpath -m "$BUILD_DIR")"
ARGS=("$@")
# Resolve file input before changing cwd for the application's relative QSS path.
if [[ $# -ge 2 && "$1" == --adas ]]; then
  ARGS[1]="$(realpath -m -- "$2")"
fi
[[ -x "$BUILD_DIR/img_recv_demo" ]] || { printf 'Build first: bash %s/build.sh\n' "$ROOT_DIR" >&2; exit 1; }
export LD_LIBRARY_PATH="$BUILD_DIR:$ROOT_DIR/third_party/rknn/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cd "$ROOT_DIR"
exec "$BUILD_DIR/img_recv_demo" "${ARGS[@]}"
