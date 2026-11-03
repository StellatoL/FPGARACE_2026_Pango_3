#!/bin/sh
set -eu
ADAS_ROOT="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd -P)"
LAUNCHER="$ADAS_ROOT/build_adas/run_img_recv_demo.sh"
if [ ! -x "$LAUNCHER" ]; then
  printf '[launch] ERROR: Build launcher missing or not executable: %s\n' "$LAUNCHER" >&2
  printf '[launch] Run "%s/build.sh" to prepare the build output.\n' "$ADAS_ROOT" >&2
  exit 1
fi
exec "$LAUNCHER" "$@"
