#!/usr/bin/env bash
set -euo pipefail
ADAS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LD_LIBRARY_PATH="$ADAS_ROOT/third_party/rknn/lib:$ADAS_ROOT/build_adas${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cd "$ADAS_ROOT"
exec "$ADAS_ROOT/build_adas/img_recv_demo" "$@"
