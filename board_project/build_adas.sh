#!/usr/bin/env bash
# Compatibility entry point; build.sh owns dependency checks and output safety.
set -euo pipefail
ADAS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
exec "$ADAS_ROOT/build.sh" "$@"
