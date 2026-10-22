#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
BUILD_DIR="${BUILD_DIR:-$ROOT_DIR/build_adas}"
BUILD_JOBS="${BUILD_JOBS:-${ADAS_BUILD_JOBS:-2}}"
CHECK_ONLY=0
INSTALL_DEPS=0
CLEAN_BUILD=0
NO_RGA=0
BUILD_MARKER=.adas-build-root
REQUIRED_PACKAGES=(build-essential qtbase5-dev qt5-qmake pkg-config libopencv-dev binutils)
REQUIRED_FILES=(
  ZYNQ_video_show.pro
  third_party/rknn/include/rknn_api.h
  third_party/rknn/lib/librknnrt.so
  third_party/ztu/cv.hpp
  third_party/ztu/nn/structures.hpp
  third_party/ztu/rk/ez_rknn.hpp
  third_party/ztu/rk/rknn_dtype.hpp
  models/traffic_yolov5.rknn
  models/plate_yolov5.rknn
  models/plate_lprnet.rknn
  resources/style.qss
  config/adas.ini
)

log() { printf '[build] %s\n' "$*"; }
fail() { printf '[build] ERROR: %s\n' "$*" >&2; exit 1; }
dependency_error() {
  fail "$*; on Ubuntu/Debian run ./build.sh --install-deps (or install: ${REQUIRED_PACKAGES[*]})"
}
usage() {
  cat <<'HELP'
Usage: ./build.sh [--check] [--install-deps] [--clean] [--no-rga]
  --check         Check dependencies/assets/output safety, without building.
  --install-deps  Explicitly install missing Ubuntu/Debian build packages first.
  --clean         Remove an existing output only if this script marked it.
  --no-rga        Diagnostic/legacy build; neural ADAS reports RGA unavailable.
Environment: BUILD_DIR (default: source/build_adas), BUILD_JOBS (default: 2),
             ADAS_BUILD_JOBS (legacy fallback), QMAKE_BIN (Qt 5 qmake).
No driver, FPGA, model, or system RKNN installation is performed.
HELP
}
for argument in "$@"; do
  case "$argument" in
    --check) CHECK_ONLY=1 ;;
    --install-deps) INSTALL_DEPS=1 ;;
    --clean) CLEAN_BUILD=1 ;;
    --no-rga) NO_RGA=1 ;;
    -h|--help) usage; exit 0 ;;
    *) fail "Unknown argument: $argument (see --help)" ;;
  esac
done
[[ "$BUILD_JOBS" =~ ^[1-9][0-9]*$ ]] || fail "BUILD_JOBS must be a positive integer"

check_platform() {
  local arch bits
  arch="$(uname -m)"
  bits="$(getconf LONG_BIT)"
  case "$arch:$bits" in
    aarch64:64|arm64:64) ;;
    *) fail "Native 64-bit AArch64 Linux is required by the bundled RKNN library (detected $arch/$bits-bit)" ;;
  esac
  log "Platform: $arch, $bits-bit userspace"
}

check_output_directory() {
  command -v realpath >/dev/null 2>&1 || dependency_error "realpath (coreutils) is missing"
  local candidate="$BUILD_DIR" component protected
  [[ "$candidate" == /* ]] || candidate="$PWD/$candidate"
  # A slash after a directory symlink makes `test -L` inspect its target.
  # Strip it before walking components, including repeated trailing slashes.
  while [[ "$candidate" != / && "$candidate" == */ ]]; do candidate="${candidate%/}"; done
  component="$candidate"
  # Check before resolving: an alias must never make cleanup target another tree.
  while [[ "$component" != / ]]; do
    [[ ! -L "$component" ]] || fail "BUILD_DIR must not contain a symbolic link: $component"
    component="$(dirname -- "$component")"
  done
  BUILD_DIR="$(realpath -m -- "$candidate")"
  [[ "$ROOT_DIR" != "$BUILD_DIR" && "$ROOT_DIR" != "$BUILD_DIR/"* ]] ||
    fail "BUILD_DIR cannot be the source directory or its ancestor: $BUILD_DIR"
  case "$BUILD_DIR" in
    /|/home|/root|/tmp|/var|/bin|/bin/*|/sbin|/sbin/*|/boot|/boot/*|/etc|/etc/*|/usr|/usr/*|/lib|/lib/*|/lib64|/lib64/*|/dev|/dev/*|/proc|/proc/*|/sys|/sys/*|/run|/run/*|/var/lib|/var/lib/*|/var/log|/var/log/*|/var/cache|/var/cache/*)
      fail "Unsafe BUILD_DIR: $BUILD_DIR" ;;
  esac
  for protected in src third_party models resources config FPGA drivers .codex tests docs tools video_example build_release; do
    [[ "$BUILD_DIR" != "$ROOT_DIR/$protected" && "$BUILD_DIR" != "$ROOT_DIR/$protected/"* ]] ||
      fail "BUILD_DIR overlaps protected source/baseline content: $BUILD_DIR"
  done
  [[ ! -e "$BUILD_DIR" || -d "$BUILD_DIR" ]] || fail "BUILD_DIR is not a directory: $BUILD_DIR"
  if [[ -e "$BUILD_DIR/$BUILD_MARKER" || -L "$BUILD_DIR/$BUILD_MARKER" ]]; then
    [[ -f "$BUILD_DIR/$BUILD_MARKER" && ! -L "$BUILD_DIR/$BUILD_MARKER" ]] || fail "Invalid build marker"
    [[ "$(cat "$BUILD_DIR/$BUILD_MARKER")" == "$ROOT_DIR" ]] ||
      fail "Output belongs to another source tree; choose a fresh BUILD_DIR"
  elif [[ -d "$BUILD_DIR" ]]; then
    [[ "$CLEAN_BUILD" == 0 ]] || fail "Refusing --clean: directory has no $BUILD_MARKER marker: $BUILD_DIR"
    # Adopt the existing project build without deleting its objects or backup.
    [[ "$BUILD_DIR" == "$ROOT_DIR/build_adas" || -z "$(ls -A -- "$BUILD_DIR")" ]] ||
      fail "Unmarked BUILD_DIR is not empty; choose an empty directory"
  fi
}

verify_repo_assets() {
  local asset
  for asset in "${REQUIRED_FILES[@]}"; do
    [[ -s "$ROOT_DIR/$asset" ]] || fail "Required repository asset missing or empty: $asset"
  done
  if [[ ! -s "$ROOT_DIR/models/yolov5s_traffic.rknn" ]]; then
    log "Optional vehicle model models/yolov5s_traffic.rknn is absent; vehicle detection will be unavailable"
  fi
}

install_missing_packages() {
  local missing=() package
  [[ -r /etc/os-release ]] || fail "Cannot identify distro for --install-deps"
  # shellcheck disable=SC1091
  source /etc/os-release
  [[ "${ID:-}" == ubuntu || "${ID:-}" == debian || "${ID_LIKE:-}" == *debian* ]] ||
    fail "--install-deps supports Ubuntu/Debian only; install build dependencies manually"
  for package in "${REQUIRED_PACKAGES[@]}"; do
    if ! dpkg-query -W -f='${Status}' "$package" 2>/dev/null | grep -q 'install ok installed'; then
      missing+=("$package")
    fi
  done
  if [[ "${#missing[@]}" == 0 ]]; then log "Build packages are already installed"; return; fi
  local elevate=()
  if [[ "$EUID" != 0 ]]; then
    command -v sudo >/dev/null 2>&1 || fail "sudo is required to install build packages"
    elevate=(sudo)
  fi
  log "Explicit package installation: ${missing[*]}"
  "${elevate[@]}" apt-get update
  "${elevate[@]}" apt-get install -y "${missing[@]}"
}

verify_toolchain() {
  local tool target qt_version cv_version elf dependencies
  for tool in g++ make pkg-config readelf ldd; do
    command -v "$tool" >/dev/null 2>&1 || dependency_error "Missing build tool: $tool"
  done
  if [[ -n "${QMAKE_BIN:-}" ]]; then
    QMAKE_BIN="$(command -v "$QMAKE_BIN")" || dependency_error "QMAKE_BIN is not executable"
  elif command -v qmake >/dev/null 2>&1; then
    QMAKE_BIN="$(command -v qmake)"
  elif command -v qmake-qt5 >/dev/null 2>&1; then
    QMAKE_BIN="$(command -v qmake-qt5)"
  else
    dependency_error "Qt 5 qmake is missing"
  fi
  qt_version="$("$QMAKE_BIN" -query QT_VERSION)" || dependency_error "Cannot query qmake QT_VERSION"
  [[ "$qt_version" == 5.* ]] || dependency_error "Qt 5 is required (qmake reports $qt_version)"
  pkg-config --exists Qt5Widgets Qt5Network || dependency_error "Qt 5 Widgets/Network development files are missing"
  pkg-config --exists opencv4 || dependency_error "pkg-config cannot find opencv4"
  cv_version="$(pkg-config --modversion opencv4)"
  [[ "$cv_version" == 4.* ]] || dependency_error "OpenCV 4 is required (found $cv_version)"
  target="$(g++ -dumpmachine)"
  [[ "$target" == aarch64* ]] || fail "g++ must target AArch64 (found $target)"
  printf '#include <filesystem>\nstatic_assert(sizeof(void*) == 8);\nint main() { std::filesystem::path p; }\n' |
    g++ -std=c++17 -fsyntax-only -x c++ - || dependency_error "g++ cannot compile C++17/filesystem"
  elf="$(LC_ALL=C readelf -h "$ROOT_DIR/third_party/rknn/lib/librknnrt.so" 2>&1)" || fail "Invalid RKNN ELF library: $elf"
  if ! grep -q 'Class:.*ELF64' <<< "$elf" || ! grep -q 'Machine:.*AArch64' <<< "$elf"; then
    fail "Bundled RKNN library must be ELF64 AArch64"
  fi
  dependencies="$(LC_ALL=C ldd "$ROOT_DIR/third_party/rknn/lib/librknnrt.so" 2>&1)" ||
    fail "RKNN userspace dependencies are incompatible: $dependencies"
  ! grep -q 'not found' <<< "$dependencies" || fail "Missing RKNN userspace dependencies: $dependencies"
  log "Toolchain: $target, Qt $qt_version, OpenCV $cv_version; RKNN ELF/dependencies checked"
  # Match the current .pro feature gate, including all three headers used by C++.
  if [[ "$NO_RGA" == 0 ]]; then
    if [[ ! -r /usr/include/rga/rga.h || ! -r /usr/include/rga/RgaApi.h || ! -r /usr/include/rga/drmrga.h ]] ||
        ! printf '#include <rga/RgaApi.h>\n#include <rga/drmrga.h>\n#include <rga/rga.h>\n' |
          g++ -fsyntax-only -x c++ - >/dev/null 2>&1; then
      NO_RGA=1
    fi
  fi
  if [[ "$NO_RGA" == 1 ]]; then log "RGA: disabled (CONFIG+=no_rga); neural ADAS unavailable, no CPU image fallback";
  else log "RGA: headers found; neural ADAS requires hardware self-check (no CPU fallback)"; fi
}

build_project() {
  if [[ "$CLEAN_BUILD" == 1 && -d "$BUILD_DIR" ]]; then
    log "Cleaning marked build directory: $BUILD_DIR"
    rm -rf -- "$BUILD_DIR"
  fi
  mkdir -p -- "$BUILD_DIR"
  printf '%s\n' "$ROOT_DIR" > "$BUILD_DIR/$BUILD_MARKER"
  local resource destination relative
  for resource in models resources config; do
    destination="$BUILD_DIR/$resource"
    if [[ -e "$destination" || -L "$destination" ]]; then
      [[ -L "$destination" && "$(realpath -m -- "$destination")" == "$ROOT_DIR/$resource" ]] ||
        fail "Output resource path already exists; use a fresh output or --clean: $destination"
    else
      relative="$(realpath --relative-to="$BUILD_DIR" -- "$ROOT_DIR/$resource")"
      ln -s -- "$relative" "$destination"
    fi
  done
  local qmake_arguments=("$ROOT_DIR/ZYNQ_video_show.pro" CONFIG+=release)
  [[ "$NO_RGA" == 0 ]] || qmake_arguments+=(CONFIG+=no_rga)
  # qmake changes link objects/defines when optional RGA is toggled, but make
  # does not invalidate an object solely because its compiler flags changed.
  # Clean with the previous Makefile before generating the new configuration.
  if [[ -f "$BUILD_DIR/Makefile" ]]; then
    local previous_no_rga=1
    if grep -q '^DEFINES.*-DPANGO_HAVE_RGA' "$BUILD_DIR/Makefile"; then previous_no_rga=0; fi
    if [[ "$previous_no_rga" != "$NO_RGA" ]]; then
      log "RGA configuration changed; rebuilding objects with the new defines"
      (cd "$BUILD_DIR"; make clean)
    fi
  fi
  log "Building in $BUILD_DIR with $BUILD_JOBS jobs"
  (cd "$BUILD_DIR"; "$QMAKE_BIN" "${qmake_arguments[@]}"; make -j"$BUILD_JOBS")
  cp --remove-destination -- "$ROOT_DIR/third_party/rknn/lib/librknnrt.so" "$BUILD_DIR/librknnrt.so"
  cat > "$BUILD_DIR/run_img_recv_demo.sh" <<'LAUNCHER'
#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
MODEL_DIR="$(cd "$SCRIPT_DIR/models" && pwd -P)"
REPO_ROOT="$(dirname "$MODEL_DIR")"
export LD_LIBRARY_PATH="$SCRIPT_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cd "$REPO_ROOT"
exec "$SCRIPT_DIR/img_recv_demo" "$@"
LAUNCHER
  chmod +x "$BUILD_DIR/run_img_recv_demo.sh"
  log "Build finished. Launcher: $BUILD_DIR/run_img_recv_demo.sh"
}

check_platform
check_output_directory
verify_repo_assets
[[ "$INSTALL_DEPS" == 0 ]] || install_missing_packages
verify_toolchain
if [[ "$CHECK_ONLY" == 1 ]]; then
  log "Checks passed; no build directory created, cleaned, or compiled"
  exit 0
fi
build_project
