#!/usr/bin/env bash
# Exercise the real build orchestration in throwaway source trees. make/apt are
# replaced at the process boundary so this never compiles the app or installs packages.
set -euo pipefail
TEST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
TEST_TMP="$(mktemp -d /tmp/adas-build-script-test.XXXXXX)"
trap 'rm -rf -- "$TEST_TMP"' EXIT
TEST_PATH="$PATH"

new_fixture() {
  CASE_DIR="$TEST_TMP/$1"
  SOURCE_DIR="$CASE_DIR/source tree"
  TOOL_DIR="$CASE_DIR/tools"
  mkdir -p "$SOURCE_DIR" "$TOOL_DIR"
  cp "$TEST_ROOT/build.sh" "$SOURCE_DIR/build.sh"
  chmod +x "$SOURCE_DIR/build.sh"
  local asset
  for asset in ZYNQ_video_show.pro third_party/rknn/include/rknn_api.h \
      models/adas/manifest.json models/adas/traffic.rknn models/adas/vehicle.rknn \
      drivers/pango_pci_driver_mul.ko resources/style.qss config/adas.ini; do
    mkdir -p "$SOURCE_DIR/$(dirname "$asset")"
    printf 'fixture\n' > "$SOURCE_DIR/$asset"
  done
  mkdir -p "$SOURCE_DIR/third_party/rknn/lib"
  ln -s "$TEST_ROOT/third_party/rknn/lib/librknnrt.so" "$SOURCE_DIR/third_party/rknn/lib/librknnrt.so"
  cat > "$TOOL_DIR/tool" <<'STUB'
#!/usr/bin/env bash
set -euo pipefail
case "$(basename "$0")" in
  uname) printf '%s\n' "${TEST_ARCH:-aarch64}" ;;
  getconf) printf '%s\n' "${TEST_BITS:-64}" ;;
  g++)
    if [[ "${1:-}" == -dumpmachine ]]; then
      printf '%s\n' "${TEST_CXX_TARGET:-aarch64-linux-gnu}"
    elif [[ "${TEST_CXX_FAIL:-0}" == 1 ]]; then
      exit 1
    else
      exec /usr/bin/g++ "$@"
    fi ;;
  pkg-config)
    [[ "${TEST_MISSING_CV:-0}" != 1 ]] || exit 1
    if [[ "${1:-}" == --modversion ]]; then
      if [[ "$*" == *opencv4* ]]; then printf '%s\n' "${TEST_CV_VERSION:-4.2.0}";
      else printf '5.12.8\n'; fi
    fi ;;
  qmake|custom-qmake)
    if [[ "${1:-}" == -query ]]; then
      printf '%s\n' "${TEST_QT_VERSION:-5.12.8}"
    else
      printf 'qmake %s\n' "$*" >> "$CASE_DIR/actions"
      : > Makefile
    fi ;;
  make)
    printf 'make %s\n' "$*" >> "$CASE_DIR/actions"
    if [[ "${1:-}" == clean ]]; then rm -f stale-rga.o; exit 0; fi
    cat > img_recv_demo <<'PROGRAM'
#!/usr/bin/env bash
set -euo pipefail
binary_dir="$(cd "$(dirname "$0")" && pwd -P)"
[[ -s "$binary_dir/resources/style.qss" && -s "$binary_dir/config/adas.ini" ]]
[[ -s "$binary_dir/models/adas/manifest.json" && -s "$binary_dir/models/adas/traffic.rknn" && -s "$binary_dir/models/adas/vehicle.rknn" ]]
[[ ":${LD_LIBRARY_PATH:-}:" == *":$binary_dir:"* ]]
printf '%s\n' "$PWD" "$@"
PROGRAM
    chmod +x img_recv_demo ;;
  ldd)
    if [[ "${TEST_MISSING_SDK_LIB:-0}" == 1 ]]; then printf 'libstdc++.so.6 => not found\n';
    else printf 'libc.so.6 => /lib/libc.so.6\n'; fi ;;
  dpkg-query)
    [[ "${TEST_MISSING_PACKAGE:-0}" != 1 ]] || exit 1
    printf 'install ok installed\n' ;;
  apt-get)
    printf 'apt-get %s\n' "$*" >> "$CASE_DIR/actions" ;;
  sudo) exec "$@" ;;
  nproc) printf '16\n' ;;
esac
STUB
  chmod +x "$TOOL_DIR/tool"
  local tool
  for tool in uname getconf g++ pkg-config qmake custom-qmake make ldd dpkg-query apt-get sudo nproc; do
    ln -s tool "$TOOL_DIR/$tool"
  done
  export CASE_DIR SOURCE_DIR TOOL_DIR
  export PATH="$TOOL_DIR:$TEST_PATH"
  unset BUILD_DIR BUILD_JOBS ADAS_BUILD_JOBS QMAKE_BIN TEST_ARCH TEST_BITS TEST_CXX_TARGET \
    TEST_CXX_FAIL TEST_MISSING_CV TEST_CV_VERSION TEST_QT_VERSION TEST_MISSING_SDK_LIB TEST_MISSING_PACKAGE
}

expect_failure() {
  if "$@" > "$CASE_DIR/error.log" 2>&1; then
    printf 'Expected rejection: %s\n' "$*" >&2
    return 1
  fi
}

test_script_is_directly_executable() {
  [[ -x "$TEST_ROOT/build.sh" ]]
  ! LC_ALL=C grep -q $'\r' "$TEST_ROOT/build.sh"
}

test_check_has_no_build_or_install_side_effects() {
  new_fixture check
  export BUILD_DIR="$CASE_DIR/absent output" TEST_MISSING_PACKAGE=1
  "$SOURCE_DIR/build.sh" --check
  [[ ! -e "$BUILD_DIR" && ! -e "$CASE_DIR/actions" ]]
}

test_missing_dependencies_do_not_install_implicitly() {
  new_fixture missing
  export TEST_MISSING_CV=1 TEST_MISSING_PACKAGE=1
  expect_failure "$SOURCE_DIR/build.sh"
  [[ ! -e "$CASE_DIR/actions" && ! -e "$SOURCE_DIR/build_adas" ]]
  grep -q -- '--install-deps' "$CASE_DIR/error.log"
}

test_rejects_wrong_platform_and_toolchain() {
  new_fixture platform
  export TEST_ARCH=armv7l
  expect_failure "$SOURCE_DIR/build.sh" --check
  grep -qi '64\|aarch64' "$CASE_DIR/error.log"
  unset TEST_ARCH
  export TEST_BITS=32
  expect_failure "$SOURCE_DIR/build.sh" --check
  unset TEST_BITS
  export TEST_CXX_TARGET=arm-linux-gnueabihf
  expect_failure "$SOURCE_DIR/build.sh" --check
  unset TEST_CXX_TARGET
  export TEST_QT_VERSION=6.8.0
  expect_failure "$SOURCE_DIR/build.sh" --check
  unset TEST_QT_VERSION
  export TEST_CV_VERSION=3.4.0
  expect_failure "$SOURCE_DIR/build.sh" --check
  unset TEST_CV_VERSION
  export TEST_CXX_FAIL=1
  expect_failure "$SOURCE_DIR/build.sh" --check
  [[ ! -e "$CASE_DIR/actions" ]]
}

test_rejects_missing_assets_and_incompatible_runtime() {
  new_fixture assets
  local asset
  for asset in models/adas/manifest.json models/adas/traffic.rknn models/adas/vehicle.rknn drivers/pango_pci_driver_mul.ko; do
    mv "$SOURCE_DIR/$asset" "$CASE_DIR/saved-asset"
    expect_failure "$SOURCE_DIR/build.sh" --check
    grep -Fq "$asset" "$CASE_DIR/error.log"
    mv "$CASE_DIR/saved-asset" "$SOURCE_DIR/$asset"
  done
  export TEST_MISSING_SDK_LIB=1
  expect_failure "$SOURCE_DIR/build.sh" --check
  grep -q 'libstdc++' "$CASE_DIR/error.log"
  unset TEST_MISSING_SDK_LIB
  rm "$SOURCE_DIR/third_party/rknn/lib/librknnrt.so"
  printf 'not an ELF library\n' > "$SOURCE_DIR/third_party/rknn/lib/librknnrt.so"
  expect_failure "$SOURCE_DIR/build.sh" --check
  [[ ! -e "$CASE_DIR/actions" ]]
}

test_default_build_is_incremental_and_preserves_release() {
  new_fixture default
  mkdir "$SOURCE_DIR/build_release" "$SOURCE_DIR/build_adas"
  printf 'release\n' > "$SOURCE_DIR/build_release/sentinel"
  printf 'incremental\n' > "$SOURCE_DIR/build_adas/sentinel"
  "$SOURCE_DIR/build.sh" --no-rga
  [[ -e "$SOURCE_DIR/build_release/sentinel" && -e "$SOURCE_DIR/build_adas/sentinel" ]]
  [[ -f "$SOURCE_DIR/build_adas/.adas-build-root" ]]
  grep -q '^make -j2$' "$CASE_DIR/actions"
  grep -q 'CONFIG+=no_rga' "$CASE_DIR/actions"
}

test_clean_requires_owned_safe_directory() {
  new_fixture clean
  export BUILD_DIR="$CASE_DIR/unmarked"
  mkdir "$BUILD_DIR"
  printf 'keep\n' > "$BUILD_DIR/sentinel"
  expect_failure "$SOURCE_DIR/build.sh" --clean
  [[ -e "$BUILD_DIR/sentinel" ]]
  export BUILD_DIR="$SOURCE_DIR"
  expect_failure "$SOURCE_DIR/build.sh" --clean
  export BUILD_DIR="$CASE_DIR"
  expect_failure "$SOURCE_DIR/build.sh" --clean
  export BUILD_DIR=/
  expect_failure "$SOURCE_DIR/build.sh" --clean
  export BUILD_DIR="$SOURCE_DIR/models/new-build"
  expect_failure "$SOURCE_DIR/build.sh" --clean
  export BUILD_DIR="$SOURCE_DIR/build_release"
  expect_failure "$SOURCE_DIR/build.sh" --clean
  # A valid marker must not hide a directory alias. Without it the old code
  # rejects for another reason and the trailing-slash regression goes untested.
  printf '%s\n' "$SOURCE_DIR" > "$CASE_DIR/unmarked/.adas-build-root"
  export BUILD_DIR="$CASE_DIR/link-output"
  ln -s "$CASE_DIR/unmarked" "$BUILD_DIR"
  expect_failure "$SOURCE_DIR/build.sh" --clean
  export BUILD_DIR="$CASE_DIR/link-output/"
  expect_failure "$SOURCE_DIR/build.sh" --clean
  export BUILD_DIR="$CASE_DIR/link-output//"
  expect_failure "$SOURCE_DIR/build.sh" --clean
  [[ -e "$CASE_DIR/unmarked/sentinel" && ! -e "$CASE_DIR/actions" ]]
  export BUILD_DIR="$CASE_DIR/owned"
  "$SOURCE_DIR/build.sh" --no-rga
  printf 'old\n' > "$BUILD_DIR/sentinel"
  "$SOURCE_DIR/build.sh" --clean --no-rga
  [[ ! -e "$BUILD_DIR/sentinel" && -x "$BUILD_DIR/img_recv_demo" ]]
}

test_external_output_launcher() {
  new_fixture external
  export BUILD_DIR="$CASE_DIR/external output" ADAS_BUILD_JOBS=3 QMAKE_BIN="$TOOL_DIR/custom-qmake"
  (cd /; "$SOURCE_DIR/build.sh" --no-rga)
  grep -q '^make -j3$' "$CASE_DIR/actions"
  [[ -s "$BUILD_DIR/librknnrt.so" ]]
  (cd /; "$BUILD_DIR/run_img_recv_demo.sh" --mode raw --report 'relative report.json') > "$CASE_DIR/launch.log"
  grep -Fx -- "$SOURCE_DIR" "$CASE_DIR/launch.log"
  grep -Fx -- raw "$CASE_DIR/launch.log"
  grep -Fx -- 'relative report.json' "$CASE_DIR/launch.log"
}

test_installation_requires_explicit_flag() {
  new_fixture install
  export TEST_MISSING_PACKAGE=1
  "$SOURCE_DIR/build.sh" --install-deps --check
  grep -q '^apt-get update$' "$CASE_DIR/actions"
  grep -q '^apt-get install -y ' "$CASE_DIR/actions"
  ! grep -q '^make\|^qmake' "$CASE_DIR/actions"
  [[ ! -e "$SOURCE_DIR/build_adas" ]]
}

test_switching_rga_rebuilds_old_objects() {
  new_fixture rga-switch
  mkdir "$SOURCE_DIR/build_adas"
  printf 'DEFINES = -DPANGO_HAVE_RGA\n' > "$SOURCE_DIR/build_adas/Makefile"
  touch "$SOURCE_DIR/build_adas/stale-rga.o"
  "$SOURCE_DIR/build.sh" --no-rga
  [[ ! -e "$SOURCE_DIR/build_adas/stale-rga.o" ]]
  grep -q '^make clean$' "$CASE_DIR/actions"
}

test_rejects_invalid_arguments_before_side_effects() {
  new_fixture arguments
  expect_failure "$SOURCE_DIR/build.sh" --unknown-option
  export BUILD_JOBS=0
  expect_failure "$SOURCE_DIR/build.sh"
  [[ ! -e "$CASE_DIR/actions" ]]
}

TEST_FAILED=0
TEST_COUNT=0
for TEST_NAME in test_script_is_directly_executable \
    test_check_has_no_build_or_install_side_effects \
    test_missing_dependencies_do_not_install_implicitly \
    test_rejects_wrong_platform_and_toolchain \
    test_rejects_missing_assets_and_incompatible_runtime \
    test_default_build_is_incremental_and_preserves_release \
    test_clean_requires_owned_safe_directory \
    test_external_output_launcher \
    test_installation_requires_explicit_flag \
    test_switching_rga_rebuilds_old_objects \
    test_rejects_invalid_arguments_before_side_effects; do
  TEST_COUNT=$((TEST_COUNT+1))
  set +e
  (set -e; "$TEST_NAME") > "$TEST_TMP/$TEST_NAME.log" 2>&1
  TEST_STATUS=$?
  set -e
  if [[ "$TEST_STATUS" -eq 0 ]]; then
    printf 'PASS %s\n' "$TEST_NAME"
  else
    TEST_FAILED=$((TEST_FAILED+1))
    printf 'FAIL %s\n' "$TEST_NAME"
    tail -n 12 "$TEST_TMP/$TEST_NAME.log"
  fi
done
printf 'Build script tests: %s/%s passed\n' "$((TEST_COUNT-TEST_FAILED))" "$TEST_COUNT"
[[ "$TEST_FAILED" -eq 0 ]]
