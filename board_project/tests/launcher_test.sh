#!/usr/bin/env bash
# Test both real launch scripts with a stand-in process; never open hardware.
set -euo pipefail
TEST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
TEST_TMP="$(mktemp -d /tmp/adas-launcher-test.XXXXXX)"
trap 'rm -rf -- "$TEST_TMP"' EXIT

new_fixture() {
  CASE_DIR="$TEST_TMP/$1"
  SOURCE_DIR="$CASE_DIR/source tree"
  OUTPUT_DIR="$SOURCE_DIR/build_adas"
  [[ "${2:-}" != external ]] || OUTPUT_DIR="$CASE_DIR/external output"
  mkdir -p "$OUTPUT_DIR" "$SOURCE_DIR/models/adas" "$SOURCE_DIR/resources" "$SOURCE_DIR/config"
  cp "$TEST_ROOT/run_adas.sh" "$SOURCE_DIR/run_adas.sh"
  # Read the exact template used by build.sh, without running a build.
  awk '/^  cat > .*<<.LAUNCHER./ { copying=1; next }
       copying && /^LAUNCHER$/ { exit }
       copying { print }' "$TEST_ROOT/build.sh" > "$OUTPUT_DIR/run_img_recv_demo.sh"
  [[ -s "$OUTPUT_DIR/run_img_recv_demo.sh" ]]
  local resource
  for resource in models resources config; do
    ln -s "$SOURCE_DIR/$resource" "$OUTPUT_DIR/$resource"
  done
  for resource in models/adas/manifest.json resources/style.qss config/adas.ini; do
    printf 'fixture\n' > "$SOURCE_DIR/$resource"
  done
  printf 'fixture runtime\n' > "$OUTPUT_DIR/librknnrt.so"
  cat > "$OUTPUT_DIR/img_recv_demo" <<'PROGRAM'
#!/bin/sh
set -eu
printf '%s\000' "$PWD" "${LD_LIBRARY_PATH-}" "${DISPLAY-__unset__}" "${XAUTHORITY-__unset__}" "$@" > "$LAUNCH_CAPTURE"
exit "${STUB_STATUS:-0}"
PROGRAM
  chmod +x "$SOURCE_DIR/run_adas.sh" "$OUTPUT_DIR/run_img_recv_demo.sh" "$OUTPUT_DIR/img_recv_demo"
  export LAUNCH_CAPTURE="$CASE_DIR/capture"
}

assert_capture() {
  local -a actual=()
  mapfile -d '' -t actual < "$LAUNCH_CAPTURE"
  [[ "${actual[0]}" == "$SOURCE_DIR" ]]
  [[ "${actual[1]}" == "$OUTPUT_DIR:/caller/lib one:/caller/lib two" ]]
  [[ "${actual[2]}" == :77 && "${actual[3]}" == '/caller/auth file' ]]
  [[ "${actual[4]}" == --mode && "${actual[5]}" == raw ]]
  [[ "${actual[6]}" == --report && "${actual[7]}" == 'relative report.json' ]]
  [[ "${actual[8]}" == '' && "${actual[9]}" == 'literal $value * "quotes"' ]]
  [[ "${#actual[@]}" == 10 ]]
}

launch_from_tmp() {
  (cd /tmp; DISPLAY=:77 XAUTHORITY='/caller/auth file' \
    LD_LIBRARY_PATH='/caller/lib one:/caller/lib two' \
    "$@" --mode raw --report 'relative report.json' '' 'literal $value * "quotes"')
  assert_capture
}

expect_failure() {
  if (cd /tmp; "$@") > "$CASE_DIR/error.log" 2>&1; then
    printf 'Expected launch failure: %s\n' "$*" >&2
    return 1
  fi
  [[ ! -e "$LAUNCH_CAPTURE" ]]
  grep -Fq '[launch] ERROR:' "$CASE_DIR/error.log"
}

test_root_direct_from_tmp_with_spaces() {
  new_fixture root-direct
  launch_from_tmp "$SOURCE_DIR/run_adas.sh"
}

test_root_bash_from_tmp_with_spaces() {
  new_fixture root-bash
  launch_from_tmp bash "$SOURCE_DIR/run_adas.sh"
}

test_root_sh_from_tmp_with_spaces() {
  new_fixture root-sh
  launch_from_tmp sh "$SOURCE_DIR/run_adas.sh"
}

test_generated_sh_with_external_output() {
  new_fixture external external
  launch_from_tmp sh "$OUTPUT_DIR/run_img_recv_demo.sh"
}

test_child_exit_status_and_unset_desktop_environment() {
  new_fixture child-status
  local status=0
  (cd /tmp; unset DISPLAY XAUTHORITY LD_LIBRARY_PATH; STUB_STATUS=37 "$SOURCE_DIR/run_adas.sh") || status=$?
  [[ "$status" == 37 ]]
  local -a actual=()
  mapfile -d '' -t actual < "$LAUNCH_CAPTURE"
  [[ "${actual[1]}" == "$OUTPUT_DIR" ]]
  [[ "${actual[2]}" == __unset__ && "${actual[3]}" == __unset__ ]]
}

test_missing_generated_launcher_has_build_instruction() {
  new_fixture no-launcher
  rm "$OUTPUT_DIR/run_img_recv_demo.sh"
  expect_failure sh "$SOURCE_DIR/run_adas.sh"
  grep -Fq "$OUTPUT_DIR/run_img_recv_demo.sh" "$CASE_DIR/error.log"
  grep -Fq 'build.sh' "$CASE_DIR/error.log"
}

test_missing_binary_has_build_instruction() {
  new_fixture no-binary
  rm "$OUTPUT_DIR/img_recv_demo"
  expect_failure "$SOURCE_DIR/run_adas.sh"
  grep -Fq "$OUTPUT_DIR/img_recv_demo" "$CASE_DIR/error.log"
  grep -Fq 'build.sh' "$CASE_DIR/error.log"
}

test_missing_build_resources_are_named() {
  local resource
  for resource in models resources config models/adas/manifest.json resources/style.qss config/adas.ini; do
    new_fixture "resource-${resource//\//-}"
    if [[ -L "$OUTPUT_DIR/$resource" ]]; then
      rm "$OUTPUT_DIR/$resource"
    else
      rm "$SOURCE_DIR/$resource"
    fi
    expect_failure "$SOURCE_DIR/run_adas.sh"
    grep -Fq "$OUTPUT_DIR/$resource" "$CASE_DIR/error.log"
  done
}

test_missing_bundled_runtime_is_named() {
  new_fixture no-runtime
  rm "$OUTPUT_DIR/librknnrt.so"
  expect_failure "$SOURCE_DIR/run_adas.sh"
  grep -Fq "$OUTPUT_DIR/librknnrt.so" "$CASE_DIR/error.log"
}

TEST_FAILED=0
TEST_COUNT=0
for TEST_NAME in test_root_direct_from_tmp_with_spaces \
    test_root_bash_from_tmp_with_spaces \
    test_root_sh_from_tmp_with_spaces \
    test_generated_sh_with_external_output \
    test_child_exit_status_and_unset_desktop_environment \
    test_missing_generated_launcher_has_build_instruction \
    test_missing_binary_has_build_instruction \
    test_missing_build_resources_are_named \
    test_missing_bundled_runtime_is_named; do
  TEST_COUNT=$((TEST_COUNT+1))
  set +e
  (set -e; "$TEST_NAME") > "$TEST_TMP/$TEST_NAME.log" 2>&1
  TEST_STATUS=$?
  set -e
  if [[ "$TEST_STATUS" == 0 ]]; then
    printf 'PASS %s\n' "$TEST_NAME"
  else
    TEST_FAILED=$((TEST_FAILED+1))
    printf 'FAIL %s\n' "$TEST_NAME"
    tail -n 8 "$TEST_TMP/$TEST_NAME.log"
  fi
done
printf 'Launcher tests: %s/%s passed\n' "$((TEST_COUNT-TEST_FAILED))" "$TEST_COUNT"
[[ "$TEST_FAILED" == 0 ]]
