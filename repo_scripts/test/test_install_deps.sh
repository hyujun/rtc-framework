#!/bin/bash
# test_install_deps.sh — install_deps.sh::install_onnxruntime 의 digest 검증 계약.
#
# 검증 대상 (#153 M8): GitHub 릴리즈 tarball 은 pin 된 sha256 과 일치할 때만
# 설치된다. 미검증 tarball 은 어떤 경로로도 /opt 에 도달하지 않는다.
#
# 격리 방식: install_onnxruntime 은 ONNXRT_DIR / ONNXRT_LIB_CONF 를 caller 가
# 덮어쓸 수 있으므로(MJ_DIR 선례) 실제 /opt·/etc 를 건드리지 않고 temp 트리에서
# 돈다. dpkg / apt-get / wget / uname / ldconfig / sudo 는 함수로 shadow 한다.
#
# ⚠ 다운로드·검증·추출은 install_onnxruntime 안의 **subshell** 에서 일어난다.
#   따라서 mock 의 호출 기록은 변수가 아니라 **파일**에 남겨야 한다 (subshell 의
#   변수 변경은 부모로 전파되지 않는다).
#
# 실행: ./test_install_deps.sh   (exit 0 = PASS)
# colcon test가 ament_add_test로 자동 실행한다.
set -eu -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB_DIR="${SCRIPT_DIR}/../scripts/lib"

PASS=0
FAIL=0
FAIL_MSGS=()

fail() { FAIL=$((FAIL+1)); FAIL_MSGS+=("$1"); }
pass() { PASS=$((PASS+1)); }

expect_eq() {
  # expect_eq "label" expected actual
  local label="$1" expected="$2" actual="$3"
  if [[ "$expected" == "$actual" ]]; then
    pass
  else
    fail "[$label] expected='$expected' actual='$actual'"
  fi
}

# ── Logger stubs (install_deps.sh 가 caller scope 에서 기대) ─────────────────
# subshell 안에서도 보이도록 파일에 적는다.
LOG_FILE=""
info()    { echo "INFO $*" >>"$LOG_FILE"; }
warn()    { echo "WARN $*" >>"$LOG_FILE"; }
success() { echo "OK $*"   >>"$LOG_FILE"; }
error()   { echo "ERR $*"  >>"$LOG_FILE"; return 1; }
section() { :; }

# shellcheck disable=SC1091
source "${LIB_DIR}/install_deps.sh"

# ── Command mocks ───────────────────────────────────────────────────────────
CALL_LOG=""
MOCK_ARCH="x86_64"
MOCK_DPKG_INSTALLED="false"
MOCK_APT_SUCCEEDS="false"
WGET_FIXTURE=""
WGET_SUCCEEDS="true"

uname() { echo "$MOCK_ARCH"; }

dpkg() {
  echo "dpkg $*" >>"$CALL_LOG"
  if [[ "$MOCK_DPKG_INSTALLED" == "true" ]]; then
    echo "Status: install ok installed"
    return 0
  fi
  return 1
}

# shellcheck disable=SC2317
apt-get() {
  echo "apt-get $*" >>"$CALL_LOG"
  [[ "$MOCK_APT_SUCCEEDS" == "true" ]]
}

# sudo 는 passthrough — tar/ln/tee 는 진짜로 돌아서 temp 트리에 결과를 남긴다.
sudo() { echo "sudo $*" >>"$CALL_LOG"; "$@"; }

ldconfig() { echo "ldconfig" >>"$CALL_LOG"; }

wget() {
  local out=""
  local url=""
  while (( $# )); do
    case "$1" in
      -O) out="$2"; shift 2 ;;
      -*) shift ;;
      *)  url="$1"; shift ;;
    esac
  done
  echo "wget ${url} -> ${out}" >>"$CALL_LOG"
  [[ "$WGET_SUCCEEDS" == "true" ]] || return 1
  cp -- "$WGET_FIXTURE" "$out"
}

called() {
  # called <pattern> → "true"/"false"
  grep -q -- "$1" "$CALL_LOG" && echo "true" || echo "false"
}

# ── Fixture ─────────────────────────────────────────────────────────────────
TEST_VER="9.9.9-test"   # 프로덕션 pin 과 섞이지 않도록 가짜 버전을 쓴다

make_fixture() {
  # $1=root $2=arch → stdout: fixture tarball 의 sha256
  local root="$1" arch="$2"
  local top="onnxruntime-linux-${arch}-${TEST_VER}"
  local stage="${root}/stage"
  mkdir -p "${stage}/${top}/lib" "${stage}/${top}/include"
  echo "fake-so"  >"${stage}/${top}/lib/libonnxruntime.so"
  echo "fake-hdr" >"${stage}/${top}/include/onnxruntime_cxx_api.h"
  tar -czf "${root}/fixture.tgz" -C "$stage" "$top"
  sha256sum "${root}/fixture.tgz" | awk '{print $1}'
}

TEST_ROOT=""
setup_case() {
  # 매 케이스마다 깨끗한 트리 + 로그
  TEST_ROOT="$(mktemp -d)"
  mkdir -p "${TEST_ROOT}/opt" "${TEST_ROOT}/etc" "${TEST_ROOT}/tmp"
  LOG_FILE="${TEST_ROOT}/log"
  CALL_LOG="${TEST_ROOT}/calls"
  : >"$LOG_FILE"
  : >"$CALL_LOG"
  MOCK_ARCH="x86_64"
  MOCK_DPKG_INSTALLED="false"
  MOCK_APT_SUCCEEDS="false"
  WGET_SUCCEEDS="true"
  WGET_FIXTURE="${TEST_ROOT}/fixture.tgz"
}

teardown_case() {
  rm -rf -- "$TEST_ROOT"
}

run_install() {
  # ONNXRT_DIR / ONNXRT_LIB_CONF / TMPDIR 를 temp 트리로 돌리고 호출한다.
  # TMPDIR 을 지정해야 mktemp -d 가 남긴 것이 없는지 확인할 수 있다.
  ONNXRT_VERSION="${1:-$TEST_VER}" \
  ONNXRT_DIR="${TEST_ROOT}/opt/onnxruntime" \
  ONNXRT_LIB_CONF="${TEST_ROOT}/etc/onnxruntime.conf" \
  TMPDIR="${TEST_ROOT}/tmp" \
    install_onnxruntime
}

tmp_leftover_count() {
  find "${TEST_ROOT}/tmp" -mindepth 1 -maxdepth 1 | wc -l
}

# ── Tests ───────────────────────────────────────────────────────────────────

test_x64_digest_match_installs() {
  setup_case
  local sha; sha=$(make_fixture "$TEST_ROOT" "x64")
  ONNXRT_SHA256["${TEST_VER}:x64"]="$sha"

  run_install

  expect_eq "x64.symlink_exists" "true" \
    "$([[ -L "${TEST_ROOT}/opt/onnxruntime" ]] && echo true || echo false)"
  expect_eq "x64.lib_resolves" "true" \
    "$([[ -f "${TEST_ROOT}/opt/onnxruntime/lib/libonnxruntime.so" ]] && echo true || echo false)"
  expect_eq "x64.header_resolves" "true" \
    "$([[ -f "${TEST_ROOT}/opt/onnxruntime/include/onnxruntime_cxx_api.h" ]] && echo true || echo false)"
  expect_eq "x64.ldconf_written" "${TEST_ROOT}/opt/onnxruntime/lib" \
    "$(cat "${TEST_ROOT}/etc/onnxruntime.conf")"
  expect_eq "x64.ldconfig_ran" "true" "$(called '^ldconfig$')"
  expect_eq "x64.temp_cleaned" "0" "$(tmp_leftover_count)"
  expect_eq "x64.url_arch" "true" "$(called 'onnxruntime-linux-x64-'"${TEST_VER}"'\.tgz')"

  unset 'ONNXRT_SHA256[${TEST_VER}:x64]'
  teardown_case
}

test_aarch64_digest_match_installs() {
  setup_case
  MOCK_ARCH="aarch64"
  local sha; sha=$(make_fixture "$TEST_ROOT" "aarch64")
  ONNXRT_SHA256["${TEST_VER}:aarch64"]="$sha"

  run_install

  expect_eq "aarch64.lib_resolves" "true" \
    "$([[ -f "${TEST_ROOT}/opt/onnxruntime/lib/libonnxruntime.so" ]] && echo true || echo false)"
  expect_eq "aarch64.url_arch" "true" \
    "$(called 'onnxruntime-linux-aarch64-'"${TEST_VER}"'\.tgz')"
  expect_eq "aarch64.temp_cleaned" "0" "$(tmp_leftover_count)"

  unset 'ONNXRT_SHA256[${TEST_VER}:aarch64]'
  teardown_case
}

test_digest_mismatch_refuses() {
  setup_case
  make_fixture "$TEST_ROOT" "x64" >/dev/null
  # 올바른 형태이지만 틀린 digest — upstream 자산 교체 / 변조 상황
  ONNXRT_SHA256["${TEST_VER}:x64"]="0000000000000000000000000000000000000000000000000000000000000000"

  run_install

  expect_eq "mismatch.no_symlink" "false" \
    "$([[ -e "${TEST_ROOT}/opt/onnxruntime" ]] && echo true || echo false)"
  # 미검증 tarball 이 추출조차 되지 않아야 한다
  expect_eq "mismatch.nothing_extracted" "0" \
    "$(find "${TEST_ROOT}/opt" -mindepth 1 -maxdepth 1 | wc -l)"
  expect_eq "mismatch.no_tar" "false" "$(called 'sudo tar')"
  expect_eq "mismatch.no_ldconfig" "false" "$(called '^ldconfig$')"
  expect_eq "mismatch.no_ldconf_file" "false" \
    "$([[ -f "${TEST_ROOT}/etc/onnxruntime.conf" ]] && echo true || echo false)"
  expect_eq "mismatch.temp_cleaned" "0" "$(tmp_leftover_count)"
  expect_eq "mismatch.warned" "true" \
    "$(grep -q "SHA256 mismatch" "$LOG_FILE" && echo true || echo false)"

  unset 'ONNXRT_SHA256[${TEST_VER}:x64]'
  teardown_case
}

test_unknown_arch_fails_closed_before_download() {
  setup_case
  MOCK_ARCH="armv7l"
  # ⚠ armv7l 용 digest 를 **일부러 등록**한다. 등록하지 않으면 arch 가드를
  #   지워도 digest 미등록 가드가 대신 잡아 이 테스트가 엉뚱한 이유로 green 이
  #   된다 (실제 mutation 에서 관측됨). digest 가 있으면 arch 가드만이 유일한
  #   방어선이므로, 그것을 지우면 armv7l tarball 이 설치돼 red 가 된다.
  local sha; sha=$(make_fixture "$TEST_ROOT" "armv7l")
  ONNXRT_SHA256["${TEST_VER}:armv7l"]="$sha"

  run_install

  expect_eq "unknown_arch.no_download" "false" "$(called 'wget ')"
  expect_eq "unknown_arch.no_symlink" "false" \
    "$([[ -e "${TEST_ROOT}/opt/onnxruntime" ]] && echo true || echo false)"
  expect_eq "unknown_arch.warned" "true" \
    "$(grep -q "unsupported architecture 'armv7l'" "$LOG_FILE" && echo true || echo false)"

  unset 'ONNXRT_SHA256[${TEST_VER}:armv7l]'
  teardown_case
}

test_missing_digest_fails_closed_before_download() {
  # install.sh 의 ONNXRT_VERSION 만 올리고 digest 를 잊은 회귀.
  # 옛 버전 digest 로 통과하는 대신 다운로드 전에 멈춰야 한다.
  setup_case
  make_fixture "$TEST_ROOT" "x64" >/dev/null

  run_install "1.99.0-unpinned"

  expect_eq "missing_digest.no_download" "false" "$(called 'wget ')"
  expect_eq "missing_digest.no_symlink" "false" \
    "$([[ -e "${TEST_ROOT}/opt/onnxruntime" ]] && echo true || echo false)"
  expect_eq "missing_digest.warned" "true" \
    "$(grep -q "no pinned SHA256 for '1.99.0-unpinned:x64'" "$LOG_FILE" && echo true || echo false)"
  teardown_case
}

test_download_failure_leaves_no_temp() {
  setup_case
  local sha; sha=$(make_fixture "$TEST_ROOT" "x64")
  ONNXRT_SHA256["${TEST_VER}:x64"]="$sha"
  WGET_SUCCEEDS="false"

  run_install

  expect_eq "dl_fail.no_symlink" "false" \
    "$([[ -e "${TEST_ROOT}/opt/onnxruntime" ]] && echo true || echo false)"
  expect_eq "dl_fail.no_tar" "false" "$(called 'sudo tar')"
  expect_eq "dl_fail.temp_cleaned" "0" "$(tmp_leftover_count)"

  unset 'ONNXRT_SHA256[${TEST_VER}:x64]'
  teardown_case
}

test_apt_installed_short_circuits() {
  # apt 경로는 digest 검증 대상이 아니다 (dpkg 서명 체인) — 다운로드 자체가 없다.
  setup_case
  MOCK_DPKG_INSTALLED="true"

  run_install

  expect_eq "apt_short.no_download" "false" "$(called 'wget ')"
  expect_eq "apt_short.reported" "true" \
    "$(grep -q "already installed (apt)" "$LOG_FILE" && echo true || echo false)"
  teardown_case
}

test_existing_opt_install_short_circuits() {
  # 이미 설치된 트리도 재검증 대상이 아니다 — 과거 설치 결과를 신뢰한다.
  setup_case
  mkdir -p "${TEST_ROOT}/opt/onnxruntime/lib" "${TEST_ROOT}/opt/onnxruntime/include"
  echo x >"${TEST_ROOT}/opt/onnxruntime/lib/libonnxruntime.so"
  echo x >"${TEST_ROOT}/opt/onnxruntime/include/onnxruntime_cxx_api.h"

  run_install

  expect_eq "opt_short.no_download" "false" "$(called 'wget ')"
  expect_eq "opt_short.no_dpkg_install" "false" "$(called 'apt-get install')"
  expect_eq "opt_short.reported" "true" \
    "$(grep -q "already installed at" "$LOG_FILE" && echo true || echo false)"
  teardown_case
}

test_production_pins_are_intact() {
  # TOFU pin 의 회귀 센서 — 리터럴 중복은 의도적이다. digest 를 조용히 바꾸면
  # 여기가 red 가 되어 "두 곳을 의식적으로 고치는" 절차를 강제한다.
  # 값 자체는 공식 릴리즈 자산에서 독립 재계산으로 확인했다 (#153 M8).
  expect_eq "pin.1.17.1:x64" \
    "89b153af88746665909c758a06797175ae366280cbf25502c41eb5955f9a555e" \
    "${ONNXRT_SHA256[1.17.1:x64]:-MISSING}"
  expect_eq "pin.1.17.1:aarch64" \
    "70b6f536bb7ab5961d128e9dbd192368ac1513bffb74fe92f97aac342fbd0ac1" \
    "${ONNXRT_SHA256[1.17.1:aarch64]:-MISSING}"

  # install.sh 가 지금 설치하려는 버전에 pin 이 실제로 존재하는가.
  # (버전만 올리고 digest 를 잊으면 런타임엔 skip, 여기선 red)
  local declared_ver
  declared_ver=$(grep -oP '^ONNXRT_VERSION="\K[^"]+' "${SCRIPT_DIR}/../../install.sh")
  expect_eq "pin.declared_version_x64_present" "true" \
    "$([[ -n "${ONNXRT_SHA256[${declared_ver}:x64]:-}" ]] && echo true || echo false)"
  expect_eq "pin.declared_version_aarch64_present" "true" \
    "$([[ -n "${ONNXRT_SHA256[${declared_ver}:aarch64]:-}" ]] && echo true || echo false)"
}

test_x64_digest_match_installs
test_aarch64_digest_match_installs
test_digest_mismatch_refuses
test_unknown_arch_fails_closed_before_download
test_missing_digest_fails_closed_before_download
test_download_failure_leaves_no_temp
test_apt_installed_short_circuits
test_existing_opt_install_short_circuits
test_production_pins_are_intact

echo
echo "── test_install_deps.sh summary ──"
echo "  PASS: $PASS"
echo "  FAIL: $FAIL"
if (( FAIL > 0 )); then
  printf '  %s\n' "${FAIL_MSGS[@]}"
  exit 1
fi
exit 0
