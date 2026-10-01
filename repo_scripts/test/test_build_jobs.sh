#!/bin/bash
# test_build_jobs.sh — 빌드 병렬도 계약.
#
# 대상: rt_common.sh 의 build_jobs_for / get_default_build_jobs / makeflags_* /
# resolve_build_makeflags / parse_common_args(-j), 그리고 그것을 쓰는 setup_env.sh ·
# build.sh · build_deps.sh 의 배선.
#
# 계약: 패키지는 한 번에 하나씩 빌드하고 (--parallel-workers 1) make job 수는
# MAKEFLAGS 의 -j 하나로 정한다. 기본값은 min(물리 코어, RAM / 4 GB) 이고 우선순위는
# CLI -j > RTC_BUILD_JOBS > 기존 MAKEFLAGS 의 -j > 기본값이다.
#
# 깨졌던 경로: colcon 기본값은 패키지 8개 × `make -j<논리 코어>` 였고 build.sh 의
# -j 는 패키지 수만 줄였다. 물리 16코어 / 32 GB 호스트가 빌드 중 메모리 고갈로
# 섰다 (무거운 TU 하나가 컴파일러에서 3.5 GB 까지 쓴다).
#
# 메모리 상한: build.sh · build_deps.sh 는 빌드를 MemoryMax (기본 RAM 의 75%) +
# MemorySwapMax=0 이 걸린 systemd user scope 에서 돌리고, scope 가 OOM 으로
# 정리된 실패를 평범한 빌드 실패와 구별해 알린다. user session 이 없으면 상한
# 없이 빌드한다. 여기서 systemd-run · systemctl 은 stub 이므로 **cgroup 이 실제로
# 한도를 강제하는지는 이 파일이 보지 못한다** — 고정하는 것은 넘기는 인자와 분기다.
#
# 테스트: 기본은 빌드하지 않는다 (--tests 로 켠다). build.sh 는 -DBUILD_TESTING 을
# 매 빌드마다 ON/OFF 로 명시한다.
# ccache: 깔려 있으면 쓰고 (--no-ccache 로 끔) launcher 도 매 빌드 명시한다.
#
# 격리 방식: setup_env.sh · build.sh 는 임시 가짜 워크스페이스 (<tmp>/ws/src/repo)
# 에서 돌린다 — lib 은 실물을 symlink 하고, colcon · cmake · systemd-run · systemctl ·
# ros2 · cset · sudo 와 CMake python 은 stub 이다. stub colcon 은 받은 인자와 MAKEFLAGS 를 파일에 남기고, 판정은
# 그 파일로 한다 (build.sh 가 출력하는 안내 문구가 아니라 colcon 이 실제로 받은 것).
# 물리 코어 수는 프로세스 경계를 넘어 shadow 할 수 없으므로 메모리 쪽을
# RTC_PROC_MEMINFO 로 조여 기대값이 nproc 과 달라지게 한다.
#
# 실행: ./test_build_jobs.sh   (exit 0 = PASS)
# colcon test가 ament_add_test로 자동 실행한다.
set -eu -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
LIB_DIR="${SCRIPT_DIR}/../scripts/lib"

_RT_LOG_PREFIX="test"
# shellcheck disable=SC1091
source "${LIB_DIR}/rt_common.sh"

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/lib/assert.sh"

TMP="$(mktemp -d)"
trap 'rm -rf -- "$TMP"' EXIT

GB_KB=$((1024 * 1024))

# 실제 호스트의 MemTotal 은 공칭보다 작다 — 공칭 N GB 의 97.7% 를 넣는다
# (이 repo 의 개발 PC: 공칭 32 GB → 32770940 kB).
nominal_gb_kb() { echo $(( $1 * GB_KB * 977 / 1000 )); }

write_meminfo() {  # $1=path $2=MemTotal kB
  printf 'MemTotal:       %s kB\nMemFree:         1234 kB\n' "$2" >"$1"
}

# ── build_jobs_for: 순수 산정식 ────────────────────────────────────────────
test_jobs_formula_is_min_of_cores_and_ram() {
  # 메모리가 묶는 경우 (보고된 호스트: 16C / 32 GB) 와 코어가 묶는 경우 (6C / 32 GB).
  expect_eq "16C/32GB -> RAM-bound" 8 "$(build_jobs_for 16 "$(nominal_gb_kb 32)")"
  expect_eq "6C/32GB -> core-bound" 6 "$(build_jobs_for 6 "$(nominal_gb_kb 32)")"
  expect_eq "4C/8GB" 2 "$(build_jobs_for 4 "$(nominal_gb_kb 8)")"
  expect_eq "32C/64GB" 16 "$(build_jobs_for 32 "$(nominal_gb_kb 64)")"
}

test_jobs_formula_never_returns_zero() {
  expect_eq "2C/2GB floors at 1" 1 "$(build_jobs_for 2 "$(nominal_gb_kb 2)")"
  expect_eq "1C/64GB" 1 "$(build_jobs_for 1 "$(nominal_gb_kb 64)")"
  expect_eq "cores unreadable" 1 "$(build_jobs_for "" "$(nominal_gb_kb 64)")"
  expect_eq "cores=0" 1 "$(build_jobs_for 0 "$(nominal_gb_kb 64)")"
}

test_jobs_formula_is_conservative_without_meminfo() {
  # 메모리를 못 읽으면 코어 수가 아니라 2 — 코어 수를 믿으면 그게 OOM 경로다.
  expect_eq "mem unreadable, 16C" 2 "$(build_jobs_for 16 "")"
  expect_eq "mem garbage, 16C" 2 "$(build_jobs_for 16 "n/a")"
  expect_eq "mem unreadable, 1C" 1 "$(build_jobs_for 1 "")"
}

test_default_jobs_reads_host_topology_and_meminfo() {
  local meminfo="$TMP/meminfo_default"
  write_meminfo "$meminfo" "$(nominal_gb_kb 32)"
  (
    eval 'get_physical_cores() { echo 16; }'
    RTC_PROC_MEMINFO="$meminfo" get_default_build_jobs
  ) >"$TMP/out_default"
  expect_eq "16C/32GB via host readers" 8 "$(cat "$TMP/out_default")"

  # 없는 meminfo → 보수적 기본값 (awk 가 실패해도 죽지 않는다).
  (
    eval 'get_physical_cores() { echo 16; }'
    RTC_PROC_MEMINFO="$TMP/does_not_exist" get_default_build_jobs
  ) >"$TMP/out_default"
  expect_eq "missing meminfo" 2 "$(cat "$TMP/out_default")"
}

# ── MAKEFLAGS 문자열 다루기 ────────────────────────────────────────────────
has_jobs() { if makeflags_has_jobs "$1"; then echo yes; else echo no; fi; }

test_makeflags_has_jobs_recognises_every_spelling() {
  expect_eq "empty" no "$(has_jobs "")"
  expect_eq "-j4" yes "$(has_jobs "-j4")"
  expect_eq "-j 4" yes "$(has_jobs "-k -j 4")"
  expect_eq "bare -j" yes "$(has_jobs "-j")"
  expect_eq "--jobs=4" yes "$(has_jobs "--jobs=4 -k")"
  expect_eq "--jobs 4" yes "$(has_jobs "--jobs 4")"
  # -l (load average) 은 job 수가 아니다 — 이것만 있으면 make 는 직렬로 돈다.
  expect_eq "-l only" no "$(has_jobs "-l8")"
  expect_eq "unrelated" no "$(has_jobs "-k --no-print-directory")"
}

test_makeflags_set_jobs_replaces_and_preserves() {
  expect_eq "append to empty" "-j6" "$(makeflags_set_jobs "" 6)"
  expect_eq "replace -jN" "-j6" "$(makeflags_set_jobs "-j12" 6)"
  expect_eq "replace '-j N'" "-k -j6" "$(makeflags_set_jobs "-j 12 -k" 6)"
  expect_eq "replace --jobs=N" "-k -l9 -j6" "$(makeflags_set_jobs "-k --jobs=12 -l9" 6)"
  expect_eq "replace '--jobs N'" "-j6" "$(makeflags_set_jobs "--jobs 12" 6)"
  # 숫자 없는 -j (무제한) 뒤의 플래그를 job 수로 먹지 않는다.
  expect_eq "bare -j keeps next flag" "-k -j6" "$(makeflags_set_jobs "-j -k" 6)"
  expect_eq "several -j collapse" "-j6" "$(makeflags_set_jobs "-j2 --jobs=3 -j 4" 6)"
}

test_makeflags_get_jobs() {
  expect_eq "none" "" "$(makeflags_get_jobs "-k")"
  expect_eq "-jN" 7 "$(makeflags_get_jobs "-k -j7")"
  expect_eq "-j N" 7 "$(makeflags_get_jobs "-j 7 -k")"
  expect_eq "--jobs=N" 7 "$(makeflags_get_jobs "--jobs=7")"
  expect_eq "--jobs N" 7 "$(makeflags_get_jobs "--jobs 7")"
  expect_eq "last wins" 7 "$(makeflags_get_jobs "-j3 -j7")"
  expect_eq "bare -j is unlimited" "" "$(makeflags_get_jobs "-j")"
}

# ── resolve_build_makeflags: 우선순위 ──────────────────────────────────────
# 네 출처에 서로 다른 수를 준다 (CLI 9 / env 5 / MAKEFLAGS 3 / 기본 2) — 같은 수면
# 어느 출처가 이겼는지 구별되지 않는다.
resolve_with() {  # $1=cli $2=RTC_BUILD_JOBS $3=MAKEFLAGS
  local meminfo="$TMP/meminfo_resolve" rc=0 out
  write_meminfo "$meminfo" "$(nominal_gb_kb 8)"
  out="$(
    eval 'get_physical_cores() { echo 16; }'
    MAKEFLAGS="$3" RTC_BUILD_JOBS="$2" RTC_PROC_MEMINFO="$meminfo" resolve_build_makeflags "$1"
  )" || rc=$?
  echo "rc=${rc} out=${out}"
}

test_resolve_precedence() {
  expect_eq "cli beats all" "rc=0 out=-k -j9" "$(resolve_with 9 5 "-k -j3")"
  expect_eq "env beats MAKEFLAGS" "rc=0 out=-k -j5" "$(resolve_with "" 5 "-k -j3")"
  expect_eq "user MAKEFLAGS kept verbatim" "rc=0 out=-k -j 3" "$(resolve_with "" "" "-k -j 3")"
  expect_eq "default when nothing set" "rc=0 out=-j2" "$(resolve_with "" "" "")"
  expect_eq "default keeps other flags" "rc=0 out=-k -l9 -j2" "$(resolve_with "" "" "-k -l9")"
}

test_resolve_rejects_non_positive_knobs() {
  local bad
  for bad in abc 0 -3 4x 1.5 "6 7"; do
    expect_eq "cli '$bad'" "rc=2 out=" "$(resolve_with "$bad" "" "-j3")"
    expect_eq "env '$bad'" "rc=2 out=" "$(resolve_with "" "$bad" "-j3")"
  done
}

# ── parse_common_args: -j 검증 ─────────────────────────────────────────────
test_parse_jobs_accepts_positive_int() {
  parse_common_args sim -j 8
  expect_eq "-j 8" 8 "$_COMMON_PARALLEL_JOBS"
  parse_common_args --jobs 3 robot
  expect_eq "--jobs 3" 3 "$_COMMON_PARALLEL_JOBS"
  parse_common_args sim
  expect_eq "absent" "" "$_COMMON_PARALLEL_JOBS"
}

test_parse_jobs_rejects_garbage() {
  local bad rc
  # `-j` 뒤에 다음 옵션이 오는 경우 (-j -c) 도 포함 — 이전에는 "-c" 를 job 수로 받았다.
  for bad in abc 0 -c ""; do
    rc=0
    (parse_common_args sim -j "$bad") >/dev/null 2>&1 || rc=$?
    expect_eq "-j '$bad' exits 1" 1 "$rc"
  done
  rc=0
  (parse_common_args sim -j) >/dev/null 2>&1 || rc=$?
  expect_eq "trailing -j exits 1" 1 "$rc"
}

# ── 가짜 워크스페이스 ───────────────────────────────────────────────────────
FAKE_WS="$TMP/ws"
FAKE_REPO="$FAKE_WS/src/repo"
STUB_BIN="$TMP/bin"
COLCON_LOG="$TMP/colcon.log"
CMAKE_LOG="$TMP/cmake.log"
SDRUN_LOG="$TMP/systemd-run.log"
SYSTEMCTL_LOG="$TMP/systemctl.log"

make_fake_workspace() {
  mkdir -p "$FAKE_REPO/repo_scripts/scripts" "$STUB_BIN"
  ln -s "$REPO_DIR/build.sh" "$FAKE_REPO/build.sh"
  ln -s "$REPO_DIR/repo_scripts/scripts/lib" "$FAKE_REPO/repo_scripts/scripts/lib"
  ln -s "$REPO_DIR/repo_scripts/scripts/setup_env.sh" "$FAKE_REPO/repo_scripts/scripts/setup_env.sh"
  ln -s "$REPO_DIR/repo_scripts/scripts/build_deps.sh" "$FAKE_REPO/repo_scripts/scripts/build_deps.sh"
  # build_deps.sh: 소스가 이미 import 된 것처럼 보이게 해 vcs 를 타지 않게 한다.
  mkdir -p "$FAKE_WS/deps/src/fmt" "$FAKE_WS/deps/src/mimalloc" "$FAKE_WS/deps/src/aligator/.git"

  # cmake (build_deps.sh): configure / build / install 호출을 한 줄씩 남긴다.
  printf '#!/bin/bash\necho "cmake $*" >>"%s"\n' "$CMAKE_LOG" >"$STUB_BIN/cmake"

  # colcon: 받은 것을 그대로 남긴다. 한 줄에 하나 — 인자 경계를 잃지 않게.
  cat >"$STUB_BIN/colcon" <<EOF
#!/bin/bash
{
  echo "MAKEFLAGS=\${MAKEFLAGS-<unset>}"
  printf 'ARG=%s\n' "\$@"
} >>"$COLCON_LOG"
exit "\${STUB_COLCON_RC:-0}"
EOF
  # systemd-run: 받은 줄을 남기고, 실물처럼 `--` 뒤의 명령을 같은 환경으로 exec
  # 한다. `--` 가 없는 호출은 build_mem_scope_prefix 의 probe (`… true`) 다.
  # STUB_SYSTEMD_RUN_RC≠0 은 user session 이 없는 호스트 (probe 부터 실패).
  cat >"$STUB_BIN/systemd-run" <<EOF
#!/bin/bash
echo "\$*" >>"$SDRUN_LOG"
[[ "\${STUB_SYSTEMD_RUN_RC:-0}" -ne 0 ]] && exit "\${STUB_SYSTEMD_RUN_RC}"
while [[ \$# -gt 0 ]]; do
  if [[ "\$1" == "--" ]]; then shift; exec "\$@"; fi
  shift
done
exit 0
EOF
  # systemctl: `show … -p Result --value` 에 scope 의 종료 사유를 답한다.
  cat >"$STUB_BIN/systemctl" <<EOF
#!/bin/bash
echo "\$*" >>"$SYSTEMCTL_LOG"
[[ "\$*" == *" show "* ]] && echo "\${STUB_SCOPE_RESULT:-success}"
exit 0
EOF
  # ros2: ensure_ros2_sourced 는 PATH 에 있는지만 본다.
  printf '#!/bin/bash\nexit 0\n' >"$STUB_BIN/ros2"
  # cset · sudo: auto_release_cpu_shield 가 격리된 호스트에서 실물을 부르지 않게.
  printf '#!/bin/bash\nexit 1\n' >"$STUB_BIN/cset"
  printf '#!/bin/bash\necho "sudo must not be reached: $*" >&2\nexit 1\n' >"$STUB_BIN/sudo"
  # CMake python: append_cmake_python_args 의 import 확인만 통과시킨다.
  printf '#!/bin/bash\nexit 0\n' >"$STUB_BIN/python-stub"
  chmod +x "$STUB_BIN"/*

  write_meminfo "$TMP/meminfo_ws" "$(nominal_gb_kb 8)"
}

# 깨끗한 환경에서 가짜 워크스페이스의 스크립트를 돌린다. ROS_DISTRO 를 미리 줘서
# setup_env.sh 가 호스트의 /opt/ros 를 source 하지 않게 한다 (CI 에는 없고, 있으면
# 느리다). $1.. = `VAR=value` 들, `--` 뒤가 명령.
run_clean() {
  local -a envs=()
  while [[ "$1" != "--" ]]; do envs+=("$1"); shift; done
  shift
  env -i PATH="$STUB_BIN:/usr/bin:/bin" HOME="$TMP" ROS_DISTRO=test \
    RTC_SYSTEM_PYTHON="$STUB_BIN/python-stub" RTC_PROC_MEMINFO="$TMP/meminfo_ws" \
    "${envs[@]}" "$@"
}

# 가짜 워크스페이스의 기대 기본값: 8 GB → RAM 이 2 로 묶는다. 1코어 호스트면 1.
expected_default_jobs() {
  build_jobs_for "$(get_physical_cores)" "$(nominal_gb_kb 8)"
}

# ── setup_env.sh ───────────────────────────────────────────────────────────
source_env() {  # $@ = `VAR=value` 들. source 뒤의 MAKEFLAGS 를 출력.
  # shellcheck disable=SC2016  # $1 · $MAKEFLAGS 는 자식 bash 가 펼친다
  run_clean "$@" -- bash -c 'source "$1" >/dev/null; echo "MAKEFLAGS=${MAKEFLAGS-<unset>}"' \
    _ "$FAKE_REPO/repo_scripts/scripts/setup_env.sh"
}

test_setup_env_exports_default_jobs() {
  local want
  want="$(expected_default_jobs)"
  expect_eq "plain shell" "MAKEFLAGS=-j${want}" "$(source_env 2>/dev/null)"
  expect_eq "other flags kept" "MAKEFLAGS=-k -j${want}" "$(source_env MAKEFLAGS=-k 2>/dev/null)"
}

test_setup_env_respects_explicit_choices() {
  expect_eq "user -j untouched" "MAKEFLAGS=-j3" "$(source_env MAKEFLAGS=-j3 2>/dev/null)"
  expect_eq "RTC_BUILD_JOBS wins" "MAKEFLAGS=-j5" "$(source_env MAKEFLAGS=-j3 RTC_BUILD_JOBS=5 2>/dev/null)"
}

test_setup_env_bad_knob_warns_and_falls_back() {
  local want err
  want="$(expected_default_jobs)"
  expect_eq "falls back to default" "MAKEFLAGS=-j${want}" "$(source_env RTC_BUILD_JOBS=abc 2>/dev/null)"
  err="$(source_env RTC_BUILD_JOBS=abc 2>&1 >/dev/null)"
  if [[ "$err" == *"RTC_BUILD_JOBS='abc'"* ]]; then pass; else fail "[bad knob warning] stderr='$err'"; fi
  # 잘못된 knob 이 사용자의 -j 를 지우지 않는다.
  expect_eq "bad knob keeps user -j" "MAKEFLAGS=-j3" "$(source_env MAKEFLAGS=-j3 RTC_BUILD_JOBS=0 2>/dev/null)"
}

test_setup_env_is_safe_to_source() {
  local out
  # build.sh 는 `set -eo pipefail` 아래에서 source 한다 — 여기서 죽으면 출력 없이 끝난다.
  # shellcheck disable=SC2016
  out="$(run_clean RTC_BUILD_JOBS=abc -- bash -c 'set -euo pipefail; source "$1" >/dev/null 2>&1; echo alive' \
    _ "$FAKE_REPO/repo_scripts/scripts/setup_env.sh")" || true
  expect_eq "survives set -e with a bad knob" alive "$out"
  # 두 번 source 해도 -j 가 쌓이지 않는다.
  # shellcheck disable=SC2016
  out="$(run_clean -- bash -c 'source "$1" >/dev/null; source "$1" >/dev/null; echo "$MAKEFLAGS"' \
    _ "$FAKE_REPO/repo_scripts/scripts/setup_env.sh")"
  expect_eq "idempotent" "-j$(expected_default_jobs)" "$out"
  # 헬퍼를 사용자 셸에 남기지 않는다.
  # shellcheck disable=SC2016
  out="$(run_clean -- bash -c 'source "$1" >/dev/null; declare -F resolve_build_makeflags get_physical_cores || echo none' \
    _ "$FAKE_REPO/repo_scripts/scripts/setup_env.sh")"
  expect_eq "no helper functions leak" none "$out"
}

# ── build.sh ───────────────────────────────────────────────────────────────
# stub colcon 이 남긴 기록에서 값을 꺼낸다.
logged_makeflags() { sed -n 's/^MAKEFLAGS=//p' "$COLCON_LOG"; }
logged_arg_after() {  # $1 = 옵션 이름 → 바로 다음 인자
  awk -v opt="ARG=$1" 'found {sub(/^ARG=/, ""); print; exit} $0 == opt {found=1}' "$COLCON_LOG"
}

run_build() {  # $1.. = `VAR=value` 들, `--` 뒤가 build.sh 인자. rc 를 출력.
  local -a envs=()
  while [[ "$1" != "--" ]]; do envs+=("$1"); shift; done
  shift
  local rc=0
  rm -f "$COLCON_LOG"
  run_clean "${envs[@]}" -- bash "$FAKE_REPO/build.sh" -p pkg_a,pkg_b "$@" >"$TMP/build.out" 2>&1 || rc=$?
  echo "$rc"
}

test_build_default_is_one_package_and_capped_jobs() {
  expect_eq "rc" 0 "$(run_build --)"
  expect_eq "one package at a time" 1 "$(logged_arg_after --parallel-workers)"
  expect_eq "make jobs" "-j$(expected_default_jobs)" "$(logged_makeflags)"
  expect_eq "packages reach colcon" pkg_a "$(logged_arg_after --packages-select)"
}

test_build_jobs_flag_sets_make_jobs_not_workers() {
  expect_eq "rc" 0 "$(run_build -- -j 9)"
  expect_eq "-j 9 -> make" "-j9" "$(logged_makeflags)"
  expect_eq "-j 9 leaves workers at 1" 1 "$(logged_arg_after --parallel-workers)"
}

test_build_knob_precedence() {
  expect_eq "rc" 0 "$(run_build RTC_BUILD_JOBS=5 MAKEFLAGS=-j3 --)"
  expect_eq "env beats MAKEFLAGS" "-j5" "$(logged_makeflags)"
  expect_eq "rc" 0 "$(run_build RTC_BUILD_JOBS=5 MAKEFLAGS=-j3 -- -j 9)"
  expect_eq "cli beats env" "-j9" "$(logged_makeflags)"
  expect_eq "rc" 0 "$(run_build MAKEFLAGS=-j3 --)"
  expect_eq "user MAKEFLAGS kept" "-j3" "$(logged_makeflags)"
}

test_build_rejects_bad_jobs_before_colcon() {
  expect_eq "-j abc" 1 "$(run_build -- -j abc)"
  if [[ -e "$COLCON_LOG" ]]; then fail "[-j abc] colcon was invoked"; else pass; fi
  expect_eq "-j 0" 1 "$(run_build -- -j 0)"
  if [[ -e "$COLCON_LOG" ]]; then fail "[-j 0] colcon was invoked"; else pass; fi
}

test_build_propagates_colcon_failure() {
  expect_eq "colcon rc 3 -> build.sh fails" 1 "$(run_build STUB_COLCON_RC=3 --)"
}

# ── 테스트 빌드 여부 ───────────────────────────────────────────────────────
# colcon 이 받은 --cmake-args 중 BUILD_TESTING 의 값 (없으면 빈 문자열).
logged_build_testing() { sed -n 's/^ARG=-DBUILD_TESTING=//p' "$COLCON_LOG"; }

test_build_does_not_build_tests_by_default() {
  expect_eq "rc" 0 "$(run_build --)"
  expect_eq "default says OFF" OFF "$(logged_build_testing)"
  # 기본 경로가 테스트를 빼는 것은 조용하면 안 된다 — 그 트리의 `colcon test` 는
  # 0개를 green 으로 보고한다.
  if grep -q "Tests: NOT built" "$TMP/build.out"; then pass; else fail "[default] did not say tests are skipped"; fi
}

test_build_tests_flag_and_env() {
  # 값은 **양쪽 다** 매 빌드 명시한다: CMake 가 캐시하므로, 한쪽을 생략하면 그
  # 방향으로는 직전 빌드의 선택이 남는다.
  expect_eq "rc" 0 "$(run_build -- --tests)"
  expect_eq "--tests says ON" ON "$(logged_build_testing)"
  expect_eq "rc" 0 "$(run_build RTC_BUILD_TESTS=on --)"
  expect_eq "RTC_BUILD_TESTS=on" ON "$(logged_build_testing)"
  expect_eq "rc" 0 "$(run_build RTC_BUILD_TESTS=on -- --no-tests)"
  expect_eq "--no-tests beats env" OFF "$(logged_build_testing)"
  expect_eq "rc" 0 "$(run_build RTC_BUILD_TESTS=off -- --tests)"
  expect_eq "--tests beats env" ON "$(logged_build_testing)"
  expect_eq "bad RTC_BUILD_TESTS" 1 "$(run_build RTC_BUILD_TESTS=maybe --)"
  if [[ -e "$COLCON_LOG" ]]; then fail "[bad RTC_BUILD_TESTS] colcon was invoked"; else pass; fi
}

# ── ccache ─────────────────────────────────────────────────────────────────
# colcon 이 받은 C / CXX 컴파일러 launcher. `c=<값> cxx=<값>`; 인자가 없으면 <none>.
logged_launchers() {
  local c cxx
  c="$(sed -n 's/^ARG=-DCMAKE_C_COMPILER_LAUNCHER=//p' "$COLCON_LOG")"
  cxx="$(sed -n 's/^ARG=-DCMAKE_CXX_COMPILER_LAUNCHER=//p' "$COLCON_LOG")"
  grep -q '^ARG=-DCMAKE_C_COMPILER_LAUNCHER=' "$COLCON_LOG" || c="<none>"
  grep -q '^ARG=-DCMAKE_CXX_COMPILER_LAUNCHER=' "$COLCON_LOG" || cxx="<none>"
  echo "c=${c} cxx=${cxx}"
}

# ccache 가 깔린 호스트를 흉내 낸다: PATH 앞에 stub ccache 를 둔 디렉토리를 더한다.
CCACHE_DIR_STUB="$TMP/ccache-bin"
run_build_with_ccache() {  # run_build 와 같되 ccache 가 PATH 에 있다.
  mkdir -p "$CCACHE_DIR_STUB"
  printf '#!/bin/bash\nexit 0\n' >"$CCACHE_DIR_STUB/ccache"
  chmod +x "$CCACHE_DIR_STUB/ccache"
  local -a envs=()
  while [[ "$1" != "--" ]]; do envs+=("$1"); shift; done
  shift
  run_build "PATH=$CCACHE_DIR_STUB:$STUB_BIN:/usr/bin:/bin" "${envs[@]}" -- "$@"
}

test_build_ccache_auto() {
  local host_ccache
  host_ccache="$(PATH="$STUB_BIN:/usr/bin:/bin" command -v ccache || true)"
  expect_eq "rc" 0 "$(run_build --)"
  if [[ -n "$host_ccache" ]]; then
    # 이 호스트에 ccache 가 깔려 있다 — auto 는 그 실물을 고른다.
    expect_eq "auto picks the installed ccache" "c=$host_ccache cxx=$host_ccache" "$(logged_launchers)"
  else
    # 없으면 빈 launcher 를 **명시적으로** 넘긴다 — 캐시에 남은 옛 값을 지우려고.
    expect_eq "absent -> explicit empty launcher" "c= cxx=" "$(logged_launchers)"
    expect_eq "--ccache without ccache fails" 1 "$(run_build -- --ccache)"
    if [[ -e "$COLCON_LOG" ]]; then fail "[--ccache absent] colcon was invoked"; else pass; fi
  fi

  expect_eq "rc" 0 "$(run_build_with_ccache --)"
  expect_eq "installed -> absolute path" \
    "c=$CCACHE_DIR_STUB/ccache cxx=$CCACHE_DIR_STUB/ccache" "$(logged_launchers)"
}

test_build_ccache_can_be_turned_off() {
  expect_eq "rc" 0 "$(run_build_with_ccache -- --no-ccache)"
  expect_eq "--no-ccache clears the launcher" "c= cxx=" "$(logged_launchers)"
  expect_eq "rc" 0 "$(run_build_with_ccache RTC_CCACHE=off --)"
  expect_eq "RTC_CCACHE=off" "c= cxx=" "$(logged_launchers)"
  expect_eq "rc" 0 "$(run_build_with_ccache RTC_CCACHE=off -- --ccache)"
  expect_eq "flag beats env" \
    "c=$CCACHE_DIR_STUB/ccache cxx=$CCACHE_DIR_STUB/ccache" "$(logged_launchers)"
  expect_eq "bad RTC_CCACHE" 1 "$(run_build_with_ccache RTC_CCACHE=maybe --)"
  if [[ -e "$COLCON_LOG" ]]; then fail "[bad RTC_CCACHE] colcon was invoked"; else pass; fi
}

# ── build_deps.sh ──────────────────────────────────────────────────────────
run_deps() {  # $@ = `VAR=value` 들. rc 를 출력.
  local rc=0
  rm -f "$CMAKE_LOG"
  run_clean "$@" -- bash "$FAKE_REPO/repo_scripts/scripts/build_deps.sh" >"$TMP/deps.out" 2>&1 || rc=$?
  echo "$rc"
}
# 세 dep (fmt · mimalloc · aligator) 의 `cmake --build … --parallel N` 에서 N 들.
logged_deps_jobs() {
  sed -n 's/^cmake --build .* --parallel \([0-9]*\)$/\1/p' "$CMAKE_LOG" | tr '\n' ' '
}

test_build_deps_uses_the_same_knob() {
  local want
  want="$(expected_default_jobs)"
  expect_eq "rc" 0 "$(run_deps)"
  expect_eq "default (was nproc)" "$want $want $want " "$(logged_deps_jobs)"
  expect_eq "rc" 0 "$(run_deps RTC_BUILD_JOBS=5 MAKEFLAGS=-j3)"
  expect_eq "RTC_BUILD_JOBS" "5 5 5 " "$(logged_deps_jobs)"
  expect_eq "rc" 0 "$(run_deps MAKEFLAGS=-j3)"
  expect_eq "user MAKEFLAGS" "3 3 3 " "$(logged_deps_jobs)"
  expect_eq "rc" 0 "$(run_deps PARALLEL_JOBS=4 RTC_BUILD_JOBS=5)"
  expect_eq "legacy PARALLEL_JOBS still wins" "4 4 4 " "$(logged_deps_jobs)"
}

test_build_deps_rejects_bad_jobs_before_cmake() {
  expect_eq "RTC_BUILD_JOBS=abc" 1 "$(run_deps RTC_BUILD_JOBS=abc)"
  if [[ -e "$CMAKE_LOG" ]]; then fail "[deps bad knob] cmake was invoked"; else pass; fi
  expect_eq "PARALLEL_JOBS=0" 1 "$(run_deps PARALLEL_JOBS=0)"
  if [[ -e "$CMAKE_LOG" ]]; then fail "[deps bad legacy knob] cmake was invoked"; else pass; fi
}

# ── 메모리 상한 ────────────────────────────────────────────────────────────
mem_max_with() {  # $1=RTC_BUILD_MEM_MAX ("" = 미설정) $2=meminfo 경로
  local rc=0 out
  out="$(RTC_BUILD_MEM_MAX="$1" RTC_PROC_MEMINFO="$2" get_build_mem_max)" || rc=$?
  echo "rc=${rc} out=${out}"
}

test_mem_max_default_is_three_quarters_of_ram() {
  local meminfo="$TMP/meminfo_cap"
  write_meminfo "$meminfo" 32770940  # 개발 PC 의 실제 MemTotal
  expect_eq "32 GB host" "rc=0 out=24002M" "$(mem_max_with "" "$meminfo")"
  # 메모리를 못 읽으면 상한을 지어내지 않는다 (0M 같은 값은 빌드를 즉시 죽인다).
  expect_eq "unreadable meminfo" "rc=0 out=" "$(mem_max_with "" "$TMP/does_not_exist")"
  # … 그리고 `set -e` 인 caller (build.sh) 를 죽이지도 않는다. 위 mem_max_with 는
  # `||` 의 왼쪽이라 errexit 가 꺼져 있어 이 경로를 보지 못한다.
  expect_eq "unreadable meminfo under set -e" "alive" \
    "$(set -e; RTC_PROC_MEMINFO="$TMP/does_not_exist"; get_build_mem_max; echo alive)"
}

test_mem_max_knob() {
  local meminfo="$TMP/meminfo_cap" v
  write_meminfo "$meminfo" 32770940
  for v in 12G 20000M 60% 8589934592; do
    expect_eq "accepts '$v'" "rc=0 out=$v" "$(mem_max_with "$v" "$meminfo")"
  done
  for v in 0 off none; do
    expect_eq "'$v' turns it off" "rc=0 out=" "$(mem_max_with "$v" "$meminfo")"
  done
  for v in banana 12GB -4G 1.5G "12 G" "%"; do
    expect_eq "rejects '$v'" "rc=2 out=" "$(mem_max_with "$v" "$meminfo")"
  done
}

# 가짜 워크스페이스의 기본 상한: MemTotal 8195670 kB (공칭 8 GB) 의 75%.
FAKE_WS_CAP="6002M"

run_build_capped() {  # run_build 와 같되 systemd 쪽 기록도 지운다.
  rm -f "$SDRUN_LOG" "$SYSTEMCTL_LOG"
  run_build "$@"
}
# colcon 을 감싼 systemd-run 호출 (probe 가 아닌 것) 한 줄.
logged_scope_line() { grep -- ' -- colcon build' "$SDRUN_LOG" 2>/dev/null || true; }

test_build_runs_colcon_inside_a_memory_scope() {
  local line
  expect_eq "rc" 0 "$(run_build_capped --)"
  line="$(logged_scope_line)"
  case "$line" in
    "--user --scope --quiet --unit=rtc-build-"*" -p MemoryMax=${FAKE_WS_CAP} -p MemorySwapMax=0 -- colcon build "*) pass ;;
    *) fail "[scope line] got '$line'" ;;
  esac
  # scope 안에서도 colcon 은 같은 인자와 환경을 받는다.
  expect_eq "colcon still gets workers" 1 "$(logged_arg_after --parallel-workers)"
  expect_eq "colcon still gets MAKEFLAGS" "-j$(expected_default_jobs)" "$(logged_makeflags)"
}

test_build_memory_cap_knob() {
  expect_eq "rc" 0 "$(run_build_capped RTC_BUILD_MEM_MAX=12G --)"
  case "$(logged_scope_line)" in
    *" -p MemoryMax=12G -p MemorySwapMax=0 -- colcon build "*) pass ;;
    *) fail "[cap knob] got '$(logged_scope_line)'" ;;
  esac

  expect_eq "rc (off)" 0 "$(run_build_capped RTC_BUILD_MEM_MAX=off --)"
  if [[ -e "$SDRUN_LOG" ]]; then fail "[cap off] systemd-run was invoked"; else pass; fi
  expect_eq "off still builds" pkg_a "$(logged_arg_after --packages-select)"

  expect_eq "bad cap" 1 "$(run_build_capped RTC_BUILD_MEM_MAX=banana --)"
  if [[ -e "$COLCON_LOG" ]]; then fail "[bad cap] colcon was invoked"; else pass; fi
}

test_build_without_user_session_builds_uncapped() {
  # probe 가 실패하는 호스트 (컨테이너 · CI): 빌드는 돌고, 상한이 없다고 알린다.
  expect_eq "rc" 0 "$(run_build_capped STUB_SYSTEMD_RUN_RC=1 --)"
  expect_eq "colcon ran" pkg_a "$(logged_arg_after --packages-select)"
  expect_eq "only the probe reached systemd-run" "" "$(logged_scope_line)"
  if grep -q "Memory cap unavailable" "$TMP/build.out"; then pass; else fail "[no session] no warning"; fi
}

test_build_reports_oom_distinctly() {
  # scope 가 OOM 으로 정리된 실패 — 원인과 두 knob 을 말한다.
  expect_eq "oom rc" 1 "$(run_build_capped STUB_COLCON_RC=143 STUB_SCOPE_RESULT=oom-kill --)"
  if grep -q "Build stopped: it needed more than the ${FAKE_WS_CAP} memory cap" "$TMP/build.out"; then
    pass
  else
    fail "[oom message] $(tail -2 "$TMP/build.out")"
  fi
  if grep -q "reset-failed rtc-build-" "$SYSTEMCTL_LOG"; then pass; else fail "[oom] failed unit left behind"; fi
  # 같은 exit code 라도 scope 가 멀쩡하면 평범한 빌드 실패다 (컴파일 에러를 메모리 탓으로 돌리지 않는다).
  expect_eq "plain failure rc" 1 "$(run_build_capped STUB_COLCON_RC=143 STUB_SCOPE_RESULT=success --)"
  if grep -q "Build stopped" "$TMP/build.out"; then fail "[plain failure] blamed memory"; else pass; fi
  if grep -q "Build failed" "$TMP/build.out"; then pass; else fail "[plain failure] no message"; fi
}

test_build_deps_runs_each_dep_inside_a_memory_scope() {
  rm -f "$SDRUN_LOG"
  expect_eq "rc" 0 "$(run_deps)"
  expect_eq "three capped builds" 3 "$(grep -c -- "-p MemoryMax=${FAKE_WS_CAP} -p MemorySwapMax=0 -- cmake --build" "$SDRUN_LOG")"
  expect_eq "builds still ran" 3 "$(grep -c '^cmake --build' "$CMAKE_LOG")"
  rm -f "$SDRUN_LOG"
  expect_eq "bad cap" 1 "$(run_deps RTC_BUILD_MEM_MAX=banana)"
  if [[ -e "$CMAKE_LOG" ]]; then fail "[deps bad cap] cmake was invoked"; else pass; fi
}

# ── fresh PC: 설치 경로 ────────────────────────────────────────────────────
host_summary() {  # $@ = `VAR=value` 들. print_build_host_summary 의 출력 + rc.
  local rc=0
  # shellcheck disable=SC2016
  run_clean "$@" -- bash -c 'source "$1" && print_build_host_summary' \
    _ "$FAKE_REPO/repo_scripts/scripts/lib/rt_common.sh" 2>&1 || rc=$?
  echo "rc=${rc}"
}

test_host_summary_reports_what_is_missing() {
  local out
  out="$(host_summary)"
  case "$out" in *"make -j$(expected_default_jobs) by default"*) pass ;; *) fail "[summary jobs] $out" ;; esac
  case "$out" in *"Memory cap: available, ${FAKE_WS_CAP}"*) pass ;; *) fail "[summary cap] $out" ;; esac
  case "$out" in *"rc=0") pass ;; *) fail "[summary rc] $out" ;; esac

  # user session 이 없는 호스트 — 빌드는 되지만 상한이 없다는 것을 말한다.
  out="$(host_summary STUB_SYSTEMD_RUN_RC=1)"
  case "$out" in *"Memory cap: NOT available"*"rc=0") pass ;; *) fail "[summary no session] $out" ;; esac

  # 틀린 knob 도 보고만 하고 설치를 멈추지 않는다 (build.sh 가 거부한다).
  out="$(host_summary RTC_BUILD_MEM_MAX=banana)"
  case "$out" in *"RTC_BUILD_MEM_MAX='banana' is not a size"*"rc=0") pass ;; *) fail "[summary bad cap] $out" ;; esac

  mkdir -p "$CCACHE_DIR_STUB"
  printf '#!/bin/bash\nexit 0\n' >"$CCACHE_DIR_STUB/ccache"
  chmod +x "$CCACHE_DIR_STUB/ccache"
  out="$(host_summary "PATH=$CCACHE_DIR_STUB:$STUB_BIN:/usr/bin:/bin")"
  case "$out" in *"ccache: $CCACHE_DIR_STUB/ccache"*) pass ;; *) fail "[summary ccache present] $out" ;; esac
  if ! PATH="$STUB_BIN:/usr/bin:/bin" command -v ccache >/dev/null; then
    out="$(host_summary)"
    case "$out" in *"ccache: not installed"*) pass ;; *) fail "[summary ccache absent] $out" ;; esac
  fi
}

# install.sh 의 setup_workspace: ccache 는 깔되, 못 깔아도 설치를 멈추지 않는다.
run_setup_workspace() {  # $1 = `apt-get install -y ccache` 의 exit code. sudo 호출 기록 + 출력 + rc.
  local rc=0 ccache_rc="$1" sudo_log="$TMP/sudo.log"
  rm -f "$sudo_log"
  (
    ROS_PKG_PREFIX="ros-test"
    apt_update_if_stale() { :; }
    # 호출은 파일에 남긴다 — setup_workspace 가 apt 의 stdout 을 /dev/null 로 보낸다.
    sudo() {
      echo "sudo $*" >>"$sudo_log"
      [[ "$*" == "apt-get install -y ccache" ]] && return "$ccache_rc"
      return 0
    }
    # shellcheck disable=SC1091
    source "${LIB_DIR}/install_ros2.sh"
    setup_workspace
  ) 2>&1 || rc=$?
  cat "$sudo_log" 2>/dev/null || true
  echo "rc=${rc}"
}

test_install_gets_ccache_but_does_not_require_it() {
  local out
  out="$(run_setup_workspace 0)"
  case "$out" in *"sudo apt-get install -y ccache"*) pass ;; *) fail "[install ccache] not requested: $out" ;; esac
  case "$out" in *"ccache installed"*"rc=0") pass ;; *) fail "[install ccache ok] $out" ;; esac
  # 필수 목록과 한 호출에 묶지 않는다 — 묶으면 universe 가 꺼진 호스트에서 빌드
  # 도구 전체가 설치되지 않는다.
  if grep -q "python3-colcon-common-extensions.*ccache\|ccache.*python3-colcon" <<<"$out"; then
    fail "[install ccache] bundled with the mandatory build tools"
  else
    pass
  fi

  out="$(run_setup_workspace 100)"
  case "$out" in *"ccache could not be installed"*"rc=0") pass ;; *) fail "[install ccache unavailable] $out" ;; esac
}

# ── Run ────────────────────────────────────────────────────────────────────
test_jobs_formula_is_min_of_cores_and_ram
test_jobs_formula_never_returns_zero
test_jobs_formula_is_conservative_without_meminfo
test_default_jobs_reads_host_topology_and_meminfo

test_makeflags_has_jobs_recognises_every_spelling
test_makeflags_set_jobs_replaces_and_preserves
test_makeflags_get_jobs

test_resolve_precedence
test_resolve_rejects_non_positive_knobs

test_parse_jobs_accepts_positive_int
test_parse_jobs_rejects_garbage

make_fake_workspace

test_setup_env_exports_default_jobs
test_setup_env_respects_explicit_choices
test_setup_env_bad_knob_warns_and_falls_back
test_setup_env_is_safe_to_source

test_build_default_is_one_package_and_capped_jobs
test_build_jobs_flag_sets_make_jobs_not_workers
test_build_knob_precedence
test_build_rejects_bad_jobs_before_colcon
test_build_propagates_colcon_failure
test_build_does_not_build_tests_by_default
test_build_tests_flag_and_env
test_build_ccache_auto
test_build_ccache_can_be_turned_off

test_build_deps_uses_the_same_knob
test_build_deps_rejects_bad_jobs_before_cmake

test_mem_max_default_is_three_quarters_of_ram
test_mem_max_knob
test_build_runs_colcon_inside_a_memory_scope
test_build_memory_cap_knob
test_build_without_user_session_builds_uncapped
test_build_reports_oom_distinctly
test_build_deps_runs_each_dep_inside_a_memory_scope

test_host_summary_reports_what_is_missing
test_install_gets_ccache_but_does_not_require_it

summary_and_exit test_build_jobs.sh
