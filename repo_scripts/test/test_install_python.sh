#!/bin/bash
# test_install_python.sh — install_python.sh::ensure_venv 의 venv base 계약.
#
# 검증 대상: ensure_venv 가 끝나면 workspace .venv 는 RTC_SYSTEM_PYTHON (apt 배포판
# python) 을 base 로, --system-site-packages 로 만들어져 있다. 그래야 ROS 가 apt 로
# 까는 python 모듈 (catkin_pkg · yaml · rclpy 가 쓰는 것들) 이 venv 에서 보인다.
#
# 깨졌던 두 경로 (fresh PC 에서 rtc_base configure 가 catkin_pkg ImportError 로 죽었다):
#   1. `uv venv --python 3.12` — uv 기본값 (python-preference=managed) 이 uv-managed
#      3.12 를 base 로 골랐다. 그 venv 는 /usr/lib/python3/dist-packages 를 못 본다.
#   2. setup_env.sh 가 기존 .venv 를 먼저 활성화하므로 ensure_venv 가 "already active"
#      로 곧장 빠져 재생성 검사에 도달하지 못했다 — install.sh 를 다시 돌려도 안 고쳐졌다.
#
# 격리 방식: uv 는 함수로 shadow 하고 경로 1 의 선택 규칙만 흉내 낸다 — 버전을 주면
# managed 가, 경로를 주면 그 경로가 base 가 된다 (uv 0.11.16 에 managed 3.12 를 격리
# 설치해 실측한 동작). host 의 /usr/bin/python3.12 · 네트워크 · sudo 는 쓰지 않는다.
#
# 판정 oracle 은 테스트가 직접 가진다 (bin/python 의 링크 대상 + pyvenv.cfg) —
# 제품의 venv_uses_system_python 으로 판정하면 그 함수가 자기 자신을 검사한다.
#
# 실행: ./test_install_python.sh   (exit 0 = PASS)
# colcon test가 ament_add_test로 자동 실행한다.
# shellcheck disable=SC2317  # stub 들은 export -f 로 자식 bash 에서 불린다
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

# 링크 대상과 비교하므로 TMP 자체를 정규화해 둔다 (TMPDIR 이 symlink 아래일 수 있다).
TMP="$(cd "$(mktemp -d)" && pwd -P)"
trap 'rm -rf -- "$TMP"' EXIT

FAKE_SYSTEM_PY="$TMP/usr/bin/python3.12"
MANAGED_PY="$TMP/uv-python/cpython-3.12.13/bin/python3.12"
CALL_LOG="$TMP/calls.log"

_mk_fake_python() {
  mkdir -p "$(dirname "$1")"
  # 옛 코드의 버전 검사 (`python -c ...version_info...`) 가 실제 3.12 처럼 읽도록.
  printf '#!/bin/sh\necho 3.12\n' >"$1"
  chmod +x "$1"
}
_mk_fake_python "$FAKE_SYSTEM_PY"
_mk_fake_python "$MANAGED_PY"

_mk_venv() {
  # uv 가 만드는 모양: bin/python → base 절대경로, python3 → python, pyvenv.cfg, activate
  # $1=venv_dir $2=base interpreter $3=include-system-site-packages (true|false)
  mkdir -p "$1/bin"
  ln -sfn "$2" "$1/bin/python"
  ln -sfn python "$1/bin/python3"
  printf 'home = %s\ninclude-system-site-packages = %s\n' "$(dirname "$2")" "$3" >"$1/pyvenv.cfg"
  # shellcheck disable=SC2016  # $VIRTUAL_ENV / $PATH 는 activate 가 source 될 때 푼다
  printf 'VIRTUAL_ENV=%q\nexport VIRTUAL_ENV\nPATH="$VIRTUAL_ENV/bin:$PATH"\n' "$1" >"$1/bin/activate"
}

# ── ensure_venv 가 caller scope 에서 기대하는 것들 (자식 bash 로 export) ──────
info()    { :; }
warn()    { echo "warn $*" >>"$CALL_LOG"; }
success() { :; }
error()   { echo "error $*" >>"$CALL_LOG"; return 1; }
ensure_uv()           { :; }
apt_update_if_stale() { :; }
sudo()                { echo "sudo $*" >>"$CALL_LOG"; }

uv() {
  # uv venv [--python <spec>] [--system-site-packages] <dir>
  echo "uv $*" >>"$CALL_LOG"
  [[ "$1" == "venv" ]] || return 1
  shift
  local spec="" ssp=false dir=""
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --python) spec="$2"; shift 2 ;;
      --system-site-packages) ssp=true; shift ;;
      -*) shift ;;
      *) dir="$1"; shift ;;
    esac
  done
  case "$spec" in
    /*) _mk_venv "$dir" "$spec" "$ssp" ;;        # 경로 → 그 인터프리터
    *)  _mk_venv "$dir" "$MANAGED_PY" "$ssp" ;;  # 버전 → 설치된 managed 가 이긴다
  esac
}

export -f _mk_venv info warn success error ensure_uv apt_update_if_stale sudo uv
export CALL_LOG MANAGED_PY

_run_ensure_venv() {
  # ensure_venv 를 별도 bash 에서 install.sh 와 같은 옵션으로 돌리고, 끝난 뒤의
  # VIRTUAL_ENV 를 출력한다. $(...) 서브셸은 bash 가 errexit 를 꺼 버려 중간 실패가
  # 조용히 지나가므로 여기서 직접 부르지 않는다. 테스트를 돌리는 셸의 venv
  # (colcon test 는 venv 활성 상태로 돈다) 는 끄고 케이스가 정한 것만 켠다.
  # $1=WORKSPACE  $2=이미 활성인 venv ("" = 없음)
  WORKSPACE="$1" ACTIVE_VENV="$2" LIB_DIR="$LIB_DIR" FAKE_SYSTEM_PY="$FAKE_SYSTEM_PY" bash -c '
    set -eo pipefail
    unset VIRTUAL_ENV
    if [[ -n "$ACTIVE_VENV" ]]; then
      export VIRTUAL_ENV="$ACTIVE_VENV" PATH="$ACTIVE_VENV/bin:$PATH"
    fi
    _RT_LOG_PREFIX=test
    source "$LIB_DIR/rt_common.sh"
    source "$LIB_DIR/install_python.sh"
    RTC_SYSTEM_PYTHON="$FAKE_SYSTEM_PY"
    ensure_venv >/dev/null
    echo "${VIRTUAL_ENV:-<none>}"
  ' 2>>"$CALL_LOG" || echo "<ensure_venv failed>"
}

# 테스트 자신의 판정: base 가 FAKE_SYSTEM_PY 이고 system-site-packages 가 켜졌는가.
_venv_base() {
  if [[ ! -e "$1/bin/python" ]]; then
    echo "absent"
    return
  fi
  local target ssp
  target=$(readlink -f "$1/bin/python")
  ssp=$(sed -n 's/^include-system-site-packages = //p' "$1/pyvenv.cfg")
  if [[ "$target" == "$FAKE_SYSTEM_PY" && "$ssp" == "true" ]]; then
    echo "system"
  elif [[ "$target" == "$MANAGED_PY" ]]; then
    echo "managed"
  else
    echo "other:${target}:${ssp}"
  fi
}

_uv_venv_calls() { grep -c '^uv venv' "$CALL_LOG" || true; }

# ── 1. fresh PC: .venv 없음 · 활성 venv 없음 → system base 로 만든다 ─────────
# managed 3.12 가 이미 설치된 머신을 흉내 낸다 (uv stub). 옛 코드는 여기서 managed.
test_fresh_workspace_gets_system_base() {
  local ws="$TMP/fresh"
  mkdir -p "$ws"
  : >"$CALL_LOG"
  local active
  active=$(_run_ensure_venv "$ws" "")
  expect_eq "fresh.active"   "$ws/.venv" "$active"
  expect_eq "fresh.base"     "system"    "$(_venv_base "$ws/.venv")"
  expect_eq "fresh.uv_calls" "1"         "$(_uv_venv_calls)"
}

# ── 2. install.sh 재실행: setup_env.sh 가 managed base .venv 를 이미 활성화 ──
test_active_workspace_venv_with_wrong_base_is_recreated() {
  local ws="$TMP/rerun_active"
  mkdir -p "$ws"
  _mk_venv "$ws/.venv" "$MANAGED_PY" true
  : >"$CALL_LOG"
  local active
  active=$(_run_ensure_venv "$ws" "$ws/.venv")
  expect_eq "rerun_active.active" "$ws/.venv" "$active"
  expect_eq "rerun_active.base"   "system"    "$(_venv_base "$ws/.venv")"
}

# ── 3. 비활성 managed base .venv — 옛 코드는 버전(3.12)만 보고 재사용했다 ────
test_inactive_workspace_venv_with_wrong_base_is_recreated() {
  local ws="$TMP/rerun_inactive"
  mkdir -p "$ws"
  _mk_venv "$ws/.venv" "$MANAGED_PY" true
  : >"$CALL_LOG"
  local active
  active=$(_run_ensure_venv "$ws" "")
  expect_eq "rerun_inactive.active" "$ws/.venv" "$active"
  expect_eq "rerun_inactive.base"   "system"    "$(_venv_base "$ws/.venv")"
}

# ── 4. system base 지만 system-site-packages 가 꺼진 .venv 도 재생성 ─────────
# 1-3 과 base 는 같고 이 속성 하나만 다르다.
test_workspace_venv_without_system_site_packages_is_recreated() {
  local ws="$TMP/no_ssp"
  mkdir -p "$ws"
  _mk_venv "$ws/.venv" "$FAKE_SYSTEM_PY" false
  : >"$CALL_LOG"
  _run_ensure_venv "$ws" "" >/dev/null
  expect_eq "no_ssp.base" "system" "$(_venv_base "$ws/.venv")"
}

# ── 5. 멀쩡한 .venv 는 건드리지 않는다 (재생성하면 매 install 마다 lock 재동기화) ──
test_valid_workspace_venv_is_kept() {
  local ws="$TMP/valid"
  mkdir -p "$ws"
  _mk_venv "$ws/.venv" "$FAKE_SYSTEM_PY" true
  : >"$ws/.venv/marker"
  : >"$CALL_LOG"
  local active
  active=$(_run_ensure_venv "$ws" "$ws/.venv")
  expect_eq "valid.active"   "$ws/.venv" "$active"
  expect_eq "valid.kept"     "yes"       "$([[ -e "$ws/.venv/marker" ]] && echo yes || echo no)"
  expect_eq "valid.uv_calls" "0"         "$(_uv_venv_calls)"
}

# ── 6. 사용자가 켠 다른 venv 는 그대로 쓴다 (지우지도 고치지도 않는다) ─────────
test_foreign_active_venv_is_left_alone() {
  local ws="$TMP/foreign_ws" other="$TMP/foreign_venv"
  mkdir -p "$ws"
  _mk_venv "$other" "$MANAGED_PY" true
  : >"$CALL_LOG"
  local active
  active=$(_run_ensure_venv "$ws" "$other")
  expect_eq "foreign.active"     "$other"   "$active"
  expect_eq "foreign.untouched"  "managed"  "$(_venv_base "$other")"
  expect_eq "foreign.no_ws_venv" "absent"   "$(_venv_base "$ws/.venv")"
  expect_eq "foreign.uv_calls"   "0"        "$(_uv_venv_calls)"
}

# ── 7. 활성화된 채로 .venv 를 지운 셸 (우회 절차 `rm -rf .venv` 직후) ───────────
# VIRTUAL_ENV 가 없는 경로를 가리킨다. 옛 코드는 "already active" 로 빠져 venv 없이
# 진행했고, 뒤이은 uv pip sync 가 없는 venv 에 쓰려다 죽었다.
test_stale_virtual_env_pointing_at_deleted_workspace_venv() {
  local ws="$TMP/stale"
  mkdir -p "$ws"
  : >"$CALL_LOG"
  local active
  active=$(_run_ensure_venv "$ws" "$ws/.venv")
  expect_eq "stale.active" "$ws/.venv" "$active"
  expect_eq "stale.base"   "system"    "$(_venv_base "$ws/.venv")"
}

test_fresh_workspace_gets_system_base
test_active_workspace_venv_with_wrong_base_is_recreated
test_inactive_workspace_venv_with_wrong_base_is_recreated
test_workspace_venv_without_system_site_packages_is_recreated
test_valid_workspace_venv_is_kept
test_foreign_active_venv_is_left_alone
test_stale_virtual_env_pointing_at_deleted_workspace_venv

echo
echo "── test_install_python.sh summary ──"
echo "  PASS: $PASS"
echo "  FAIL: $FAIL"
if (( FAIL > 0 )); then
  printf '  %s\n' "${FAIL_MSGS[@]}"
  exit 1
fi
exit 0
