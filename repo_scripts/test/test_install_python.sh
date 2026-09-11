#!/bin/bash
# test_install_python.sh — venv · CMake Python 계약.
#
# 대상: install_python.sh::ensure_venv 와 그것·build.sh 가 쓰는 rt_common.sh 의
# get_system_python / venv_uses_system_python / append_cmake_python_args.
#
# 계약: workspace .venv 의 base 와 CMake 인터프리터는 get_system_python (배포판
# python — 기본 `readlink -f /usr/bin/python3`) 이다. apt 가 까는 ROS python 모듈
# (catkin_pkg · yaml · rclpy 가 쓰는 것들) 은 /usr/lib/python3/dist-packages 에 있고
# 그걸 보는 것은 그 인터프리터뿐이다.
#
# 깨졌던 경로 (fresh PC 에서 rtc_base configure 가 catkin_pkg ImportError 로 죽었다):
#   1. `uv venv --python 3.12` — uv 기본값 (python-preference=managed) 이 uv-managed
#      3.12 를 base 로 골랐다. 그 venv 는 /usr/lib/python3/dist-packages 를 못 본다.
#   2. get_system_python 이 venv 링크를 따라가 그 base 를 CMake 에 넘겼다.
#   3. setup_env.sh 가 기존 .venv 를 먼저 활성화하므로 ensure_venv 가 "already active"
#      로 곧장 빠져 재생성 검사에 도달하지 못했다.
#   4. (#513 리뷰) activate 를 다시 source 하면 uv activate 의 `deactivate
#      nondestructive` 가 PATH 를 첫 활성화 시점으로 되돌려 ensure_uv 가 넣은
#      ~/.local/bin 이 사라지고, 뒤이은 `uv pip sync` 가 죽었다.
#   5. (#513 리뷰) build.sh 가 venv 가 있을 때만 인터프리터를 고정했다 — 없으면
#      FindPython 이 PATH 순서로 찾아 ~/.local/bin/python3.12 (uv python install) 를 잡는다.
#
# 격리 방식: uv 는 함수로 shadow 하고 경로 1 의 선택 규칙만 흉내 낸다 — 버전을 주면
# managed 가, 경로를 주면 그 경로가 base 가 된다 (uv 0.11.16 실측). fixture 의
# activate 는 uv 의 것처럼 `deactivate nondestructive` 로 PATH 를 되돌린다 — 그게
# 없으면 경로 4 가 안 보인다 (실제로 한 번 그렇게 통과했다). host 의
# /usr/bin/python3.12 · 네트워크 · sudo 는 쓰지 않는다.
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

# 헬퍼 함수 테스트용 (logger 도 여기서 정의된다 — ensure_venv 는 자식 bash 에서 돈다).
_RT_LOG_PREFIX="test"
# shellcheck disable=SC1091
source "${LIB_DIR}/rt_common.sh"

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
BROKEN_PY="$TMP/broken/bin/python3.12"   # ament 모듈을 못 보는 인터프리터
LOCAL_BIN="$TMP/localbin"                # ensure_uv 가 PATH 에 넣는 ~/.local/bin
CALL_LOG="$TMP/calls.log"

_mk_fake_python() {
  # $1=path $2=`-c ...` 의 종료코드 (0 = catkin_pkg · ament_package import 성공)
  mkdir -p "$(dirname "$1")"
  # 옛 코드의 버전 검사 (`python -c ...version_info...`) 가 실제 3.12 처럼 읽도록.
  printf '#!/bin/sh\necho 3.12\nexit %s\n' "${2:-0}" >"$1"
  chmod +x "$1"
}
_mk_fake_python "$FAKE_SYSTEM_PY"
_mk_fake_python "$MANAGED_PY"
_mk_fake_python "$BROKEN_PY" 1
mkdir -p "$LOCAL_BIN"

_mk_venv() {
  # uv 가 만드는 모양: bin/python → base 절대경로, python3 → python, pyvenv.cfg, activate
  # $1=venv_dir $2=base interpreter $3=include-system-site-packages (true|false)
  mkdir -p "$1/bin"
  ln -sfn "$2" "$1/bin/python"
  ln -sfn python "$1/bin/python3"
  printf 'home = %s\ninclude-system-site-packages = %s\n' "$(dirname "$2")" "$3" >"$1/pyvenv.cfg"
  # uv activate 의 PATH 의미론: 먼저 `deactivate nondestructive` 로 이전 활성화의
  # _OLD_VIRTUAL_PATH 를 복원한 뒤 지금 PATH 를 저장하고 bin 을 붙인다.
  {
    cat <<'EOF'
deactivate () {
    if ! [ -z "${_OLD_VIRTUAL_PATH:+_}" ] ; then
        PATH="$_OLD_VIRTUAL_PATH"
        export PATH
        unset _OLD_VIRTUAL_PATH
    fi
    unset VIRTUAL_ENV
    if [ ! "${1:-}" = "nondestructive" ] ; then
        unset -f deactivate
    fi
}
deactivate nondestructive
EOF
    printf 'VIRTUAL_ENV=%q\n' "$1"
    cat <<'EOF'
export VIRTUAL_ENV
_OLD_VIRTUAL_PATH="$PATH"
PATH="$VIRTUAL_ENV/bin:$PATH"
export PATH
EOF
  } >"$1/bin/activate"
}

# ── ensure_venv 의 caller scope 의존 (자식 bash 로 export) ────────────────────
ensure_uv() {
  # 실제 ensure_uv 처럼 ~/.local/bin 을 PATH 앞에 붙인다.
  echo "ensure_uv" >>"$CALL_LOG"
  [[ ":${PATH}:" == *":${LOCAL_BIN}:"* ]] || export PATH="${LOCAL_BIN}:${PATH}"
}
apt_update_if_stale() { :; }
sudo() {
  echo "sudo $*" >>"$CALL_LOG"
  [[ -z "${MOCK_SUDO_FAILS:-}" ]]   # dpkg lock · 권한 거부 흉내
}

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

export -f _mk_venv ensure_uv apt_update_if_stale sudo uv
export CALL_LOG MANAGED_PY LOCAL_BIN

_run_ensure_venv() {
  # ensure_venv 를 별도 bash 에서 install.sh 와 같은 옵션으로 돌린다. $(...) 서브셸은
  # bash 가 errexit 를 꺼 버려 중간 실패가 조용히 지나가므로 여기서 직접 부르지 않는다.
  # 테스트를 돌리는 셸의 venv (colcon test 는 venv 활성으로 돈다) 는 끄고 케이스가
  # 정한 것만 켠다.
  #   $1=WORKSPACE  $2=이미 활성인 venv ("" = 없음)  $3=활성화 방식
  #     source — setup_env.sh 처럼 같은 셸에서 activate 를 source (_OLD_VIRTUAL_PATH 있음)
  #     env    — 부모 터미널에서 켜고 install.sh 를 실행 (VIRTUAL_ENV · PATH 만 상속)
  # 출력: "<끝난 뒤 VIRTUAL_ENV> <localbin|no-localbin>" — 뒤는 ensure_uv 가 넣은 PATH 생존
  WORKSPACE="$1" ACTIVE_VENV="$2" ACTIVATION="${3:-env}" LIB_DIR="$LIB_DIR" \
    FAKE_SYSTEM_PY="$FAKE_SYSTEM_PY" bash -c '
    set -eo pipefail
    unset VIRTUAL_ENV _OLD_VIRTUAL_PATH
    export PATH=/usr/bin:/bin
    if [[ -n "$ACTIVE_VENV" ]]; then
      if [[ "$ACTIVATION" == source ]]; then
        source "$ACTIVE_VENV/bin/activate"
      else
        export VIRTUAL_ENV="$ACTIVE_VENV" PATH="$ACTIVE_VENV/bin:$PATH"
      fi
    fi
    _RT_LOG_PREFIX=test
    source "$LIB_DIR/rt_common.sh"
    source "$LIB_DIR/install_python.sh"
    RTC_SYSTEM_PYTHON="$FAKE_SYSTEM_PY"
    ensure_venv >/dev/null
    case ":$PATH:" in *":$LOCAL_BIN:"*) p=localbin ;; *) p=no-localbin ;; esac
    echo "${VIRTUAL_ENV:-<none>} $p"
  ' 2>>"$CALL_LOG" || echo "<ensure_venv-failed> -"
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

# ═══ ensure_venv ═════════════════════════════════════════════════════════════

# ── fresh PC: .venv 없음 · 활성 venv 없음 → system base 로 만든다 ─────────────
# managed 3.12 가 이미 설치된 머신을 흉내 낸다 (uv stub). 옛 코드는 여기서 managed.
test_fresh_workspace_gets_system_base() {
  local ws="$TMP/fresh" out
  mkdir -p "$ws"
  : >"$CALL_LOG"
  out=$(_run_ensure_venv "$ws" "")
  expect_eq "fresh.active"   "$ws/.venv" "${out% *}"
  expect_eq "fresh.path"     "localbin"  "${out##* }"
  expect_eq "fresh.base"     "system"    "$(_venv_base "$ws/.venv")"
  expect_eq "fresh.uv_calls" "1"         "$(_uv_venv_calls)"
}

# ── install.sh 재실행: setup_env.sh 가 managed base .venv 를 같은 셸에서 활성화 ──
# 같은 터미널 재실행 — ~/.local/bin 이 터미널 PATH 에 없으므로 ensure_uv 가 넣은 것이
# 살아남아야 뒤이은 `uv pip sync` 가 돈다.
test_active_workspace_venv_with_wrong_base_is_recreated() {
  local ws="$TMP/rerun_active" out
  mkdir -p "$ws"
  _mk_venv "$ws/.venv" "$MANAGED_PY" true
  : >"$CALL_LOG"
  out=$(_run_ensure_venv "$ws" "$ws/.venv" source)
  expect_eq "rerun_active.active" "$ws/.venv" "${out% *}"
  expect_eq "rerun_active.path"   "localbin"  "${out##* }"
  expect_eq "rerun_active.base"   "system"    "$(_venv_base "$ws/.venv")"
}

# ── 부모 터미널에서 켠 managed base .venv (VIRTUAL_ENV · PATH 만 상속) ─────────
test_inherited_workspace_venv_with_wrong_base_is_recreated() {
  local ws="$TMP/rerun_env" out
  mkdir -p "$ws"
  _mk_venv "$ws/.venv" "$MANAGED_PY" true
  : >"$CALL_LOG"
  out=$(_run_ensure_venv "$ws" "$ws/.venv" env)
  expect_eq "rerun_env.active" "$ws/.venv" "${out% *}"
  expect_eq "rerun_env.path"   "localbin"  "${out##* }"
  expect_eq "rerun_env.base"   "system"    "$(_venv_base "$ws/.venv")"
}

# ── 비활성 managed base .venv — 옛 코드는 버전(3.12)만 보고 재사용했다 ──────
test_inactive_workspace_venv_with_wrong_base_is_recreated() {
  local ws="$TMP/rerun_inactive" out
  mkdir -p "$ws"
  _mk_venv "$ws/.venv" "$MANAGED_PY" true
  : >"$CALL_LOG"
  out=$(_run_ensure_venv "$ws" "")
  expect_eq "rerun_inactive.active" "$ws/.venv" "${out% *}"
  expect_eq "rerun_inactive.base"   "system"    "$(_venv_base "$ws/.venv")"
}

# ── system base 지만 system-site-packages 가 꺼진 .venv 도 재생성 ─────────────
# 위 케이스들과 base 는 같고 이 속성 하나만 다르다.
test_workspace_venv_without_system_site_packages_is_recreated() {
  local ws="$TMP/no_ssp"
  mkdir -p "$ws"
  _mk_venv "$ws/.venv" "$FAKE_SYSTEM_PY" false
  : >"$CALL_LOG"
  _run_ensure_venv "$ws" "" >/dev/null
  expect_eq "no_ssp.base" "system" "$(_venv_base "$ws/.venv")"
}

# ── 멀쩡한 .venv 는 건드리지 않는다 (재생성하면 매 install 마다 lock 재동기화) ──
# 재생성이 없어도 activate 를 다시 source 하면 PATH 가 되돌려진다 — 가장 흔한 재실행.
test_valid_workspace_venv_is_kept() {
  local ws="$TMP/valid" out
  mkdir -p "$ws"
  _mk_venv "$ws/.venv" "$FAKE_SYSTEM_PY" true
  : >"$ws/.venv/marker"
  : >"$CALL_LOG"
  out=$(_run_ensure_venv "$ws" "$ws/.venv" source)
  expect_eq "valid.active"   "$ws/.venv" "${out% *}"
  expect_eq "valid.path"     "localbin"  "${out##* }"
  expect_eq "valid.kept"     "yes"       "$([[ -e "$ws/.venv/marker" ]] && echo yes || echo no)"
  expect_eq "valid.uv_calls" "0"         "$(_uv_venv_calls)"
}

# ── apt 가 실패하면 기존 .venv 를 지우지 않는다 (dpkg lock · sudo 거부) ────────
test_failed_apt_keeps_the_existing_venv() {
  local ws="$TMP/apt_fail" out
  mkdir -p "$ws"
  _mk_venv "$ws/.venv" "$MANAGED_PY" true
  : >"$CALL_LOG"
  out=$(MOCK_SUDO_FAILS=1 _run_ensure_venv "$ws" "")
  expect_eq "apt_fail.exits" "<ensure_venv-failed>" "${out% *}"
  expect_eq "apt_fail.kept"  "managed"              "$(_venv_base "$ws/.venv")"
}

# ── 사용자가 켠 다른 venv 는 그대로 쓴다 (지우지도 고치지도 않는다) ─────────────
test_foreign_active_venv_is_left_alone() {
  local ws="$TMP/foreign_ws" other="$TMP/foreign_venv" out
  mkdir -p "$ws"
  _mk_venv "$other" "$MANAGED_PY" true
  : >"$CALL_LOG"
  out=$(_run_ensure_venv "$ws" "$other" source)
  expect_eq "foreign.active"     "$other"  "${out% *}"
  expect_eq "foreign.untouched"  "managed" "$(_venv_base "$other")"
  expect_eq "foreign.no_ws_venv" "absent"  "$(_venv_base "$ws/.venv")"
  expect_eq "foreign.uv_calls"   "0"       "$(_uv_venv_calls)"
}

# ── 없는 경로를 가리키는 VIRTUAL_ENV 는 활성으로 치지 않는다 ────────────────────
# (a) 활성화한 채 .venv 를 지운 셸. 옛 코드는 "already active" 로 빠져 venv 없이
#     진행했고 뒤이은 `uv pip sync` 가 없는 venv 에 쓰려다 죽었다.
# (b) 지워진 다른 venv — (a) 만 workspace 경로로 알아보면 이쪽은 여전히 빠진다.
test_stale_virtual_env_is_not_active() {
  local ws out
  ws="$TMP/stale_ws"
  mkdir -p "$ws"
  : >"$CALL_LOG"
  out=$(_run_ensure_venv "$ws" "$ws/.venv" env)
  expect_eq "stale_ws.active" "$ws/.venv" "${out% *}"
  expect_eq "stale_ws.base"   "system"    "$(_venv_base "$ws/.venv")"

  ws="$TMP/stale_other"
  mkdir -p "$ws"
  : >"$CALL_LOG"
  out=$(_run_ensure_venv "$ws" "$TMP/deleted_venv" env)
  expect_eq "stale_other.active" "$ws/.venv" "${out% *}"
  expect_eq "stale_other.base"   "system"    "$(_venv_base "$ws/.venv")"
}

# ═══ rt_common 헬퍼 ══════════════════════════════════════════════════════════

# ── get_system_python: venv 링크도 PATH 도 따라가지 않는다 ─────────────────────
# 기본값은 host 의 /usr/bin/python3 해석 경로라 값 자체는 host 가 정한다 — 그래서
# "managed 가 아니다 · /usr/bin 아래다" 만 단언한다. 옛 코드는 여기서 managed 를 냈다.
test_get_system_python_ignores_venv_and_path() {
  _mk_venv "$TMP/sp_managed" "$MANAGED_PY" true
  ln -sfn "$MANAGED_PY" "$LOCAL_BIN/python3"
  ln -sfn "$MANAGED_PY" "$LOCAL_BIN/python3.12"
  local got
  got=$(unset RTC_SYSTEM_PYTHON
    VIRTUAL_ENV="$TMP/sp_managed" PATH="$TMP/sp_managed/bin:$LOCAL_BIN:$PATH" get_system_python 2>/dev/null) \
    || got="<get_system_python failed>"
  expect_eq "sys_python.not_managed" "yes" "$([[ "$got" != "$MANAGED_PY" ]] && echo yes || echo no)"
  expect_eq "sys_python.distro_path" "yes" "$([[ "$got" == /usr/bin/python3* ]] && echo yes || echo "no:$got")"
  expect_eq "sys_python.override" "/opt/x/python3.10" "$(RTC_SYSTEM_PYTHON=/opt/x/python3.10 get_system_python)"
  rm -f "$LOCAL_BIN/python3" "$LOCAL_BIN/python3.12"
}

# ── venv_uses_system_python ─────────────────────────────────────────────────
test_venv_uses_system_python() {
  local root="$TMP/vusp"
  ln -sfn python3.12 "$TMP/usr/bin/python3"
  _mk_venv "$root/ok" "$FAKE_SYSTEM_PY" true
  _mk_venv "$root/managed" "$MANAGED_PY" true
  _mk_venv "$root/no_ssp" "$FAKE_SYSTEM_PY" false
  # stdlib `python3 -m venv` 모양: python → python3 → /usr/bin/python3 → python3.12
  mkdir -p "$root/stdlib/bin"
  ln -sfn "$TMP/usr/bin/python3" "$root/stdlib/bin/python3"
  ln -sfn python3 "$root/stdlib/bin/python"
  printf 'home = %s\ninclude-system-site-packages = true\n' "$TMP/usr/bin" >"$root/stdlib/pyvenv.cfg"
  # release upgrade 로 base 가 사라진 venv — 링크 대상과 기대값이 둘 다 없는 같은 경로라
  # readlink -f 문자열 비교만으로는 같아 보인다.
  _mk_venv "$root/dangling" "$TMP/gone/usr/bin/python3.12" true

  expect_eq "venv_base.system"         "yes" "$(_vusp "$root/ok")"
  expect_eq "venv_base.stdlib_layout"  "yes" "$(_vusp "$root/stdlib")"
  expect_eq "venv_base.uv_managed"     "no"  "$(_vusp "$root/managed")"
  expect_eq "venv_base.no_system_site" "no"  "$(_vusp "$root/no_ssp")"
  expect_eq "venv_base.missing_dir"    "no"  "$(_vusp "$root/absent")"
  expect_eq "venv_base.dangling"       "no"  "$(_vusp "$root/dangling" "$TMP/gone/usr/bin/python3.12")"
}

_vusp() {
  # $1=venv_dir $2=RTC_SYSTEM_PYTHON (기본 FAKE_SYSTEM_PY)
  RTC_SYSTEM_PYTHON="${2:-$FAKE_SYSTEM_PY}" venv_uses_system_python "$1" && echo yes || echo no
}

# ── append_cmake_python_args: venv 유무와 무관하게 고정 · 사전 확인 · 경고 ──────
# 각 케이스는 서브셸에서 CMAKE_ARGS 를 만들어 결과를 한 줄로 낸다.
_cmake_python_case() {
  # $1=RTC_SYSTEM_PYTHON  $2=활성 venv ("" = 없음)
  (
    set +e
    unset VIRTUAL_ENV
    [[ -n "$2" ]] && export VIRTUAL_ENV="$2"
    RTC_SYSTEM_PYTHON="$1"
    CMAKE_ARGS=("-DCMAKE_BUILD_TYPE=Release")
    append_cmake_python_args >"$TMP/cmake_python.log" 2>&1
    local rc=$?
    local warned=no
    grep -q "is not based on" "$TMP/cmake_python.log" && warned=yes
    echo "rc=${rc} warned=${warned} args=${CMAKE_ARGS[*]}"
  )
}

test_append_cmake_python_args() {
  _mk_venv "$TMP/cp_managed" "$MANAGED_PY" true
  _mk_venv "$TMP/cp_system" "$FAKE_SYSTEM_PY" true
  local pin="-DPython3_EXECUTABLE=$FAKE_SYSTEM_PY -DPython3_FIND_VIRTUALENV=STANDARD"

  # venv 가 없어도 고정한다 — 옛 build.sh 는 이때 FindPython 에 맡겼다 (경로 5).
  expect_eq "cmake_python.no_venv" \
    "rc=0 warned=no args=-DCMAKE_BUILD_TYPE=Release $pin" \
    "$(_cmake_python_case "$FAKE_SYSTEM_PY" "")"
  # base 가 맞는 venv: 경고 없음.
  expect_eq "cmake_python.system_venv" \
    "rc=0 warned=no args=-DCMAKE_BUILD_TYPE=Release $pin" \
    "$(_cmake_python_case "$FAKE_SYSTEM_PY" "$TMP/cp_system")"
  # managed base venv: 빌드는 고정값으로 가고 경고만 한다.
  expect_eq "cmake_python.managed_venv" \
    "rc=0 warned=yes args=-DCMAKE_BUILD_TYPE=Release $pin" \
    "$(_cmake_python_case "$FAKE_SYSTEM_PY" "$TMP/cp_managed")"
  # ament 모듈을 못 보는 인터프리터: 1 을 내고 CMAKE_ARGS 는 그대로 둔다.
  expect_eq "cmake_python.broken" \
    "rc=1 warned=no args=-DCMAKE_BUILD_TYPE=Release" \
    "$(_cmake_python_case "$BROKEN_PY" "")"
}

test_fresh_workspace_gets_system_base
test_active_workspace_venv_with_wrong_base_is_recreated
test_inherited_workspace_venv_with_wrong_base_is_recreated
test_inactive_workspace_venv_with_wrong_base_is_recreated
test_workspace_venv_without_system_site_packages_is_recreated
test_valid_workspace_venv_is_kept
test_failed_apt_keeps_the_existing_venv
test_foreign_active_venv_is_left_alone
test_stale_virtual_env_is_not_active

test_get_system_python_ignores_venv_and_path
test_venv_uses_system_python
test_append_cmake_python_args

echo
echo "── test_install_python.sh summary ──"
echo "  PASS: $PASS"
echo "  FAIL: $FAIL"
if (( FAIL > 0 )); then
  printf '  %s\n' "${FAIL_MSGS[@]}"
  exit 1
fi
exit 0
