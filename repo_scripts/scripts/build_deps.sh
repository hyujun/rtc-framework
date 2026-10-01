#!/usr/bin/env bash
# build_deps.sh — fmt + mimalloc + aligator 를 deps/install/ 로 소스 빌드.
#
# 의존성 위상: fmt → mimalloc → aligator (aligator 가 fmt + mimalloc + pinocchio + coal 요구).
# pinocchio · coal 은 ROS distribution (jazzy 또는 humble) 판을 그대로 사용 (ABI 호환 확인됨).
# (pinocchio 4.0 은 hpp-fcl → coal 로 충돌 라이브러리 교체; pinocchioConfig 가 coal 을 강제.)
#
# 산출물: deps/install/{lib,include,lib/cmake/...}
# RPATH: $ORIGIN/../lib + deps/install/lib + ROS lib — 시스템 /usr/local 참조 없음.

set -eo pipefail

# 위치: src/rtc-framework/repo_scripts/scripts/build_deps.sh → 4 up = <rtc_ws>
_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "${_SCRIPT_DIR}/../../../.." && pwd)"
REPO="$(cd "${_SCRIPT_DIR}/../.." && pwd)"
DEPS_PREFIX="${WS}/deps/install"

# ── ROS 소싱 (ROS_DISTRO + pinocchio/coal CMAKE_PREFIX_PATH resolve 용) ──────
# NOTE: /opt/ros/*/setup.bash references unbound vars, so we don't use `set -u`.
if [[ -z "${ROS_DISTRO:-}" ]]; then
  for _distro in jazzy humble; do
    if [[ -f "/opt/ros/${_distro}/setup.bash" ]]; then
      # shellcheck source=/dev/null
      source "/opt/ros/${_distro}/setup.bash"
      break
    fi
  done
  unset _distro
fi

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ERROR: ROS 2 not found. Install jazzy or humble first:" >&2
  echo "       ./install.sh --skip-build  # apt deps + ROS auto-install only" >&2
  exit 1
fi

# make job 수 — build.sh 와 같은 knob (lib/rt_common.sh resolve_build_makeflags):
# PARALLEL_JOBS (이 스크립트의 기존 변수) > RTC_BUILD_JOBS > MAKEFLAGS 의 -j >
# min(물리 코어, RAM/4GB). aligator 의 TU 는 워크스페이스에서 가장 무거운 축에
# 들므로 nproc 을 그대로 쓰면 코어 많은 호스트의 RAM 이 여기서 먼저 바닥난다.
# shellcheck source=lib/rt_common.sh
source "${_SCRIPT_DIR}/lib/rt_common.sh"
_mf="$(resolve_build_makeflags "${PARALLEL_JOBS:-}")" || {
  echo "ERROR: PARALLEL_JOBS / RTC_BUILD_JOBS must be a positive integer" \
    "(got '${PARALLEL_JOBS:-${RTC_BUILD_JOBS:-}}')" >&2
  exit 1
}
PARALLEL_JOBS="$(makeflags_get_jobs "$_mf")"
PARALLEL_JOBS="${PARALLEL_JOBS:-$(get_default_build_jobs)}"
unset _mf
# 메모리 상한 표기는 첫 dep 을 빌드하기 전에 확인한다 (rt_common.sh get_build_mem_max).
get_build_mem_max >/dev/null || {
  echo "ERROR: RTC_BUILD_MEM_MAX must be a size like 24G / 20000M / 60%, or 'off'" \
    "(got '${RTC_BUILD_MEM_MAX:-}')" >&2
  exit 1
}

log() { printf '\n\033[1;34m▶ %s\033[0m\n' "$*"; }

# deps/src 가 비어있으면 deps.repos 로 자동 import (fresh checkout 재현).
# COLCON_IGNORE 는 deps/ 가 ws-root 의 src/ 처럼 colcon 에 picked up 되는 것을
# 차단 (aligator/package.xml 가 ROS 패키지로 보이면 duplicate package name error).
mkdir -p "${WS}/deps"
touch "${WS}/deps/COLCON_IGNORE"
if [[ ! -d "${WS}/deps/src/aligator/.git" ]]; then
  log "Importing deps sources (${REPO}/deps.repos → ${WS}/deps/src)"
  mkdir -p "${WS}/deps/src"
  (cd "${WS}/deps/src" && vcs import . < "${REPO}/deps.repos")
  (cd "${WS}/deps/src/aligator" && git submodule update --init --recursive --depth 1)
fi

build_one() {
  local name="$1"; shift
  local src="${WS}/deps/src/${name}"
  local bld="${WS}/deps/build/${name}"
  if [[ ! -d "$src" ]]; then
    echo "ERROR: missing $src — check ${REPO}/deps.repos" >&2
    exit 1
  fi
  log "Configuring $name"
  cmake -S "$src" -B "$bld" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$DEPS_PREFIX" \
        -DCMAKE_INSTALL_RPATH="\$ORIGIN/../lib:${DEPS_PREFIX}/lib" \
        -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON \
        -DBUILD_SHARED_LIBS=ON \
        "$@"
  # build.sh 와 같은 상한 안에서 빌드한다 — 한도를 넘으면 이 빌드만 끝난다.
  build_mem_scope_prefix
  local cap="none" rc=0
  [[ -n "$BUILD_MEM_SCOPE_UNIT" ]] && cap="$BUILD_MEM_SCOPE_MAX"
  log "Building $name (-j${PARALLEL_JOBS}, memory cap ${cap})"
  "${BUILD_MEM_SCOPE_PREFIX[@]}" cmake --build "$bld" --parallel "$PARALLEL_JOBS" || rc=$?
  if [[ "$rc" -ne 0 ]]; then
    if build_mem_scope_oom_killed; then
      echo "ERROR: $name build stopped: it needed more than the ${BUILD_MEM_SCOPE_MAX} memory cap" \
        "at -j${PARALLEL_JOBS}. Lower RTC_BUILD_JOBS or raise RTC_BUILD_MEM_MAX (<size>|off)." >&2
    fi
    exit "$rc"
  fi
  log "Installing $name → $DEPS_PREFIX"
  cmake --install "$bld"
}

build_one fmt \
  -DFMT_TEST=OFF \
  -DFMT_DOC=OFF

build_one mimalloc \
  -DMI_BUILD_TESTS=OFF \
  -DMI_BUILD_OBJECT=OFF

build_one aligator \
  -DBUILD_TESTING=OFF \
  -DBUILD_BENCHMARKS=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DINSTALL_DOCUMENTATION=OFF \
  -DBUILD_PYTHON_INTERFACE=OFF \
  -DBUILD_WITH_PINOCCHIO_SUPPORT=ON \
  -Dfmt_DIR="${DEPS_PREFIX}/lib/cmake/fmt"

log "All deps built: $DEPS_PREFIX"
ls -la "${DEPS_PREFIX}"/lib/lib{aligator,fmt,mimalloc}* 2>/dev/null || true
