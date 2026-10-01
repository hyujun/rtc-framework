#!/usr/bin/env bash
# setup_env.sh — rtc_ws 개발 환경 활성화.
#
# Source 순서:
#   1. ROS 2 (jazzy 우선, humble fallback) — pinocchio · hpp-fcl · proxsuite · rclcpp 등
#   2. deps/install  (fmt · mimalloc · aligator — 소스 빌드, scripts/build_deps.sh 산출물)
#   3. .venv         (Python 의존성; ROS Python 모듈은 system-site-packages 로 상속)
#   4. install/      (워크스페이스 overlay — 있을 때만)
#
# 위치: src/rtc-framework/repo_scripts/scripts/setup_env.sh
#   scripts/ ← repo_scripts/ ← src/rtc-framework/ ← src/ ← <rtc_ws> (4 up)
#
# 사용:
#   source <rtc_ws>/src/rtc-framework/repo_scripts/scripts/setup_env.sh

# ROS 2 — distro 자동 탐색 (jazzy 우선, humble fallback).
# fresh PC: ROS 미설치 시 silent 통과 — install.sh 가 이후 auto-install 처리.
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

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
_WS_ROOT="$(cd "${_SCRIPT_DIR}/../../../.." && pwd)"
_REPO_ROOT="$(cd "${_SCRIPT_DIR}/../.." && pwd)"

# colcon defaults.yaml — cwd 와 무관하게 적용 (_REPO_ROOT/.colcon/defaults.yaml)
export COLCON_DEFAULTS_FILE="${_REPO_ROOT}/.colcon/defaults.yaml"

# make job 수 — plain `colcon build` 도 build.sh 와 같은 병렬도를 쓰게 한다.
# colcon-cmake 는 MAKEFLAGS 에 -j 가 없으면 `-j<논리 코어>` 를 붙이고, 코어가 많은
# 호스트는 그 값만으로 RAM 이 바닥난다. 산정식과 우선순위 (RTC_BUILD_JOBS > 기존
# MAKEFLAGS 의 -j > min(물리 코어, RAM/4GB)) 는 lib/rt_common.sh 의
# resolve_build_makeflags 가 SSoT 다 — 이 셸에 함수를 남기지 않도록 서브프로세스로
# 부른다. `|| true`: `set -e` 인 caller (build.sh) 가 여기서 죽지 않게.
_mf="$(MAKEFLAGS="${MAKEFLAGS:-}" RTC_BUILD_JOBS="${RTC_BUILD_JOBS:-}" \
  bash -c 'source "$1" && resolve_build_makeflags' _ "${_SCRIPT_DIR}/lib/rt_common.sh" 2>/dev/null)" || true
if [[ -z "${_mf}" && -n "${RTC_BUILD_JOBS:-}" ]]; then
  echo "setup_env.sh: RTC_BUILD_JOBS='${RTC_BUILD_JOBS}' is not a positive integer — using the default job count" >&2
  _mf="$(MAKEFLAGS="${MAKEFLAGS:-}" RTC_BUILD_JOBS="" \
    bash -c 'source "$1" && resolve_build_makeflags' _ "${_SCRIPT_DIR}/lib/rt_common.sh" 2>/dev/null)" || true
fi
[[ -n "${_mf}" ]] && export MAKEFLAGS="${_mf}"
unset _mf

# deps/install prefix (fmt/mimalloc/aligator)
export RTC_DEPS_PREFIX="${_WS_ROOT}/deps/install"
export CMAKE_PREFIX_PATH="${RTC_DEPS_PREFIX}:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="${RTC_DEPS_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
export PKG_CONFIG_PATH="${RTC_DEPS_PREFIX}/lib/pkgconfig:${PKG_CONFIG_PATH:-}"

# ONNX Runtime (rtc_inference) — manual /opt install. Exported here so plain
# `colcon build` finds it without build.sh's wrapper logic.
[[ -d /opt/onnxruntime ]] && export CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:-}:/opt/onnxruntime"

# MuJoCo binary tarball (no cmake config — rtc_mujoco_sim falls back to find_library).
# Pick the latest /opt/mujoco-*/ that exists. Use `sort -V` (version sort, matches
# build.sh) — a lexical glob would rank mujoco-3.7.0 above mujoco-3.10.0.
# `|| true`: MuJoCo 가 없는 호스트에서 ls 가 2 로 끝나고, caller 가 `set -eo pipefail`
# (build.sh · install.sh) 이면 이 대입이 그 스크립트를 출력 없이 종료시킨다.
_mj=$(ls -d /opt/mujoco-* 2>/dev/null | sort -V | tail -1) || true
[[ -n "${_mj:-}" && -d "$_mj" && -f "$_mj/lib/libmujoco.so" ]] && export MUJOCO_DIR="$_mj"
unset _mj

# rtc_mujoco_sim find_package(mujoco) hint — build.sh injects -Dmujoco_ROOT
# via cmake-args; mirror it as env var so plain `colcon build` works too.
[[ -n "${MUJOCO_DIR:-}" ]] && export mujoco_ROOT="${MUJOCO_DIR}"

# Python venv (system-site-packages=true: ROS rclpy/ament_* 상속)
if [[ -f "${_WS_ROOT}/.venv/bin/activate" ]]; then
  # shellcheck source=/dev/null
  source "${_WS_ROOT}/.venv/bin/activate"
fi

# Workspace overlay (colcon build 이후에만 존재)
if [[ -f "${_WS_ROOT}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${_WS_ROOT}/install/setup.bash"
fi

unset _SCRIPT_DIR _WS_ROOT _REPO_ROOT
