#!/usr/bin/env bash
# setup_env.sh — rtc_ws 개발 환경 활성화.
#
# Source 순서:
#   1. ROS 2 Jazzy (pinocchio · hpp-fcl · proxsuite · rclcpp 등)
#   2. deps/install  (fmt · mimalloc · aligator — 소스 빌드, scripts/build_deps.sh 산출물)
#   3. .venv         (Python 의존성; ROS Python 모듈은 system-site-packages 로 상속)
#   4. install/      (워크스페이스 overlay — 있을 때만)
#
# 위치: src/rtc-framework/rtc_scripts/scripts/setup_env.sh
#   scripts/ ← rtc_scripts/ ← src/rtc-framework/ ← src/ ← <rtc_ws> (4 up)
#
# 사용:
#   source <rtc_ws>/src/rtc-framework/rtc_scripts/scripts/setup_env.sh

# ROS 2
if [[ -z "${ROS_DISTRO:-}" ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
fi

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
_WS_ROOT="$(cd "${_SCRIPT_DIR}/../../../.." && pwd)"
_REPO_ROOT="$(cd "${_SCRIPT_DIR}/../.." && pwd)"

# colcon defaults.yaml — cwd 와 무관하게 적용 (_REPO_ROOT/.colcon/defaults.yaml)
export COLCON_DEFAULTS_FILE="${_REPO_ROOT}/.colcon/defaults.yaml"

# deps/install prefix (fmt/mimalloc/aligator)
export RTC_DEPS_PREFIX="${_WS_ROOT}/deps/install"
export CMAKE_PREFIX_PATH="${RTC_DEPS_PREFIX}:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="${RTC_DEPS_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
export PKG_CONFIG_PATH="${RTC_DEPS_PREFIX}/lib/pkgconfig:${PKG_CONFIG_PATH:-}"

# MuJoCo binary tarball (no cmake config — rtc_mujoco_sim falls back to find_library).
# Pick the latest /opt/mujoco-*/ that exists.
for _mj in /opt/mujoco-3.*; do
  [[ -d "$_mj" && -f "$_mj/lib/libmujoco.so" ]] && export MUJOCO_DIR="$_mj"
done
unset _mj

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
