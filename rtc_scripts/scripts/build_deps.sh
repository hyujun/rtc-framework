#!/usr/bin/env bash
# build_deps.sh — fmt + mimalloc + aligator 를 deps/install/ 로 소스 빌드.
#
# 의존성 위상: fmt → mimalloc → aligator (aligator 가 fmt + mimalloc + pinocchio + hpp-fcl 요구).
# pinocchio · hpp-fcl 은 ROS distribution (jazzy 또는 humble) 판을 그대로 사용 (ABI 호환 확인됨).
#
# 산출물: deps/install/{lib,include,lib/cmake/...}
# RPATH: $ORIGIN/../lib + deps/install/lib + ROS lib — 시스템 /usr/local 참조 없음.

set -eo pipefail

# 위치: src/rtc-framework/rtc_scripts/scripts/build_deps.sh → 4 up = <rtc_ws>
_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "${_SCRIPT_DIR}/../../../.." && pwd)"
REPO="$(cd "${_SCRIPT_DIR}/../.." && pwd)"
DEPS_PREFIX="${WS}/deps/install"

# ── ROS 소싱 (hpp-fcl_DIR resolve 용) ─────────────────────────────────────
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

HPPFCL_CMAKE_DIR="/opt/ros/${ROS_DISTRO}/lib/x86_64-linux-gnu/cmake/hpp-fcl"
if [[ ! -d "$HPPFCL_CMAKE_DIR" ]]; then
  echo "ERROR: ROS hpp-fcl cmake dir not found: $HPPFCL_CMAKE_DIR" >&2
  echo "       Install: sudo apt-get install ros-${ROS_DISTRO}-hpp-fcl" >&2
  exit 1
fi

PARALLEL_JOBS="${PARALLEL_JOBS:-$(nproc)}"

log() { printf '\n\033[1;34m▶ %s\033[0m\n' "$*"; }

# deps/src 가 비어있으면 deps.repos 로 자동 import (fresh checkout 재현)
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
  log "Building $name (-j${PARALLEL_JOBS})"
  cmake --build "$bld" --parallel "$PARALLEL_JOBS"
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
  -Dhpp-fcl_DIR="$HPPFCL_CMAKE_DIR" \
  -Dfmt_DIR="${DEPS_PREFIX}/lib/cmake/fmt"

log "All deps built: $DEPS_PREFIX"
ls -la "${DEPS_PREFIX}/lib/" | grep -E 'libaligator|libfmt|libmimalloc' || true
