#!/bin/bash
# install_deps.sh — C++ / external dependencies for install.sh
#
# 제공 함수:
#   install_ur_driver         — ur-robot-driver + cyclonedds (apt)
#   install_pinocchio         — apt: pinocchio + tinyxml2 + yaml-cpp
#   install_proxsuite         — apt: proxsuite (TSID QP)
#   install_mpc_deps          — repo_scripts/scripts/build_deps.sh
#                               (fmt + mimalloc + aligator → <ws>/deps/install)
#   verify_mpc_deps           — MPC artifact presence check
#   install_behaviortree      — apt: behaviortree-cpp
#   install_onnxruntime       — apt 또는 GitHub release tarball
#   install_mujoco            — MuJoCo 3.x tarball
#
# Caller scope 의존:
#   ROS_PKG_PREFIX, MJ_DIR, MJ_VERSION, FMT_VERSION, MIMALLOC_VERSION,
#   ALIGATOR_VERSION, SKIP_MPC, INSTALL_SCRIPT_DIR, 로거

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "ERROR: This file should be sourced, not executed." >&2
  exit 1
fi

[[ -n "${_INSTALL_DEPS_LOADED:-}" ]] && return 0
_INSTALL_DEPS_LOADED=1

# ── UR Robot Driver (robot + full) ─────────────────────────────────────────────
install_ur_driver() {
  info "Installing UR robot driver and dependencies (${ROS_PKG_PREFIX})..."
  sudo apt-get install -y \
      ${ROS_PKG_PREFIX}-ur-robot-driver \
      ${ROS_PKG_PREFIX}-ur-msgs \
      ${ROS_PKG_PREFIX}-ur-description \
      ${ROS_PKG_PREFIX}-control-msgs \
      ${ROS_PKG_PREFIX}-rmw-cyclonedds-cpp \
      > /dev/null
  success "UR robot driver and CycloneDDS installed"
}

# ── Pinocchio + rtc_urdf_bridge deps ────────────────────────────────────
# Pinocchio: needed by ClikController / DemoTaskController / OSC / rtc_urdf_bridge
# tinyxml2, yaml-cpp: needed by rtc_urdf_bridge
install_pinocchio() {
  info "Installing Pinocchio and rtc_urdf_bridge dependencies..."
  sudo apt-get install -y libtinyxml2-dev libyaml-cpp-dev > /dev/null
  success "tinyxml2 and yaml-cpp installed (rtc_urdf_bridge)"

  info "Installing Pinocchio (${ROS_PKG_PREFIX})..."
  if sudo apt-get install -y ${ROS_PKG_PREFIX}-pinocchio >/dev/null 2>&1; then
    success "Pinocchio installed via ${ROS_PKG_PREFIX}-pinocchio"
  else
    error "${ROS_PKG_PREFIX}-pinocchio not found — isolation plan requires ROS distribution pinocchio. \
See repo_scripts/README.md for the reason robotpkg fallback was removed."
  fi
}

# ── ProxSuite (TSID QP solver) ─────────────────────────────────────────────────
# ProxSuite: required by rtc_tsid (WQP/HQP formulations used by DemoWbcController).
# Hard dependency of integrated_bringup via rtc_tsid — must be installed for all modes.
install_proxsuite() {
  info "Installing ProxSuite (${ROS_PKG_PREFIX})..."
  if sudo apt-get install -y ${ROS_PKG_PREFIX}-proxsuite >/dev/null 2>&1; then
    success "ProxSuite installed via ${ROS_PKG_PREFIX}-proxsuite"
  else
    error "${ROS_PKG_PREFIX}-proxsuite not found — isolation plan requires ROS distribution proxsuite. \
See repo_scripts/README.md for the reason robotpkg fallback was removed."
  fi
}

# ── MPC deps: fmt / mimalloc / aligator ────────────────────────────────────────
# Isolation landed 2026-04-21: source build lives in
# repo_scripts/scripts/build_deps.sh with install prefix = $WS_ROOT/deps/install.
# No more /usr/local or ~/libs/. See repo_scripts/README.md for details.
install_mpc_deps() {
  if [[ "$SKIP_MPC" -eq 1 ]]; then
    info "Skipping MPC source-built deps (--skip-mpc)"
    return
  fi
  info "Building MPC deps via repo_scripts/scripts/build_deps.sh (fmt + mimalloc + aligator → deps/install)"

  # Ensure sources are present (vcs import if missing).
  # deps.repos 는 repo 내부 (src/rtc-framework/), deps/src|install 은 workspace 루트 (../../).
  if [[ ! -d "${INSTALL_SCRIPT_DIR}/../../deps/src/aligator/.git" ]]; then
    info "  Importing deps sources (deps.repos)..."
    mkdir -p "${INSTALL_SCRIPT_DIR}/../../deps/src"
    (cd "${INSTALL_SCRIPT_DIR}/../../deps/src" \
        && vcs import . < "${INSTALL_SCRIPT_DIR}/deps.repos" > /dev/null 2>&1 \
        && (cd aligator && git submodule update --init --recursive --depth 1 >/dev/null 2>&1)) \
      || { warn "vcs import failed — check deps.repos"; return; }
  fi

  if ! bash "${INSTALL_SCRIPT_DIR}/repo_scripts/scripts/build_deps.sh"; then
    warn "repo_scripts/scripts/build_deps.sh failed — see output above"
    return
  fi
  success "MPC deps built → $(cd "${INSTALL_SCRIPT_DIR}/../../deps/install" && pwd)"
}

# ── Verify MPC install artifacts (verify mode + post-install check) ──────────
verify_mpc_deps() {
  local failed=0
  local deps_prefix="${INSTALL_SCRIPT_DIR}/../../deps/install"
  echo ""
  info "━━━ MPC Dependency Check (${deps_prefix}) ━━━"

  if [[ -f "${deps_prefix}/lib/libfmt.so.${FMT_VERSION}" ]]; then
    success "fmt ${FMT_VERSION}      → ${deps_prefix}/lib/libfmt.so.${FMT_VERSION}"
  else
    warn "fmt ${FMT_VERSION} MISSING (expected ${deps_prefix}/lib/libfmt.so.${FMT_VERSION})"
    failed=1
  fi

  if [[ -f "${deps_prefix}/lib/libmimalloc.so.2.1" ]]; then
    success "mimalloc ${MIMALLOC_VERSION} → ${deps_prefix}/lib/libmimalloc.so.2.1"
  else
    warn "mimalloc ${MIMALLOC_VERSION} MISSING"
    failed=1
  fi

  if [[ -f "${deps_prefix}/lib/libaligator.so.${ALIGATOR_VERSION}" ]]; then
    success "Aligator ${ALIGATOR_VERSION}  → ${deps_prefix}/lib/libaligator.so.${ALIGATOR_VERSION}"
  else
    warn "Aligator ${ALIGATOR_VERSION} MISSING (run: $0 --skip-rt --skip-build to rebuild deps)"
    failed=1
  fi

  [[ $failed -eq 1 ]] && warn "MPC dependency check: one or more artifacts missing"
  return $failed
}

# ── BehaviorTree.CPP (BT coordinator) ──────────────────────────────────────────
install_behaviortree() {
  info "Installing BehaviorTree.CPP (${ROS_PKG_PREFIX})..."
  if sudo apt-get install -y ${ROS_PKG_PREFIX}-behaviortree-cpp >/dev/null 2>&1; then
    success "BehaviorTree.CPP installed via ${ROS_PKG_PREFIX}-behaviortree-cpp"
  else
    warn "${ROS_PKG_PREFIX}-behaviortree-cpp not found — ur5e_bt_coordinator will not build"
    warn "Install manually: sudo apt install ${ROS_PKG_PREFIX}-behaviortree-cpp"
  fi
}

install_onnxruntime() {
  # ONNX Runtime C++ API (fingertip F/T inference)
  local ONNXRT_VER="1.17.1"
  local ONNXRT_DIR="/opt/onnxruntime"

  # apt에서 설치되어 있는지 확인 (dpkg -s로 실제 설치 상태 검증)
  if dpkg -s libonnxruntime-dev 2>/dev/null | grep -q "^Status:.*install ok installed"; then
    success "ONNX Runtime already installed (apt)"
    return
  fi

  # /opt/onnxruntime에 이미 설치된 경우 (라이브러리 + 헤더 모두 확인)
  if [[ -d "$ONNXRT_DIR" && -f "$ONNXRT_DIR/lib/libonnxruntime.so" && -f "$ONNXRT_DIR/include/onnxruntime_cxx_api.h" ]]; then
    success "ONNX Runtime already installed at ${ONNXRT_DIR}"
    return
  fi

  info "Installing ONNX Runtime ${ONNXRT_VER}..."

  # 방법 1: apt
  if sudo apt-get install -y libonnxruntime-dev > /dev/null 2>&1; then
    success "ONNX Runtime installed via apt"
    return
  fi

  # 방법 2: GitHub 릴리즈 다운로드
  local ARCH
  ARCH=$(uname -m)
  if [[ "$ARCH" == "x86_64" ]]; then
    ARCH="x64"
  elif [[ "$ARCH" == "aarch64" ]]; then
    ARCH="aarch64"
  fi

  local DL_URL="https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRT_VER}/onnxruntime-linux-${ARCH}-${ONNXRT_VER}.tgz"
  local TMP_TAR="/tmp/onnxruntime-${ONNXRT_VER}.tgz"

  if ! wget -q --show-progress -O "$TMP_TAR" "$DL_URL"; then
    warn "ONNX Runtime download failed. F/T inference will not be available."
    warn "  Manual install: wget $DL_URL && sudo tar -xzf ... -C /opt/"
    return
  fi

  sudo tar -xzf "$TMP_TAR" -C /opt/
  sudo ln -sf "/opt/onnxruntime-linux-${ARCH}-${ONNXRT_VER}" "$ONNXRT_DIR"
  rm -f "$TMP_TAR"

  # ldconfig 등록
  local ONNXRT_LIB_CONF="/etc/ld.so.conf.d/onnxruntime.conf"
  if [[ ! -f "$ONNXRT_LIB_CONF" ]]; then
    echo "${ONNXRT_DIR}/lib" | sudo tee "$ONNXRT_LIB_CONF" > /dev/null
    sudo ldconfig
  fi

  success "ONNX Runtime ${ONNXRT_VER} installed at ${ONNXRT_DIR}"
}

install_mujoco() {
  if [[ -d "$MJ_DIR" ]]; then
    success "MuJoCo ${MJ_VERSION} already installed at ${MJ_DIR}"
    return
  fi

  info "Installing MuJoCo ${MJ_VERSION}..."

  # Additional GLFW/OpenGL deps for the viewer
  sudo apt-get install -y \
      libglfw3-dev \
      libgl1-mesa-dev \
      libglu1-mesa-dev \
      > /dev/null

  local ARCH
  ARCH=$(uname -m)
  local TMP_TAR="/tmp/mujoco-${MJ_VERSION}-linux-${ARCH}.tar.gz"
  local DL_URL="https://github.com/google-deepmind/mujoco/releases/download/${MJ_VERSION}/mujoco-${MJ_VERSION}-linux-${ARCH}.tar.gz"

  info "Downloading MuJoCo ${MJ_VERSION}..."
  if ! wget -q --show-progress -O "$TMP_TAR" "$DL_URL"; then
    warn "Download failed. Install MuJoCo manually:"
    warn "  wget $DL_URL"
    warn "  sudo tar -xzf mujoco-${MJ_VERSION}-linux-${ARCH}.tar.gz -C /opt/"
    MJ_DIR=""
    return
  fi

  sudo tar -xzf "$TMP_TAR" -C /opt/
  rm -f "$TMP_TAR"

  # Add library path for runtime
  local MJ_LIB_CONF="/etc/ld.so.conf.d/mujoco.conf"
  if [[ ! -f "$MJ_LIB_CONF" ]]; then
    echo "${MJ_DIR}/lib" | sudo tee "$MJ_LIB_CONF" > /dev/null
    sudo ldconfig
  fi

  success "MuJoCo ${MJ_VERSION} installed at ${MJ_DIR}"
}
