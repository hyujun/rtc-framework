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
#   ALIGATOR_VERSION, ONNXRT_VERSION, SKIP_MPC, INSTALL_SCRIPT_DIR, 로거

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
# Pinocchio: needed by rtc_urdf_bridge, rtc_controllers' model-based laws, and
#            the integrated_bringup bindings (DemoJoint / DemoTask / DemoWbc)
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
  # COLCON_IGNORE 는 deps/ 가 ws-root 의 src/ 처럼 colcon 에 picked up 되는 것을
  # 차단 (aligator/package.xml 가 ROS 패키지로 보이면 duplicate package name error).
  local deps_root="${INSTALL_SCRIPT_DIR}/../../deps"
  mkdir -p "${deps_root}"
  touch "${deps_root}/COLCON_IGNORE"
  if [[ ! -d "${deps_root}/src/aligator/.git" ]]; then
    info "  Importing deps sources (deps.repos)..."
    mkdir -p "${deps_root}/src"
    (cd "${deps_root}/src" \
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

  # mimalloc soname is major.minor (e.g. 2.1.7 → libmimalloc.so.2.1).
  local mimalloc_soname="libmimalloc.so.${MIMALLOC_VERSION%.*}"
  if [[ -f "${deps_prefix}/lib/${mimalloc_soname}" ]]; then
    success "mimalloc ${MIMALLOC_VERSION} → ${deps_prefix}/lib/${mimalloc_soname}"
  else
    warn "mimalloc ${MIMALLOC_VERSION} MISSING (expected ${deps_prefix}/lib/${mimalloc_soname})"
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

# ── ONNX Runtime release digests (pin) ─────────────────────────────────────
# MuJoCo 의 "upstream .sha256 자산을 받아서 검증" 패턴은 여기 쓸 수 없다 —
# onnxruntime 은 `.sha256` 자산을 배포하지 않는다(404). 따라서 아래 값은
# **공식 HTTPS 릴리즈 tarball 에서 계산해 박은 pin** 이다. 보호 대상은
# "pin 이후의 자산 교체 / 전송 경로 변조" 다.
#
# 키는 `<version>:<arch>` 다 — install.sh 의 ONNXRT_VERSION 만 올리고 여기를
# 잊으면 옛 버전 digest 로 통과하는 대신 **키 부재로 fail-closed** 된다.
#
# 버전 bump 절차 (ONNXRT_VERSION 변경 시 필수):
#   1. arch 마다 공식 자산을 받는다
#      curl -fsSL -o /tmp/ort-<ARCH>.tgz \
#        https://github.com/microsoft/onnxruntime/releases/download/v<VER>/onnxruntime-linux-<ARCH>-<VER>.tgz
#   2. sha256sum /tmp/ort-<ARCH>.tgz
#   3. GitHub 이 서버 측에서 계산한 asset digest 와 대조한다 — 받은 쪽과 독립인
#      두 번째 계산이다 (1.17.1 시절엔 이 필드가 null 이라 2인 재계산이 유일한
#      방어였다):
#        gh api repos/microsoft/onnxruntime/releases/tags/v<VER> \
#          -q '.assets[] | select(.name|test("linux-(x64|aarch64)-[0-9.]+\\.tgz$")) | [.name,.digest] | @tsv'
#      일치하면 `<VER>:<ARCH>` 항목을 아래에 추가한다. digest 가 null 이면 2인 재계산.
# 1.17.1 pin 은 x64/aarch64 모두 독립 재계산으로 일치 확인됨 (#153 M8).
# 1.28.2 pin 은 x64/aarch64 모두 로컬 sha256sum == GitHub asset digest (2026-09-11).
# 1.17.1 은 IR ≤ 9 만 연다 — 1.28.2 는 IR 10 정책(ur5e_p1b demo_inference)을 위해 올렸다.
# 옛 항목은 롤백(ONNXRT_VERSION 되돌리기)이 fail-closed 되지 않도록 남겨 둔다.
declare -A ONNXRT_SHA256=(
  ["1.17.1:x64"]="89b153af88746665909c758a06797175ae366280cbf25502c41eb5955f9a555e"
  ["1.17.1:aarch64"]="70b6f536bb7ab5961d128e9dbd192368ac1513bffb74fe92f97aac342fbd0ac1"
  ["1.28.2:x64"]="d7209b8751b27b862b0c76332c2e20e203396edb5dab700ecf4bb485cf147415"
  ["1.28.2:aarch64"]="f020b3d31106cc7db03889b4a5c21e7c38ce4a09ad26119c11d1ad6d3fa0ec04"
)

# 핀과 다른 버전의 tarball 트리를 지운다 — 설치가 끝난 머신에는 핀 버전 하나만
# 남긴다 (2026-09-11 사용자 결정). 옛 트리가 남아 있으면 rtc_inference 의 탐색
# fallback·수동 rpath·다른 스크립트가 그것을 잡을 수 있고, 머신에 "어느 런타임이
# 도는가" 가 둘 이상이 된다.
#
# 지우는 것은 이 파일이 푸는 이름 그대로인 디렉토리 — `onnxruntime-linux-<arch>-`
# 뒤에 숫자로 시작하는 버전 — 뿐이다. symlink 는 따라가지 않고, gpu 빌드
# (`…-gpu-<ver>`)·다른 arch·이름이 다른 트리는 건드리지 않는다.
#
# 반드시 핀 트리가 검증·추출되고 ${ONNXRT_DIR} symlink 가 그것을 가리킨 **뒤에만**
# 부른다. 실패한 업그레이드(다운로드 실패·digest mismatch)가 기존 런타임까지
# 지우면 머신에 ORT 가 하나도 남지 않는다.
#
# ONNXRT_KEEP_OTHER_VERSIONS=1 이면 건너뛴다 — 제어 PC 는 다른 제어 프로젝트와
# 공존할 수 있고 (CLAUDE.md §9.2), 그쪽이 옛 버전 트리를 직접 참조할 수 있다.
_prune_other_onnxruntime_versions() {
  local root="$1" arch="$2" keep_dir="$3"
  if [[ "${ONNXRT_KEEP_OTHER_VERSIONS:-0}" == "1" ]]; then
    info "ONNXRT_KEEP_OTHER_VERSIONS=1 — other ONNX Runtime trees under ${root} are kept"
    return
  fi
  local d
  for d in "${root}/onnxruntime-linux-${arch}-"[0-9]*; do
    [[ -d "$d" && ! -L "$d" ]] || continue
    [[ "$d" == "$keep_dir" ]] && continue
    info "Removing ONNX Runtime tree $(basename "$d") (pinned: $(basename "$keep_dir"))"
    sudo rm -rf -- "$d"
  done
}

install_onnxruntime() {
  # ONNX Runtime C++ API (fingertip F/T inference)
  # Version is centralized as ONNXRT_VERSION in install.sh (caller scope).
  #
  # 검증 범위: **GitHub 릴리즈 tarball 경로만** digest 검증 대상이다. apt 패키지
  # (`libonnxruntime-dev`) 와 이미 설치된 ${ONNXRT_DIR} short-circuit 은 각각
  # dpkg 서명 체인 / 과거 설치 결과이므로 여기서 재검증하지 않는다.
  #
  # 실패 계약: 검증 실패(미지원 arch / digest 미등록 / mismatch)는 **ONNX 기능만
  # warn+skip** 이고 install.sh 전체를 중단하지 않는다 — 바로 아래 install_mujoco
  # 의 mismatch 처리와 같은 계약이며, ONNX 는 optional dep 이다. 어느 경우에도
  # 미검증 tarball 은 설치되지 않는다.
  local ONNXRT_VER="${ONNXRT_VERSION}"
  # ONNXRT_DIR / ONNXRT_LIB_CONF 는 caller 가 덮어쓸 수 있다 (MJ_DIR 선례) —
  # test 가 hermetic 하게 돌기 위한 seam 이기도 하다.
  local ONNXRT_DIR="${ONNXRT_DIR:-/opt/onnxruntime}"
  local ONNXRT_LIB_CONF="${ONNXRT_LIB_CONF:-/etc/ld.so.conf.d/onnxruntime.conf}"

  # arch 는 다운로드뿐 아니라 옛 트리 정리에도 쓰이므로 여기서 정한다. 미지원
  # arch 의 거부는 다운로드 직전에 한다 (기존 설치 확인보다 앞서 막지 않는다).
  local ARCH=""
  case "$(uname -m)" in
    x86_64)  ARCH="x64" ;;
    aarch64) ARCH="aarch64" ;;
  esac
  local extract_root
  extract_root="$(dirname "$ONNXRT_DIR")"
  local pinned_tree="${extract_root}/onnxruntime-linux-${ARCH}-${ONNXRT_VER}"

  # apt에서 설치되어 있는지 확인 (dpkg -s로 실제 설치 상태 검증)
  if dpkg -s libonnxruntime-dev 2>/dev/null | grep -q "^Status:.*install ok installed"; then
    success "ONNX Runtime already installed (apt)"
    return
  fi

  # ${ONNXRT_DIR}에 이미 설치된 경우 (라이브러리 + 헤더 모두 확인)
  #
  # 버전은 tarball 이 싣는 VERSION_NUMBER 로만 판정한다. 그게 pin 과 **다르면**
  # 재설치로 넘어간다 — 존재만 보고 끝냈다면 ONNXRT_VERSION 을 올려도 이미 깔린
  # 모든 머신(dev box · 제어 PC)이 옛 런타임에 그대로 머물렀다. 파일이 없으면
  # (수동 빌드 등) 버전을 알 수 없으므로 예전처럼 신뢰하고 둔다.
  local upgrading="false"
  if [[ -d "$ONNXRT_DIR" && -f "$ONNXRT_DIR/lib/libonnxruntime.so" && -f "$ONNXRT_DIR/include/onnxruntime_cxx_api.h" ]]; then
    local installed_ver=""
    if [[ -f "$ONNXRT_DIR/VERSION_NUMBER" ]]; then
      installed_ver="$(tr -d '[:space:]' <"$ONNXRT_DIR/VERSION_NUMBER")"
    fi
    if [[ -z "$installed_ver" || "$installed_ver" == "$ONNXRT_VER" ]]; then
      success "ONNX Runtime already installed at ${ONNXRT_DIR}${installed_ver:+ (${installed_ver})}"
      # 이미 핀인 머신도 정리한다 — 재실행이 "핀 하나만" 으로 수렴하게. 단 symlink
      # 가 이 파일이 만든 핀 트리를 가리킬 때만: 버전을 모르는 수동 설치나 다른
      # 배치에서는 무엇이 현재 런타임인지 단정할 수 없다.
      if [[ -n "$installed_ver" && -n "$ARCH" &&
            "$(readlink -f "$ONNXRT_DIR")" == "$(readlink -f "$pinned_tree")" ]]; then
        _prune_other_onnxruntime_versions "$extract_root" "$ARCH" "$pinned_tree"
      fi
      return
    fi
    info "ONNX Runtime ${installed_ver} at ${ONNXRT_DIR} differs from pinned ${ONNXRT_VER} — upgrading"
    upgrading="true"
  fi

  info "Installing ONNX Runtime ${ONNXRT_VER}..."

  # 방법 1: apt — 업그레이드 중에는 건너뛴다. 이 머신은 이미 tarball 경로를 쓰고
  # 있고(${ONNXRT_DIR}), apt 가 다른 버전을 깔고 끝나면 symlink 는 옛 버전을
  # 가리킨 채 남는다.
  if [[ "$upgrading" == "false" ]] && sudo apt-get install -y libonnxruntime-dev > /dev/null 2>&1; then
    success "ONNX Runtime installed via apt"
    return
  fi

  # 방법 2: GitHub 릴리즈 다운로드 (digest 검증 대상)
  if [[ -z "$ARCH" ]]; then
    warn "ONNX Runtime: unsupported architecture '$(uname -m)' — skipping"
    warn "  Prebuilt tarballs exist for x86_64/aarch64 only."
    warn "  F/T inference will not be available."
    return
  fi

  # digest 미등록이면 다운로드 전에 fail-closed (버전 bump 회귀 차단)
  local sha_key="${ONNXRT_VER}:${ARCH}"
  local expected_sha="${ONNXRT_SHA256[$sha_key]:-}"
  if [[ -z "$expected_sha" ]]; then
    warn "ONNX Runtime: no pinned SHA256 for '${sha_key}' — refusing to install"
    warn "  Add it to ONNXRT_SHA256 in install_deps.sh (see bump procedure there)."
    warn "  F/T inference will not be available."
    return
  fi

  local DL_URL="https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRT_VER}/onnxruntime-linux-${ARCH}-${ONNXRT_VER}.tgz"

  # download → verify → extract 를 owner-only(0700) mktemp 디렉토리에서 연속
  # 수행한다. 예측 가능한 /tmp 경로는 검증과 `sudo tar` 사이에 파일을 바꿔치기할
  # 수 있는 TOCTOU 창을 남긴다. subshell + EXIT trap 이라 중간 실패·시그널에도
  # 임시 파일이 남지 않고, 부모 셸의 trap 을 건드리지 않는다.
  if ! (
    set -e
    tmp_dir="$(mktemp -d)"
    trap 'rm -rf -- "$tmp_dir"' EXIT
    tmp_tar="${tmp_dir}/onnxruntime-linux-${ARCH}-${ONNXRT_VER}.tgz"

    if ! wget -q --show-progress -O "$tmp_tar" "$DL_URL"; then
      warn "ONNX Runtime download failed. F/T inference will not be available."
      warn "  Manual install: wget $DL_URL && sudo tar -xzf ... -C ${extract_root}/"
      exit 1
    fi

    actual_sha="$(sha256sum "$tmp_tar" | awk '{print $1}')"
    if [[ "$actual_sha" != "$expected_sha" ]]; then
      warn "ONNX Runtime tarball SHA256 mismatch — refusing to install"
      warn "  expected: ${expected_sha}"
      warn "  actual:   ${actual_sha}"
      warn "  Upstream re-released the asset, or the download was tampered with."
      warn "  Re-derive the digest (bump procedure in install_deps.sh) before trusting it."
      exit 1
    fi

    sudo tar -xzf "$tmp_tar" -C "${extract_root}/"
  ); then
    # 검증 실패 → symlink 갱신 / ldconfig 등록 없이 종료 (기존 설치 그대로)
    return
  fi

  # -n: ${ONNXRT_DIR} 가 이미 **디렉토리를 가리키는 symlink** 면(= 업그레이드)
  # `ln -sf` 는 그 링크를 따라가 옛 트리 **안에** 새 링크를 만들고 끝난다 — 링크는
  # 여전히 옛 버전을 가리키고 에러도 없다. 최초 설치에선 드러나지 않던 결함이다.
  sudo ln -sfn "$pinned_tree" "$ONNXRT_DIR"

  # 새 핀 트리가 검증·추출되고 symlink 가 그것을 가리킨 지금에서야 옛 트리를 지운다.
  _prune_other_onnxruntime_versions "$extract_root" "$ARCH" "$pinned_tree"

  # ldconfig 등록. conf 는 symlink 경로를 가리키므로 한 번만 쓰면 되지만, 캐시는
  # 매 (재)설치마다 갱신해야 한다 — soname 이 버전마다 다르다
  # (1.17.1 은 libonnxruntime.so.1.17.1, 1.28.2 는 libonnxruntime.so.1).
  if [[ ! -f "$ONNXRT_LIB_CONF" ]]; then
    echo "${ONNXRT_DIR}/lib" | sudo tee "$ONNXRT_LIB_CONF" > /dev/null
  fi
  sudo ldconfig

  success "ONNX Runtime ${ONNXRT_VER} installed at ${ONNXRT_DIR} (sha256 verified)"
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
  local TMP_SHA="${TMP_TAR}.sha256"
  local DL_URL="https://github.com/google-deepmind/mujoco/releases/download/${MJ_VERSION}/mujoco-${MJ_VERSION}-linux-${ARCH}.tar.gz"
  local SHA_URL="${DL_URL}.sha256"

  info "Downloading MuJoCo ${MJ_VERSION}..."
  if ! wget -q --show-progress -O "$TMP_TAR" "$DL_URL"; then
    warn "Download failed. Install MuJoCo manually:"
    warn "  wget $DL_URL"
    warn "  sudo tar -xzf mujoco-${MJ_VERSION}-linux-${ARCH}.tar.gz -C /opt/"
    MJ_DIR=""
    return
  fi

  if wget -q -O "$TMP_SHA" "$SHA_URL"; then
    if ! (cd /tmp && sha256sum -c "$(basename "$TMP_SHA")" > /dev/null 2>&1); then
      warn "MuJoCo tarball SHA256 mismatch — refusing to install"
      rm -f "$TMP_TAR" "$TMP_SHA"
      MJ_DIR=""
      return
    fi
  else
    warn "SHA256 file not available — proceeding without checksum verification"
  fi

  sudo tar -xzf "$TMP_TAR" -C /opt/
  rm -f "$TMP_TAR" "$TMP_SHA"

  # Add library path for runtime
  local MJ_LIB_CONF="/etc/ld.so.conf.d/mujoco.conf"
  if [[ ! -f "$MJ_LIB_CONF" ]]; then
    echo "${MJ_DIR}/lib" | sudo tee "$MJ_LIB_CONF" > /dev/null
    sudo ldconfig
  fi

  success "MuJoCo ${MJ_VERSION} installed at ${MJ_DIR}"
}
