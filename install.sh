#!/bin/bash
# install.sh — RTC (Real-Time Controller) Installation
#
# Requires: GCC 10+ or Clang 13+ (C++20 support).
# All C++ packages use CMAKE_CXX_STANDARD 20 with strict compiler warnings.
#
# Usage:
#   ./install.sh              # full installation (default)
#   ./install.sh sim          # MuJoCo simulation only
#   ./install.sh robot        # Real robot only
#   ./install.sh full         # Explicit full installation
#   ./install.sh --help       # Show this help

set -e

# ── Script directory (absolute path, safe across cd) ──────────────────────────
INSTALL_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Colors ─────────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# ── Helper functions (인자 파싱 전에 정의 — error() 등에서 사용) ────────────────
info()    { echo -e "${BLUE}▶ $*${NC}"; }
success() { echo -e "${GREEN}✔ $*${NC}"; }
warn()    { echo -e "${YELLOW}⚠ $*${NC}"; }
error()   { echo -e "${RED}✘ $*${NC}"; exit 1; }

# ── 공통 유틸리티 라이브러리 (get_physical_cores, compute_cpu_layout 등) ──────
_RT_COMMON="${INSTALL_SCRIPT_DIR}/rtc_scripts/scripts/lib/rt_common.sh"
if [[ -f "$_RT_COMMON" ]]; then
  # shellcheck source=rtc_scripts/scripts/lib/rt_common.sh
  source "$_RT_COMMON"
fi
# rt_common.sh가 bracket 스타일 로깅을 정의하므로 install.sh 이모지 스타일로 복원
info()    { echo -e "${BLUE}▶ $*${NC}"; }
success() { echo -e "${GREEN}✔ $*${NC}"; }
warn()    { echo -e "${YELLOW}⚠ $*${NC}"; }
error()   { echo -e "${RED}✘ $*${NC}"; exit 1; }

# ── apt-get update 이중 호출 방지 ─────────────────────────────────────────────
_LAST_APT_UPDATE=0
apt_update_if_stale() {
  local now; now=$(date +%s)
  if (( now - _LAST_APT_UPDATE > 300 )); then
    sudo apt-get update -qq
    _LAST_APT_UPDATE=$now
  fi
}

# ── Mode & argument parsing ────────────────────────────────────────────────────
MODE="full"
CLEAN_BUILD=0
BUILD_TYPE="Release"
PARALLEL_JOBS=""
SKIP_DEPS=0
SKIP_BUILD=0
SKIP_RT=0
SKIP_DEBUG_SETUP=0
CUSTOM_PACKAGES=()
MJ_DIR=""
SET_PTRACE_SCOPE=0

show_help() {
  echo ""
  echo -e "${BOLD}RTC (Real-Time Controller) — install.sh${NC}"
  echo ""
  echo "Usage: $0 [MODE] [OPTIONS]"
  echo ""
  echo "Modes:"
  echo "  sim    — MuJoCo simulation only"
  echo "             Installs: ROS2 build tools, Pinocchio, MuJoCo 3.x"
  echo "             Skips:    UR robot driver, RT scheduling permissions"
  echo ""
  echo "  robot  — Real robot only"
  echo "             Installs: ROS2 build tools, UR driver, Pinocchio, RT permissions"
  echo "             Skips:    MuJoCo"
  echo ""
  echo "  full   — Complete installation (default)"
  echo "             Installs: everything above"
  echo ""
  echo "Options:"
  echo "  -d, --debug       Build with CMAKE_BUILD_TYPE=Debug"
  echo "  -r, --release     Build with CMAKE_BUILD_TYPE=Release (default)"
  echo "  -c, --clean       Remove build/, install/, and log/ before building"
  echo "  -p, --packages    Comma-separated list of specific packages to build"
  echo "  -j, --jobs N      Limit parallel workers for colcon (e.g. -j 4)"
  echo "  --skip-deps       Skip installing apt system dependencies"
  echo "  --skip-build      Skip compiling the packages (only download/setup)"
  echo "  --skip-rt         Skip configuring RT permissions, IRQ affinity, UDP optimization, NVIDIA setup, and CPU governor"
  echo "  --skip-debug      Skip GDB/debugger tools installation"
  echo "  --ptrace-scope    Set ptrace_scope=0 for VS Code Attach debugger"
  echo "                    (Required for 'Attach to Node' launch configuration)"
  echo "  --mujoco <path>   Use specific MuJoCo path"
  echo "  --help            Show this help"
  echo ""
  echo "Examples:"
  echo "  chmod +x install.sh"
  echo "  ./install.sh sim"
  echo "  ./install.sh robot --skip-deps"
  echo "  ./install.sh full -c -j 4"
  echo ""
  exit 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    sim|simulation)
      MODE=sim
      shift
      ;;
    robot|realrobot|real)
      MODE=robot
      shift
      ;;
    full|all)
      MODE=full
      shift
      ;;
    -d|--debug)
      BUILD_TYPE="Debug"
      shift
      ;;
    -r|--release)
      BUILD_TYPE="Release"
      shift
      ;;
    -c|--clean)
      CLEAN_BUILD=1
      shift
      ;;
    -p|--packages)
      [[ -z "${2:-}" ]] && error "--packages requires a comma-separated list"
      IFS=',' read -r -a CUSTOM_PACKAGES <<< "$2"
      shift 2
      ;;
    -j|--jobs)
      [[ -z "${2:-}" ]] && error "--jobs requires a number"
      PARALLEL_JOBS="$2"
      shift 2
      ;;
    --skip-deps)
      SKIP_DEPS=1
      shift
      ;;
    --skip-build)
      SKIP_BUILD=1
      shift
      ;;
    --skip-rt)
      SKIP_RT=1
      shift
      ;;
    --skip-debug)
      SKIP_DEBUG_SETUP=1
      shift
      ;;
    --ptrace-scope)
      SET_PTRACE_SCOPE=1
      shift
      ;;
    --mujoco)
      [[ -z "${2:-}" ]] && error "--mujoco requires a path argument"
      MJ_DIR="$2"
      shift 2
      ;;
    -h|--help|help)
      show_help
      ;;
    *)
      error "Unknown argument: '$1'  (run $0 --help for usage)"
      ;;
  esac
done

case "$MODE" in
  sim)   MODE_DESC="Simulation  (MuJoCo + Pinocchio, hand: fake response, no RT perms)" ;;
  robot) MODE_DESC="Real Robot  (UR driver + Pinocchio + RT permissions, no MuJoCo)" ;;
  full)  MODE_DESC="Full        (UR driver + Pinocchio + MuJoCo + RT permissions)" ;;
esac

# ── Banner ─────────────────────────────────────────────────────────────────────
echo ""
echo -e "${BOLD}${BLUE}╔══════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}${BLUE}║    RTC (Real-Time Controller) — Installation Script  ║${NC}"
echo -e "${BOLD}${BLUE}╚══════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "  Mode : ${CYAN}${BOLD}${MODE_DESC}${NC}"
echo ""

# ── CPU shield 자동 관리 (빌드 전) ──────────────────────────────────────────
# cset shield가 활성이면 자동 해제하여 전체 코어로 빌드한다.
auto_release_cpu_shield() {
  local isolated
  isolated=$(cat /sys/devices/system/cpu/isolated 2>/dev/null || echo "")

  if [[ -z "$isolated" ]]; then
    return 0
  fi

  local available total
  available=$(nproc)
  total=$(nproc --all)
  warn "CPU 격리 감지: Core ${isolated} 격리 중 (${available}/${total} 코어 사용 가능)"

  local SHIELD_SCRIPT="${INSTALL_SCRIPT_DIR}/rtc_scripts/scripts/cpu_shield.sh"
  if command -v cset &>/dev/null && cset shield -s 2>/dev/null | grep -q "user"; then
    info "cset shield 감지 → 빌드를 위해 자동 해제 중..."
    if [[ -f "$SHIELD_SCRIPT" ]]; then
      sudo bash "$SHIELD_SCRIPT" off 2>/dev/null || sudo cset shield --reset 2>/dev/null || true
    else
      sudo cset shield --reset 2>/dev/null || true
    fi
    success "CPU 격리 해제 완료 — 전체 ${total} 코어로 빌드합니다"
    return 0
  fi

  if grep -q "isolcpus=" /proc/cmdline 2>/dev/null; then
    warn "isolcpus GRUB 파라미터 감지 — 재부팅 없이 해제 불가"
    warn "빌드에 ${available}/${total} 코어만 사용됩니다"
    warn "권장: isolcpus를 GRUB에서 제거하고 cset shield 방식으로 전환하세요"
    return 0
  fi

  return 0
}

# ── System info (physical vs logical core detection) ──────────────────────────
{
  LOGICAL_CORES=$(nproc --all)
  PHYS_CORES=$(get_physical_cores)
  echo -e "  CPU  : ${CYAN}${BOLD}${PHYS_CORES} physical cores${NC} (${LOGICAL_CORES} logical)"
  if [[ "$LOGICAL_CORES" -ne "$PHYS_CORES" ]]; then
    echo -e "         ${YELLOW}SMT/HT detected — thread_config.hpp uses physical core count${NC}"
  fi
  if lspci 2>/dev/null | grep -qi 'nvidia'; then
    echo -e "  GPU  : ${CYAN}${BOLD}NVIDIA detected${NC} (RT-safe setup will be applied)"
  fi
  echo ""
}

# ── Common: Workspace structure check ──────────────────────────────────────────
check_workspace_structure() {
  info "Checking workspace directory structure..."
  local SCRIPT_DIR
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  local SRC_DIR
  SRC_DIR="$(dirname "$SCRIPT_DIR")"
  local DETECTED_WS
  DETECTED_WS="$(dirname "$SRC_DIR")"

  if [[ "$(basename "$SRC_DIR")" != "src" ]]; then
    echo -e "${RED}✘ Invalid directory structure. ROS2 packages must be located inside a 'src' directory.${NC}"
    echo -e "  Expected: ${BOLD}<workspace_dir>/src/<repository_name>${NC}"
    echo -e "  Current:  ${BOLD}${SCRIPT_DIR}${NC}"
    echo -e "  Example:  mkdir -p ~/ros2_ws/ur5e_ws/src && mv ${SCRIPT_DIR} ~/ros2_ws/ur5e_ws/src/"
    exit 1
  fi

  WORKSPACE="$DETECTED_WS"
  success "Workspace correctly configured at: $WORKSPACE"
}

# ── ROS2 Installation (when not found) ────────────────────────────────────────
install_ros2() {
  local ubuntu_ver="$1"
  local ros_distro=""

  case "$ubuntu_ver" in
    22.04) ros_distro="humble" ;;
    24.04) ros_distro="jazzy"  ;;
    *)
      error "Cannot auto-install ROS2: unsupported Ubuntu ${ubuntu_ver}. Supported: 22.04 (Humble), 24.04 (Jazzy)."
      ;;
  esac

  info "Installing ROS2 ${ros_distro} for Ubuntu ${ubuntu_ver}..."

  # ── Locale setup ──────────────────────────────────────────────────────────
  info "Setting up locale (UTF-8)..."
  sudo apt-get update -qq
  sudo apt-get install -y locales > /dev/null
  sudo locale-gen en_US en_US.UTF-8 > /dev/null
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8

  # ── Add ROS2 apt repository ───────────────────────────────────────────────
  info "Adding ROS2 apt repository..."
  sudo apt-get install -y software-properties-common curl > /dev/null
  sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
      | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

  # ── Install ROS2 ─────────────────────────────────────────────────────────
  info "Updating package index and installing ros-${ros_distro}-desktop..."
  sudo apt-get update -qq
  sudo apt-get install -y ros-${ros_distro}-desktop > /dev/null

  # ── Install rosdep ────────────────────────────────────────────────────────
  info "Installing rosdep..."
  sudo apt-get install -y python3-rosdep2 > /dev/null 2>&1 \
      || sudo apt-get install -y python3-rosdep > /dev/null 2>&1 \
      || true
  if command -v rosdep &>/dev/null; then
    sudo rosdep init 2>/dev/null || true
    rosdep update --rosdistro="${ros_distro}" 2>/dev/null || true
  fi

  # ── Source the newly installed ROS2 ──────────────────────────────────────
  # shellcheck disable=SC1090
  # NOTE: set -e 하에서 setup.bash 내부 non-zero 반환 방지
  source "/opt/ros/${ros_distro}/setup.bash" || true

  success "ROS2 ${ros_distro} installed successfully"
}

# ── Common: ROS2 + Ubuntu check ────────────────────────────────────────────────
check_prerequisites() {
  # ── Detect Ubuntu version early (needed for ROS2 auto-install) ───────────
  UBUNTU_VERSION=$(lsb_release -rs 2>/dev/null || echo "unknown")
  local SUPPORTED_UBUNTU=("22.04" "24.04")
  if [[ ! " ${SUPPORTED_UBUNTU[*]} " =~ " ${UBUNTU_VERSION} " ]]; then
    warn "Unsupported Ubuntu (${UBUNTU_VERSION}). Supported: 22.04 (Humble), 24.04 (Jazzy). Continuing..."
  fi

  # If ros2 command is not in PATH, try to find and source ROS2 from /opt/ros/
  if ! command -v ros2 &>/dev/null; then
    warn "ros2 command not found in PATH. Searching /opt/ros/ ..."
    local _found_ros2=0
    if [[ -d /opt/ros ]]; then
      for _distro in jazzy humble iron rolling; do
        if [[ -f "/opt/ros/${_distro}/setup.bash" ]]; then
          info "Found ROS2 ${_distro} at /opt/ros/${_distro} — sourcing setup.bash ..."
          # shellcheck disable=SC1090
          # NOTE: set -e 하에서 setup.bash 내부 non-zero 반환 방지
          source "/opt/ros/${_distro}/setup.bash" || true
          _found_ros2=1
          break
        fi
      done
    fi
    if [[ "$_found_ros2" -eq 0 ]]; then
      warn "ROS2 not found on this system."
      if [[ " ${SUPPORTED_UBUNTU[*]} " =~ " ${UBUNTU_VERSION} " ]]; then
        info "Supported Ubuntu ${UBUNTU_VERSION} detected — attempting automatic ROS2 installation..."
        install_ros2 "$UBUNTU_VERSION"
      else
        error "ROS2 not found and auto-install unavailable for Ubuntu ${UBUNTU_VERSION}. Install ROS2 manually: https://docs.ros.org/en/jazzy/Installation.html"
      fi
    fi
  fi

  # Detect ROS2 distro via three-tier fallback:
  #   1. $ROS_DISTRO env var  — set automatically when setup.bash is sourced (most reliable)
  #   2. /opt/ros/ directory  — works even without sourcing
  #   3. ros2 --version       — parses "(humble)" from "ros2, version X.Y.Z (humble)"
  if [[ -n "${ROS_DISTRO:-}" ]]; then
    ROS_DISTRO_DETECTED="$ROS_DISTRO"
  elif ls /opt/ros/ &>/dev/null; then
    ROS_DISTRO_DETECTED=$(ls /opt/ros/ 2>/dev/null | head -1 || echo "unknown")
  else
    ROS_DISTRO_DETECTED=$(ros2 --version 2>/dev/null | grep -oP '\(\K[^)]+' || echo "unknown")
  fi

  success "ROS2 detected: ${ROS_DISTRO_DETECTED}"

  # ── Distro-aware package prefix ─────────────────────────────────────────────
  # ros-humble-* or ros-jazzy-* selected automatically based on detected distro.
  ROS_PKG_PREFIX="ros-${ROS_DISTRO_DETECTED}"
  PYTHON_ROBOTPKG_SUFFIX="py310"   # Humble / Python 3.10 default
  if [[ "$ROS_DISTRO_DETECTED" == "jazzy" ]]; then
    PYTHON_ROBOTPKG_SUFFIX="py312"  # Jazzy / Python 3.12
  fi

  if [[ "$ROS_DISTRO_DETECTED" != "humble" && "$ROS_DISTRO_DETECTED" != "jazzy" ]]; then
    warn "Unsupported ROS2 distro: ${ROS_DISTRO_DETECTED}. Supported: humble, jazzy. Continuing..."
  fi

  # ── venv compatibility check ─────────────────────────────────────────────────
  if [[ -n "${VIRTUAL_ENV:-}" ]]; then
    warn "Active virtual environment: ${VIRTUAL_ENV}"
    warn "ROS2 colcon builds and apt-installed Python packages are system-level."
    warn "For best compatibility create your venv with:"
    warn "  python3 -m venv .venv --system-site-packages"
    warn "This script will also pip-install Python deps into the venv as a fallback."
  fi
}

# ── Common: Workspace + build tools ───────────────────────────────────────────
setup_workspace() {
  info "Installing ROS2 build tools (${ROS_PKG_PREFIX})..."
  apt_update_if_stale
  sudo apt-get install -y \
      ${ROS_PKG_PREFIX}-ament-cmake \
      ${ROS_PKG_PREFIX}-ament-cmake-gtest \
      ${ROS_PKG_PREFIX}-ament-lint-auto \
      ${ROS_PKG_PREFIX}-ament-lint-common \
      ${ROS_PKG_PREFIX}-controller-manager-msgs \
      python3-colcon-common-extensions \
      python3-vcstool \
      ethtool \
      > /dev/null
  success "Build tools installed"
}

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

# ── Python dev headers + NumPy (required by eigenpy cmake detection) ───────────
# eigenpy's python.cmake calls FIND_NUMPY at cmake configure time.
# python3-dev provides Python.h; python3-numpy provides numpy headers.
# Must be installed before colcon build (and ideally before pinocchio apt install).
install_python_base_deps() {
  info "Installing Python dev headers and NumPy (required by eigenpy cmake)..."
  sudo apt-get install -y \
      python3-dev \
      python3-numpy \
      > /dev/null
  success "python3-dev and python3-numpy installed"

  # If a venv is active, eigenpy's cmake may pick the venv Python which lacks
  # numpy. Install numpy inside the venv as well to cover that case.
  if [[ -n "${VIRTUAL_ENV:-}" ]]; then
    warn "Virtual environment detected: ${VIRTUAL_ENV}"
    warn "apt packages (python3-numpy etc.) are NOT visible inside the venv unless"
    warn "it was created with --system-site-packages. Installing numpy via pip too."
    info "Installing numpy inside the active venv..."
    python3 -m pip install numpy --quiet || true
    success "numpy installed in venv"
  fi
}

# ── Pinocchio (all modes — needed by ClikController / DemoTaskController / OSC) ──
install_pinocchio() {
  info "Installing Pinocchio (model-based controllers, ${ROS_PKG_PREFIX})..."
  if sudo apt-get install -y ${ROS_PKG_PREFIX}-pinocchio >/dev/null 2>&1; then
    success "Pinocchio installed via ${ROS_PKG_PREFIX}-pinocchio"
  else
    warn "${ROS_PKG_PREFIX}-pinocchio not found, trying robotpkg..."
    # Ubuntu 22.04+ deprecates apt-key; Ubuntu 24.04 removes it entirely.
    # Use gpg keyring method (works on both 22.04 and 24.04).
    local ROBOTPKG_KEYRING="/usr/share/keyrings/robotpkg-archive-keyring.gpg"
    curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.key \
        | sudo gpg --batch --yes --dearmor -o "$ROBOTPKG_KEYRING" 2>/dev/null || true
    sudo sh -c "echo 'deb [arch=amd64 signed-by=/usr/share/keyrings/robotpkg-archive-keyring.gpg] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -sc) robotpkg' \
        > /etc/apt/sources.list.d/robotpkg.list"
    sudo apt-get update -qq
    if sudo apt-get install -y robotpkg-${PYTHON_ROBOTPKG_SUFFIX}-pinocchio >/dev/null 2>&1; then
      success "Pinocchio installed via robotpkg (${PYTHON_ROBOTPKG_SUFFIX})"
      grep -q "openrobots" ~/.bashrc || {
        echo "export PATH=/opt/openrobots/bin:\$PATH" >> ~/.bashrc
        echo "export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:\$PKG_CONFIG_PATH" >> ~/.bashrc
        echo "export LD_LIBRARY_PATH=/opt/openrobots/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc
        echo "export CMAKE_PREFIX_PATH=/opt/openrobots:\$CMAKE_PREFIX_PATH" >> ~/.bashrc
      }
    else
      warn "Pinocchio not installed — ClikController / DemoTaskController / OperationalSpaceController unavailable"
      warn "See: https://stack-of-tasks.github.io/pinocchio/download.html"
    fi
  fi
}

# ── MuJoCo 3.x (sim + full) ────────────────────────────────────────────────────
MJ_VERSION="3.2.4"
[[ -z "$MJ_DIR" ]] && MJ_DIR="/opt/mujoco-${MJ_VERSION}"

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

  local TMP_TAR="/tmp/mujoco-${MJ_VERSION}-linux-x86_64.tar.gz"
  local DL_URL="https://github.com/google-deepmind/mujoco/releases/download/${MJ_VERSION}/mujoco-${MJ_VERSION}-linux-x86_64.tar.gz"

  info "Downloading MuJoCo ${MJ_VERSION}..."
  if ! wget -q --show-progress -O "$TMP_TAR" "$DL_URL"; then
    warn "Download failed. Install MuJoCo manually:"
    warn "  wget $DL_URL"
    warn "  sudo tar -xzf mujoco-${MJ_VERSION}-linux-x86_64.tar.gz -C /opt/"
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

# ── Python dependencies ─────────────────────────────────────────────────────────
install_python_deps() {
  info "Installing Python dependencies (via apt)..."
  apt_update_if_stale
  sudo apt-get install -y \
      python3-matplotlib \
      python3-pandas \
      python3-numpy \
      python3-scipy \
      python3-pyqt5 \
      > /dev/null
  success "Python dependencies installed via apt"

  # apt packages are installed into the system Python's site-packages.
  # A venv created WITHOUT --system-site-packages cannot see them.
  # Install the same packages via pip so they are importable inside the venv.
  if [[ -n "${VIRTUAL_ENV:-}" ]]; then
    info "Installing Python dependencies inside the active venv (pip)..."
    python3 -m pip install --quiet \
        matplotlib \
        pandas \
        numpy \
        scipy \
        PyQt5 \
      || warn "One or more pip installs failed — check output above"
    success "Python dependencies installed in venv"
  fi
}

# ── VS Code / GDB debug tools ────────────────────────────────────────────
install_vscode_debug_tools() {
  info "Installing GDB and debug tools for VS Code..."
  sudo apt-get install -y \
      gdb \
      gdb-multiarch \
      > /dev/null
  success "GDB installed"

  # ── ptrace_scope: Attach to running process ────────────────────────────
  # VS Code의 'Attach to Node' 누시엕 쫐피구는 GDB ptrace 사용.
  # Ubuntu 기본값 ptrace_scope=1은 자식 프로세스 외 attach 거부.
  local PTRACE_CURRENT
  PTRACE_CURRENT=$(cat /proc/sys/kernel/yama/ptrace_scope 2>/dev/null || echo "unknown")

  if [[ "$SET_PTRACE_SCOPE" -eq 1 ]]; then
    info "Setting ptrace_scope=0 for VS Code Attach debugger..."
    echo 0 | sudo tee /proc/sys/kernel/yama/ptrace_scope > /dev/null
    # 영구 적용 (sysctl.conf)
    if ! grep -q 'kernel.yama.ptrace_scope' /etc/sysctl.d/99-ptrace.conf 2>/dev/null; then
      echo 'kernel.yama.ptrace_scope = 0' | sudo tee /etc/sysctl.d/99-ptrace.conf > /dev/null
      sudo sysctl -p /etc/sysctl.d/99-ptrace.conf > /dev/null 2>&1 || true
    fi
    success "ptrace_scope set to 0 (VS Code Attach enabled, persists across reboots)"
    warn "Security note: ptrace_scope=0 allows any process to attach to another."
    warn "Only use this on a development machine, not a production/robot system."
  else
    if [[ "$PTRACE_CURRENT" != "0" ]]; then
      echo ""
      warn "ptrace_scope is currently ${PTRACE_CURRENT} (not 0)."
      warn "VS Code 'Attach to Node' debugger may fail with 'Operation not permitted'."
      warn "To enable: re-run with --ptrace-scope, or run manually:"
      warn "  echo 0 | sudo tee /proc/sys/kernel/yama/ptrace_scope"
      echo ""
    else
      success "ptrace_scope=0 already set (VS Code Attach ready)"
    fi
  fi
}

# ── Clone / update package ─────────────────────────────────────────────────────
setup_package() {
  # Determine the repo directory.
  # If install.sh is already inside <workspace>/src/<repo>/, use that directly.
  local SCRIPT_DIR
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  local REPO_NAME
  REPO_NAME="$(basename "$SCRIPT_DIR")"

  cd "$WORKSPACE/src"

  if [[ -d "$REPO_NAME" && "$SCRIPT_DIR" == "$WORKSPACE/src/$REPO_NAME" ]]; then
    info "Repository already present at $WORKSPACE/src/$REPO_NAME — skipping clone"
  elif [[ ! -d "ur5e-rt-controller" ]]; then
    info "Cloning ur5e-rt-controller..."
    git clone https://github.com/hyujun/ur5e-rt-controller.git
    REPO_NAME="ur5e-rt-controller"
  else
    warn "ur5e-rt-controller already exists — skipping clone"
    REPO_NAME="ur5e-rt-controller"
  fi

  # Symlink packages from repo root into workspace src/
  for pkg in rtc_msgs rtc_base ur5e_description rtc_status_monitor rtc_controller_manager ur5e_hand_driver rtc_mujoco_sim rtc_tools; do
    if [[ ! -e "$pkg" ]]; then
      ln -s "${REPO_NAME}/$pkg" "$pkg"
    fi
  done
}

# ── Build ──────────────────────────────────────────────────────────────────────
build_package() {
  info "Building all ur5e packages..."
  cd "$WORKSPACE"

  if [[ "$CLEAN_BUILD" -eq 1 ]]; then
    info "Cleaning previous build artifacts..."
    rm -rf build/ install/ log/
  fi

  local CMAKE_ARGS=("-DCMAKE_BUILD_TYPE=${BUILD_TYPE}")

  # When a venv is active, CMake's FindPython prefers the venv Python, which
  # may lack numpy and cause eigenpy's cmake to fail. Force system Python so
  # pinocchio/eigenpy find the apt-installed numpy.
  if [[ -n "${VIRTUAL_ENV:-}" ]]; then
    local SYS_PYTHON
    SYS_PYTHON=$(command -v python3 || true)
    # Resolve the real system python, not the venv symlink
    SYS_PYTHON=$(readlink -f "${SYS_PYTHON}" 2>/dev/null || echo "${SYS_PYTHON}")
    if [[ "$SYS_PYTHON" == "${VIRTUAL_ENV}"* ]]; then
      # Still pointing inside venv — use /usr/bin/python3 directly
      SYS_PYTHON="/usr/bin/python3"
    fi
    CMAKE_ARGS+=("-DPython3_EXECUTABLE=${SYS_PYTHON}")
    CMAKE_ARGS+=("-DPython3_FIND_VIRTUALENV=STANDARD")
    warn "Venv detected — cmake will use system Python: ${SYS_PYTHON}"
  fi

  # compile_commands.json: Debug 빌드 시 자동 활성화 — .vscode/c_cpp_properties.json IntelliSense 연동
  if [[ "$BUILD_TYPE" == "Debug" ]]; then
    CMAKE_ARGS+=("-DCMAKE_EXPORT_COMPILE_COMMANDS=ON")
    info "compile_commands.json generation enabled (VS Code IntelliSense)"
  fi

  if [[ -n "$MJ_DIR" && -d "$MJ_DIR" ]]; then
    # MuJoCo binary release (tarball) does NOT include lib/cmake/mujoco/.
    # Pass mujoco_ROOT so CMakeLists.txt can locate the .so and headers via find_library.
    CMAKE_ARGS+=("-Dmujoco_ROOT=${MJ_DIR}")
    info "MuJoCo root: ${MJ_DIR}"
  fi

  # Build order: rtc_base first (header-only, no deps), then the rest
  local PACKAGES=()
  if [[ ${#CUSTOM_PACKAGES[@]} -gt 0 ]]; then
    PACKAGES=("${CUSTOM_PACKAGES[@]}")
  else
    PACKAGES=(rtc_msgs rtc_base rtc_communication rtc_controller_interface rtc_controllers rtc_controller_manager rtc_status_monitor rtc_inference rtc_scripts ur5e_description ur5e_hand_driver ur5e_bringup rtc_tools)
    if [[ -n "$MJ_DIR" && -d "$MJ_DIR" ]]; then
      PACKAGES+=(rtc_mujoco_sim)
    fi
  fi

  local COLCON_ARGS=("--packages-select" "${PACKAGES[@]}" "--symlink-install")

  if [[ -n "$PARALLEL_JOBS" ]]; then
    COLCON_ARGS+=("--parallel-workers" "$PARALLEL_JOBS")
  fi

  if [[ ${#CMAKE_ARGS[@]} -gt 0 ]]; then
    COLCON_ARGS+=("--cmake-args" "${CMAKE_ARGS[@]}")
  fi

  colcon build "${COLCON_ARGS[@]}" || error "Build failed!"

  # 빌드 후 최신 overlay 소싱 (verify_installation 등 후속 작업용)
  source "${WORKSPACE}/install/setup.bash" || true
  success "All packages built and sourced"
}

# ── RT permissions (robot + full) ──────────────────────────────────────────────
install_rt_permissions() {
  info "Configuring RT scheduling permissions..."
  sudo groupadd -f realtime
  sudo usermod -aG realtime "$USER"

  if ! grep -q "@realtime.*rtprio" /etc/security/limits.conf; then
    echo "@realtime - rtprio 99" | sudo tee -a /etc/security/limits.conf > /dev/null
  fi
  # memlock unlimited 설정 (이미 올바르면 건너뜀)
  if ! grep -q '^@realtime - memlock unlimited$' /etc/security/limits.conf 2>/dev/null; then
    sudo sed -i '/@realtime.*memlock/d' /etc/security/limits.conf
    echo "@realtime - memlock unlimited" | sudo tee -a /etc/security/limits.conf > /dev/null
  fi

  success "RT permissions configured (rtprio=99, memlock=unlimited)"
  warn "IMPORTANT: Log out and log back in for RT permissions to take effect"
  warn "Verify with: ulimit -l  (MUST be 'unlimited' to avoid RMW load errors with mlockall)"
}

# get_physical_cores() — rt_common.sh에서 제공

# ── Passwordless sudo for RT scripts (robot + full) ────────────────────────
# cpu_shield.sh는 launch 파일에서 비대화형으로 sudo 호출되므로
# 비밀번호 없이 실행 가능하도록 sudoers drop-in 파일을 설정한다.
setup_rt_sudoers() {
  info "Configuring passwordless sudo for RT scripts..."

  local SUDOERS_FILE="/etc/sudoers.d/ur5e-rt-controller"

  # 설치된 cpu_shield.sh 경로
  local INSTALLED_SHIELD="${WORKSPACE}/install/rtc_controller_manager/lib/rtc_controller_manager/cpu_shield.sh"
  # 소스 cpu_shield.sh 경로
  local SOURCE_SHIELD="${INSTALL_SCRIPT_DIR}/rtc_scripts/scripts/cpu_shield.sh"

  # cset 명령 경로
  local CSET_PATH
  CSET_PATH="$(command -v cset 2>/dev/null || echo "/usr/bin/cset")"

  local SUDOERS_CONTENT
  SUDOERS_CONTENT="# RTC (Real-Time Controller) — passwordless sudo for CPU shield management
# Generated by install.sh — do not edit manually
# cpu_shield.sh: launch 파일에서 비대화형 sudo 호출에 필요
%realtime ALL=(ALL) NOPASSWD: ${INSTALLED_SHIELD} *
%realtime ALL=(ALL) NOPASSWD: ${SOURCE_SHIELD} *
%realtime ALL=(ALL) NOPASSWD: ${CSET_PATH} shield *"

  # 기존 파일과 동일하면 건너뜀
  if [[ -f "$SUDOERS_FILE" ]] && echo "$SUDOERS_CONTENT" | diff -q - "$SUDOERS_FILE" &>/dev/null; then
    success "RT sudoers already configured"
    return
  fi

  # sudoers 파일 생성 (visudo로 문법 검증)
  local TEMP_SUDOERS
  TEMP_SUDOERS=$(mktemp)
  echo "$SUDOERS_CONTENT" > "$TEMP_SUDOERS"
  chmod 0440 "$TEMP_SUDOERS"

  if sudo visudo -cf "$TEMP_SUDOERS" &>/dev/null; then
    sudo cp "$TEMP_SUDOERS" "$SUDOERS_FILE"
    sudo chmod 0440 "$SUDOERS_FILE"
    rm -f "$TEMP_SUDOERS"
    success "Passwordless sudo configured for cpu_shield.sh"
    info "  Sudoers file: $SUDOERS_FILE"
  else
    rm -f "$TEMP_SUDOERS"
    warn "Sudoers syntax check failed — skipping"
    warn "Run manually: sudo visudo -f $SUDOERS_FILE"
  fi
}

# ── cpuset tools for dynamic CPU isolation ──────────────────────────────────
# cset shield 명령을 사용하여 런타임에 CPU 격리를 on/off 할 수 있다.
# isolcpus GRUB 파라미터 없이도 빌드 시 전체 코어, 실행 시 격리 코어를 사용.
install_cset_tools() {
  info "Installing cpuset tools for dynamic CPU isolation..."
  if command -v cset &>/dev/null; then
    success "cset already installed"
    return
  fi
  if sudo apt-get install -y cpuset > /dev/null 2>&1; then
    success "cset installed via cpuset package"
  elif sudo apt-get install -y python3-cpuset > /dev/null 2>&1; then
    success "cset installed via python3-cpuset"
  else
    warn "cset 설치 실패 — /sys/fs/cgroup/cpuset fallback 사용"
    warn "수동 설치: sudo apt-get install -y cpuset"
  fi
}

# ── [방안 D] NIC IRQ affinity (robot + full) ────────────────────────────────
# RT 코어를 NIC 인터럽트로부터 보호.
# 모든 하드웨어 IRQ를 OS 코어로 제한한다.
# CPU 레이아웃 계산은 setup_irq_affinity.sh 내부에서 rt_common.sh로 처리.
setup_irq_affinity() {
  local SCRIPT="${INSTALL_SCRIPT_DIR}/rtc_scripts/scripts/setup_irq_affinity.sh"

  if [[ ! -f "$SCRIPT" ]]; then
    warn "setup_irq_affinity.sh not found — skipping IRQ affinity setup"
    return
  fi

  info "Configuring NIC IRQ affinity..."
  if sudo bash "$SCRIPT"; then
    success "IRQ affinity configured — RT cores protected from NIC interrupts"
    warn "NOTE: IRQ affinity resets on reboot. To make permanent:"
    warn "  Add 'sudo $SCRIPT' to /etc/rc.local or a systemd oneshot service"
  else
    warn "IRQ affinity setup failed — continuing without it"
    warn "Run manually: sudo $SCRIPT [NIC_NAME]"
  fi
}

# ── [UDP/DDS] NIC & kernel network stack optimization (robot + full) ──────────
setup_udp_optimization() {
  local SCRIPT
  SCRIPT="${INSTALL_SCRIPT_DIR}/rtc_scripts/scripts/setup_udp_optimization.sh"

  if [[ ! -f "$SCRIPT" ]]; then
    warn "setup_udp_optimization.sh not found — skipping UDP optimization"
    warn "Run manually after build: sudo $WORKSPACE/src/ur5e-rt-controller/rtc_scripts/scripts/setup_udp_optimization.sh"
    return
  fi

  info "Configuring NIC & kernel network stack for real-time UDP/DDS..."
  if sudo bash "$SCRIPT"; then
    success "UDP optimization applied (coalescing off, offloads off, sysctl buffers set)"
    warn "NOTE: sysctl settings persist via /etc/sysctl.d/99-ros2-udp.conf"
    warn "NOTE: ethtool NIC settings (coalescing, offload, ring) reset on reboot."
    warn "  To make permanent: add 'sudo $SCRIPT' to /etc/rc.local or a systemd service"
  else
    warn "UDP optimization failed — continuing without it"
    warn "Run manually: sudo $SCRIPT [NIC_NAME]"
  fi
}

# ── [NVIDIA] RT-safe GPU configuration (robot + full) ────────────────────────
# NVIDIA GPU가 감지되면 RT 커널 호환성 + 화면 티어링 방지를 설정한다.
setup_nvidia_rt() {
  # NVIDIA GPU 존재 여부 확인
  if ! lspci 2>/dev/null | grep -qi 'nvidia'; then
    info "No NVIDIA GPU detected — skipping NVIDIA RT setup"
    return
  fi

  local SCRIPT
  SCRIPT="${INSTALL_SCRIPT_DIR}/rtc_scripts/scripts/setup_nvidia_rt.sh"

  if [[ ! -f "$SCRIPT" ]]; then
    warn "setup_nvidia_rt.sh not found — skipping NVIDIA RT setup"
    warn "Run manually: sudo $WORKSPACE/src/ur5e-rt-controller/rtc_scripts/scripts/setup_nvidia_rt.sh"
    return
  fi

  info "NVIDIA GPU detected — configuring RT-safe GPU settings (11 steps)..."
  info "  Includes: modprobe, GRUB, IRQ affinity, persistence, nouveau blacklist,"
  info "            X11 anti-tearing, compositor boost, DKMS RT build, CPU governor"
  if sudo bash "$SCRIPT"; then
    success "NVIDIA RT setup complete (DKMS, CPU governor, anti-tearing, etc.)"
    warn "NOTE: A reboot is required for GRUB and X11 changes to take effect"
  else
    warn "NVIDIA RT setup failed — continuing without it"
    warn "Run manually: sudo $SCRIPT"
  fi
}

# ── CPU Governor → performance (비-NVIDIA 시스템용) ───────────────────────────
# NVIDIA GPU가 있는 시스템은 setup_nvidia_rt.sh [10/11]에서 처리.
# NVIDIA가 없는 시스템에서도 powersave → performance 전환이 필요하다.
setup_cpu_governor() {
  # NVIDIA GPU가 있으면 setup_nvidia_rt.sh에서 이미 처리됨
  if lspci 2>/dev/null | grep -qi 'nvidia'; then
    return
  fi

  # cpufreq 디렉토리가 없으면 고정 클럭 CPU (VM 등)
  if [[ ! -d /sys/devices/system/cpu/cpu0/cpufreq ]]; then
    return
  fi

  # 현재 governor 확인
  local non_perf=0
  for gov_file in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    if [[ -f "$gov_file" ]]; then
      local gov
      gov=$(cat "$gov_file" 2>/dev/null)
      [[ "$gov" != "performance" ]] && ((non_perf++)) || true
    fi
  done

  if [[ "$non_perf" -eq 0 ]]; then
    success "CPU governor: all cores already set to performance"
    return
  fi

  info "Setting CPU governor to performance (${non_perf} cores need change)..."

  # 즉시 적용
  for gov_file in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    echo "performance" | sudo tee "$gov_file" > /dev/null 2>&1 || true
  done

  # systemd 서비스로 영구 적용
  local GOV_SERVICE="/etc/systemd/system/cpu-governor-performance.service"
  if [[ ! -f "$GOV_SERVICE" ]]; then
    sudo tee "$GOV_SERVICE" > /dev/null << 'GOVEOF'
[Unit]
Description=Set CPU governor to performance for RT kernel
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/bin/bash -c 'if command -v cpupower &>/dev/null; then cpupower frequency-set -g performance; else for f in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do echo performance > "$f" 2>/dev/null || true; done; fi'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
GOVEOF
    sudo timeout 30 systemctl daemon-reload || true
    sudo systemctl enable cpu-governor-performance.service 2>/dev/null || true
    success "CPU governor → performance (service enabled for persistence)"
  else
    success "CPU governor → performance (service already exists)"
  fi
}

# ── [RT] GRUB 커널 파라미터 + sysctl (NVIDIA 무관, robot + full) ──────────────
# clocksource=tsc, nmi_watchdog=0, sched_rt_runtime_us=-1 등 RT 필수 설정.
# NVIDIA 시스템은 setup_nvidia_rt.sh [3/11]에서도 같은 GRUB 파라미터를 설정하지만,
# "이미 존재" 로 건너뛰므로 중복 문제는 없다.
setup_rt_kernel_params() {
  # rt_common.sh의 compute_cpu_layout 사용
  compute_cpu_layout

  # ── RT_CORES 계산 (SMT 시블링 포함) ──
  # rt_common.sh의 compute_expected_isolated() 재사용 (get_os_logical_cpus + _format_cpu_range)
  local RT_CORES
  RT_CORES=$(compute_expected_isolated)

  info "RT kernel params: RT_CORES=${RT_CORES}, OS_CORES=${OS_CORES_DESC}"

  # ── 1) GRUB 커널 파라미터 ──
  local GRUB_FILE="/etc/default/grub"
  if [[ -f "$GRUB_FILE" ]]; then
    declare -A GRUB_PARAMS_WITH_VALUE=(
      ["nohz_full"]="${RT_CORES}"
      ["rcu_nocbs"]="${RT_CORES}"
      ["processor.max_cstate"]="1"
      ["clocksource"]="tsc"
      ["tsc"]="reliable"
      ["nmi_watchdog"]="0"
    )
    local -a GRUB_PARAMS_WITHOUT_VALUE=("threadirqs" "nosoftlockup")
    local GRUB_VAR="GRUB_CMDLINE_LINUX_DEFAULT"
    local CURRENT_CMDLINE=""
    if grep -q "^${GRUB_VAR}=" "$GRUB_FILE"; then
      CURRENT_CMDLINE=$(grep "^${GRUB_VAR}=" "$GRUB_FILE" | sed "s/^${GRUB_VAR}=\"\(.*\)\"/\1/")
    fi

    local GRUB_MODIFIED=0
    local NEW_CMDLINE="$CURRENT_CMDLINE"

    local param value
    for param in "${!GRUB_PARAMS_WITH_VALUE[@]}"; do
      value="${GRUB_PARAMS_WITH_VALUE[$param]}"
      if ! echo "$NEW_CMDLINE" | grep -qE "(^| )${param}="; then
        NEW_CMDLINE="${NEW_CMDLINE:+${NEW_CMDLINE} }${param}=${value}"
        GRUB_MODIFIED=1
        info "  GRUB 추가: ${param}=${value}"
      fi
    done
    for param in "${GRUB_PARAMS_WITHOUT_VALUE[@]}"; do
      if ! echo "$NEW_CMDLINE" | grep -qE "(^| )${param}( |$)"; then
        NEW_CMDLINE="${NEW_CMDLINE:+${NEW_CMDLINE} }${param}"
        GRUB_MODIFIED=1
        info "  GRUB 추가: ${param}"
      fi
    done

    if [[ "$GRUB_MODIFIED" -eq 1 ]]; then
      # 백업
      sudo cp "$GRUB_FILE" "${GRUB_FILE}.bak.$(date +%Y%m%d_%H%M%S)"

      if grep -q "^${GRUB_VAR}=" "$GRUB_FILE"; then
        local ESCAPED_CMDLINE
        ESCAPED_CMDLINE=$(printf '%s' "$NEW_CMDLINE" | sed 's/[&#\\/]/\\&/g')
        sudo sed -i "s#^${GRUB_VAR}=.*#${GRUB_VAR}=\"${ESCAPED_CMDLINE}\"#" "$GRUB_FILE"
      else
        echo "${GRUB_VAR}=\"${NEW_CMDLINE}\"" | sudo tee -a "$GRUB_FILE" > /dev/null
      fi

      sudo update-grub 2>/dev/null || true
      success "GRUB RT 파라미터 설정 완료 (재부팅 필요)"
    else
      success "GRUB RT 파라미터: 이미 모두 설정됨"
    fi
  else
    warn "GRUB 설정 파일(${GRUB_FILE})을 찾을 수 없습니다 — 건너뜀"
  fi

  # ── 2) sched_rt_runtime_us = -1 (RT 스로틀링 해제) ──
  local RT_RUNTIME
  RT_RUNTIME=$(cat /proc/sys/kernel/sched_rt_runtime_us 2>/dev/null || echo "")
  if [[ "$RT_RUNTIME" != "-1" ]]; then
    sudo sysctl -w kernel.sched_rt_runtime_us=-1 > /dev/null 2>&1 || true
    # 영구 설정
    local SYSCTL_RT="/etc/sysctl.d/99-rt-sched.conf"
    if [[ ! -f "$SYSCTL_RT" ]] || ! grep -q "sched_rt_runtime_us" "$SYSCTL_RT" 2>/dev/null; then
      echo "kernel.sched_rt_runtime_us=-1" | sudo tee "$SYSCTL_RT" > /dev/null
      success "sched_rt_runtime_us=-1 설정 완료 (즉시 + 영구)"
    else
      sudo sysctl -w kernel.sched_rt_runtime_us=-1 > /dev/null 2>&1 || true
      success "sched_rt_runtime_us=-1 즉시 적용 (영구 설정 이미 존재)"
    fi
  else
    success "sched_rt_runtime_us: 이미 -1 (RT 스로틀링 없음)"
  fi
}

# ── Verify installation ─────────────────────────────────────────────────────────
verify_installation() {
  info "Verifying installation..."
  source "$WORKSPACE/install/setup.bash" || true
  local failed=0
  for pkg in rtc_msgs rtc_base ur5e_description rtc_status_monitor rtc_controller_manager ur5e_hand_driver rtc_tools; do
    # Check ament index directly (more reliable than ros2 pkg list in scripts)
    if [[ -d "$WORKSPACE/install/${pkg}" ]]; then
      success "Package installed: $pkg"
    else
      warn "Package not found: $pkg"
      failed=1
    fi
  done
  [[ $failed -eq 1 ]] && error "Installation verification failed"

  info "Available executables (rtc_controller_manager):"
  ros2 pkg executables rtc_controller_manager 2>/dev/null || true
  info "Available executables (ur5e_hand_driver):"
  ros2 pkg executables ur5e_hand_driver 2>/dev/null || true

  mkdir -p "${WORKSPACE}/logging_data/stats" "${WORKSPACE}/logging_data/ur_plot"
  success "Log directories ready (${WORKSPACE}/logging_data, ${WORKSPACE}/logging_data/ur_plot)"

  # RT 환경 검증 (robot/full 모드)
  if [[ "$MODE" == "robot" || "$MODE" == "full" ]]; then
    local CHECK_SCRIPT
    CHECK_SCRIPT="${INSTALL_SCRIPT_DIR}/rtc_scripts/scripts/check_rt_setup.sh"
    if [[ -f "$CHECK_SCRIPT" ]]; then
      echo ""
      info "━━━ RT System Configuration Check ━━━"
      bash "$CHECK_SCRIPT" --verbose || true
    fi
  fi
}

# ── Quick start summary ─────────────────────────────────────────────────────────
print_summary() {
  echo ""
  echo -e "${BOLD}${GREEN}╔══════════════════════════════════════════════════════╗${NC}"
  echo -e "${BOLD}${GREEN}║               Installation Complete!                 ║${NC}"
  echo -e "${BOLD}${GREEN}╚══════════════════════════════════════════════════════╝${NC}"
  echo ""

  case "$MODE" in
    sim)
      echo -e "${CYAN}${BOLD}── Simulation Quick Start ──────────────────────────────${NC}"
      echo ""
      echo "  # Free-run simulation (viewer opens automatically)"
      echo "  ros2 launch rtc_mujoco_sim mujoco_sim.launch.py"
      echo ""
      echo "  # Sync-step mode (1:1 with controller)"
      echo "  ros2 launch rtc_mujoco_sim mujoco_sim.launch.py sim_mode:=sync_step"
      echo ""
      echo "  # Headless (no viewer window)"
      echo "  ros2 launch rtc_mujoco_sim mujoco_sim.launch.py enable_viewer:=false"
      echo ""
      echo "  # Hand simulation is built-in (fake_hand_response in mujoco_simulator.yaml)"
      echo "  # No need to run ur5e_hand_driver separately"
      echo ""
      echo "  # Send test commands"
      echo "  ros2 topic pub /target_joint_positions std_msgs/msg/Float64MultiArray \\"
      echo "      \"data: [0.0, -1.57, 0.0, 0.0, 0.0, 0.0]\""
      echo ""
      echo "  # Monitor RTF and steps"
      echo "  ros2 topic echo /sim/status"
      echo ""
      if [[ -n "$MJ_DIR" && -d "$MJ_DIR" ]]; then
        echo -e "  MuJoCo path : ${MJ_DIR}"
        echo -e "  cmake arg   : -Dmujoco_ROOT=${MJ_DIR}"
      else
        echo -e "  ${YELLOW}MuJoCo was not installed automatically.${NC}"
        echo -e "  ${YELLOW}See: https://github.com/google-deepmind/mujoco/releases${NC}"
      fi
      ;;

    robot)
      echo -e "${CYAN}${BOLD}── Real Robot Quick Start ──────────────────────────────${NC}"
      echo ""
      echo "  # Full system launch (replace IP as needed)"
      echo "  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
      echo "  ros2 launch rtc_controller_manager ur_control.launch.py robot_ip:=192.168.1.10"
      echo ""
      echo "  # Fake hardware (no physical robot — for testing)"
      echo "  ros2 launch rtc_controller_manager ur_control.launch.py use_fake_hardware:=true"
      echo ""
      echo "  # Hand UDP nodes only"
      echo "  ros2 launch ur5e_hand_driver hand_udp.launch.py"
      echo ""
      echo "  # Monitor control loop rate (should be ~500 Hz)"
      echo "  ros2 topic hz /forward_position_controller/commands"
      echo ""
      echo "  # Check E-STOP status"
      echo "  ros2 topic echo /system/estop_status"
      echo ""
      echo -e "  ${YELLOW}RT permissions: log out and back in, then verify:${NC}"
      echo "    ulimit -r   # should print 99"
      echo "    ulimit -l   # should print unlimited (Required to prevent RMW load failure)"
      ;;

    full)
      echo -e "${CYAN}${BOLD}── Quick Start ─────────────────────────────────────────${NC}"
      echo ""
      echo "  # Real robot"
      echo "  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
      echo "  ros2 launch rtc_controller_manager ur_control.launch.py robot_ip:=192.168.1.10"
      echo ""
      echo "  # MuJoCo simulation"
      echo "  ros2 launch rtc_mujoco_sim mujoco_sim.launch.py"
      echo "  ros2 launch rtc_mujoco_sim mujoco_sim.launch.py sim_mode:=sync_step"
      echo ""
      echo "  # Fake hardware (no robot, no simulation)"
      echo "  ros2 launch rtc_controller_manager ur_control.launch.py use_fake_hardware:=true"
      echo ""
      echo "  # Hand UDP only"
      echo "  ros2 launch ur5e_hand_driver hand_udp.launch.py"
      echo ""
      echo -e "  ${YELLOW}RT permissions: log out and back in, then verify:${NC}"
      echo "    ulimit -r   # should print 99"
      echo "    ulimit -l   # should print unlimited (Required to prevent RMW load failure)"
      ;;
  esac

  echo ""
  echo -e "${CYAN}${BOLD}── Monitoring & Validation ─────────────────────────────${NC}"
  echo "  ros2 run rtc_tools plot_ur_trajectory <workspace>/logging_data/ur5e_control_log_YYMMDD_HHMM.csv"
  echo "  ros2 run rtc_tools motion_editor_gui"
  echo "  ros2 run rtc_tools compare_mjcf_urdf        # MJCF vs URDF parameter comparison"
  echo ""
  echo -e "${CYAN}${BOLD}── VS Code Debugging ───────────────────────────────────${NC}"
  echo "  # Debug build + IntelliSense:"
  echo "  ./build.sh -d                 # or Ctrl+Shift+B in VS Code"
  echo ""
  echo "  # Launch debugger (F5 in VS Code):"
  echo "  #   C++: Launch rt_controller (Debug)"
  echo "  #   C++: Launch mujoco_simulator_node (Debug)"
  echo ""
  echo "  # Attach to running node:"
  echo "  #   C++: Attach to Node (Pick Process)"
  echo "  #   Requires: ptrace_scope=0"
  echo "  #   Run: echo 0 | sudo tee /proc/sys/kernel/yama/ptrace_scope"
  echo "  #   Or re-run: ./install.sh --ptrace-scope"
  echo ""
  echo "  See: docs/VSCODE_DEBUGGING.md"
  echo ""
  echo -e "${CYAN}${BOLD}── Documentation ───────────────────────────────────────${NC}"
  echo "  docs/CLAUDE.md           — full API and architecture reference"
  echo "  docs/VSCODE_DEBUGGING.md — VS Code GDB debugging guide"
  echo "  docs/RT_OPTIMIZATION.md  — RT tuning guide"
  echo "  docs/SHELL_SCRIPTS.md    — shell scripts usage guide"
  echo ""
}

# ══════════════════════════════════════════════════════════════════════════════
# Main installation sequence
# ══════════════════════════════════════════════════════════════════════════════

if [[ "$SKIP_DEPS" -eq 0 ]]; then
  check_workspace_structure
  check_prerequisites
  setup_workspace

  install_python_base_deps

  install_onnxruntime

  case "$MODE" in
    sim)
      install_pinocchio
      install_mujoco
      ;;
    robot)
      install_ur_driver
      install_pinocchio
      ;;
    full)
      install_ur_driver
      install_pinocchio
      install_mujoco
      ;;
  esac

  install_python_deps

  if [[ "$SKIP_DEBUG_SETUP" -eq 0 ]]; then
    install_vscode_debug_tools
  else
    info "Skipping GDB/debugger tools installation (--skip-debug)"
  fi
else
  info "Skipping system dependencies installation (--skip-deps)"
  check_workspace_structure
fi

setup_package

if [[ "$SKIP_BUILD" -eq 0 ]]; then
  auto_release_cpu_shield
  build_package
else
  info "Skipping colcon build (--skip-build)"
fi

if [[ "$SKIP_RT" -eq 0 ]]; then
  case "$MODE" in
    robot|full)
      install_cset_tools
      install_rt_permissions
      setup_rt_sudoers
      setup_rt_kernel_params
      setup_irq_affinity
      setup_udp_optimization
      setup_nvidia_rt
      setup_cpu_governor
      ;;
  esac
else
  info "Skipping RT permissions, IRQ affinity, UDP optimization, NVIDIA setup, and CPU governor (--skip-rt)"
fi

verify_installation
print_summary
