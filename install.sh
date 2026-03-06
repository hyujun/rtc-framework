#!/bin/bash
# install.sh — UR5e RT Controller Installation
#
# Usage:
#   ./install.sh              # full installation (default)
#   ./install.sh sim          # MuJoCo simulation only
#   ./install.sh robot        # Real robot only
#   ./install.sh full         # Explicit full installation
#   ./install.sh --help       # Show this help

set -e

# ── Colors ─────────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# ── Mode selection ─────────────────────────────────────────────────────────────
MODE="${1:-full}"
case "$MODE" in
  sim|simulation)
    MODE=sim
    MODE_DESC="Simulation  (MuJoCo + Pinocchio, no UR driver, no RT perms)"
    ;;
  robot|realrobot|real)
    MODE=robot
    MODE_DESC="Real Robot  (UR driver + Pinocchio + RT permissions, no MuJoCo)"
    ;;
  full|all|"")
    MODE=full
    MODE_DESC="Full        (UR driver + Pinocchio + MuJoCo + RT permissions)"
    ;;
  -h|--help|help)
    echo ""
    echo -e "${BOLD}UR5e RT Controller — install.sh${NC}"
    echo ""
    echo "Usage: $0 [MODE]"
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
    echo "Examples:"
    echo "  chmod +x install.sh"
    echo "  ./install.sh sim      # developer workstation, no hardware"
    echo "  ./install.sh robot    # robot-side machine"
    echo "  ./install.sh          # full setup"
    echo ""
    exit 0
    ;;
  *)
    echo -e "${RED}Unknown mode: '$1'${NC}"
    echo "Usage: $0 [sim|robot|full]  or  $0 --help"
    exit 1
    ;;
esac

# ── Banner ─────────────────────────────────────────────────────────────────────
echo ""
echo -e "${BOLD}${BLUE}╔══════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}${BLUE}║     UR5e RT Controller — Installation Script         ║${NC}"
echo -e "${BOLD}${BLUE}╚══════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "  Mode : ${CYAN}${BOLD}${MODE_DESC}${NC}"
echo ""

# ── Helper functions ───────────────────────────────────────────────────────────
info()    { echo -e "${BLUE}▶ $*${NC}"; }
success() { echo -e "${GREEN}✔ $*${NC}"; }
warn()    { echo -e "${YELLOW}⚠ $*${NC}"; }
error()   { echo -e "${RED}✘ $*${NC}"; exit 1; }

# ── Common: ROS2 + Ubuntu check ────────────────────────────────────────────────
check_prerequisites() {
  if ! command -v ros2 &>/dev/null; then
    error "ROS2 not found. Install ROS2 Humble or Jazzy first: https://docs.ros.org/en/jazzy/Installation.html"
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

  UBUNTU_VERSION=$(lsb_release -rs 2>/dev/null || echo "unknown")
  local SUPPORTED_UBUNTU=("22.04" "24.04")
  if [[ ! " ${SUPPORTED_UBUNTU[*]} " =~ " ${UBUNTU_VERSION} " ]]; then
    warn "Unsupported Ubuntu (${UBUNTU_VERSION}). Supported: 22.04 (Humble), 24.04 (Jazzy). Continuing..."
  fi
  if [[ "$ROS_DISTRO_DETECTED" != "humble" && "$ROS_DISTRO_DETECTED" != "jazzy" ]]; then
    warn "Unsupported ROS2 distro: ${ROS_DISTRO_DETECTED}. Supported: humble, jazzy. Continuing..."
  fi
}

# ── Common: Workspace + build tools ───────────────────────────────────────────
setup_workspace() {
  # Auto-detect workspace from script location.
  # Expected layout: <workspace>/src/<repo>/install.sh
  local SCRIPT_DIR
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  local SRC_DIR
  SRC_DIR="$(dirname "$SCRIPT_DIR")"
  local DETECTED_WS
  DETECTED_WS="$(dirname "$SRC_DIR")"

  if [[ "$(basename "$SRC_DIR")" == "src" && -d "$DETECTED_WS" ]]; then
    WORKSPACE="$DETECTED_WS"
    info "Workspace auto-detected: $WORKSPACE"
  else
    WORKSPACE=~/ur_ws
    warn "Script is not under <workspace>/src/<repo>/ — using default: $WORKSPACE"
  fi

  info "Setting up workspace: $WORKSPACE"
  mkdir -p "$WORKSPACE/src"

  info "Installing ROS2 build tools (${ROS_PKG_PREFIX})..."
  sudo apt-get update -qq
  sudo apt-get install -y \
      ${ROS_PKG_PREFIX}-ament-cmake \
      ${ROS_PKG_PREFIX}-ament-cmake-gtest \
      python3-colcon-common-extensions \
      python3-vcstool \
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
      ${ROS_PKG_PREFIX}-industrial-msgs \
      > /dev/null
  success "UR robot driver installed"
}

# ── Pinocchio (all modes — needed by PinocchioController / ClikController) ────
install_pinocchio() {
  info "Installing Pinocchio (model-based controllers, ${ROS_PKG_PREFIX})..."
  if sudo apt-get install -y ${ROS_PKG_PREFIX}-pinocchio >/dev/null 2>&1; then
    success "Pinocchio installed via ${ROS_PKG_PREFIX}-pinocchio"
  else
    warn "${ROS_PKG_PREFIX}-pinocchio not found, trying robotpkg..."
    sudo sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -sc) robotpkg' \
        > /etc/apt/sources.list.d/robotpkg.list"
    curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.key \
        | sudo apt-key add - 2>/dev/null || true
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
      warn "Pinocchio not installed — PinocchioController / ClikController / OperationalSpaceController unavailable"
      warn "See: https://stack-of-tasks.github.io/pinocchio/download.html"
    fi
  fi
}

# ── MuJoCo 3.x (sim + full) ────────────────────────────────────────────────────
MJ_VERSION="3.2.4"
MJ_DIR="/opt/mujoco-${MJ_VERSION}"

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
  sudo apt-get update -qq
  sudo apt-get install -y \
      python3-matplotlib \
      python3-pandas \
      python3-numpy \
      python3-scipy \
      python3-pyqt5 \
      > /dev/null
  success "Python dependencies installed via apt"
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
  for pkg in ur5e_rt_base ur5e_rt_controller ur5e_hand_udp ur5e_mujoco_sim ur5e_tools; do
    if [[ ! -e "$pkg" ]]; then
      ln -s "${REPO_NAME}/$pkg" "$pkg"
    fi
  done
}

# ── Build ──────────────────────────────────────────────────────────────────────
build_package() {
  info "Building all ur5e packages..."
  cd "$WORKSPACE"

  local CMAKE_ARGS=()
  if [[ -n "$MJ_DIR" && -d "$MJ_DIR" ]]; then
    # MuJoCo binary release (tarball) does NOT include lib/cmake/mujoco/.
    # Pass mujoco_ROOT so CMakeLists.txt can locate the .so and headers via find_library.
    CMAKE_ARGS+=("-Dmujoco_ROOT=${MJ_DIR}")
    info "MuJoCo root: ${MJ_DIR}"
  fi

  # Build order: ur5e_rt_base first (header-only, no deps), then the rest
  local PACKAGES=(ur5e_rt_base ur5e_rt_controller ur5e_hand_udp ur5e_tools)
  if [[ -n "$MJ_DIR" && -d "$MJ_DIR" ]]; then
    PACKAGES+=(ur5e_mujoco_sim)
  fi

  if [[ ${#CMAKE_ARGS[@]} -gt 0 ]]; then
    colcon build \
        --packages-select "${PACKAGES[@]}" \
        --symlink-install \
        --cmake-args "${CMAKE_ARGS[@]}"
  else
    colcon build \
        --packages-select "${PACKAGES[@]}" \
        --symlink-install
  fi

  source install/setup.bash
  grep -qF "$WORKSPACE/install/setup.bash" ~/.bashrc || \
      echo "source $WORKSPACE/install/setup.bash" >> ~/.bashrc
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
  if ! grep -q "@realtime.*memlock" /etc/security/limits.conf; then
    echo "@realtime - memlock unlimited" | sudo tee -a /etc/security/limits.conf > /dev/null
  fi

  success "RT permissions configured (rtprio=99, memlock=unlimited)"
  warn "IMPORTANT: Log out and log back in for RT permissions to take effect"
  warn "Verify with: ulimit -r  (should print 99)"
}

# ── [방안 D] NIC IRQ affinity (robot + full) ────────────────────────────────
# RT 코어(Core 2-5)를 NIC 인터럽트로부터 보호.
# 모든 하드웨어 IRQ를 Core 0-1로 제한한다.
setup_irq_affinity() {
  local SCRIPT
  SCRIPT="$(dirname "$0")/ur5e_rt_controller/scripts/setup_irq_affinity.sh"

  if [[ ! -f "$SCRIPT" ]]; then
    warn "setup_irq_affinity.sh not found — skipping IRQ affinity setup"
    warn "Run manually after build: sudo $WORKSPACE/src/ur5e-rt-controller/ur5e_rt_controller/scripts/setup_irq_affinity.sh"
    return
  fi

  info "Configuring NIC IRQ affinity (Core 0-1 only)..."
  if sudo bash "$SCRIPT"; then
    success "IRQ affinity configured — RT cores 2-5 protected from NIC interrupts"
    warn "NOTE: IRQ affinity resets on reboot. To make permanent:"
    warn "  Add 'sudo $SCRIPT' to /etc/rc.local or a systemd oneshot service"
  else
    warn "IRQ affinity setup failed — continuing without it"
    warn "Run manually: sudo $SCRIPT [NIC_NAME]"
  fi
}

# ── Verify installation ─────────────────────────────────────────────────────────
verify_installation() {
  info "Verifying installation..."
  source "$WORKSPACE/install/setup.bash"
  local failed=0
  for pkg in ur5e_rt_base ur5e_rt_controller ur5e_hand_udp ur5e_tools; do
    if ros2 pkg list 2>/dev/null | grep -q "^${pkg}$"; then
      success "Package registered: $pkg"
    else
      warn "Package not found: $pkg"
      failed=1
    fi
  done
  [[ $failed -eq 1 ]] && error "Installation verification failed"

  info "Available executables (ur5e_rt_controller):"
  ros2 pkg executables ur5e_rt_controller 2>/dev/null || true
  info "Available executables (ur5e_hand_udp):"
  ros2 pkg executables ur5e_hand_udp 2>/dev/null || true

  mkdir -p /tmp/ur5e_logs /tmp/ur5e_stats ~/ur_plots
  success "Log directories ready (/tmp/ur5e_logs, /tmp/ur5e_stats, ~/ur_plots)"
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
      echo "  ros2 launch ur5e_mujoco_sim mujoco_sim.launch.py"
      echo ""
      echo "  # Sync-step mode (1:1 with controller)"
      echo "  ros2 launch ur5e_mujoco_sim mujoco_sim.launch.py sim_mode:=sync_step"
      echo ""
      echo "  # Headless (no viewer window)"
      echo "  ros2 launch ur5e_mujoco_sim mujoco_sim.launch.py enable_viewer:=false"
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
        echo -e "  cmake arg   : -Dmujoco_DIR=${MJ_DIR}/lib/cmake/mujoco"
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
      echo "  ros2 launch ur5e_rt_controller ur_control.launch.py robot_ip:=192.168.1.10"
      echo ""
      echo "  # Fake hardware (no physical robot — for testing)"
      echo "  ros2 launch ur5e_rt_controller ur_control.launch.py use_fake_hardware:=true"
      echo ""
      echo "  # Hand UDP nodes only"
      echo "  ros2 launch ur5e_hand_udp hand_udp.launch.py"
      echo ""
      echo "  # Monitor control loop rate (should be ~500 Hz)"
      echo "  ros2 topic hz /forward_position_controller/commands"
      echo ""
      echo "  # Check E-STOP status"
      echo "  ros2 topic echo /system/estop_status"
      echo ""
      echo -e "  ${YELLOW}RT permissions: log out and back in, then verify:${NC}"
      echo "    ulimit -r   # should print 99"
      echo "    ulimit -l   # should print unlimited"
      ;;

    full)
      echo -e "${CYAN}${BOLD}── Quick Start ─────────────────────────────────────────${NC}"
      echo ""
      echo "  # Real robot"
      echo "  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
      echo "  ros2 launch ur5e_rt_controller ur_control.launch.py robot_ip:=192.168.1.10"
      echo ""
      echo "  # MuJoCo simulation"
      echo "  ros2 launch ur5e_mujoco_sim mujoco_sim.launch.py"
      echo "  ros2 launch ur5e_mujoco_sim mujoco_sim.launch.py sim_mode:=sync_step"
      echo ""
      echo "  # Fake hardware (no robot, no simulation)"
      echo "  ros2 launch ur5e_rt_controller ur_control.launch.py use_fake_hardware:=true"
      echo ""
      echo "  # Hand UDP only"
      echo "  ros2 launch ur5e_hand_udp hand_udp.launch.py"
      echo ""
      echo -e "  ${YELLOW}RT permissions: log out and back in, then verify:${NC}"
      echo "    ulimit -r   # should print 99"
      echo "    ulimit -l   # should print unlimited"
      ;;
  esac

  echo ""
  echo -e "${CYAN}${BOLD}── Monitoring ──────────────────────────────────────────${NC}"
  echo "  ros2 run ur5e_tools monitor_data_health.py"
  echo "  ros2 run ur5e_tools plot_ur_trajectory.py /tmp/ur5e_control_log.csv"
  echo "  ros2 run ur5e_tools motion_editor_gui.py"
  echo ""
  echo -e "${CYAN}${BOLD}── Documentation ───────────────────────────────────────${NC}"
  echo "  docs/RT_OPTIMIZATION.md  — RT tuning guide"
  echo "  docs/CHANGELOG.md        — version history"
  echo "  CLAUDE.md                — full API and architecture reference"
  echo ""
}

# ══════════════════════════════════════════════════════════════════════════════
# Main installation sequence
# ══════════════════════════════════════════════════════════════════════════════

check_prerequisites
setup_workspace

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
setup_package
build_package

case "$MODE" in
  robot|full)
    install_rt_permissions
    setup_irq_affinity
    ;;
esac

verify_installation
print_summary
