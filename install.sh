#!/bin/bash
# install.sh — RTC (Real-Time Controller) Installation
#
# Requires: GCC 10+ or Clang 13+ (C++20 support).
# All C++ packages use CMAKE_CXX_STANDARD 20 with strict compiler warnings.
#
# Usage:
#   ./install.sh              # deps + build (default, no RT setup)
#   ./install.sh sim          # MuJoCo simulation only
#   ./install.sh robot        # Real robot deps + build (no RT setup)
#   ./install.sh robot --all  # Real robot deps + build + RT setup
#   ./install.sh robot --rt   # RT setup only (skip deps/build)
#   ./install.sh --all --skip-rt  # deps + build, no RT setup
#   ./install.sh --help       # Show this help
#
# Function bodies for ROS2 / Python / external deps / dev tools / RT system
# setup live in repo_scripts/scripts/lib/install_*.sh. This file owns argument
# parsing, top-level orchestration, banner / quick-start summary.

set -eo pipefail

# ── 공통 부트스트랩 (rt_common source + logger + setup_env auto-source) ────
# shellcheck source=repo_scripts/scripts/lib/bootstrap.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/repo_scripts/scripts/lib/bootstrap.sh" "INSTALL"
INSTALL_SCRIPT_DIR="$_RT_SCRIPT_DIR"

# ── Modular install steps (functions live in lib/install_*.sh) ─────────────
# shellcheck source=repo_scripts/scripts/lib/install_ros2.sh
source "${INSTALL_SCRIPT_DIR}/repo_scripts/scripts/lib/install_ros2.sh"
# shellcheck source=repo_scripts/scripts/lib/install_python.sh
source "${INSTALL_SCRIPT_DIR}/repo_scripts/scripts/lib/install_python.sh"
# shellcheck source=repo_scripts/scripts/lib/install_deps.sh
source "${INSTALL_SCRIPT_DIR}/repo_scripts/scripts/lib/install_deps.sh"
# shellcheck source=repo_scripts/scripts/lib/install_dev.sh
source "${INSTALL_SCRIPT_DIR}/repo_scripts/scripts/lib/install_dev.sh"
# shellcheck source=repo_scripts/scripts/lib/install_rt.sh
source "${INSTALL_SCRIPT_DIR}/repo_scripts/scripts/lib/install_rt.sh"

# ── apt-get update 이중 호출 방지 ─────────────────────────────────────────────
_LAST_APT_UPDATE=0
apt_update_if_stale() {
  local now; now=$(date +%s)
  if (( now - _LAST_APT_UPDATE > 300 )); then
    sudo apt-get update -qq
    _LAST_APT_UPDATE=$now
  fi
}

# ── MPC dependency versions (scripts/build_deps.sh + verify_mpc_deps 에서 소비) ──
FMT_VERSION="11.1.4"
MIMALLOC_VERSION="2.1.7"
ALIGATOR_VERSION="0.19.0"

# ── MuJoCo version + default path (install_mujoco 에서 소비) ────────────────
MJ_VERSION="3.2.4"

# ── Mode & argument parsing ────────────────────────────────────────────────────
SKIP_DEPS=0
SKIP_BUILD=0
SKIP_MPC=0
MODE_VERIFY=0
DO_RT=0
SKIP_DEBUG_SETUP=0
SET_PTRACE_SCOPE=0
SET_PERF_TOOLS=0

show_help() {
  echo ""
  echo -e "${BOLD}RTC (Real-Time Controller) — install.sh${NC}"
  echo ""
  echo "Usage: $0 [MODE] [OPTIONS]"
  echo ""
  echo "Modes:"
  echo "  sim    — MuJoCo simulation only"
  echo "             Installs: ROS2 build tools, Pinocchio, ProxSuite, MPC deps (fmt/mimalloc/aligator), MuJoCo 3.x"
  echo "             Skips:    UR robot driver, RT scheduling permissions"
  echo ""
  echo "  robot  — Real robot only"
  echo "             Installs: ROS2 build tools, UR driver, Pinocchio, ProxSuite, MPC deps, RT permissions"
  echo "             Skips:    MuJoCo"
  echo ""
  echo "  full   — Complete installation (default)"
  echo "             Installs: everything above"
  echo ""
  echo "  verify — Verify installation only (no deps, no build, no RT setup)"
  echo "             Checks: MPC libraries under /usr/local, Panda URDF, workspace install/"
  echo ""
  echo "Scope:"
  echo "  --all             Install everything: deps + build + RT system setup"
  echo "  --rt              RT system setup only (skip deps and build)"
  echo "                    Configures: RT permissions, IRQ affinity, UDP optimization,"
  echo "                    NVIDIA RT setup, CPU governor, kernel params"
  echo ""
  echo "Options:"
  echo "  -d, --debug       Build with CMAKE_BUILD_TYPE=Debug"
  echo "  -r, --release     Build with CMAKE_BUILD_TYPE=Release (default)"
  echo "  -c, --clean       Remove build/, install/, and log/ before building"
  echo "  -p, --packages    Comma-separated list of specific packages to build"
  echo "  -j, --jobs N      Limit parallel workers for colcon (e.g. -j 4)"
  echo "  --skip-deps       Skip installing apt system dependencies"
  echo "  --skip-build      Skip compiling the packages (only download/setup)"
  echo "  --skip-rt         Skip RT system setup (overrides --all)"
  echo "  --skip-mpc        Skip MPC source-built deps (fmt/mimalloc/aligator)"
  echo "  --skip-debug      Skip GDB/debugger tools installation"
  echo "  --ptrace-scope    Set ptrace_scope=0 for VS Code Attach debugger"
  echo "                    (Required for 'Attach to Node' launch configuration)"
  echo "  --perf            Install Linux perf + Hotspot for profiling, set"
  echo "                    perf_event_paranoid=1 (enables non-root perf record)."
  echo "                    Required for 'ros2 launch ... enable_perf:=true'."
  echo "  --mujoco <path>   Use specific MuJoCo path"
  echo "  --help            Show this help"
  echo ""
  echo "Examples:"
  echo "  chmod +x install.sh"
  echo "  ./install.sh sim                # deps + build (simulation only)"
  echo "  ./install.sh robot --all        # deps + build + RT setup (real robot)"
  echo "  ./install.sh robot --rt         # RT setup only (already built)"
  echo "  ./install.sh full -c -j 4       # clean build, 4 parallel jobs"
  echo ""
  exit 0
}

# 공통 옵션 파싱 (rt_common.sh parse_common_args)
parse_common_args "$@"
MODE="$_COMMON_MODE"
BUILD_TYPE="$_COMMON_BUILD_TYPE"
CLEAN_BUILD="$_COMMON_CLEAN_BUILD"
PARALLEL_JOBS="$_COMMON_PARALLEL_JOBS"
MJ_DIR="$_COMMON_MJ_DIR"
CUSTOM_PACKAGES=("${_COMMON_CUSTOM_PACKAGES[@]}")
set -- "${REMAINING_ARGS[@]}"

# install.sh 고유 옵션 파싱
while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-deps)
      SKIP_DEPS=1
      shift
      ;;
    --skip-build)
      SKIP_BUILD=1
      shift
      ;;
    --all)
      DO_RT=1
      shift
      ;;
    --rt)
      DO_RT=1
      SKIP_DEPS=1
      SKIP_BUILD=1
      shift
      ;;
    --skip-rt)
      DO_RT=0
      shift
      ;;
    --skip-mpc)
      SKIP_MPC=1
      shift
      ;;
    --skip-debug)
      SKIP_DEBUG_SETUP=1
      shift
      ;;
    verify)
      MODE_VERIFY=1
      SKIP_DEPS=1
      SKIP_BUILD=1
      DO_RT=0
      shift
      ;;
    --ptrace-scope)
      SET_PTRACE_SCOPE=1
      shift
      ;;
    --perf)
      SET_PERF_TOOLS=1
      shift
      ;;
    -h|--help|help)
      show_help
      ;;
    *)
      error "Unknown argument: '$1'  (run $0 --help for usage)"
      ;;
  esac
done

# MJ_DIR default (parse_common_args 가 비워둔 경우)
[[ -z "$MJ_DIR" ]] && MJ_DIR="/opt/mujoco-${MJ_VERSION}"

case "$MODE" in
  sim)   MODE_DESC="Simulation  (MuJoCo + Pinocchio + ProxSuite + MPC deps, hand: fake response, no RT perms)" ;;
  robot) MODE_DESC="Real Robot  (UR driver + Pinocchio + ProxSuite + MPC deps + RT permissions, no MuJoCo)" ;;
  full)  MODE_DESC="Full        (UR driver + Pinocchio + ProxSuite + MPC deps + MuJoCo + RT permissions)" ;;
esac
[[ "$MODE_VERIFY" -eq 1 ]] && MODE_DESC="Verify      (no deps, no build — check install artifacts only)"

# ── Banner ─────────────────────────────────────────────────────────────────────
echo ""
echo -e "${BOLD}${BLUE}╔══════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}${BLUE}║    RTC (Real-Time Controller) — Installation Script  ║${NC}"
echo -e "${BOLD}${BLUE}╚══════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "  Mode : ${CYAN}${BOLD}${MODE_DESC}${NC}"
echo ""

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
  elif [[ ! -d "rtc-framework" ]]; then
    info "Cloning rtc-framework..."
    git clone https://github.com/hyujun/rtc-framework.git
    REPO_NAME="rtc-framework"
  else
    warn "rtc-framework already exists — skipping clone"
    REPO_NAME="rtc-framework"
  fi
}

# ── Build (build.sh에 위임) ───────────────────────────────────────────────────
build_package() {
  info "Building packages via build.sh..."

  local BUILD_ARGS=("$MODE" "--no-banner")
  [[ "$BUILD_TYPE" == "Debug" ]] && BUILD_ARGS+=("--debug")
  [[ "$CLEAN_BUILD" -eq 1 ]] && BUILD_ARGS+=("--clean")
  [[ -n "$PARALLEL_JOBS" ]] && BUILD_ARGS+=("--jobs" "$PARALLEL_JOBS")
  [[ -n "$MJ_DIR" ]] && BUILD_ARGS+=("--mujoco" "$MJ_DIR")
  [[ ${#CUSTOM_PACKAGES[@]} -gt 0 ]] && BUILD_ARGS+=("--packages" "$(IFS=','; echo "${CUSTOM_PACKAGES[*]}")")

  bash "${INSTALL_SCRIPT_DIR}/build.sh" "${BUILD_ARGS[@]}" || error "Build failed!"

  # 빌드 후 최신 overlay 소싱 (verify_installation 등 후속 작업용)
  source "${WORKSPACE}/install/setup.bash" || true
  success "All packages built and sourced"
}

# ── Verify installation ─────────────────────────────────────────────────────────
verify_installation() {
  info "Verifying installation..."
  source "$WORKSPACE/install/setup.bash" || true
  local failed=0
  local all_pkgs
  read -r -a all_pkgs <<< "$(get_base_packages) $(get_robot_packages)"
  for pkg in "${all_pkgs[@]}"; do
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
  info "Available executables (udp_hand_driver):"
  ros2 pkg executables udp_hand_driver 2>/dev/null || true

  mkdir -p "${WORKSPACE}/logging_data/stats" "${WORKSPACE}/logging_data/ur_plot"
  success "Log directories ready (${WORKSPACE}/logging_data, ${WORKSPACE}/logging_data/ur_plot)"

  # RT 환경 검증 (robot/full 모드; verify 모드에선 skip)
  if [[ "$MODE_VERIFY" -eq 0 ]] && [[ "$MODE" == "robot" || "$MODE" == "full" ]]; then
    local CHECK_SCRIPT
    CHECK_SCRIPT="${INSTALL_SCRIPT_DIR}/repo_scripts/scripts/check_rt_setup.sh"
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
      echo "  # Hand simulation is built-in (fake_response in integrated_bringup/config/ur5e_hand/mujoco_simulator.yaml)"
      echo "  # No need to run udp_hand_driver separately"
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
      echo "  ros2 launch integrated_bringup robot.launch.py robot_ip:=192.168.1.10"
      echo ""
      echo "  # Fake hardware (no physical robot — for testing)"
      echo "  ros2 launch integrated_bringup robot.launch.py use_fake_hardware:=true"
      echo ""
      echo "  # Hand UDP nodes only"
      echo "  ros2 launch udp_hand_driver udp_hand.launch.py"
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
      echo "  ros2 launch integrated_bringup robot.launch.py robot_ip:=192.168.1.10"
      echo ""
      echo "  # MuJoCo simulation"
      echo "  ros2 launch integrated_bringup sim.launch.py"
      echo ""
      echo "  # Fake hardware (no robot, no simulation)"
      echo "  ros2 launch integrated_bringup robot.launch.py use_fake_hardware:=true"
      echo ""
      echo "  # Hand UDP only"
      echo "  ros2 launch udp_hand_driver udp_hand.launch.py"
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
  echo "  #   C++: Launch integrated_rt_controller (Debug)"
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
  echo "  CLAUDE.md                — agent context (harness, invariants, escalation)"
  echo "  docs/VSCODE_DEBUGGING.md — VS Code GDB debugging guide"
  echo "  docs/RT_OPTIMIZATION.md  — RT tuning guide"
  echo "  repo_scripts/README.md   — shell scripts usage guide (RT setup, env, build)"
  echo ""
}

# ══════════════════════════════════════════════════════════════════════════════
# Main installation sequence
# ══════════════════════════════════════════════════════════════════════════════

# ── Verify-only short circuit ─────────────────────────────────────────────────
if [[ "$MODE_VERIFY" -eq 1 ]]; then
  check_workspace_structure "$INSTALL_SCRIPT_DIR"
  check_prerequisites
  verify_mpc_deps || true
  if [[ -d "${WORKSPACE}/install" ]]; then
    verify_installation
  else
    warn "Workspace not built yet — skipping colcon package verification"
  fi
  exit 0
fi

if [[ "$SKIP_DEPS" -eq 0 ]]; then
  check_workspace_structure "$INSTALL_SCRIPT_DIR"
  check_prerequisites
  ensure_venv
  setup_workspace

  install_python_base_deps

  install_onnxruntime

  install_behaviortree

  case "$MODE" in
    sim)
      install_pinocchio
      install_proxsuite
      install_mpc_deps
      install_mujoco
      ;;
    robot)
      install_ur_driver
      install_pinocchio
      install_proxsuite
      install_mpc_deps
      ;;
    full)
      install_ur_driver
      install_pinocchio
      install_proxsuite
      install_mpc_deps
      install_mujoco
      ;;
  esac

  install_python_deps

  if [[ "$SKIP_DEBUG_SETUP" -eq 0 ]]; then
    install_vscode_debug_tools
  else
    info "Skipping GDB/debugger tools installation (--skip-debug)"
  fi

  if [[ "$SET_PERF_TOOLS" -eq 1 ]]; then
    install_perf_tools
  fi
else
  info "Skipping system dependencies installation (--skip-deps)"
  check_workspace_structure "$INSTALL_SCRIPT_DIR"
fi

setup_package

if [[ "$SKIP_BUILD" -eq 0 ]]; then
  auto_release_cpu_shield
  build_package
else
  info "Skipping colcon build (--skip-build)"
fi

if [[ "$DO_RT" -eq 1 ]]; then
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
    sim)
      info "Skipping RT setup (sim mode does not require RT configuration)"
      ;;
  esac
else
  if [[ "$MODE" == "robot" || "$MODE" == "full" ]]; then
    info "RT setup skipped (use --all or --rt to configure RT permissions, kernel params, etc.)"
  fi
fi

verify_installation
verify_mpc_deps || true
print_summary
