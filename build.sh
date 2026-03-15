#!/bin/bash
# build.sh — UR5e RT Controller Build Script
#
# Usage:
#   ./build.sh                              # full build (robot + sim)
#   ./build.sh sim                          # simulation only (MuJoCo required)
#   ./build.sh robot                        # real robot only (no MuJoCo)
#   ./build.sh full                         # explicit full build
#   ./build.sh sim --mujoco /opt/mujoco-3.2.4  # sim with custom MuJoCo path
#   ./build.sh -d --export-compile-commands # debug build with IntelliSense DB
#   ./build.sh --help                       # show help

set -e

# ── Colors ─────────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# ── Helper functions ───────────────────────────────────────────────────────────
info()    { echo -e "${BLUE}▶ $*${NC}"; }
success() { echo -e "${GREEN}✔ $*${NC}"; }
warn()    { echo -e "${YELLOW}⚠ $*${NC}"; }
error()   { echo -e "${RED}✘ $*${NC}"; exit 1; }

# ── CPU shield 자동 관리 (빌드 전) ──────────────────────────────────────────
# cset shield가 활성이면 자동 해제하여 전체 코어로 빌드한다.
# isolcpus(GRUB 고정)는 재부팅 없이 해제 불가 → 경고만 출력.
auto_release_cpu_shield() {
  local isolated
  isolated=$(cat /sys/devices/system/cpu/isolated 2>/dev/null || echo "")

  if [[ -z "$isolated" ]]; then
    return 0  # 격리 없음 → 전체 코어 사용 가능
  fi

  local available total
  available=$(nproc)
  total=$(nproc --all)
  warn "CPU 격리 감지: Core ${isolated} 격리 중 (${available}/${total} 코어 사용 가능)"

  # Case 1: cset shield 활성 → 자동 해제
  local SCRIPT_DIR_BUILD
  SCRIPT_DIR_BUILD="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  local SHIELD_SCRIPT="${SCRIPT_DIR_BUILD}/ur5e_rt_controller/scripts/cpu_shield.sh"
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

  # Case 2: isolcpus (GRUB 고정) → 해제 불가, 경고만
  if grep -q "isolcpus=" /proc/cmdline 2>/dev/null; then
    warn "isolcpus GRUB 파라미터 감지 — 재부팅 없이 해제 불가"
    warn "빌드에 ${available}/${total} 코어만 사용됩니다"
    warn "권장: isolcpus를 GRUB에서 제거하고 cset shield 방식으로 전환하세요"
    return 0
  fi

  return 0
}

# ── ROS2 auto-source ─────────────────────────────────────────────────────────
# ros2 커맨드가 PATH에 없으면 /opt/ros/ 에서 탐색 후 자동 소싱.
# build.sh만 실행해도 빌드 후 ros2 launch 등 사용 가능.
ensure_ros2_sourced() {
  if command -v ros2 &>/dev/null; then
    return 0
  fi

  warn "ros2 command not found in PATH. Searching /opt/ros/ ..."
  if [[ -d /opt/ros ]]; then
    for _distro in jazzy humble iron rolling; do
      if [[ -f "/opt/ros/${_distro}/setup.bash" ]]; then
        info "Found ROS2 ${_distro} at /opt/ros/${_distro} — sourcing setup.bash ..."
        # shellcheck disable=SC1090
        source "/opt/ros/${_distro}/setup.bash"
        success "ROS2 ${_distro} sourced"
        return 0
      fi
    done
  fi

  error "ROS2 not found. Install ROS2 or source setup.bash before running build.sh"
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

# ── Mode & argument parsing ────────────────────────────────────────────────────
MODE="full"
MJ_DIR=""
BUILD_TYPE="Release"
CLEAN_BUILD=0
CUSTOM_PACKAGES=()
PARALLEL_JOBS=""
NO_SYMLINK=0
EXPORT_COMPILE_COMMANDS=0

# Default MuJoCo search path
MJ_DEFAULT="/opt/mujoco-3.2.4"

show_help() {
  echo ""
  echo -e "${BOLD}UR5e RT Controller — build.sh${NC}"
  echo ""
  echo "Usage: $0 [MODE] [OPTIONS]"
  echo ""
  echo "Modes:"
  echo "  robot   Build packages for real robot (no MuJoCo)"
  echo "            Packages: ur5e_msgs, ur5e_rt_base, ur5e_description, ur5e_status_monitor,"
  echo "                      ur5e_rt_controller, ur5e_hand_udp, ur5e_tools"
  echo ""
  echo "  sim     Build for simulation (MuJoCo required, hand uses fake response)"
  echo "            Packages: ur5e_msgs, ur5e_rt_base, ur5e_description, ur5e_status_monitor,"
  echo "                      ur5e_rt_controller, ur5e_hand_udp, ur5e_mujoco_sim, ur5e_tools"
  echo "            Note: ur5e_hand_udp is a build dependency but not used at runtime"
  echo ""
  echo "  full    Build all packages (default)"
  echo "            Packages: all of the above"
  echo ""
  echo "Options:"
  echo "  -d, --debug                Build with CMAKE_BUILD_TYPE=Debug"
  echo "  -r, --release              Build with CMAKE_BUILD_TYPE=Release (default)"
  echo "  -c, --clean                Remove build/, install/, and log/ before building"
  echo "  -p, --packages             Comma-separated list of specific packages to build"
  echo "                             (Overrides default packages for the chosen mode)"
  echo "  -j, --jobs N               Limit parallel workers (e.g. -j 4)"
  echo "  -e, --export-compile-commands  Generate compile_commands.json for VS Code"
  echo "                             IntelliSense and clangd (sets CMAKE_EXPORT_COMPILE_COMMANDS=ON)"
  echo "  --no-symlink               Do not use --symlink-install"
  echo "  --mujoco <path>            Path to MuJoCo install dir (e.g. /opt/mujoco-3.2.4)"
  echo "                             Auto-detected from $MJ_DEFAULT if not specified"
  echo "  --help                     Show this help"
  echo ""
  echo "Examples:"
  echo "  ./build.sh robot"
  echo "  ./build.sh sim"
  echo "  ./build.sh sim --mujoco /opt/mujoco-3.2.4"
  echo "  ./build.sh full"
  echo ""
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    robot|real|realrobot)
      MODE=robot
      shift
      ;;
    sim|simulation)
      MODE=sim
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
    -e|--export-compile-commands)
      EXPORT_COMPILE_COMMANDS=1
      shift
      ;;
    --no-symlink)
      NO_SYMLINK=1
      shift
      ;;
    --mujoco)
      [[ -z "${2:-}" ]] && error "--mujoco requires a path argument"
      MJ_DIR="$2"
      shift 2
      ;;
    -h|--help|help)
      show_help
      exit 0
      ;;
    *)
      error "Unknown argument: '$1'  (run $0 --help for usage)"
      ;;
  esac
done

# ── MuJoCo path resolution ─────────────────────────────────────────────────────
# sim/full 모드에서 --mujoco 미지정 시 기본 경로 자동 탐색
if [[ "$MODE" != "robot" && -z "$MJ_DIR" ]]; then
  if [[ -d "$MJ_DEFAULT" ]]; then
    MJ_DIR="$MJ_DEFAULT"
    info "MuJoCo auto-detected: $MJ_DIR"
  else
    # /opt/mujoco-* 패턴 중 가장 최신 버전 탐색
    LATEST_MJ=$(ls -d /opt/mujoco-* 2>/dev/null | sort -V | tail -1 || true)
    if [[ -n "$LATEST_MJ" ]]; then
      MJ_DIR="$LATEST_MJ"
      warn "MuJoCo auto-detected (non-default): $MJ_DIR"
    fi
  fi
fi

# sim 모드에서 MuJoCo 없으면 에러
if [[ "$MODE" == "sim" && ( -z "$MJ_DIR" || ! -d "$MJ_DIR" ) ]]; then
  error "MuJoCo not found for sim mode. Install MuJoCo or specify --mujoco <path>"
fi

# full 모드에서 MuJoCo 없으면 경고 후 robot 패키지만 빌드
if [[ "$MODE" == "full" && ( -z "$MJ_DIR" || ! -d "$MJ_DIR" ) ]]; then
  warn "MuJoCo not found — ur5e_mujoco_sim will be skipped"
  warn "Specify --mujoco <path> to include simulation packages"
  MJ_DIR=""
fi

# ── Banner ─────────────────────────────────────────────────────────────────────
case "$MODE" in
  robot) MODE_DESC="Real Robot  (ur5e_msgs, ur5e_rt_base, ur5e_status_monitor, ur5e_rt_controller, ur5e_hand_udp, ur5e_tools)" ;;
  sim)   MODE_DESC="Simulation  (ur5e_msgs, ..., ur5e_mujoco_sim — hand: fake response)" ;;
  full)  MODE_DESC="Full        (all packages)" ;;
esac

echo ""
echo -e "${BOLD}${BLUE}╔══════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}${BLUE}║         UR5e RT Controller — Build Script            ║${NC}"
echo -e "${BOLD}${BLUE}╚══════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "  Mode : ${CYAN}${BOLD}${MODE_DESC}${NC}"
echo -e "  Build: ${CYAN}${BOLD}${BUILD_TYPE}${NC}"
echo ""

# ── ROS2 environment ──────────────────────────────────────────────────────────
ensure_ros2_sourced

# ── Workspace detection ────────────────────────────────────────────────────────
check_workspace_structure

# ── Package selection by mode ──────────────────────────────────────────────────
if [[ ${#CUSTOM_PACKAGES[@]} -gt 0 ]]; then
  PACKAGES=("${CUSTOM_PACKAGES[@]}")
  info "Using custom package list"
else
  case "$MODE" in
    robot)
      PACKAGES=(ur5e_msgs ur5e_rt_base ur5e_description ur5e_status_monitor ur5e_rt_controller ur5e_hand_udp ur5e_tools)
      ;;
    sim)
      PACKAGES=(ur5e_msgs ur5e_rt_base ur5e_description ur5e_status_monitor ur5e_rt_controller ur5e_hand_udp ur5e_mujoco_sim ur5e_tools)
      ;;
    full)
      PACKAGES=(ur5e_msgs ur5e_rt_base ur5e_description ur5e_status_monitor ur5e_rt_controller ur5e_hand_udp ur5e_tools)
      if [[ -n "$MJ_DIR" && -d "$MJ_DIR" ]]; then
        PACKAGES+=(ur5e_mujoco_sim)
      fi
      ;;
  esac
fi

# ── CPU shield auto-release (빌드 전 격리 해제) ────────────────────────────────
auto_release_cpu_shield

# ── Build ──────────────────────────────────────────────────────────────────────
info "Building: ${PACKAGES[*]}"
cd "$WORKSPACE"

if [[ "$CLEAN_BUILD" -eq 1 ]]; then
  info "Cleaning previous build artifacts..."
  rm -rf build/ install/ log/
  success "Clean complete"
fi

CMAKE_ARGS=("-DCMAKE_BUILD_TYPE=${BUILD_TYPE}")
if [[ -n "$MJ_DIR" && -d "$MJ_DIR" ]]; then
  CMAKE_ARGS+=("-Dmujoco_ROOT=${MJ_DIR}")
  info "MuJoCo root path: ${MJ_DIR}"
fi

# ── venv: force system Python for CMake ───────────────────────────────────────
# When a venv is active, CMake's FindPython picks the venv Python, which may
# lack numpy headers and cause eigenpy/pinocchio cmake configuration to fail.
# Force system Python so pinocchio/eigenpy find the apt-installed numpy.
if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  SYS_PYTHON=$(command -v python3 || true)
  SYS_PYTHON=$(readlink -f "${SYS_PYTHON}" 2>/dev/null || echo "${SYS_PYTHON}")
  if [[ "$SYS_PYTHON" == "${VIRTUAL_ENV}"* ]]; then
    SYS_PYTHON="/usr/bin/python3"
  fi
  CMAKE_ARGS+=("-DPython3_EXECUTABLE=${SYS_PYTHON}")
  CMAKE_ARGS+=("-DPython3_FIND_VIRTUALENV=STANDARD")
  warn "Venv detected — cmake will use system Python: ${SYS_PYTHON}"
fi

# compile_commands.json: Debug 빌드이거나 --export-compile-commands 지정 시 자동 활성화
if [[ "$EXPORT_COMPILE_COMMANDS" -eq 1 || "$BUILD_TYPE" == "Debug" ]]; then
  CMAKE_ARGS+=("-DCMAKE_EXPORT_COMPILE_COMMANDS=ON")
  info "compile_commands.json will be generated (VS Code IntelliSense)"
fi

COLCON_ARGS=("--packages-select" "${PACKAGES[@]}")

if [[ "$NO_SYMLINK" -eq 0 ]]; then
  COLCON_ARGS+=("--symlink-install")
fi

if [[ -n "$PARALLEL_JOBS" ]]; then
  COLCON_ARGS+=("--parallel-workers" "$PARALLEL_JOBS")
  info "Limiting parallel workers to $PARALLEL_JOBS"
fi

if [[ ${#CMAKE_ARGS[@]} -gt 0 ]]; then
  COLCON_ARGS+=("--cmake-args" "${CMAKE_ARGS[@]}")
fi

colcon build "${COLCON_ARGS[@]}" || error "Build failed!"

source install/setup.bash

# ── compile_commands.json — VS Code IntelliSense 연동 ─────────────────────────
# Debug 빌드 또는 --export-compile-commands 옵션 사용 시 자동 생성됨.
# .vscode/c_cpp_properties.json 의 compileCommands 경로와 일치:
#   ${workspaceFolder}/../../build/ur5e_rt_controller/compile_commands.json
if [[ "$EXPORT_COMPILE_COMMANDS" -eq 1 || "$BUILD_TYPE" == "Debug" ]]; then
  CC_SRC="$WORKSPACE/build/ur5e_rt_controller/compile_commands.json"
  if [[ -f "$CC_SRC" ]]; then
    success "compile_commands.json generated: $CC_SRC"
    success "VS Code IntelliSense will use this file automatically"
  else
    warn "compile_commands.json not found at $CC_SRC — IntelliSense may be limited"
  fi
fi

# ── RT Setup Verification ─────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CHECK_SCRIPT="${SCRIPT_DIR}/ur5e_rt_controller/scripts/check_rt_setup.sh"

if [[ -f "$CHECK_SCRIPT" ]]; then
  case "$MODE" in
    robot|full)
      echo ""
      info "Verifying RT system configuration..."
      bash "$CHECK_SCRIPT" --summary || true
      ;;
    sim)
      # sim 모드: RT permissions만 간략 체크
      MEMLOCK=$(ulimit -l)
      if [[ "$MEMLOCK" != "unlimited" ]]; then
        echo ""
        warn "Your current memlock limit is $MEMLOCK (not unlimited)."
        warn "Running the custom_controller may fail with RMW load errors due to mlockall()."
        warn "Run: ./install.sh robot  or  ulimit -l unlimited"
        echo ""
      fi
      ;;
  esac
else
  # fallback: check_rt_setup.sh 미발견 시 기존 로직
  MEMLOCK=$(ulimit -l)
  if [[ "$MEMLOCK" != "unlimited" ]]; then
    echo ""
    warn "Your current memlock limit is $MEMLOCK (not unlimited)."
    warn "Running the custom_controller may fail with RMW load errors due to mlockall()."
    warn "  ulimit -l unlimited"
    echo ""
  fi
fi

# ── GDB/Debugger Hint ────────────────────────────────────────────────────────
if [[ "$BUILD_TYPE" == "Debug" ]]; then
  echo ""
  echo -e "${CYAN}${BOLD}── VS Code Debugging ───────────────────────────────────${NC}"
  echo "  Debug build complete — ready for GDB debugging in VS Code."
  echo ""
  echo "  Launch debugger : F5 → 'C++: Launch custom_controller (Debug)'"
  echo "  Attach debugger : F5 → 'C++: Attach to Node (Pick Process)'"
  echo ""
  echo "  If attach fails with 'Operation not permitted', run:"
  echo "    echo 0 | sudo tee /proc/sys/kernel/yama/ptrace_scope"
  echo ""
  echo "  See: docs/VSCODE_DEBUGGING.md"
fi

success "Build complete [mode: $MODE, type: $BUILD_TYPE]"
