#!/bin/bash
# build.sh — RTC (Real-Time Controller) Build Script
#
# All C++ packages require C++20 (CMAKE_CXX_STANDARD 20) and strict compiler
# warnings (-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion),
# configured individually in each package's CMakeLists.txt.
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

# ── 공통 유틸리티 라이브러리 ──────────────────────────────────────────────
_SCRIPT_DIR_BUILD="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_RT_COMMON="${_SCRIPT_DIR_BUILD}/rtc_scripts/scripts/lib/rt_common.sh"
if [[ -f "$_RT_COMMON" ]]; then
  source "$_RT_COMMON"
fi
make_logger "BUILD" emoji

# ── 격리 환경 자동 활성화 (rtc_scripts/README.md 배포 가이드) ─────────────
# setup_env.sh 가 아직 source 되지 않았으면 자동 source — RTC_DEPS_PREFIX 가
# 지정되어 있는지로 판정. (deps/install 의 fmt/mimalloc/aligator 경로 + venv + ROS)
_SETUP_ENV="${_SCRIPT_DIR_BUILD}/rtc_scripts/scripts/setup_env.sh"
if [[ -z "${RTC_DEPS_PREFIX:-}" && -f "$_SETUP_ENV" ]]; then
  # shellcheck source=/dev/null
  source "$_SETUP_ENV"
fi

# ── Mode & argument parsing ────────────────────────────────────────────────────
NO_SYMLINK=0
EXPORT_COMPILE_COMMANDS=0
SHOW_BANNER=1

# Default MuJoCo search path
MJ_DEFAULT="/opt/mujoco-3.2.4"

show_help() {
  echo ""
  echo -e "${BOLD}RTC (Real-Time Controller) — build.sh${NC}"
  echo ""
  echo "Usage: $0 [MODE] [OPTIONS]"
  echo ""
  echo "Modes:"
  echo "  robot   Build packages for real robot (no MuJoCo)"
  echo "            Packages: rtc_msgs, rtc_base, rtc_communication, rtc_controller_interface,"
  echo "                      rtc_urdf_bridge, rtc_tsid, rtc_controllers, rtc_controller_manager,"
  echo "                      rtc_inference, rtc_scripts, rtc_tools,"
  echo "                      shape_estimation_msgs, shape_estimation, ur5e_description,"
  echo "                      ur5e_hand_driver, ur5e_bringup,"
  echo "                      ur5e_bt_coordinator"
  echo ""
  echo "  sim     Build for simulation (MuJoCo required, hand uses fake response)"
  echo "            Packages: all robot packages + rtc_mujoco_sim"
  echo ""
  echo "  full    Build all packages (default)"
  echo "            Packages: all of the above + rtc_digital_twin"
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
  echo "  --no-banner                Suppress the build banner (used by install.sh)"
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

# 공통 옵션 파싱 (rt_common.sh parse_common_args)
parse_common_args "$@"
MODE="$_COMMON_MODE"
BUILD_TYPE="$_COMMON_BUILD_TYPE"
CLEAN_BUILD="$_COMMON_CLEAN_BUILD"
PARALLEL_JOBS="$_COMMON_PARALLEL_JOBS"
MJ_DIR="$_COMMON_MJ_DIR"
CUSTOM_PACKAGES=("${_COMMON_CUSTOM_PACKAGES[@]}")
set -- "${REMAINING_ARGS[@]}"

# build.sh 고유 옵션 파싱
while [[ $# -gt 0 ]]; do
  case "$1" in
    -e|--export-compile-commands)
      EXPORT_COMPILE_COMMANDS=1
      shift
      ;;
    --no-symlink)
      NO_SYMLINK=1
      shift
      ;;
    --no-banner)
      SHOW_BANNER=0
      shift
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
  warn "MuJoCo not found — rtc_mujoco_sim will be skipped"
  warn "Specify --mujoco <path> to include simulation packages"
  MJ_DIR=""
fi

# ── Banner ─────────────────────────────────────────────────────────────────────
case "$MODE" in
  robot) MODE_DESC="Real Robot  (rtc_* framework + ur5e_* robot-specific)" ;;
  sim)   MODE_DESC="Simulation  (rtc_* + rtc_mujoco_sim — hand: fake response)" ;;
  full)  MODE_DESC="Full        (all packages)" ;;
esac

if [[ "$SHOW_BANNER" -eq 1 ]]; then
  echo ""
  echo -e "${BOLD}${BLUE}╔══════════════════════════════════════════════════════╗${NC}"
  echo -e "${BOLD}${BLUE}║       RTC (Real-Time Controller) — Build Script      ║${NC}"
  echo -e "${BOLD}${BLUE}╚══════════════════════════════════════════════════════╝${NC}"
  echo ""
  echo -e "  Mode : ${CYAN}${BOLD}${MODE_DESC}${NC}"
  echo -e "  Build: ${CYAN}${BOLD}${BUILD_TYPE}${NC}"
  echo ""
fi

# ── ROS2 environment ──────────────────────────────────────────────────────────
ensure_ros2_sourced

# ── Workspace detection ────────────────────────────────────────────────────────
check_workspace_structure "$_SCRIPT_DIR_BUILD"

# ── Package selection by mode ──────────────────────────────────────────────────
if [[ ${#CUSTOM_PACKAGES[@]} -gt 0 ]]; then
  PACKAGES=("${CUSTOM_PACKAGES[@]}")
  info "Using custom package list"
else
  # ── Base packages (build order matters for colcon dependency resolution) ──
  read -r -a BASE_PACKAGES <<< "$(get_base_packages)"
  read -r -a ROBOT_PACKAGES <<< "$(get_robot_packages)"

  case "$MODE" in
    robot)
      PACKAGES=("${BASE_PACKAGES[@]}" "${ROBOT_PACKAGES[@]}")
      ;;
    sim)
      PACKAGES=("${BASE_PACKAGES[@]}" rtc_mujoco_sim "${ROBOT_PACKAGES[@]}")
      ;;
    full)
      PACKAGES=("${BASE_PACKAGES[@]}" "${ROBOT_PACKAGES[@]}")
      if [[ -n "$MJ_DIR" && -d "$MJ_DIR" ]]; then
        PACKAGES+=(rtc_mujoco_sim)
      fi
      PACKAGES+=(rtc_digital_twin)
      ;;
  esac
fi

# ── CPU shield auto-release (빌드 전 격리 해제) ────────────────────────────────
auto_release_cpu_shield "${_SCRIPT_DIR_BUILD}/rtc_scripts/scripts/cpu_shield.sh"

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
if is_venv_active; then
  SYS_PYTHON=$(get_system_python)
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

# ONNX Runtime: /opt/onnxruntime 수동 설치 경로 cmake 전파
if [[ -d "/opt/onnxruntime" ]]; then
  existing_prefix="${CMAKE_PREFIX_PATH:-}"
  CMAKE_ARGS+=("-DCMAKE_PREFIX_PATH=${existing_prefix:+${existing_prefix};}/opt/onnxruntime")
fi

COLCON_ARGS+=("--cmake-args" "${CMAKE_ARGS[@]}")

colcon build "${COLCON_ARGS[@]}" || error "Build failed!"

# 빌드 후 최신 overlay 소싱 (check_rt_setup.sh 등 후속 작업용)
source "${WORKSPACE}/install/setup.bash" || true

# ── compile_commands.json — VS Code IntelliSense 연동 ─────────────────────────
# Debug 빌드 또는 --export-compile-commands 옵션 사용 시 자동 생성됨.
# .vscode/c_cpp_properties.json 의 compileCommands 경로와 일치:
#   ${workspaceFolder}/../../build/rtc_controller_manager/compile_commands.json
if [[ "$EXPORT_COMPILE_COMMANDS" -eq 1 || "$BUILD_TYPE" == "Debug" ]]; then
  CC_SRC="$WORKSPACE/build/rtc_controller_manager/compile_commands.json"
  if [[ -f "$CC_SRC" ]]; then
    success "compile_commands.json generated: $CC_SRC"
    success "VS Code IntelliSense will use this file automatically"
  else
    warn "compile_commands.json not found at $CC_SRC — IntelliSense may be limited"
  fi
fi

# ── RT Setup Verification ─────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CHECK_SCRIPT="${SCRIPT_DIR}/rtc_scripts/scripts/check_rt_setup.sh"

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
        warn "Running the rt_controller may fail with RMW load errors due to mlockall()."
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
    warn "Running the rt_controller may fail with RMW load errors due to mlockall()."
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
  echo "  Launch debugger : F5 → 'C++: Launch rt_controller (Debug)'"
  echo "  Attach debugger : F5 → 'C++: Attach to Node (Pick Process)'"
  echo ""
  echo "  If attach fails with 'Operation not permitted', run:"
  echo "    echo 0 | sudo tee /proc/sys/kernel/yama/ptrace_scope"
  echo ""
  echo "  See: docs/VSCODE_DEBUGGING.md"
fi

success "Build complete [mode: $MODE, type: $BUILD_TYPE]"
