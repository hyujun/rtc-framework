#!/bin/bash
# build.sh — UR5e RT Controller Build Script
#
# Usage:
#   ./build.sh                              # full build (robot + sim)
#   ./build.sh sim                          # simulation only (MuJoCo required)
#   ./build.sh robot                        # real robot only (no MuJoCo)
#   ./build.sh full                         # explicit full build
#   ./build.sh sim --mujoco /opt/mujoco-3.2.4  # sim with custom MuJoCo path
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

# ── Mode & argument parsing ────────────────────────────────────────────────────
MODE="full"
MJ_DIR=""

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
  echo "            Packages: ur5e_rt_base, ur5e_rt_controller,"
  echo "                      ur5e_hand_udp, ur5e_tools"
  echo ""
  echo "  sim     Build packages for simulation (requires MuJoCo)"
  echo "            Packages: ur5e_rt_base, ur5e_rt_controller,"
  echo "                      ur5e_mujoco_sim, ur5e_tools"
  echo ""
  echo "  full    Build all packages (default)"
  echo "            Packages: all of the above"
  echo ""
  echo "Options:"
  echo "  --mujoco <path>   Path to MuJoCo install dir (e.g. /opt/mujoco-3.2.4)"
  echo "                    Auto-detected from $MJ_DEFAULT if not specified"
  echo "  --help            Show this help"
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
  robot) MODE_DESC="Real Robot  (ur5e_rt_base, ur5e_rt_controller, ur5e_hand_udp, ur5e_tools)" ;;
  sim)   MODE_DESC="Simulation  (ur5e_rt_base, ur5e_rt_controller, ur5e_mujoco_sim, ur5e_tools)" ;;
  full)  MODE_DESC="Full        (all packages)" ;;
esac

echo ""
echo -e "${BOLD}${BLUE}╔══════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}${BLUE}║         UR5e RT Controller — Build Script            ║${NC}"
echo -e "${BOLD}${BLUE}╚══════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "  Mode : ${CYAN}${BOLD}${MODE_DESC}${NC}"
echo ""

# ── Workspace detection ────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(dirname "$SCRIPT_DIR")"
DETECTED_WS="$(dirname "$SRC_DIR")"

if [[ "$(basename "$SRC_DIR")" == "src" && -d "$DETECTED_WS" ]]; then
  WORKSPACE="$DETECTED_WS"
  info "Workspace auto-detected: $WORKSPACE"
else
  WORKSPACE=~/ur_ws
  warn "Script is not under <workspace>/src/<repo>/ — using default: $WORKSPACE"
fi

[[ ! -d "$WORKSPACE" ]] && error "Workspace not found: $WORKSPACE"

# ── Package selection by mode ──────────────────────────────────────────────────
case "$MODE" in
  robot)
    PACKAGES=(ur5e_rt_base ur5e_rt_controller ur5e_hand_udp ur5e_tools)
    ;;
  sim)
    PACKAGES=(ur5e_rt_base ur5e_rt_controller ur5e_mujoco_sim ur5e_tools)
    ;;
  full)
    PACKAGES=(ur5e_rt_base ur5e_rt_controller ur5e_hand_udp ur5e_tools)
    if [[ -n "$MJ_DIR" && -d "$MJ_DIR" ]]; then
      PACKAGES+=(ur5e_mujoco_sim)
    fi
    ;;
esac

# ── Build ──────────────────────────────────────────────────────────────────────
info "Building: ${PACKAGES[*]}"
cd "$WORKSPACE"

CMAKE_ARGS=()
if [[ -n "$MJ_DIR" && -d "$MJ_DIR" ]]; then
  CMAKE_ARGS+=("-Dmujoco_DIR=${MJ_DIR}/lib/cmake/mujoco")
  info "MuJoCo cmake path: ${MJ_DIR}/lib/cmake/mujoco"
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

success "Build complete [mode: $MODE]"
