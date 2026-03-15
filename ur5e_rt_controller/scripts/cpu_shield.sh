#!/bin/bash
# cpu_shield.sh — 동적 CPU 격리 관리 (모드별 tiered isolation)
#
# cset shield를 사용하여 런타임에 CPU 격리를 on/off 할 수 있다.
# isolcpus GRUB 파라미터 없이도 RT 코어를 동적으로 보호한다.
#
# Usage:
#   sudo cpu_shield.sh on [--robot|--sim]  # 격리 활성화
#   sudo cpu_shield.sh off                 # 격리 해제
#   cpu_shield.sh status                   # 상태 확인 (sudo 불필요)
#
# Tier 모델:
#   Tier 1 (RT-critical): rt_control + sensor_io  — 항상 격리
#   Tier 2 (RT-support):  udp_recv + logging + aux — 로봇 모드에서만 격리
#   Tier 3 (Flexible):    sim/monitoring/build     — 격리하지 않음
#
# Modes:
#   --robot (기본): Tier 1 + Tier 2 격리
#   --sim:          Tier 1만 격리 (Tier 2는 해제, MuJoCo 성능 확보)
#
# 실행 시점:
#   - 로봇 런치: ur_control.launch.py에서 자동 호출
#   - 시뮬 런치: mujoco_sim.launch.py에서 자동 호출
#   - 빌드 전:   build.sh / install.sh에서 자동 해제

set -euo pipefail

# ── Colors ────────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

info()    { echo -e "${CYAN}[SHIELD]${NC} $*"; }
success() { echo -e "${GREEN}[SHIELD]${NC} $*"; }
warn()    { echo -e "${YELLOW}[SHIELD]${NC} $*"; }
error()   { echo -e "${RED}[SHIELD]${NC} $*" >&2; }

# ── Helper: physical CPU core count (SMT/HT 제외) ─────────────────────────
get_physical_cores() {
  if command -v lscpu &>/dev/null; then
    local count
    count=$(lscpu -p=Core,Socket 2>/dev/null | grep -v '^#' | sort -u | wc -l)
    if [[ "$count" -gt 0 ]]; then
      echo "$count"
      return
    fi
  fi
  if [[ -f /sys/devices/system/cpu/cpu0/topology/core_id ]]; then
    local seen=""
    local count=0
    for cpu_dir in /sys/devices/system/cpu/cpu[0-9]*/; do
      local pkg_file="${cpu_dir}topology/physical_package_id"
      local core_file="${cpu_dir}topology/core_id"
      [[ -f "$pkg_file" && -f "$core_file" ]] || continue
      local key
      key="$(cat "$pkg_file"):$(cat "$core_file")"
      if [[ ! " $seen " == *" $key "* ]]; then
        seen="$seen $key"
        ((count++))
      fi
    done
    if [[ "$count" -gt 0 ]]; then
      echo "$count"
      return
    fi
  fi
  nproc --all
}

# ── Compute shield cores based on mode and core count ─────────────────────
compute_shield_cores() {
  local mode="$1"
  local phys_cores="$2"

  if [[ "$mode" == "sim" ]]; then
    # Tier 1만: rt_control + sensor_io
    if [[ "$phys_cores" -le 4 ]]; then
      echo "1-2"
    elif [[ "$phys_cores" -ge 16 ]]; then
      echo "4-5"
    else
      echo "2-3"
    fi
  else
    # robot: Tier 1 + Tier 2
    if [[ "$phys_cores" -le 4 ]]; then
      echo "1-3"
    elif [[ "$phys_cores" -le 7 ]]; then
      echo "2-5"
    elif [[ "$phys_cores" -le 9 ]]; then
      echo "2-6"
    elif [[ "$phys_cores" -le 15 ]]; then
      echo "2-6"
    else
      echo "4-8"
    fi
  fi
}

# ── Status mode marker file ──────────────────────────────────────────────────
SHIELD_MODE_FILE="/tmp/cpu_shield_mode"

# ── Command: on ──────────────────────────────────────────────────────────────
do_on() {
  local mode="${1:-robot}"

  if [[ "$EUID" -ne 0 ]]; then
    error "Root privileges required. Run: sudo $0 on --${mode}"
    exit 1
  fi

  local phys_cores
  phys_cores=$(get_physical_cores)
  local shield_cores
  shield_cores=$(compute_shield_cores "$mode" "$phys_cores")

  # Check if already shielded with same cores
  if command -v cset &>/dev/null; then
    local current_shield
    current_shield=$(cset shield -s 2>/dev/null | grep "^system" | grep -oP 'cpus: \K[^ ]+' || true)
    if [[ -n "$current_shield" ]]; then
      # Shield already active — check if same mode
      if [[ -f "$SHIELD_MODE_FILE" ]]; then
        local current_mode
        current_mode=$(cat "$SHIELD_MODE_FILE" 2>/dev/null || echo "")
        if [[ "$current_mode" == "$mode" ]]; then
          info "Shield already active (mode: ${mode}, cores: ${shield_cores})"
          return 0
        fi
      fi
      # Different mode — reset and re-apply
      info "Resetting existing shield for mode change..."
      cset shield --reset 2>/dev/null || true
    fi
  fi

  info "Activating CPU shield (mode: ${mode}, cores: ${shield_cores}, ${phys_cores}-core system)"

  # Try cset shield first
  if command -v cset &>/dev/null; then
    if cset shield --cpu="${shield_cores}" --kthread=on 2>/dev/null; then
      echo "$mode" > "$SHIELD_MODE_FILE"
      success "cset shield activated: Core ${shield_cores} isolated"
      success "Mode: ${mode} (${phys_cores}-core system)"
      return 0
    else
      warn "cset shield failed — trying cpuset fallback"
    fi
  fi

  # Fallback: /sys/fs/cgroup/cpuset direct manipulation
  local cpuset_root="/sys/fs/cgroup/cpuset"
  if [[ -d "$cpuset_root" ]]; then
    local shield_dir="${cpuset_root}/rt_shield"
    mkdir -p "$shield_dir" 2>/dev/null || true
    if [[ -d "$shield_dir" ]]; then
      echo "$shield_cores" > "${shield_dir}/cpuset.cpus" 2>/dev/null
      echo 0 > "${shield_dir}/cpuset.mems" 2>/dev/null
      echo 1 > "${shield_dir}/cpuset.cpu_exclusive" 2>/dev/null || true
      echo "$mode" > "$SHIELD_MODE_FILE"
      success "cpuset fallback: Core ${shield_cores} isolated via ${shield_dir}"
      return 0
    fi
  fi

  error "No isolation mechanism available (cset not installed, cpuset not accessible)"
  error "Install: sudo apt-get install -y cpuset"
  return 1
}

# ── Command: off ─────────────────────────────────────────────────────────────
do_off() {
  if [[ "$EUID" -ne 0 ]]; then
    error "Root privileges required. Run: sudo $0 off"
    exit 1
  fi

  # Try cset shield reset
  if command -v cset &>/dev/null; then
    if cset shield --reset 2>/dev/null; then
      rm -f "$SHIELD_MODE_FILE"
      success "cset shield deactivated — all cores available"
      return 0
    fi
  fi

  # Fallback: remove cpuset rt_shield
  local shield_dir="/sys/fs/cgroup/cpuset/rt_shield"
  if [[ -d "$shield_dir" ]]; then
    # Move tasks back to root cpuset
    while IFS= read -r pid; do
      echo "$pid" > /sys/fs/cgroup/cpuset/tasks 2>/dev/null || true
    done < "${shield_dir}/tasks" 2>/dev/null || true
    rmdir "$shield_dir" 2>/dev/null || true
    rm -f "$SHIELD_MODE_FILE"
    success "cpuset fallback shield removed — all cores available"
    return 0
  fi

  # Check if isolcpus is in effect
  if grep -q "isolcpus=" /proc/cmdline 2>/dev/null; then
    warn "isolcpus GRUB parameter active — cannot disable without reboot"
    warn "Remove isolcpus from GRUB and switch to cset shield (recommended)"
    return 1
  fi

  info "No active shield found"
  rm -f "$SHIELD_MODE_FILE"
  return 0
}

# ── Command: status ──────────────────────────────────────────────────────────
do_status() {
  local phys_cores
  phys_cores=$(get_physical_cores)
  local logical_cores
  logical_cores=$(nproc --all)
  local available_cores
  available_cores=$(nproc)
  local isolated
  isolated=$(cat /sys/devices/system/cpu/isolated 2>/dev/null || echo "")

  echo ""
  info "CPU Shield Status"
  info "  Physical cores: ${phys_cores}"
  info "  Logical cores:  ${logical_cores}"
  info "  Available cores: ${available_cores}"

  if [[ -n "$isolated" ]]; then
    info "  Isolated cores: ${isolated}"

    # Detect isolation method
    if grep -q "isolcpus=" /proc/cmdline 2>/dev/null; then
      info "  Method: isolcpus (GRUB — static, reboot required to change)"
      warn "  Recommendation: switch to cset shield for dynamic isolation"
    elif command -v cset &>/dev/null && cset shield -s 2>/dev/null | grep -q "user"; then
      local shield_mode="unknown"
      if [[ -f "$SHIELD_MODE_FILE" ]]; then
        shield_mode=$(cat "$SHIELD_MODE_FILE" 2>/dev/null || echo "unknown")
      fi
      info "  Method: cset shield (dynamic)"
      info "  Mode: ${shield_mode}"
    else
      info "  Method: unknown (cpuset or other)"
    fi
  else
    info "  Isolated cores: none"
    info "  All ${available_cores} cores available for general use"
  fi

  # Show expected layout for this core count
  echo ""
  if [[ "$phys_cores" -le 4 ]]; then
    info "  Expected layout (${phys_cores}-core):"
    info "    Core 0:   OS / DDS / NIC IRQ"
    info "    Core 1:   rt_control (SCHED_FIFO 90)"
    info "    Core 2:   sensor_io + udp_recv"
    info "    Core 3:   logger + aux + monitors"
  elif [[ "$phys_cores" -le 7 ]]; then
    info "  Expected layout (${phys_cores}-core):"
    info "    Core 0-1: OS / DDS / NIC IRQ"
    info "    Core 2:   rt_control (SCHED_FIFO 90)"
    info "    Core 3:   sensor_io (SCHED_FIFO 70)"
    info "    Core 4:   logger + status_mon + hand_detect"
    info "    Core 5:   udp_recv + aux"
  else
    info "  Expected layout (${phys_cores}-core):"
    info "    Core 0-1: OS / DDS / NIC IRQ"
    info "    Core 2:   rt_control (SCHED_FIFO 90)"
    info "    Core 3:   sensor_io (SCHED_FIFO 70)"
    info "    Core 4:   udp_recv (SCHED_FIFO 65)"
    info "    Core 5:   logger"
    info "    Core 6:   aux + monitors"
    info "    Core 7+:  spare / sim / monitoring"
  fi
  echo ""
}

# ── Usage ────────────────────────────────────────────────────────────────────
usage() {
  echo ""
  echo "Usage: cpu_shield.sh <command> [options]"
  echo ""
  echo "Commands:"
  echo "  on [--robot|--sim]  Activate CPU isolation (default: --robot)"
  echo "    --robot           Tier 1 + Tier 2 isolation (full RT protection)"
  echo "    --sim             Tier 1 only (lightweight, MuJoCo-friendly)"
  echo "  off                 Deactivate CPU isolation"
  echo "  status              Show current isolation status"
  echo ""
  echo "Examples:"
  echo "  sudo cpu_shield.sh on --robot    # Full RT isolation"
  echo "  sudo cpu_shield.sh on --sim      # Lightweight simulation isolation"
  echo "  sudo cpu_shield.sh off           # Release all isolation"
  echo "  cpu_shield.sh status             # Check current state"
  echo ""
  exit 1
}

# ── Main ─────────────────────────────────────────────────────────────────────
COMMAND="${1:-}"
shift || true

case "$COMMAND" in
  on)
    MODE="robot"
    while [[ $# -gt 0 ]]; do
      case "$1" in
        --robot) MODE="robot"; shift ;;
        --sim)   MODE="sim"; shift ;;
        *)       error "Unknown option: $1"; usage ;;
      esac
    done
    do_on "$MODE"
    ;;
  off)
    do_off
    ;;
  status)
    do_status
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    error "Unknown command: ${COMMAND:-<none>}"
    usage
    ;;
esac
