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
# Shield model (layout v4.1):
#   shielded = RT controller cores + MPC cores (rt_control Core 1,
#              rt_callback Core 2, mpc_main + workers from Core 3)
#   released = OS/DDS (Core 0), nrt_*, arm_driver, hand_driver,
#              sim_thread/viewer (cpu_core=-1, no pin), spare
#
# Modes:
#   --robot (default): same shield range as --sim. driver processes pin via
#                      taskset (rtc_tools.launch.thread_layout) and run on
#                      SCHED_OTHER — they do not need cset protection.
#   --sim:             identical shield. MuJoCo physics + viewer roam over the
#                      released cores under CFS (cpu_core=-1 on every tier).
#
# 실행 시점:
#   - 로봇 런치: ur_control.launch.py에서 자동 호출
#   - 시뮬 런치: mujoco_sim.launch.py에서 자동 호출
#   - 빌드 전:   build.sh / install.sh에서 자동 해제

set -euo pipefail

# ── 공통 라이브러리 로드 ────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/rt_common.sh
source "${SCRIPT_DIR}/lib/rt_common.sh"
make_logger "SHIELD"

# ── Compute shield cores based on mode and core count ─────────────────────
#
# Layout v4.1: the "user" cpuset shields RT controller threads (rt_control
# Core 1, rt_callback Core 2) plus MPC main + workers (Core 3 + 4 + 5).
# Process-level driver cores (arm_driver, hand_driver) are *not* shielded —
# the driver processes themselves run there under SCHED_OTHER, so isolating
# them from "user" tasks gains nothing and wastes a core. sim_thread /
# viewer cores are -1 (no pin) on every tier; MuJoCo roams over the released
# cores under CFS.
#
# Both modes shield the same range now (RT + MPC only). The legacy "Tier 1
# vs Tier 1+2" distinction is gone — driver/IO cores are SCHED_OTHER and do
# not need cset protection. The function signature keeps the mode argument
# for forward compatibility and so callers (`on --sim`, `on --robot`) need
# no further changes.
#
# SSoT: get_rt_cores() in rt_common.sh — the same CSV is consumed by
# setup_grub_rt.sh (via get_rt_cores_with_siblings) for nohz_full / rcu_nocbs
# and by setup_irq_affinity.sh for IRQ affinity. Range expansion comes from
# _format_cpu_range so cset gets compact notation (e.g. "1-5") regardless of
# tier.
compute_shield_cores() {
  # Args $1 (mode) and $2 (phys_cores) are accepted for forward compat with
  # legacy callers; both modes share the same shield range, and get_rt_cores
  # auto-detects the physical core count via rt_common.sh.
  local rt_csv
  rt_csv=$(get_rt_cores)
  local rt_ids
  rt_ids=$(_rt_parse_cpulist "$rt_csv")
  # shellcheck disable=SC2086  # word splitting intended
  _format_cpu_range $rt_ids
}

# ── Status mode marker file ──────────────────────────────────────────────────
SHIELD_MODE_FILE="/tmp/cpu_shield_mode"

# ── Command: on ──────────────────────────────────────────────────────────────
do_on() {
  local mode="${1:-robot}"

  if [[ "$EUID" -ne 0 ]]; then
    fatal "Root privileges required. Run: sudo $0 on --${mode}"
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

  # Disable RT throttling (sched_rt_runtime_us=-1) so RT threads can use 100% CPU
  local current_rt_runtime
  current_rt_runtime=$(sysctl -n kernel.sched_rt_runtime_us 2>/dev/null || echo "unknown")
  if [[ "$current_rt_runtime" != "-1" ]]; then
    sysctl -w kernel.sched_rt_runtime_us=-1 >/dev/null 2>&1 || true
    info "sched_rt_runtime_us: ${current_rt_runtime} → -1 (RT throttling disabled)"
  fi

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

  # Fallback: cgroup v2 통합 경로 또는 cgroup v1 cpuset
  local cpuset_root=""
  if [[ -f /sys/fs/cgroup/cgroup.controllers ]]; then
    # cgroup v2: 통합 경로
    cpuset_root="/sys/fs/cgroup"
  elif [[ -d /sys/fs/cgroup/cpuset ]]; then
    # cgroup v1: 별도 cpuset 경로
    cpuset_root="/sys/fs/cgroup/cpuset"
  fi

  if [[ -n "$cpuset_root" ]]; then
    local shield_dir="${cpuset_root}/rt_shield"
    mkdir -p "$shield_dir" 2>/dev/null || true
    if [[ -d "$shield_dir" ]]; then
      if [[ -f /sys/fs/cgroup/cgroup.controllers ]]; then
        # cgroup v2: cpuset 컨트롤러 활성화
        echo "+cpuset" > "${cpuset_root}/cgroup.subtree_control" 2>/dev/null || {
          warn "Failed to enable cpuset controller on cgroup v2"
        }
        echo "$shield_cores" > "${shield_dir}/cpuset.cpus" 2>/dev/null || {
          warn "Failed to set cpuset.cpus=${shield_cores} in cgroup v2"
        }
        echo "0" > "${shield_dir}/cpuset.mems" 2>/dev/null || true
      else
        # cgroup v1
        echo "$shield_cores" > "${shield_dir}/cpuset.cpus" 2>/dev/null
        echo 0 > "${shield_dir}/cpuset.mems" 2>/dev/null
        echo 1 > "${shield_dir}/cpuset.cpu_exclusive" 2>/dev/null || true
      fi
      echo "$mode" > "$SHIELD_MODE_FILE"
      success "cpuset fallback: Core ${shield_cores} isolated via ${shield_dir}"
      return 0
    fi
  fi

  fatal "No isolation mechanism available (cset not installed, cpuset not accessible). Install: sudo apt-get install -y cpuset"
}

# ── Command: off ─────────────────────────────────────────────────────────────
do_off() {
  if [[ "$EUID" -ne 0 ]]; then
    fatal "Root privileges required. Run: sudo $0 off"
  fi

  # Restore default RT throttling (950000µs per 1000000µs period = 95%)
  local current_rt_runtime
  current_rt_runtime=$(sysctl -n kernel.sched_rt_runtime_us 2>/dev/null || echo "unknown")
  if [[ "$current_rt_runtime" == "-1" ]]; then
    sysctl -w kernel.sched_rt_runtime_us=950000 >/dev/null 2>&1 || true
    info "sched_rt_runtime_us: -1 → 950000 (RT throttling restored)"
  fi

  # Try cset shield reset
  if command -v cset &>/dev/null; then
    if cset shield --reset 2>/dev/null; then
      rm -f "$SHIELD_MODE_FILE"
      success "cset shield deactivated — all cores available"
      return 0
    fi
  fi

  # Fallback: remove cpuset rt_shield (cgroup v1 or v2)
  local shield_dir=""
  if [[ -d /sys/fs/cgroup/rt_shield ]]; then
    shield_dir="/sys/fs/cgroup/rt_shield"
  elif [[ -d /sys/fs/cgroup/cpuset/rt_shield ]]; then
    shield_dir="/sys/fs/cgroup/cpuset/rt_shield"
  fi

  if [[ -n "$shield_dir" && -d "$shield_dir" ]]; then
    # Move tasks back to parent cgroup
    local tasks_file="${shield_dir}/cgroup.procs"
    [[ -f "$tasks_file" ]] || tasks_file="${shield_dir}/tasks"
    local parent_tasks
    parent_tasks="$(dirname "$shield_dir")/cgroup.procs"
    [[ -f "$parent_tasks" ]] || parent_tasks="$(dirname "$shield_dir")/tasks"

    if [[ -f "$tasks_file" && -f "$parent_tasks" ]]; then
      while IFS= read -r pid; do
        echo "$pid" > "$parent_tasks" 2>/dev/null || true
      done < "$tasks_file" 2>/dev/null || true
    fi
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

  # Show expected layout for this core count (SSoT: rt_common::print_thread_layout)
  echo ""
  print_thread_layout "$phys_cores"
  echo ""
}

# ── Usage ────────────────────────────────────────────────────────────────────
usage() {
  echo ""
  echo "Usage: cpu_shield.sh <command> [options]"
  echo ""
  echo "Commands:"
  echo "  on [--robot|--sim]  Activate CPU isolation (default: --robot)"
  echo "    --robot           Shield RT + MPC cores (driver cores released)"
  echo "    --sim             Same shield as --robot; sim_thread runs outside"
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
