#!/bin/bash
# setup_irq_affinity.sh — NIC IRQ 친화성 설정
#
# RT 코어를 NIC 인터럽트로부터 보호.
# 이더넷 NIC 및 모든 하드웨어 IRQ를 OS 코어로 제한하여 ControlLoop() jitter를 줄인다.
#
# 멱등성: 이미 올바르게 설정된 IRQ는 건너뛴다.
#
# Usage:
#   sudo ./setup_irq_affinity.sh           # NIC 자동 감지
#   sudo ./setup_irq_affinity.sh eth0      # NIC 이름 명시
#   sudo ./setup_irq_affinity.sh enp3s0   # PCIe NIC 예시
#
# 실행 시점: 부팅 후, ROS2 실행 전
# 검증:
#   cat /proc/interrupts | grep eth    # 해당 NIC의 IRQ 번호 확인
#   cat /proc/irq/<NUM>/smp_affinity   # 출력이 "3" = Core 0,1

set -euo pipefail

# ── 공통 라이브러리 로드 ────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/rt_common.sh
source "${SCRIPT_DIR}/lib/rt_common.sh"
make_logger "IRQ"

# ── Root check ────────────────────────────────────────────────────────────────
require_root

# ── CPU affinity mask ─────────────────────────────────────────────────────────
# compute_cpu_layout()는 TOTAL_CORES, LOGICAL_CORES, IRQ_AFFINITY_MASK,
# OS_CORES_DESC, RT_CORES_START, RT_CORES_END 등을 설정한다.
compute_cpu_layout

AFFINITY_MASK="$IRQ_AFFINITY_MASK"
AFFINITY_HEX="0x${IRQ_AFFINITY_MASK}"
OS_CORES="$OS_CORES_DESC"
RT_CORES="${RT_CORES_START}-${RT_CORES_END}"

if [[ "$LOGICAL_CORES" -ne "$TOTAL_CORES" ]]; then
  info "SMT/HT detected: ${LOGICAL_CORES} logical, ${TOTAL_CORES} physical cores"
  info "Using physical core count (${TOTAL_CORES}) to match thread_config.hpp layout"
fi

if [[ "$TOTAL_CORES" -lt 4 ]]; then
  warn "Physical core count is ${TOTAL_CORES} (< 4). RT core isolation may not be effective."
  warn "At least 4 physical cores recommended."
fi

# ── Detect or use specified NIC ───────────────────────────────────────────────
NIC="${1:-}"
if [[ -z "$NIC" ]]; then
  NIC=$(detect_physical_nic)
  [[ -z "$NIC" ]] && fatal "No physical network interface found. Specify one: sudo $0 <NIC>"
  warn "Auto-detected physical NIC: ${NIC}  (specify explicitly if wrong: sudo $0 <NIC>)"
fi

info "Restricting IRQs to Core ${OS_CORES} (affinity mask: ${AFFINITY_HEX})"
info "Target NIC: ${NIC}"
echo ""

# ── Helper: 현재 IRQ의 affinity가 이미 올바른지 확인 ──────────────────────────
is_irq_already_pinned() {
  local smp_file="$1"
  local mask
  mask=$(cat "$smp_file" 2>/dev/null) || return 1
  # mask에서 콤마, 선행 0 제거 후 비교
  mask=$(echo "$mask" | tr -d ',' | sed 's/^0*//' | tr '[:upper:]' '[:lower:]')
  [[ -z "$mask" ]] && mask="0"
  [[ "$mask" == "$AFFINITY_MASK" ]]
}

# ── Pin NIC-specific IRQs ─────────────────────────────────────────────────────
NIC_IRQS=$(grep -w "${NIC}" /proc/interrupts 2>/dev/null | awk -F: '{print $1}' | tr -d ' ')

if [[ -z "$NIC_IRQS" ]]; then
  warn "No IRQ entries found for '${NIC}' in /proc/interrupts"
  warn "Verify NIC name with: ip link show"
else
  for irq in $NIC_IRQS; do
    SMP_FILE="/proc/irq/${irq}/smp_affinity"
    if [[ -f "$SMP_FILE" ]]; then
      if is_irq_already_pinned "$SMP_FILE"; then
        info "  IRQ ${irq} (${NIC}) → already on Core ${OS_CORES}"
      else
        echo "$AFFINITY_MASK" > "$SMP_FILE"
        info "  IRQ ${irq} (${NIC}) → Core ${OS_CORES}"
      fi
    fi
  done
fi

# ── Pin all other IRQs to OS cores ────────────────────────────────────────────
# Protects RT cores from any hardware interrupt
info "Pinning all other IRQs to Core ${OS_CORES}..."
PINNED=0
SKIPPED=0
ALREADY_OK=0
for irq_dir in /proc/irq/*/; do
  irq=$(basename "$irq_dir")
  [[ "$irq" == "0" ]] && continue   # Timer IRQ — do not touch
  smp_file="${irq_dir}smp_affinity"
  if [[ -f "$smp_file" ]]; then
    if is_irq_already_pinned "$smp_file"; then
      ((ALREADY_OK++)) || true
    elif echo "$AFFINITY_MASK" > "$smp_file" 2>/dev/null; then
      ((PINNED++)) || true
    else
      ((SKIPPED++)) || true
    fi
  fi
done
info "  Pinned: ${PINNED} IRQs  |  Already OK: ${ALREADY_OK}  |  Skipped (read-only): ${SKIPPED} IRQs"

# ── Post-apply verification: RT 코어에 잔여 IRQ 확인 ──────────────────────────
RT_RESIDUAL=0
for irq_dir in /proc/irq/*/; do
  irq=$(basename "$irq_dir")
  [[ "$irq" == "0" || "$irq" == "default_smp_affinity" ]] && continue
  smp_file="${irq_dir}smp_affinity"
  [[ -f "$smp_file" ]] || continue
  local_mask=$(cat "$smp_file" 2>/dev/null) || continue
  local_mask=$(echo "$local_mask" | tr -d ',' | sed 's/^0*//' | tr '[:upper:]' '[:lower:]')
  [[ -z "$local_mask" ]] && local_mask="0"
  local_dec=$((16#${local_mask}))
  # RT 코어에 할당되어 있는지 확인
  for ((core=RT_CORES_START; core<=RT_CORES_END; core++)); do
    if (( local_dec & (1 << core) )); then
      ((RT_RESIDUAL++)) || true
      break
    fi
  done
done

echo ""
if [[ "$RT_RESIDUAL" -gt 0 ]]; then
  warn "Verification: ${RT_RESIDUAL} IRQs still assigned to RT cores (read-only, kernel-protected)"
  warn "These IRQs cannot be reassigned but are typically low-frequency."
else
  info "Verification: All IRQs pinned to OS cores — RT cores fully protected"
fi

# ── Verify NIC IRQs ──────────────────────────────────────────────────────────
if [[ -n "$NIC_IRQS" ]]; then
  for irq in $NIC_IRQS; do
    if [[ -f "/proc/irq/${irq}/smp_affinity" ]]; then
      VAL=$(cat "/proc/irq/${irq}/smp_affinity")
      info "  IRQ ${irq} smp_affinity = ${VAL}  $([ "$(echo "$VAL" | tr -d ',' | sed 's/^0*//')" = "$AFFINITY_MASK" ] && echo '✔' || echo '✘ (unexpected)')"
    fi
  done
fi

echo ""
info "IRQ affinity configuration complete."
info "RT cores ${RT_CORES} are now protected from NIC and hardware interrupts."
info ""

if [[ "$TOTAL_CORES" -le 4 ]]; then
  info "Thread layout (${TOTAL_CORES}-core):"
  info "  Core 0:   OS / DDS / NIC IRQ"
  info "  Core 1:   rt_control   (SCHED_FIFO 90, 500Hz control + 50Hz E-STOP)"
  info "  Core 2:   sensor_io    (SCHED_FIFO 70) + udp_recv (SCHED_FIFO 65)"
  info "  Core 3:   logger       (SCHED_OTHER nice -5) + aux + hand_detect"
elif [[ "$TOTAL_CORES" -le 7 ]]; then
  info "Thread layout (${TOTAL_CORES}-core):"
  info "  Core 0-1: OS / DDS / NIC IRQ"
  info "  Core 2:   rt_control   (SCHED_FIFO 90, 500Hz control + 50Hz E-STOP)"
  info "  Core 3:   sensor_io    (SCHED_FIFO 70, joint_state/target/hand callbacks)"
  info "  Core 4:   logger       (SCHED_OTHER nice -5, 100Hz CSV drain)"
  info "  Core 4:   hand_detect  (SCHED_OTHER nice -2, 50Hz hand failure detector)"
  info "  Core 5:   udp_recv     (SCHED_FIFO 65, hand UDP polling)"
  info "  Core 5:   aux          (SCHED_OTHER nice 0, E-STOP publisher)"
else
  info "Thread layout (${TOTAL_CORES}-core):"
  info "  Core 0-1: OS / DDS / NIC IRQ"
  info "  Core 2:   rt_control   (SCHED_FIFO 90, 500Hz control + 50Hz E-STOP)"
  info "  Core 3:   sensor_io    (SCHED_FIFO 70, joint_state/target/hand callbacks)"
  info "  Core 4:   udp_recv     (SCHED_FIFO 65, hand UDP polling)"
  info "  Core 5:   logger       (SCHED_OTHER nice -5, 100Hz CSV drain)"
  info "  Core 6:   aux          (SCHED_OTHER nice 0) + hand_detect"
  info "  Core 7:   spare        (monitoring, cyclictest measurement)"
fi

info ""
info "To make persistent across reboots, add to /etc/rc.local or a systemd service:"
info "  sudo $(realpath "$0") ${NIC}"
