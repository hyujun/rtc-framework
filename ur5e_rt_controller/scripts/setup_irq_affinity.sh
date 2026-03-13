#!/bin/bash
# setup_irq_affinity.sh — [방안 D] NIC IRQ 친화성 설정
#
# RT 코어(Core 2-5)를 NIC 인터럽트로부터 보호.
# 이더넷 NIC의 IRQ를 Core 0-1로 제한하여 ControlLoop() jitter를 줄인다.
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

# ── Colors ────────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

info()  { echo -e "${GREEN}[IRQ]${NC} $*"; }
warn()  { echo -e "${YELLOW}[IRQ]${NC} $*"; }
error() { echo -e "${RED}[IRQ]${NC} $*" >&2; exit 1; }

# ── Root check ────────────────────────────────────────────────────────────────
if [[ "$EUID" -ne 0 ]]; then
  error "Root privileges required. Run: sudo $0 $*"
fi

# ── CPU affinity mask ─────────────────────────────────────────────────────────
# IRQ를 non-RT 코어에만 할당한다.
# thread_config.hpp의 코어 레이아웃과 일치시킨다:
#   4코어: Core 0 = OS (isolcpus=1-3) → IRQ mask 0x1
#   6코어: Core 0-1 = OS (isolcpus=2-5) → IRQ mask 0x3
#   8코어+: Core 0-1 = OS (isolcpus=2-N) → IRQ mask 0x3
TOTAL_CORES=$(nproc)
if [[ "$TOTAL_CORES" -lt 4 ]]; then
  warn "CPU core count is ${TOTAL_CORES} (< 4). RT core isolation may not be effective."
  warn "At least 4 cores recommended."
fi

if [[ "$TOTAL_CORES" -le 4 ]]; then
  # 4코어: Core 0만 OS, Core 1은 RT Control 전용
  AFFINITY_MASK="1"
  AFFINITY_HEX="0x1"
  OS_CORES="0"
  RT_CORES="1-$((TOTAL_CORES - 1))"
else
  # 6코어+: Core 0-1이 OS
  AFFINITY_MASK="3"
  AFFINITY_HEX="0x3"
  OS_CORES="0-1"
  RT_CORES="2-$((TOTAL_CORES - 1))"
fi

# ── Helper: detect physical NIC ──────────────────────────────────────────────
# /sys/class/net/<iface>/device 심볼릭 링크는 물리 NIC에만 존재한다.
# 가상 인터페이스(docker0, veth*, br-* 등)를 자동으로 제외한다.
detect_physical_nic() {
  local iface
  for iface in /sys/class/net/*/; do
    iface=$(basename "$iface")
    [[ "$iface" == "lo" ]] && continue
    # 물리 디바이스 확인 (PCI/USB 등)
    [[ -e "/sys/class/net/${iface}/device" ]] || continue
    # UP 상태 우선
    if ip link show "$iface" 2>/dev/null | grep -q 'state UP'; then
      echo "$iface"
      return
    fi
  done
  # UP 상태 NIC가 없으면 물리 NIC 중 첫 번째 반환
  for iface in /sys/class/net/*/; do
    iface=$(basename "$iface")
    [[ "$iface" == "lo" ]] && continue
    [[ -e "/sys/class/net/${iface}/device" ]] || continue
    echo "$iface"
    return
  done
}

# ── Detect or use specified NIC ───────────────────────────────────────────────
NIC="${1:-}"
if [[ -z "$NIC" ]]; then
  NIC=$(detect_physical_nic)
  [[ -z "$NIC" ]] && error "No physical network interface found. Specify one: sudo $0 <NIC>"
  warn "Auto-detected physical NIC: ${NIC}  (specify explicitly if wrong: sudo $0 <NIC>)"
fi

info "Restricting IRQs to Core ${OS_CORES} (affinity mask: ${AFFINITY_HEX})"
info "Target NIC: ${NIC}"
echo ""

# ── Pin NIC-specific IRQs ─────────────────────────────────────────────────────
NIC_IRQS=$(grep -w "${NIC}" /proc/interrupts 2>/dev/null | awk -F: '{print $1}' | tr -d ' ')

if [[ -z "$NIC_IRQS" ]]; then
  warn "No IRQ entries found for '${NIC}' in /proc/interrupts"
  warn "Verify NIC name with: ip link show"
else
  for irq in $NIC_IRQS; do
    if [[ -f "/proc/irq/${irq}/smp_affinity" ]]; then
      echo "$AFFINITY_MASK" > "/proc/irq/${irq}/smp_affinity"
      info "  IRQ ${irq} (${NIC}) → Core ${OS_CORES}"
    fi
  done
fi

# ── Pin all other IRQs to OS cores ────────────────────────────────────────────
# Protects RT cores from any hardware interrupt
info "Pinning all other IRQs to Core ${OS_CORES}..."
PINNED=0
SKIPPED=0
for irq_dir in /proc/irq/*/; do
  irq=$(basename "$irq_dir")
  [[ "$irq" == "0" ]] && continue   # Timer IRQ — do not touch
  smp_file="${irq_dir}smp_affinity"
  if [[ -f "$smp_file" ]]; then
    if echo "$AFFINITY_MASK" > "$smp_file" 2>/dev/null; then
      ((PINNED++)) || true
    else
      ((SKIPPED++)) || true
    fi
  fi
done
info "  Pinned: ${PINNED} IRQs  |  Skipped (read-only): ${SKIPPED} IRQs"

# ── Verify ────────────────────────────────────────────────────────────────────
echo ""
info "Verification:"
info "  /proc/irq/<N>/smp_affinity should read '${AFFINITY_MASK}' (= Core ${OS_CORES})"
if [[ -n "$NIC_IRQS" ]]; then
  for irq in $NIC_IRQS; do
    if [[ -f "/proc/irq/${irq}/smp_affinity" ]]; then
      VAL=$(cat "/proc/irq/${irq}/smp_affinity")
      info "  IRQ ${irq} smp_affinity = ${VAL}  $([ "$VAL" = "$AFFINITY_MASK" ] && echo '✔' || echo '✘ (unexpected)')"
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
  info "  Core 3:   logger       (SCHED_OTHER nice -5) + aux + status_mon + hand_detect"
elif [[ "$TOTAL_CORES" -le 7 ]]; then
  info "Thread layout (${TOTAL_CORES}-core):"
  info "  Core 0-1: OS / DDS / NIC IRQ"
  info "  Core 2:   rt_control   (SCHED_FIFO 90, 500Hz control + 50Hz E-STOP)"
  info "  Core 3:   sensor_io    (SCHED_FIFO 70, joint_state/target/hand callbacks)"
  info "  Core 4:   logger       (SCHED_OTHER nice -5, 100Hz CSV drain)"
  info "  Core 4:   status_mon   (SCHED_OTHER nice -2, 10Hz status monitor)"
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
  info "  Core 6:   aux          (SCHED_OTHER nice 0) + status_mon + hand_detect"
  info "  Core 7:   spare        (monitoring, cyclictest measurement)"
fi

info ""
info "To make persistent across reboots, add to /etc/rc.local or a systemd service:"
info "  sudo $(realpath "$0") ${NIC}"
