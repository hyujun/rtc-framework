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
# IRQ를 non-RT 코어(Core 0-1)에만 할당한다.
# 코어가 4개 미만이면 RT 보호가 의미 없으므로 경고한다.
TOTAL_CORES=$(nproc)
if [[ "$TOTAL_CORES" -lt 4 ]]; then
  warn "CPU core count is ${TOTAL_CORES} (< 4). RT core isolation may not be effective."
  warn "At least 4 cores recommended (Core 0-1: IRQ/OS, Core 2+: RT)."
fi

# 0x3 = 0b0011 = Core 0 + Core 1 only
AFFINITY_MASK="3"
AFFINITY_HEX="0x3"

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

info "Restricting IRQs to Core 0-1 (affinity mask: ${AFFINITY_HEX})"
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
      info "  IRQ ${irq} (${NIC}) → Core 0-1"
    fi
  done
fi

# ── Pin all other IRQs to Core 0-1 ────────────────────────────────────────────
# Protects RT cores 2-5 from any hardware interrupt
info "Pinning all other IRQs to Core 0-1..."
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
info "  /proc/irq/<N>/smp_affinity should read '${AFFINITY_MASK}' (= Core 0,1)"
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
info "RT cores 2-5 are now protected from NIC and hardware interrupts."
info ""
info "To make persistent across reboots, add to /etc/rc.local or a systemd service:"
info "  sudo $(realpath "$0") ${NIC}"
