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
# 0x3 = 0b0011 = Core 0 + Core 1 only
AFFINITY_MASK="3"
AFFINITY_HEX="0x3"

# ── Detect or use specified NIC ───────────────────────────────────────────────
NIC="${1:-}"
if [[ -z "$NIC" ]]; then
  # Auto-detect: prefer the first non-loopback interface that is UP
  NIC=$(ip link show | awk -F': ' '/^[0-9]+: / && !/lo/ && /UP/ {print $2; exit}' | cut -d'@' -f1)
  if [[ -z "$NIC" ]]; then
    NIC=$(ip link show | awk -F': ' '/^[0-9]+: / && !/lo/ {print $2; exit}' | cut -d'@' -f1)
  fi
  [[ -z "$NIC" ]] && error "No network interface found. Specify one: sudo $0 <NIC>"
  warn "Auto-detected NIC: ${NIC}  (specify explicitly if wrong: sudo $0 <NIC>)"
fi

info "Restricting IRQs to Core 0-1 (affinity mask: ${AFFINITY_HEX})"
info "Target NIC: ${NIC}"
echo ""

# ── Pin NIC-specific IRQs ─────────────────────────────────────────────────────
NIC_IRQS=$(grep "${NIC}" /proc/interrupts 2>/dev/null | awk -F: '{print $1}' | tr -d ' ')

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
