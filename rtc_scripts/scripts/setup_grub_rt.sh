#!/bin/bash
# setup_grub_rt.sh — GRUB RT 커널 파라미터 통합 관리
#
# setup_nvidia_rt.sh (Stage 3)와 install.sh (setup_rt_kernel_params)에서
# 분리된 GRUB RT 파라미터의 단일 진실 공급원(single source of truth).
#
# 멱등성: 이미 설정된 파라미터는 건너뛴다.
# 백업: /etc/default/grub 수정 전 자동 백업 생성.
#
# 관리 파라미터:
#   nohz_full=<RT_CORES>       — RT 코어에서 타이머 틱 비활성화
#   rcu_nocbs=<RT_CORES>       — RT 코어에서 RCU 콜백 오프로드
#   processor.max_cstate=1     — 깊은 C-state 진입 방지 (wake-up latency 제거)
#   clocksource=tsc            — TSC 클럭소스 명시 (HPET 대비 50-100x 빠름)
#   tsc=reliable               — TSC 안정성 마킹 (fallback 방지)
#   nmi_watchdog=0             — NMI watchdog 비활성화 (100-350µs jitter 제거)
#   threadirqs                 — 하드웨어 IRQ 핸들러 스레드화
#   nosoftlockup               — RT 스레드 장시간 점유 시 soft lockup 경고 방지
#
# 추가로 sched_rt_runtime_us=-1 sysctl 설정 (RT 스레드 CPU 100% 점유 허용).
#
# Usage:
#   sudo ./setup_grub_rt.sh        # GRUB RT 파라미터 설정
#   sudo ./setup_grub_rt.sh --help # 도움말 출력
#
# 실행 시점: RT 커널 설치 후, 재부팅 전
# 검증:
#   cat /proc/cmdline              # 현재 부팅 파라미터 확인
#   sysctl kernel.sched_rt_runtime_us  # -1이면 RT 스레드 무제한 허용

set -euo pipefail

# ── 공통 라이브러리 로드 ────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/rt_common.sh
source "${SCRIPT_DIR}/lib/rt_common.sh"
make_logger "GRUB-RT"

# ── --help 처리 ──────────────────────────────────────────────────────────────
if [[ "${1:-}" == "--help" ]]; then
  echo "Usage: sudo $0"
  echo ""
  echo "Configures GRUB kernel parameters and sysctl settings for real-time operation."
  echo ""
  echo "GRUB parameters added to GRUB_CMDLINE_LINUX_DEFAULT:"
  echo "  nohz_full=<RT_CORES>       Disable timer ticks on RT cores"
  echo "  rcu_nocbs=<RT_CORES>       Offload RCU callbacks from RT cores"
  echo "  processor.max_cstate=1     Prevent deep C-state entry"
  echo "  clocksource=tsc            Use TSC clock source (fastest)"
  echo "  tsc=reliable               Mark TSC as reliable"
  echo "  nmi_watchdog=0             Disable NMI watchdog"
  echo "  threadirqs                 Thread hardware IRQ handlers"
  echo "  nosoftlockup               Suppress soft lockup warnings"
  echo ""
  echo "Sysctl settings (immediate + persistent):"
  echo "  kernel.sched_rt_runtime_us=-1    Allow RT threads 100% CPU"
  echo ""
  echo "Options:"
  echo "  --help    Show this help message"
  exit 0
fi

# ── Root check ────────────────────────────────────────────────────────────────
require_root "${1:-}"

# ── CPU 레이아웃 계산 ─────────────────────────────────────────────────────────
# compute_cpu_layout()은 TOTAL_CORES, LOGICAL_CORES, HAS_SMT,
# RT_CORES_START, RT_CORES_END 등을 설정한다.
compute_cpu_layout

# compute_expected_isolated()로 SMT 시블링을 포함한 정확한 RT 코어 범위를 얻는다.
RT_CORES=$(compute_expected_isolated)

if [[ "$LOGICAL_CORES" -ne "$TOTAL_CORES" ]]; then
  info "SMT/HT detected: ${LOGICAL_CORES} logical, ${TOTAL_CORES} physical cores"
  info "RT cores (including HT siblings): ${RT_CORES}"
else
  info "CPU layout: ${TOTAL_CORES} cores, RT cores: ${RT_CORES}"
fi

echo ""

# ══════════════════════════════════════════════════════════════════════════════
# Step 1: GRUB 커널 파라미터 설정
# ══════════════════════════════════════════════════════════════════════════════
section "Step 1: GRUB kernel parameters..."

GRUB_FILE="/etc/default/grub"

if [[ ! -f "$GRUB_FILE" ]]; then
  fatal "${GRUB_FILE} not found. Is GRUB installed?"
fi

# ── 추가할 파라미터 목록 ──────────────────────────────────────────────────────
# 값이 있는 파라미터와 없는 파라미터를 구분한다.
# NOTE: isolcpus는 cset shield로 대체되어 포함하지 않음.
# cset shield는 런타임에 동적 CPU 격리 on/off가 가능하여
# 빌드 시 전체 코어를 사용할 수 있다. (cpu_shield.sh 참조)
# nohz_full과 rcu_nocbs는 isolcpus와 독립적으로 유효하므로 유지.
#
# Phase 5 note: RT_CORES comes from compute_expected_isolated which returns
# every non-OS core. MPC cores (get_mpc_cores in rt_common.sh) are a strict
# subset of non-OS cores, so they are automatically covered without needing
# a separate merge step.
declare -A GRUB_PARAMS_WITH_VALUE=(
  ["nohz_full"]="${RT_CORES}"
  ["rcu_nocbs"]="${RT_CORES}"
  ["processor.max_cstate"]="1"
  # TSC 클럭소스 명시 지정 — HPET 대비 50-100x 빠른 타이머 읽기
  ["clocksource"]="tsc"
  ["tsc"]="reliable"
  # NMI watchdog 비활성화 — RT 코어에서 간헐적 100-350µs 지연 유발
  ["nmi_watchdog"]="0"
)

# threadirqs: 하드웨어 IRQ 핸들러를 스레드화하여 RT 스케줄링 적용
# nosoftlockup: RT 스레드가 장시간 CPU를 점유해도 soft lockup 경고 방지
GRUB_PARAMS_WITHOUT_VALUE=("threadirqs" "nosoftlockup")

# ── 기존 GRUB_CMDLINE_LINUX_DEFAULT 파싱 ─────────────────────────────────────
# recovery mode에는 영향 없도록 GRUB_CMDLINE_LINUX_DEFAULT를 사용한다.
GRUB_VAR="GRUB_CMDLINE_LINUX_DEFAULT"
CURRENT_CMDLINE=""
if grep -q "^${GRUB_VAR}=" "$GRUB_FILE"; then
  CURRENT_CMDLINE=$(grep "^${GRUB_VAR}=" "$GRUB_FILE" | sed "s/^${GRUB_VAR}=\"\(.*\)\"/\1/")
fi

GRUB_MODIFIED=0
NEW_CMDLINE="$CURRENT_CMDLINE"

# ── 값이 있는 파라미터: 이미 존재하면 건너뛰고, 없으면 추가 ──────────────────
for param in "${!GRUB_PARAMS_WITH_VALUE[@]}"; do
  value="${GRUB_PARAMS_WITH_VALUE[$param]}"
  # 파라미터 이름이 이미 존재하는지 확인 (값이 다를 수 있으므로 이름만 체크)
  if ! echo "$NEW_CMDLINE" | grep -qE "(^| )${param}="; then
    NEW_CMDLINE="${NEW_CMDLINE:+${NEW_CMDLINE} }${param}=${value}"
    GRUB_MODIFIED=1
    info "  Adding: ${param}=${value}"
  else
    info "  Already present: ${param} — skipped"
  fi
done

# ── 값이 없는 파라미터 ─────────────────────────────────────────────────────────
for param in "${GRUB_PARAMS_WITHOUT_VALUE[@]}"; do
  if ! echo "$NEW_CMDLINE" | grep -qE "(^| )${param}( |$)"; then
    NEW_CMDLINE="${NEW_CMDLINE:+${NEW_CMDLINE} }${param}"
    GRUB_MODIFIED=1
    info "  Adding: ${param}"
  else
    info "  Already present: ${param} — skipped"
  fi
done

# ── GRUB 파일 업데이트 ────────────────────────────────────────────────────────
if [[ "$GRUB_MODIFIED" -eq 1 ]]; then
  # 수정 전 백업 생성
  cp "$GRUB_FILE" "${GRUB_FILE}.bak.$(date +%Y%m%d_%H%M%S)"
  info "  Backup created: ${GRUB_FILE}.bak.*"

  # GRUB_CMDLINE_LINUX_DEFAULT 라인 교체 또는 추가
  if grep -q "^${GRUB_VAR}=" "$GRUB_FILE"; then
    # sed로 해당 라인 교체 (구분자 # 사용 — cmdline에 |가 있을 수 있으므로)
    ESCAPED_CMDLINE=$(printf '%s' "$NEW_CMDLINE" | sed 's/[&#\\/]/\\&/g')
    sed -i "s#^${GRUB_VAR}=.*#${GRUB_VAR}=\"${ESCAPED_CMDLINE}\"#" "$GRUB_FILE"
  else
    echo "${GRUB_VAR}=\"${NEW_CMDLINE}\"" >> "$GRUB_FILE"
  fi

  success "GRUB configuration updated"

  # update-grub 실행
  info "  Running update-grub..."
  update-grub 2>/dev/null || true
  success "update-grub complete"
else
  info "  All GRUB parameters already configured — no changes needed"
fi

echo ""

# ══════════════════════════════════════════════════════════════════════════════
# Step 2: sched_rt_runtime_us sysctl 설정
# ══════════════════════════════════════════════════════════════════════════════
# 기본값 950000 (95%) → -1 (무제한)로 변경하여
# RT 스레드가 CPU를 100% 점유할 수 있게 한다.
# 기본값에서는 RT 스레드가 5%의 시간 동안 강제 선점되어 jitter 발생.
section "Step 2: sched_rt_runtime_us sysctl..."

SYSCTL_CONF="/etc/sysctl.d/99-rt-sched.conf"
SYSCTL_CONTENT="# RT scheduler — allow RT threads 100% CPU usage
# Generated by setup_grub_rt.sh
# Default: 950000 (95%) — causes forced preemption jitter on RT threads
# Value -1 disables the RT bandwidth limiter entirely.
kernel.sched_rt_runtime_us = -1"

if write_file_if_changed "$SYSCTL_CONF" "$SYSCTL_CONTENT"; then
  info "  Config written to ${SYSCTL_CONF}"
else
  info "  ${SYSCTL_CONF} already up-to-date — skipped"
fi

# 즉시 적용 (재부팅 없이)
sysctl -w kernel.sched_rt_runtime_us=-1 > /dev/null 2>&1 || {
  warn "  Failed to apply sysctl immediately (will take effect after reboot)"
}

info "  kernel.sched_rt_runtime_us = -1 (RT threads may use 100% CPU)"

echo ""

# ══════════════════════════════════════════════════════════════════════════════
# 완료 요약
# ══════════════════════════════════════════════════════════════════════════════
info "GRUB RT parameter configuration complete."
echo ""
info "Parameters configured in ${GRUB_FILE}:"
info "  nohz_full=${RT_CORES}  rcu_nocbs=${RT_CORES}"
info "  processor.max_cstate=1  clocksource=tsc  tsc=reliable"
info "  nmi_watchdog=0  threadirqs  nosoftlockup"
info ""
info "Sysctl configured in ${SYSCTL_CONF}:"
info "  kernel.sched_rt_runtime_us=-1"
echo ""

if [[ "$GRUB_MODIFIED" -eq 1 ]]; then
  warn "REBOOT REQUIRED for GRUB parameters to take effect."
  info "After reboot, verify with: cat /proc/cmdline"
else
  info "To verify current boot params: cat /proc/cmdline"
fi

info "To verify sysctl: sysctl kernel.sched_rt_runtime_us"
