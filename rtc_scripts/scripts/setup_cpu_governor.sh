#!/bin/bash
# setup_cpu_governor.sh — RT 환경용 CPU governor 설정 (performance 모드)
#
# CPU governor를 performance로 설정하여 최대 클럭을 유지한다.
# powersave governor는 클럭을 동적으로 낮춰 RT 제어 루프 jitter를 증가시킨다.
#
# 기능:
#   1. 모든 CPU의 현재 governor 확인 및 performance 전환
#   2. cpupower 도구 설치 (미설치 시)
#   3. 재부팅 시 자동 적용을 위한 systemd oneshot 서비스 생성
#   4. Intel P-state 드라이버 및 Turbo Boost 상태 보고
#
# 원본: setup_nvidia_rt.sh Stage 10 / install.sh setup_cpu_governor()
#
# Usage:
#   sudo ./setup_cpu_governor.sh          # governor 설정 + 서비스 등록
#   sudo ./setup_cpu_governor.sh --help   # 도움말
#
# 실행 시점: 부팅 후, ROS2 실행 전
# 검증:
#   cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
#   systemctl status cpu-governor-performance

set -euo pipefail

# ── 공통 라이브러리 로드 ────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/rt_common.sh
source "${SCRIPT_DIR}/lib/rt_common.sh"
make_logger "CPU-GOV"

# ── 도움말 ──────────────────────────────────────────────────────────────────
usage() {
  echo ""
  echo "Usage: sudo setup_cpu_governor.sh [OPTIONS]"
  echo ""
  echo "Options:"
  echo "  -h, --help    도움말 출력"
  echo ""
  echo "Description:"
  echo "  모든 CPU governor를 performance로 설정하고,"
  echo "  재부팅 시 자동 적용되도록 systemd 서비스를 등록한다."
  echo ""
  echo "  powersave governor는 클럭을 동적으로 낮춰"
  echo "  RT 제어 루프 jitter를 증가시키므로,"
  echo "  RT 환경에서는 반드시 performance를 사용해야 한다."
  echo ""
  echo "Examples:"
  echo "  sudo ./setup_cpu_governor.sh"
  echo ""
  echo "Verification:"
  echo "  cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor"
  echo "  systemctl status cpu-governor-performance"
  echo ""
  exit 0
}

# ── 인자 파싱 ───────────────────────────────────────────────────────────────
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "${1:-}" == "help" ]]; then
  usage
fi

# ── root 권한 확인 ──────────────────────────────────────────────────────────
require_root "$@"

# ── 현재 governor 상태 확인 ─────────────────────────────────────────────────
# 모든 CPU의 governor를 조회하여 performance가 아닌 것이 있는지 확인한다.
check_current_governor() {
  local change_needed=0
  local cpu_count=0
  local non_perf_count=0

  for cpu_dir in /sys/devices/system/cpu/cpu[0-9]*/; do
    local gov_file="${cpu_dir}cpufreq/scaling_governor"
    if [[ -f "$gov_file" ]]; then
      ((cpu_count++)) || true
      local current_gov
      current_gov=$(cat "$gov_file" 2>/dev/null || echo "unknown")
      if [[ "$current_gov" != "performance" ]]; then
        ((non_perf_count++)) || true
        change_needed=1
      fi
    fi
  done

  if [[ "$cpu_count" -eq 0 ]]; then
    warn "cpufreq sysfs 인터페이스를 찾을 수 없습니다"
    warn "가상 머신이거나 cpufreq 드라이버가 로드되지 않았을 수 있습니다"
    return 2
  fi

  if [[ "$change_needed" -eq 1 ]]; then
    info "${non_perf_count}/${cpu_count} CPU가 performance governor가 아닙니다"
    return 0
  else
    info "모든 CPU(${cpu_count}개)가 이미 performance governor 사용 중"
    return 1
  fi
}

# ── sysfs를 통해 모든 CPU governor를 performance로 설정 ─────────────────────
set_governor_via_sysfs() {
  local set_count=0
  local fail_count=0

  for cpu_dir in /sys/devices/system/cpu/cpu[0-9]*/; do
    local gov_file="${cpu_dir}cpufreq/scaling_governor"
    if [[ -f "$gov_file" ]]; then
      if echo "performance" > "$gov_file" 2>/dev/null; then
        ((set_count++)) || true
      else
        ((fail_count++)) || true
      fi
    fi
  done

  if [[ "$set_count" -gt 0 ]]; then
    success "CPU governor → performance (${set_count}개 CPU 적용)"
  fi
  if [[ "$fail_count" -gt 0 ]]; then
    warn "${fail_count}개 CPU governor 설정 실패"
  fi
}

# ── cpupower 설치 시도 ──────────────────────────────────────────────────────
# cpupower가 없으면 linux-tools-common, linux-tools-generic 패키지를 설치한다.
# 실패해도 sysfs fallback이 있으므로 fatal하지 않는다.
install_cpupower() {
  if command -v cpupower &>/dev/null; then
    info "cpupower 이미 설치됨"
    return 0
  fi

  info "cpupower가 설치되어 있지 않습니다 — 설치 중..."
  if apt-get install -y linux-tools-common linux-tools-generic 2>/dev/null; then
    # 현재 커널 버전에 맞는 linux-tools도 설치 시도
    apt-get install -y "linux-tools-$(uname -r)" 2>/dev/null || true
    if command -v cpupower &>/dev/null; then
      success "cpupower 설치 완료"
      return 0
    else
      warn "cpupower 설치 실패 — sysfs를 통해 직접 설정합니다"
      return 1
    fi
  else
    warn "linux-tools 패키지 설치 실패 — sysfs를 통해 직접 설정합니다"
    return 1
  fi
}

# ── systemd oneshot 서비스 생성 ─────────────────────────────────────────────
# 재부팅 시 자동으로 governor를 performance로 설정하는 서비스를 등록한다.
# cpupower가 있으면 사용하고, 없으면 sysfs 직접 설정으로 fallback한다.
create_governor_service() {
  local service_path="/etc/systemd/system/cpu-governor-performance.service"

  # 서비스 파일 내용: cpupower 우선, sysfs fallback
  local service_content
  read -r -d '' service_content <<'UNIT' || true
[Unit]
Description=Set CPU governor to performance for RT kernel
After=sysinit.target systemd-modules-load.service
ConditionPathIsDirectory=/sys/devices/system/cpu/cpu0/cpufreq

[Service]
Type=oneshot
# cpupower가 있으면 사용, 없으면 sysfs 직접 설정
ExecStart=/bin/bash -c 'if [ -x /usr/bin/cpupower ]; then /usr/bin/cpupower frequency-set -g performance; else for f in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do echo performance > "$f" 2>/dev/null || true; done; fi'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
UNIT

  # 멱등 파일 쓰기: 내용이 동일하면 건너뜀
  if write_file_if_changed "$service_path" "$service_content"; then
    success "CPU governor 서비스 생성: ${service_path}"
    return 0
  else
    info "CPU governor 서비스가 이미 동일한 설정으로 존재합니다 — 건너뜀"
    return 1
  fi
}

# ── Intel P-state 및 Turbo Boost 상태 확인 ──────────────────────────────────
# Intel P-state 드라이버가 로드된 시스템에서 Turbo Boost 상태를 보고한다.
# Turbo Boost는 RT 안정성에 영향을 줄 수 있으므로 비활성화를 권장한다.
check_intel_pstate() {
  if [[ ! -f /sys/devices/system/cpu/intel_pstate/no_turbo ]]; then
    return 0
  fi

  info "Intel P-state 드라이버 감지"

  local turbo_state
  turbo_state=$(cat /sys/devices/system/cpu/intel_pstate/no_turbo 2>/dev/null || echo "unknown")

  if [[ "$turbo_state" == "0" ]]; then
    info "  Turbo Boost: 활성 (RT jitter 증가 가능, 비활성화 권장)"
    info "  비활성화: echo 1 > /sys/devices/system/cpu/intel_pstate/no_turbo"
  elif [[ "$turbo_state" == "1" ]]; then
    success "  Turbo Boost: 비활성 (RT 안정성 최적)"
  else
    warn "  Turbo Boost 상태 확인 불가"
  fi
}

# ── Main ─────────────────────────────────────────────────────────────────────
section "CPU Governor 설정 시작 (Performance 모드)"
echo ""

# 1단계: 현재 governor 확인 및 변경
check_result=0
check_current_governor || check_result=$?

if [[ "$check_result" -eq 0 ]]; then
  # performance가 아닌 CPU가 있음 → 즉시 설정
  set_governor_via_sysfs
elif [[ "$check_result" -eq 2 ]]; then
  # cpufreq sysfs 없음 → 경고만 출력하고 계속 진행
  :
fi
# check_result=1이면 이미 모든 CPU가 performance → 건너뜀

echo ""

# 2단계: cpupower 설치 시도
install_cpupower || true

echo ""

# 3단계: systemd 서비스 생성 (재부팅 영속화)
service_changed=0
create_governor_service && service_changed=1 || true

# daemon-reload + enable (서비스가 새로 생성되었거나 변경된 경우)
if [[ "$service_changed" -eq 1 ]]; then
  systemctl daemon-reload 2>/dev/null || warn "systemctl daemon-reload 실패"
fi
systemctl enable cpu-governor-performance.service 2>/dev/null || warn "서비스 enable 실패"

echo ""

# 4단계: Intel P-state / Turbo Boost 확인
check_intel_pstate

echo ""
success "CPU governor 설정 완료"
info "검증: cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor"
info "서비스: systemctl status cpu-governor-performance"
