#!/bin/bash
# check_rt_setup.sh — RT 환경 설정 검증 스크립트
#
# RT 제어 루프(500Hz) 실행 전 시스템 설정 상태를 점검한다.
# sudo 불필요 (read-only). 일부 항목은 권한 없으면 SKIP 표시.
#
# Usage:
#   ./check_rt_setup.sh              # 상세 출력 (기본)
#   ./check_rt_setup.sh --summary    # 카테고리당 1줄 (build.sh용)
#   ./check_rt_setup.sh --json       # CI용 JSON 출력
#   ./check_rt_setup.sh --fix        # 실패 항목별 수정 명령 제안
#   ./check_rt_setup.sh --help
#
# 검증 카테고리 (8개):
#   1. RT Kernel        — PREEMPT_RT 활성 여부
#   2. CPU Isolation    — isolcpus 설정 확인 (thread_config.hpp 매칭)
#   3. GRUB Parameters  — isolcpus, nohz_full, rcu_nocbs, threadirqs 등
#   4. RT Permissions   — ulimit, realtime 그룹, limits.conf
#   5. IRQ Affinity     — 모든 IRQ가 OS 코어에 pinned 여부
#   6. Network/UDP      — sysctl 버퍼, NIC coalescing/offload
#   7. NVIDIA (optional)— persistence mode, IRQ affinity, modprobe
#   8. CPU Frequency    — governor=performance 여부
#
# Exit codes: 0=all pass, 1=warnings only, 2=failures exist

set -u

# ── Colors ────────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
BOLD='\033[1m'
DIM='\033[2m'
NC='\033[0m'

# ── Output mode ──────────────────────────────────────────────────────────────
OUTPUT_MODE="verbose"   # verbose | summary | json
SHOW_FIX=0

# ── Counters ─────────────────────────────────────────────────────────────────
PASS_COUNT=0
WARN_COUNT=0
FAIL_COUNT=0
SKIP_COUNT=0

# ── JSON accumulator ────────────────────────────────────────────────────────
JSON_CATEGORIES=""

# ── Script directory (for --fix suggestions) ─────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Argument parsing ────────────────────────────────────────────────────────
show_help() {
  echo ""
  echo -e "${BOLD}check_rt_setup.sh — RT 환경 설정 검증${NC}"
  echo ""
  echo "Usage: $0 [OPTIONS]"
  echo ""
  echo "Options:"
  echo "  --verbose     상세 출력 (기본)"
  echo "  --summary     카테고리당 1줄 요약"
  echo "  --json        JSON 출력 (CI용)"
  echo "  --fix         실패 항목별 수정 명령 표시"
  echo "  --help        이 도움말"
  echo ""
  echo "Exit codes:"
  echo "  0  모든 항목 PASS"
  echo "  1  경고(WARN)만 있음"
  echo "  2  실패(FAIL) 항목 존재"
  echo ""
  exit 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --verbose)  OUTPUT_MODE="verbose"; shift ;;
    --summary)  OUTPUT_MODE="summary"; shift ;;
    --json)     OUTPUT_MODE="json"; shift ;;
    --fix)      SHOW_FIX=1; shift ;;
    -h|--help)  show_help ;;
    *)          echo "Unknown option: $1"; show_help ;;
  esac
done

# ── Output helpers ───────────────────────────────────────────────────────────
_pass() {
  ((PASS_COUNT++)) || true
  if [[ "$OUTPUT_MODE" == "verbose" ]]; then
    echo -e "  ${GREEN}[PASS]${NC} $*"
  fi
}

_warn() {
  ((WARN_COUNT++)) || true
  if [[ "$OUTPUT_MODE" == "verbose" ]]; then
    echo -e "  ${YELLOW}[WARN]${NC} $*"
  fi
}

_fail() {
  ((FAIL_COUNT++)) || true
  if [[ "$OUTPUT_MODE" == "verbose" ]]; then
    echo -e "  ${RED}[FAIL]${NC} $*"
  fi
}

_skip() {
  ((SKIP_COUNT++)) || true
  if [[ "$OUTPUT_MODE" == "verbose" ]]; then
    echo -e "  ${DIM}[SKIP]${NC} $*"
  fi
}

_fix() {
  if [[ "$SHOW_FIX" -eq 1 && "$OUTPUT_MODE" == "verbose" ]]; then
    echo -e "         ${CYAN}fix:${NC} $*"
  fi
}

_section() {
  if [[ "$OUTPUT_MODE" == "verbose" ]]; then
    echo ""
    echo -e "${BOLD}[$1] $2${NC}"
  fi
}

# ── Summary helpers (카테고리 결과 추적) ──────────────────────────────────────
# 각 카테고리의 최악 상태를 저장
declare -A CATEGORY_STATUS
declare -A CATEGORY_DETAIL

_category_start() {
  CATEGORY_STATUS["$1"]="PASS"
  CATEGORY_DETAIL["$1"]=""
}

_category_update() {
  local cat="$1" status="$2"
  local current="${CATEGORY_STATUS[$cat]}"
  # FAIL > WARN > PASS
  if [[ "$status" == "FAIL" ]]; then
    CATEGORY_STATUS["$cat"]="FAIL"
  elif [[ "$status" == "WARN" && "$current" != "FAIL" ]]; then
    CATEGORY_STATUS["$cat"]="WARN"
  fi
}

_category_set_detail() {
  CATEGORY_DETAIL["$1"]="$2"
}

# ── Helper: physical CPU core count (SMT/HT 제외) ────────────────────────────
# nproc은 논리 코어(HT 포함)를 반환하므로, 물리 코어만 카운트한다.
# 예: i7-8700 (6C/12T) → nproc=12 이지만 이 함수는 6을 반환.
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
  nproc
}

# ── CPU layout auto-detection ────────────────────────────────────────────────
# thread_config.hpp와 동일한 로직 (물리 코어 기준)
LOGICAL_CORES=$(nproc)
TOTAL_CORES=$(get_physical_cores)

if [[ "$TOTAL_CORES" -le 4 ]]; then
  EXPECTED_ISOLATED="1-$((TOTAL_CORES - 1))"
  EXPECTED_IRQ_MASK="1"      # 0x1 = Core 0 only
  OS_CORES_DESC="0"
  RT_CORES_START=1
else
  EXPECTED_ISOLATED="2-$((TOTAL_CORES - 1))"
  EXPECTED_IRQ_MASK="3"      # 0x3 = Core 0-1
  OS_CORES_DESC="0-1"
  RT_CORES_START=2
fi
RT_CORES_END=$((TOTAL_CORES - 1))

# ══════════════════════════════════════════════════════════════════════════════
# [1/8] RT Kernel
# ══════════════════════════════════════════════════════════════════════════════
check_rt_kernel() {
  _section "1/8" "RT Kernel"
  _category_start "rt_kernel"

  local kernel_ver kernel_detail
  kernel_ver=$(uname -r)
  kernel_detail=$(uname -v)

  if echo "$kernel_detail" | grep -q "PREEMPT_RT"; then
    _pass "PREEMPT_RT 커널 활성: ${kernel_ver}"
    _category_set_detail "rt_kernel" "PREEMPT_RT ${kernel_ver}"
  elif echo "$kernel_ver" | grep -qi "rt\|realtime"; then
    _pass "RT 커널 감지: ${kernel_ver}"
    _category_set_detail "rt_kernel" "RT ${kernel_ver}"
  elif echo "$kernel_detail" | grep -q "PREEMPT_DYNAMIC"; then
    _warn "LowLatency 커널 (PREEMPT_DYNAMIC): ${kernel_ver}"
    _warn "PREEMPT_RT보다 RT 성능이 낮습니다"
    _fix "sudo ${SCRIPT_DIR}/build_rt_kernel.sh"
    _category_update "rt_kernel" "WARN"
    _category_set_detail "rt_kernel" "PREEMPT_DYNAMIC ${kernel_ver}"
  else
    _fail "일반 커널: ${kernel_ver} (PREEMPT_RT 아님)"
    _fix "sudo ${SCRIPT_DIR}/build_rt_kernel.sh"
    _fix "또는: sudo apt install linux-lowlatency-hwe-\$(lsb_release -rs)"
    _category_update "rt_kernel" "FAIL"
    _category_set_detail "rt_kernel" "Generic ${kernel_ver}"
  fi
}

# ══════════════════════════════════════════════════════════════════════════════
# [2/8] CPU Isolation
# ══════════════════════════════════════════════════════════════════════════════
check_cpu_isolation() {
  _section "2/8" "CPU Isolation"
  _category_start "cpu_isolation"

  local isolated_file="/sys/devices/system/cpu/isolated"
  if [[ ! -f "$isolated_file" ]]; then
    _skip "파일 없음: ${isolated_file}"
    _category_set_detail "cpu_isolation" "unknown"
    return
  fi

  local isolated
  isolated=$(cat "$isolated_file" 2>/dev/null || echo "")

  if [[ -z "$isolated" ]]; then
    _fail "격리된 CPU 없음 (isolcpus 미설정)"
    _fix "GRUB에 isolcpus=${EXPECTED_ISOLATED} 추가"
    _fix "sudo ${SCRIPT_DIR}/setup_nvidia_rt.sh"
    _category_update "cpu_isolation" "FAIL"
    _category_set_detail "cpu_isolation" "none"
  elif [[ "$isolated" == "$EXPECTED_ISOLATED" ]]; then
    _pass "CPU 격리 일치: ${isolated} (기대값: ${EXPECTED_ISOLATED})"
    _category_set_detail "cpu_isolation" "${isolated}"
  else
    _warn "CPU 격리 불일치: ${isolated} (기대값: ${EXPECTED_ISOLATED})"
    _fix "GRUB의 isolcpus=${isolated} → isolcpus=${EXPECTED_ISOLATED} 로 변경"
    _category_update "cpu_isolation" "WARN"
    _category_set_detail "cpu_isolation" "${isolated} (expected ${EXPECTED_ISOLATED})"
  fi
}

# ══════════════════════════════════════════════════════════════════════════════
# [3/8] GRUB Parameters
# ══════════════════════════════════════════════════════════════════════════════
check_grub_params() {
  _section "3/8" "GRUB Parameters"
  _category_start "grub_params"

  local cmdline
  cmdline=$(cat /proc/cmdline 2>/dev/null || echo "")

  if [[ -z "$cmdline" ]]; then
    _skip "/proc/cmdline 읽기 실패"
    _category_set_detail "grub_params" "unknown"
    return
  fi

  local found=0 total=0

  # 필수 파라미터 (값 포함)
  local -a params_with_val=("isolcpus" "nohz_full" "rcu_nocbs" "processor.max_cstate")
  for param in "${params_with_val[@]}"; do
    ((total++)) || true
    if echo "$cmdline" | grep -qE "(^| )${param}="; then
      _pass "${param} 설정됨"
      ((found++)) || true
    else
      _fail "${param} 미설정"
      _category_update "grub_params" "FAIL"
    fi
  done

  # 선택 파라미터 (값 없음)
  local -a params_no_val=("threadirqs" "nosoftlockup")
  for param in "${params_no_val[@]}"; do
    ((total++)) || true
    if echo "$cmdline" | grep -qE "(^| )${param}( |$)"; then
      _pass "${param} 설정됨"
      ((found++)) || true
    else
      _warn "${param} 미설정 (권장)"
      _category_update "grub_params" "WARN"
    fi
  done

  _fix "sudo ${SCRIPT_DIR}/setup_nvidia_rt.sh"
  _category_set_detail "grub_params" "${found}/${total} params"
}

# ══════════════════════════════════════════════════════════════════════════════
# [4/8] RT Permissions
# ══════════════════════════════════════════════════════════════════════════════
check_rt_permissions() {
  _section "4/8" "RT Permissions"
  _category_start "rt_permissions"

  # ulimit -r (rtprio)
  local rtprio
  rtprio=$(ulimit -r 2>/dev/null || echo "0")
  if [[ "$rtprio" == "99" ]]; then
    _pass "rtprio = 99"
  elif [[ "$rtprio" -ge 90 ]]; then
    _warn "rtprio = ${rtprio} (99 권장)"
    _category_update "rt_permissions" "WARN"
  else
    _fail "rtprio = ${rtprio} (99 필요, SCHED_FIFO 90 사용 불가)"
    _fix "sudo groupadd -f realtime && sudo usermod -aG realtime \$USER"
    _fix "echo '@realtime - rtprio 99' | sudo tee -a /etc/security/limits.conf"
    _fix "로그아웃 후 재로그인 필요"
    _category_update "rt_permissions" "FAIL"
  fi

  # ulimit -l (memlock)
  local memlock
  memlock=$(ulimit -l 2>/dev/null || echo "0")
  if [[ "$memlock" == "unlimited" ]]; then
    _pass "memlock = unlimited"
  else
    _fail "memlock = ${memlock} (unlimited 필요, mlockall() 실패 위험)"
    _fix "echo '@realtime - memlock unlimited' | sudo tee -a /etc/security/limits.conf"
    _fix "로그아웃 후 재로그인 필요"
    _category_update "rt_permissions" "FAIL"
  fi

  # realtime 그룹 멤버십
  if id -nG 2>/dev/null | grep -qw "realtime"; then
    _pass "realtime 그룹 멤버"
  else
    _warn "realtime 그룹에 속하지 않음 (현재 세션에서는 OK일 수 있음)"
    _fix "sudo usermod -aG realtime \$USER && newgrp realtime"
    _category_update "rt_permissions" "WARN"
  fi

  # limits.conf 엔트리
  if [[ -f /etc/security/limits.conf ]]; then
    if grep -q "@realtime.*rtprio" /etc/security/limits.conf 2>/dev/null; then
      _pass "limits.conf: rtprio 엔트리 존재"
    else
      _warn "limits.conf: @realtime rtprio 엔트리 없음"
      _category_update "rt_permissions" "WARN"
    fi
    if grep -q "@realtime.*memlock" /etc/security/limits.conf 2>/dev/null; then
      _pass "limits.conf: memlock 엔트리 존재"
    else
      _warn "limits.conf: @realtime memlock 엔트리 없음"
      _category_update "rt_permissions" "WARN"
    fi
  fi

  _category_set_detail "rt_permissions" "rtprio=${rtprio}, memlock=${memlock}"
}

# ══════════════════════════════════════════════════════════════════════════════
# [5/8] IRQ Affinity
# ══════════════════════════════════════════════════════════════════════════════
check_irq_affinity() {
  _section "5/8" "IRQ Affinity"
  _category_start "irq_affinity"

  local rt_irq_count=0
  local os_irq_count=0
  local unreadable=0
  local total_irqs=0

  for irq_dir in /proc/irq/*/; do
    local irq
    irq=$(basename "$irq_dir")
    [[ "$irq" == "0" || "$irq" == "default_smp_affinity" ]] && continue

    local smp_file="${irq_dir}smp_affinity"
    [[ -f "$smp_file" ]] || continue

    ((total_irqs++)) || true

    local mask
    mask=$(cat "$smp_file" 2>/dev/null)
    if [[ $? -ne 0 || -z "$mask" ]]; then
      ((unreadable++)) || true
      continue
    fi

    # mask를 10진수로 변환 (선행 0 및 콤마 제거)
    mask=$(echo "$mask" | tr -d ',' | sed 's/^0*//' | tr '[:upper:]' '[:lower:]')
    [[ -z "$mask" ]] && mask="0"
    local mask_dec=$((16#${mask}))

    # RT 코어에 IRQ가 할당되어 있는지 확인
    local on_rt=0
    for ((core=RT_CORES_START; core<=RT_CORES_END; core++)); do
      if (( mask_dec & (1 << core) )); then
        on_rt=1
        break
      fi
    done

    if [[ "$on_rt" -eq 1 ]]; then
      ((rt_irq_count++)) || true
    else
      ((os_irq_count++)) || true
    fi
  done

  if [[ "$total_irqs" -eq 0 ]]; then
    _skip "IRQ 정보를 읽을 수 없음"
    _category_set_detail "irq_affinity" "unknown"
    return
  fi

  if [[ "$unreadable" -eq "$total_irqs" ]]; then
    _skip "모든 IRQ 파일 읽기 권한 없음 (sudo 없이는 확인 불가)"
    _category_set_detail "irq_affinity" "no permission"
    return
  fi

  if [[ "$rt_irq_count" -eq 0 ]]; then
    _pass "모든 IRQ가 OS 코어(Core ${OS_CORES_DESC})에 pinned (${os_irq_count}개)"
    _category_set_detail "irq_affinity" "all on OS cores (${os_irq_count} IRQs)"
  elif [[ "$rt_irq_count" -le 3 ]]; then
    _warn "RT 코어에 ${rt_irq_count}개 IRQ 존재 (OS: ${os_irq_count}개)"
    _fix "sudo ${SCRIPT_DIR}/setup_irq_affinity.sh"
    _category_update "irq_affinity" "WARN"
    _category_set_detail "irq_affinity" "${rt_irq_count} on RT cores"
  else
    _fail "RT 코어에 ${rt_irq_count}개 IRQ 존재 — 제어 루프 jitter 위험"
    _fix "sudo ${SCRIPT_DIR}/setup_irq_affinity.sh"
    _category_update "irq_affinity" "FAIL"
    _category_set_detail "irq_affinity" "${rt_irq_count} on RT cores"
  fi

  if [[ "$unreadable" -gt 0 ]]; then
    _skip "${unreadable}개 IRQ 파일 읽기 불가 (권한 부족)"
  fi
}

# ══════════════════════════════════════════════════════════════════════════════
# [6/8] Network / UDP
# ══════════════════════════════════════════════════════════════════════════════
check_network_udp() {
  _section "6/8" "Network / UDP"
  _category_start "network_udp"

  # sysctl 값 검증 (setup_udp_optimization.sh와 일치)
  local -A expected_sysctl=(
    ["net.core.rmem_max"]="2147483647"
    ["net.core.wmem_max"]="2147483647"
    ["net.core.rmem_default"]="2147483647"
    ["net.core.wmem_default"]="2147483647"
    ["net.core.netdev_max_backlog"]="5000"
  )

  local sysctl_ok=0
  local sysctl_total=0

  for key in "${!expected_sysctl[@]}"; do
    ((sysctl_total++)) || true
    local proc_path="/proc/sys/$(echo "$key" | tr '.' '/')"
    local actual expected
    expected="${expected_sysctl[$key]}"

    if [[ -f "$proc_path" ]]; then
      actual=$(cat "$proc_path" 2>/dev/null || echo "?")
      if [[ "$actual" == "$expected" ]]; then
        _pass "${key} = ${actual}"
        ((sysctl_ok++)) || true
      elif [[ "$actual" -ge "$expected" ]] 2>/dev/null; then
        # 현재 값이 기대값 이상이면 OK
        _pass "${key} = ${actual} (>= ${expected})"
        ((sysctl_ok++)) || true
      else
        _fail "${key} = ${actual} (기대값: ${expected})"
        _category_update "network_udp" "FAIL"
      fi
    else
      _skip "${key}: 파일 없음 (${proc_path})"
    fi
  done

  # 영구 설정 파일 존재 여부
  local sysctl_conf="/etc/sysctl.d/99-ros2-udp.conf"
  if [[ -f "$sysctl_conf" ]]; then
    _pass "영구 설정 파일 존재: ${sysctl_conf}"
  else
    _warn "영구 설정 파일 없음: ${sysctl_conf} (재부팅 시 sysctl 초기화)"
    _category_update "network_udp" "WARN"
  fi

  # NIC coalescing/offload (ethtool — 권한 필요할 수 있음)
  if command -v ethtool &>/dev/null; then
    # 물리 NIC 자동 감지
    local nic=""
    for iface in /sys/class/net/*/; do
      iface=$(basename "$iface")
      [[ "$iface" == "lo" ]] && continue
      [[ -e "/sys/class/net/${iface}/device" ]] || continue
      if ip link show "$iface" 2>/dev/null | grep -q 'state UP'; then
        nic="$iface"
        break
      fi
    done
    # fallback: 첫 번째 물리 NIC
    if [[ -z "$nic" ]]; then
      for iface in /sys/class/net/*/; do
        iface=$(basename "$iface")
        [[ "$iface" == "lo" ]] && continue
        [[ -e "/sys/class/net/${iface}/device" ]] || continue
        nic="$iface"
        break
      done
    fi

    if [[ -n "$nic" ]]; then
      # Coalescing 확인
      local coal_out
      coal_out=$(ethtool -c "$nic" 2>&1)
      if [[ $? -eq 0 ]]; then
        local rx_usecs
        rx_usecs=$(echo "$coal_out" | grep "rx-usecs:" | awk '{print $2}')
        if [[ "$rx_usecs" == "0" ]]; then
          _pass "NIC ${nic}: rx-usecs = 0 (coalescing off)"
        elif [[ -n "$rx_usecs" ]]; then
          _warn "NIC ${nic}: rx-usecs = ${rx_usecs} (0 권장)"
          _category_update "network_udp" "WARN"
        fi
      else
        _skip "NIC ${nic}: ethtool -c 권한 부족"
      fi

      # Offload 확인
      local offload_out
      offload_out=$(ethtool -k "$nic" 2>&1)
      if [[ $? -eq 0 ]]; then
        local gro_status
        gro_status=$(echo "$offload_out" | grep "generic-receive-offload:" | awk '{print $2}')
        if [[ "$gro_status" == "off" ]]; then
          _pass "NIC ${nic}: GRO off"
        elif [[ -n "$gro_status" ]]; then
          _warn "NIC ${nic}: GRO ${gro_status} (off 권장)"
          _category_update "network_udp" "WARN"
        fi
      else
        _skip "NIC ${nic}: ethtool -k 권한 부족"
      fi
    fi
  else
    _skip "ethtool 미설치 — NIC 설정 확인 생략"
  fi

  _fix "sudo ${SCRIPT_DIR}/setup_udp_optimization.sh"
  _category_set_detail "network_udp" "sysctl ${sysctl_ok}/${sysctl_total}"
}

# ══════════════════════════════════════════════════════════════════════════════
# [7/8] NVIDIA (optional)
# ══════════════════════════════════════════════════════════════════════════════
check_nvidia() {
  _section "7/8" "NVIDIA (optional)"
  _category_start "nvidia"

  # GPU 존재 확인
  if ! command -v lspci &>/dev/null || ! lspci 2>/dev/null | grep -qi nvidia; then
    _skip "NVIDIA GPU 미감지 — 건너뜀"
    _category_set_detail "nvidia" "no GPU"
    return
  fi

  # Persistence mode
  if command -v nvidia-smi &>/dev/null; then
    local persist
    persist=$(nvidia-smi -q -d PERFORMANCE 2>/dev/null | grep -i "persistence" | head -1 || echo "")
    if echo "$persist" | grep -qi "enabled"; then
      _pass "Persistence mode: Enabled"
    elif [[ -n "$persist" ]]; then
      _warn "Persistence mode: Disabled (RT 지연 원인 가능)"
      _category_update "nvidia" "WARN"
    else
      _skip "nvidia-smi persistence 정보 없음"
    fi
  else
    _skip "nvidia-smi 미설치"
  fi

  # modprobe 설정
  local modprobe_conf="/etc/modprobe.d/nvidia-rt.conf"
  if [[ -f "$modprobe_conf" ]]; then
    if grep -q "NVreg_EnableGpuFirmware=0" "$modprobe_conf" 2>/dev/null; then
      _pass "modprobe: NVreg_EnableGpuFirmware=0 설정됨"
    else
      _warn "modprobe: NVreg_EnableGpuFirmware=0 미설정"
      _category_update "nvidia" "WARN"
    fi
  else
    _warn "${modprobe_conf} 파일 없음"
    _category_update "nvidia" "WARN"
  fi

  # NVIDIA IRQ affinity systemd 서비스
  if systemctl is-enabled nvidia-irq-affinity.service &>/dev/null; then
    _pass "nvidia-irq-affinity.service: enabled"
  else
    _warn "nvidia-irq-affinity.service: not enabled"
    _category_update "nvidia" "WARN"
  fi

  # nouveau 블랙리스트
  if [[ -f "/etc/modprobe.d/blacklist-nouveau.conf" ]]; then
    _pass "nouveau 블랙리스트 존재"
  else
    _warn "nouveau 블랙리스트 파일 없음"
    _category_update "nvidia" "WARN"
  fi

  _fix "sudo ${SCRIPT_DIR}/setup_nvidia_rt.sh"
  _category_set_detail "nvidia" "GPU detected"
}

# ══════════════════════════════════════════════════════════════════════════════
# [8/8] CPU Frequency
# ══════════════════════════════════════════════════════════════════════════════
check_cpu_frequency() {
  _section "8/8" "CPU Frequency"
  _category_start "cpu_frequency"

  local gov_dir="/sys/devices/system/cpu/cpu0/cpufreq"
  if [[ ! -d "$gov_dir" ]]; then
    _skip "cpufreq 미지원 (고정 클럭 CPU 또는 VM)"
    _category_set_detail "cpu_frequency" "not available"
    return
  fi

  local non_perf=0
  local checked=0

  for ((core=RT_CORES_START; core<=RT_CORES_END; core++)); do
    local gov_file="/sys/devices/system/cpu/cpu${core}/cpufreq/scaling_governor"
    if [[ -f "$gov_file" ]]; then
      local gov
      gov=$(cat "$gov_file" 2>/dev/null || echo "unknown")
      ((checked++)) || true
      if [[ "$gov" != "performance" ]]; then
        ((non_perf++)) || true
        if [[ "$OUTPUT_MODE" == "verbose" ]]; then
          _warn "Core ${core}: governor = ${gov} (performance 권장)"
        fi
      fi
    fi
  done

  if [[ "$checked" -eq 0 ]]; then
    _skip "RT 코어의 cpufreq 정보 없음"
    _category_set_detail "cpu_frequency" "unknown"
    return
  fi

  if [[ "$non_perf" -eq 0 ]]; then
    _pass "RT 코어(${RT_CORES_START}-${RT_CORES_END}) 모두 performance governor"
    _category_set_detail "cpu_frequency" "all performance"
  else
    _warn "RT 코어 중 ${non_perf}개가 performance 아님"
    _fix "sudo cpupower frequency-set -g performance"
    _category_update "cpu_frequency" "WARN"
    _category_set_detail "cpu_frequency" "${non_perf} non-performance"
  fi
}

# ══════════════════════════════════════════════════════════════════════════════
# Summary output
# ══════════════════════════════════════════════════════════════════════════════
print_summary() {
  local categories=("rt_kernel" "cpu_isolation" "grub_params" "rt_permissions"
                    "irq_affinity" "network_udp" "nvidia" "cpu_frequency")
  local labels=("RT Kernel" "CPU Isolation" "GRUB Params" "RT Permissions"
                "IRQ Affinity" "Network/UDP" "NVIDIA" "CPU Frequency")

  if [[ "$OUTPUT_MODE" == "summary" ]]; then
    echo -e "${BOLD}RT Setup Check (${TOTAL_CORES}-core)${NC}"
    for i in "${!categories[@]}"; do
      local cat="${categories[$i]}"
      local label="${labels[$i]}"
      local status="${CATEGORY_STATUS[$cat]:-SKIP}"
      local detail="${CATEGORY_DETAIL[$cat]:-}"

      local color="$GREEN"
      local icon="PASS"
      case "$status" in
        WARN) color="$YELLOW"; icon="WARN" ;;
        FAIL) color="$RED"; icon="FAIL" ;;
        SKIP) color="$DIM"; icon="SKIP" ;;
      esac

      echo -e "  ${color}[${icon}]${NC} $(printf '%-16s' "$label") ${DIM}${detail}${NC}"
    done
  fi

  # 최종 요약 (verbose & summary 공통)
  if [[ "$OUTPUT_MODE" != "json" ]]; then
    echo ""
    local total=$((PASS_COUNT + WARN_COUNT + FAIL_COUNT))
    echo -ne "  ${BOLD}Result:${NC} "
    [[ "$PASS_COUNT" -gt 0 ]] && echo -ne "${GREEN}${PASS_COUNT} pass${NC}  "
    [[ "$WARN_COUNT" -gt 0 ]] && echo -ne "${YELLOW}${WARN_COUNT} warn${NC}  "
    [[ "$FAIL_COUNT" -gt 0 ]] && echo -ne "${RED}${FAIL_COUNT} fail${NC}  "
    [[ "$SKIP_COUNT" -gt 0 ]] && echo -ne "${DIM}${SKIP_COUNT} skip${NC}"
    echo ""
  fi
}

# ══════════════════════════════════════════════════════════════════════════════
# JSON output
# ══════════════════════════════════════════════════════════════════════════════
print_json() {
  local categories=("rt_kernel" "cpu_isolation" "grub_params" "rt_permissions"
                    "irq_affinity" "network_udp" "nvidia" "cpu_frequency")

  echo "{"
  echo "  \"timestamp\": \"$(date -Iseconds)\","
  echo "  \"hostname\": \"$(hostname)\","
  echo "  \"cpu_cores\": ${TOTAL_CORES},"
  echo "  \"categories\": {"

  local first=1
  for cat in "${categories[@]}"; do
    local status="${CATEGORY_STATUS[$cat]:-SKIP}"
    local detail="${CATEGORY_DETAIL[$cat]:-}"
    # JSON 특수문자 이스케이프
    detail=$(echo "$detail" | sed 's/\\/\\\\/g; s/"/\\"/g')

    [[ "$first" -eq 0 ]] && echo ","
    printf "    \"%s\": {\"status\": \"%s\", \"detail\": \"%s\"}" "$cat" "$status" "$detail"
    first=0
  done

  echo ""
  echo "  },"
  echo "  \"summary\": {\"pass\": ${PASS_COUNT}, \"warn\": ${WARN_COUNT}, \"fail\": ${FAIL_COUNT}, \"skip\": ${SKIP_COUNT}}"
  echo "}"
}

# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════
main() {
  if [[ "$OUTPUT_MODE" == "verbose" ]]; then
    echo ""
    echo -e "${BOLD}${CYAN}RT Setup Verification (${TOTAL_CORES}-core system)${NC}"
    echo -e "${DIM}OS cores: ${OS_CORES_DESC}  |  RT cores: ${RT_CORES_START}-${RT_CORES_END}  |  IRQ mask: 0x${EXPECTED_IRQ_MASK}${NC}"
  fi

  check_rt_kernel
  check_cpu_isolation
  check_grub_params
  check_rt_permissions
  check_irq_affinity
  check_network_udp
  check_nvidia
  check_cpu_frequency

  if [[ "$OUTPUT_MODE" == "json" ]]; then
    print_json
  else
    print_summary
  fi

  # Exit code
  if [[ "$FAIL_COUNT" -gt 0 ]]; then
    exit 2
  elif [[ "$WARN_COUNT" -gt 0 ]]; then
    exit 1
  else
    exit 0
  fi
}

main
