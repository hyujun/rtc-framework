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
  nproc --all
}

# ── CPU layout auto-detection ────────────────────────────────────────────────
# thread_config.hpp는 물리 코어 수로 레이아웃(4/6/8-core)을 선택하고,
# 물리 코어 인덱스(= 논리 CPU 번호)에 스레드를 pinning한다.
# 하지만 isolcpus는 논리 CPU 번호를 사용하므로, SMT/HT 시스템에서는
# RT 물리 코어의 HT 시블링 논리 CPU도 격리 대상에 포함해야 한다.
# nproc --all: isolcpus로 격리된 CPU 포함 전체 논리 코어 수 반환
# nproc (without --all)은 격리된 CPU를 제외하므로 isolcpus 설정 시 잘못된 값 반환
LOGICAL_CORES=$(nproc --all)
TOTAL_CORES=$(get_physical_cores)
HAS_SMT=0
if [[ "$LOGICAL_CORES" -ne "$TOTAL_CORES" ]]; then
  HAS_SMT=1
fi

if [[ "$TOTAL_CORES" -le 4 ]]; then
  EXPECTED_IRQ_MASK="1"      # 0x1 = Core 0 only
  OS_CORES_DESC="0"
  OS_PHYS_START=0
  OS_PHYS_END=0
  RT_CORES_START=1           # 물리 코어 기준 (thread_config.hpp 인덱스)
else
  EXPECTED_IRQ_MASK="3"      # 0x3 = Core 0-1
  OS_CORES_DESC="0-1"
  OS_PHYS_START=0
  OS_PHYS_END=1
  RT_CORES_START=2           # 물리 코어 기준
fi
RT_CORES_END=$((TOTAL_CORES - 1))

# ── Helper: sysfs에서 OS 물리 코어에 속하는 모든 논리 CPU 번호 조회 ──────────
# 반환값: OS에 해당하는 논리 CPU 번호 집합 (나머지가 isolcpus 대상)
get_os_logical_cpus() {
  local os_cpus=""
  for cpu_dir in /sys/devices/system/cpu/cpu[0-9]*/; do
    local cpu_num
    cpu_num=$(basename "$cpu_dir" | sed 's/cpu//')
    local core_file="${cpu_dir}topology/core_id"
    [[ -f "$core_file" ]] || continue
    local core_id
    core_id=$(cat "$core_file" 2>/dev/null)
    if [[ "$core_id" -ge "$OS_PHYS_START" && "$core_id" -le "$OS_PHYS_END" ]]; then
      os_cpus="${os_cpus} ${cpu_num}"
    fi
  done
  echo "$os_cpus"
}

# ── isolcpus 기대값 계산 (논리 CPU 기준) ──────────────────────────────────────
# 비-SMT: 물리 = 논리이므로 단순 범위 (예: "2-5")
# SMT: OS 물리 코어의 모든 HT 시블링을 제외한 나머지 (예: "2-5,8-11")
compute_expected_isolated() {
  if [[ "$HAS_SMT" -eq 0 ]]; then
    # 비-SMT: 단순 범위
    if [[ "$TOTAL_CORES" -le 4 ]]; then
      echo "1-$((TOTAL_CORES - 1))"
    else
      echo "2-$((TOTAL_CORES - 1))"
    fi
    return
  fi

  # SMT 시스템: OS 코어에 속하지 않는 모든 논리 CPU를 구한다
  local os_cpus
  os_cpus=$(get_os_logical_cpus)

  local isolated_list=()
  for ((cpu=0; cpu<LOGICAL_CORES; cpu++)); do
    local is_os=0
    for os_cpu in $os_cpus; do
      if [[ "$cpu" -eq "$os_cpu" ]]; then
        is_os=1
        break
      fi
    done
    if [[ "$is_os" -eq 0 ]]; then
      isolated_list+=("$cpu")
    fi
  done

  # 연속된 번호를 범위 표기로 변환 (예: 2 3 4 5 8 9 10 11 → "2-5,8-11")
  local result=""
  local range_start="" range_end=""
  for cpu in "${isolated_list[@]}"; do
    if [[ -z "$range_start" ]]; then
      range_start=$cpu
      range_end=$cpu
    elif [[ "$cpu" -eq $((range_end + 1)) ]]; then
      range_end=$cpu
    else
      # 이전 범위 출력
      if [[ "$range_start" -eq "$range_end" ]]; then
        result="${result:+${result},}${range_start}"
      else
        result="${result:+${result},}${range_start}-${range_end}"
      fi
      range_start=$cpu
      range_end=$cpu
    fi
  done
  # 마지막 범위 출력
  if [[ -n "$range_start" ]]; then
    if [[ "$range_start" -eq "$range_end" ]]; then
      result="${result:+${result},}${range_start}"
    else
      result="${result:+${result},}${range_start}-${range_end}"
    fi
  fi
  echo "$result"
}

EXPECTED_ISOLATED=$(compute_expected_isolated)

# IRQ affinity 체크에 사용할 논리 RT CPU 목록 (RT 물리 코어 + HT 시블링 포함)
# should_be_isolated[cpu]=1 이면 해당 논리 CPU에 IRQ가 있으면 안 됨
declare -A IS_RT_LOGICAL_CPU
if [[ "$HAS_SMT" -eq 0 ]]; then
  for ((core=RT_CORES_START; core<=RT_CORES_END; core++)); do
    IS_RT_LOGICAL_CPU[$core]=1
  done
else
  _os_cpus=$(get_os_logical_cpus)
  for ((_cpu=0; _cpu<LOGICAL_CORES; _cpu++)); do
    _is_os=0
    for _os_cpu in $_os_cpus; do
      [[ "$_cpu" -eq "$_os_cpu" ]] && { _is_os=1; break; }
    done
    if [[ "$_is_os" -eq 0 ]]; then
      IS_RT_LOGICAL_CPU[$_cpu]=1
    fi
  done
  unset _os_cpus _cpu _is_os _os_cpu
fi

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
    _warn "CPU 격리 미활성 — 로봇/시뮬 실행 시 자동 활성화됩니다"
    _fix "수동 활성화: sudo ${SCRIPT_DIR}/cpu_shield.sh on [--robot|--sim]"
    _category_update "cpu_isolation" "WARN"
    _category_set_detail "cpu_isolation" "none (auto-activate on launch)"
  elif [[ "$isolated" == "$EXPECTED_ISOLATED" ]]; then
    # Detect isolation method
    if grep -q "isolcpus=" /proc/cmdline 2>/dev/null; then
      _pass "CPU 격리 일치 (isolcpus): ${isolated}"
      _warn "isolcpus는 빌드 성능에 영향. cset shield 전환 권장"
      _fix "GRUB에서 isolcpus 제거 후 sudo cpu_shield.sh on --robot 사용"
      _category_update "cpu_isolation" "WARN"
    else
      _pass "CPU 격리 일치 (cset shield): ${isolated}"
    fi
    if [[ "$HAS_SMT" -eq 1 ]]; then
      _pass "SMT/HT 시블링 포함 격리 확인됨 (${LOGICAL_CORES} logical CPUs)"
    fi
    _category_set_detail "cpu_isolation" "${isolated}"
  else
    # 부분 일치 여부 확인: 기대값의 코어가 모두 포함되어 있는지
    local partial_ok=1
    # EXPECTED_ISOLATED의 범위를 파싱하여 actual에 모두 포함되는지 확인
    # (단순 문자열 불일치이지만 실제로는 기능적으로 충분할 수 있음)
    _warn "CPU 격리 불일치: ${isolated} (기대값: ${EXPECTED_ISOLATED})"
    if [[ "$HAS_SMT" -eq 1 ]]; then
      _warn "SMT/HT 시스템: 물리 ${TOTAL_CORES}코어 / 논리 ${LOGICAL_CORES}코어"
      _warn "isolcpus는 논리 CPU 번호를 사용해야 합니다 (HT 시블링 포함)"
    fi
    _fix "GRUB의 isolcpus=${isolated} → isolcpus=${EXPECTED_ISOLATED} 로 변경"
    _fix "sudo ${SCRIPT_DIR}/setup_nvidia_rt.sh"
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
  # isolcpus는 cset shield로 대체되어 선택 사항으로 변경
  local -a params_required=("nohz_full" "rcu_nocbs" "processor.max_cstate")
  for param in "${params_required[@]}"; do
    ((total++)) || true
    if echo "$cmdline" | grep -qE "(^| )${param}="; then
      _pass "${param} 설정됨"
      ((found++)) || true
    else
      _fail "${param} 미설정"
      _category_update "grub_params" "FAIL"
    fi
  done

  # isolcpus: 선택 사항 (cset shield로 대체 가능)
  ((total++)) || true
  if echo "$cmdline" | grep -qE "(^| )isolcpus="; then
    _warn "isolcpus 설정됨 (cset shield로 전환 권장 — 빌드 성능 향상)"
    _fix "GRUB에서 isolcpus 제거 후 cpu_shield.sh 사용"
    ((found++)) || true
    _category_update "grub_params" "WARN"
  else
    _pass "isolcpus 미사용 (cset shield 동적 격리 방식 — 정상)"
    ((found++)) || true
  fi

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

    # RT 논리 CPU에 IRQ가 할당되어 있는지 확인 (SMT 시블링 포함)
    local on_rt=0
    for cpu in "${!IS_RT_LOGICAL_CPU[@]}"; do
      if (( mask_dec & (1 << cpu) )); then
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

  # nvidia-smi 동작 여부 (DKMS 모듈 빌드 상태 반영)
  if command -v nvidia-smi &>/dev/null; then
    if nvidia-smi &>/dev/null; then
      _pass "nvidia-smi 정상 작동"

      # Persistence mode
      local persist
      persist=$(nvidia-smi -q -d PERFORMANCE 2>/dev/null | grep -i "persistence" | head -1 || echo "")
      if echo "$persist" | grep -qi "enabled"; then
        _pass "Persistence mode: Enabled"
      elif [[ -n "$persist" ]]; then
        _warn "Persistence mode: Disabled (RT 지연 원인 가능)"
        _category_update "nvidia" "WARN"
      fi
    else
      _fail "nvidia-smi 실행 실패 — NVIDIA 커널 모듈 미로드 (DKMS 빌드 필요)"
      _fix "sudo IGNORE_PREEMPT_RT_PRESENCE=1 dkms autoinstall && sudo modprobe nvidia"
      _fix "또는: sudo ${SCRIPT_DIR}/setup_nvidia_rt.sh"
      _category_update "nvidia" "FAIL"
    fi
  else
    _fail "nvidia-smi 미설치 — NVIDIA 드라이버 패키지 또는 DKMS 빌드 필요"
    _fix "sudo ${SCRIPT_DIR}/setup_nvidia_rt.sh"
    _category_update "nvidia" "FAIL"
  fi

  # DKMS 모듈 상태
  if command -v dkms &>/dev/null; then
    local dkms_status
    dkms_status=$(dkms status nvidia 2>/dev/null | grep "$(uname -r)" || true)
    if echo "$dkms_status" | grep -q "installed"; then
      _pass "NVIDIA DKMS 모듈: installed ($(uname -r))"
    elif [[ -n "$dkms_status" ]]; then
      _warn "NVIDIA DKMS 모듈: $(echo "$dkms_status" | head -1)"
      _category_update "nvidia" "WARN"
    else
      _warn "NVIDIA DKMS 모듈: 현재 커널($(uname -r))용 미빌드"
      _fix "sudo IGNORE_PREEMPT_RT_PRESENCE=1 dkms autoinstall"
      _category_update "nvidia" "WARN"
    fi
  fi

  # IGNORE_PREEMPT_RT_PRESENCE 영구 설정 확인
  if [[ -f /etc/environment ]] && grep -q "IGNORE_PREEMPT_RT_PRESENCE" /etc/environment 2>/dev/null; then
    _pass "IGNORE_PREEMPT_RT_PRESENCE=1 영구 설정 (/etc/environment)"
  else
    local kernel_ver
    kernel_ver=$(uname -r)
    if echo "$kernel_ver" | grep -qi "rt\|realtime"; then
      _warn "IGNORE_PREEMPT_RT_PRESENCE=1 미설정 — DKMS 자동 빌드 시 실패 가능"
      _fix "echo 'IGNORE_PREEMPT_RT_PRESENCE=1' | sudo tee -a /etc/environment"
      _category_update "nvidia" "WARN"
    fi
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

  # 모든 코어의 governor 확인 (OS 코어도 powersave이면 compositor 프레임 드롭 발생)
  local non_perf=0
  local checked=0
  local rt_non_perf=0

  for ((core=0; core<LOGICAL_CORES; core++)); do
    local gov_file="/sys/devices/system/cpu/cpu${core}/cpufreq/scaling_governor"
    if [[ -f "$gov_file" ]]; then
      local gov
      gov=$(cat "$gov_file" 2>/dev/null || echo "unknown")
      ((checked++)) || true
      if [[ "$gov" != "performance" ]]; then
        ((non_perf++)) || true
        # RT 코어인지 확인
        if [[ -n "${IS_RT_LOGICAL_CPU[$core]:-}" ]]; then
          ((rt_non_perf++)) || true
        fi
        if [[ "$OUTPUT_MODE" == "verbose" ]]; then
          _warn "Core ${core}: governor = ${gov} (performance 권장)"
        fi
      fi
    fi
  done

  if [[ "$checked" -eq 0 ]]; then
    _skip "cpufreq 정보 없음"
    _category_set_detail "cpu_frequency" "unknown"
    return
  fi

  if [[ "$non_perf" -eq 0 ]]; then
    _pass "모든 코어(${checked}개) performance governor"
    _category_set_detail "cpu_frequency" "all performance"
  else
    if [[ "$rt_non_perf" -gt 0 ]]; then
      _warn "RT 코어 중 ${rt_non_perf}개가 performance 아님 — 제어 루프 jitter 위험"
      _category_update "cpu_frequency" "WARN"
    fi
    if [[ "$((non_perf - rt_non_perf))" -gt 0 ]]; then
      _warn "OS 코어 중 $((non_perf - rt_non_perf))개가 performance 아님 — 화면 끊김 위험"
      _category_update "cpu_frequency" "WARN"
    fi
    _fix "sudo cpupower frequency-set -g performance"
    _fix "또는: sudo ${SCRIPT_DIR}/setup_nvidia_rt.sh  (영구 서비스 포함)"
    _category_set_detail "cpu_frequency" "${non_perf}/${checked} non-performance"
  fi

  # cpu-governor-performance.service 상태 확인
  if systemctl is-enabled cpu-governor-performance.service &>/dev/null; then
    _pass "cpu-governor-performance.service: enabled (재부팅 시 자동 적용)"
  else
    if [[ "$non_perf" -gt 0 ]]; then
      _warn "cpu-governor-performance.service: not enabled (재부팅 시 powersave 복원)"
      _fix "sudo ${SCRIPT_DIR}/setup_nvidia_rt.sh  (서비스 자동 생성)"
      _category_update "cpu_frequency" "WARN"
    fi
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
    local smt_info=""
    if [[ "$HAS_SMT" -eq 1 ]]; then
      smt_info="  |  SMT: ${LOGICAL_CORES} logical"
    fi
    echo -e "${BOLD}${CYAN}RT Setup Verification (${TOTAL_CORES}-core system${smt_info})${NC}"
    echo -e "${DIM}OS cores: ${OS_CORES_DESC}  |  RT cores: ${RT_CORES_START}-${RT_CORES_END}  |  isolcpus: ${EXPECTED_ISOLATED}  |  IRQ mask: 0x${EXPECTED_IRQ_MASK}${NC}"
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
