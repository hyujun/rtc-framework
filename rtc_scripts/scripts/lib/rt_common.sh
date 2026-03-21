#!/bin/bash
# rt_common.sh — RT 스크립트 공통 함수 라이브러리
#
# 모든 RT 설정 스크립트에서 공유하는 유틸리티 함수를 제공한다.
# 사용법: source "$(dirname "${BASH_SOURCE[0]}")/lib/rt_common.sh"
#
# 제공 함수:
#   get_physical_cores     — 물리 CPU 코어 수 (SMT/HT 제외)
#   detect_physical_nic    — UP 상태의 물리 NIC 자동 감지
#   setup_colors           — 터미널 색상 변수 초기화
#   make_logger PREFIX     — 로깅 함수(info/warn/error/success/section) 생성

# 이 파일은 직접 실행하지 않는다.
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "ERROR: This file should be sourced, not executed." >&2
  exit 1
fi

# ── 중복 source 방지 ───────────────────────────────────────────────────────
[[ -n "${_RT_COMMON_LOADED:-}" ]] && return 0
_RT_COMMON_LOADED=1

# ── Colors ──────────────────────────────────────────────────────────────────
# 호출 스크립트에서 직접 사용할 수 있도록 export하지 않고 전역 변수로 설정.
setup_colors() {
  if [[ -t 1 ]]; then
    GREEN='\033[0;32m'
    YELLOW='\033[1;33m'
    RED='\033[0;31m'
    BLUE='\033[0;34m'
    CYAN='\033[0;36m'
    BOLD='\033[1m'
    DIM='\033[2m'
    NC='\033[0m'
  else
    GREEN='' YELLOW='' RED='' BLUE='' CYAN='' BOLD='' DIM='' NC=''
  fi
}

# 기본 호출: source 시 자동 초기화
setup_colors

# ── Logging helpers ─────────────────────────────────────────────────────────
# make_logger "PREFIX" 를 호출하면 info/warn/error/success/section 함수를 생성한다.
# 예: make_logger "IRQ" → info() { echo -e "${GREEN}[IRQ]${NC} $*"; }
make_logger() {
  local prefix="$1"
  # 함수를 동적으로 정의 (eval 대신 직접 정의)
  # 호출 스크립트에서 이미 정의된 함수가 있으면 덮어쓴다.
  _RT_LOG_PREFIX="$prefix"
}

# 공통 로깅 함수 — make_logger 호출 후 사용 가능
info()    { echo -e "${GREEN}[${_RT_LOG_PREFIX:-RT}]${NC} $*"; }
warn()    { echo -e "${YELLOW}[${_RT_LOG_PREFIX:-RT}]${NC} $*"; }
error()   { echo -e "${RED}[${_RT_LOG_PREFIX:-RT}]${NC} $*" >&2; }
success() { echo -e "${GREEN}[${_RT_LOG_PREFIX:-RT}]${NC} $*"; }
section() { echo -e "${BLUE}[${_RT_LOG_PREFIX:-RT}]${NC} $*"; }

# ── Physical CPU core count (SMT/HT 제외) ──────────────────────────────────
# nproc은 논리 코어(HT 포함)를 반환하므로, 물리 코어만 카운트한다.
# 예: i7-8700 (6C/12T) → nproc=12 이지만 이 함수는 6을 반환.
# thread_config.hpp의 코어 레이아웃은 물리 코어 기준이므로 반드시 물리 코어를 사용해야 한다.
get_physical_cores() {
  # 방법 1: lscpu (가장 빠름)
  if command -v lscpu &>/dev/null; then
    local count
    count=$(lscpu -p=Core,Socket 2>/dev/null | grep -v '^#' | sort -u | wc -l)
    if [[ "$count" -gt 0 ]]; then
      echo "$count"
      return
    fi
  fi
  # 방법 2: sysfs topology 직접 파싱
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
  # 방법 3: 최후 수단 (VM, 컨테이너 등에서 sysfs 미지원 시)
  nproc --all
}

# ── Physical NIC auto-detection ─────────────────────────────────────────────
# /sys/class/net/<iface>/device 심볼릭 링크는 물리 NIC에만 존재한다.
# 가상 인터페이스(docker0, veth*, br-* 등)를 자동으로 제외한다.
# UP 상태 NIC를 우선 반환, 없으면 첫 번째 물리 NIC를 반환한다.
detect_physical_nic() {
  local iface
  # 1차: UP 상태 물리 NIC
  for iface in /sys/class/net/*/; do
    iface=$(basename "$iface")
    [[ "$iface" == "lo" ]] && continue
    [[ -e "/sys/class/net/${iface}/device" ]] || continue
    if ip link show "$iface" 2>/dev/null | grep -q 'state UP'; then
      echo "$iface"
      return
    fi
  done
  # 2차: 아무 물리 NIC
  for iface in /sys/class/net/*/; do
    iface=$(basename "$iface")
    [[ "$iface" == "lo" ]] && continue
    [[ -e "/sys/class/net/${iface}/device" ]] || continue
    echo "$iface"
    return
  done
}

# ── Root privilege check ────────────────────────────────────────────────────
require_root() {
  if [[ "$EUID" -ne 0 ]]; then
    error "Root privileges required. Run: sudo $0 ${*}"
    exit 1
  fi
}

# ── Idempotent file write ───────────────────────────────────────────────────
# 파일 내용이 동일하면 skip, 다르면 백업 후 덮어쓴다.
# Usage: write_file_if_changed "/path/to/file" "$CONTENT" [backup=true]
# Returns: 0 if written, 1 if skipped (already identical)
write_file_if_changed() {
  local file="$1"
  local content="$2"
  local do_backup="${3:-true}"

  if [[ -f "$file" ]] && diff -q <(echo "$content") "$file" &>/dev/null; then
    return 1  # 동일 — skip
  fi

  if [[ "$do_backup" == "true" && -f "$file" ]]; then
    cp "$file" "${file}.bak.$(date +%Y%m%d_%H%M%S)"
  fi

  echo "$content" > "$file"
  return 0  # 기록됨
}

# ── CPU layout helpers ──────────────────────────────────────────────────────
# thread_config.hpp의 코어 레이아웃과 일치하는 IRQ affinity mask 및 코어 범위를 계산.
# nproc --all: isolcpus로 격리된 CPU 포함 전체 논리 코어 수 반환
compute_cpu_layout() {
  LOGICAL_CORES=$(nproc --all)
  TOTAL_CORES=$(get_physical_cores)
  HAS_SMT=0
  if [[ "$LOGICAL_CORES" -ne "$TOTAL_CORES" ]]; then
    HAS_SMT=1
  fi

  if [[ "$TOTAL_CORES" -le 4 ]]; then
    IRQ_AFFINITY_MASK="1"   # 0x1 = Core 0 only
    OS_CORES_DESC="0"
    OS_PHYS_START=0
    OS_PHYS_END=0
    RT_CORES_START=1
  else
    IRQ_AFFINITY_MASK="3"   # 0x3 = Core 0-1
    OS_CORES_DESC="0-1"
    OS_PHYS_START=0
    OS_PHYS_END=1
    RT_CORES_START=2
  fi
  RT_CORES_END=$((TOTAL_CORES - 1))
}

# ── OS 물리 코어에 속하는 논리 CPU 목록 ────────────────────────────────────
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

# ── isolcpus 기대값 계산 (SMT 시블링 포함) ─────────────────────────────────
# 비-SMT: 단순 범위 (예: "2-5")
# SMT: OS 물리 코어의 HT 시블링을 제외한 나머지 (예: "2-5,8-11")
compute_expected_isolated() {
  if [[ "$HAS_SMT" -eq 0 ]]; then
    if [[ "$TOTAL_CORES" -le 4 ]]; then
      echo "1-$((TOTAL_CORES - 1))"
    else
      echo "2-$((TOTAL_CORES - 1))"
    fi
    return
  fi

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
  _format_cpu_range "${isolated_list[@]}"
}

# ── CPU 번호 배열을 범위 표기 문자열로 변환 ─────────────────────────────────
# 예: _format_cpu_range 2 3 4 5 8 9 10 11 → "2-5,8-11"
_format_cpu_range() {
  local result=""
  local range_start="" range_end=""
  for cpu in "$@"; do
    if [[ -z "$range_start" ]]; then
      range_start=$cpu
      range_end=$cpu
    elif [[ "$cpu" -eq $((range_end + 1)) ]]; then
      range_end=$cpu
    else
      if [[ "$range_start" -eq "$range_end" ]]; then
        result="${result:+${result},}${range_start}"
      else
        result="${result:+${result},}${range_start}-${range_end}"
      fi
      range_start=$cpu
      range_end=$cpu
    fi
  done
  if [[ -n "$range_start" ]]; then
    if [[ "$range_start" -eq "$range_end" ]]; then
      result="${result:+${result},}${range_start}"
    else
      result="${result:+${result},}${range_start}-${range_end}"
    fi
  fi
  echo "$result"
}
