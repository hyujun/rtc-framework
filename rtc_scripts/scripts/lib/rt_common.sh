#!/bin/bash
# rt_common.sh — RT 스크립트 공통 함수 라이브러리
#
# 모든 RT 설정 스크립트에서 공유하는 유틸리티 함수를 제공한다.
# 사용법: source "$(dirname "${BASH_SOURCE[0]}")/lib/rt_common.sh"
#
# 제공 함수:
#   get_physical_cores        — 물리 CPU 코어 수 (SMT/HT 제외)
#   detect_physical_nic       — UP 상태의 물리 NIC 자동 감지
#   setup_colors              — 터미널 색상 변수 초기화
#   make_logger PREFIX [STYLE]— 로깅 함수 생성 (bracket 또는 emoji 스타일)
#   compute_cpu_layout        — IRQ affinity mask / OS·RT 코어 범위 계산
#   compute_irq_affinity_mask — SMT-aware IRQ affinity bitmask 계산
#   compute_expected_isolated — isolcpus 기대값 계산
#   require_root              — root 권한 확인
#   write_file_if_changed     — 멱등 파일 쓰기
#   auto_release_cpu_shield   — 빌드 전 CPU shield 자동 해제
#   check_workspace_structure — ROS2 워크스페이스 구조 검증
#   ensure_ros2_sourced       — ROS2 환경 자동 탐색 및 소싱
#   create_oneshot_service    — systemd oneshot 서비스 생성 헬퍼

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
# make_logger "PREFIX" [STYLE]
#   STYLE: "bracket" (기본) — [PREFIX] 형식
#          "emoji"          — ▶ ✔ ⚠ ✘ 형식 (build.sh / install.sh용)
# 예: make_logger "IRQ"         → [IRQ] info message
#     make_logger "BUILD" emoji  → ▶ info message
_RT_LOG_PREFIX="RT"
_RT_LOG_STYLE="bracket"

make_logger() {
  _RT_LOG_PREFIX="${1:-RT}"
  _RT_LOG_STYLE="${2:-bracket}"
  _define_log_functions
}

_define_log_functions() {
  if [[ "$_RT_LOG_STYLE" == "emoji" ]]; then
    # emoji 스타일: build.sh / install.sh 호환
    eval 'info()    { echo -e "${BLUE}▶ $*${NC}"; }'
    eval 'warn()    { echo -e "${YELLOW}⚠ $*${NC}"; }'
    eval 'error()   { echo -e "${RED}✘ $*${NC}" >&2; }'
    eval 'success() { echo -e "${GREEN}✔ $*${NC}"; }'
    eval 'section() { echo -e "${CYAN}── $*${NC}"; }'
  else
    # bracket 스타일: RT 스크립트 기본
    local p="$_RT_LOG_PREFIX"
    eval "info()    { echo -e \"\${GREEN}[${p}]\${NC} \$*\"; }"
    eval "warn()    { echo -e \"\${YELLOW}[${p}]\${NC} \$*\"; }"
    eval "error()   { echo -e \"\${RED}[${p}]\${NC} \$*\" >&2; }"
    eval "success() { echo -e \"\${GREEN}[${p}]\${NC} \$*\"; }"
    eval "section() { echo -e \"\${BLUE}[${p}]\${NC} \$*\"; }"
  fi
}

# 초기 기본 로깅 함수 정의
_define_log_functions

# ── fatal(): error + exit 1 ────────────────────────────────────────────────
# error()와 달리 항상 프로세스를 종료한다.
# 개별 스크립트에서 error()를 재정의할 필요 없이 fatal()을 사용하면 된다.
fatal() {
  echo -e "${RED}[${_RT_LOG_PREFIX:-RT}]${NC} $*" >&2
  exit 1
}

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
    fatal "Root privileges required. Run: sudo $0 ${*}"
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
    OS_CORES_DESC="0"
    OS_PHYS_START=0
    OS_PHYS_END=0
    RT_CORES_START=1
  else
    OS_CORES_DESC="0-1"
    OS_PHYS_START=0
    OS_PHYS_END=1
    RT_CORES_START=2
  fi
  RT_CORES_END=$((TOTAL_CORES - 1))

  # SMT-aware IRQ affinity mask (HT 시블링 포함)
  IRQ_AFFINITY_MASK=$(compute_irq_affinity_mask)
}

# ── SMT-aware IRQ affinity mask 계산 ────────────────────────────────────────
# 비-SMT: 물리 코어 기반 단순 mask (0x1 또는 0x3)
# SMT: OS 물리 코어의 HT 시블링까지 포함한 mask
# 예: 8C/16T에서 물리 Core 0,1 → 논리 CPU 0,1,8,9 → mask=0x303
compute_irq_affinity_mask() {
  local logical
  logical=$(nproc --all)
  local physical
  physical=$(get_physical_cores)

  if [[ "$logical" -eq "$physical" ]]; then
    # 비-SMT: 단순 mask
    if [[ "$physical" -le 4 ]]; then
      echo "1"
    else
      echo "3"
    fi
    return
  fi

  # SMT: OS 물리 코어에 속하는 모든 논리 CPU의 bitmask
  local os_end=0
  [[ "$physical" -gt 4 ]] && os_end=1

  local mask=0
  for cpu_dir in /sys/devices/system/cpu/cpu[0-9]*/; do
    local cpu_num
    cpu_num=$(basename "$cpu_dir" | sed 's/cpu//')
    local core_file="${cpu_dir}topology/core_id"
    [[ -f "$core_file" ]] || continue
    local core_id
    core_id=$(cat "$core_file" 2>/dev/null)
    if [[ "$core_id" -ge 0 && "$core_id" -le "$os_end" ]]; then
      mask=$((mask | (1 << cpu_num)))
    fi
  done

  printf '%x' "$mask"
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

# ── CPU shield 자동 관리 (빌드 전) ──────────────────────────────────────────
# cset shield가 활성이면 자동 해제하여 전체 코어로 빌드한다.
# isolcpus(GRUB 고정)는 재부팅 없이 해제 불가 → 경고만 출력.
# $1: cpu_shield.sh 경로 (선택)
auto_release_cpu_shield() {
  local shield_script="${1:-}"
  local isolated
  isolated=$(cat /sys/devices/system/cpu/isolated 2>/dev/null || echo "")

  if [[ -z "$isolated" ]]; then
    return 0  # 격리 없음 → 전체 코어 사용 가능
  fi

  local available total
  available=$(nproc)
  total=$(nproc --all)
  warn "CPU 격리 감지: Core ${isolated} 격리 중 (${available}/${total} 코어 사용 가능)"

  # Case 1: cset shield 활성 → 자동 해제
  if command -v cset &>/dev/null && cset shield -s 2>/dev/null | grep -q "user"; then
    info "cset shield 감지 → 빌드를 위해 자동 해제 중..."
    if [[ -n "$shield_script" && -f "$shield_script" ]]; then
      sudo bash "$shield_script" off 2>/dev/null || sudo cset shield --reset 2>/dev/null || true
    else
      sudo cset shield --reset 2>/dev/null || true
    fi
    success "CPU 격리 해제 완료 — 전체 ${total} 코어로 빌드합니다"
    return 0
  fi

  # Case 2: isolcpus (GRUB 고정) → 해제 불가, 경고만
  if grep -q "isolcpus=" /proc/cmdline 2>/dev/null; then
    warn "isolcpus GRUB 파라미터 감지 — 재부팅 없이 해제 불가"
    warn "빌드에 ${available}/${total} 코어만 사용됩니다"
    warn "권장: isolcpus를 GRUB에서 제거하고 cset shield 방식으로 전환하세요"
    return 0
  fi

  return 0
}

# ── Workspace structure check ───────────────────────────────────────────────
# ROS2 워크스페이스 디렉토리 구조를 검증하고 WORKSPACE 전역 변수를 설정한다.
# $1: 스크립트 디렉토리 (기본: 호출자의 BASH_SOURCE[0] 기준)
check_workspace_structure() {
  local script_dir="${1:-$(cd "$(dirname "${BASH_SOURCE[1]}")" && pwd)}"
  info "Checking workspace directory structure..."
  local src_dir
  src_dir="$(dirname "$script_dir")"
  local detected_ws
  detected_ws="$(dirname "$src_dir")"

  if [[ "$(basename "$src_dir")" != "src" ]]; then
    echo -e "${RED}✘ Invalid directory structure. ROS2 packages must be located inside a 'src' directory.${NC}"
    echo -e "  Expected: ${BOLD}<workspace_dir>/src/<repository_name>${NC}"
    echo -e "  Current:  ${BOLD}${script_dir}${NC}"
    echo -e "  Example:  mkdir -p ~/ros2_ws/ur5e_ws/src && mv ${script_dir} ~/ros2_ws/ur5e_ws/src/"
    exit 1
  fi

  WORKSPACE="$detected_ws"
  success "Workspace correctly configured at: $WORKSPACE"
}

# ── ROS2 auto-source ─────────────────────────────────────────────────────────
# ros2 커맨드가 PATH에 없으면 /opt/ros/ 에서 탐색 후 자동 소싱.
ensure_ros2_sourced() {
  if command -v ros2 &>/dev/null; then
    return 0
  fi

  warn "ros2 command not found in PATH. Searching /opt/ros/ ..."
  if [[ -d /opt/ros ]]; then
    for _distro in jazzy humble iron rolling; do
      if [[ -f "/opt/ros/${_distro}/setup.bash" ]]; then
        info "Found ROS2 ${_distro} at /opt/ros/${_distro} — sourcing setup.bash ..."
        # shellcheck disable=SC1090
        source "/opt/ros/${_distro}/setup.bash" || true
        success "ROS2 ${_distro} sourced"
        return 0
      fi
    done
  fi

  fatal "ROS2 not found. Install ROS2 or source setup.bash before running this script"
}

# ── systemd oneshot 서비스 생성 헬퍼 ────────────────────────────────────────
# Usage: create_oneshot_service "SERVICE_PATH" "DESCRIPTION" "EXEC_START" "AFTER" "WANTED_BY"
# Returns: 0 if created/updated, 1 if already identical
create_oneshot_service() {
  local service_path="$1"
  local description="$2"
  local exec_start="$3"
  local after="${4:-multi-user.target}"
  local wanted_by="${5:-multi-user.target}"

  local content="[Unit]
Description=${description}
After=${after}

[Service]
Type=oneshot
ExecStart=${exec_start}
RemainAfterExit=yes

[Install]
WantedBy=${wanted_by}"

  if write_file_if_changed "$service_path" "$content"; then
    return 0  # 생성/변경됨
  fi
  return 1  # 이미 동일
}

# ── 공통 패키지 리스트 (build.sh / install.sh 공유) ─────────────────────────
# 이 함수들은 패키지 리스트의 single source of truth를 제공한다.
get_base_packages() {
  echo "rtc_msgs rtc_base rtc_communication rtc_controller_interface rtc_controllers rtc_controller_manager rtc_status_monitor rtc_inference rtc_scripts rtc_tools"
}

get_robot_packages() {
  echo "ur5e_description ur5e_hand_driver ur5e_hand_status_monitor ur5e_bringup"
}
