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
    eval 'error()   { echo -e "${RED}✘ $*${NC}" >&2; exit 1; }'
    eval 'success() { echo -e "${GREEN}✔ $*${NC}"; }'
    eval 'section() { echo -e "${CYAN}── $*${NC}"; }'
  else
    # bracket 스타일: RT 스크립트 기본
    local p="$_RT_LOG_PREFIX"
    eval "info()    { echo -e \"\${GREEN}[${p}]\${NC} \$*\"; }"
    eval "warn()    { echo -e \"\${YELLOW}[${p}]\${NC} \$*\"; }"
    eval "error()   { echo -e \"\${RED}[${p}]\${NC} \$*\" >&2; exit 1; }"
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
  # 방법 1: lscpu (가장 빠름).
  # `-p=Core,Socket` 는 헤더(`#` 시작) 외에 일반적으로 빈 줄을 emit 하지 않지만,
  # rtc_tools/launch/thread_layout.py 의 Python mirror 와 정확히 동일한 행 필터를
  # 보장하기 위해 빈 줄도 명시적으로 제외한다 (drift 방지).
  if command -v lscpu &>/dev/null; then
    local count
    count=$(lscpu -p=Core,Socket 2>/dev/null | grep -v '^#' | grep -v '^$' | sort -u | wc -l)
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

  # Layout v4.1: Core 0 alone is reserved for OS / DDS / IRQ on every tier.
  # The RT cluster (rt_control, rt_callback, mpc_*) starts at Core 1.
  OS_CORES_DESC="0"
  OS_PHYS_START=0
  OS_PHYS_END=0
  RT_CORES_START=1
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
    # 비-SMT: Core 0 만 OS / IRQ (layout v4.1) → mask = 0x1
    echo "1"
    return
  fi

  # SMT: Core 0 의 HT 시블링까지 포함한 bitmask (layout v4.1)
  # OS_PHYS_END 는 compute_cpu_layout 이 설정. 단독 호출 시 0 fallback.
  local os_end="${OS_PHYS_END:-0}"

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
    # Layout v4.1: RT cluster starts at Core 1 on every tier.
    echo "1-$((TOTAL_CORES - 1))"
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

# ── Venv detection helpers ──────────────────────────────────────────────────
# venv 활성 여부 확인
is_venv_active() {
  [[ -n "${VIRTUAL_ENV:-}" ]]
}

# venv 내에서도 시스템 Python 경로를 반환
# eigenpy/pinocchio cmake가 apt-installed numpy를 찾을 수 있도록 함
get_system_python() {
  local py
  py=$(command -v python3 2>/dev/null || echo "/usr/bin/python3")
  py=$(readlink -f "$py" 2>/dev/null || echo "$py")
  # venv 내부 Python이면 시스템 Python으로 대체
  if is_venv_active && [[ "$py" == "${VIRTUAL_ENV}"* ]]; then
    py="/usr/bin/python3"
  fi
  echo "$py"
}

# ── 공통 argument parsing (build.sh / install.sh 공유) ─────────────────────
# 공통 옵션을 파싱하고 전역 변수에 설정한다.
# 각 스크립트 고유 옵션은 REMAINING_ARGS 배열로 반환된다.
# 사용법:
#   parse_common_args "$@"
#   MODE="$_COMMON_MODE" BUILD_TYPE="$_COMMON_BUILD_TYPE" ...
#   set -- "${REMAINING_ARGS[@]}"  # 나머지 인자로 재설정
_COMMON_MODE="full"
_COMMON_BUILD_TYPE="Release"
_COMMON_CLEAN_BUILD=0
_COMMON_PARALLEL_JOBS=""
_COMMON_MJ_DIR=""
_COMMON_CUSTOM_PACKAGES=()
REMAINING_ARGS=()

parse_common_args() {
  _COMMON_MODE="full"
  _COMMON_BUILD_TYPE="Release"
  _COMMON_CLEAN_BUILD=0
  _COMMON_PARALLEL_JOBS=""
  _COMMON_MJ_DIR=""
  _COMMON_CUSTOM_PACKAGES=()
  REMAINING_ARGS=()

  while [[ $# -gt 0 ]]; do
    case "$1" in
      robot|real|realrobot)
        _COMMON_MODE=robot; shift ;;
      sim|simulation)
        _COMMON_MODE=sim; shift ;;
      full)
        _COMMON_MODE=full; shift ;;
      -d|--debug)
        _COMMON_BUILD_TYPE="Debug"; shift ;;
      -r|--release)
        _COMMON_BUILD_TYPE="Release"; shift ;;
      -c|--clean)
        _COMMON_CLEAN_BUILD=1; shift ;;
      -p|--packages)
        [[ -z "${2:-}" ]] && fatal "--packages requires a comma-separated list"
        IFS=',' read -r -a _COMMON_CUSTOM_PACKAGES <<< "$2"
        shift 2 ;;
      -j|--jobs)
        [[ -z "${2:-}" ]] && fatal "--jobs requires a number"
        _COMMON_PARALLEL_JOBS="$2"
        shift 2 ;;
      --mujoco)
        [[ -z "${2:-}" ]] && fatal "--mujoco requires a path argument"
        _COMMON_MJ_DIR="$2"
        shift 2 ;;
      *)
        REMAINING_ARGS+=("$1"); shift ;;
    esac
  done
}

# ── 공통 패키지 리스트 (build.sh / install.sh 공유) ─────────────────────────
# 이 함수들은 패키지 리스트의 single source of truth를 제공한다.
get_base_packages() {
  echo "rtc_msgs rtc_base rtc_communication rtc_controller_interface rtc_urdf_bridge rtc_mpc rtc_tsid rtc_controllers rtc_controller_manager rtc_inference repo_scripts rtc_tools robot_descriptions shape_estimation_msgs shape_estimation"
}

get_robot_packages() {
  echo "udp_hand_driver integrated_bringup ur5e_bt_coordinator"
}

# ── MPC core layout helpers (layout v4.1) ────────────────────────────────────
# Single source of truth for MPC thread core assignment. Must stay in sync
# with rtc_base/threading/thread_config.hpp (SelectThreadConfigs dispatch).
#
# Layout v4.1 policy (RT cluster starts at Core 1; Core 0 = OS/DDS/IRQ only):
#   ≤4 cores  → MPC on Core 3, SCHED_OTHER (degraded).
#   5-9      → Core 3 dedicated to MPC main (FIFO 60).
#   10-11    → Core 3 main + Core 4 worker 0.
#   12-13    → Core 3 main + Core 4-5 workers (2 workers).
#   14-15    → Same as 12-13.
#   16+      → Same as 12-13 (cset shield "user" removed in v4.1).
#
# Prints a comma-separated list of cores. First entry is always the MPC
# main thread's core.
get_mpc_cores() {
  local ncpu
  ncpu=$(get_physical_cores)
  case "$ncpu" in
    1|2|3|4)      echo "3" ;;
    5|6|7)        echo "3" ;;
    8|9)          echo "3" ;;
    10|11)        echo "3,4" ;;
    12|13)        echo "3,4,5" ;;
    14|15)        echo "3,4,5" ;;
    *)            echo "3,4,5" ;;
  esac
}

# Print just the main MPC core (first entry of get_mpc_cores).
get_mpc_main_core() {
  get_mpc_cores | cut -d',' -f1
}

# Print the list of RT cores (rt_control + rt_callback + MPC main + workers).
# Used by IRQ affinity and GRUB nohz_full/rcu_nocbs. Order is not guaranteed.
# Layout v4.1: rt_control=1, rt_callback=2 (DDS recv co-pin via launch taskset,
# CFS — not an RT thread), MPC follows from Core 3. hand UDP receive thread
# lives inside the hand_driver process (cpu_core=-1 sentinel) and is not
# represented here. SSoT: rtc_base/threading/thread_config.hpp.
get_rt_cores() {
  local ncpu
  ncpu=$(get_physical_cores)
  local mpc
  mpc=$(get_mpc_cores)
  case "$ncpu" in
    1|2|3|4)      echo "1,2,${mpc}" ;;       # rt_control=1, rt_callback=2 (FIFO 70), MPC=3 (CFS, degraded)
    *)            echo "1,2,${mpc}" ;;       # rt_control=1, rt_callback=2, MPC starts at 3
  esac
}

# Print the list of OS cores (complement of get_rt_cores).
# Layout v4.1: Core 0 alone is reserved for OS / DDS / IRQ; nrt_* moved to
# dedicated cores in every ≥ 6-core tier.
get_os_cores() {
  echo "0"
}

# ── Canonical thread layout printout ───────────────────────────────────────
# Mirrors rtc_base/threading/thread_config.hpp::SelectThreadConfigs().
# Callers (cpu_shield.sh::do_status, setup_irq_affinity.sh) used to keep
# their own copies of this table; consolidating it here removes the drift
# risk between the shell- and C++-side views.
#
# Usage:
#   print_thread_layout                  # auto-detect physical core count
#   print_thread_layout 12               # explicit core count
#
# Each line is indented with two spaces and uses `info`-style coloring so
# the output blends with the surrounding [PREFIX] log lines.
print_thread_layout() {
  local ncpu="${1:-$(get_physical_cores)}"
  echo -e "  ${BOLD}Thread layout (${ncpu}-core, layout v4.1)${NC}"
  if [[ "$ncpu" -le 4 ]]; then
    echo "    Core 0:   OS / DDS / NIC IRQ + nrt_logging + nrt_callback + arm/hand_driver (degraded)"
    echo "    Core 1:   rt_control   (SCHED_FIFO 90)"
    echo "    Core 2:   rt_callback  (SCHED_FIFO 70; DDS recv co-pin, degraded)"
    echo "    Core 3:   mpc_main     (CFS, degraded)"
  elif [[ "$ncpu" -le 7 ]]; then
    echo "    Core 0:   OS / DDS / NIC IRQ"
    echo "    Core 1:   rt_control   (SCHED_FIFO 90)"
    echo "    Core 2:   rt_callback  (SCHED_FIFO 70; DDS recv co-pin via taskset, CFS)"
    echo "    Core 3:   mpc_main     (SCHED_FIFO 60)"
    echo "    Core 4:   arm_driver + hand_driver (shared, degraded)"
    echo "    Core 5:   nrt_logging (CFS -5) + nrt_callback (CFS 0) shared (degraded)"
  elif [[ "$ncpu" -le 9 ]]; then
    echo "    Core 0:   OS / DDS / NIC IRQ"
    echo "    Core 1:   rt_control   (SCHED_FIFO 90)"
    echo "    Core 2:   rt_callback  (SCHED_FIFO 70; DDS recv co-pin via taskset, CFS)"
    echo "    Core 3:   mpc_main     (SCHED_FIFO 60)"
    echo "    Core 4:   arm_driver   (CFS, taskset pin)"
    echo "    Core 5:   hand_driver  (CFS, taskset pin; internal recv thread FIFO 65)"
    echo "    Core 6:   nrt_logging  (CFS -5)"
    echo "    Core 7:   nrt_callback (CFS 0)"
  elif [[ "$ncpu" -le 11 ]]; then
    echo "    Core 0:   OS / DDS / NIC IRQ"
    echo "    Core 1:   rt_control   (SCHED_FIFO 90)"
    echo "    Core 2:   rt_callback  (SCHED_FIFO 70; DDS recv co-pin via taskset, CFS)"
    echo "    Core 3-4: mpc_main + worker_0 (SCHED_FIFO 60 / 55)"
    echo "    Core 5:   arm_driver   (CFS, taskset pin)"
    echo "    Core 6:   hand_driver  (CFS, taskset pin)"
    echo "    Core 7:   nrt_logging  (CFS -5)"
    echo "    Core 8:   nrt_callback (CFS 0)"
    echo "    Core 9:   spare"
  elif [[ "$ncpu" -le 13 ]]; then
    echo "    Core 0:   OS / DDS / NIC IRQ"
    echo "    Core 1:   rt_control   (SCHED_FIFO 90)"
    echo "    Core 2:   rt_callback  (SCHED_FIFO 70; DDS recv co-pin via taskset, CFS)"
    echo "    Core 3-5: mpc_main + workers (SCHED_FIFO 60 / 55 / 55)"
    echo "    Core 6:   arm_driver   (CFS, taskset pin)"
    echo "    Core 7:   hand_driver  (CFS, taskset pin)"
    echo "    Core 8:   nrt_logging  (CFS -5)"
    echo "    Core 9:   nrt_callback (CFS 0)"
    echo "    Core 10-${ncpu}: spare"
  elif [[ "$ncpu" -le 15 ]]; then
    echo "    Core 0:   OS / DDS / NIC IRQ"
    echo "    Core 1:   rt_control   (SCHED_FIFO 90)"
    echo "    Core 2:   rt_callback  (SCHED_FIFO 70; DDS recv co-pin via taskset, CFS)"
    echo "    Core 3-5: mpc_main + workers (SCHED_FIFO 60 / 55 / 55)"
    echo "    Core 6:   arm_driver   (CFS, taskset pin)"
    echo "    Core 7:   hand_driver  (CFS, taskset pin)"
    echo "    Core 8:   nrt_logging  (CFS -5)"
    echo "    Core 9:   nrt_callback (CFS 0)"
    echo "    Core 10-${ncpu}: spare / user shield"
  else
    echo "    Core 0:     OS / DDS / NIC IRQ"
    echo "    Core 1:     rt_control   (SCHED_FIFO 90)"
    echo "    Core 2:     rt_callback  (SCHED_FIFO 70; DDS recv co-pin via taskset, CFS)"
    echo "    Core 3-5:   mpc_main + workers (SCHED_FIFO 60 / 55 / 55)"
    echo "    Core 6:     arm_driver   (CFS, taskset pin)"
    echo "    Core 7:     hand_driver  (CFS, taskset pin)"
    echo "    Core 8:     nrt_logging  (CFS -5)"
    echo "    Core 9:     nrt_callback (CFS 0)"
    echo "    Core 10-15: spare / user shield"
    echo "    Core 16+:   spare / monitoring"
  fi
}

# ── Intel hybrid CPU detection (Stage A) ────────────────────────────────────
# C++ 측 rtc::DetectCpuTopology() (cpu_topology.hpp)와 동일한 감지 로직을
# shell에서 재현한다. 두 구현은 같은 입력(sysfs + /proc/cpuinfo)을 소비하므로
# 결과가 일치해야 한다 — 테스트(test_rt_common.sh)가 이를 강제한다.
#
# Stage A에서는 tier 선택이나 IRQ affinity에 영향을 주지 않는다. 감지 결과는
# check_rt_setup.sh의 사용자 표시와 BIOS HT off FAIL 판정에만 사용된다.
#
# 테스트 훅: $RTC_SYSFS_ROOT (default: /sys), $RTC_PROC_CPUINFO (default: /proc/cpuinfo)
# 환경변수: $RTC_FORCE_HYBRID_GENERATION — 세대 enum만 override (§1.3 결정).
#           id 리스트는 생성하지 않으며 sysfs 감지 결과를 유지한다.

# Expand a sysfs cpulist ("0-7,12-15" or "0,2,4") to space-separated ids.
_rt_parse_cpulist() {
  local raw="$1"
  [[ -z "$raw" ]] && return 0
  local out=""
  local part
  IFS=',' read -ra parts <<<"$raw"
  for part in "${parts[@]}"; do
    part="${part// /}"
    [[ -z "$part" ]] && continue
    if [[ "$part" == *-* ]]; then
      local a="${part%-*}"
      local b="${part#*-}"
      local i
      for ((i=a; i<=b; i++)); do out="${out} ${i}"; done
    else
      out="${out} ${part}"
    fi
  done
  # Trim leading space
  echo "${out# }"
}

# Read first non-empty line of a file, trimmed. Echoes nothing on failure.
_rt_read_trim() {
  local p="$1"
  [[ -r "$p" ]] || return 0
  local v
  v=$(head -n1 -- "$p" 2>/dev/null)
  # strip CR/whitespace
  v="${v//$'\r'/}"
  v="${v// /}"
  v="${v//	/}"
  echo "$v"
}

# Try <dir>/cpus, then <dir>/cpulist. Different kernels expose different names.
_rt_read_cpulist_file() {
  local dir="$1"
  local raw
  raw=$(_rt_read_trim "$dir/cpus")
  if [[ -z "$raw" ]]; then
    raw=$(_rt_read_trim "$dir/cpulist")
  fi
  _rt_parse_cpulist "$raw"
}

# Check whether /proc/cpuinfo (or $RTC_PROC_CPUINFO) exposes the "hybrid" flag.
_rt_cpuinfo_has_hybrid() {
  local p="${RTC_PROC_CPUINFO:-/proc/cpuinfo}"
  [[ -r "$p" ]] || return 1
  # Match the flag at a word boundary in a "flags" line.
  awk '/^flags/ {
    for (i=1; i<=NF; ++i) if ($i == "hybrid") { exit 0 }
    exit 1
  }' "$p"
}

# Read vendor_id / cpu family / model from /proc/cpuinfo. Only the first
# processor block is parsed — all logical CPUs share family/model. Sets:
#   _RT_CPU_VENDOR  — "GenuineIntel" / "AuthenticAMD" / "" (unknown)
#   _RT_CPU_FAMILY  — integer (0 if unknown or non-numeric)
#   _RT_CPU_MODEL   — integer (0 if unknown or non-numeric)
# Tolerant: missing fields stay at defaults so existing mocks (which omit
# family/model lines) continue to round-trip.
_rt_read_cpu_vendor_family_model() {
  _RT_CPU_VENDOR=""
  _RT_CPU_FAMILY=0
  _RT_CPU_MODEL=0
  local p="${RTC_PROC_CPUINFO:-/proc/cpuinfo}"
  [[ -r "$p" ]] || return 0
  local vendor family model
  vendor=$(awk -F':' '/^vendor_id[ \t]*:/ {
    gsub(/^[ \t]+|[ \t]+$/, "", $2); print $2; exit
  }' "$p")
  family=$(awk -F':' '/^cpu family[ \t]*:/ {
    gsub(/^[ \t]+|[ \t]+$/, "", $2); print $2; exit
  }' "$p")
  # "model" line — distinguish from "model name" by requiring optional
  # whitespace then ":" immediately after "model".
  model=$(awk -F':' '/^model[ \t]*:/ {
    gsub(/^[ \t]+|[ \t]+$/, "", $2); print $2; exit
  }' "$p")
  _RT_CPU_VENDOR="${vendor:-}"
  # Use `if` blocks (not `[[ ]] && cmd`) so a missing field doesn't make the
  # function return non-zero under `set -e` in callers.
  if [[ "$family" =~ ^[0-9]+$ ]]; then _RT_CPU_FAMILY="$family"; fi
  if [[ "$model"  =~ ^[0-9]+$ ]]; then _RT_CPU_MODEL="$model"; fi
  return 0
}

# Lookup human-friendly platform label and "no-HT-by-design" flag from
# Intel CPUID family.model. Maps modern Intel hybrid silicon to its
# silicon family + form factor, so callers can distinguish e.g.
# Raptor Lake-S desktop (i9-13900K, model 0xBF) from Raptor Lake-P mobile
# (NUC 13 Pro, model 0xBA) — they share the (P-HT, no LP-E) topology
# fingerprint and would otherwise be indistinguishable.
#
# SSOT: Linux kernel arch/x86/include/asm/intel-family.h
#   https://github.com/torvalds/linux/blob/master/arch/x86/include/asm/intel-family.h
#
# Inputs:  $1 = cpu family (int), $2 = cpu model (int)
# Outputs (globals):
#   _RT_PLATFORM_LABEL           — human-friendly string, "" if unknown
#   _RT_PLATFORM_NO_HT_BY_DESIGN — 1 if silicon design lacks Hyper-Threading
#                                  (Lion Cove cores: Arrow/Lunar Lake);
#                                  0 otherwise. Used by check_rt_setup.sh to
#                                  distinguish "BIOS disabled HT" (FAIL) from
#                                  "silicon has no HT to begin with" (PASS).
_rt_lookup_platform_label() {
  _RT_PLATFORM_LABEL=""
  _RT_PLATFORM_NO_HT_BY_DESIGN=0
  local family="$1" model="$2"
  [[ "$family" == "6" ]] || return 0
  case "$model" in
    151) _RT_PLATFORM_LABEL="Alder Lake-S desktop" ;;        # 0x97
    154) _RT_PLATFORM_LABEL="Alder Lake-P mobile" ;;         # 0x9A
    190) _RT_PLATFORM_LABEL="Alder Lake-N (Atom-only)" ;;    # 0xBE
    183) _RT_PLATFORM_LABEL="Raptor Lake" ;;                 # 0xB7 (base alias)
    186) _RT_PLATFORM_LABEL="Raptor Lake-P mobile" ;;        # 0xBA (NUC 13 Pro)
    191) _RT_PLATFORM_LABEL="Raptor Lake-S desktop" ;;       # 0xBF (i9-13900K)
    170) _RT_PLATFORM_LABEL="Meteor Lake-L mobile" ;;        # 0xAA (NUC 14 Pro)
    172) _RT_PLATFORM_LABEL="Meteor Lake-M mobile" ;;        # 0xAC
    189) _RT_PLATFORM_LABEL="Lunar Lake mobile"
         _RT_PLATFORM_NO_HT_BY_DESIGN=1 ;;                   # 0xBD
    198) _RT_PLATFORM_LABEL="Arrow Lake-S desktop"
         _RT_PLATFORM_NO_HT_BY_DESIGN=1 ;;                   # 0xC6
    197) _RT_PLATFORM_LABEL="Arrow Lake-H mobile"
         _RT_PLATFORM_NO_HT_BY_DESIGN=1 ;;                   # 0xC5
    181) _RT_PLATFORM_LABEL="Arrow Lake-U mobile"
         _RT_PLATFORM_NO_HT_BY_DESIGN=1 ;;                   # 0xB5
    *)   _RT_PLATFORM_LABEL="" ;;
  esac
}

# Enumerate online logical CPU ids by scanning cpuN directories under sysfs.
# Output: space-separated ids in numeric order. Empty on failure.
_rt_enumerate_online_cpus() {
  local cpu_root="$1"
  local cpu_dir cpu out=""
  for cpu_dir in "$cpu_root"/cpu[0-9]*; do
    [[ -d "$cpu_dir" ]] || continue
    cpu="${cpu_dir##*/cpu}"
    [[ "$cpu" =~ ^[0-9]+$ ]] || continue
    out="${out} ${cpu}"
  done
  # Numeric sort for determinism.
  echo "$out" | tr ' ' '\n' | sed '/^$/d' | sort -n | tr '\n' ' ' | sed 's/ $//'
}

# ─── Hybrid-detection fallback helpers ──────────────────────────────────────
# When the primary sysfs path (`types/intel_core`, `types/intel_atom` +
# /proc/cpuinfo `hybrid` flag) fails — as happens on older RT kernels and on
# custom builds where the hybrid topology is not exported — these fallbacks
# try to classify P vs E cores from information that is always exposed.
#
#   1. CPUID leaf 0x1A (`cpuid` tool, per-CPU via `taskset`).
#      Most authoritative: reads the Intel "Native Model ID Enumeration"
#      leaf directly. Requires `cpuid` and `taskset` on PATH and leaf 0x1A
#      returning a non-zero EAX.
#      EAX[31:24]: 0x40 = Intel Core (P), 0x20 = Intel Atom (E/LP-E).
#
#   2. `cpuinfo_max_freq` clustering (zero-dependency, sysfs only).
#      On every hybrid x86 Intel SoC, the per-CPU `cpuinfo_max_freq` differs
#      between P / E / LP-E tiers by a wide margin (20 %–50 %). If spread
#      >= 15 % and we can form non-empty P/E sets, treat as hybrid.
#
# Both helpers return the tentative `p_cpus` / `e_cpus` cpulists via the
# globals `_RT_FB_P_CPUS` / `_RT_FB_E_CPUS`. The shared population step
# (`_rt_populate_hybrid_from_cpus`) then derives SMT siblings and E / LP-E
# split by reusing the sysfs topology and cpufreq files that *are* present.

# Fallback 1: CPUID leaf 0x1A.
# Returns 0 on success, sets _RT_FB_P_CPUS / _RT_FB_E_CPUS.
# Returns 1 (and leaves globals empty) if cpuid / taskset are missing, the
# leaf is unsupported, or the split is homogeneous.
_rt_detect_hybrid_via_cpuid() {
  _RT_FB_P_CPUS=""
  _RT_FB_E_CPUS=""
  command -v cpuid   >/dev/null 2>&1 || return 1
  command -v taskset >/dev/null 2>&1 || return 1

  local cpu_root="${RTC_SYSFS_ROOT:-/sys}/devices/system/cpu"
  local cpus
  cpus=$(_rt_enumerate_online_cpus "$cpu_root")
  [[ -z "$cpus" ]] && return 1

  local cpu line eax hex core_type
  local p_list="" e_list=""
  for cpu in $cpus; do
    # Extract leaf 0x1A subleaf 0x00 for this specific CPU. The raw line
    # format is e.g. "   0x0000001a 0x00: eax=0x40000002 ebx=0x00000000 ...".
    # `timeout 1` caps per-CPU wait — observed in field that `cpuid` can block
    # reading `/dev/cpu/N/cpuid` on certain kernel+CPU combinations, freezing
    # the entire fallback. On timeout, $line is empty and we fall through to
    # the freq-clustering path below.
    line=$(timeout 1 taskset -c "$cpu" cpuid -r -1 -l 0x1a 2>/dev/null \
           | awk '/0x0000001a 0x00:/ {print; exit}')
    [[ -z "$line" ]] && return 1
    hex=$(echo "$line" | grep -oE 'eax=0x[0-9a-fA-F]+' | head -1 | cut -d= -f2)
    [[ -z "$hex" ]] && return 1
    # Bash arithmetic can eat the hex directly.
    eax=$(( hex ))
    # Leaf 0x1A returns EAX=0 when the CPU does not support hybrid reporting.
    (( eax == 0 )) && return 1
    core_type=$(( (eax >> 24) & 0xFF ))
    case "$core_type" in
      64)  p_list="${p_list} ${cpu}" ;;   # 0x40 → Intel Core (P)
      32)  e_list="${e_list} ${cpu}" ;;   # 0x20 → Intel Atom (E/LP-E)
      *)   return 1 ;;                    # Unknown vendor-extension value
    esac
  done

  # Only claim hybrid if both tiers are present. Homogeneous P-only or
  # E-only silicon (e.g., Alder-Lake-N) stays "not hybrid".
  [[ -z "$p_list" || -z "$e_list" ]] && return 1

  _RT_FB_P_CPUS="${p_list# }"
  _RT_FB_E_CPUS="${e_list# }"
  return 0
}

# Fallback 2: cpuinfo_max_freq clustering.
# Returns 0 on success, sets _RT_FB_P_CPUS / _RT_FB_E_CPUS.
# Threshold: P-cores are those with max_freq >= 85 % of system-wide max.
# Guard: demands >= 15 % spread between min and max — filters out AMD /
# homogeneous Intel whose per-core freqs are within a percent of each other.
_rt_detect_hybrid_via_freq() {
  _RT_FB_P_CPUS=""
  _RT_FB_E_CPUS=""
  local cpu_root="${RTC_SYSFS_ROOT:-/sys}/devices/system/cpu"
  local cpus
  cpus=$(_rt_enumerate_online_cpus "$cpu_root")
  [[ -z "$cpus" ]] && return 1

  local cpu f max_f=0 min_f=0 missing=0
  local -A freqs=()
  for cpu in $cpus; do
    f=$(_rt_read_trim "$cpu_root/cpu${cpu}/cpufreq/cpuinfo_max_freq")
    if [[ -z "$f" ]]; then
      missing=1
      break
    fi
    freqs[$cpu]=$f
    (( f > max_f )) && max_f=$f
    if (( min_f == 0 )) || (( f < min_f )); then min_f=$f; fi
  done
  (( missing == 1 )) && return 1
  (( max_f == 0 ))   && return 1

  local spread_pct=$(( (max_f - min_f) * 100 / max_f ))
  (( spread_pct < 15 )) && return 1   # Homogeneous silicon.

  local p_threshold=$(( max_f * 85 / 100 ))
  local p_list="" e_list=""
  for cpu in $cpus; do
    f=${freqs[$cpu]}
    if (( f >= p_threshold )); then
      p_list="${p_list} ${cpu}"
    else
      e_list="${e_list} ${cpu}"
    fi
  done

  [[ -z "$p_list" || -z "$e_list" ]] && return 1
  _RT_FB_P_CPUS="${p_list# }"
  _RT_FB_E_CPUS="${e_list# }"
  return 0
}

# Derive NUM_*, P_CORE_*, E_CORE_IDS, LPE_CORE_IDS, HAS_LP_E_CORES,
# P_CORE_HAS_SMT from a (p_cpus, e_cpus) split. Extracted from the original
# body of detect_hybrid_capability so all three detection paths can share it.
_rt_populate_hybrid_from_cpus() {
  local cpu_root="$1"
  local p_cpus="$2"
  local e_cpus="$3"

  NUM_P_PHYSICAL=0
  NUM_P_LOGICAL=0
  NUM_E_CORES=0
  NUM_LPE_CORES=0
  P_CORE_PHYSICAL_IDS=""
  P_CORE_SIBLING_IDS=""
  E_CORE_IDS=""
  LPE_CORE_IDS=""
  P_CORE_HAS_SMT=0
  HAS_LP_E_CORES=0

  [[ -z "$p_cpus" ]] && return 0

  # Group P-cores by topology/core_id; within each group, the lowest logical
  # id is the "physical" representative and the second is the SMT sibling.
  local -A core_to_cpus=()
  local cpu cid
  for cpu in $p_cpus; do
    cid=$(_rt_read_trim "$cpu_root/cpu${cpu}/topology/core_id")
    [[ -z "$cid" ]] && continue
    if [[ -z "${core_to_cpus[$cid]+x}" ]]; then
      core_to_cpus[$cid]="$cpu"
    else
      core_to_cpus[$cid]="${core_to_cpus[$cid]} $cpu"
    fi
  done

  # Build (physical, sibling) pairs from each core_id group, then emit the list
  # sorted by *physical logical id ascending*. Without this outer sort, BIOSes
  # that assign a non-monotonic core_id to cpu 0 (e.g. some NUC 14 Pro Meteor
  # Lake firmware: cpu 0 shares core_id with cpu 5, so its group lands in the
  # middle of the iteration) produce a P-physical list like [1, 3, 0, 6, 8, 10]
  # — and the v4.1 layout assumption "slot 0 == lowest logical cpu == OS"
  # breaks. Sorting by physical restores the invariant on every enumeration
  # order, while keeping each (physical, sibling) pair contiguous.
  local pairs="" cid cpus sorted_cpus first second
  for cid in "${!core_to_cpus[@]}"; do
    cpus="${core_to_cpus[$cid]}"
    sorted_cpus=$(printf '%s\n' $cpus | sort -n | tr '\n' ' ')
    read -r first second _ <<<"$sorted_cpus"
    pairs+="${first} ${second:--1}"$'\n'
    NUM_P_PHYSICAL=$((NUM_P_PHYSICAL + 1))
  done
  local sorted_pairs
  sorted_pairs=$(printf '%s' "$pairs" | sort -n -k1,1)
  while IFS= read -r line; do
    [[ -z "$line" ]] && continue
    read -r first second _ <<<"$line"
    P_CORE_PHYSICAL_IDS="${P_CORE_PHYSICAL_IDS} ${first}"
    if [[ -n "$second" && "$second" != "-1" ]]; then
      P_CORE_SIBLING_IDS="${P_CORE_SIBLING_IDS} ${second}"
    fi
  done <<<"$sorted_pairs"
  P_CORE_PHYSICAL_IDS="${P_CORE_PHYSICAL_IDS# }"
  P_CORE_SIBLING_IDS="${P_CORE_SIBLING_IDS# }"

  NUM_P_LOGICAL=$(echo "$p_cpus" | wc -w)
  local num_siblings
  num_siblings=$(echo "$P_CORE_SIBLING_IDS" | wc -w)
  if (( NUM_P_LOGICAL > NUM_P_PHYSICAL && num_siblings == NUM_P_PHYSICAL )); then
    P_CORE_HAS_SMT=1
  fi

  # E-core vs LP-E-core split via cpuinfo_max_freq 70 % rule.
  if [[ -n "$e_cpus" ]]; then
    local max_freq=0
    local -A e_freq=()
    local f
    for cpu in $e_cpus; do
      f=$(_rt_read_trim "$cpu_root/cpu${cpu}/cpufreq/cpuinfo_max_freq")
      [[ -z "$f" ]] && f=0
      e_freq[$cpu]=$f
      if (( f > max_freq )); then max_freq=$f; fi
    done
    local threshold=0
    if (( max_freq > 0 )); then
      threshold=$(( max_freq * 70 / 100 ))
    fi
    for cpu in $e_cpus; do
      f=${e_freq[$cpu]:-0}
      if (( threshold > 0 && f > 0 && f < threshold )); then
        LPE_CORE_IDS="${LPE_CORE_IDS} ${cpu}"
      else
        E_CORE_IDS="${E_CORE_IDS} ${cpu}"
      fi
    done
    # Sort ascending — non-standard BIOS enumerations may emit e_cpus in
    # non-monotonic order. Logical-id ascending gives a stable mapping
    # consumers (PHYSICAL_CORE_SLOTS, check_rt_setup display) can rely on.
    E_CORE_IDS=$(printf '%s\n' $E_CORE_IDS | sort -n | tr '\n' ' ')
    LPE_CORE_IDS=$(printf '%s\n' $LPE_CORE_IDS | sort -n | tr '\n' ' ')
    E_CORE_IDS="${E_CORE_IDS% }"
    LPE_CORE_IDS="${LPE_CORE_IDS% }"
    NUM_E_CORES=$(echo "$E_CORE_IDS" | wc -w)
    NUM_LPE_CORES=$(echo "$LPE_CORE_IDS" | wc -w)
    if (( NUM_LPE_CORES > 0 )); then HAS_LP_E_CORES=1; fi
  fi
}

# Populate PHYSICAL_CORE_SLOTS — ordered first-logical-id of every unique
# physical core. Mirrors CpuTopology::physical_core_slots (cpu_topology.hpp).
# Sequence:
#   hybrid     → P-physical → E-core → LP-E (already split by sysfs/freq path)
#   non-hybrid → first logical of each (pkg, core_id) group, cpu ascending
# Consumed by RT thread-affinity translation (ApplyThreadConfig in C++).
# Excludes SMT siblings so RT thread pinning by slot index never lands on a
# P-core's hyperthread.
_rt_populate_physical_core_slots() {
  PHYSICAL_CORE_SLOTS=""
  local cpu_root="$1"

  if (( IS_HYBRID == 1 )); then
    local slots=""
    [[ -n "$P_CORE_PHYSICAL_IDS" ]] && slots="${P_CORE_PHYSICAL_IDS}"
    [[ -n "$E_CORE_IDS" ]]         && slots="${slots:+$slots }${E_CORE_IDS}"
    [[ -n "$LPE_CORE_IDS" ]]       && slots="${slots:+$slots }${LPE_CORE_IDS}"
    PHYSICAL_CORE_SLOTS="$slots"
    return 0
  fi

  # Non-hybrid (AMD SMT, SMT-off Intel, container). Walk cpus ascending and
  # record the first cpu that introduces each unique (pkg, core_id) pair —
  # that cpu is the "primary" of its physical core (sibling, if any, has
  # a higher logical id and is skipped).
  local cpu_dir cpu pkg core key
  declare -A _seen_keys=()
  local sorted_cpu_dirs
  sorted_cpu_dirs=$(ls -d "$cpu_root"/cpu[0-9]* 2>/dev/null | sort -V)
  while IFS= read -r cpu_dir; do
    [[ -d "$cpu_dir" ]] || continue
    cpu="${cpu_dir##*/cpu}"
    [[ "$cpu" =~ ^[0-9]+$ ]] || continue
    pkg=$(_rt_read_trim "$cpu_dir/topology/physical_package_id")
    core=$(_rt_read_trim "$cpu_dir/topology/core_id")
    [[ -z "$pkg" || -z "$core" ]] && continue
    key="${pkg}_${core}"
    if [[ -z "${_seen_keys[$key]+x}" ]]; then
      _seen_keys[$key]=1
      PHYSICAL_CORE_SLOTS="${PHYSICAL_CORE_SLOTS:+$PHYSICAL_CORE_SLOTS }${cpu}"
    fi
  done <<<"$sorted_cpu_dirs"
  return 0
}

# Sanity hook: when enabled (RTC_HYBRID_SANITY=1), cross-check the primary
# detection result against the freq-clustering fallback. Emits a single
# stderr warning if they disagree on the P/E split. Does not alter globals
# — this is diagnostic only. Intended for CI / staging; off by default so
# production boots stay quiet.
_rt_hybrid_sanity_check() {
  local source="$1" p_primary="$2" e_primary="$3"
  local p_fb e_fb
  # Save the freq-fallback outputs so we can re-run without clobbering the
  # canonical _RT_FB_* globals (the primary path may have filled them).
  _RT_FB_P_CPUS=""; _RT_FB_E_CPUS=""
  if ! _rt_detect_hybrid_via_freq; then
    return 0   # Freq-clustering cannot form an opinion — nothing to check.
  fi
  p_fb="$_RT_FB_P_CPUS"
  e_fb="$_RT_FB_E_CPUS"

  local p_sorted_primary p_sorted_fb
  p_sorted_primary=$(echo "$p_primary" | tr ' ' '\n' | sort -n | tr '\n' ' ')
  p_sorted_fb=$(echo "$p_fb" | tr ' ' '\n' | sort -n | tr '\n' ' ')
  if [[ "$p_sorted_primary" != "$p_sorted_fb" ]]; then
    {
      echo "[rt_common] hybrid sanity: P-core sets disagree"
      echo "[rt_common]   primary(${source}):   ${p_sorted_primary% }"
      echo "[rt_common]   freq_cluster:         ${p_sorted_fb% }"
      echo "[rt_common]   set RTC_HYBRID_SANITY=0 to silence"
    } >&2
  fi
}

# Populate IS_HYBRID, NUM_*, NUC_GENERATION, id lists. Idempotent — callers may
# re-invoke; later calls overwrite globals. Uses $RTC_SYSFS_ROOT (default /sys).
#
# Detection priority (matches cpu_topology.hpp):
#   1. sysfs `types/intel_core` + `types/intel_atom` (+ optional cpuinfo
#      `hybrid` flag). Requires kernel topology exposure that some custom
#      RT kernel builds strip out.
#   2. CPUID leaf 0x1A via the `cpuid` tool.
#   3. cpuinfo_max_freq clustering (>= 15 % spread).
# First match wins. HYBRID_DETECT_SOURCE records which path populated the
# globals ("sysfs_types" / "cpuid_0x1a" / "cpufreq_cluster" / "none").
detect_hybrid_capability() {
  local root="${RTC_SYSFS_ROOT:-/sys}"
  local cpu_root="$root/devices/system/cpu"

  IS_HYBRID=0
  P_CORE_HAS_SMT=0
  HAS_LP_E_CORES=0
  NUM_P_PHYSICAL=0
  NUM_P_LOGICAL=0
  NUM_E_CORES=0
  NUM_LPE_CORES=0
  P_CORE_PHYSICAL_IDS=""
  P_CORE_SIBLING_IDS=""
  E_CORE_IDS=""
  LPE_CORE_IDS=""
  PHYSICAL_CORE_SLOTS=""
  NUC_GENERATION="none"
  HYBRID_DETECT_SOURCE="none"

  # ── CPU identity (vendor / family / model + platform label) ──────────────
  # Populated independently of hybrid detection: even non-hybrid Intel and
  # AMD chips get vendor + family + model exposed so callers can render a
  # meaningful identifier. PLATFORM_LABEL stays "" on unknown silicon.
  _rt_read_cpu_vendor_family_model
  CPU_VENDOR="$_RT_CPU_VENDOR"
  CPU_FAMILY="$_RT_CPU_FAMILY"
  CPU_MODEL="$_RT_CPU_MODEL"
  _rt_lookup_platform_label "$CPU_FAMILY" "$CPU_MODEL"
  PLATFORM_LABEL="$_RT_PLATFORM_LABEL"
  PLATFORM_NO_HT_BY_DESIGN="$_RT_PLATFORM_NO_HT_BY_DESIGN"

  local p_cpus="" e_cpus=""

  # ── Primary: sysfs types + cpuinfo hybrid flag ───────────────────────────
  local sysfs_p sysfs_e
  sysfs_p=$(_rt_read_cpulist_file "$cpu_root/types/intel_core")
  sysfs_e=$(_rt_read_cpulist_file "$cpu_root/types/intel_atom")
  local cpuinfo_hybrid=0
  if _rt_cpuinfo_has_hybrid; then cpuinfo_hybrid=1; fi
  local types_present=0
  [[ -n "$sysfs_p" || -n "$sysfs_e" ]] && types_present=1

  if [[ -n "$sysfs_p" && -n "$sysfs_e" ]] \
     || (( types_present == 1 && cpuinfo_hybrid == 1 )); then
    IS_HYBRID=1
    p_cpus="$sysfs_p"
    e_cpus="$sysfs_e"
    HYBRID_DETECT_SOURCE="sysfs_types"
  fi

  # ── Fallback 1: CPUID leaf 0x1A ──────────────────────────────────────────
  if (( IS_HYBRID == 0 )); then
    if _rt_detect_hybrid_via_cpuid; then
      IS_HYBRID=1
      p_cpus="$_RT_FB_P_CPUS"
      e_cpus="$_RT_FB_E_CPUS"
      HYBRID_DETECT_SOURCE="cpuid_0x1a"
    fi
  fi

  # ── Fallback 2: cpuinfo_max_freq clustering ──────────────────────────────
  if (( IS_HYBRID == 0 )); then
    if _rt_detect_hybrid_via_freq; then
      IS_HYBRID=1
      p_cpus="$_RT_FB_P_CPUS"
      e_cpus="$_RT_FB_E_CPUS"
      HYBRID_DETECT_SOURCE="cpufreq_cluster"
    fi
  fi

  # ── Derive detailed fields from whichever (p_cpus, e_cpus) split won ────
  if (( IS_HYBRID == 1 )); then
    _rt_populate_hybrid_from_cpus "$cpu_root" "$p_cpus" "$e_cpus"
  fi

  # ── Optional cross-check when RTC_HYBRID_SANITY=1 ───────────────────────
  # Compares the primary split to the freq-clustering split. Diagnostic
  # only — used by CI/staging; off by default.
  if (( IS_HYBRID == 1 )) && [[ "${RTC_HYBRID_SANITY:-0}" == "1" ]]; then
    _rt_hybrid_sanity_check "$HYBRID_DETECT_SOURCE" "$p_cpus" "$e_cpus"
  fi

  # Classify generation from (is_hybrid, p_core_has_smt, has_lp_e_cores).
  if (( IS_HYBRID == 0 )); then
    NUC_GENERATION="none"
  elif (( P_CORE_HAS_SMT == 0 && HAS_LP_E_CORES == 1 )); then
    NUC_GENERATION="arrow_lake_h"
  elif (( P_CORE_HAS_SMT == 1 && HAS_LP_E_CORES == 1 )); then
    NUC_GENERATION="meteor_lake"
  elif (( P_CORE_HAS_SMT == 1 && HAS_LP_E_CORES == 0 )); then
    NUC_GENERATION="raptor_lake_p"
  else
    NUC_GENERATION="raptor_lake_p_ht_off"
  fi

  # Env-var hint override — generation enum only; id lists untouched.
  case "${RTC_FORCE_HYBRID_GENERATION:-}" in
    raptor_lake_p|meteor_lake|arrow_lake_h|raptor_lake_p_ht_off|none)
      NUC_GENERATION="$RTC_FORCE_HYBRID_GENERATION"
      ;;
  esac

  # Populate slot-index→logical-id mapping for RT thread placement.
  _rt_populate_physical_core_slots "$cpu_root"
}

# Thin accessors — call detect_hybrid_capability once first, or rely on
# existing globals (these helpers do not re-detect to keep callers fast).
get_nuc_generation()      { echo "${NUC_GENERATION:-none}"; }
get_p_core_physical_ids() { echo "${P_CORE_PHYSICAL_IDS:-}"; }
get_p_core_sibling_ids()  { echo "${P_CORE_SIBLING_IDS:-}"; }
get_e_core_ids()          { echo "${E_CORE_IDS:-}"; }
get_lpe_core_ids()        { echo "${LPE_CORE_IDS:-}"; }
get_hybrid_detect_source() { echo "${HYBRID_DETECT_SOURCE:-none}"; }
get_cpu_vendor()          { echo "${CPU_VENDOR:-}"; }
get_cpu_family()          { echo "${CPU_FAMILY:-0}"; }
get_cpu_model()           { echo "${CPU_MODEL:-0}"; }
get_platform_label()      { echo "${PLATFORM_LABEL:-}"; }
get_platform_no_ht_by_design() { echo "${PLATFORM_NO_HT_BY_DESIGN:-0}"; }
get_physical_core_slots() { echo "${PHYSICAL_CORE_SLOTS:-}"; }
