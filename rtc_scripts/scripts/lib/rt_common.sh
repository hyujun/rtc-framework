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
  echo "rtc_msgs rtc_base rtc_communication rtc_controller_interface rtc_urdf_bridge rtc_mpc rtc_tsid rtc_controllers rtc_controller_manager rtc_inference rtc_scripts rtc_tools shape_estimation_msgs shape_estimation"
}

get_robot_packages() {
  echo "ur5e_description ur5e_hand_driver ur5e_bringup ur5e_bt_coordinator"
}

# ── MPC core layout helpers (Phase 5 + unified 10/12/14 rework) ──────────────
# Single source of truth for MPC thread core assignment. Must stay in sync
# with rtc_base/threading/thread_config.hpp (SelectThreadConfigs dispatch).
#
# Layout policy (unified — low-numbered RT cores in every 10+ tier):
#   ≤4 cores  → MPC on Core 3, SCHED_OTHER (degraded).
#   5-9      → Core 4 dedicated to MPC main (FIFO 60).
#   10-11    → Core 4 main + Core 5 worker 0.
#   12-13    → Core 4 main + Core 5-6 workers (2 workers).
#   14-15    → Same as 12-13; Core 10 is dedicated MuJoCo sim.
#   16+      → Core 9 main + Core 10-11 workers (legacy Option A layout).
#
# Prints a comma-separated list of cores. First entry is always the MPC
# main thread's core.
get_mpc_cores() {
  local ncpu
  ncpu=$(get_physical_cores)
  case "$ncpu" in
    1|2|3|4)      echo "3" ;;
    5|6|7)        echo "4" ;;
    8|9)          echo "4" ;;
    10|11)        echo "4,5" ;;
    12|13)        echo "4,5,6" ;;
    14|15)        echo "4,5,6" ;;
    *)            echo "9,10,11" ;;
  esac
}

# Print just the main MPC core (first entry of get_mpc_cores).
get_mpc_main_core() {
  get_mpc_cores | cut -d',' -f1
}

# Print the list of RT cores (rt_control + sensor_io + udp_recv + MPC).
# Used by IRQ affinity and GRUB nohz_full/rcu_nocbs. Order is not guaranteed.
get_rt_cores() {
  local ncpu
  ncpu=$(get_physical_cores)
  local mpc
  mpc=$(get_mpc_cores)
  # rt_control + sensor_io + udp_recv cores — match thread_config.hpp.
  case "$ncpu" in
    1|2|3|4)      echo "1,2,${mpc}" ;;
    5|6|7)        echo "2,3,5,${mpc}" ;;     # RT=2, sensor=3, udp=5
    8|9)          echo "2,3,5,${mpc}" ;;     # RT=2, sensor=3, udp=5
    10|11)        echo "2,3,6,${mpc}" ;;     # RT=2, sensor=3, udp=6, MPC=4,5
    12|13)        echo "2,3,7,${mpc}" ;;     # RT=2, sensor=3, udp=7, MPC=4,5,6
    14|15)        echo "2,3,7,${mpc}" ;;     # RT=2, sensor=3, udp=7, MPC=4,5,6
    *)            echo "2,3,12,${mpc}" ;;    # 16+: RT=2, sensor=3, udp=12, MPC=9-11
  esac
}

# Print the list of OS cores (complement of get_rt_cores).
get_os_cores() {
  local ncpu
  ncpu=$(get_physical_cores)
  if [[ "$ncpu" -le 4 ]]; then
    echo "0"
  else
    echo "0,1"
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

# Populate IS_HYBRID, NUM_*, NUC_GENERATION, id lists. Idempotent — callers may
# re-invoke; later calls overwrite globals. Uses $RTC_SYSFS_ROOT (default /sys).
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
  NUC_GENERATION="none"

  local p_cpus e_cpus
  p_cpus=$(_rt_read_cpulist_file "$cpu_root/types/intel_core")
  e_cpus=$(_rt_read_cpulist_file "$cpu_root/types/intel_atom")

  local cpuinfo_hybrid=0
  if _rt_cpuinfo_has_hybrid; then cpuinfo_hybrid=1; fi

  # types/intel_{core,atom} 동시 존재 or (존재 + cpuinfo hybrid flag) → hybrid.
  local types_present=0
  if [[ -n "$p_cpus" || -n "$e_cpus" ]]; then types_present=1; fi
  if [[ -n "$p_cpus" && -n "$e_cpus" ]]; then IS_HYBRID=1; fi
  if (( types_present == 1 && cpuinfo_hybrid == 1 )); then IS_HYBRID=1; fi

  if (( IS_HYBRID == 1 )) && [[ -n "$p_cpus" ]]; then
    # Group P-cores by core_id. Produce "core_id:cpu" pairs, sort, unique by core_id.
    local -A core_to_cpus=()
    local cpu
    for cpu in $p_cpus; do
      local cid
      cid=$(_rt_read_trim "$cpu_root/cpu${cpu}/topology/core_id")
      [[ -z "$cid" ]] && continue
      if [[ -z "${core_to_cpus[$cid]+x}" ]]; then
        core_to_cpus[$cid]="$cpu"
      else
        core_to_cpus[$cid]="${core_to_cpus[$cid]} $cpu"
      fi
    done

    # Sort core_ids numerically for deterministic ordering.
    local sorted_cids
    sorted_cids=$(printf '%s\n' "${!core_to_cpus[@]}" | sort -n)
    NUM_P_PHYSICAL=0
    local cid cpus sorted_cpus first second
    while IFS= read -r cid; do
      [[ -z "$cid" ]] && continue
      cpus="${core_to_cpus[$cid]}"
      # Sort logical cpus within this physical core numerically.
      sorted_cpus=$(printf '%s\n' $cpus | sort -n | tr '\n' ' ')
      read -r first second _ <<<"$sorted_cpus"
      P_CORE_PHYSICAL_IDS="${P_CORE_PHYSICAL_IDS} ${first}"
      if [[ -n "$second" ]]; then
        P_CORE_SIBLING_IDS="${P_CORE_SIBLING_IDS} ${second}"
      fi
      NUM_P_PHYSICAL=$((NUM_P_PHYSICAL + 1))
    done <<<"$sorted_cids"
    P_CORE_PHYSICAL_IDS="${P_CORE_PHYSICAL_IDS# }"
    P_CORE_SIBLING_IDS="${P_CORE_SIBLING_IDS# }"

    NUM_P_LOGICAL=$(echo "$p_cpus" | wc -w)
    local num_siblings
    num_siblings=$(echo "$P_CORE_SIBLING_IDS" | wc -w)
    if (( NUM_P_LOGICAL > NUM_P_PHYSICAL && num_siblings == NUM_P_PHYSICAL )); then
      P_CORE_HAS_SMT=1
    fi

    # E-core / LP-E split via cpuinfo_max_freq 70% rule.
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
      E_CORE_IDS="${E_CORE_IDS# }"
      LPE_CORE_IDS="${LPE_CORE_IDS# }"
      NUM_E_CORES=$(echo "$E_CORE_IDS" | wc -w)
      NUM_LPE_CORES=$(echo "$LPE_CORE_IDS" | wc -w)
      if (( NUM_LPE_CORES > 0 )); then HAS_LP_E_CORES=1; fi
    fi
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
}

# Thin accessors — call detect_hybrid_capability once first, or rely on
# existing globals (these helpers do not re-detect to keep callers fast).
get_nuc_generation()      { echo "${NUC_GENERATION:-none}"; }
get_p_core_physical_ids() { echo "${P_CORE_PHYSICAL_IDS:-}"; }
get_p_core_sibling_ids()  { echo "${P_CORE_SIBLING_IDS:-}"; }
get_e_core_ids()          { echo "${E_CORE_IDS:-}"; }
get_lpe_core_ids()        { echo "${LPE_CORE_IDS:-}"; }
