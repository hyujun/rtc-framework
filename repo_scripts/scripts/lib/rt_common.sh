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
#   write_file_if_changed     — 멱등 atomic 파일 쓰기 (mktemp + mv, mode 보존)
#   with_temporary_disable    — postinst hook 일시 비활성화 (trap EXIT 복원)
#   auto_release_cpu_shield   — 빌드 전 CPU shield 자동 해제
#   check_workspace_structure — ROS2 워크스페이스 구조 검증
#   ensure_ros2_sourced       — ROS2 환경 자동 탐색 및 소싱
#   create_oneshot_service    — systemd oneshot 서비스 생성 헬퍼
#   lttng_kernel_build_version      — 커널 헤더 Makefile 에서 V.P.S (uname 아님)
#   lttng_modules_min_version_for_kernel — 그 커널이 요구하는 lttng-modules 최소 버전
#   lttng_modules_compat_verdict    — ok|incompatible|unknown 3-값 판정 (issue #190)

# 이 파일은 직접 실행하지 않는다.
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "ERROR: This file should be sourced, not executed." >&2
  exit 1
fi

# ── 중복 source 방지 ───────────────────────────────────────────────────────
[[ -n "${_RT_COMMON_LOADED:-}" ]] && return 0
_RT_COMMON_LOADED=1

# ── Generated CPU layout table ──────────────────────────────────────────────
# tier → slot/policy/priority/nice assignment for every role, generated from
# repo_scripts/config/thread_layout.yaml (issue #153 M1). Provides:
#   get_role_spec / get_role_slot / get_role_policy / get_role_priority /
#   get_role_nice / get_mpc_cores / get_rt_cores / get_nrt_cores /
#   get_arm_driver_slot / get_hand_driver_slot / get_os_cores /
#   rtc_expected_threads
# Never edit the generated file: change the manifest and run
#   python3 repo_scripts/scripts/gen_thread_layout.py --write
# It is pure (core count in, slots out) so install.sh can source it before the
# workspace is built.
# shellcheck source=repo_scripts/scripts/lib/thread_layout_generated.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/thread_layout_generated.sh"

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

# ── Idempotent atomic file write ────────────────────────────────────────────
# 파일 내용이 동일하면 skip, 다르면 백업 후 atomic 으로 덮어쓴다.
#
# Atomicity: same-directory mktemp + mv (rename(2)) — mid-write crash 시
# target 파일은 이전 상태 그대로 유지. systemd/sysctl/grub 등 시스템 설정에 필수.
# 같은 디렉토리에 tmp 를 두는 이유는 rename(2) 가 동일 filesystem 내에서만
# atomic 이기 때문 (cross-fs 는 EXDEV).
#
# Mode preservation: 기존 file 의 mode 를 stat 으로 읽어 보존. 없으면 0644 default.
# 호출처가 write 후 chmod 로 mode 를 조정하는 패턴 (executable script 등) 은 그대로 호환.
#
# Usage: write_file_if_changed "/path/to/file" "$CONTENT" [backup=true]
# Returns: 0 if written, 1 if skipped (already identical)
write_file_if_changed() {
  local file="$1"
  local content="$2"
  local do_backup="${3:-true}"

  if [[ -f "$file" ]] && diff -q <(printf '%s\n' "$content") "$file" &>/dev/null; then
    return 1  # 동일 — skip
  fi

  # 기존 file 의 mode 보존 (없으면 0644)
  local mode="0644"
  if [[ -f "$file" ]]; then
    mode=$(stat -c '%a' "$file" 2>/dev/null || echo "0644")
  fi

  if [[ "$do_backup" == "true" && -f "$file" ]]; then
    cp -p "$file" "${file}.bak.$(date +%Y%m%d_%H%M%S)"
  fi

  # Same-directory tmp 로 atomic 보장. parent 디렉토리는 시스템 경로라 항상 존재.
  local dir tmp
  dir=$(dirname "$file")
  tmp=$(mktemp "${dir}/.$(basename "$file").XXXXXX") || fatal "mktemp failed in ${dir}"

  # printf 로 trailing newline 1개 보장 (systemd/sysctl 관례).
  # tmp 쓰기 실패 시 cleanup.
  if ! printf '%s\n' "$content" > "$tmp"; then
    rm -f "$tmp"
    fatal "Failed to write tmp file ${tmp}"
  fi

  chmod "$mode" "$tmp" 2>/dev/null || true
  mv -f "$tmp" "$file"
  return 0  # 기록됨
}

# ── Temporarily disable executable hook, restore on EXIT ────────────────────
# chmod -x <hook> 후 <command> 를 실행하고, set -e / signal / 정상 종료 어느
# 경로로 빠져나가든 trap EXIT 으로 chmod +x 복원한다.
#
# 배경: dpkg -i 가 /etc/kernel/postinst.d/dkms 를 호출하는데 NVIDIA DKMS 가 RT
# 커널에서 실패하면 dpkg 전체가 실패한다. 이를 우회하려고 hook 을 일시 비활성화
# 해야 하나, dpkg 실패 시 hook 이 영구 비활성화되는 위험이 있다 (set -e + 미복원).
# trap EXIT 으로 무조건 복원 보장.
#
# Usage:
#   with_temporary_disable /etc/kernel/postinst.d/dkms -- dpkg -i pkg1.deb pkg2.deb
#
# Returns: 내부 명령의 exit code (성공/실패 모두 caller 로 propagate)
with_temporary_disable() {
  local hook="$1"; shift
  [[ "$1" == "--" ]] || fatal "with_temporary_disable: '--' separator missing"
  shift

  if [[ ! -f "$hook" ]]; then
    # Hook 자체가 없으면 그냥 명령만 실행
    "$@"
    return $?
  fi

  # 원래 실행 가능 여부 기록 (이미 -x 가 없으면 복원도 -x 추가하지 않음)
  local was_executable=0
  [[ -x "$hook" ]] && was_executable=1

  if [[ "$was_executable" -eq 1 ]]; then
    # trap 은 함수 scope 가 아닌 shell scope 라서 RETURN 으로는 깔끔히 못 잡는다.
    # subshell 로 격리하여 EXIT trap 이 해당 subshell 만 cleanup 하도록 한다.
    (
      # shellcheck disable=SC2154  # rc is assigned inside the trap string (rc=$?)
      trap 'rc=$?; set +e; chmod +x "'"$hook"'" 2>/dev/null; exit $rc' EXIT
      chmod -x "$hook" || exit 1
      "$@"
    )
    return $?
  fi

  # 이미 비활성화 상태였으면 그냥 명령 실행
  "$@"
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
  echo "rtc_msgs rtc_base rtc_math rtc_communication rtc_controller_interface rtc_urdf_bridge rtc_mpc rtc_tsid rtc_controllers rtc_controller_manager rtc_inference repo_scripts rtc_tools robot_descriptions shape_estimation_msgs shape_estimation"
}

get_robot_packages() {
  echo "udp_hand_driver integrated_bringup ur5e_bt_coordinator"
}

# ── MPC / RT / NRT core layout helpers ──────────────────────────────────────
# get_mpc_cores, get_rt_cores, get_nrt_cores, get_arm_driver_slot,
# get_hand_driver_slot and get_os_cores are GENERATED into
# thread_layout_generated.sh (sourced above) from
# repo_scripts/config/thread_layout.yaml, which is also the source of the C++
# tier constants and the Python launch mirror. Each takes an optional physical
# core count as $1 so callers (print_thread_layout, the unit tests) can render a
# tier this machine does not have. Every value is a *slot index*, not a kernel
# logical CPU id -- run it through slot_to_logical_cpu before comparing against
# a /proc affinity mask.

# get_rt_cores() feeds IRQ affinity and GRUB nohz_full/rcu_nocbs. Order is not
# guaranteed. The hand UDP receive thread lives inside the hand_driver process
# (cpu_core = -1 sentinel) and is deliberately not represented there.
#
# ── CPU shield marker (issue #350) ─────────────────────────────────────────
# cpu_shield.sh writes "<mode> <profile>" here on `on`, and removes it on `off`.
# The path lives here rather than in cpu_shield.sh because verify_rt_runtime.sh
# has to read it too, and sourcing cpu_shield.sh would run its CLI dispatch.
RTC_SHIELD_MARKER_FILE="${RTC_SHIELD_MARKER_FILE:-/tmp/cpu_shield_mode}"

# Layout profile of the ACTIVE shield, plus WHERE that answer came from:
# echoes "<profile> <source>". Never fails.
#
# The three fallback cases are not equally well-founded and #386 C is about the
# gap between them:
#
#   marker + valid profile  → evidence. The shield was built for this profile.
#   marker, no profile field → still evidence. A pre-#350 marker means a shield
#       IS up and it was built with the full MPC reservation, which is the
#       default profile. Inference, but grounded in an observed shield.
#   no marker at all         → NOT evidence. It means "no shield is up", which
#       says nothing about whether MPC was configured. Folding it into the
#       default silently turned "unknown" into "mpc_on is confirmed", and a box
#       that legitimately runs without MPC got a hard FAIL for a missing
#       mpc_main. The default is kept (a verifier that refuses to run is worse),
#       but the caller now has the string it needs to say so out loud.
#
# An unparseable profile string also falls back rather than failing: this is a
# diagnostic input, and refusing to verify because a marker got corrupted would
# turn a cosmetic problem into "no RT verification at all".
shield_marker_profile_with_source() {
  local raw mode profile
  if [[ -r "$RTC_SHIELD_MARKER_FILE" ]] && raw=$(cat "$RTC_SHIELD_MARKER_FILE" 2>/dev/null); then
    read -r mode profile <<<"$raw"
    if [[ -n "${profile:-}" ]] && rtc_is_layout_profile "$profile"; then
      printf '%s %s\n' "$profile" "shield marker"
      return 0
    fi
    printf '%s %s\n' "$(rtc_default_profile)" "shield marker (pre-#350, profile 필드 없음)"
    return 0
  fi
  printf '%s %s\n' "$(rtc_default_profile)" "default (shield marker 부재 — profile 미확인)"
}

# Profile only. Kept because most callers do not care where it came from; the
# classification lives in exactly one place so the two answers cannot drift.
shield_marker_profile() {
  local profile _src
  read -r profile _src <<<"$(shield_marker_profile_with_source)"
  echo "$profile"
}

# ── Active-isolation detection (issue #349 D14, moved here by #386) ─────────
#
# Which cores are ACTUALLY held away from the general scheduler, and by which
# mechanism. Two consumers need the same answer and they source different
# files: cpu_shield.sh (stale-shield judgement, do_status) and
# check_rt_setup.sh (the CPU Isolation category). While these lived in
# cpu_shield.sh only the first could reuse them, and the second re-implemented
# the question as "is /sys/devices/system/cpu/isolated non-empty" — which only
# isolcpus ever answers yes to, so a live cset shield read as "no isolation".
# normalise_cpu_set: "2-9,12-13" / "12,13,2-9" / " 2 3 " → canonical sorted
# space-separated ids. Pure string→string, no I/O: this is the part the tests
# drive (test_cpu_shield_mask.sh), because the reader below cannot be tested
# on a box without cset.
#
# Whitespace is folded to commas first because `_rt_parse_cpulist` splits on
# commas only and then *strips* spaces inside each field: " 2 3 " reached it as
# one field and came back as the single id 23. That made this function
# non-idempotent — its own output ("2 3 4 ...") fed back in collapsed to one
# nonexistent cpu id — and made the third example in the line above false. No
# caller passes a space-separated list today; one that did would have compared
# the desired mask against a bogus id, never matched, and reset a shield that
# was already correct on every launch.
normalise_cpu_set() {
  local raw="${1:-}"
  [[ -z "$raw" ]] && { echo ""; return 0; }
  # Collapse any whitespace run to a single comma, then drop the empty fields
  # that leading/trailing whitespace produces.
  raw="${raw//[[:space:]]/,}"
  while [[ "$raw" == *,,* ]]; do raw="${raw//,,/,}"; done
  raw="${raw#,}"
  raw="${raw%,}"
  [[ -z "$raw" ]] && { echo ""; return 0; }
  local ids
  ids=$(_rt_parse_cpulist "$raw") || return 1
  [[ -z "$ids" ]] && { echo ""; return 0; }
  # shellcheck disable=SC2086
  printf '%s\n' $ids | sort -n -u | tr '\n' ' ' | sed 's/ $//'
}

# Best-effort read of the ACTIVE user cpuset. Prints the cpu list on success.
#
# Fails (non-zero, empty output) when cset is absent, no shield is up, or the
# output cannot be parsed. Callers MUST treat that as "unknown", never as
# "matches" — an unreadable mask is exactly the state that produced the stale
# shield in the first place. cset's status format is not stable across
# versions and this repo's dev boxes do not all have cset installed, so the
# cpuset filesystem is tried first: it is the kernel's own record and does not
# depend on how cset chooses to print.
shield_actual_user_cpus() {
  local path
  for path in /sys/fs/cgroup/cpuset/user/cpuset.cpus /sys/fs/cgroup/cpuset/user/cpus \
    /cpusets/user/cpuset.cpus /cpusets/user/cpus; do
    if [[ -r "$path" ]]; then
      local raw
      raw=$(tr -d '[:space:]' <"$path" 2>/dev/null || true)
      [[ -n "$raw" ]] && { normalise_cpu_set "$raw"; return $?; }
    fi
  done

  command -v cset &>/dev/null || return 1
  # Fall back to parsing cset's own status. Accept any line naming the user
  # cpuset and take the first CPUSPEC(...) / cpus: <list> / bare list on it.
  # Capture first and filter in the shell rather than piping through `head`:
  # under `set -o pipefail` a `grep | head -1` pipeline reports 141 when grep
  # is SIGPIPE'd writing the second matching line, which would turn a
  # perfectly readable mask into "cannot tell" and trigger a needless rebuild.
  local status_out line spec
  status_out=$(cset shield -s 2>/dev/null) || return 1
  line=$(grep -i -m1 'user' <<<"$status_out") || return 1
  [[ -z "$line" ]] && return 1
  spec=$(sed -nE 's/.*CPUSPEC\(([0-9,-]+)\).*/\1/p' <<<"$line")
  [[ -z "$spec" ]] && spec=$(sed -nE 's/.*cpus:[[:space:]]*([0-9,-]+).*/\1/p' <<<"$line")
  [[ -z "$spec" ]] && return 1
  normalise_cpu_set "$spec"
}

# Which isolation is ACTUALLY in force. Echoes "cset <cpus>", "isolcpus <cpus>"
# or "none"; never fails.
#
# The cset shield is asked first and independently of
# /sys/devices/system/cpu/isolated, because **only isolcpus writes that file**.
# do_status used to gate its whole method-detection block on it, so on a box
# running a live cset shield it took the else branch and reported "Isolated
# cores: none / All N cores available for general use" while the kernel held
# CPUSPEC(1-3,7-9) — measured on the 6-core dev box once cset was installed.
# The nested `elif ... cset shield -s | grep user` branch could not save it: it
# required `isolated` to be non-empty *and* the cmdline to lack isolcpus, which
# together essentially never happen.
# Cores isolcpus took out at boot, "" when none. A named function so the
# precedence below can be exercised without a real /sys read, and $RTC_SYSFS_ROOT
# so a fixture tree can drive it the way the rest of this file is driven --
# stubbing the function is still what test_cpu_shield_mask.sh does, because the
# *precedence* it fixes needs both axes faked at once, not one real file.
isolcpus_isolated_cpus() {
  cat "${RTC_SYSFS_ROOT:-/sys}/devices/system/cpu/isolated" 2>/dev/null || echo ""
}

shield_isolation_method() {
  local cset_cpus isolated
  cset_cpus=$(shield_actual_user_cpus 2>/dev/null || true)
  if [[ -n "$cset_cpus" ]]; then
    echo "cset ${cset_cpus}"
    return 0
  fi
  isolated=$(isolcpus_isolated_cpus)
  if [[ -n "$isolated" ]]; then
    echo "isolcpus ${isolated}"
    return 0
  fi
  echo "none"
}

# Print just the main MPC core (first entry of get_mpc_cores). Derived, so it
# stays here rather than in the generated file.
get_mpc_main_core() {
  get_mpc_cores "$@" | cut -d',' -f1
}

# Expand a CSV / range list of physical core ids into a CSV of logical CPU ids
# that includes each core's HT sibling (when SMT is enabled).
#
# Inputs:
#   $1 — physical core id list, accepts CSV ("1,2,3"), range ("1-3"), or mix
#        ("1-3,5"). Whitespace is tolerated.
# Outputs (stdout):
#   - non-SMT: same logical id set as input, normalized to range notation
#     (e.g. "1,2,3" → "1-3").
#   - SMT:     input physical ids resolved to their cpuN entries plus the
#     matching HT sibling cpus, sorted and range-collapsed
#     (e.g. on 6C/12T with input "1-2" → "1-2,7-8").
#
# Test hook: honors $RTC_SYSFS_ROOT (default: /sys) when walking the topology
# tree, mirroring detect_hybrid_capability so unit tests can construct fake
# sysfs trees without root.
#
# Used by setup_grub_rt.sh for nohz_full / rcu_nocbs values — RT-only kernel
# features must include every logical cpu that a SCHED_FIFO thread can land
# on, otherwise the HT sibling keeps ticking and invalidates the RT core's
# cache. Distinct from compute_expected_isolated() which returns *all* non-OS
# cores (RT + nrt + driver), suitable for cset shield / isolcpus but too
# broad for nohz_full semantics.
_rt_expand_smt_siblings() {
  local raw="$1"
  local cpu_root="${RTC_SYSFS_ROOT:-/sys}/devices/system/cpu"

  # Parse input → space-separated physical core id list.
  local phys_ids
  phys_ids=$(_rt_parse_cpulist "$raw")
  [[ -z "$phys_ids" ]] && { echo ""; return 0; }

  # Build core_id → "cpu0 cpu1 ..." map from sysfs topology.
  local -A core_to_cpus=()
  local cpu_dir cpu cid
  for cpu_dir in "$cpu_root"/cpu[0-9]*; do
    [[ -d "$cpu_dir" ]] || continue
    cpu="${cpu_dir##*/cpu}"
    [[ "$cpu" =~ ^[0-9]+$ ]] || continue
    cid=$(_rt_read_trim "$cpu_dir/topology/core_id")
    [[ -z "$cid" ]] && continue
    if [[ -z "${core_to_cpus[$cid]+x}" ]]; then
      core_to_cpus[$cid]="$cpu"
    else
      core_to_cpus[$cid]="${core_to_cpus[$cid]} $cpu"
    fi
  done

  # Resolve each requested physical core to its full sibling set.
  local result_cpus=()
  local pid sib
  for pid in $phys_ids; do
    if [[ -n "${core_to_cpus[$pid]+x}" ]]; then
      for sib in ${core_to_cpus[$pid]}; do
        result_cpus+=("$sib")
      done
    else
      # Fallback: sysfs missing or core_id mismatch — pass through input id.
      result_cpus+=("$pid")
    fi
  done

  # Deduplicate + sort ascending, then collapse to range notation.
  local sorted
  sorted=$(printf '%s\n' "${result_cpus[@]}" | sort -nu | tr '\n' ' ')
  # shellcheck disable=SC2086  # word splitting intended
  _format_cpu_range $sorted
}

# Print the RT-thread cpuset for GRUB nohz_full / rcu_nocbs.
# Layout v4.1: base = get_rt_cores() (rt_control 1, rt_callback 2, MPC 3+).
# On SMT systems, each RT physical core's HT sibling is included so that the
# sibling does not keep ticking and invalidate the RT core's L1/L2 cache.
# Output is range-collapsed CSV (e.g. "1-3" or "1-3,7-9").
get_rt_cores_with_siblings() {
  local rt_csv
  rt_csv=$(get_rt_cores)
  _rt_expand_smt_siblings "$rt_csv"
}

# Translate a thread_config *slot index* into the kernel logical CPU id that
# ApplyThreadConfig actually pins. Mirrors C++ rtc::SlotToLogicalCpu():
# PHYSICAL_CORE_SLOTS[slot] is the primary logical cpu of that physical core
# (SMT sibling excluded). Canonical home — verify_rt_runtime.sh consumes this
# too (previously a local duplicate). Empty PHYSICAL_CORE_SLOTS (container /
# no sysfs) or out-of-range slot → identity fallback (legacy behaviour).
# Requires detect_hybrid_capability to have populated PHYSICAL_CORE_SLOTS.
slot_to_logical_cpu() {
  local slot="$1"
  if [[ "$slot" -lt 0 ]]; then
    echo "$slot"
    return 0
  fi
  local -a _slot_arr=()
  read -ra _slot_arr <<<"${PHYSICAL_CORE_SLOTS:-}"
  local count=${#_slot_arr[@]}
  if (( count == 0 || slot >= count )); then
    echo "$slot"
    return 0
  fi
  echo "${_slot_arr[$slot]}"
}

# hex affinity mask → comma-separated CPU list ("none" when empty). Canonical
# home; verify_rt_runtime.sh consumed a local copy until rt_classify_external_rt_threads
# (below) needed the same conversion from a sourceable location.
mask_to_cpus() {
  local mask_hex="$1"
  local mask_dec=$((16#${mask_hex}))
  local cpus=""
  local i=0
  while [[ $((mask_dec >> i)) -gt 0 ]]; do
    if (( mask_dec & (1 << i) )); then
      cpus="${cpus:+${cpus},}${i}"
    fi
    ((i++))
  done
  echo "${cpus:-none}"
}

# Judge whether an *external driver* process has its RT loop where the config
# says it should be. Pure (no /proc, no globals) so it can be fixture-tested;
# verify_rt_runtime.sh collects the records and calls this.
#
#   stdin : one "<tid> <policy> <prio> <mask_hex>" record per thread
#   args  : <expected_prio> <expected_logical_cpu>
#   stdout: "<VERDICT>|<detail>"  — OK / NO_FIFO / WRONG_PRIO / WRONG_CPU
#
# Selection is by scheduling policy, not thread name, and that is forced rather
# than chosen: controller_manager gives its loop thread no pthread name, so every
# non-DDS TID in the process shares the comm "ros2_control_no" (measured, 29
# threads). SCHED_FIFO at the configured priority is the only usable selector.
#
# NO_FIFO is a FAIL condition, not a WARN: the loop thread requests SCHED_FIFO at
# startup and a denied request (missing realtime group / RLIMIT_RTPRIO) leaves it
# on SCHED_OTHER with one warning buried in the launch log, while its affinity
# still applies. Checking affinity alone therefore passes a process whose RT loop
# has silently degraded to a fair-scheduled thread — the sag this verdict exists
# to surface (issue #343).
rt_classify_external_rt_threads() {
  local expected_prio="$1" expected_cpu="$2"
  local expected_mask=$((1 << expected_cpu))

  local total=0 fifo=0 matched=0 off="" prios=""
  local tid policy prio mask_hex mask_dec
  while read -r tid policy prio mask_hex; do
    [[ -z "$tid" ]] && continue
    ((total++)) || true
    [[ "$policy" != "SCHED_FIFO" ]] && continue
    ((fifo++)) || true
    prios="${prios} ${prio}"
    [[ "$prio" != "$expected_prio" ]] && continue
    mask_dec=$((16#${mask_hex}))
    if [[ "$mask_dec" -eq "$expected_mask" ]]; then
      ((matched++)) || true
    else
      off="${off} ${tid}:{$(mask_to_cpus "$mask_hex")}"
    fi
  done

  if [[ "$fifo" -eq 0 ]]; then
    echo "NO_FIFO|${total} threads, none SCHED_FIFO"
  elif [[ "$matched" -eq 0 && -z "$off" ]]; then
    echo "WRONG_PRIO|SCHED_FIFO prio present:${prios} (expected ${expected_prio})"
  elif [[ -n "$off" ]]; then
    echo "WRONG_CPU|prio ${expected_prio} off cpu ${expected_cpu} —${off}"
  else
    echo "OK|${matched}/${fifo} SCHED_FIFO prio ${expected_prio} on cpu ${expected_cpu}"
  fi
}

# Classify an external driver whose threads are NOT uniform (issue #345).
#
# rt_classify_external_rt_threads() above assumes one interesting RT thread and
# an anonymous remainder; the all-threads affinity check assumes every TID shares
# one core. udp_hand_node is neither: after the RT/non-RT split its CommLoop is
# SCHED_FIFO 65 on the hand_driver core, its hand_aux_io lane is SCHED_OTHER on
# the aux core, and everything else (executor, DDS) is SCHED_OTHER on the
# hand_driver core. A single verdict has to cover all three.
#
#   $1  default logical cpu for threads with no named expectation
#   $2  spec: space-separated "comm:cpu:policy:prio" entries. policy is
#       SCHED_FIFO or SCHED_OTHER; prio is compared only for SCHED_FIFO.
#   stdin: one "<tid> <comm> <policy> <prio> <mask_hex>" record per line.
#          comm must not contain spaces (/proc/<pid>/task/<tid>/comm never does
#          for RTC threads — pthread_setname_np names are single tokens).
#
# stdout: "<verdict>|<detail>" with verdict in OK / MISSING / WRONG_SCHED /
# WRONG_CPU / NO_THREADS. Ordering is deliberate: a thread that is absent, or
# running under the wrong scheduler, is a worse finding than one merely on the
# wrong core, and reporting the worst first keeps the message actionable.
#
# Note the tier collapse is not special-cased. On <=5 physical cores the
# hand_driver slot IS the OS slot, so the caller resolves both expectations to
# the same logical cpu and every thread legitimately shares it — the spec then
# passes by construction rather than by an exemption that could rot.
rt_classify_external_thread_layout() {
  local default_cpu="$1" spec="$2"
  local -a spec_comms=() spec_cpus=() spec_policies=() spec_prios=() spec_seen=()
  local entry
  for entry in $spec; do
    IFS=':' read -r _c _cpu _pol _prio <<<"$entry"
    spec_comms+=("$_c")
    spec_cpus+=("$_cpu")
    spec_policies+=("$_pol")
    spec_prios+=("$_prio")
    spec_seen+=(0)
  done

  local total=0 wrong_cpu="" wrong_sched="" i
  local tid comm policy prio mask_hex mask_dec want_cpu want_pol want_prio idx
  while read -r tid comm policy prio mask_hex; do
    [[ -z "$tid" ]] && continue
    ((total++)) || true

    idx=-1
    for i in "${!spec_comms[@]}"; do
      if [[ "$comm" == "${spec_comms[$i]}" ]]; then
        idx=$i
        break
      fi
    done

    if [[ "$idx" -ge 0 ]]; then
      spec_seen[$idx]=1
      want_cpu="${spec_cpus[$idx]}"
      want_pol="${spec_policies[$idx]}"
      want_prio="${spec_prios[$idx]}"
      if [[ "$policy" != "$want_pol" ]]; then
        wrong_sched="${wrong_sched} ${comm}:${policy}(want ${want_pol})"
      elif [[ "$want_pol" == "SCHED_FIFO" && "$prio" != "$want_prio" ]]; then
        wrong_sched="${wrong_sched} ${comm}:prio${prio}(want ${want_prio})"
      fi
    else
      want_cpu="$default_cpu"
    fi

    [[ -z "$mask_hex" ]] && continue
    # A negative expectation is the documented "no pin" sentinel (thread_config.hpp
    # cpu_core = -1; slot_to_logical_cpu passes negatives through unchanged), so
    # the thread is deliberately unconfined and there is no mask to match. Without
    # this guard $((1 << -1)) silently evaluates to -9223372036854775808 and every
    # real mask compares unequal — a false WRONG_CPU for a correct configuration.
    [[ "$want_cpu" -lt 0 ]] && continue
    mask_dec=$((16#${mask_hex}))
    if [[ "$mask_dec" -ne $((1 << want_cpu)) ]]; then
      wrong_cpu="${wrong_cpu} ${comm}(${tid}):{$(mask_to_cpus "$mask_hex")}(want cpu ${want_cpu})"
    fi
  done

  if [[ "$total" -eq 0 ]]; then
    echo "NO_THREADS|스레드 상태를 하나도 읽지 못했다"
    return 0
  fi

  local missing=""
  for i in "${!spec_comms[@]}"; do
    [[ "${spec_seen[$i]}" -eq 0 ]] && missing="${missing} ${spec_comms[$i]}"
  done

  if [[ -n "$missing" ]]; then
    echo "MISSING|기대한 스레드 없음:${missing} (총 ${total} 스레드)"
  elif [[ -n "$wrong_sched" ]]; then
    echo "WRONG_SCHED|스케줄러 불일치:${wrong_sched}"
  elif [[ -n "$wrong_cpu" ]]; then
    echo "WRONG_CPU|코어 불일치:${wrong_cpu}"
  else
    echo "OK|${total} 스레드, 명명된 ${#spec_comms[@]}개 정책·코어 일치, 나머지 cpu ${default_cpu}"
  fi
}

# Expand a CSV/range list of LOGICAL cpu ids to include each one's SMT siblings
# (cpus sharing the same physical_package_id + core_id), sorted + range-collapsed.
# Distinct from _rt_expand_smt_siblings(), which takes *physical core ids* and
# matches them against core_id; this takes already-resolved logical cpus. Honors
# $RTC_SYSFS_ROOT for unit tests.
_rt_expand_logical_siblings() {
  local raw="$1"
  local cpu_root="${RTC_SYSFS_ROOT:-/sys}/devices/system/cpu"
  local ids
  ids=$(_rt_parse_cpulist "$raw")
  [[ -z "$ids" ]] && { echo ""; return 0; }

  # Build (pkg,core_id) → "cpuA cpuB ..." map from sysfs topology.
  local -A key_to_cpus=()
  local cpu_dir cpu pkg core key
  for cpu_dir in "$cpu_root"/cpu[0-9]*; do
    [[ -d "$cpu_dir" ]] || continue
    cpu="${cpu_dir##*/cpu}"
    [[ "$cpu" =~ ^[0-9]+$ ]] || continue
    core=$(_rt_read_trim "$cpu_dir/topology/core_id")
    [[ -z "$core" ]] && continue
    pkg=$(_rt_read_trim "$cpu_dir/topology/physical_package_id")
    key="${pkg:-0}_${core}"
    key_to_cpus[$key]="${key_to_cpus[$key]:+${key_to_cpus[$key]} }$cpu"
  done

  # Resolve each logical cpu to its full sibling set.
  local result_cpus=()
  local lg pkgl corel keyl sib
  for lg in $ids; do
    corel=$(_rt_read_trim "$cpu_root/cpu${lg}/topology/core_id")
    pkgl=$(_rt_read_trim "$cpu_root/cpu${lg}/topology/physical_package_id")
    keyl="${pkgl:-0}_${corel}"
    if [[ -n "$corel" && -n "${key_to_cpus[$keyl]+x}" ]]; then
      for sib in ${key_to_cpus[$keyl]}; do
        result_cpus+=("$sib")
      done
    elif [[ -d "$cpu_root/cpu${lg}" ]]; then
      # cpuN 디렉토리는 있으나 topology(core_id) 만 없음 — 그대로 통과.
      result_cpus+=("$lg")
    fi
    # cpuN 디렉토리 자체가 없으면 존재하지 않는 코어(저코어 머신에서 degraded
    # layout 이 만든 phantom, 예: 2코어에서 core 2·3) → shield 대상에서 제외한다.
    # cset/taskset 에 없는 코어를 넘겨 실패하는 것을 막는다.
  done

  local sorted
  sorted=$(printf '%s\n' "${result_cpus[@]}" | sort -nu | tr '\n' ' ')
  # shellcheck disable=SC2086  # word splitting intended
  _format_cpu_range $sorted
}

# The LOGICAL CPU set that cset/cgroup must isolate for RT threads.
# get_rt_cores() returns *slot indices* (== C++ ThreadConfig::cpu_core), NOT
# logical cpu ids. On SMT/hybrid systems the two differ (e.g. slot 1 → logical
# cpu 2 on a P-core-HT NUC), so shielding the raw slots would leave the real RT
# logical CPUs unprotected and shield the wrong siblings. This translates each
# RT slot → logical cpu (slot_to_logical_cpu) and adds HT siblings, matching
# where ApplyThreadConfig pins and keeping the shared L1/L2 free of other work.
# Range-collapsed CSV (e.g. "1-3" non-SMT, "2-7" interleaved SMT, "1-3,7-9"
# primaries-first SMT).
#
# NOTE: get_rt_cores_with_siblings() (GRUB nohz_full/rcu_nocbs) still expands via
# core_id, not slot→logical. The two agree on primaries-first enumeration but can
# diverge on interleaved/hybrid layouts — unifying GRUB onto this logical basis is
# tracked as a follow-up (needs its own real-hardware validation).
#
# $1 = launch profile (issue #350), default `rtc_default_profile`. A profile
# that drops MPC narrows this to the rt_control + rt_callback span.
get_rt_shield_cpus() {
  local profile="${1:-$(rtc_default_profile)}"
  if ! rtc_is_layout_profile "$profile"; then
    echo "get_rt_shield_cpus: unknown layout profile '${profile}' (declared: $(rtc_layout_profiles))" >&2
    return 1
  fi
  detect_hybrid_capability >/dev/null 2>&1 || true
  # Build a CSV of logical cpus (_rt_expand_logical_siblings / _rt_parse_cpulist
  # split on commas, not whitespace — a space-joined list would collapse to one
  # token, e.g. "1 2 3" → "123").
  local slot logicals="" lg
  for slot in $(_rt_parse_cpulist "$(get_rt_cores "" "$profile")"); do
    lg=$(slot_to_logical_cpu "$slot")
    logicals="${logicals:+$logicals,}$lg"
  done
  _rt_expand_logical_siblings "$logicals"
}

# The LOGICAL CPU set that a cset "user" cpuset must allow so the WHOLE
# integrated_rt_controller process fits inside it. Distinct from
# get_rt_shield_cpus(), which returns only the RT threads' cores: the CM
# process also hosts nrt_logging / nrt_callback / nrt_publish, and cset moves a
# *process* (all its threads) into one cpuset. If the cpuset omitted a core an
# nrt thread self-pins to, that self-pin would hit the same EINVAL RT threads
# hit when the CM is trapped in "system" (thread_utils.hpp returns before
# SCHED_FIFO on affinity failure). So the cpuset the CM is moved into
# (cpu_shield.sh) must be the union of RT + nrt cores. Issue #151.
#
# Union = get_rt_cores() ∪ get_nrt_cores(), minus OS slots (get_os_cores):
# on the degraded (<6-core) tier nrt shares Core 0 with the OS, and Core 0 must
# never be shielded — dropping the OS slot collapses this to the RT-only span
# there. Each surviving slot → logical cpu (slot_to_logical_cpu) + HT siblings.
# Range-collapsed CSV (e.g. "2-7" NUC13, "1-3" non-SMT — every tier since #380).
#
# v5 (#349) narrowed the result on every tier: the nrt lanes now sit on the
# rt_callback slot, so the union no longer widens past the RT span and the
# NUC13 shield went "2-9,12-13" -> "2-9", handing logical 12,13 back to the
# system cpuset; #380 then reclaimed the mpc_worker slots, taking it to "2-7". The union is deliberately NOT collapsed into
# get_rt_shield_cpus() — keeping it is what makes a future tier that splits an
# nrt lane back out widen the cpuset automatically instead of silently leaving
# that thread outside the shield. Issue #151's condition is dormant, not gone.
#
# NOTE: arm_driver / hand_driver are *separate processes* pinned by launch
# taskset (slots 4,5 since v6/#383 -> logical 8,9 on NUC13, both in "system");
# they are not CM threads and are deliberately excluded from this cpuset. That
# exclusion is why #383 could move them without touching this function: a full
# tier x profile sweep of the result was byte-identical before and after.
#
# $1 = launch profile (issue #350), default `rtc_default_profile`. Only the RT
# half narrows: get_nrt_cores() is profile-blind, so the three CFS lanes stay in
# the cpuset no matter which profile is active and never lose their self-pin
# target (the #151 EINVAL). On NUC13 tier 12 this is "2-7" under mpc_on and
# "2-5" under mpc_off — the P3 pair (logical 6,7) goes back to "system". The
# "2-9" once written here was the pre-#380 value, from when the mpc_worker
# slots still widened the union; both numbers are pinned by
# test_get_cm_shield_cpus_nuc13_hybrid so prose that drifts off them is
# detectable rather than merely wrong (#379).
get_cm_shield_cpus() {
  local profile="${1:-$(rtc_default_profile)}"
  if ! rtc_is_layout_profile "$profile"; then
    echo "get_cm_shield_cpus: unknown layout profile '${profile}' (declared: $(rtc_layout_profiles))" >&2
    return 1
  fi
  detect_hybrid_capability >/dev/null 2>&1 || true
  local os_slots
  os_slots=" $(_rt_parse_cpulist "$(get_os_cores)") "
  # Comma-join RT + nrt so _rt_parse_cpulist (splits on commas) sees one flat
  # slot list — a space-join would fuse the boundary tokens ("5" + "8" → "5 8"
  # → "58" after whitespace strip).
  local combined
  combined="$(get_rt_cores "" "$profile"),$(get_nrt_cores)"
  local slot logicals="" lg
  for slot in $(_rt_parse_cpulist "$combined"); do
    # Never shield the OS core, even if a degraded-tier nrt slot lands on it.
    case "$os_slots" in *" $slot "*) continue ;; esac
    lg=$(slot_to_logical_cpu "$slot")
    logicals="${logicals:+$logicals,}$lg"
  done
  _rt_expand_logical_siblings "$logicals"
}

# ── Canonical thread layout printout ───────────────────────────────────────
# Operator-facing view of the same table the C++ constants and the launch
# mirror use. Callers (cpu_shield.sh::do_status, setup_irq_affinity.sh) used to
# keep their own copies; consolidating it here removed that drift risk.
#
# Every number below comes from the generated helpers -- no core id is written
# out by hand. #353 was the expensive lesson: a re-typed tier table turns the
# *verifier* into a liar (it reports PASS against a stale expectation), and the
# same applies to a diagram operators read to decide whether a box is healthy.
# The prose annotations (which lane is degraded, which shares a core) stay hand
# written; only the placement is derived.
#
# Usage:
#   print_thread_layout                  # auto-detect physical core count
#   print_thread_layout 12               # explicit core count
#
# Each line is indented with two spaces and uses `info`-style coloring so
# the output blends with the surrounding [PREFIX] log lines.
print_thread_layout() {
  local ncpu="${1:-$(get_physical_cores)}"
  echo -e "  ${BOLD}Thread layout (${ncpu}-core, layout ${RTC_THREAD_LAYOUT_VERSION})${NC}"

  local os_slot rt_ctl rt_cb mpc_main arm hand nrt_log nrt_cb nrt_pub
  os_slot=$(get_os_cores)
  rt_ctl=$(get_role_slot rt_control "$ncpu")
  rt_cb=$(get_role_slot rt_callback "$ncpu")
  mpc_main=$(get_role_slot mpc_main "$ncpu")
  arm=$(get_arm_driver_slot "$ncpu")
  hand=$(get_hand_driver_slot "$ncpu")
  nrt_log=$(get_role_slot nrt_logging "$ncpu")
  nrt_cb=$(get_role_slot nrt_callback "$ncpu")
  # nrt_publish is its own thread with its own comm name (#349 D15), not part
  # of nrt_callback. Omitting it here showed the operator one thread on a core
  # where verify_rt_runtime.sh now expects two -- the same class of drift the
  # header warns about, one layer up from the numbers.
  nrt_pub=$(get_role_slot nrt_publish "$ncpu")

  # MPC lane: a single thread on every tier (#380 removed the worker slots).
  local mpc_cores mpc_last mpc_label mpc_sched mpc_policy prios
  mpc_cores=$(get_mpc_cores "$ncpu")
  mpc_last="${mpc_cores##*,}"
  prios=$(get_role_priority mpc_main "$ncpu")
  mpc_label="mpc_main"
  mpc_policy=$(get_role_policy mpc_main "$ncpu")
  if [[ "$mpc_policy" == "SCHED_OTHER" ]]; then
    mpc_sched="CFS, degraded"
  else
    mpc_sched="${mpc_policy} ${prios}"
  fi

  if [[ "$ncpu" -le 5 ]]; then
    # Degraded: everything that is not RT collapses onto the OS slot.
    _ptl_row "Core ${os_slot}" "OS / DDS / NIC IRQ + nrt_logging + nrt_callback + nrt_publish + arm/hand_driver (slot ${arm}/${hand}) + hand_aux_io (degraded)"
    _ptl_row "Core ${rt_ctl}" "rt_control   ($(_ptl_sched rt_control "$ncpu"))"
    _ptl_row "Core ${rt_cb}" "rt_callback  ($(_ptl_sched rt_callback "$ncpu"); DDS recv co-pin, degraded)"
    _ptl_row "$(_ptl_span "$mpc_main" "$mpc_last")" "${mpc_label}     (${mpc_sched})"
    return 0
  fi

  # v5 (#349): the three CFS nrt lanes fold onto the rt_callback slot, which is
  # then the "aux slot". Build that row's tail here so a co-resident lane is
  # listed on the core it actually runs on instead of getting a second row for
  # the same core. Derived per-role, not hardcoded to slot 2 -- a tier that
  # splits a lane back out still gets its own row from the block below.
  local aux_tail="" role
  for role in nrt_logging nrt_callback nrt_publish; do
    if [[ "$(get_role_slot "$role" "$ncpu")" == "$rt_cb" ]]; then
      aux_tail="${aux_tail} + ${role} ($(_ptl_sched "$role" "$ncpu"))"
    fi
  done

  _ptl_row "Core ${os_slot}" "OS / DDS / NIC IRQ + hand_aux_io (CFS; hand aux lane, issue #345)"
  _ptl_row "Core ${rt_ctl}" "rt_control   ($(_ptl_sched rt_control "$ncpu"))"
  _ptl_row "Core ${rt_cb}" "rt_callback  ($(_ptl_sched rt_callback "$ncpu"); DDS recv co-pin via taskset, CFS)${aux_tail:+${aux_tail} [aux slot]}"
  _ptl_row "$(_ptl_span "$mpc_main" "$mpc_last")" "${mpc_label} (${mpc_sched})"
  if [[ "$arm" == "$hand" ]]; then
    _ptl_row "Core ${arm}" "arm_driver + hand_driver (shared, degraded; hand_udp_recv FIFO 65)"
  else
    _ptl_row "Core ${arm}" "arm_driver   (CFS, taskset pin)"
    _ptl_row "Core ${hand}" "hand_driver  (CFS, in-process self-pin; hand_udp_recv FIFO 65)"
  fi
  # Any nrt lane that did NOT fold onto the aux slot gets its own row, grouped
  # by slot so two lanes on one core print once instead of twice. Under v5 every
  # tier >= 6 folds all three, so this block emits nothing; it stays because it
  # is what makes a tier that splits a lane back out show it rather than hide it.
  #
  # The old "shared (degraded)" label lived here and keyed off nrt_logging ==
  # nrt_callback. v5 makes that condition true on EVERY tier, which would have
  # printed the primary 12-core target as degraded -- the co-residency is the
  # design now, so the label is gone. Genuine degradation is still labelled on
  # the arm/hand row above and in the tier notes.
  local slot seen_slots="" label
  for slot in "$nrt_log" "$nrt_cb" "$nrt_pub"; do
    [[ "$slot" == "$rt_cb" ]] && continue
    case " ${seen_slots} " in *" ${slot} "*) continue ;; esac
    seen_slots="${seen_slots} ${slot}"
    label=""
    for role in nrt_logging nrt_callback nrt_publish; do
      if [[ "$(get_role_slot "$role" "$ncpu")" == "$slot" ]]; then
        label="${label:+${label} + }${role} ($(_ptl_sched "$role" "$ncpu"))"
      fi
    done
    _ptl_row "Core ${slot}" "${label}"
  done

  # Slots past the highest claimed one are spare. Derived, so a tier that grows
  # a role automatically shrinks the spare span instead of overstating it.
  local highest spare_lo last
  highest=$(printf '%s\n' "$rt_ctl" "$rt_cb" "$mpc_last" "$arm" "$hand" "$nrt_log" "$nrt_cb" "$nrt_pub" |
    sort -n | tail -1)
  spare_lo=$((highest + 1))
  last=$((ncpu - 1))
  if [[ "$spare_lo" -le "$last" ]]; then
    _ptl_row "$(_ptl_span "$spare_lo" "$last")" "spare / user shield"
  fi
}

# Scheduler label for one role: "SCHED_FIFO 90" for an RT policy, "CFS -5" for
# SCHED_OTHER. Derived rather than written out, so a manifest policy change
# cannot leave this diagram telling the operator a policy the layout dropped
# (the numbers were already generated; the policy words were not).
_ptl_sched() {
  local role="$1" ncpu="$2" policy
  policy=$(get_role_policy "$role" "$ncpu") || return 1
  if [[ "$policy" == "SCHED_OTHER" ]]; then
    echo "CFS $(get_role_nice "$role" "$ncpu")"
  else
    echo "${policy} $(get_role_priority "$role" "$ncpu")"
  fi
}

# "Core 3:" / "Core 3-5:" padded to a common column so the lanes line up.
_ptl_row() {
  printf '    %-9s %s\n' "$1:" "$2"
}

_ptl_span() {
  if [[ "$1" == "$2" ]]; then echo "Core $1"; else echo "Core $1-$2"; fi
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

# ── LTTng kernel module ↔ kernel 호환 판정 (issue #190) ─────────────────────
#
# lttng-modules 는 커널 tracepoint 프로토타입을 자기 헤더에 재선언하므로, 커널이
# 그 프로토타입을 바꾸면 **결정론적 컴파일 실패**가 난다 (재시도·재부팅 무효).
# 실측: `hrtimer_start` 가 3번째 인자 `bool was_armed` 를 얻은 커널에서
# lttng-modules 2.14.0 의 `probes/lttng-probe-timer.o` 가
# `conflicting types for 'trace_hrtimer_start'` 로 죽는다. 대응 가드는
# upstream v2.14.6 (2026-06-19) 에서 들어왔다.
#
# 이 함수들이 표로 갖는 것은 **upstream `include/instrumentation/events/timer.h`
# 의 가드 그 자체**이지 "이 커널은 나쁘다" 가 아니다 — 그래야 커널 타임라인을
# 되살리는 쪽(upstream DKMS 등록)이 같은 표를 "그럼 어느 버전이 필요한가" 로
# 읽는다. 소비 지점이 둘이지 판정이 둘이 아니다.
#
# **판정은 3-값이고 두 개는 "시도한다" 다.** 막는 것(incompatible)은 표의 규칙에
# *양성으로* 걸릴 때뿐이다 — 표는 외부 사실의 사본이라 반드시 stale 해지는데,
# 그때 나는 오류가 "되는 조합을 막는 것"(사용자가 커널 트레이싱을 잃고, 게다가
# 안내까지 틀림)이면 지금보다 나빠지고, "안 되는 조합을 시도하는 것"(600초 낭비)
# 이면 지금과 같기 때문이다. 편향을 후자로 둔다.
#
# 표 밖의 한계 — 미래 커널이 *다른* tracepoint 로 깨지면 이 표는 모른다. 그리고
# `>= 7.1.0` 행이 요구하는 2.14.6 은 **하한이지 보증이 아니다** (7.1 이 또 다른
# 가드를 요구할 수 있다). 하한을 과소평가해도 C 경로의 skip 판정은 옳고 (2.14.0
# < 2.14.6 은 여전히 참), 틀리는 것은 upstream 등록 시의 태그 힌트뿐이다.
#
# 표 갱신 시: upstream 태그의 timer.h `hrtimer_start` 가드를 그대로 옮긴다.
#   https://github.com/lttng/lttng-modules/blob/v2.14.6/include/instrumentation/events/timer.h

# `hrtimer_start` 3-인자 가드 (upstream v2.14.6 기준).
# 각 행: "<min> <max_exclusive|-> <필요한 lttng-modules 최소 버전>"
LTTNG_KERNEL_GUARD_TABLE=(
  "7.1.0    -        2.14.6"
  "7.0.10   7.1.0    2.14.6"
  "6.18.33  6.19.0   2.14.6"
  "6.12.91  6.13.0   2.14.6"
  "6.6.141  6.7.0    2.14.6"
  "6.1.175  6.2.0    2.14.6"
)

# lttng_kernel_build_version <kernel_release> [headers_root]
#
# 판정에 쓸 커널 버전을 **헤더 Makefile** 에서 읽는다. `uname -r` 을 쓰면 안 된다 —
# 이 박스가 정확히 그 함정이다: `uname -r` 은 `7.0.0-28-generic` 인데 헤더의
# 실제 값은 `VERSION=7 PATCHLEVEL=0 SUBLEVEL=12` (= 7.0.12) 다. 가드가 보는 것은
# `LINUX_VERSION_CODE`, 즉 헤더 쪽이므로 `uname -r` 로 판정하면 7.0.0 이
# `[7.0.10, 7.1.0)` 에 안 들어가 **비호환을 호환으로 오판**한다.
#
# stdout: "V.P.S" (읽었을 때) 또는 빈 문자열 (헤더 없음 → 판정 불가).
lttng_kernel_build_version() {
  local release="$1" root="${2:-/lib/modules}"
  local makefile="${root}/${release}/build/Makefile"
  [[ -r "$makefile" ]] || return 0

  local v p s
  v=$(sed -n 's/^VERSION[[:space:]]*=[[:space:]]*\([0-9]\+\).*/\1/p'    "$makefile" | head -1)
  p=$(sed -n 's/^PATCHLEVEL[[:space:]]*=[[:space:]]*\([0-9]\+\).*/\1/p' "$makefile" | head -1)
  s=$(sed -n 's/^SUBLEVEL[[:space:]]*=[[:space:]]*\([0-9]\+\).*/\1/p'   "$makefile" | head -1)
  [[ -n "$v" && -n "$p" ]] || return 0
  echo "${v}.${p}.${s:-0}"
}

# lttng_modules_min_version_for_kernel <kernel_version>
#
# stdout: 그 커널이 요구하는 lttng-modules 최소 버전, 제약이 없으면 빈 문자열.
# 이것이 **upstream DKMS 등록 경로가 소비할 함수**다 ("어느 태그를 가져오나").
lttng_modules_min_version_for_kernel() {
  local kver="$1" row min max req
  [[ "$kver" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || return 0

  for row in "${LTTNG_KERNEL_GUARD_TABLE[@]}"; do
    read -r min max req <<<"$row"
    dpkg --compare-versions "$kver" ge "$min" || continue
    if [[ "$max" != "-" ]]; then
      dpkg --compare-versions "$kver" lt "$max" || continue
    fi
    echo "$req"
    return 0
  done
}

# lttng_modules_compat_verdict <kernel_release> <candidate_version> [headers_root]
#
# stdout 한 줄: "<verdict> <kernel_version> <required>"
#   ok           — 알려진 제약 없음, 또는 candidate 가 이미 충족 (→ 설치 시도)
#   incompatible — 표에 양성으로 걸리고 candidate 가 미달        (→ 시도 금지)
#   unknown      — 커널 버전을 못 읽음                            (→ 설치 시도)
# `<kernel_version>`/`<required>` 는 해당 없으면 "-".
#
# candidate 는 apt 후보 문자열 그대로 받아도 된다 — Debian revision
# (`2.14.0-1ubuntu1.1~24.04.1` 의 `-1ubuntu...`) 은 upstream 버전이 아니므로
# 잘라내고 비교한다.
#
# 탈출구: `RTC_LTTNG_SKIP_COMPAT_GATE=1` 이면 항상 unknown 을 내 시도로 되돌린다.
# 배포판이 upstream 버전을 안 올린 채 수정을 backport 하면 (예:
# `2.14.0-1ubuntu1.2` 가 가드를 포함) 버전 비교로는 그것을 알 수 없고, 그때
# 게이트가 되는 조합을 막는 유일한 경우가 되기 때문이다.
lttng_modules_compat_verdict() {
  local release="$1" candidate="$2" root="${3:-/lib/modules}"

  if [[ "${RTC_LTTNG_SKIP_COMPAT_GATE:-0}" == "1" ]]; then
    echo "unknown - -"
    return 0
  fi

  local kver
  kver=$(lttng_kernel_build_version "$release" "$root")
  if [[ -z "$kver" ]]; then
    echo "unknown - -"
    return 0
  fi

  local req
  req=$(lttng_modules_min_version_for_kernel "$kver")
  if [[ -z "$req" ]]; then
    echo "ok ${kver} -"
    return 0
  fi

  # upstream 버전만 남긴다: "2.14.0-1ubuntu1.1~24.04.1" → "2.14.0"
  local upstream="${candidate%%-*}"
  if [[ -n "$upstream" ]] && dpkg --compare-versions "$upstream" ge "$req"; then
    echo "ok ${kver} ${req}"
  else
    echo "incompatible ${kver} ${req}"
  fi
}
