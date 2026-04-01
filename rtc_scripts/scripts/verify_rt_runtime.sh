#!/bin/bash
# verify_rt_runtime.sh — 제어기 구동 중 RT 설정 런타임 검증 스크립트
#
# 제어기(rt_controller)가 실행 중일 때, 각 스레드의 RT 설정이
# thread_config.hpp에 정의된 대로 올바르게 적용되었는지 실시간 확인한다.
# check_rt_setup.sh가 정적 시스템 설정을 검증하는 반면,
# 이 스크립트는 실행 중인 프로세스의 런타임 상태를 검증한다.
#
# Usage:
#   ./verify_rt_runtime.sh              # 상세 출력 (기본)
#   ./verify_rt_runtime.sh --summary    # 카테고리당 1줄 요약
#   ./verify_rt_runtime.sh --json       # CI용 JSON 출력
#   ./verify_rt_runtime.sh --watch      # 3초 간격 반복 모니터링
#   ./verify_rt_runtime.sh --watch 5    # 5초 간격 반복 모니터링
#   ./verify_rt_runtime.sh --help
#
# 검증 카테고리 (7개):
#   1. Process Discovery    — rt_controller 프로세스 및 스레드 감지
#   2. Scheduling Policy    — SCHED_FIFO/OTHER 정책 및 우선순위 검증
#   3. CPU Affinity         — 스레드별 코어 할당 일치 여부
#   4. Memory Locking       — mlockall 적용 및 page fault 추적
#   5. Context Switches     — 비자발적 컨텍스트 스위치 비율 감시
#   6. CPU Migration        — RT 스레드의 코어 이동 감지
#   7. RT Throttling        — sched_rt_runtime_us 쓰로틀링 발생 여부
#
# Exit codes: 0=all pass, 1=warnings only, 2=failures exist, 3=controller not running

set -u

# ── 공통 라이브러리 로드 ────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/rt_common.sh
source "${SCRIPT_DIR}/lib/rt_common.sh"

# ── Output mode ──────────────────────────────────────────────────────────────
OUTPUT_MODE="verbose"   # verbose | summary | json
WATCH_MODE=0
WATCH_INTERVAL=3

# ── Counters ─────────────────────────────────────────────────────────────────
PASS_COUNT=0
WARN_COUNT=0
FAIL_COUNT=0
SKIP_COUNT=0

# ── Category tracking ────────────────────────────────────────────────────────
declare -A CATEGORY_STATUS
declare -A CATEGORY_DETAIL

# ── Process info (populated by check_process_discovery) ───────────────────────
CONTROLLER_PID=""
declare -A THREAD_TIDS    # name → tid
declare -A THREAD_NAMES   # tid → name

# ── Argument parsing ─────────────────────────────────────────────────────────
show_help() {
  echo ""
  echo -e "${BOLD}verify_rt_runtime.sh — 제어기 RT 런타임 검증${NC}"
  echo ""
  echo "Usage: $0 [OPTIONS]"
  echo ""
  echo "Options:"
  echo "  --verbose     상세 출력 (기본)"
  echo "  --summary     카테고리당 1줄 요약"
  echo "  --json        JSON 출력 (CI용)"
  echo "  --watch [N]   N초 간격 반복 모니터링 (기본 3초)"
  echo "  --help        이 도움말"
  echo ""
  echo "Exit codes:"
  echo "  0  모든 항목 PASS"
  echo "  1  경고(WARN)만 있음"
  echo "  2  실패(FAIL) 항목 존재"
  echo "  3  제어기 미실행"
  echo ""
  exit 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --verbose)  OUTPUT_MODE="verbose"; shift ;;
    --summary)  OUTPUT_MODE="summary"; shift ;;
    --json)     OUTPUT_MODE="json"; shift ;;
    --watch)
      WATCH_MODE=1; shift
      if [[ $# -gt 0 && "$1" =~ ^[0-9]+$ ]]; then
        WATCH_INTERVAL="$1"; shift
      fi
      ;;
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

_section() {
  if [[ "$OUTPUT_MODE" == "verbose" ]]; then
    echo ""
    echo -e "${BOLD}[$1] $2${NC}"
  fi
}

_category_start() {
  CATEGORY_STATUS["$1"]="PASS"
  CATEGORY_DETAIL["$1"]=""
}

_category_update() {
  local cat="$1" status="$2"
  local current="${CATEGORY_STATUS[$cat]}"
  if [[ "$status" == "FAIL" ]]; then
    CATEGORY_STATUS["$cat"]="FAIL"
  elif [[ "$status" == "WARN" && "$current" != "FAIL" ]]; then
    CATEGORY_STATUS["$cat"]="WARN"
  fi
}

_category_set_detail() {
  CATEGORY_DETAIL["$1"]="$2"
}

# ── CPU layout ───────────────────────────────────────────────────────────────
compute_cpu_layout
PHYSICAL_CORES="$TOTAL_CORES"

# ── thread_config.hpp 기반 기대값 테이블 ──────────────────────────────────────
# 코어 수에 따라 기대값이 달라진다.
# 형식: "thread_name:expected_cpu:expected_policy:expected_priority"
# policy: 1=SCHED_FIFO, 0=SCHED_OTHER
declare -a EXPECTED_THREADS

build_expected_threads() {
  # thread_config.hpp / SelectThreadConfigs() 기반 기대값
  # 형식: "thread_name:expected_cpu:expected_policy:expected_priority[:optional]"
  # optional 필드가 있으면 해당 스레드 미발견 시 WARN 대신 SKIP 처리.
  #   - udp_recv:   Transceiver 사용 시에만 생성 (현재 미사용)
  EXPECTED_THREADS=()
  if [[ "$PHYSICAL_CORES" -ge 16 ]]; then
    # Core 0-1: OS, Core 2-3: RT, Core 4-8: cset shield, Core 9-11: system
    EXPECTED_THREADS=(
      "rt_control:2:1:90"
      "sensor_io:3:1:70"
      "udp_recv:9:1:65:optional"
      "logger:10:0:0"
      "aux:11:0:0"
      "rt_publish:11:0:0"
    )
  elif [[ "$PHYSICAL_CORES" -ge 12 ]]; then
    # Core 0-1: OS, Core 2-6: cset shield, Core 7-11: system
    EXPECTED_THREADS=(
      "rt_control:7:1:90"
      "sensor_io:8:1:70"
      "udp_recv:9:1:65:optional"
      "logger:10:0:0"
      "aux:11:0:0"
      "rt_publish:11:0:0"
    )
  elif [[ "$PHYSICAL_CORES" -ge 10 ]]; then
    # Core 0-1: OS, Core 2-6: cset shield, Core 7-9: system (shared Core 9)
    EXPECTED_THREADS=(
      "rt_control:7:1:90"
      "sensor_io:8:1:70"
      "udp_recv:9:1:65:optional"
      "logger:9:0:0"
      "aux:9:0:0"
      "rt_publish:9:0:0"
    )
  elif [[ "$PHYSICAL_CORES" -ge 8 ]]; then
    # Core 0-1: OS, Core 2-6: RT threads (no cset shield)
    EXPECTED_THREADS=(
      "rt_control:2:1:90"
      "sensor_io:3:1:70"
      "udp_recv:4:1:65:optional"
      "logger:5:0:0"
      "aux:6:0:0"
      "rt_publish:6:0:0"
    )
  elif [[ "$PHYSICAL_CORES" -ge 6 ]]; then
    # Core 0-1: OS, Core 2-5: RT threads, udp_recv shares Core 5 with aux
    EXPECTED_THREADS=(
      "rt_control:2:1:90"
      "sensor_io:3:1:70"
      "udp_recv:5:1:65:optional"
      "logger:4:0:0"
      "aux:5:0:0"
      "rt_publish:5:0:0"
    )
  else
    # 4-core fallback: Core 0: OS, Core 1-3: RT, udp_recv shares Core 2
    EXPECTED_THREADS=(
      "rt_control:1:1:90"
      "sensor_io:2:1:70"
      "udp_recv:2:1:65:optional"
      "logger:3:0:0"
      "aux:3:0:0"
      "rt_publish:3:0:0"
    )
  fi
}

build_expected_threads

# ── 스레드의 스케줄링 정책 읽기 ──────────────────────────────────────────────
# /proc/<pid>/task/<tid>/sched 또는 chrt -p <tid>로 확인
get_thread_sched_policy() {
  local tid="$1"
  if command -v chrt &>/dev/null; then
    local out
    out=$(chrt -p "$tid" 2>/dev/null) || return 1
    # "pid N's current scheduling policy: SCHED_FIFO"
    # "pid N's current scheduling priority: 90"
    local policy
    policy=$(echo "$out" | grep -i "policy" | grep -oE "SCHED_[A-Z]+")
    echo "$policy"
  else
    # fallback: /proc/<pid>/sched
    local sched_file="/proc/${tid}/sched"
    if [[ -f "$sched_file" ]]; then
      local policy_raw
      policy_raw=$(grep "^policy" "$sched_file" 2>/dev/null | awk '{print $NF}')
      case "$policy_raw" in
        0) echo "SCHED_OTHER" ;;
        1) echo "SCHED_FIFO" ;;
        2) echo "SCHED_RR" ;;
        *) echo "UNKNOWN($policy_raw)" ;;
      esac
    fi
  fi
}

get_thread_sched_priority() {
  local tid="$1"
  if command -v chrt &>/dev/null; then
    local out
    out=$(chrt -p "$tid" 2>/dev/null) || return 1
    # "pid 44607's current scheduling priority: 90"
    # Extract only the number after the colon (priority value), not the pid
    echo "$out" | grep -i "priority" | sed 's/.*: *//'
  fi
}

# ── 스레드의 CPU affinity 읽기 ───────────────────────────────────────────────
get_thread_cpu_affinity() {
  local tid="$1"
  if command -v taskset &>/dev/null; then
    local out
    out=$(taskset -p "$tid" 2>/dev/null) || return 1
    # "pid N's current affinity mask: 4" → hex mask
    echo "$out" | grep -oE "[0-9a-fA-F]+$"
  fi
}

# hex mask → CPU 리스트 문자열
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

# ══════════════════════════════════════════════════════════════════════════════
# [1/7] Process Discovery
# ══════════════════════════════════════════════════════════════════════════════
check_process_discovery() {
  _section "1/7" "Process Discovery"
  _category_start "process_discovery"

  # rt_controller 프로세스 찾기 (ur5e_rt_controller 등 변형 실행 파일명도 포함)
  # pgrep -f는 전체 커맨드라인에 매칭. ROS2 실행 시 --ros-args 등이 붙으므로
  # $ 앵커 대신 공백 또는 문자열 끝( |$)으로 실행파일명의 끝을 매칭.
  CONTROLLER_PID=$(pgrep -f '(^|/)(ur5e_)?rt_controller( |$)' 2>/dev/null | head -1 || true)

  if [[ -z "$CONTROLLER_PID" ]]; then
    _fail "rt_controller / ur5e_rt_controller 프로세스 미발견 — 제어기가 실행 중이 아닙니다"
    _category_update "process_discovery" "FAIL"
    _category_set_detail "process_discovery" "not running"
    return 1
  fi

  _pass "rt_controller PID: ${CONTROLLER_PID}"

  # 스레드 목록 수집 (/proc/<pid>/task/)
  local task_dir="/proc/${CONTROLLER_PID}/task"
  if [[ ! -d "$task_dir" ]]; then
    _fail "스레드 정보 접근 불가: ${task_dir}"
    _category_update "process_discovery" "FAIL"
    _category_set_detail "process_discovery" "PID ${CONTROLLER_PID}, no task dir"
    return 1
  fi

  local thread_count=0
  local known_count=0

  for tid_dir in "${task_dir}"/*/; do
    local tid
    tid=$(basename "$tid_dir")
    [[ "$tid" =~ ^[0-9]+$ ]] || continue

    ((thread_count++)) || true

    # 스레드 이름 읽기
    local comm_file="${tid_dir}comm"
    local tname=""
    if [[ -f "$comm_file" ]]; then
      tname=$(cat "$comm_file" 2>/dev/null || echo "")
    fi

    THREAD_NAMES["$tid"]="$tname"

    # 알려진 스레드 이름 매칭
    for entry in "${EXPECTED_THREADS[@]}"; do
      local ename
      ename=$(echo "$entry" | cut -d: -f1)
      if [[ "$tname" == "$ename" ]]; then
        THREAD_TIDS["$ename"]="$tid"
        ((known_count++)) || true
        break
      fi
    done
  done

  _pass "총 스레드: ${thread_count}개"

  # 알려진 스레드 매칭 결과 — optional 스레드 미발견은 SKIP 처리
  local required_count=0
  local required_found=0
  local optional_missing=0
  for entry in "${EXPECTED_THREADS[@]}"; do
    local ename eopt
    ename=$(echo "$entry" | cut -d: -f1)
    eopt=$(echo "$entry" | cut -d: -f5 -s)
    if [[ "$eopt" == "optional" ]]; then
      if [[ -z "${THREAD_TIDS[$ename]:-}" ]]; then
        ((optional_missing++)) || true
      fi
    else
      ((required_count++)) || true
      if [[ -n "${THREAD_TIDS[$ename]:-}" ]]; then
        ((required_found++)) || true
      fi
    fi
  done

  local expected_count=${#EXPECTED_THREADS[@]}
  if [[ "$known_count" -eq "$expected_count" ]]; then
    _pass "thread_config.hpp 스레드 전체 감지: ${known_count}/${expected_count}"
  elif [[ "$required_found" -eq "$required_count" ]]; then
    _pass "필수 스레드 전체 감지: ${required_found}/${required_count}"
    # optional 스레드 미발견은 정보성 표시
    for entry in "${EXPECTED_THREADS[@]}"; do
      local ename eopt
      ename=$(echo "$entry" | cut -d: -f1)
      eopt=$(echo "$entry" | cut -d: -f5 -s)
      if [[ "$eopt" == "optional" && -z "${THREAD_TIDS[$ename]:-}" ]]; then
        _skip "  선택적 스레드 미활성: ${ename}"
      fi
    done
  elif [[ "$known_count" -gt 0 ]]; then
    _warn "thread_config.hpp 스레드 일부 감지: ${known_count}/${expected_count}"
    # 누락된 필수 스레드 표시
    for entry in "${EXPECTED_THREADS[@]}"; do
      local ename eopt
      ename=$(echo "$entry" | cut -d: -f1)
      eopt=$(echo "$entry" | cut -d: -f5 -s)
      if [[ -z "${THREAD_TIDS[$ename]:-}" ]]; then
        if [[ "$eopt" == "optional" ]]; then
          _skip "  선택적 스레드 미활성: ${ename}"
        else
          _warn "  미발견: ${ename}"
        fi
      fi
    done
    _category_update "process_discovery" "WARN"
  else
    _fail "thread_config.hpp 스레드 감지 실패 (${known_count}/${expected_count})"
    _category_update "process_discovery" "FAIL"
  fi

  _category_set_detail "process_discovery" "PID ${CONTROLLER_PID}, ${known_count}/${expected_count} threads"
}

# ══════════════════════════════════════════════════════════════════════════════
# [2/7] Scheduling Policy
# ══════════════════════════════════════════════════════════════════════════════
check_scheduling_policy() {
  _section "2/7" "Scheduling Policy"
  _category_start "scheduling_policy"

  if [[ -z "$CONTROLLER_PID" ]]; then
    _skip "제어기 미실행"
    _category_set_detail "scheduling_policy" "skipped"
    return
  fi

  local ok=0 total=0

  for entry in "${EXPECTED_THREADS[@]}"; do
    local ename ecpu epolicy eprio
    ename=$(echo "$entry" | cut -d: -f1)
    epolicy=$(echo "$entry" | cut -d: -f3)
    eprio=$(echo "$entry" | cut -d: -f4)

    local tid="${THREAD_TIDS[$ename]:-}"
    if [[ -z "$tid" ]]; then
      continue
    fi

    ((total++)) || true

    local expected_policy_name
    if [[ "$epolicy" -eq 1 ]]; then
      expected_policy_name="SCHED_FIFO"
    else
      expected_policy_name="SCHED_OTHER"
    fi

    local actual_policy actual_prio
    actual_policy=$(get_thread_sched_policy "$tid" 2>/dev/null || echo "")
    actual_prio=$(get_thread_sched_priority "$tid" 2>/dev/null || echo "")

    if [[ -z "$actual_policy" ]]; then
      _skip "${ename} (TID ${tid}): 스케줄링 정책 읽기 실패"
      continue
    fi

    # 정책 검증
    local policy_ok=1 prio_ok=1

    if [[ "$actual_policy" != "$expected_policy_name" ]]; then
      policy_ok=0
    fi

    # SCHED_FIFO 스레드의 우선순위 검증
    if [[ "$epolicy" -eq 1 && -n "$actual_prio" && "$actual_prio" != "$eprio" ]]; then
      prio_ok=0
    fi

    if [[ "$policy_ok" -eq 1 && "$prio_ok" -eq 1 ]]; then
      if [[ "$epolicy" -eq 1 ]]; then
        _pass "${ename} (TID ${tid}): ${actual_policy} prio ${actual_prio}"
      else
        _pass "${ename} (TID ${tid}): ${actual_policy}"
      fi
      ((ok++)) || true
    elif [[ "$policy_ok" -eq 0 ]]; then
      _fail "${ename} (TID ${tid}): ${actual_policy} (기대값: ${expected_policy_name})"
      _category_update "scheduling_policy" "FAIL"
    else
      _warn "${ename} (TID ${tid}): ${actual_policy} prio ${actual_prio} (기대값: prio ${eprio})"
      _category_update "scheduling_policy" "WARN"
    fi
  done

  _category_set_detail "scheduling_policy" "${ok}/${total} correct"
}

# ══════════════════════════════════════════════════════════════════════════════
# [3/7] CPU Affinity
# ══════════════════════════════════════════════════════════════════════════════
check_cpu_affinity() {
  _section "3/7" "CPU Affinity"
  _category_start "cpu_affinity"

  if [[ -z "$CONTROLLER_PID" ]]; then
    _skip "제어기 미실행"
    _category_set_detail "cpu_affinity" "skipped"
    return
  fi

  if ! command -v taskset &>/dev/null; then
    _skip "taskset 미설치 — CPU affinity 확인 불가"
    _category_set_detail "cpu_affinity" "no taskset"
    return
  fi

  local ok=0 total=0

  for entry in "${EXPECTED_THREADS[@]}"; do
    local ename ecpu
    ename=$(echo "$entry" | cut -d: -f1)
    ecpu=$(echo "$entry" | cut -d: -f2)

    local tid="${THREAD_TIDS[$ename]:-}"
    if [[ -z "$tid" ]]; then
      continue
    fi

    ((total++)) || true

    local mask_hex
    mask_hex=$(get_thread_cpu_affinity "$tid" 2>/dev/null || echo "")

    if [[ -z "$mask_hex" ]]; then
      _skip "${ename} (TID ${tid}): affinity 읽기 실패"
      continue
    fi

    # 기대 mask 계산 (단일 코어 pin)
    local expected_mask_dec=$((1 << ecpu))
    local actual_mask_dec=$((16#${mask_hex}))
    local actual_cpus
    actual_cpus=$(mask_to_cpus "$mask_hex")

    if [[ "$actual_mask_dec" -eq "$expected_mask_dec" ]]; then
      _pass "${ename} (TID ${tid}): Core ${ecpu} (mask 0x${mask_hex})"
      ((ok++)) || true
    elif (( actual_mask_dec & expected_mask_dec )); then
      # 기대 코어 포함하지만 다른 코어도 포함
      _warn "${ename} (TID ${tid}): CPU {${actual_cpus}} (기대값: Core ${ecpu} only)"
      _category_update "cpu_affinity" "WARN"
    else
      _fail "${ename} (TID ${tid}): CPU {${actual_cpus}} (기대값: Core ${ecpu})"
      _category_update "cpu_affinity" "FAIL"
    fi
  done

  _category_set_detail "cpu_affinity" "${ok}/${total} pinned"
}

# ══════════════════════════════════════════════════════════════════════════════
# [4/7] Memory Locking
# ══════════════════════════════════════════════════════════════════════════════
check_memory_locking() {
  _section "4/7" "Memory Locking"
  _category_start "memory_locking"

  if [[ -z "$CONTROLLER_PID" ]]; then
    _skip "제어기 미실행"
    _category_set_detail "memory_locking" "skipped"
    return
  fi

  local status_file="/proc/${CONTROLLER_PID}/status"
  if [[ ! -r "$status_file" ]]; then
    _skip "프로세스 상태 읽기 불가: ${status_file}"
    _category_set_detail "memory_locking" "no permission"
    return
  fi

  # VmLck — mlockall()이 적용되면 0보다 큰 값
  local vmlck
  vmlck=$(grep "^VmLck:" "$status_file" 2>/dev/null | awk '{print $2}')
  local vmlck_unit
  vmlck_unit=$(grep "^VmLck:" "$status_file" 2>/dev/null | awk '{print $3}')

  if [[ -z "$vmlck" ]]; then
    _skip "VmLck 정보 없음"
    _category_set_detail "memory_locking" "unknown"
    return
  fi

  if [[ "$vmlck" -gt 0 ]]; then
    _pass "mlockall 적용됨: VmLck = ${vmlck} ${vmlck_unit}"
  else
    _fail "mlockall 미적용: VmLck = 0 (page fault 위험)"
    _category_update "memory_locking" "FAIL"
  fi

  # Page fault 추적 (/proc/<pid>/stat)
  # field 10: minflt (minor faults), field 12: majflt (major faults)
  local stat_content
  stat_content=$(cat "/proc/${CONTROLLER_PID}/stat" 2>/dev/null || echo "")
  if [[ -n "$stat_content" ]]; then
    # comm 필드에 괄호와 공백이 포함될 수 있으므로 ')' 이후부터 파싱
    local fields_after_comm
    fields_after_comm="${stat_content#*) }"

    local minflt majflt
    # ')' 이후: state(1) minflt 은 3번째 필드에 해당 → index 7(전체 기준 10)
    # 전체 stat에서: pid(1) comm(2) state(3) ppid(4) pgrp(5) session(6) tty_nr(7)
    #               tpgid(8) flags(9) minflt(10) cminflt(11) majflt(12) cmajflt(13)
    # ')' 이후 필드 기준: state(1) ... minflt(8번째 = index 7)
    minflt=$(echo "$fields_after_comm" | awk '{print $8}')
    majflt=$(echo "$fields_after_comm" | awk '{print $10}')

    if [[ -n "$majflt" && "$majflt" -gt 0 ]]; then
      _warn "Major page faults: ${majflt} (I/O 지연 발생 가능)"
      _category_update "memory_locking" "WARN"
    elif [[ -n "$majflt" ]]; then
      _pass "Major page faults: 0"
    fi

    if [[ -n "$minflt" ]]; then
      _pass "Minor page faults: ${minflt}"
    fi
  fi

  _category_set_detail "memory_locking" "VmLck=${vmlck}${vmlck_unit}"
}

# ══════════════════════════════════════════════════════════════════════════════
# [5/7] Context Switches
# ══════════════════════════════════════════════════════════════════════════════
check_context_switches() {
  _section "5/7" "Context Switches"
  _category_start "context_switches"

  if [[ -z "$CONTROLLER_PID" ]]; then
    _skip "제어기 미실행"
    _category_set_detail "context_switches" "skipped"
    return
  fi

  local rt_invol_total=0
  local rt_vol_total=0
  local rt_thread_count=0

  for entry in "${EXPECTED_THREADS[@]}"; do
    local ename epolicy
    ename=$(echo "$entry" | cut -d: -f1)
    epolicy=$(echo "$entry" | cut -d: -f3)

    local tid="${THREAD_TIDS[$ename]:-}"
    if [[ -z "$tid" ]]; then
      continue
    fi

    local status_file="/proc/${CONTROLLER_PID}/task/${tid}/status"
    if [[ ! -r "$status_file" ]]; then
      continue
    fi

    local vol_cs invol_cs
    vol_cs=$(grep "^voluntary_ctxt_switches:" "$status_file" 2>/dev/null | awk '{print $2}')
    invol_cs=$(grep "^nonvoluntary_ctxt_switches:" "$status_file" 2>/dev/null | awk '{print $2}')

    if [[ -z "$vol_cs" || -z "$invol_cs" ]]; then
      continue
    fi

    # RT(SCHED_FIFO) 스레드의 비자발적 컨텍스트 스위치는 최소여야 함
    if [[ "$epolicy" -eq 1 ]]; then
      ((rt_thread_count++)) || true
      rt_invol_total=$((rt_invol_total + invol_cs))
      rt_vol_total=$((rt_vol_total + vol_cs))

      # 비자발적 cs가 자발적의 10% 이상이면 경고
      local ratio=0
      if [[ "$vol_cs" -gt 0 ]]; then
        ratio=$((invol_cs * 100 / vol_cs))
      fi

      if [[ "$invol_cs" -gt 1000 && "$ratio" -gt 10 ]]; then
        _warn "${ename} (TID ${tid}): vol=${vol_cs} invol=${invol_cs} (비자발적 ${ratio}% — 선점 과다)"
        _category_update "context_switches" "WARN"
      else
        _pass "${ename} (TID ${tid}): vol=${vol_cs} invol=${invol_cs}"
      fi
    else
      if [[ "$OUTPUT_MODE" == "verbose" ]]; then
        _pass "${ename} (TID ${tid}): vol=${vol_cs} invol=${invol_cs}"
      fi
    fi
  done

  if [[ "$rt_thread_count" -gt 0 ]]; then
    _category_set_detail "context_switches" "RT invol=${rt_invol_total}, vol=${rt_vol_total}"
  else
    _category_set_detail "context_switches" "no RT threads found"
  fi
}

# ══════════════════════════════════════════════════════════════════════════════
# [6/7] CPU Migration
# ══════════════════════════════════════════════════════════════════════════════
check_cpu_migration() {
  _section "6/7" "CPU Migration"
  _category_start "cpu_migration"

  if [[ -z "$CONTROLLER_PID" ]]; then
    _skip "제어기 미실행"
    _category_set_detail "cpu_migration" "skipped"
    return
  fi

  local ok=0 total=0

  for entry in "${EXPECTED_THREADS[@]}"; do
    local ename ecpu epolicy
    ename=$(echo "$entry" | cut -d: -f1)
    ecpu=$(echo "$entry" | cut -d: -f2)
    epolicy=$(echo "$entry" | cut -d: -f3)

    # RT 스레드만 검사 (SCHED_FIFO)
    [[ "$epolicy" -eq 1 ]] || continue

    local tid="${THREAD_TIDS[$ename]:-}"
    if [[ -z "$tid" ]]; then
      continue
    fi

    ((total++)) || true

    # /proc/<pid>/task/<tid>/stat의 39번째 필드: processor (마지막 실행 CPU)
    local stat_content
    stat_content=$(cat "/proc/${CONTROLLER_PID}/task/${tid}/stat" 2>/dev/null || echo "")
    if [[ -z "$stat_content" ]]; then
      _skip "${ename} (TID ${tid}): stat 읽기 실패"
      continue
    fi

    local fields_after_comm
    fields_after_comm="${stat_content#*) }"

    # ')' 이후 37번째 필드 = 전체 39번째 필드 (processor)
    local current_cpu
    current_cpu=$(echo "$fields_after_comm" | awk '{print $37}')

    if [[ -z "$current_cpu" ]]; then
      _skip "${ename} (TID ${tid}): CPU 정보 없음"
      continue
    fi

    if [[ "$current_cpu" -eq "$ecpu" ]]; then
      _pass "${ename} (TID ${tid}): 현재 CPU ${current_cpu} (기대값: Core ${ecpu})"
      ((ok++)) || true
    else
      _fail "${ename} (TID ${tid}): 현재 CPU ${current_cpu} (기대값: Core ${ecpu}) — 코어 이동 발생!"
      _category_update "cpu_migration" "FAIL"
    fi
  done

  if [[ "$total" -eq 0 ]]; then
    _skip "확인 가능한 RT 스레드 없음"
    _category_set_detail "cpu_migration" "no RT threads"
  else
    _category_set_detail "cpu_migration" "${ok}/${total} on expected core"
  fi
}

# ══════════════════════════════════════════════════════════════════════════════
# [7/7] RT Throttling
# ══════════════════════════════════════════════════════════════════════════════
check_rt_throttling() {
  _section "7/7" "RT Throttling"
  _category_start "rt_throttling"

  if [[ -z "$CONTROLLER_PID" ]]; then
    _skip "제어기 미실행"
    _category_set_detail "rt_throttling" "skipped"
    return
  fi

  # sched_rt_runtime_us 확인
  local rt_runtime
  rt_runtime=$(sysctl -n kernel.sched_rt_runtime_us 2>/dev/null || echo "unknown")

  if [[ "$rt_runtime" == "-1" ]]; then
    _pass "sched_rt_runtime_us: -1 (RT 쓰로틀링 비활성)"
  elif [[ "$rt_runtime" == "unknown" ]]; then
    _skip "sched_rt_runtime_us: 읽기 실패"
  else
    _fail "sched_rt_runtime_us: ${rt_runtime} (기본 950000 → SCHED_FIFO 스레드 95% 제한)"
    _category_update "rt_throttling" "FAIL"
  fi

  # /proc/sys/kernel/sched_rt_period_us
  local rt_period
  rt_period=$(sysctl -n kernel.sched_rt_period_us 2>/dev/null || echo "unknown")

  # RT throttle event 확인 (/proc/sched_debug에서 rt_throttled 카운터)
  if [[ -r "/proc/sched_debug" ]]; then
    local throttled_count
    throttled_count=$(grep -c "rt_throttled" /proc/sched_debug 2>/dev/null || echo "0")
    if [[ "$throttled_count" -gt 0 && "$rt_runtime" != "-1" ]]; then
      _warn "RT 쓰로틀링 이벤트 감지 (sched_debug에 rt_throttled 존재)"
      _category_update "rt_throttling" "WARN"
    fi
  fi

  # RT 스레드 CPU 사용률 확인 (/proc/<pid>/task/<tid>/stat의 utime+stime)
  for entry in "${EXPECTED_THREADS[@]}"; do
    local ename epolicy
    ename=$(echo "$entry" | cut -d: -f1)
    epolicy=$(echo "$entry" | cut -d: -f3)

    [[ "$epolicy" -eq 1 ]] || continue

    local tid="${THREAD_TIDS[$ename]:-}"
    if [[ -z "$tid" ]]; then
      continue
    fi

    # sched 파일에서 nr_switches 확인
    local sched_file="/proc/${CONTROLLER_PID}/task/${tid}/sched"
    if [[ -r "$sched_file" ]]; then
      local nr_switches
      nr_switches=$(grep "^nr_switches" "$sched_file" 2>/dev/null | awk '{print $NF}')
      local wait_sum
      wait_sum=$(grep "^se.statistics.wait_sum" "$sched_file" 2>/dev/null | awk '{print $NF}')

      if [[ -n "$nr_switches" && -n "$wait_sum" ]]; then
        # wait_sum이 높으면 (>1초 = 1000000000ns) 쓰로틀링/대기 과다
        local wait_sum_int=${wait_sum%%.*}
        if [[ "$wait_sum_int" -gt 1000000000 ]]; then
          local wait_ms=$((wait_sum_int / 1000000))
          _warn "${ename} (TID ${tid}): 누적 대기 ${wait_ms}ms (쓰로틀링 또는 선점 지연)"
          _category_update "rt_throttling" "WARN"
        else
          local wait_ms=$((wait_sum_int / 1000000))
          _pass "${ename} (TID ${tid}): 누적 대기 ${wait_ms}ms, switches=${nr_switches}"
        fi
      fi
    fi
  done

  _category_set_detail "rt_throttling" "rt_runtime=${rt_runtime}"
}

# ══════════════════════════════════════════════════════════════════════════════
# Summary output
# ══════════════════════════════════════════════════════════════════════════════
print_summary() {
  local categories=("process_discovery" "scheduling_policy" "cpu_affinity"
                    "memory_locking" "context_switches" "cpu_migration" "rt_throttling")
  local labels=("Process Discovery" "Scheduling Policy" "CPU Affinity"
                "Memory Locking" "Context Switches" "CPU Migration" "RT Throttling")

  if [[ "$OUTPUT_MODE" == "summary" ]]; then
    echo -e "${BOLD}RT Runtime Verification (${PHYSICAL_CORES}-core, PID ${CONTROLLER_PID:-N/A})${NC}"
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

      echo -e "  ${color}[${icon}]${NC} $(printf '%-20s' "$label") ${DIM}${detail}${NC}"
    done
  fi

  # 최종 요약
  if [[ "$OUTPUT_MODE" != "json" ]]; then
    echo ""
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
  local categories=("process_discovery" "scheduling_policy" "cpu_affinity"
                    "memory_locking" "context_switches" "cpu_migration" "rt_throttling")

  echo "{"
  echo "  \"timestamp\": \"$(date -Iseconds)\","
  echo "  \"hostname\": \"$(hostname)\","
  echo "  \"controller_pid\": ${CONTROLLER_PID:-null},"
  echo "  \"cpu_cores\": ${PHYSICAL_CORES},"
  echo "  \"categories\": {"

  local first=1
  for cat in "${categories[@]}"; do
    local status="${CATEGORY_STATUS[$cat]:-SKIP}"
    local detail="${CATEGORY_DETAIL[$cat]:-}"
    detail=$(echo "$detail" | sed 's/\\/\\\\/g; s/"/\\"/g')

    [[ "$first" -eq 0 ]] && echo ","
    printf "    \"%s\": {\"status\": \"%s\", \"detail\": \"%s\"}" "$cat" "$status" "$detail"
    first=0
  done

  echo ""
  echo "  },"

  # 스레드 상세 정보
  echo "  \"threads\": {"
  first=1
  for entry in "${EXPECTED_THREADS[@]}"; do
    local ename ecpu epolicy eprio
    ename=$(echo "$entry" | cut -d: -f1)
    ecpu=$(echo "$entry" | cut -d: -f2)
    epolicy=$(echo "$entry" | cut -d: -f3)
    eprio=$(echo "$entry" | cut -d: -f4)

    local tid="${THREAD_TIDS[$ename]:-}"

    [[ "$first" -eq 0 ]] && echo ","
    if [[ -n "$tid" ]]; then
      local apolicy aprio amask
      apolicy=$(get_thread_sched_policy "$tid" 2>/dev/null || echo "unknown")
      aprio=$(get_thread_sched_priority "$tid" 2>/dev/null || echo "0")
      amask=$(get_thread_cpu_affinity "$tid" 2>/dev/null || echo "0")
      printf "    \"%s\": {\"tid\": %s, \"expected_cpu\": %s, \"actual_affinity\": \"0x%s\", \"expected_policy\": %s, \"actual_policy\": \"%s\", \"actual_priority\": %s}" \
        "$ename" "$tid" "$ecpu" "${amask:-0}" "$epolicy" "${apolicy:-unknown}" "${aprio:-0}"
    else
      printf "    \"%s\": {\"tid\": null, \"expected_cpu\": %s, \"expected_policy\": %s}" \
        "$ename" "$ecpu" "$epolicy"
    fi
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
run_checks() {
  # 카운터 초기화
  PASS_COUNT=0
  WARN_COUNT=0
  FAIL_COUNT=0
  SKIP_COUNT=0
  CONTROLLER_PID=""
  THREAD_TIDS=()
  THREAD_NAMES=()

  if [[ "$OUTPUT_MODE" == "verbose" ]]; then
    echo ""
    echo -e "${BOLD}${CYAN}RT Runtime Verification (${PHYSICAL_CORES}-core system)${NC}"
    echo -e "${DIM}$(date '+%Y-%m-%d %H:%M:%S')${NC}"
  fi

  check_process_discovery

  if [[ -z "$CONTROLLER_PID" ]]; then
    if [[ "$OUTPUT_MODE" == "json" ]]; then
      print_json
    else
      print_summary
    fi
    return 3
  fi

  check_scheduling_policy
  check_cpu_affinity
  check_memory_locking
  check_context_switches
  check_cpu_migration
  check_rt_throttling

  if [[ "$OUTPUT_MODE" == "json" ]]; then
    print_json
  else
    print_summary
  fi

  if [[ "$FAIL_COUNT" -gt 0 ]]; then
    return 2
  elif [[ "$WARN_COUNT" -gt 0 ]]; then
    return 1
  else
    return 0
  fi
}

main() {
  if [[ "$WATCH_MODE" -eq 1 ]]; then
    echo -e "${BOLD}${CYAN}RT Runtime Watch Mode (${WATCH_INTERVAL}s interval, Ctrl+C to stop)${NC}"
    while true; do
      clear
      run_checks || true
      sleep "$WATCH_INTERVAL"
    done
  else
    run_checks
    exit $?
  fi
}

main
