#!/bin/bash
# verify_rt_runtime.sh — 제어기 구동 중 RT 설정 런타임 검증 스크립트
#
# Robot bringup exec(예: integrated_rt_controller)가 실행 중일 때, 각 스레드의 RT 설정이
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
#   1. Process Discovery    — robot bringup exec(integrated_rt_controller 등) 감지
#                             + external driver 프로세스(arm_driver=ros2_control_node,
#                               hand_driver=udp_hand_node) 별도 PID 발견
#   2. Scheduling Policy    — SCHED_FIFO/OTHER 정책 및 우선순위 검증
#   3. CPU Affinity         — in-process 스레드 + external driver 프로세스의 코어 할당 일치 여부
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

# ── Shared reporter infrastructure ───────────────────────────────────────────
# _pass/_warn/_fail/_skip/_section, _category_*, print_category_row,
# print_result_line, emit_json_*  — see lib/rt_report.sh.
# shellcheck source=lib/rt_report.sh
source "${SCRIPT_DIR}/lib/rt_report.sh"
init_report_state

# ── Process info (populated by check_process_discovery) ───────────────────────
CONTROLLER_PID=""
declare -A THREAD_TIDS    # name → tid
declare -A THREAD_NAMES   # tid → name

# ── External driver processes (arm_driver / hand_driver) ──────────────────────
# arm_driver / hand_driver are SEPARATE processes pinned by the launch file via
# taskset — NOT pthread_setname_np'd threads inside the controller
# (thread_config.hpp:72-74; SystemThreadConfigs.{arm,hand}_driver in
# thread_utils.hpp). They can never appear in /proc/<controller>/task, so they
# are discovered as their own PIDs by process comm. See build_external_drivers().
declare -a EXTERNAL_DRIVER_NAMES=("arm_driver" "hand_driver")
declare -A EXTERNAL_DRIVER_SLOTS       # name → expected slot index
declare -A EXTERNAL_DRIVER_RT_PRIO     # name → SCHED_FIFO prio of its RT loop ("" = n/a)
declare -A EXTERNAL_DRIVER_THREAD_LAYOUT  # name → "comm:cpu:policy:prio ..." ("" = n/a)
declare -A EXTERNAL_DRIVER_COMMS       # name → space-separated comm candidates
declare -A EXTERNAL_DRIVER_PIDS        # name → discovered PID (empty if absent)

# ── CycloneDDS thread names (issue #164) ──────────────────────────────────────
# The threads the launch DDS sweep co-pins onto the rt_callback core. Duplicated
# from rtc_tools/rtc_tools/launch/pinning.py::CYCLONEDDS_THREAD_NAMES (the SSoT)
# because bash cannot import it — kept honest by a drift test, the same
# arrangement RTC_OWNED_THREAD_NAMES has with thread_config.hpp.
#
# Measured against CycloneDDS 0.10.5, not read from the library: `strings
# libddsc.so` reports the prefix `dq.builtin`, while the thread's real comm is
# `dq.builtins`. `recvMC` exists only with multicast enabled.
CYCLONEDDS_THREAD_NAMES="dq.builtins dq.user gc recv recvMC recvUC tev"

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

# ── CPU layout ───────────────────────────────────────────────────────────────
compute_cpu_layout
PHYSICAL_CORES="$TOTAL_CORES"

# Populate PHYSICAL_CORE_SLOTS (rt_common.sh::detect_hybrid_capability) so the
# slot→logical-id translator below can map ThreadConfig::cpu_core (a *slot
# index*, not a kernel logical CPU id) into the actual logical CPU that
# ApplyThreadConfig pinned. Without this, SMT-on hybrid (NUC13/14, i9-13900K)
# and AMD SMT systems would erroneously report PASS/FAIL against the wrong cpu
# because slot 1 maps to logical cpu 2 (P-core 1 primary), not cpu 1 (sibling).
detect_hybrid_capability

# slot_to_logical_cpu() is now canonical in rt_common.sh (sourced above) — the
# same translator cpu_shield.sh::get_rt_shield_cpus uses, so the shield set and
# the runtime affinity check agree by construction. (Previously duplicated here.)

# ── 기대값 테이블 (생성물 소비) ──────────────────────────────────────────────
# 형식: "thread_name:expected_slot:expected_policy:expected_priority[:optional]"
# policy: 1=SCHED_FIFO, 0=SCHED_OTHER
#
# IMPORTANT: expected_slot 은 ThreadConfig::cpu_core 와 동일한 *slot index*
# (physical core 번호) 이며, kernel logical CPU id 가 아니다. check_cpu_affinity
# 가 slot_to_logical_cpu() 로 변환해 실제 affinity mask 와 비교한다. SMT-on
# hybrid (NUC13/14, i9-13900K) 에서 slot 1 → logical cpu 2 (P-core 1 primary),
# SMT-off / 4-core CI 에서 slot 1 → logical cpu 1 (identity). 같은 표가 양쪽
# 환경 모두에서 올바르게 동작.
#
# 이 표는 컨트롤러의 **in-process 스레드만** 담는다 (rt_control, rt_callback,
# mpc_main, mpc_worker_*, nrt_logging, nrt_callback) — 전부 pthread_setname_np
# 되어 /proc/<controller>/task 에 보인다. arm_driver / hand_driver 는 launch 가
# taskset 으로 pin 하는 **별도 프로세스**라 여기 있으면 항상 false-WARN 이었고,
# build_external_drivers() + discover_external_drivers() 가 PID 로 발견한다.
#
# optional 필드가 있으면 해당 스레드 미발견 시 WARN 대신 SKIP 처리한다.
#   - mpc_worker_*: MPCThread::Start (rtc_mpc/src/thread/mpc_thread.cpp) 의 worker
#                   생성 lambda 가 ApplyThreadConfig 호출 후 즉시 종료하는 구조라
#                   (parallel solver 가 별도로 thread 를 spawn 하는 패턴)
#                   std::jthread destructor 가 join 하고, verify 의 process
#                   discovery 시점엔 이미 사라져 매칭 불가. 실제 worker 활용
#                   여부는 별도 진단 task.
#   - hand_udp_recv: hand_driver 프로세스 내부의 receive thread.
#                    cpu_core=-1 sentinel — process taskset 으로 affinity 상속,
#                    별도 cpu pin 검증 skip (priority 65 만 확인).
declare -a EXPECTED_THREADS

build_expected_threads() {
  # 표 자체는 rt_common.sh 가 source 하는 thread_layout_generated.sh 의
  # rtc_expected_threads() 가 갖는다 (SSoT: repo_scripts/config/thread_layout.yaml).
  # 여기 tier 표를 다시 적으면 검증기만 옛 기대값을 들고 **PASS 를 낸다** — 센서가
  # 거짓말하는 방향이라 그냥 틀린 것보다 비싸다 (issue #353 이 arm/hand 축에서
  # 겪은 것과 같은 실패이고, #153 M1 이 나머지 축을 같은 자리로 모았다).
  mapfile -t EXPECTED_THREADS < <(rtc_expected_threads "$PHYSICAL_CORES")
}

# ── External driver process 기대값 (arm_driver / hand_driver) ─────────────────
# 이들은 컨트롤러 내부 스레드가 아니라 launch 가 taskset 으로 pin 하는 별도
# 프로세스다 (thread_config.hpp:72-74, thread_utils.hpp SystemThreadConfigs).
# slot 은 build_expected_threads 에서 제거된 arm/hand 행과 동일하게 layout v4.1
# 을 미러링한다: 12c+ arm6/hand7, 10c arm5/hand6, 8c arm4/hand5, 6c arm4/hand4
# (공유), 4c arm0/hand0.
build_external_drivers() {
  # comm 후보 (15-char /proc/*/comm, TASK_COMM_LEN-1):
  #   arm_driver  → ros2_control_node → "ros2_control_no" (UR ros2_control_node)
  #   hand_driver → udp_hand_node (15자 이내라 그대로)
  # sim launch 는 mujoco_simulator_node 만 (sim slot 으로) pin 하고 arm/hand
  # driver 프로세스는 존재하지 않는다 → discovery 가 SKIP (정상). 다른 로봇은
  # RTC_ARM_DRIVER_COMM / RTC_HAND_DRIVER_COMM (공백 구분 후보 리스트) 로 override.
  EXTERNAL_DRIVER_COMMS[arm_driver]="${RTC_ARM_DRIVER_COMM:-ros2_control_no}"
  EXTERNAL_DRIVER_COMMS[hand_driver]="${RTC_HAND_DRIVER_COMM:-udp_hand_node}"

  # "전 스레드가 같은 코어에" 축(EXTERNAL_DRIVER_ALLTHREADS, taskset -a, issue
  # #245)은 은퇴했다. hand_driver 는 #345 로 프로세스 안에서 스레드마다 다른
  # 코어에 붙으므로(hand_aux_io 는 aux 코어) 균일 검사가 정상 배치를 FAIL 로
  # 만들고, arm_driver 는 #343 으로 RT_PRIO 축으로 옮겨갔다. 둘 다 0 이 된
  # 시점에 소비 분기는 도달 불가가 됐다 (#353). 남은 두 축(RT_PRIO /
  # THREAD_LAYOUT)은 여전히 배타적이다.

  # RT-loop 계약 (SCHED_FIFO 우선순위). 값이 있으면 affinity 대신 *그 우선순위의
  # FIFO 스레드* 를 찾아 검사한다 — arm_driver 는 프로세스 affinity 를 봐선 안
  # 된다: main thread 는 executor 이고 500 Hz 루프는 별도 스레드다 (issue #343).
  # 50 은 launch 가 controller_manager 에 주입하는 thread_priority 와 같은 값이며
  # (rtc_tools.launch.cm_rt_params.DEFAULT_THREAD_PRIORITY), 둘은 함께 움직여야
  # 한다. RTC_ARM_DRIVER_RT_PRIO 로 override 가능.
  #
  # hand_driver 는 이 축도 쓰지 않는다 — 단일 RT 스레드가 아니라 세 레인이므로
  # THREAD_LAYOUT 로 검사한다. 65 는 거기 spec 안에 있다.
  EXTERNAL_DRIVER_RT_PRIO[arm_driver]="${RTC_ARM_DRIVER_RT_PRIO:-50}"
  EXTERNAL_DRIVER_RT_PRIO[hand_driver]=""

  # 기대 slot 은 rt_common.sh 의 get_{arm,hand}_driver_slot 이 유일한 shell 출처다.
  # 이 tier 표를 여기 다시 적으면 표가 바뀔 때 검증기만 옛 기대값을 들고 **PASS 를
  # 낸다** — 센서가 거짓말하는 방향이라 그냥 틀린 것보다 비싸다 (issue #353).
  local arm_slot hand_slot
  arm_slot=$(get_arm_driver_slot "$PHYSICAL_CORES")
  hand_slot=$(get_hand_driver_slot "$PHYSICAL_CORES")
  EXTERNAL_DRIVER_SLOTS[arm_driver]=$arm_slot
  EXTERNAL_DRIVER_SLOTS[hand_driver]=$hand_slot

  # ── Per-thread layout 기대 (issue #345) ───────────────────────────────────
  # udp_hand_node 는 세 레인이라 균일 검사도, 단일 RT 스레드 검사도 표현하지
  # 못한다. spec 은 "comm:logical_cpu:policy:prio" 이고, spec 에 없는 TID
  # (executor, DDS 등)는 driver slot 에 있어야 한다.
  #
  # aux 는 OS slot (get_os_cores) 이며 노드의 aux_cpu_slot 기본값과 같은 값이다.
  # 두 곳이 이 사실을 알게 되는 drift 는 감수한 설계 결정이고(#345 D2), 이
  # 주석이 shell 쪽 SSoT 를 가리킨다. RTC_HAND_AUX_SLOT 로 override 가능.
  #
  # <=5 코어 tier 에서는 hand slot 과 OS slot 이 둘 다 0 이라 두 기대가 같은
  # 논리 CPU 로 풀린다 — 예외 처리 없이 통과한다 (그 tier 는 원래 Core 0 에
  # OS/DDS/nrt/arm/hand 가 전부 모이는 degraded 배치다).
  local hand_logical aux_slot aux_logical
  hand_logical=$(slot_to_logical_cpu "$hand_slot")
  aux_slot="${RTC_HAND_AUX_SLOT:-$(get_os_cores)}"
  aux_logical=$(slot_to_logical_cpu "$aux_slot")
  EXTERNAL_DRIVER_THREAD_LAYOUT[hand_driver]="hand_udp_recv:${hand_logical}:SCHED_FIFO:65 hand_aux_io:${aux_logical}:SCHED_OTHER:0"
  EXTERNAL_DRIVER_THREAD_LAYOUT[arm_driver]=""
}

build_expected_threads
build_external_drivers

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
    # "pid N's current affinity mask: 2,00000000" — taskset groups large masks
    # into comma-separated 32-bit words (most-significant first). Strip the label
    # and commas to reassemble the full hex; grabbing only the trailing word
    # (`[0-9a-fA-F]+$`) would drop every CPU ≥ 32 → false FAIL / false migration.
    echo "$out" | sed 's/.*: *//' | tr -d ','
  fi
}

# mask_to_cpus (hex mask → CPU 리스트) 는 rt_common.sh 로 이관됐다 —
# rt_classify_external_rt_threads 가 같은 변환을 쓰는데 그 함수는 fixture 테스트를
# 위해 sourceable 해야 하기 때문. slot_to_logical_cpu 와 같은 이유·같은 자리.

# ── External driver 프로세스 발견 ─────────────────────────────────────────────
# 각 driver 를 process comm 으로 매칭 (pgrep -nx, 정확 comm — rtc_tools.launch.
# pinning 과 동일 matcher). 미발견 = SKIP (sim / arm-only / hand-only 정당).
# EXTERNAL_DRIVER_PIDS[name] 를 채운다 (미발견 시 빈 값).
discover_external_drivers() {
  local name comm pid
  for name in "${EXTERNAL_DRIVER_NAMES[@]}"; do
    pid=""
    for comm in ${EXTERNAL_DRIVER_COMMS[$name]}; do
      pid=$(pgrep -nx "$comm" 2>/dev/null | head -1 || true)
      if [[ -n "$pid" ]]; then
        EXTERNAL_DRIVER_PIDS["$name"]="$pid"
        _pass "external driver 발견: ${name} (PID ${pid}, comm '${comm}')"
        break
      fi
    done
    if [[ -z "$pid" ]]; then
      EXTERNAL_DRIVER_PIDS["$name"]=""
      _skip "external driver 미발견: ${name} (sim / 미구성 — comm 후보: ${EXTERNAL_DRIVER_COMMS[$name]})"
    fi
  done
}

# ══════════════════════════════════════════════════════════════════════════════
# [1/7] Process Discovery
# ══════════════════════════════════════════════════════════════════════════════
check_process_discovery() {
  _section "1/7" "Process Discovery"
  _category_start "process_discovery"

  # Robot bringup의 RT controller exec 찾기. 현재 컨벤션: <robot>_rt_controller
  # (예: integrated_rt_controller). pgrep -f는 전체 커맨드라인에 매칭.
  # ROS2 실행 시 --ros-args 등이 붙으므로 공백 또는 문자열 끝( |$)으로
  # 실행파일명의 끝을 매칭.
  CONTROLLER_PID=$(pgrep -f '(^|/)[a-zA-Z0-9_]+_rt_controller( |$)' 2>/dev/null | head -1 || true)

  if [[ -z "$CONTROLLER_PID" ]]; then
    _fail "RT controller exec(*_rt_controller) 프로세스 미발견 — 제어기가 실행 중이 아닙니다"
    _category_update "process_discovery" "FAIL"
    _category_set_detail "process_discovery" "not running"
    return 1
  fi

  _pass "RT controller PID: ${CONTROLLER_PID}"

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

  # PHYSICAL_CORE_SLOTS 진단: slot index → logical CPU id 룩업 테이블이 어떻게
  # 채워졌는지 한 줄 표시. 사용자가 affinity / migration 결과를 보고 "왜 slot 2
  # 가 cpu 4 로 변환되는가" 같은 의문을 가질 때 즉시 답이 됨. 비어 있으면
  # identity fallback (SMT-off CI / container) 이라는 점 surface.
  if [[ -n "${PHYSICAL_CORE_SLOTS:-}" ]]; then
    _pass "Slot→logical 매핑: [${PHYSICAL_CORE_SLOTS}] (slot index 순서대로 logical CPU id)"
  else
    _warn "PHYSICAL_CORE_SLOTS 미감지 — slot→logical 변환이 identity fallback (sysfs topology 미가용 / 비-Intel 비-hybrid 시스템)"
  fi

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
    # 누락된 필수 스레드 표시
    local _missing_required=0
    for entry in "${EXPECTED_THREADS[@]}"; do
      local ename eopt
      ename=$(echo "$entry" | cut -d: -f1)
      eopt=$(echo "$entry" | cut -d: -f5 -s)
      if [[ -z "${THREAD_TIDS[$ename]:-}" ]]; then
        if [[ "$eopt" == "optional" ]]; then
          _skip "  선택적 스레드 미활성: ${ename}"
        else
          _fail "  미발견: ${ename}"
          ((_missing_required++)) || true
        fi
      fi
    done

    # 필수 스레드 미발견은 FAIL 이다 (issue #344). WARN 이었을 때 이 경로가 정확히
    # 무엇을 감췄는지가 승격 근거다: ApplyThreadConfig 는 ① affinity → ② sched
    # policy → ③ pthread_setname_np 순서인데 ① 실패 시 ②③ 을 건너뛴다. 즉 shield
    # 활성 상태에서 adopt 가 빠지면 RT 스레드가 SCHED_FIFO 를 못 얻는 동시에
    # **이름도 안 붙어**, 이름으로 찾는 이 발견 단계가 실패하고 아래 정책·affinity
    # 검사가 그 스레드들을 통째로 건너뛴다 (total 도 안 센다). 그래서 500 Hz 루프가
    # CFS 로 도는 상태가 WARN 한 줄로 지나갔다.
    if (( _missing_required > 0 )); then
      _fail "thread_config.hpp 필수 스레드 미발견: ${known_count}/${expected_count} — 이름 미설정(ApplyThreadConfig early-return) 의심"
      _category_update "process_discovery" "FAIL"
    else
      _warn "thread_config.hpp 스레드 일부 감지: ${known_count}/${expected_count} (필수는 전부 있음)"
      _category_update "process_discovery" "WARN"
    fi

    # 진단 보조: 필수 스레드가 미발견이면 controller process 의 모든 RT-like
    # thread comm 을 dump. 이름이 박힌 thread 가 어떤 게 있고, EXPECTED 와
    # 어떻게 다른지 즉시 확인 가능 — pthread_setname_np 실패(silent),
    # ApplyThreadConfig early-return (FIFO priority permission 부족, affinity
    # 실패), thread 미생성, race condition 등 가설을 좁힘.
    if (( _missing_required > 0 )); then
      local rt_like_threads="" other_threads=""
      local seen_comms=""
      for tid in "${!THREAD_NAMES[@]}"; do
        local cname="${THREAD_NAMES[$tid]:-}"
        [[ -z "$cname" ]] && continue
        # Dedupe — N threads with the same comm only printed once.
        case " $seen_comms " in
          *" $cname "*) continue ;;
        esac
        seen_comms="${seen_comms} ${cname}"
        # RT-like keyword filter to keep the dump readable.
        if [[ "$cname" =~ (rt_|mpc|callback|control|worker|driver|nrt|spin|exec) ]]; then
          rt_like_threads="${rt_like_threads}    ${cname}"$'\n'
        else
          other_threads="${other_threads}${cname} "
        fi
      done
      if [[ -n "$rt_like_threads" ]]; then
        echo "    [diag] controller process 의 RT-like thread comm (unique):"
        printf '%s' "$rt_like_threads"
      fi
      if [[ -n "$other_threads" ]]; then
        # Other threads — single line, no per-thread output (just so user knows
        # they exist). Truncate to ~120 chars to avoid wrapping noise.
        local other_short="${other_threads:0:120}"
        echo "    [diag] 기타 thread comm: ${other_short}..."
      fi
      echo "    [hint] 실패 가능성:"
      echo "    [hint]   1. realtime 권한 부족 (SCHED_FIFO 90/70 한도 초과) — ulimit -r 확인, @realtime 그룹 멤버십"
      echo "    [hint]   2. controller 가 setname 전에 thread 종료 (sim 모드의 짧은 lifecycle)"
      echo "    [hint]   3. pthread_setname_np truncate (15 chars 초과) — thread_config.hpp 의 .name 길이 확인"
      echo "    [hint]   4. ApplyThreadConfig early-return (affinity / sched 실패) — controller stderr 의 [WARN] 메시지 확인"

      # Auto-diagnose rtprio limit — single most common cause. If the limit is
      # between two SCHED_FIFO priorities used by EXPECTED_THREADS (e.g. 60 ≤
      # limit < 70), only the lower-priority threads succeed and the higher
      # ones silent-fail. Read the controller process's actual limits.
      local controller_rtprio=""
      if [[ -r "/proc/${CONTROLLER_PID}/limits" ]]; then
        controller_rtprio=$(awk '/Max realtime priority/ { print $4 " (soft) / " $5 " (hard)"; exit }' \
                            "/proc/${CONTROLLER_PID}/limits" 2>/dev/null || echo "")
      fi
      if [[ -n "$controller_rtprio" ]]; then
        echo "    [diag] controller PID ${CONTROLLER_PID} 의 Max realtime priority: ${controller_rtprio}"
        # Pull soft limit only for the range check.
        local rtprio_soft
        rtprio_soft=$(echo "$controller_rtprio" | awk '{print $1}')
        if [[ "$rtprio_soft" =~ ^[0-9]+$ ]]; then
          # Highest required FIFO priority across EXPECTED_THREADS (typically 90).
          local max_required_prio=0
          for entry in "${EXPECTED_THREADS[@]}"; do
            local epolicy eprio
            epolicy=$(echo "$entry" | cut -d: -f3)
            eprio=$(echo "$entry" | cut -d: -f4)
            if [[ "$epolicy" -eq 1 && "$eprio" -gt "$max_required_prio" ]]; then
              max_required_prio="$eprio"
            fi
          done
          if (( rtprio_soft < max_required_prio )); then
            echo "    [hint] ↑ rtprio soft limit (${rtprio_soft}) < required (${max_required_prio}). FIFO ${rtprio_soft} 초과 priority 모두 silent-fail."
            echo "    [fix]  sudo bash -c 'echo \"@realtime - rtprio 99\" >> /etc/security/limits.d/99-realtime.conf' (그 후 재로그인)"
            echo "    [fix]  또는 controller 실행 시 'sudo -E' 사용 (개발 환경)"
          fi
        fi
      fi
    fi
  else
    _fail "thread_config.hpp 스레드 감지 실패 (${known_count}/${expected_count})"
    _category_update "process_discovery" "FAIL"
  fi

  # External driver 프로세스(arm_driver / hand_driver)는 컨트롤러 밖의 별도 PID다.
  discover_external_drivers

  local ext_found=0 _n
  for _n in "${EXTERNAL_DRIVER_NAMES[@]}"; do
    if [[ -n "${EXTERNAL_DRIVER_PIDS[$_n]:-}" ]]; then
      ((ext_found++)) || true
    fi
  done
  _category_set_detail "process_discovery" \
    "PID ${CONTROLLER_PID}, ${known_count}/${expected_count} threads, ${ext_found}/${#EXTERNAL_DRIVER_NAMES[@]} ext drivers"
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

    if [[ "$policy_ok" -eq 0 ]]; then
      _fail "${ename} (TID ${tid}): ${actual_policy} (기대값: ${expected_policy_name})"
      _category_update "scheduling_policy" "FAIL"
    elif [[ "$epolicy" -eq 1 && -z "$actual_prio" ]]; then
      # 정책은 맞지만 우선순위를 못 읽음 (chrt 미설치 — /proc fallback 없음).
      # 빈 값을 PASS 로 위장하지 않고 "미검증" WARN 으로 노출한다.
      _warn "${ename} (TID ${tid}): ${actual_policy} (우선순위 확인 불가 — chrt 미설치)"
      _category_update "scheduling_policy" "WARN"
    elif [[ "$prio_ok" -eq 1 ]]; then
      if [[ "$epolicy" -eq 1 ]]; then
        _pass "${ename} (TID ${tid}): ${actual_policy} prio ${actual_prio}"
      else
        _pass "${ename} (TID ${tid}): ${actual_policy}"
      fi
      ((ok++)) || true
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
    local ename eslot
    ename=$(echo "$entry" | cut -d: -f1)
    eslot=$(echo "$entry" | cut -d: -f2)

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

    # Translate slot → logical CPU id via PHYSICAL_CORE_SLOTS, then compute
    # the single-core pin mask. On SMT-on hybrid (NUC13/14, i9-13900K) slot 1
    # maps to logical cpu 2 (P-core 1 primary); on SMT-off CI it stays at 1.
    local elogical
    elogical=$(slot_to_logical_cpu "$eslot")
    local expected_mask_dec=$((1 << elogical))
    local actual_mask_dec=$((16#${mask_hex}))
    local actual_cpus
    actual_cpus=$(mask_to_cpus "$mask_hex")

    # When slot ≠ logical (hybrid SMT-on), surface both numbers so misalignment
    # is easy to diagnose. Otherwise drop the slot annotation to keep output
    # readable on the common SMT-off case.
    local pin_label
    if [[ "$eslot" == "$elogical" ]]; then
      pin_label="Core ${elogical}"
    else
      pin_label="slot ${eslot} → logical cpu ${elogical}"
    fi

    if [[ "$actual_mask_dec" -eq "$expected_mask_dec" ]]; then
      _pass "${ename} (TID ${tid}): ${pin_label} (mask 0x${mask_hex})"
      ((ok++)) || true
    elif (( actual_mask_dec & expected_mask_dec )); then
      _warn "${ename} (TID ${tid}): CPU {${actual_cpus}} (기대값: ${pin_label} only)"
      _category_update "cpu_affinity" "WARN"
    else
      _fail "${ename} (TID ${tid}): CPU {${actual_cpus}} (기대값: ${pin_label})"
      _category_update "cpu_affinity" "FAIL"
    fi
  done

  # ── CycloneDDS 스레드 co-pin (issue #164) ────────────────────────────────────
  # Layout v4.1 은 DDS dispatch 와 rt_callback 이 L1/L2 를 공유하도록 launch 의
  # pin_dds_threads_to_slot 이 DDS 스레드를 rt_callback 코어로 옮긴다. 그런데
  # 지금까지 그 결과를 검사하는 곳이 없었다 — launch 는 옮긴 개수만 찍고,
  # 위 EXPECTED_THREADS 표는 RTC 가 이름을 붙인 스레드만 담는다. 즉 sweep 이
  # 아무것도 못 옮겨도 (타이머가 DDS domain init 보다 빨랐거나 CycloneDDS 가
  # 이름을 바꿨거나) 이 스크립트는 조용히 green 이었다.
  #
  # 이름은 CycloneDDS 0.10.5 실측 (rtc_tools/.../pinning.py CYCLONEDDS_THREAD_NAMES
  # 가 SSoT). recvMC 는 멀티캐스트 활성일 때만 존재하므로 부재는 정상이고,
  # "하나도 없음" 만 문제다 — DDS 를 쓰는 프로세스에 DDS 스레드가 없을 수는 없다.
  local dds_slot="" dds_entry
  for dds_entry in "${EXPECTED_THREADS[@]}"; do
    if [[ "$(echo "$dds_entry" | cut -d: -f1)" == "rt_callback" ]]; then
      dds_slot=$(echo "$dds_entry" | cut -d: -f2)
      break
    fi
  done

  if [[ -n "$dds_slot" ]]; then
    local dds_logical dds_mask_dec dds_pin_label
    dds_logical=$(slot_to_logical_cpu "$dds_slot")
    dds_mask_dec=$((1 << dds_logical))
    if [[ "$dds_slot" == "$dds_logical" ]]; then
      dds_pin_label="Core ${dds_logical}"
    else
      dds_pin_label="slot ${dds_slot} → logical cpu ${dds_logical}"
    fi

    local dds_seen=0 dds_on=0 dds_off=""
    local dtdir dtid dcomm dmask dmask_dec
    for dtdir in "/proc/${CONTROLLER_PID}/task"/*/; do
      dtid=$(basename "$dtdir")
      [[ "$dtid" =~ ^[0-9]+$ ]] || continue
      dcomm=$(cat "${dtdir}comm" 2>/dev/null || echo "")
      case " ${CYCLONEDDS_THREAD_NAMES} " in
        *" ${dcomm} "*) ;;
        *) continue ;;
      esac
      ((dds_seen++)) || true
      dmask=$(get_thread_cpu_affinity "$dtid" 2>/dev/null || echo "")
      [[ -z "$dmask" ]] && continue
      dmask_dec=$((16#${dmask}))
      if [[ "$dmask_dec" -eq "$dds_mask_dec" ]]; then
        ((dds_on++)) || true
      else
        dds_off="${dds_off} ${dcomm}:{$(mask_to_cpus "$dmask")}"
      fi
    done

    ((total++)) || true
    if [[ "$dds_seen" -eq 0 ]]; then
      # DDS 를 쓰는 프로세스에 DDS 스레드가 하나도 없다 = 이름 집합이 실제와
      # 어긋났다는 뜻. sweep 도 같은 집합을 쓰므로 둘 다 침묵했을 것이다.
      _fail "CycloneDDS 스레드 0개 발견 (PID ${CONTROLLER_PID}) — 이름 집합이 실제와 불일치? 기대: ${CYCLONEDDS_THREAD_NAMES}"
      _category_update "cpu_affinity" "FAIL"
    elif [[ "$dds_on" -eq "$dds_seen" ]]; then
      _pass "CycloneDDS 스레드 ${dds_on}/${dds_seen} co-pinned on ${dds_pin_label} (rt_callback)"
      ((ok++)) || true
    else
      _fail "CycloneDDS 스레드 ${dds_on}/${dds_seen} 만 ${dds_pin_label} — off:${dds_off}"
      _category_update "cpu_affinity" "FAIL"
    fi
  fi

  # ── External driver 프로세스 (arm_driver / hand_driver) ──────────────────────
  # 이들은 컨트롤러 밖 별도 PID이며 launch taskset 으로 프로세스-레벨 pin 된다.
  # 미발견은 discovery 에서 이미 SKIP 보고했으므로 여기서는 건너뛴다.
  for ename in "${EXTERNAL_DRIVER_NAMES[@]}"; do
    local pid="${EXTERNAL_DRIVER_PIDS[$ename]:-}"
    [[ -z "$pid" ]] && continue

    local eslot elogical expected_mask_dec pin_label
    eslot="${EXTERNAL_DRIVER_SLOTS[$ename]}"
    elogical=$(slot_to_logical_cpu "$eslot")
    expected_mask_dec=$((1 << elogical))
    if [[ "$eslot" == "$elogical" ]]; then
      pin_label="Core ${elogical}"
    else
      pin_label="slot ${eslot} → logical cpu ${elogical}"
    fi

    ((total++)) || true

    if [[ -n "${EXTERNAL_DRIVER_THREAD_LAYOUT[$ename]:-}" ]]; then
      # Per-thread layout (issue #345): 스레드마다 기대 코어·정책이 다르다.
      # 이름별 policy/priority 검사는 여기가 처음이다 — check_scheduling_policy 는
      # 컨트롤러 내부 EXPECTED_THREADS 만 순회하므로 external driver 의 스레드는
      # 지금까지 어떤 정책으로 도는지 아무도 보지 않았다.
      local lspec="${EXTERNAL_DRIVER_THREAD_LAYOUT[$ename]}"
      local lrecords="" ltid ldir lcomm lpolicy lprio lmask
      for ldir in "/proc/${pid}/task"/*/; do
        ltid=$(basename "$ldir")
        [[ "$ltid" =~ ^[0-9]+$ ]] || continue
        lcomm=$(cat "/proc/${pid}/task/${ltid}/comm" 2>/dev/null || echo "")
        [[ -z "$lcomm" ]] && continue
        lpolicy=$(get_thread_sched_policy "$ltid" 2>/dev/null || echo "")
        [[ -z "$lpolicy" ]] && continue
        lprio=$(get_thread_sched_priority "$ltid" 2>/dev/null || echo "0")
        lmask=$(get_thread_cpu_affinity "$ltid" 2>/dev/null || echo "")
        lrecords+="${ltid} ${lcomm} ${lpolicy} ${lprio} ${lmask}"$'\n'
      done

      local lverdict ldetail
      lverdict=$(printf '%s' "$lrecords" | rt_classify_external_thread_layout "$elogical" "$lspec")
      ldetail="${lverdict#*|}"
      lverdict="${lverdict%%|*}"
      case "$lverdict" in
        OK)
          _pass "${ename} (PID ${pid}): ${ldetail}"
          ((ok++)) || true
          ;;
        NO_THREADS)
          _skip "${ename} (PID ${pid}): ${ldetail}"
          ;;
        *)
          _fail "${ename} (PID ${pid}): [${lverdict}] ${ldetail}"
          _category_update "cpu_affinity" "FAIL"
          ;;
      esac
    elif [[ -n "${EXTERNAL_DRIVER_RT_PRIO[$ename]:-}" ]]; then
      # RT-loop pin (issue #343): 프로세스 affinity 를 보면 안 된다 — main thread
      # 는 executor 이고 제어 루프는 별도 스레드다. 이름이 없으므로
      # (전 TID 가 같은 comm) 설정된 우선순위의 SCHED_FIFO 스레드로 찾는다.
      local eprio="${EXTERNAL_DRIVER_RT_PRIO[$ename]}"
      local records="" rtid rdir rpolicy rprio rmask
      for rdir in "/proc/${pid}/task"/*/; do
        rtid=$(basename "$rdir")
        [[ "$rtid" =~ ^[0-9]+$ ]] || continue
        rpolicy=$(get_thread_sched_policy "$rtid" 2>/dev/null || echo "")
        [[ -z "$rpolicy" ]] && continue
        rprio=$(get_thread_sched_priority "$rtid" 2>/dev/null || echo "0")
        rmask=$(get_thread_cpu_affinity "$rtid" 2>/dev/null || echo "")
        [[ -z "$rmask" ]] && continue
        records+="${rtid} ${rpolicy} ${rprio} ${rmask}"$'\n'
      done

      if [[ -z "$records" ]]; then
        _skip "${ename} (PID ${pid}): 스레드 상태 읽기 실패"
        continue
      fi

      local verdict detail
      verdict=$(printf '%s' "$records" | rt_classify_external_rt_threads "$eprio" "$elogical")
      detail="${verdict#*|}"
      verdict="${verdict%%|*}"

      case "$verdict" in
        OK)
          _pass "${ename} (PID ${pid}): RT loop ${detail} (${pin_label})"
          ((ok++)) || true
          ;;
        NO_FIFO)
          # FIFO 요청이 EPERM 으로 거부돼도 affinity 는 적용된다 — 그래서 위치만
          # 보는 검사는 이 상태를 통과시킨다. realtime 그룹 / RLIMIT_RTPRIO 확인.
          _fail "${ename} (PID ${pid}): SCHED_FIFO 스레드 없음 (${detail}) — RT 권한 미설정? 제어 루프가 CFS 로 강등됐다"
          _category_update "cpu_affinity" "FAIL"
          ;;
        WRONG_PRIO)
          _fail "${ename} (PID ${pid}): ${detail} — 주입한 thread_priority 와 불일치"
          _category_update "cpu_affinity" "FAIL"
          ;;
        *)
          _fail "${ename} (PID ${pid}): RT loop ${detail} (기대값: ${pin_label})"
          _category_update "cpu_affinity" "FAIL"
          ;;
      esac
    else
      # main-thread-only pin: taskset -p <pid> 가 main thread mask 를 준다.
      local mask_hex
      mask_hex=$(get_thread_cpu_affinity "$pid" 2>/dev/null || echo "")
      if [[ -z "$mask_hex" ]]; then
        _skip "${ename} (PID ${pid}): affinity 읽기 실패"
        continue
      fi
      local actual_mask_dec=$((16#${mask_hex}))
      local actual_cpus
      actual_cpus=$(mask_to_cpus "$mask_hex")
      if [[ "$actual_mask_dec" -eq "$expected_mask_dec" ]]; then
        _pass "${ename} (PID ${pid}): ${pin_label} (mask 0x${mask_hex})"
        ((ok++)) || true
      elif (( actual_mask_dec & expected_mask_dec )); then
        _warn "${ename} (PID ${pid}): CPU {${actual_cpus}} (기대값: ${pin_label} only)"
        _category_update "cpu_affinity" "WARN"
      else
        _fail "${ename} (PID ${pid}): CPU {${actual_cpus}} (기대값: ${pin_label})"
        _category_update "cpu_affinity" "FAIL"
      fi
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

  # ── External driver 의 mlockall (issue #343) ────────────────────────────────
  # launch 가 controller_manager 에 lock_memory:true 를 주입하므로 arm_driver 는
  # 자기 페이지를 잠가야 한다. RT_PRIO 계약이 있는 드라이버 = 프로세스 안에서
  # RT 루프를 도는 드라이버이므로 같은 조건을 건다. 실패는 WARN 이다 — FIFO 와
  # 달리 mlockall 부재는 지터를 키울 뿐 루프를 강등시키지는 않는다.
  local ename
  for ename in "${EXTERNAL_DRIVER_NAMES[@]}"; do
    local epid="${EXTERNAL_DRIVER_PIDS[$ename]:-}"
    [[ -z "$epid" ]] && continue
    [[ -z "${EXTERNAL_DRIVER_RT_PRIO[$ename]:-}" ]] && continue

    local evmlck
    evmlck=$(grep "^VmLck:" "/proc/${epid}/status" 2>/dev/null | awk '{print $2}')
    if [[ -z "$evmlck" ]]; then
      _skip "${ename} (PID ${epid}): VmLck 정보 없음"
    elif [[ "$evmlck" -gt 0 ]]; then
      _pass "${ename} (PID ${epid}): mlockall 적용됨 (VmLck = ${evmlck} kB)"
    else
      _warn "${ename} (PID ${epid}): mlockall 미적용 (VmLck = 0) — lock_memory 주입 실패?"
      _category_update "memory_locking" "WARN"
    fi
  done

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
    local ename eslot epolicy
    ename=$(echo "$entry" | cut -d: -f1)
    eslot=$(echo "$entry" | cut -d: -f2)
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

    # Translate slot → logical CPU id via PHYSICAL_CORE_SLOTS so comparison is
    # against the actual logical CPU that ApplyThreadConfig pinned. Without
    # this, SMT-on hybrid hosts report false "core migration" because slot 2
    # is mistaken for logical cpu 2 instead of the P-physical it maps to.
    local elogical
    elogical=$(slot_to_logical_cpu "$eslot")

    local pin_label
    if [[ "$eslot" == "$elogical" ]]; then
      pin_label="Core ${elogical}"
    else
      pin_label="slot ${eslot} → logical cpu ${elogical}"
    fi

    if [[ "$current_cpu" -eq "$elogical" ]]; then
      _pass "${ename} (TID ${tid}): 현재 CPU ${current_cpu} (기대값: ${pin_label})"
      ((ok++)) || true
    else
      _fail "${ename} (TID ${tid}): 현재 CPU ${current_cpu} (기대값: ${pin_label}) — 코어 이동 발생!"
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

  # RT throttle event 확인. sched_debug 는 rt_rq 마다 `.rt_throttled : 0` 필드를
  # 출력하므로 `grep -c "rt_throttled"` 는 *필드 존재*(≈nCPU, 항상 >0)를 세어
  # 튜닝 안 된 모든 머신에서 오탐한다. 값이 실제로 1 이상인 줄만 센다.
  if [[ -r "/proc/sched_debug" ]]; then
    local throttled_count
    throttled_count=$(grep -cE 'rt_throttled[[:space:]]*:[[:space:]]*[1-9]' /proc/sched_debug 2>/dev/null) || throttled_count=0
    if [[ "$throttled_count" -gt 0 && "$rt_runtime" != "-1" ]]; then
      _warn "RT 쓰로틀링 이벤트 감지 (${throttled_count}개 rt_rq 에서 rt_throttled≥1)"
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
      print_category_row "${categories[$i]}" "${labels[$i]}" 20
    done
  fi

  if [[ "$OUTPUT_MODE" != "json" ]]; then
    print_result_line
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
  emit_json_categories "${categories[@]}"
  echo "  },"

  # 스레드 상세 정보 (verify_rt_runtime 고유)
  echo "  \"threads\": {"
  local first=1
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

  # External driver 프로세스 상세 (verify_rt_runtime 고유)
  echo "  \"external_drivers\": {"
  local efirst=1
  for ename in "${EXTERNAL_DRIVER_NAMES[@]}"; do
    local eslot pid
    eslot="${EXTERNAL_DRIVER_SLOTS[$ename]}"
    pid="${EXTERNAL_DRIVER_PIDS[$ename]:-}"
    [[ "$efirst" -eq 0 ]] && echo ","
    # rt_loop_prio: 이 드라이버가 RT-loop 축으로 검사됐으면 그 우선순위, 아니면
    # null. actual_affinity 는 main thread 값이므로 rt_loop_prio 가 non-null 인
    # 드라이버에서는 판정 근거가 아니다 (issue #343) — 진단용으로만 남긴다.
    local eprio_json="${EXTERNAL_DRIVER_RT_PRIO[$ename]:-}"
    [[ -z "$eprio_json" ]] && eprio_json="null"
    # thread_layout 은 판정에 실제로 쓰인 spec 이라 null 이 아니면 rt_loop_prio
    # 보다 그쪽이 근거다 (issue #345).
    local elayout_json="${EXTERNAL_DRIVER_THREAD_LAYOUT[$ename]:-}"
    if [[ -z "$elayout_json" ]]; then
      elayout_json="null"
    else
      elayout_json="\"${elayout_json}\""
    fi
    if [[ -n "$pid" ]]; then
      local amask
      amask=$(get_thread_cpu_affinity "$pid" 2>/dev/null || echo "0")
      printf "    \"%s\": {\"pid\": %s, \"expected_slot\": %s, \"rt_loop_prio\": %s, \"thread_layout\": %s, \"main_thread_affinity\": \"0x%s\"}" \
        "$ename" "$pid" "$eslot" "$eprio_json" "$elayout_json" "${amask:-0}"
    else
      printf "    \"%s\": {\"pid\": null, \"expected_slot\": %s, \"rt_loop_prio\": %s, \"thread_layout\": %s}" \
        "$ename" "$eslot" "$eprio_json" "$elayout_json"
    fi
    efirst=0
  done
  echo ""
  echo "  },"
  emit_json_summary
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
  EXTERNAL_DRIVER_PIDS=()   # watch 모드 재실행마다 PID 재발견 (slot/comm 은 불변)

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

# Executed → run. Sourced → define only, so test_rt_common.sh can call
# build_external_drivers() and assert the expected slots against
# get_{arm,hand}_driver_slot directly. Without this guard that check could only
# be a text grep, which cannot tell a live call from a comment mentioning one
# (issue #353).
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  main
fi
