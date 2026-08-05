#!/bin/bash
# cpu_shield.sh — 동적 CPU 격리 관리 (모드별 tiered isolation)
#
# cset shield를 사용하여 런타임에 CPU 격리를 on/off 할 수 있다.
# isolcpus GRUB 파라미터 없이도 RT 코어를 동적으로 보호한다.
#
# Usage:
#   sudo cpu_shield.sh on [--robot|--sim]  # 격리 활성화
#   sudo cpu_shield.sh off                 # 격리 해제
#   cpu_shield.sh status                   # 상태 확인 (sudo 불필요)
#
# Shield model (layout v4.1):
#   shielded = RT controller cores + MPC cores (rt_control Core 1,
#              rt_callback Core 2, mpc_main + workers from Core 3)
#   released = OS/DDS (Core 0), nrt_*, arm_driver, hand_driver,
#              sim_thread/viewer (cpu_core=-1, no pin), spare
#
# Modes:
#   --robot (default): same shield range as --sim. driver processes sit on their
#                      own cores outside the shield and run on SCHED_OTHER — they
#                      do not need cset protection. (The UR driver's internal
#                      control loop is FIFO 50 and is pinned by controller_manager
#                      parameters, not taskset — issue #343; it is an accepted
#                      degraded exception that its core is not cset-isolated.)
#   --sim:             identical shield. MuJoCo physics + viewer roam over the
#                      released cores under CFS (cpu_core=-1 on every tier).
#
# 실행 시점:
#   - 로봇 런치: ur_control.launch.py에서 자동 호출
#   - 시뮬 런치: mujoco_sim.launch.py에서 자동 호출
#   - 빌드 전:   build.sh / install.sh에서 자동 해제

set -euo pipefail

# ── 공통 라이브러리 로드 ────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/rt_common.sh
source "${SCRIPT_DIR}/lib/rt_common.sh"
make_logger "SHIELD"

# ── Compute shield cores based on mode and core count ─────────────────────
#
# Layout v4.1: the "user" cpuset shields RT controller threads (rt_control
# Core 1, rt_callback Core 2) plus MPC main + workers (Core 3 + 4 + 5).
# Process-level driver cores (arm_driver, hand_driver) are *not* shielded —
# the driver processes themselves run there under SCHED_OTHER, so isolating
# them from "user" tasks gains nothing and wastes a core. The one RT thread on
# those cores (the UR driver's FIFO 50 control loop, and the hand CommLoop's
# FIFO 65) therefore gets affinity + priority but no cpuset isolation — an
# accepted degraded exception, gated on measurement (issue #343). sim_thread /
# viewer cores are -1 (no pin) on every tier; MuJoCo roams over the released
# cores under CFS.
#
# Both modes shield the same range now (RT + MPC only). The legacy "Tier 1
# vs Tier 1+2" distinction is gone — driver/IO cores are SCHED_OTHER and do
# not need cset protection. The function signature keeps the mode argument
# for forward compatibility and so callers (`on --sim`, `on --robot`) need
# no further changes.
#
# SSoT: get_cm_shield_cpus() in rt_common.sh. get_rt_cores() alone returns slot
# indices, NOT logical CPU ids — on SMT/hybrid the RT threads pin to logical
# cpus that differ from the slot numbers (slot→logical via PHYSICAL_CORE_SLOTS,
# same map C++ ApplyThreadConfig uses), and their HT siblings must be shielded
# too.
#
# The cpuset must fit the WHOLE integrated_rt_controller process, because
# do_on moves that process into the "user" cpuset (issue #151) and cset moves a
# *process* — all its threads. The CM hosts nrt_logging / nrt_callback too, and
# on the NUC13 hybrid those pin to logical {12,13}, outside the RT-only span
# 2-9. get_cm_shield_cpus() = get_rt_shield_cpus() ∪ nrt cores (OS slot dropped)
# so nrt self-pin does not EINVAL against a too-narrow cpuset. Compact range
# notation (e.g. "1-3,5" non-SMT 6c, "2-9,12-13" NUC13 hybrid 12c).
compute_shield_cores() {
  # Args $1 (mode) and $2 (phys_cores) are accepted for forward compat with
  # legacy callers; both modes share the same shield range, and get_cm_shield_cpus
  # auto-detects core count + topology via rt_common.sh.
  get_cm_shield_cpus
}

# ── Status mode marker file ──────────────────────────────────────────────────
SHIELD_MODE_FILE="/tmp/cpu_shield_mode"

# ── Stale-shield detection (issue #349 D14) ──────────────────────────────────
#
# The reason this exists: `do_on` used to treat "a user cpuset exists" as "the
# shield is correct" and return early, comparing only the MODE string in
# SHIELD_MODE_FILE. It computed the desired core list, never compared it, and
# then printed it — so after a layout change a re-launch logged the new,
# narrower core list while the kernel still held the old wide one. The shield
# does not shrink and the log actively claims it did.
#
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

# Exit 0 when the active shield already matches `desired` for `mode`.
#
# Exit 2 is the narrow middle case: the cores are exactly right and only the
# mode marker disagrees (missing file after a /tmp cleanup, or a previous run
# in the other mode). It is separated from exit 1 because the remedy differs —
# rewriting a marker costs nothing, whereas `cset shield --reset` evicts every
# process already adopted into the user cpuset. Both modes currently compute
# the same core range, so a mode-only mismatch never implies a wrong mask.
#
# Exit 1 for everything else — no shield, unreadable mask, different cores —
# so the caller reconfigures.
shield_matches_desired() {
  local mode="${1:-robot}" desired="${2:-}"
  local want have
  want=$(normalise_cpu_set "$desired") || return 1
  [[ -z "$want" ]] && return 1
  have=$(shield_actual_user_cpus) || return 1
  [[ -z "$have" ]] && return 1
  [[ "$want" == "$have" ]] || return 1

  # Cores agree; the mode marker still decides, because --sim and --robot may
  # diverge again later and a stale marker would misreport `status`.
  local current_mode=""
  [[ -f "$SHIELD_MODE_FILE" ]] && current_mode=$(cat "$SHIELD_MODE_FILE" 2>/dev/null || echo "")
  [[ "$current_mode" == "$mode" ]] || return 2
}

# ── Command: on ──────────────────────────────────────────────────────────────
do_on() {
  local mode="${1:-robot}"

  if [[ "$EUID" -ne 0 ]]; then
    fatal "Root privileges required. Run: sudo $0 on --${mode}"
  fi

  local phys_cores
  phys_cores=$(get_physical_cores)
  if [[ "$phys_cores" -lt 6 ]]; then
    warn "물리 코어 ${phys_cores}개 (<6) — degraded layout: RT 결정성이 보장되지 않는다 (RTC 권장 ≥6코어)."
  fi
  local shield_cores
  shield_cores=$(compute_shield_cores "$mode" "$phys_cores")

  # Already shielded with exactly these cores, in this mode? Then nothing to
  # do. Any other state — including "a shield exists but we cannot read its
  # mask" — falls through to a reset + re-apply (issue #349 D14). The old
  # version compared only the mode string, so a layout change left the wide
  # shield in place while logging the narrow one.
  if command -v cset &>/dev/null; then
    local match_rc=0
    shield_matches_desired "$mode" "$shield_cores" || match_rc=$?
    if [[ "$match_rc" -eq 0 ]]; then
      info "Shield already active and current (mode: ${mode}, cores: ${shield_cores})"
      return 0
    fi
    if [[ "$match_rc" -eq 2 ]]; then
      # Cores are already exactly right; only the marker is stale or gone.
      # Tearing the shield down here would evict every adopted process to fix
      # a text file, and the old message said the shield "differs" while
      # printing the identical core list twice.
      info "Shield cores already correct (${shield_cores}); refreshing mode marker → ${mode}"
      echo "$mode" >"$SHIELD_MODE_FILE"
      return 0
    fi
    local actual
    actual=$(shield_actual_user_cpus || true)
    if [[ -n "$actual" ]]; then
      info "Existing shield differs (active: ${actual} / desired: ${shield_cores}) — reconfiguring..."
      cset shield --reset 2>/dev/null || true
    elif cset shield -s 2>/dev/null | grep -qi 'user'; then
      # A shield is up but its mask is unreadable. Rebuild rather than assume.
      warn "Existing shield present but its cpu mask could not be read — reconfiguring to be sure."
      cset shield --reset 2>/dev/null || true
    fi
  fi

  info "Activating CPU shield (mode: ${mode}, cores: ${shield_cores}, ${phys_cores}-core system)"

  # Disable RT throttling (sched_rt_runtime_us=-1) so RT threads can use 100% CPU
  local current_rt_runtime
  current_rt_runtime=$(sysctl -n kernel.sched_rt_runtime_us 2>/dev/null || echo "unknown")
  if [[ "$current_rt_runtime" != "-1" ]]; then
    sysctl -w kernel.sched_rt_runtime_us=-1 >/dev/null 2>&1 || true
    info "sched_rt_runtime_us: ${current_rt_runtime} → -1 (RT throttling disabled)"
  fi

  # Try cset shield first
  if command -v cset &>/dev/null; then
    if cset shield --cpu="${shield_cores}" --kthread=on 2>/dev/null; then
      echo "$mode" > "$SHIELD_MODE_FILE"
      success "cset shield activated: Core ${shield_cores} isolated"
      success "Mode: ${mode} (${phys_cores}-core system)"
      return 0
    else
      warn "cset shield failed — trying cpuset fallback"
    fi
  fi

  # cset 없이는 실질 격리 불가 — 허상 success 금지 (P1-2).
  # 이전 cgroup fallback 은 rt_shield cpuset 에 cpuset.cpus 만 세팅하고 (a) 어떤
  # 태스크도 그 안으로 옮기지 않았으며 (b) RT 코어에서 다른 태스크를 밀어내는
  # complement(housekeeping) cpuset 도 만들지 않아 격리 효과가 0 이면서 success
  # 를 보고했다. RT-HOST-3 를 못 지키면서 성공으로 위장하지 않도록 명확히 실패한다.
  # (올바른 cgroup 격리 = 전 태스크 housekeeping cpuset 이주 + cpu_exclusive; 실기
  # 검증이 필요한 별도 작업. do_off 는 legacy rt_shield 정리를 위해 유지.)
  fatal "cset 미설치 — CPU shield 를 적용할 수 없다. cgroup fallback 은 실질 격리를 제공하지 못해 비활성화됨. 설치: sudo apt-get install -y cpuset (RT 코어 격리 없이 제어 루프 실행 시 RT-HOST-3 격리 보장이 깨진다)"
}

# ── Command: adopt <pid> ─────────────────────────────────────────────────────
# Move an already-running process (+ all its threads) INTO the shield's "user"
# cpuset. `cset shield --cpu` only *creates* the isolated set and sweeps
# existing tasks into "system"; a process launched afterwards (the RT
# controller) inherits "system" and is barred from the RT logical CPUs, so its
# pthread_setaffinity_np(logical 2/4/…) returns EINVAL — and thread_utils.hpp
# aborts the whole thread config (losing SCHED_FIFO too) on that failure.
# The launch calls this on the RT controller's OnProcessStart, *before* the
# controller activates and spawns its RT threads, so those threads inherit the
# "user" cpuset and pin successfully. Idempotent, and a no-op-with-warning when
# no shield is active (fake-hw / no-sudo / cset-absent runs). Issue #151.
do_adopt() {
  local pid="${1:-}"

  if [[ -z "$pid" || ! "$pid" =~ ^[0-9]+$ ]]; then
    fatal "adopt requires a numeric PID. Usage: sudo $0 adopt <pid>"
  fi
  if [[ "$EUID" -ne 0 ]]; then
    fatal "Root privileges required. Run: sudo $0 adopt ${pid}"
  fi
  if ! command -v cset &>/dev/null; then
    warn "cset 미설치 — PID ${pid} 를 user cpuset 으로 이동할 수 없음 (adopt skip)"
    return 0
  fi
  if [[ ! -d "/proc/${pid}" ]]; then
    warn "PID ${pid} 없음 — adopt skip (프로세스가 이미 종료?)"
    return 0
  fi
  # Shield must be active (a "user" cpuset must exist) to adopt into it.
  if ! cset shield -s 2>/dev/null | grep -q "user"; then
    warn "활성 shield 없음 — PID ${pid} adopt skip (shield off 상태에서는 CM 이 full affinity 로 self-pin)"
    return 0
  fi

  # Diagnostic + guard: report what we are about to move. The caller resolves
  # the PID via `pgrep -f`, which historically matched the launch's own wrapper
  # bash instead of the RT node (issue #151). If comm is a shell, the wrong PID
  # was handed in — moving it is a no-op that leaves the node in "system", so
  # warn loudly rather than silently reporting success.
  local pcomm
  pcomm=$(cat "/proc/${pid}/comm" 2>/dev/null || echo "?")
  info "adopt target PID ${pid}: comm='${pcomm}'"
  if [[ "$pcomm" == "bash" || "$pcomm" == "sh" || "$pcomm" == "cpu_shield.sh" ]]; then
    warn "adopt target comm='${pcomm}' 는 launch wrapper 로 보인다 — RT 노드가 아님. PID 해석 오류(pgrep self-match?) 로 실제 노드는 system 에 남아 EINVAL 가능. 이동은 진행하되 검증 필요."
  fi

  # --threads moves every thread of the process; future threads inherit the
  # cpuset cgroup. `--shield --pid` moves INTO the "user" set (opposite of the
  # default sweep to "system").
  if cset shield --shield --pid "$pid" --threads >/dev/null 2>&1; then
    success "PID ${pid} (comm='${pcomm}', +threads) moved into user cpuset — RT self-pin will land in-set"
    return 0
  fi
  # 여기까지 왔다 = shield 가 활성인데 이동에 실패했다. 위의 return 0 들과 달리
  # 이건 양성 no-op 이 아니다 — 호출자가 그대로 activate 하면 RT 스레드가 affinity
  # 와 SCHED_FIFO 를 함께 잃은 채 (setaffinity EINVAL → ApplyThreadConfig 가
  # SCHED_FIFO 설정 전에 return) 500 Hz 로 돈다. non-zero 로 알린다 (issue #344).
  # warn (error 아님): rt_common.sh 의 error() 는 exit 1 이라 함수의 return 계약을
  # 깬다. 심각도는 반환 코드가 나르고, 호출자(런치)가 ERROR 로 표면화한다.
  warn "PID ${pid} adopt 실패 (cset shield --shield) — RT pin 이 EINVAL 난다. 호출자는 activate 를 거부해야 한다"
  return 1
}

# ── Command: off ─────────────────────────────────────────────────────────────
do_off() {
  if [[ "$EUID" -ne 0 ]]; then
    fatal "Root privileges required. Run: sudo $0 off"
  fi

  # Restore default RT throttling (950000µs per 1000000µs period = 95%)
  local current_rt_runtime
  current_rt_runtime=$(sysctl -n kernel.sched_rt_runtime_us 2>/dev/null || echo "unknown")
  if [[ "$current_rt_runtime" == "-1" ]]; then
    sysctl -w kernel.sched_rt_runtime_us=950000 >/dev/null 2>&1 || true
    info "sched_rt_runtime_us: -1 → 950000 (RT throttling restored)"
  fi

  # Try cset shield reset
  if command -v cset &>/dev/null; then
    if cset shield --reset 2>/dev/null; then
      rm -f "$SHIELD_MODE_FILE"
      success "cset shield deactivated — all cores available"
      return 0
    fi
  fi

  # Fallback: remove cpuset rt_shield (cgroup v1 or v2)
  local shield_dir=""
  if [[ -d /sys/fs/cgroup/rt_shield ]]; then
    shield_dir="/sys/fs/cgroup/rt_shield"
  elif [[ -d /sys/fs/cgroup/cpuset/rt_shield ]]; then
    shield_dir="/sys/fs/cgroup/cpuset/rt_shield"
  fi

  if [[ -n "$shield_dir" && -d "$shield_dir" ]]; then
    # Move tasks back to parent cgroup
    local tasks_file="${shield_dir}/cgroup.procs"
    [[ -f "$tasks_file" ]] || tasks_file="${shield_dir}/tasks"
    local parent_tasks
    parent_tasks="$(dirname "$shield_dir")/cgroup.procs"
    [[ -f "$parent_tasks" ]] || parent_tasks="$(dirname "$shield_dir")/tasks"

    if [[ -f "$tasks_file" && -f "$parent_tasks" ]]; then
      while IFS= read -r pid; do
        echo "$pid" > "$parent_tasks" 2>/dev/null || true
      done < "$tasks_file" 2>/dev/null || true
    fi
    rmdir "$shield_dir" 2>/dev/null || true
    rm -f "$SHIELD_MODE_FILE"
    success "cpuset fallback shield removed — all cores available"
    return 0
  fi

  # Check if isolcpus is in effect
  if grep -q "isolcpus=" /proc/cmdline 2>/dev/null; then
    warn "isolcpus GRUB parameter active — cannot disable without reboot"
    warn "Remove isolcpus from GRUB and switch to cset shield (recommended)"
    return 1
  fi

  info "No active shield found"
  rm -f "$SHIELD_MODE_FILE"
  return 0
}

# ── Command: status ──────────────────────────────────────────────────────────
do_status() {
  local phys_cores
  phys_cores=$(get_physical_cores)
  local logical_cores
  logical_cores=$(nproc --all)
  local available_cores
  available_cores=$(nproc)
  local isolated
  isolated=$(cat /sys/devices/system/cpu/isolated 2>/dev/null || echo "")

  echo ""
  info "CPU Shield Status"
  info "  Physical cores: ${phys_cores}"
  info "  Logical cores:  ${logical_cores}"
  info "  Available cores: ${available_cores}"

  if [[ -n "$isolated" ]]; then
    info "  Isolated cores: ${isolated}"

    # Detect isolation method
    if grep -q "isolcpus=" /proc/cmdline 2>/dev/null; then
      info "  Method: isolcpus (GRUB — static, reboot required to change)"
      warn "  Recommendation: switch to cset shield for dynamic isolation"
    elif command -v cset &>/dev/null && cset shield -s 2>/dev/null | grep -q "user"; then
      local shield_mode="unknown"
      if [[ -f "$SHIELD_MODE_FILE" ]]; then
        shield_mode=$(cat "$SHIELD_MODE_FILE" 2>/dev/null || echo "unknown")
      fi
      info "  Method: cset shield (dynamic)"
      info "  Mode: ${shield_mode}"
    else
      info "  Method: unknown (cpuset or other)"
    fi
  else
    info "  Isolated cores: none"
    info "  All ${available_cores} cores available for general use"
  fi

  # Show expected layout for this core count (SSoT: rt_common::print_thread_layout)
  echo ""
  print_thread_layout "$phys_cores"
  echo ""
}

# ── Usage ────────────────────────────────────────────────────────────────────
usage() {
  echo ""
  echo "Usage: cpu_shield.sh <command> [options]"
  echo ""
  echo "Commands:"
  echo "  on [--robot|--sim]  Activate CPU isolation (default: --robot)"
  echo "    --robot           Shield RT + MPC cores (driver cores released)"
  echo "    --sim             Same shield as --robot; sim_thread runs outside"
  echo "  check [--robot|--sim]  Exit 0 if the active shield already matches this mode"
  echo "                      (no root needed; non-zero also means \"cannot tell\")"
  echo "  adopt <pid>         Move a running process (+threads) into the user cpuset"
  echo "  off                 Deactivate CPU isolation"
  echo "  status              Show current isolation status"
  echo ""
  echo "Examples:"
  echo "  sudo cpu_shield.sh on --robot    # Full RT isolation"
  echo "  sudo cpu_shield.sh on --sim      # Lightweight simulation isolation"
  echo "  sudo cpu_shield.sh adopt 12345   # Put RT controller PID under the shield"
  echo "  sudo cpu_shield.sh off           # Release all isolation"
  echo "  cpu_shield.sh status             # Check current state"
  echo ""
  exit 1
}

# ── Main ─────────────────────────────────────────────────────────────────────
COMMAND="${1:-}"
shift || true

case "$COMMAND" in
  on)
    MODE="robot"
    while [[ $# -gt 0 ]]; do
      case "$1" in
        --robot) MODE="robot"; shift ;;
        --sim)   MODE="sim"; shift ;;
        *)       error "Unknown option: $1"; usage ;;
      esac
    done
    do_on "$MODE"
    ;;
  check)
    # Root-free query: does the ACTIVE shield already match what `on <mode>`
    # would build? Exit 0 = yes (caller can skip sudo), non-zero = no / unknown.
    # Exists so the launch helper has exactly one implementation of the mask
    # comparison to consult instead of a second copy of the logic (#349 D14).
    MODE="robot"
    while [[ $# -gt 0 ]]; do
      case "$1" in
        --robot) MODE="robot"; shift ;;
        --sim)   MODE="sim"; shift ;;
        *)       error "Unknown option: $1"; usage ;;
      esac
    done
    shield_matches_desired "$MODE" "$(compute_shield_cores "$MODE" "$(get_physical_cores)")"
    ;;
  adopt)
    do_adopt "${1:-}"
    ;;
  off)
    do_off
    ;;
  status)
    do_status
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    error "Unknown command: ${COMMAND:-<none>}"
    usage
    ;;
esac
