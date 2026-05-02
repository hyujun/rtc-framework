#!/usr/bin/env bash
# perf_record.sh — launch-side wrapper that records `perf` data for the active
# robot/sim processes and saves it under the current session's perf/ subdir.
#
# Designed to be invoked as a child of `ros2 launch` via ExecuteProcess. The
# parent sends SIGINT on shutdown; we forward it to perf so it flushes the
# trace cleanly.
#
# Usage:
#   perf_record.sh <output_path> <pid_pattern_regex> [--duration <sec>]
#                  [--frequency <hz>] [--off-cpu]
#
# Example (called from launch):
#   perf_record.sh \
#       /path/to/logging_data/260502_2330/perf/perf.data \
#       'ur5e_rt_controller|mpc_main|mujoco_simulator_node'
#
# Permission model (matches cpu_shield.sh / ptrace_scope policy):
#   * perf_event_paranoid <= 1 → run as user, no sudo
#   * paranoid >  1 → require passwordless sudo, else warn + exit 0
#
# Exit codes:
#   0  — perf finished cleanly OR pre-flight skipped (launch must continue)
#   2  — invalid arguments

set -uo pipefail

# ── Args ──────────────────────────────────────────────────────────────────────
if [[ $# -lt 2 ]]; then
  echo "[perf_record] usage: $0 <output_path> <pid_pattern> [--duration N] [--frequency N] [--off-cpu]" >&2
  exit 2
fi

OUTPUT="$1"
PID_PATTERN="$2"
shift 2

DURATION=""
FREQUENCY="999"
OFF_CPU=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration) DURATION="$2"; shift 2 ;;
    --frequency) FREQUENCY="$2"; shift 2 ;;
    --off-cpu) OFF_CPU=1; shift ;;
    *) echo "[perf_record] unknown arg: $1" >&2; exit 2 ;;
  esac
done

log() { echo "[perf_record] $*" >&2; }

# ── Pre-flight: perf binary ───────────────────────────────────────────────────
if ! command -v perf >/dev/null 2>&1; then
  log "perf not on PATH — install with: sudo apt install linux-tools-\$(uname -r) hotspot"
  log "or run ./install.sh --perf. Skipping perf capture, launch continues."
  exit 0
fi

# ── Pre-flight: target PIDs (poll for up to 30 s while lifecycle activates) ──
# `pgrep -f` matches the full command line — but the pattern itself is in OUR
# argv (and possibly our parent shell's argv that invoked us via `bash -c`),
# so naive matches include self / parent / sibling shells. Filter those out by
# rejecting any PID whose cmdline contains "perf_record.sh".
SCRIPT_NAME="$(basename -- "$0")"

PIDS=""
for _ in $(seq 1 30); do
  RAW="$(pgrep -f -- "$PID_PATTERN" 2>/dev/null || true)"
  FILTERED=""
  for pid in $RAW; do
    if [[ ! -r "/proc/$pid/cmdline" ]]; then
      continue  # PID gone or unreadable — race with pgrep
    fi
    cmdline="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || echo '')"
    # Drop matches that reference our wrapper script or are bash subshells
    # carrying our pattern as a literal arg (Claude/CI invocation style).
    if [[ "$cmdline" == *"$SCRIPT_NAME"* ]] || [[ "$cmdline" == *"$PID_PATTERN"* ]]; then
      continue
    fi
    FILTERED="${FILTERED:+$FILTERED,}$pid"
  done
  PIDS="$FILTERED"
  [[ -n "$PIDS" ]] && break
  sleep 1
done

if [[ -z "$PIDS" ]]; then
  log "no PIDs matched /$PID_PATTERN/ within 30 s — skipping perf capture"
  exit 0
fi

log "matched PIDs: $PIDS"

# ── Pre-flight: privilege model ───────────────────────────────────────────────
PARANOID="$(cat /proc/sys/kernel/perf_event_paranoid 2>/dev/null || echo 4)"
USE_SUDO=0

if [[ "$PARANOID" -le 1 ]]; then
  log "perf_event_paranoid=$PARANOID — running as user"
elif sudo -n true 2>/dev/null; then
  log "perf_event_paranoid=$PARANOID — using passwordless sudo"
  USE_SUDO=1
else
  log "perf_event_paranoid=$PARANOID requires sudo, but passwordless not configured."
  log "Fix permanently: ./install.sh --perf  (sets paranoid=1 via /etc/sysctl.d/99-perf.conf)"
  log "Or run launch with sudo. Skipping perf capture."
  exit 0
fi

# ── Output dir ────────────────────────────────────────────────────────────────
OUT_DIR="$(dirname "$OUTPUT")"
mkdir -p "$OUT_DIR" || { log "cannot create $OUT_DIR"; exit 0; }

# ── Build command ─────────────────────────────────────────────────────────────
PERF_ARGS=(record
  -F "$FREQUENCY"
  --call-graph dwarf,16384
  -p "$PIDS"
  --timestamp
  -o "$OUTPUT"
)

# Off-CPU is a perf 5.16+ feature for kernel-traced sleep stacks; older
# kernels reject the flag — best-effort attempt then fall back.
if [[ "$OFF_CPU" -eq 1 ]]; then
  if perf record --off-cpu -F 1 -o /dev/null -- /bin/true >/dev/null 2>&1; then
    PERF_ARGS+=(--off-cpu)
    log "off-CPU profiling enabled"
  else
    log "off-CPU not supported by this perf build — on-CPU only"
  fi
fi

if [[ -n "$DURATION" ]]; then
  PERF_ARGS+=(-- sleep "$DURATION")
fi

# ── Signal handling ───────────────────────────────────────────────────────────
PERF_PID=""
forward_sigint() {
  if [[ -n "$PERF_PID" ]] && kill -0 "$PERF_PID" 2>/dev/null; then
    log "forwarding SIGINT to perf (pid=$PERF_PID) for clean flush"
    if [[ "$USE_SUDO" -eq 1 ]]; then
      sudo kill -INT "$PERF_PID" 2>/dev/null || true
    else
      kill -INT "$PERF_PID" 2>/dev/null || true
    fi
  fi
}
trap forward_sigint INT TERM

# ── Run perf ──────────────────────────────────────────────────────────────────
log "starting: ${PERF_ARGS[*]} → $OUTPUT"
if [[ "$USE_SUDO" -eq 1 ]]; then
  sudo -n perf "${PERF_ARGS[@]}" &
else
  perf "${PERF_ARGS[@]}" &
fi
PERF_PID=$!

# wait + propagate
wait "$PERF_PID"
PERF_EXIT=$?

# ── Post: chown to user if root-owned ─────────────────────────────────────────
if [[ -f "$OUTPUT" && "$USE_SUDO" -eq 1 ]]; then
  USER_NAME="${SUDO_USER:-${USER:-$(id -un)}}"
  if [[ -n "$USER_NAME" ]] && id "$USER_NAME" >/dev/null 2>&1; then
    sudo -n chown "$USER_NAME:$(id -gn "$USER_NAME")" "$OUTPUT" 2>/dev/null || true
  fi
fi

if [[ -f "$OUTPUT" ]]; then
  SIZE_MB=$(du -m "$OUTPUT" 2>/dev/null | awk '{print $1}')
  log "wrote $OUTPUT (${SIZE_MB} MB, perf exit=$PERF_EXIT)"
  log "view: hotspot $OUTPUT"
else
  log "perf exited with $PERF_EXIT but no output file — capture failed"
fi

exit 0
