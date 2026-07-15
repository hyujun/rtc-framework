#!/usr/bin/env bash
# timeline.sh — convert an LTTng CTF trace into a Chrome Trace JSON.
#
# x-axis: wall-clock time. Two swimlane groups:
#   * Threads (by TID) — each ros2:callback_start/end pair becomes a
#                       horizontal bar labelled with the callback symbol.
#   * Cpus             — per-CPU lane showing which thread ran when (from
#                       sched_switch). Use this to verify ApplyThreadConfig
#                       pinning and spot migrations / IRQ leaks.
#
# Usage:
#   timeline.sh                                # default: latest ~/.ros/tracing/* trace
#   timeline.sh <trace_dir>                    # explicit CTF trace directory
#   timeline.sh <trace_dir> [output.json]
#   timeline.sh [--all-threads] [--focus-proc <substr,...>] [--keep-all] ...
#     Flags starting with '-' are forwarded verbatim to
#     rtc_tools.conversion.ctf_to_chrome_trace (see its --help).
#
# After conversion, drag-drop the JSON onto https://ui.perfetto.dev.
#
# Requires:
#   * babeltrace2 — CTF reader (apt: babeltrace2)
#   * Python 3 with `rtc_tools.conversion.ctf_to_chrome_trace` on PYTHONPATH
#     (provided by `source repo_scripts/scripts/setup_env.sh`).
#   * Optional: `python3-bt2` for the faster, structured parser. The script
#     transparently falls back to babeltrace2 CLI text parsing if absent.

set -uo pipefail

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

_walk_to_ws() {
  local dir="$_SCRIPT_DIR"
  for _ in 1 2 3 4 5 6; do
    if [[ -d "$dir/install" && -d "$dir/src" ]] || [[ -d "$dir/logging_data" ]]; then
      echo "$dir"
      return 0
    fi
    dir="$(dirname "$dir")"
  done
  return 1
}

WS="$(_walk_to_ws || true)"
if [[ -z "${WS:-}" ]]; then
  echo "[timeline] could not locate workspace root above $_SCRIPT_DIR" >&2
  exit 1
fi

# ── Args ──────────────────────────────────────────────────────────────────────
INPUT=""
OUTPUT=""
EXTRA_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      sed -n '2,23p' "$0"
      exit 0
      ;;
    --focus-proc|--keep-events)
      # Value-taking converter flags: forward the flag and its value.
      if [[ $# -lt 2 ]]; then
        echo "[timeline] $1 requires a value" >&2
        exit 2
      fi
      EXTRA_ARGS+=("$1" "$2")
      shift 2
      ;;
    -*)
      # Any other flag is forwarded verbatim to ctf_to_chrome_trace.
      EXTRA_ARGS+=("$1")
      shift
      ;;
    *)
      if [[ -z "$INPUT" ]]; then
        INPUT="$1"
      elif [[ -z "$OUTPUT" ]]; then
        OUTPUT="$1"
      else
        echo "[timeline] unexpected positional arg: $1" >&2
        exit 2
      fi
      shift
      ;;
  esac
done

# ── Resolve input ────────────────────────────────────────────────────────────
# Trace lives at <ws>/logging_data/<YYMMDD_HHMM>/tracing/<lttng_session>/.
# Pick the most recent session folder that actually has a tracing/ subdir.
if [[ -z "$INPUT" ]]; then
  INPUT="$(ls -1dt "$WS"/logging_data/[0-9]*/tracing/*/ 2>/dev/null | head -1 || true)"
  if [[ -z "$INPUT" ]]; then
    echo "[timeline] no CTF trace found under $WS/logging_data/*/tracing/" >&2
    echo "[timeline] capture first:  ros2 launch integrated_bringup sim_ur5e_p1a.launch.py enable_tracing:=true" >&2
    exit 1
  fi
  INPUT="${INPUT%/}"
  echo "[timeline] using latest: $INPUT" >&2
fi

if [[ ! -d "$INPUT" ]]; then
  echo "[timeline] input not a directory: $INPUT" >&2
  exit 1
fi

# ── Resolve output (default: trace.json next to the trace directory) ─────────
if [[ -z "$OUTPUT" ]]; then
  OUTPUT="${INPUT}/trace.json"
fi

# ── Sanity: PYTHONPATH must include rtc_tools ────────────────────────────────
if ! python3 -c "import rtc_tools.conversion.ctf_to_chrome_trace" 2>/dev/null; then
  echo "[timeline] rtc_tools.conversion.ctf_to_chrome_trace not importable." >&2
  echo "[timeline] source the workspace env first:" >&2
  echo "[timeline]   source $WS/src/rtc-framework/repo_scripts/scripts/setup_env.sh" >&2
  exit 1
fi

if ! command -v babeltrace2 >/dev/null 2>&1; then
  echo "[timeline] babeltrace2 not on PATH — install: ./install.sh --tracing" >&2
  exit 1
fi

# ── Convert ──────────────────────────────────────────────────────────────────
echo "[timeline] ctf_to_chrome_trace $INPUT → $OUTPUT" >&2
python3 -m rtc_tools.conversion.ctf_to_chrome_trace \
  --input "$INPUT" --output "$OUTPUT" "${EXTRA_ARGS[@]}"
RC=$?

if [[ $RC -ne 0 ]]; then
  echo "[timeline] conversion failed (rc=$RC)" >&2
  exit $RC
fi

SIZE_KB=$(du -k "$OUTPUT" | awk '{print $1}')
echo "[timeline] wrote $OUTPUT (${SIZE_KB} KB)"
echo "[timeline] view: open https://ui.perfetto.dev and drag-drop $OUTPUT"
