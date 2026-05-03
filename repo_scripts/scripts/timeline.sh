#!/usr/bin/env bash
# timeline.sh — convert a perf.data into a Perfetto/Chrome trace JSON.
#
# x-axis: wall-clock time. y-axis: per-thread swimlane (and per-CPU swimlane
# in a sibling group). Each sample is a vertical instant marker; hover to see
# the captured callstack (leaf-first, top-N frames).
#
# Usage:
#   timeline.sh                              # default: --mode flamechart, max-depth=5
#   timeline.sh --mode instant               # vertical markers per sample (no bars)
#   timeline.sh --mode flamechart            # horizontal bars labelled by function name
#   timeline.sh --max-depth 5                # shallower stacks → smaller JSON
#   timeline.sh <perf.data>                  # explicit input
#   timeline.sh --mode flamechart <perf.data> [output.json]
#
# Modes:
#   flamechart (default)  — adjacent-sample stitching: each function becomes
#                           a horizontal bar with a label, width = wall time
#                           the function was on stack. This answers "what ran
#                           when" with function names baked into the picture.
#   instant               — each sample is a vertical marker; hover shows the
#                           callstack. Use when exact sample timestamps matter.
#
# After conversion, drag-drop the JSON onto https://ui.perfetto.dev.
#
# Requires:
#   * perf — captures the trace
#   * Python 3 with `rtc_tools.conversion.perf_to_chrome_trace` on PYTHONPATH
#     (provided by `source repo_scripts/scripts/setup_env.sh`).

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
MAX_DEPTH=5
MODE="flamechart"
INPUT=""
OUTPUT=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --max-depth)
      MAX_DEPTH="$2"
      shift 2
      ;;
    --max-depth=*)
      MAX_DEPTH="${1#--max-depth=}"
      shift
      ;;
    --mode)
      MODE="$2"
      shift 2
      ;;
    --mode=*)
      MODE="${1#--mode=}"
      shift
      ;;
    -h|--help)
      sed -n '2,16p' "$0"
      exit 0
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

case "$MODE" in
  flamechart|instant) ;;
  *)
    echo "[timeline] --mode must be one of: flamechart | instant (got: $MODE)" >&2
    exit 2
    ;;
esac

# ── Resolve input ────────────────────────────────────────────────────────────
if [[ -z "$INPUT" ]]; then
  INPUT="$(ls -1dt "$WS"/logging_data/[0-9]*/perf/perf.data 2>/dev/null | head -1 || true)"
  if [[ -z "$INPUT" ]]; then
    echo "[timeline] no perf.data found under $WS/logging_data/*/perf/" >&2
    echo "[timeline] capture first:  ros2 launch integrated_bringup sim.launch.py enable_perf:=true" >&2
    exit 1
  fi
  echo "[timeline] using latest: $INPUT" >&2
fi

if [[ ! -f "$INPUT" ]]; then
  echo "[timeline] input not found: $INPUT" >&2
  exit 1
fi

# ── Resolve output (default: trace.json next to perf.data) ───────────────────
if [[ -z "$OUTPUT" ]]; then
  OUTPUT="$(dirname "$INPUT")/trace.json"
fi

# ── Sanity: PYTHONPATH must include rtc_tools ────────────────────────────────
if ! python3 -c "import rtc_tools.conversion.perf_to_chrome_trace" 2>/dev/null; then
  echo "[timeline] rtc_tools.conversion.perf_to_chrome_trace not importable." >&2
  echo "[timeline] source the workspace env first:" >&2
  echo "[timeline]   source $WS/src/rtc-framework/repo_scripts/scripts/setup_env.sh" >&2
  exit 1
fi

if ! command -v perf >/dev/null 2>&1; then
  echo "[timeline] perf not on PATH — install: sudo apt install linux-tools-\$(uname -r)" >&2
  exit 1
fi

# ── Convert ──────────────────────────────────────────────────────────────────
echo "[timeline] perf script | python perf_to_chrome_trace → $OUTPUT (mode=$MODE max-depth=$MAX_DEPTH)" >&2
perf script -i "$INPUT" 2>/dev/null \
  | python3 -m rtc_tools.conversion.perf_to_chrome_trace \
      --output "$OUTPUT" --max-depth "$MAX_DEPTH" --mode "$MODE"
RC=$?

if [[ $RC -ne 0 ]]; then
  echo "[timeline] conversion failed (rc=$RC)" >&2
  exit $RC
fi

SIZE_KB=$(du -k "$OUTPUT" | awk '{print $1}')
echo "[timeline] wrote $OUTPUT (${SIZE_KB} KB)"
echo "[timeline] view: open https://ui.perfetto.dev and drag-drop $OUTPUT"
