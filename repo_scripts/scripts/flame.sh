#!/usr/bin/env bash
# flame.sh — convert a perf.data into an interactive flamegraph SVG.
#
# Usage:
#   flame.sh                         # mixed: all threads merged, latest perf.data
#   flame.sh --by thread             # per-thread flamegraph (each comm/tid is a top frame)
#   flame.sh --by cpu                # per-CPU flamegraph (each [N] core is a top frame)
#   flame.sh --by mixed              # default — all samples merged
#   flame.sh <perf.data>             # explicit input
#   flame.sh --by thread <perf.data> [output.svg]
#
# The output SVG opens automatically via xdg-open. It is interactive (zoom,
# search by symbol regex, click to focus) and works in any browser without
# the Hotspot UI / perfparser.
#
# Filtering tips:
#   * Open the SVG in a browser, click "Search" (top right) and type
#     `^rtc::|^ur5e_bringup::` to highlight workspace functions only.
#   * Click any frame to zoom into its subtree.
#   * In --by thread / --by cpu mode, click a top-row frame to drill into one
#     thread or CPU core.
#
# Requires:
#   * perf  — captures the trace
#   * <ws>/deps/FlameGraph/{stackcollapse-perf.pl,flamegraph.pl} —
#     installed by `./install.sh --perf` (clones brendangregg/FlameGraph).

set -uo pipefail

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

_walk_to_ws() {
  local dir="$_SCRIPT_DIR"
  for _ in 1 2 3 4 5 6; do
    if [[ -d "$dir/deps" ]]; then
      echo "$dir"
      return 0
    fi
    dir="$(dirname "$dir")"
  done
  return 1
}

WS="$(_walk_to_ws || true)"
if [[ -z "${WS:-}" ]]; then
  echo "[flame] could not locate workspace root (deps/ not found above $_SCRIPT_DIR)" >&2
  exit 1
fi

FG_DIR="$WS/deps/FlameGraph"
if [[ ! -x "$FG_DIR/stackcollapse-perf.pl" ]] || [[ ! -x "$FG_DIR/flamegraph.pl" ]]; then
  echo "[flame] FlameGraph tools not found at $FG_DIR" >&2
  echo "[flame] run: ./install.sh --perf  (clones brendangregg/FlameGraph)" >&2
  echo "[flame] or manually:  git clone --depth=1 https://github.com/brendangregg/FlameGraph $FG_DIR" >&2
  exit 1
fi

# ── Args ──────────────────────────────────────────────────────────────────────
MODE="mixed"
INPUT=""
OUTPUT=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --by)
      MODE="$2"
      shift 2
      ;;
    --by=*)
      MODE="${1#--by=}"
      shift
      ;;
    -h|--help)
      sed -n '2,30p' "$0"
      exit 0
      ;;
    *)
      if [[ -z "$INPUT" ]]; then
        INPUT="$1"
      elif [[ -z "$OUTPUT" ]]; then
        OUTPUT="$1"
      else
        echo "[flame] unexpected positional arg: $1" >&2
        exit 2
      fi
      shift
      ;;
  esac
done

case "$MODE" in
  mixed|thread|cpu) ;;
  *)
    echo "[flame] --by must be one of: mixed | thread | cpu (got: $MODE)" >&2
    exit 2
    ;;
esac

# ── Resolve input ────────────────────────────────────────────────────────────
if [[ -z "$INPUT" ]]; then
  INPUT="$(ls -1dt "$WS"/logging_data/[0-9]*/perf/perf.data 2>/dev/null | head -1 || true)"
  if [[ -z "$INPUT" ]]; then
    echo "[flame] no perf.data found under $WS/logging_data/*/perf/" >&2
    echo "[flame] capture first:  ros2 launch ur5e_bringup sim.launch.py enable_perf:=true" >&2
    exit 1
  fi
  echo "[flame] using latest: $INPUT" >&2
fi

if [[ ! -f "$INPUT" ]]; then
  echo "[flame] input not found: $INPUT" >&2
  exit 1
fi

# ── Resolve output (default: flame[-thread|-cpu].svg next to perf.data) ──────
if [[ -z "$OUTPUT" ]]; then
  case "$MODE" in
    mixed)  OUTPUT="$(dirname "$INPUT")/flame.svg" ;;
    thread) OUTPUT="$(dirname "$INPUT")/flame-thread.svg" ;;
    cpu)    OUTPUT="$(dirname "$INPUT")/flame-cpu.svg" ;;
  esac
fi

# ── Decode + collapse (mode-specific) ────────────────────────────────────────
echo "[flame] mode=$MODE  → $OUTPUT" >&2
TMP_FOLDED="$(mktemp -t flame.folded.XXXXXX)"
trap 'rm -f "$TMP_FOLDED"' EXIT

case "$MODE" in
  mixed)
    if ! perf script -i "$INPUT" 2>/dev/null \
          | "$FG_DIR/stackcollapse-perf.pl" --all > "$TMP_FOLDED"; then
      echo "[flame] perf script | stackcollapse failed" >&2
      exit 1
    fi
    SUBTITLE="Search workspace symbols: ^rtc::|^ur5e_bringup::"
    ;;
  thread)
    # --tid emits "comm-pid/tid" as the bottom frame; this is what we want as
    # the per-thread separator. --all keeps comm anyway.
    if ! perf script -i "$INPUT" 2>/dev/null \
          | "$FG_DIR/stackcollapse-perf.pl" --tid --all > "$TMP_FOLDED"; then
      echo "[flame] perf script | stackcollapse --tid failed" >&2
      exit 1
    fi
    SUBTITLE="Per-thread view (top row = comm-pid/tid). Click a thread to zoom in."
    ;;
  cpu)
    # `perf script` first line: "comm  pid  [NNN]  time:  cycles:P:"
    # We capture [NNN] as the CPU id, then prepend "cpu_NNN;" to every folded
    # stack belonging to that sample. stackcollapse-perf.pl handles the
    # multi-line stack input; we post-process its output by cross-referencing
    # the sample → cpu mapping built from `perf script` itself.
    #
    # Strategy: pipe `perf script` through awk that re-emits the trace with the
    # CPU id glued onto the comm field, then let stackcollapse run normally.
    # This makes the bottom frame "comm[cpu_NNN]", giving per-CPU separation.
    if ! perf script -i "$INPUT" 2>/dev/null \
          | awk '
              # Sample header: "comm  pid  [cpu]  time: ..."
              /^[a-zA-Z0-9_:.-]+ +[0-9]+ +\[[0-9]+\] / {
                # Extract CPU id from the [NNN] field
                match($0, /\[[0-9]+\]/);
                cpu = substr($0, RSTART+1, RLENGTH-2);
                # Find the first space-separated token (comm) and append [cpu_NNN]
                # so that stackcollapse-perf.pl groups by it.
                sub(/^[a-zA-Z0-9_:.-]+/, "&[cpu_"cpu"]");
                print;
                next;
              }
              { print }
            ' \
          | "$FG_DIR/stackcollapse-perf.pl" --all > "$TMP_FOLDED"; then
      echo "[flame] perf script | awk | stackcollapse failed" >&2
      exit 1
    fi
    SUBTITLE="Per-CPU view (bottom frame ends with [cpu_NNN]). Click a core to zoom."
    ;;
esac

if [[ ! -s "$TMP_FOLDED" ]]; then
  echo "[flame] folded stacks file empty — input may have no resolvable samples" >&2
  exit 1
fi

"$FG_DIR/flamegraph.pl" \
    --title="$(basename "$(dirname "$(dirname "$INPUT")")") perf flame graph (--by $MODE)" \
    --subtitle="$SUBTITLE" \
    --hash \
    "$TMP_FOLDED" > "$OUTPUT"

if [[ ! -s "$OUTPUT" ]]; then
  echo "[flame] flamegraph.pl produced empty output" >&2
  exit 1
fi

SIZE_KB=$(du -k "$OUTPUT" | awk '{print $1}')
echo "[flame] wrote $OUTPUT (${SIZE_KB} KB)"
echo "[flame] open in browser: xdg-open $OUTPUT"

if command -v xdg-open >/dev/null 2>&1; then
  xdg-open "$OUTPUT" >/dev/null 2>&1 &
fi
