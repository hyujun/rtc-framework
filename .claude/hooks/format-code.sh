#!/bin/bash
# PostToolUse hook (matcher: Edit|Write) — auto-format the file just touched.
#
# Intent : keep the working tree clean of style noise so reviewers and the Stop
#          hook see only semantic diffs.
# Trigger: every Edit / Write tool call by Claude. Reads {tool_input.file_path}
#          from stdin JSON.
# Format : C++ (.cpp/.hpp/.h/.cc)  -> clang-format -i (uses repo .clang-format)
#          Python (.py)            -> ruff format + ruff check --fix
# Lookup : clang-format prefers a system binary; if absent, falls back to the
#          repo-pinned version via `uvx` (so environments without a system
#          clang-format — e.g. fresh dev boxes / CI — still auto-format instead
#          of silently skipping). ruff is searched venv-first:
#            1. $VIRTUAL_ENV/bin/ruff           (active venv)
#            2. <workspace>/.venv/bin/ruff      (rtc_ws/.venv created by install.sh)
#            3. `command -v ruff`               (PATH fallback)
# Limits : silent on tool absence (no clang-format AND no uvx / no ruff -> skip).
#          Other languages (.yaml / .md / .cmake / .sh) are NOT formatted here.
# Exit   : always 0; never blocks. clang-format / ruff stderr is suppressed.
set -euo pipefail

# Pin the uvx fallback to the .clang-format baseline version (header says
# "clang-format 18+ assumed"). A floating default would pull a newer release
# whose style evolution reflows pre-existing lines.
CLANG_FORMAT_PIN="18.1.8"

INPUT=$(cat)
FILE_PATH=$(echo "$INPUT" | jq -r '.tool_input.file_path // empty')

[ -z "$FILE_PATH" ] && exit 0
[ -f "$FILE_PATH" ] || exit 0

# Resolve ruff binary (prefer venv).
find_ruff() {
  if [[ -n "${VIRTUAL_ENV:-}" && -x "${VIRTUAL_ENV}/bin/ruff" ]]; then
    echo "${VIRTUAL_ENV}/bin/ruff"
    return 0
  fi
  # Hook runs at unknown CWD; locate workspace via this script's path
  # (.claude/hooks/format-code.sh → ../../ = repo root → ../../ = rtc_ws).
  local script_dir
  script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
  local ws_venv="${script_dir}/../../../../.venv/bin/ruff"
  if [[ -x "$ws_venv" ]]; then
    echo "$ws_venv"
    return 0
  fi
  command -v ruff 2>/dev/null && return 0
  return 1
}

case "$FILE_PATH" in
  *.cpp|*.hpp|*.h|*.cc)
    if command -v clang-format &>/dev/null; then
      clang-format -i "$FILE_PATH" 2>/dev/null
    elif command -v uvx &>/dev/null; then
      # No system clang-format: run the repo-pinned version ephemerally via uvx
      # (downloaded once, then cached). Pin to ${CLANG_FORMAT_PIN} — the
      # .clang-format baseline — so a newer default (e.g. 22.x) does not reflow
      # pre-existing lines and create churn beyond the file being edited.
      uvx --from "clang-format==${CLANG_FORMAT_PIN}" clang-format -i "$FILE_PATH" 2>/dev/null || true
    fi
    ;;
  *.py)
    if RUFF_BIN=$(find_ruff); then
      "$RUFF_BIN" format --quiet "$FILE_PATH" 2>/dev/null || true
      "$RUFF_BIN" check --fix --quiet "$FILE_PATH" 2>/dev/null || true
    fi
    ;;
esac

exit 0
