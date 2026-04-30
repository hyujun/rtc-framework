#!/bin/bash
# PostToolUse hook (matcher: Edit|Write) — auto-format the file just touched.
#
# Intent : keep the working tree clean of style noise so reviewers and the Stop
#          hook see only semantic diffs.
# Trigger: every Edit / Write tool call by Claude. Reads {tool_input.file_path}
#          from stdin JSON.
# Format : C++ (.cpp/.hpp/.h/.cc)  -> clang-format -i  (uses repo .clang-format
#                                                       if present, else LLVM
#                                                       default — verify match
#                                                       with agent_docs/conventions.md)
#          Python (.py)            -> ruff format + ruff check --fix
# Limits : silent on tool absence (no clang-format / no ruff -> skip). Other
#          languages (.yaml / .md / .cmake / .sh) are NOT formatted here.
# Exit   : always 0; never blocks. clang-format / ruff stderr is suppressed.
set -euo pipefail

INPUT=$(cat)
FILE_PATH=$(echo "$INPUT" | jq -r '.tool_input.file_path // empty')

[ -z "$FILE_PATH" ] && exit 0
[ -f "$FILE_PATH" ] || exit 0

case "$FILE_PATH" in
  *.cpp|*.hpp|*.h|*.cc)
    if command -v clang-format &>/dev/null; then
      clang-format -i "$FILE_PATH" 2>/dev/null
    fi
    ;;
  *.py)
    if command -v ruff &>/dev/null; then
      ruff format --quiet "$FILE_PATH" 2>/dev/null
      ruff check --fix --quiet "$FILE_PATH" 2>/dev/null || true
    fi
    ;;
esac

exit 0
