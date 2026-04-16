#!/bin/bash
# PostToolUse hook: auto-format C++/Python files after Edit/Write
# Success = silent. Only reports errors.
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
