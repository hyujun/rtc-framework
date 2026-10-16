#!/bin/bash
# PostToolUse hook (matcher: Edit|Write) — auto-format the file just touched.
#
# Intent : keep the working tree clean of style noise so reviewers and the Stop
#          hook see only semantic diffs.
# Trigger: every Edit / Write tool call by Claude. Reads {tool_input.file_path}
#          from stdin JSON.
# Format : C++ (.cpp/.hpp/.h/.cc)  -> clang-format -i (uses repo .clang-format)
#          Python (.py)            -> ruff check --fix (F401 unfixable), THEN ruff format
# Lookup : lib/formatters.sh (shared with verify-changes.sh Phase 5, so the
#          file written here is the file graded there): clang-format system
#          binary, else the repo-pinned version via `uvx`; ruff venv-first.
# Limits : silent on tool absence (no clang-format AND no uvx / no ruff -> skip).
#          Edit / Write only: a file written through Bash (heredoc, sed -i, a
#          rewrite script) never passes here. verify-changes.sh Phase 5 blocks
#          the formatter drift such a write introduces.
#          Other languages (.yaml / .md / .cmake / .sh) are NOT formatted here.
# Exit   : always 0; never blocks. clang-format / ruff stderr is suppressed.
set -euo pipefail

# shellcheck source=.claude/hooks/lib/formatters.sh
. "$(dirname "${BASH_SOURCE[0]}")/lib/formatters.sh"

INPUT=$(cat)
FILE_PATH=$(echo "$INPUT" | jq -r '.tool_input.file_path // empty')

[ -z "$FILE_PATH" ] && exit 0
[ -f "$FILE_PATH" ] || exit 0

case "$FILE_PATH" in
  *.cpp|*.hpp|*.h|*.cc)
    if [ "${#CLANG_FORMAT_CMD[@]}" -gt 0 ]; then
      "${CLANG_FORMAT_CMD[@]}" -i "$FILE_PATH" 2>/dev/null || true
    fi
    ;;
  *.py)
    if RUFF_BIN=$(find_ruff); then
      # Autofix first, format last. The reverse order ends on an autofix that
      # can leave a non-fixed-point file: `format` splits an over-long
      # `open(name, "r")` across three lines, UP015 then drops `"r"`, and the
      # call that now fits on one line stays split -- which verify-changes.sh
      # Phase 5 grades as drift and blocks.
      #
      # F401 (unused import) is NOT autofixed here. This hook runs after EVERY
      # Edit, and a change is several Edits: an import added in one is unused
      # until a later one writes the code that needs it, so the fix deleted it
      # in between -- and the next Edit's code then referenced a name that was
      # gone (twice in one S9a GUI change, #537). An import that really stays
      # unused is still a `ruff check` finding (AGENTS.md §4); Phase 5 grades
      # formatting only, so this changes no gate.
      "$RUFF_BIN" check --fix --unfixable F401 --quiet "$FILE_PATH" 2>/dev/null || true
      "$RUFF_BIN" format --quiet "$FILE_PATH" 2>/dev/null || true
    fi
    ;;
esac

exit 0
