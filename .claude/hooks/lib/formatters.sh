# shellcheck shell=bash
# Formatter lookup shared by format-code.sh (PostToolUse, formats in place)
# and verify-changes.sh (Stop, grades drift): one pin and one search order, so
# the file the first hook writes is the file the second hook accepts.
#
# clang-format: a system binary first, else the repo-pinned version through
#   `uvx` (fetched once, then cached), so a box without one still formats
#   rather than silently skipping. The pin is the .clang-format baseline
#   ("clang-format 18+ assumed"); a floating default would pull a release
#   whose style evolution reflows pre-existing lines.
# ruff: venv first -- $VIRTUAL_ENV/bin/ruff, then <workspace>/.venv/bin/ruff
#   (rtc_ws/.venv, made by install.sh; located from this file, as the hooks
#   run at an unknown cwd), then PATH.
#
# Sourced, not run: sets CLANG_FORMAT_PIN, CLANG_FORMAT_CMD (an argv array,
# empty when neither a binary nor uvx exists) and defines find_ruff.

CLANG_FORMAT_PIN="18.1.8"
CLANG_FORMAT_CMD=()
if command -v clang-format >/dev/null 2>&1; then
  CLANG_FORMAT_CMD=(clang-format)
elif command -v uvx >/dev/null 2>&1; then
  CLANG_FORMAT_CMD=(uvx --from "clang-format==${CLANG_FORMAT_PIN}" clang-format)
fi

# Prints the ruff binary to use; non-zero when there is none.
find_ruff() {
  if [[ -n "${VIRTUAL_ENV:-}" && -x "${VIRTUAL_ENV}/bin/ruff" ]]; then
    echo "${VIRTUAL_ENV}/bin/ruff"; return 0
  fi
  local hooks_dir ws_venv
  hooks_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
  ws_venv="${hooks_dir}/../../../../.venv/bin/ruff"
  [[ -x "$ws_venv" ]] && { echo "$ws_venv"; return 0; }
  command -v ruff 2>/dev/null && return 0
  return 1
}
