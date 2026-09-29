#!/usr/bin/env bash
# with_verify_hold.sh — run a measurement with the Stop hook's build/test held.
#
#   with_verify_hold.sh <command> [args...]
#
# An evaluation that launches one simulator per unit has none running between
# two units, and a turn that ends in that gap lets .claude/hooks/verify-changes.sh
# start `colcon test` beside the next unit. The hook defers build/test while a
# process listed in <workspace>/.rtc-verify-hold is alive (its workspace_holds
# owns the format: "<pid> <start time>", the start time being field 22 of
# /proc/<pid>/stat). This wrapper is the one writer of that line, so a driver
# does not have to get the format right by hand:
#
#   * lists ITSELF for as long as <command> runs, then takes its line out
#     (and the file, once no line is left);
#   * returns <command>'s exit code;
#   * leaves other holders' lines alone.
#
# Killed without a chance to clean up, it leaves a line the hook reads as
# stale: the pid is gone, or its start time is another process's.
#
# The workspace is the colcon workspace this checkout lives in
# (<workspace>/src/<repo>), the one the hook derives from the project
# directory. RTC_VERIFY_WORKSPACE names another (the tests do).
set -u

if [ $# -eq 0 ]; then
  echo "usage: $(basename "$0") <command> [args...]" >&2
  exit 2
fi

SELF=$(readlink -f "${BASH_SOURCE[0]}")
REPO=$(cd "$(dirname "$SELF")/../.." && pwd)
WORKSPACE=${RTC_VERIFY_WORKSPACE:-$(cd "$REPO/../.." && pwd)}
HOLD="$WORKSPACE/.rtc-verify-hold"

# comm may hold spaces; what follows the LAST ')' starts at field 3.
START=$(sed 's/^.*) //' "/proc/$$/stat" | cut -d' ' -f20)
LINE="$$ $START"

release() {
  [ -f "$HOLD" ] || return 0
  grep -vxF "$LINE" "$HOLD" >"$HOLD.$$" 2>/dev/null || true
  if [ -s "$HOLD.$$" ]; then
    mv "$HOLD.$$" "$HOLD"
  else
    rm -f "$HOLD.$$" "$HOLD"
  fi
}
trap release EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

printf '%s\n' "$LINE" >>"$HOLD" || {
  echo "$(basename "$0"): cannot write $HOLD" >&2
  exit 2
}
"$@"
