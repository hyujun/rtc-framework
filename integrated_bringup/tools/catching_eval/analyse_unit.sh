#!/bin/bash
# Catching sim evaluation: run the repo's catching_trials + tc_vector over finished units.
#   analyse_unit.sh <robot ur5e_p1b|iiwa7_leap> <unit_dir>...
# Writes <unit>/ct/{catching_trials.csv,catching_trials_summary.json,vec.csv}.
# Never while a unit is being collected (host load).
ROBOT=$1; shift
D=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$D/../../.." && pwd)
WS=$(cd "${RTC_WS:-$REPO/../..}" 2>/dev/null && pwd)
[ -d "$WS/install" ] || { echo "analyse_unit.sh: no colcon workspace at ${WS:-${RTC_WS:-$REPO/../..}} (RTC_WS names another)" >&2; exit 2; }
( cd "$WS" && source "$REPO/repo_scripts/scripts/setup_env.sh" >/dev/null 2>&1
  CFG=$(ros2 pkg prefix integrated_bringup)/share/integrated_bringup/config/$ROBOT
  for U in "$@"; do
    [ "$(cat "$U/status" 2>/dev/null)" == "DONE" ] || { echo "skip $U (not DONE)"; continue; }
    [ -f "$U/ct/vec.csv" ] && continue
    mkdir -p "$U/ct"
    /usr/bin/python3 "$D/tc_vector.py" "$U" "$CFG" "$U/ct/vec.csv" "$U/ct" > "$U/ct/log.txt" 2>&1 || echo "FAILED $U (see ct/log.txt)"
  done )
