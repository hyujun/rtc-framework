#!/bin/bash
# Catching sim evaluation, a batch of units (first used for E1-F06, G-1, #632):
#   DATA=<data dir> PLAN=<plan file> BALL_SIM_WS=<estimator workspace> run_all.sh
# Launch it through repo_scripts/scripts/with_verify_hold.sh — the wrapper holds
# the Stop hook's build/test for the whole run (the gap between two units has no
# simulator). DATA and PLAN have no default: the units, progress.log and the
# STOP file live under $DATA, never in the repository.
# Resumable: skips units whose status is DONE. A failed unit is moved aside
# (<dir>.fail<N>, kept as evidence) and retried with the same seed, at most
# MAX_TRY attempts per invocation; FAIL:host_busy and FAIL:estimator not
# activated are rig failures and are what the retries are for.
# Plan lines: <dir> <robot p1b|leap> <overlay> <n> <seed> <mode mpc|closed_form> [<key=value;key=value>]
# (the last field is run_unit.sh's EXPECT_KV: mirror values the unit must read back).
D=$(cd "$(dirname "$0")" && pwd)
[ -n "$DATA" ] || { echo "run_all.sh: DATA is not set (the data directory of this evaluation)" >&2; exit 2; }
[ -n "$PLAN" ] && [ -f "$PLAN" ] || { echo "run_all.sh: PLAN is not set or not a file (the plan of this evaluation)" >&2; exit 2; }
export DATA
MAX_TRY=${MAX_TRY:-3}
mkdir -p "$DATA"
wait_idle() {
  sleep ${IDLE_GRACE:-5}
  local waited=0
  # Judge by executable: a shell whose command TEXT mentions pytest is not a test run.
  while ps -eo comm,args | grep -v '^\(bash\|sh\|grep\|sleep\|tail\) ' | grep -q '[c]olcon \(build\|test\)\|[p]ytest\|[c]test '; do
    [ $waited -ge ${IDLE_MAX_S:-1200} ] && { echo "$(date +%T) busy after ${waited}s, starting anyway" >> "$DATA/progress.log"; return; }
    sleep 15; waited=$((waited + 15))
  done
}
while read -r DIR SHORT COND N SEED MODE KV; do
  [ -z "$DIR" ] && continue
  case $DIR in \#*) continue ;; esac
  if [ -n "$ONLY" ] && [[ "$DIR" != $ONLY* ]]; then continue; fi
  for try in $(seq 1 "$MAX_TRY"); do
    [ "$(cat "$DATA/$DIR/status" 2>/dev/null)" == "DONE" ] && break
    if [ -d "$DATA/$DIR" ]; then k=1; while [ -e "$DATA/$DIR.fail$k" ]; do k=$((k+1)); done; mv "$DATA/$DIR" "$DATA/$DIR.fail$k"; fi
    wait_idle
    echo "$(date +%T) start $DIR try $try load $(cut -d' ' -f1-3 /proc/loadavg)" >> "$DATA/progress.log"
    ARM=${MODE:-mpc} EXPECT_MODE=${MODE:-mpc} EXPECT_KV=$KV "$D/run_unit.sh" "$DATA/$DIR" "$SHORT" "$COND" "$N" "$SEED" < /dev/null
    echo "$(date +%T) end $DIR $(cat "$DATA/$DIR/status")" >> "$DATA/progress.log"
    [ -f "$DATA/STOP" ] && break 2
  done
done < "$PLAN"
echo "$(date +%T) run_all exit" >> "$DATA/progress.log"
