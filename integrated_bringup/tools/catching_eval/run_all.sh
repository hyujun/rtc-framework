#!/bin/bash
# Catching sim evaluation, a batch of units (first used for E1-F06, G-1, #632):
#   DATA=<data dir> PLAN=<plan file> BALL_SIM_WS=<estimator workspace> run_all.sh
# Launch it through repo_scripts/scripts/with_verify_hold.sh — the wrapper holds
# the Stop hook's build/test for the whole run (the gap between two units has no
# simulator). DATA and PLAN have no default: the units, progress.log and the
# STOP file live under $DATA, never in the repository.
# Resumable: skips units whose status is DONE. A unit that failed on the RIG is moved aside
# (<dir>.fail<N>, kept as evidence) and retried with the same seed, at most MAX_TRY attempts
# per invocation. A rig failure is one that does not depend on what the arm did with a throw:
# FAIL:host_busy, FAIL:estimator not activated and FAIL:startup (the bringup, before a throw) —
# RETRY_ON (an extended regex on the status) names them. Any other failure (FAIL:overlay …,
# FAIL:trials rc=…, FAIL:dirty tree, …) is NOT retried (#746): the unit stays where it failed,
# progress.log says so, and the batch goes on to the next unit. Retrying a unit for a reason
# its own throws could cause would choose which outcomes are recorded. Launching run_all.sh
# again is a person's decision: it then moves that unit aside and runs it anew.
# Plan lines:
#   <dir> <robot p1b|leap> <overlay> <n> <seed> <mode mpc|closed_form|mpc_docking> [<kv|-> [<search grid|nlp> [<ball>]]]
# The seventh field is run_unit.sh's EXPECT_KV (';'-separated mirror values the unit must read
# back; `-` for none when a later field follows). The eighth and ninth are the arm's catch-point
# search and the sim's ball (EXPECT_SEARCH, EXPECT_BALL) — an arm is search x segment x ball. A
# line without them takes the caller's EXPECT_SEARCH / EXPECT_BALL (then grid / tennis), as
# before #746.
D=$(cd "$(dirname "$0")" && pwd)
[ -n "$DATA" ] || { echo "run_all.sh: DATA is not set (the data directory of this evaluation)" >&2; exit 2; }
[ -n "$PLAN" ] && [ -f "$PLAN" ] || { echo "run_all.sh: PLAN is not set or not a file (the plan of this evaluation)" >&2; exit 2; }
export DATA
MAX_TRY=${MAX_TRY:-3}
RETRY_ON=${RETRY_ON:-'^FAIL:(host_busy|estimator not activated|startup)$'}
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
while read -r DIR SHORT COND N SEED MODE KV SEARCH BALL; do
  [ -z "$DIR" ] && continue
  case $DIR in \#*) continue ;; esac
  if [ -n "$ONLY" ] && [[ "$DIR" != $ONLY* ]]; then continue; fi
  [ "$KV" == "-" ] && KV=
  for try in $(seq 1 "$MAX_TRY"); do
    ST=$(cat "$DATA/$DIR/status" 2>/dev/null)
    [ "$ST" == "DONE" ] && break
    if [ "$try" -gt 1 ] && ! [[ "$ST" =~ $RETRY_ON ]]; then
      echo "$(date +%T) $DIR not retried: '$ST' is not a rig failure" >> "$DATA/progress.log"
      break
    fi
    if [ -d "$DATA/$DIR" ]; then k=1; while [ -e "$DATA/$DIR.fail$k" ]; do k=$((k+1)); done; mv "$DATA/$DIR" "$DATA/$DIR.fail$k"; fi
    wait_idle
    echo "$(date +%T) start $DIR try $try load $(cut -d' ' -f1-3 /proc/loadavg)" >> "$DATA/progress.log"
    ARM=${MODE:-mpc} EXPECT_MODE=${MODE:-mpc} EXPECT_KV=$KV \
      EXPECT_SEARCH=${SEARCH:-${EXPECT_SEARCH:-grid}} EXPECT_BALL=${BALL:-${EXPECT_BALL:-tennis}} \
      "$D/run_unit.sh" "$DATA/$DIR" "$SHORT" "$COND" "$N" "$SEED" < /dev/null
    echo "$(date +%T) end $DIR $(cat "$DATA/$DIR/status")" >> "$DATA/progress.log"
    [ -f "$DATA/STOP" ] && break 2
  done
done < "$PLAN"
echo "$(date +%T) run_all exit" >> "$DATA/progress.log"
