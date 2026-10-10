#!/bin/bash
# Catching sim evaluation, one unit (first used for E1-F06, G-1, #632). One launch = one unit:
#   DATA=<data dir> BALL_SIM_WS=<estimator workspace> THROWS_FILE=<catching_throw_list/1 JSON> \
#     run_unit.sh <out_dir> <robot p1b|leap> <overlay.yaml> <n_throws (ignored)> <seed>
# Runs from the source tree of the workspace it measures: the repository is the
# one this file is in, the colcon workspace the directory two levels above it
# (RTC_WS names another), and a repository that is not under <workspace>/src is
# refused — the revision written into the unit must be the tree that was built.
# Nothing is written into the repository: the unit goes to <out_dir>, the
# launch-minute stamp to $DATA/.last_minute.
# <out_dir>, the overlay, DATA and PROFILE may be relative to the caller's
# directory: they are made absolute before the script moves to the workspace.
# E1-F06 addition: the planner's search budget (not a ROS parameter — only the
# startup line "planner enabled: … budget X s" says it) goes into conditions.txt
# as planner_budget_s; a unit without that line is refused (criterion 3 needs it).
# E1-F10 additions: the unit is refused on a dirty rtc-framework tree
# (ALLOW_DIRTY=1 overrides), unless the CLIK form mirror says `dynamic`
# (EXPECT_CLIK), the mode mirror says EXPECT_MODE, and every `key=value` of
# EXPECT_KV (';'-separated, mirror names) reads back from the controller.
# E1-F17 addition: the docking planner's two budgets are in the mirror, so a plan line can
# check an overlay that moves one (EXPECT_KV).
# E1-F16 addition: the search-mode mirror says EXPECT_SEARCH (default grid) (search_mirror), and
# so does the startup `search mode:` line (mode_log); EXPECT_MODE may be closed_form, mpc or
# mpc_docking.
# E1-F19 addition: ENABLE_VIEWER=1 opens the MuJoCo viewer (default 0 — a unit is headless); the
# unit's conditions.txt says which (viewer: on | off), because a viewer shares the host with the sim.
# E1-F19 addition: the mirrors EXPECT_KV names are read too, whatever they are — the fixed list
# below is what every unit records, not the only thing a plan may check (a hand.docking.* check
# once failed on the name alone, with the value right: two units lost before a throw).
# The overlay turns the APPROACH-stop MPC on in CLOSED LOOP: the planner stores
# every segment, the RT takes a plan with its first segment and follows the
# segments from APPROACH to the end of the stop (mode mpc, #662).
# The controller's mirror must show mode mpc and the approach grid before a
# single throw, or the unit is refused.
# Two workspaces (MD-17): the sim and controller come from this workspace, the
# estimator from $BALL_SIM_WS (its shipped catching profile is the default
# PROFILE). The session's raw logs are copied into the unit at the end.
# E1-F19 part 2 addition: EXPECT_NSTOP and EXPECT_DTSTOP are the docking planner's stop.n_nodes and stop.dt_s
# the mirror must show. Their defaults are this robot's SHIPPED ones, read from its segment_mpc_docking.yaml in
# the source tree. A tuning overlay may move the stop grid, as it may the approach grid (EXPECT_NPRE,
# EXPECT_DTPRE) — it then states BOTH (an overlay that sets the count alone changes the length of the stop, and
# the unit that named only the count is refused: stop_dt). EXPECT_DTSTOP is compared as a number. stop.blocks is
# not mirrored and not checked.
# #798: a unit throws a throw list — THROWS_FILE=<catching_throw_list/1 JSON> (the shipped sets are
# integrated_bringup/config/<robot>/throw_sets/) in file order (--throws-file); the driver's frozen
# series (`--dist s35b`) is gone, so a unit without THROWS_FILE is refused before anything is
# launched. The n_throws argument is ignored (pass any number; kept for the plan-line format) and NT
# is the file's throw count (the timeout follows it; an unreadable file fails the unit).
# THROWS_LIMIT=<N> throws only the file's first N (--limit; NT follows). The path may be relative to
# the caller's directory. The seed argument seeds the ESTIMATOR ONLY (simulator_seed of
# sim_estimator.launch.py — its measurement noise); the MuJoCo sim takes no seed and the throws come
# from the file, so two units of one list with two seeds differ in the estimator's noise alone
# (#745 F5). conditions.txt records throws_file (+ sha256), throws_limit and the end rule.
# #747 addition (default off — the command line is then what it was):
# END_ON_BALL_LOW=1 adds --end-on-ball-low (a throw whose cycle
# never opened ends when the ball is below floor + margin, not at the 12 s cap), with
# BALL_LOW_MARGIN_M / BALL_LOW_GRACE_S as --ball-low-margin-m / --ball-low-grace-s (the driver's
# defaults when unset).
# #747 fix: whether the controller came up is check_startup.py's verdict. The loop used to stop on
# any `refus` in the launch log, and a healthy controller logs "… is refused without the IK
# (too_far)" at INFO just before its ready line: a poll between the two ended the unit as
# FAIL:startup before a throw. A failed bringup's first failure line goes to startup_failure.txt.
# Leaves <out_dir>/status = DONE | FAIL:<why>. Never set -u (setup_env.sh is sourced).
OUT=$1; SHORT=$2; OV=$3; NT=$4; SEED=$5
COND=${ARM:-mpc}
VIEWER=false; [ "${ENABLE_VIEWER:-0}" == "1" ] && VIEWER=true
OUT=$(realpath -ms "$OUT")
mkdir -p "$OUT"; rm -f "$OUT/status"
[ -n "$DATA" ] || { echo "FAIL:DATA is not set (the data directory of this evaluation)" | tee "$OUT/status" >&2; exit 1; }
[ -n "$BALL_SIM_WS" ] || { echo "FAIL:BALL_SIM_WS is not set (the estimator's colcon workspace)" | tee "$OUT/status" >&2; exit 1; }
BWS=$(cd "$BALL_SIM_WS" 2>/dev/null && pwd)
[ -n "$BWS" ] || { echo "FAIL:no estimator workspace $BALL_SIM_WS" | tee "$OUT/status" >&2; exit 1; }
PROFILE=${PROFILE:-$BWS/install/ball_perception_sim/share/ball_perception_sim/config/sim_profile.catching.json}
[ -f "$OV" ] || { echo "FAIL:no overlay $OV" > "$OUT/status"; exit 1; }
# Everything below runs from the workspace root (the launch resolves the
# overlay against its own directory): a relative path checked here would name
# another file there.
OV=$(realpath -ms "$OV"); DATA=$(realpath -ms "$DATA"); PROFILE=$(realpath -ms "$PROFILE")
[ -n "$THROWS_FILE" ] || { echo "FAIL:THROWS_FILE is not set (#798: a unit throws a catching_throw_list/1 file — integrated_bringup/config/<robot>/throw_sets/)" | tee "$OUT/status" >&2; exit 1; }
THROWS_FILE=$(realpath -ms "$THROWS_FILE")
NT=$(/usr/bin/python3 -c 'import json, sys; print(len(json.load(open(sys.argv[1]))["throws"]))' "$THROWS_FILE" 2>/dev/null)
case $NT in '' | *[!0-9]* | 0) echo "FAIL:cannot read a throw count from THROWS_FILE $THROWS_FILE" > "$OUT/status"; exit 1 ;; esac
if [ -n "$THROWS_LIMIT" ]; then
  case $THROWS_LIMIT in *[!0-9]* | 0 | '') echo "FAIL:THROWS_LIMIT must be a positive integer, got '$THROWS_LIMIT'" > "$OUT/status"; exit 1 ;; esac
  [ "$THROWS_LIMIT" -lt "$NT" ] && NT=$THROWS_LIMIT
fi
case $SHORT in p1b) ROBOT=ur5e_p1b ;; leap) ROBOT=iiwa7_leap ;; esac
case $ROBOT in
  ur5e_p1b)   LAUNCH=sim_ur5e_p1b.launch.py;   EXPECT_COMMIT=${EXPECT_COMMIT:-0.370}; STATE_RE='ur5e_state\|p1b_state' ;;
  iiwa7_leap) LAUNCH=sim_iiwa7_leap.launch.py; EXPECT_COMMIT=${EXPECT_COMMIT:-0.190}; STATE_RE='iiwa7_state\|leap_state' ;;
  *) echo "FAIL:unknown robot $ROBOT" > "$OUT/status"; exit 1 ;;
esac
EXPECT_TARM=${EXPECT_TARM:-0.05}
EXPECT_BALL=${EXPECT_BALL:-tennis}
if [ -e "$OUT/trials" ]; then echo "FAIL:trials dir exists (move the old unit aside)" > "$OUT/status"; exit 1; fi
D=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$D/../../.." && pwd)
WS=$(cd "${RTC_WS:-$REPO/../..}" 2>/dev/null && pwd)
case "$(readlink -f "$REPO")" in
  "$(readlink -f "${WS:-/nonexistent}")"/src/*) ;;
  *) echo "FAIL:$REPO is not under ${WS:-${RTC_WS:-?}}/src (run the tools of the workspace that was built)" | tee "$OUT/status" >&2; exit 1 ;;
esac
cd "$WS" || { echo "FAIL:no workspace $WS" > "$OUT/status"; exit 1; }
source "$REPO/repo_scripts/scripts/setup_env.sh" >/dev/null 2>&1
export ROS_DOMAIN_ID=${EVAL_DOMAIN:-88}
[ -f "$PROFILE" ] || { echo "FAIL:no profile $PROFILE" > "$OUT/status"; exit 1; }

# Session dirs are minute-named: never start in the minute of the previous launch.
mkdir -p "$DATA"
LAST=$(cat "$DATA/.last_minute" 2>/dev/null)
while [ "$(date +%y%m%d_%H%M)" == "$LAST" ]; do sleep 2; done
date +%y%m%d_%H%M > "$DATA/.last_minute"

stop_group() {  # stop_group <pgid> <seconds to wait before KILL>
  [ -z "$1" ] && return
  kill -INT -- -$1 2>/dev/null
  for i in $(seq 1 $2); do kill -0 -- -$1 2>/dev/null || return; sleep 1; done
  kill -KILL -- -$1 2>/dev/null
  echo "killed group $1 after $2 s" >> "$OUT/cleanup.log"
}
cleanup() {
  stop_group "$EPG" 20
  stop_group "$LPG" 20
  sleep 2
  # Residuals by bracket pattern (never pgrep -f in this shell), these two
  # workspaces only: another workspace may run its own ball_perception.
  for PAT in "$WS/[^ ]*[m]ujoco_simulator_node" "$WS/[^ ]*[i]ntegrated_rt_controller" "$BWS/[^ ]*[b]all_perception"; do
    ps -eo pid,args | grep "$PAT" | awk '{print $1}' | xargs -r kill -KILL 2>/dev/null
  done
  sleep 1
}

# A failed attempt keeps its session WITH the attempt (<out>/session_failed):
# every FAIL exit below leaves before the copy at the end, and without this the
# session stayed in logging_data as an orphan (E0-F02/F04: five of them, 343 MB).
keep_failed_session() {
  [ "$(cat "$OUT/status" 2>/dev/null)" == "DONE" ] && return
  local s; s=$(cat "$OUT/session.txt" 2>/dev/null)
  [ -n "$s" ] && [ -d "$WS/$s" ] && mv "$WS/$s" "$OUT/session_failed"
}
trap keep_failed_session EXIT
# A signal ends the unit AND what it started. The launch and the estimator run
# in sessions of their own (setsid), so the shell leaving does not stop them:
# they stayed on this ROS domain, where the next unit's mirror reads and throws
# then met two controllers, and kept logging into the session the EXIT trap had
# just moved away. bash runs this once the command in front has returned (a
# Ctrl-C or a hangup reaches that command too; a kill of this shell alone waits
# for it).
on_signal() {
  cleanup
  echo "FAIL:signal" > "$OUT/status"
  exit 1
}
trap on_signal INT TERM HUP

{
  echo "date_start: $(date -Is)"
  echo "robot: $ROBOT"; echo "condition: $COND"; echo "overlay: $OV"; echo "n: $NT"; echo "seed: $SEED"
  echo "throws_file: $THROWS_FILE"; echo "throws_file_sha256: $(sha256sum "$THROWS_FILE" | cut -d' ' -f1)"
  [ -n "$THROWS_LIMIT" ] && echo "throws_limit: $THROWS_LIMIT"
  [ "${END_ON_BALL_LOW:-0}" == "1" ] && echo "end_on_ball_low: 1 margin_m=${BALL_LOW_MARGIN_M:-default} grace_s=${BALL_LOW_GRACE_S:-default}"
  echo "rtc_framework_rev: $(git -C "$REPO" rev-parse --short HEAD)"
  echo "rtc_framework_dirty: $(git -C "$REPO" status --porcelain | wc -l)"
  echo "rtc_framework_dirty_files: $(git -C "$REPO" status --porcelain | tr '\n' ';')"
  echo "ball_perception_rev: $(git -C $BWS/src/ball_perception rev-parse --short HEAD)"
  echo "profile: $PROFILE"
  echo "profile_sha256: $(sha256sum "$PROFILE" | cut -d' ' -f1)"
  echo "loadavg_start: $(cut -d' ' -f1-3 /proc/loadavg)"
  echo "ros_domain_id: $ROS_DOMAIN_ID"
  echo "expect_mode: ${EXPECT_MODE:-mpc}"; echo "expect_kv: ${EXPECT_KV:-}"
  echo "viewer: $([ "$VIEWER" == "true" ] && echo on || echo off)"
} > "$OUT/conditions.txt"
if [ "${ALLOW_DIRTY:-0}" != "1" ] && [ -n "$(git -C "$REPO" status --porcelain)" ]; then
  echo "FAIL:dirty tree" > "$OUT/status"; exit 1
fi

setsid ros2 launch integrated_bringup $LAUNCH enable_viewer:=$VIEWER use_cpu_affinity:=false \
  enable_mpc:=true sim_lanes:=true sim_overlay:=$OV max_log_sessions:=60 > "$OUT/launch.log" 2>&1 &
LPG=$!
ok=0
for i in $(seq 1 120); do
  # check_startup.py: 0 ready, 1 failed (a failure line that is not an [INFO] line), 2 not yet.
  line=$(python3 "$D/check_startup.py" "$OUT/launch.log" 2>&1 >/dev/null); rc=$?
  [ $rc -eq 0 ] && { ok=1; break; }
  [ $rc -eq 1 ] && { echo "$line" > "$OUT/startup_failure.txt"; break; }
  sleep 1
done
grep -o 'logging_data/[0-9_]*' "$OUT/launch.log" | head -1 > "$OUT/session.txt"
if [ $ok -ne 1 ]; then cleanup; echo "FAIL:startup" > "$OUT/status"; exit 1; fi
sleep 3
grep -o 'commit at t_c − [0-9.]* s' "$OUT/launch.log" | head -1 > "$OUT/commit.txt"
CN=/demo_catching_controller/demo_catching_controller
for P in joint_cmd.lag.T_arm joint_cmd.lag.lead_enable planner.freeze.T_freeze \
         reference.omega reference.a_max reference.v_max control.dt \
         prediction.dt_expected io.n_min planner.search.grid.slice.dt \
         planner.segment.mpc.horizon.n_nodes planner.segment.mpc.horizon.dt_s \
         planner.segment.mpc.approach.n_pre_max planner.segment.mpc.approach.dt_pre_s \
         planner.segment.mpc.approach.rest_tol planner.segment.mpc.replan.k_max \
         planner.segment.mpc.replan.same_point planner.segment.mpc.budget.first_s \
         planner.segment.mpc.budget.replan_s planner.segment.mpc.publish.catch_pos_err_max \
         planner.segment.mpc.publish.slack_max planner.segment.mpc.eta_tau planner.segment.mpc.m_q \
         planner.segment.mpc.catch.gamma_ref planner.segment.mpc.catch.w_v_par \
         planner.segment.mpc.catch.w_v_perp planner.segment.mpc.catch.w_axis \
         planner.segment.mpc.catch.kappa planner.segment.mpc.catch.sigma_floor \
         planner.segment.mpc.catch.w_max planner.segment.mpc.catch.w_const \
         planner.segment.mpc.catch.sigma_ref planner.search.grid.gamma.eta_v \
         planner.search.grid.time.margin planner.search.grid.slice.t_lead_min \
         joint_cmd.accel_constraint planner.segment.mode planner.search.mode \
         planner.segment.mpc.switch_margin \
         planner.segment.mpc_docking.approach.n_pre_max planner.segment.mpc_docking.stop.n_nodes \
         planner.segment.mpc_docking.stop.dt_s \
         planner.segment.mpc_docking.budget.first_s planner.segment.mpc_docking.budget.replan_s \
         planner.segment.mpc.eta_v planner.search.grid.reference.omega \
         planner.search.grid.reference.a_max planner.search.grid.reference.v_max; do
  echo "$P: $(ros2 param get $CN $P 2>&1)" >> "$OUT/mirror.txt"
done
IFS=';' read -ra KVS <<< "${EXPECT_KV:-}"
for kv in "${KVS[@]}"; do
  [ -z "$kv" ] && continue
  K=${kv%%=*}
  grep -q "^$K: " "$OUT/mirror.txt" || echo "$K: $(ros2 param get $CN $K 2>&1)" >> "$OUT/mirror.txt"
done
echo "ball_type: $(ros2 param get /mujoco_simulator projectile_ball.ball_type 2>&1)" >> "$OUT/mirror.txt"
why=""
# The commit lead from the mirror, not the startup log: two processes share that
# log and once split the "−" of its commit line (E0-F04 M-50_p1b_601.fail1).
grep -q "T_arm: Double value is: ${EXPECT_TARM}\$" "$OUT/mirror.txt" || why="$why T_arm"
grep -q 'lead_enable: Boolean value is: True' "$OUT/mirror.txt" || why="$why lead"
grep -q "ball_type: String value is: ${EXPECT_BALL}\$" "$OUT/mirror.txt" || why="$why ball_type"
grep -q "T_freeze: Double value is: ${EXPECT_COMMIT%0}\$\|T_freeze: Double value is: ${EXPECT_COMMIT}\$" "$OUT/mirror.txt" || why="$why T_freeze"
grep -q "^joint_cmd.accel_constraint: String value is: ${EXPECT_CLIK:-dynamic}\$" "$OUT/mirror.txt" || why="$why clik_form"
grep -q "^planner.segment.mode: String value is: ${EXPECT_MODE:-mpc}\$" "$OUT/mirror.txt" || why="$why mode_mirror"
grep -q "^planner.search.mode: String value is: ${EXPECT_SEARCH:-grid}\$" "$OUT/mirror.txt" || why="$why search_mirror"
IFS=';' read -ra KVS <<< "${EXPECT_KV:-}"
for kv in "${KVS[@]}"; do
  [ -z "$kv" ] && continue
  # A plan written before #711 names the old mirror: refuse it by name rather than
  # let the read-back fail as an unexplained missing line.
  case ${kv%%=*} in
    planner.decel_mpc.* | supervisor.decel.mode | supervisor.decel.switch_margin | \
      planner.gamma.eta_v | planner.time.margin | planner.slice.*)
      why="$why old_mirror_name:${kv%%=*}"
      continue
      ;;
  esac
  grep -q "^${kv%%=*}: [A-Za-z]* value is: ${kv#*=}\$" "$OUT/mirror.txt" || why="$why ${kv%%=*}"
done
# The startup lines say the mode too, in both modes (the expected line must be present):
# EXPECT_MODE=closed_form is the same-day control unit (v1 law, no MPC segment planner).
python3 "$D/check_segment_mode.py" "$OUT/launch.log" "${EXPECT_MODE:-mpc}" --search "${EXPECT_SEARCH:-grid}" || why="$why mode_log"
# Each segment planner mirrors and logs its own grid: the mpc planner under
# planner.segment.mpc.*, the docking planner under planner.segment.mpc_docking.*.
if [ "${EXPECT_MODE:-mpc}" == "mpc" ]; then
grep -q "planner.segment.mpc.approach.n_pre_max: Integer value is: ${EXPECT_NPRE:-6}\$" "$OUT/mirror.txt" || why="$why n_pre_max"
grep -q 'planner.segment.mpc.horizon.n_nodes: Integer value is: 7$' "$OUT/mirror.txt" || why="$why n_nodes"
grep -q "MPC segment planner approach grid: up to ${EXPECT_NPRE:-6} x ${EXPECT_DTPRE:-0.100} s" "$OUT/launch.log" || why="$why approach_grid"
grep -q 'takes a plan with its first segment' "$OUT/launch.log" || why="$why not_e1f09_binary"
fi
if [ "${EXPECT_MODE:-mpc}" == "mpc_docking" ]; then
grep -q "planner.segment.mpc_docking.approach.n_pre_max: Integer value is: ${EXPECT_NPRE:-9}\$" "$OUT/mirror.txt" || why="$why n_pre_max"
read -r SHIP_NSTOP SHIP_DTSTOP < <(python3 -c 'import sys, yaml
s = yaml.safe_load(open(sys.argv[1]))["demo_catching_controller"]["catching"]["planner"]["segment"]["mpc_docking"]["stop"]
print(s["n_nodes"], s["dt_s"])' "$REPO/integrated_bringup/config/$ROBOT/controllers/catching/segment_mpc_docking.yaml" 2>/dev/null)
[ -n "${EXPECT_NSTOP:-$SHIP_NSTOP}" ] && [ -n "${EXPECT_DTSTOP:-$SHIP_DTSTOP}" ] || why="$why shipped_stop_unread"
grep -q "planner.segment.mpc_docking.stop.n_nodes: Integer value is: ${EXPECT_NSTOP:-$SHIP_NSTOP}\$" "$OUT/mirror.txt" || why="$why n_nodes"
DTSTOP=$(sed -n 's/^planner.segment.mpc_docking.stop.dt_s: Double value is: //p' "$OUT/mirror.txt")
awk -v a="$DTSTOP" -v b="${EXPECT_DTSTOP:-$SHIP_DTSTOP}" 'BEGIN { exit !(a != "" && b != "" && a - b < 1e-9 && b - a < 1e-9) }' || why="$why stop_dt"
grep -q "mpc_docking segment planner ready: up to ${EXPECT_NPRE:-9} x ${EXPECT_DTPRE:-0.100} s" "$OUT/launch.log" || why="$why approach_grid"
grep -q 'takes a plan with its first segment' "$OUT/launch.log" || why="$why not_e1f09_binary"
fi
grep 'MPC segment planner \(ready\|approach grid\)\|mpc_docking segment planner ready' "$OUT/launch.log" | sed 's/^.*demo_catching_controller\]: //' > "$OUT/segment_startup.txt"
BUDGET=$(grep -o 'planner enabled: wake timeout [0-9.]* s, budget [0-9.]* s' "$OUT/launch.log" | head -1 | sed 's/.*budget \([0-9.]*\) s$/\1/')
echo "planner_budget_s: ${BUDGET}" >> "$OUT/conditions.txt"
[ -n "$BUDGET" ] || why="$why no_budget_line"
if [ -n "$why" ]; then cleanup; echo "FAIL:overlay$why" > "$OUT/status"; exit 1; fi

REV=$(git -C "$REPO" rev-parse --short HEAD)
BREV=$(git -C $BWS/src/ball_perception rev-parse --short HEAD)
( source $BWS/install/local_setup.bash
  [ "$(ros2 pkg prefix ball_perception_sim)" == "$BWS/install/ball_perception_sim" ] || { echo "WRONG_PREFIX"; exit 1; }
  exec setsid ros2 launch ball_perception_sim sim_estimator.launch.py \
    profile_path:=$PROFILE producer_revision:=$BREV \
    simulator_revision:=$REV simulator_seed:=$SEED ) > "$OUT/est.log" 2>&1 &
EPG=$!
act=0
for i in $(seq 1 15); do grep -q 'activated epoch' "$OUT/est.log" && { act=1; break; }; sleep 1; done
grep -q WRONG_PREFIX "$OUT/est.log" && { cleanup; echo "FAIL:ball_perception not from BALL_SIM_WS" > "$OUT/status"; exit 1; }
[ $act -eq 1 ] || { cleanup; echo "FAIL:estimator not activated" > "$OUT/status"; exit 1; }
sleep 2
ros2 service call /rtc_cm/switch_controller rtc_msgs/srv/SwitchController "{activate_controllers: [demo_catching_controller], deactivate_controllers: [demo_joint_controller], strictness: 1, timeout: {sec: 3}}" > "$OUT/switch.log" 2>&1
SERIES=(--throws-file "$THROWS_FILE")
[ -n "$THROWS_LIMIT" ] && SERIES+=(--limit "$THROWS_LIMIT")
ENDRULE=()
if [ "${END_ON_BALL_LOW:-0}" == "1" ]; then
  ENDRULE=(--end-on-ball-low)
  [ -n "$BALL_LOW_MARGIN_M" ] && ENDRULE+=(--ball-low-margin-m "$BALL_LOW_MARGIN_M")
  [ -n "$BALL_LOW_GRACE_S" ] && ENDRULE+=(--ball-low-grace-s "$BALL_LOW_GRACE_S")
fi
timeout $((NT * 30 + 120)) ros2 run integrated_bringup catching_sim_trials "$OUT/trials" \
  --profile $ROBOT "${SERIES[@]}" --host-watch ${HOST_WATCH:-abort} --arm "$COND" "${ENDRULE[@]}" > "$OUT/trials.log" 2>&1
rc=$?
echo "loadavg_end: $(cut -d' ' -f1-3 /proc/loadavg)" >> "$OUT/conditions.txt"
echo "date_end: $(date -Is)" >> "$OUT/conditions.txt"
cleanup
if [ $rc -eq 3 ]; then echo "FAIL:host_busy" > "$OUT/status"; exit 1; fi
if [ $rc -ne 0 ]; then echo "FAIL:trials rc=$rc" > "$OUT/status"; exit 1; fi
# An empty session.txt (no `logging_data/<digits>` in the launch log) would make
# the session the workspace root: the copy below would take the whole workspace.
SREL=$(cat "$OUT/session.txt" 2>/dev/null)
if [ -z "$SREL" ] || [ ! -d "$WS/$SREL" ]; then echo "FAIL:no session directory in the launch log" > "$OUT/status"; exit 1; fi
SES=$WS/$SREL
du -sh "$SES" > "$OUT/session_size.txt"
# A hook `colcon test` that starts in the launch's minute can write fixture CSVs
# into the same time-named session dir: refuse header/row disagreement or foreign files.
for f in "$SES"/controllers/demo_catching_controller/*.csv; do
  h=$(head -1 "$f" | awk -F, '{print NF}'); r=$(sed -n 2p "$f" | awk -F, '{print NF}')
  [ -n "$r" ] && [ "$h" != "$r" ] && { echo "FAIL:contaminated $(basename $f) header $h rows $r" > "$OUT/status"; exit 1; }
done
for f in "$SES"/controllers/demo_catching_controller/*; do
  [ -e "$f" ] || continue
  basename "$f" | grep -q "^\\(catching_diag\\|planner_events\\|nlp_candidates\\|$STATE_RE\\)\\.csv\$" || { echo "FAIL:contaminated foreign file $(basename "$f")" > "$OUT/status"; exit 1; }
done
# Keep the raw session with the unit, then drop the original.
mkdir -p "$OUT/session"
cp -r "$SES"/. "$OUT/session/" || { echo "FAIL:session copy" > "$OUT/status"; exit 1; }
[ -s "$OUT/session/controllers/demo_catching_controller/catching_diag.csv" ] || { echo "FAIL:no diag in copy" > "$OUT/status"; exit 1; }
rm -rf "$SES"
echo DONE > "$OUT/status"
