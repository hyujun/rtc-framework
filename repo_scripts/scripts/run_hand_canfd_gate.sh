#!/usr/bin/env bash
# run_hand_canfd_gate.sh — one CAN FD hand gate run, end to end (G-5 / G-3).
#
#   run_hand_canfd_gate.sh [--sil] [--iface can0] [--sides right,left]
#                          [--duration 300] [--rate 250] [--recv-timeout 0.9]
#                          [--cmd-ack none] [--req-mode per_hand] [--out DIR]
#
# Records the bus, launches udp_hand_node (one node, `hand_sides`) — with
# --sil also one fake_hand_firmware per side on vcan0 — waits until every
# hand's link_status reads true, runs for --duration seconds, ends the run with
# the E-Stop (/hand/estop true, D76), stops the nodes with SIGINT (a SIGKILL
# would lose the CSV tail), then judges each hand with analyze_hand_canfd_gate
# and prints one summary line. Exit 0 when every hand passes, 1 on any FAIL,
# 2 on UNDECIDED without FAIL, 3 when the run could not be made.
#
# --req-mode broadcast (#956) sends one REQ to both hands per tick (launch
# `req_mode:=broadcast`, judged with `--req-mode broadcast`, (x) becomes
# x.broadcast_sets). Both bursts share that one receive window, so
# --recv-timeout defaults to 1.4 ms there and below 1.2 is refused: the left
# hand's REQ -> frame 2 was measured up to 1.154 ms on the rig.
#
# Everything lands in --out (default ./hand_gate_<YYMMDD_HHMMSS>): bus.log
# (candump -L), node.log, link_<side>.log, session/ (RTC_SESSION_DIR — the
# stats JSON per hand and the one timing CSV), gate_<side>.md / .json.
#
# The bus is recorded with candump when it is installed (can-utils, the rig)
# and otherwise with `python3 -m rtc_tools.analysis.candump_log` (same -L
# format; the development PC). Do not run it beside `colcon test`: the vcan0
# cases of test_canfd_hand_comm share the node ids (docs/testing.md). The
# procedure and the thresholds are docs/testing.md "G-5".
set -u

SIL=0
IFACE=can0
SIDES=right,left
DURATION=300
RATE=250
RECV_TIMEOUT=""
CMD_ACK="none"
REQ_MODE=per_hand
OUT=""

usage() { sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'; }

while [ $# -gt 0 ]; do
  case "$1" in
    --sil) SIL=1 ;;
    --iface) IFACE=$2; shift ;;
    --sides) SIDES=$2; shift ;;
    --duration) DURATION=$2; shift ;;
    --rate) RATE=$2; shift ;;
    --recv-timeout) RECV_TIMEOUT=$2; shift ;;
    --cmd-ack) CMD_ACK=$2; shift ;;
    --req-mode) REQ_MODE=$2; shift ;;
    --out) OUT=$2; shift ;;
    -h | --help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage >&2; exit 3 ;;
  esac
  shift
done
[ "$SIL" -eq 1 ] && IFACE=vcan0
case "$REQ_MODE" in
  per_hand) RECV_TIMEOUT=${RECV_TIMEOUT:-0.9} ;;
  broadcast)
    RECV_TIMEOUT=${RECV_TIMEOUT:-1.4}
    if awk -v t="$RECV_TIMEOUT" 'BEGIN { exit !(t + 0 < 1.2) }'; then
      echo "error: --req-mode broadcast needs --recv-timeout >= 1.2 (got $RECV_TIMEOUT):" \
        "the left hand's REQ -> frame 2 reaches 1.154 ms after the right burst" >&2
      exit 3
    fi
    ;;
  *) echo "error: --req-mode must be per_hand or broadcast, got $REQ_MODE" >&2; exit 3 ;;
esac

SELF=$(readlink -f "${BASH_SOURCE[0]}")
REPO=$(cd "$(dirname "$SELF")/../.." && pwd)
# shellcheck disable=SC1091  # sourced from the checkout, not followed by shellcheck
source "$REPO/repo_scripts/scripts/setup_env.sh" >/dev/null 2>&1

OUT=${OUT:-"$PWD/hand_gate_$(date +%y%m%d_%H%M%S)"}
mkdir -p "$OUT/session"
export RTC_SESSION_DIR="$OUT/session"
IFS=',' read -r -a SIDE_LIST <<<"$SIDES"

if [ ! -d "/sys/class/net/$IFACE" ]; then
  echo "error: CAN interface $IFACE not found (vcan0: repo_scripts/scripts/setup_vcan.sh)" >&2
  exit 3
fi

# Job control: every background job gets its own process group and keeps
# SIGINT (a non-interactive shell would start it with SIGINT ignored).
set -m
PIDS=()
cleanup() {
  for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null; done
}
trap cleanup EXIT

if command -v candump >/dev/null 2>&1; then
  candump -L "$IFACE" >"$OUT/bus.log" &
else
  python3 -m rtc_tools.analysis.candump_log "$IFACE" "$OUT/bus.log" &
fi
REC=$!
PIDS+=("$REC")
sleep 1
if ! kill -0 "$REC" 2>/dev/null; then
  echo "error: the bus recorder on $IFACE exited (see above)" >&2
  exit 3
fi

LAUNCH_ARGS=(protocol_version:=2 "hand_sides:=$SIDES" "loop_rate_hz:=$RATE"
  "recv_timeout_ms:=$RECV_TIMEOUT" "cmd_ack:=$CMD_ACK" "req_mode:=$REQ_MODE")
if [ "$SIL" -eq 1 ]; then
  LAUNCH_ARGS+=(sil_mode:=firmware)
else
  LAUNCH_ARGS+=("can_interface:=$IFACE")
fi
ros2 launch udp_hand_driver udp_hand.launch.py "${LAUNCH_ARGS[@]}" >"$OUT/node.log" 2>&1 &
LAUNCH=$!
PIDS+=("$LAUNCH")

for side in "${SIDE_LIST[@]}"; do
  ros2 topic echo "/hand/$side/link_status" std_msgs/msg/Bool \
    --qos-durability transient_local --qos-reliability reliable >"$OUT/link_$side.log" 2>&1 &
  PIDS+=("$!")
done

echo "waiting for link_status true on: ${SIDE_LIST[*]}"
deadline=$((SECONDS + 30))
for side in "${SIDE_LIST[@]}"; do
  until grep -q "data: true" "$OUT/link_$side.log" 2>/dev/null; do
    if [ "$SECONDS" -ge "$deadline" ]; then
      echo "error: /hand/$side/link_status never read true (see $OUT/node.log)" >&2
      kill -INT -- "-$LAUNCH" 2>/dev/null
      exit 3
    fi
    sleep 0.5
  done
done

echo "links up — running ${DURATION} s at ${RATE} Hz"
sleep "$DURATION"

# D76: the run ends with the E-Stop, so (e) has a zero-write pair per hand.
# Bounded: -w waits for the node's subscription, which a node that died
# during the run never brings.
if ! timeout 15 ros2 topic pub --once -w 1 --qos-reliability reliable /hand/estop \
  std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1; then
  echo "warning: the E-Stop was not delivered (is udp_hand_node still up? see $OUT/node.log)" >&2
fi
sleep 1

# SIGINT this run's process group only — the launch and the nodes it spawned
# (set -m gave it its own group; docs/testing.md) — not every udp_hand_node on
# the machine.
kill -INT -- "-$LAUNCH" 2>/dev/null
wait "$LAUNCH" 2>/dev/null
for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null; done
wait 2>/dev/null
trap - EXIT

RC=0
for side in "${SIDE_LIST[@]}"; do
  # One node writes both hands' lines; each hand is judged on its own tag.
  grep -F "[$side] " "$OUT/node.log" >"$OUT/node_$side.log"
  ros2 run rtc_tools analyze_hand_canfd_gate "$OUT/session" "$OUT/bus.log" \
    --hand-side "$side" --explicit-side --timing-suffix "" --loop-rate-hz "$RATE" \
    --req-mode "$REQ_MODE" \
    --min-duration-s "$DURATION" --node-log "$OUT/node_$side.log" \
    --link-status "$OUT/link_$side.log" --json-out "$OUT/gate_$side.json" \
    >"$OUT/gate_$side.md" 2>&1
  rc=$?
  # Worst first: 3 (unusable) > 1 (FAIL) > 2 (UNDECIDED) > 0.
  case "$rc" in
    3) RC=3 ;;
    1) [ "$RC" -ne 3 ] && RC=1 ;;
    2) [ "$RC" -eq 0 ] && RC=2 ;;
  esac
done

python3 - "$OUT" "${SIDE_LIST[@]}" <<'PY'
import json
import sys
from pathlib import Path

out, sides = Path(sys.argv[1]), sys.argv[2:]
parts, overlap, xcheck = [], None, None
for side in sides:
    p = out / f"gate_{side}.json"
    if not p.exists():
        parts.append(f"{side} NO_REPORT")
        continue
    doc = json.loads(p.read_text())
    parts.append(f"{side} {doc['verdict']}")
    if overlap is None and doc.get("overlap"):
        overlap = doc["overlap"]
        xcheck = next((c for c in doc["checks"] if c["key"] == "x.broadcast_sets"), None)
    for c in doc["checks"]:
        if c["verdict"] not in ("PASS", "NOT_EVALUATED"):
            parts.append(f"  {side}:{c['key']}={c['verdict']} ({c['value']})")
line = " · ".join(p for p in parts if not p.startswith("  "))
if overlap:
    gap = overlap.get("estop_gap_ms")
    b = overlap.get("broadcast")
    if b is not None:
        # The violations count leaves out the one-hand-missing rate: show the verdict.
        verdict = xcheck["verdict"] if xcheck else "NO_CHECK"
        line += (
            f" · x.broadcast_sets {verdict} (violations {overlap['violations']} · "
            f"one-hand-missing {b['one_hand_missing']}/{b['windows']})"
        )
    else:
        line += f" · overlap {overlap['violations']}"
    line += " · zero-write gap " + ("missing" if gap is None else f"{gap:+.3f} ms")
print(f"hand gate: {line}  [{out}]")
for p in parts:
    if p.startswith("  "):
        print(p)
PY
exit "$RC"
