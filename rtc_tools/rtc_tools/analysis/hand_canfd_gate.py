"""G-3 gate judge for the CAN FD hand link (epic #895 D55, H1-F06 #904).

One standalone run of ``udp_hand.launch.py`` (protocol_version 2) against the
real hand must satisfy, all at once:

  (a) duration  — the timing CSV spans >= 300 s, one run_id, no tick gap
  (b) link      — never down: stats JSON link_ok and not failure_detected, zero
                  "Hand link DOWN" lines in the node log, no silent REQ streak on
                  the bus that reaches the link-fail budget, no `false` on
                  /…/link_status
  (c) cycle p99 — t_total_us p99 < period / 1.1 (1818 us at 500 Hz), nothing
                  excluded
  (d) timeouts  — RESP-set timeouts / comm cycles < 0.1 % AND echo timeouts /
                  write attempts < 0.1 %, each on its own denominator; with
                  ``cmd_ack: none`` there is no echo stage, so the echo row is
                  NOT_EVALUATED and only the RESP-set rate is judged
  (e) E-Stop    — on the bus: Frame 1 zeros -> Frame 0 zeros (the D26 order,
                  which is the REVERSE of a normal write) -> both echoes, within
                  2 ms of the first zero frame, and no command after it; with
                  ``cmd_ack: none`` the hand sends no echoes, so the pair's own
                  order and spacing (F1 -> F0 < 2 ms) replace the echo timing
  (x) overlap   — two hands on one bus only (G-5, epic #906): no frame of one
                  hand inside the other hand's transaction (below). Absent from
                  a one-hand report, which therefore reads as before. Under
                  ``req_mode: broadcast`` it is ``x.broadcast_sets`` instead.

Two hands (G-5). One node drives both hands, back to back every tick (D69),
so their transactions must not interleave on the bus. A transaction of a hand
opens at the host's first frame to it — CMD frame 0 of a write tick, the REQ of
a read-only tick — and closes at that hand's RESP frame 2; when frame 2 never
comes, the host's first frame to the OTHER hand closes it (the host moved on
after the deadline, which is not an overlap). A violation is a frame FROM a hand
(echo or RESP) that lands while the other hand's transaction is open: it is
attributed to the hand that sent it (the intruder), so a late tail of one hand
falling into the next hand's window counts against the late hand. Order is the
candump row order, not timestamps. Everything from the first E-Stop zero frame
on is outside the check: the abort zero-writes every hand fire-and-forget, so
their echoes may interleave by design, and the loop has stopped. The gap
between the two hands' E-Stop zero frames is reported, not judged (D78). Each
hand is judged by its own run of this tool (``--hand-side``); under
``hand_sides`` the stats JSON carries ``_<side>`` and the one timing CSV does
not, hence ``--explicit-side --timing-suffix ""``.

Broadcast REQ (#956). With ``req_mode: broadcast`` (stats JSON
``canfd.req_mode``, absent in older archives and then read as ``per_hand``;
``--req-mode`` overrides) the host sends each hand its own CMD pair and then
ONE REQ to target 0xFF (``0x007FC000``), which both hands answer with their own
RESP set; the hands' arbitration, not the host, orders the two bursts. That REQ
is each hand's REQ for (a)–(e) — without it a hand would see no REQ, and two
consecutive zero writes (F0 F1 | REQ | F0 F1) would read as E-Stop pairs. The
transaction check cannot apply (both bursts answer one frame), so (x) becomes
``x.broadcast_sets``, per window from one broadcast REQ to the next, cut at the
first E-Stop zero frame exactly as above. The window's tick part ends at the
first host CMD more than half a period after the REQ (the next tick's write):
a RESP after that point came after the host gave up on the set — the node
counted a set timeout, which (d) judges — so it is reported as late, is no
violation and does not complete the set. Within the tick part (1) no hand's
RESP index twice and (2) no host CMD before the tick part's last RESP (the host
sent during its own tick's burst) — either FAILS — and (3) the windows where
exactly one hand's set is incomplete while the other's is complete, judged as a
rate with (d)'s threshold and UNDECIDED band; windows where every set is
missing are (d)'s per hand. A RESP later than the next REQ falls into the next
window and is judged there. A last window the cut leaves incomplete is not
judged for (3). The
RESP order, the per-hand REQ -> own frame 2 (the later hand's figure includes
the other hand's burst) and the write tick's first CMD frame 0 -> last frame 2
are reported. A bus that contradicts the mode — a broadcast REQ judged as
per_hand, or a per-hand REQ judged as broadcast — or an explicit ``--req-mode``
the stats JSON contradicts is exit 3 (unusable input), never a verdict.

Why each definition is what it is (the reviewed corrections of H1-F06):

  * The aggregate ``recv_timeout`` counter is bumped by BOTH the echo stage and
    the RESP-set stage, so ``recv_timeout / total_cycles`` can reach 2 per cycle
    and is not a rate. (d) therefore reads the per-request counters and divides
    each by the number of times that stage ran; the aggregate is reported as an
    upper bound only.
  * A timed-out tick's ``t_total_us`` is the real tick time; what the deadline
    truncates is the round trip, not the tick. (c) excludes nothing.
  * A zero payload alone does not identify the E-Stop: a run that commands zeros
    at 500 Hz (Run B-2) writes zero pairs all along. The ORDER does — a normal
    write sends Frame 0 then Frame 1, the E-Stop sends Frame 1 then Frame 0
    (D26 · D33) — and the loop exits after it, so nothing may follow.
  * Since #919 (D49 (b) · D59) a write tick is pipelined: CMD 0 · CMD 1 · REQ
    go out back to back and the echoes and the RESP burst share one receive
    window, so echoes and RESP frames may interleave on the bus (echo mode;
    with ``cmd_ack: none`` only the RESP burst remains). The REQ -> RESP round
    trip of such a tick includes the hand's echo handling and is not
    comparable with a read-only tick's: the read-only ticks' round trips
    are reported on their own (the H1-F06 baseline) and the write tick's
    hand-side figure is CMD frame 0 -> RESP frame 2, both beside the mixed row.
  * ``cmd_ack`` (stats JSON ``canfd.cmd_ack``, absent in older archives and
    then read as ``echo``) says whether the firmware echoes the two CMD frames.
    With ``none`` the bus carries no ECHO frames: the echo stage does not exist
    (nothing to time out), and a stray echo is counted by the driver as
    ``echo_unexpected`` rather than judged here.
  * (d) is a proportion estimated from one run. Near the threshold the run
    cannot decide it: a count within ``undecided_sigmas`` binomial standard
    deviations of the threshold count (±20 counts at 150 000 cycles) is
    UNDECIDED (repeat once) instead of PASS or FAIL by luck. The band scales
    with the denominator, so a small write count cannot hide a gross failure.
  * The per-stage ``ok`` / ``timeout`` / ``error`` counters fire once per
    attempt — every echo stage ends on both echoes, the deadline or a recv
    error (D58); ``cmd_mismatch`` fires once per unexpected FRAME, any number
    per attempt. The attempt denominator therefore sums only the
    once-per-attempt counters, and the bus write-pair count is reported beside
    it as the cross-check.
  * (c) is judged against ``--loop-rate-hz``; the stats JSON carries no rate,
    so the CSV's own median tick period must agree with it or (c) FAILS with
    the reason instead of judging against the wrong threshold.
  * candump stamps the loopback copy of a transmitted frame at TX-done, so
    every bus-side round trip here is a lower bound on what the node saw.

Inputs are the forensic artifacts the node writes under the session directory
(``device/hand_stats[_<side>].json``, ``timing/hand_timing_log[_<side>].csv``,
D51 suffix when ``hand_side`` was explicit; sessions recorded before the F08
rename, #905, are read under their old ``hand_udp_*`` names), a ``candump -L`` log, and
optionally the node's console log and a ``ros2 topic echo`` of link_status.
Exit status: 0 no check FAIL or UNDECIDED (NOT_EVALUATED checks are listed, not passed), 2 at least one UNDECIDED and no
FAIL, 1 any FAIL, 3 unusable input.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path

from rtc_tools.analysis.candump_log import (
    NODE_LEFT_HAND,
    NODE_RIGHT_HAND,
    Frame,
    Kind,
    broadcast_req_id,
    classify,
    node_id_of,
    read_candump,
    req_id,
)
from rtc_tools.analysis.hand_close import quantile
from rtc_tools.utils.session_dir import HAND_STATS_STEM, HAND_TIMING_STEM, PRE_F08_HAND_STEMS

PASS = "PASS"
FAIL = "FAIL"
UNDECIDED = "UNDECIDED"
NOT_EVALUATED = "NOT_EVALUATED"

TIMING_STEM = HAND_TIMING_STEM
STATS_STEM = HAND_STATS_STEM
LINK_DOWN_NEEDLE = "Hand link DOWN"
FAILURE_NEEDLE = "Hand failure detected: "
# Failure-detector reasons that judge the SENSOR payload, not the link: a hand
# whose sensors read zero (none fitted, or motors off) latches these within
# the first 100 ms of every run, which says nothing about link-down 0 (D55).
SENSOR_HEURISTIC_REASONS = ("hand_sensor_all_zero", "hand_sensor_duplicate")
# The per-request-kind outcome counters that fire ONCE per attempt (fixture
# udp_hand_driver/test/fixtures/hand_stats_keys_proto2.txt lists the full
# set; cmd_mismatch / mode_mismatch are per frame and excluded on purpose).
ATTEMPT_COUNTERS = ("ok", "timeout", "error", "short_or_decode")
# Values of the node's `req_mode` (stats JSON canfd.req_mode, #956).
REQ_PER_HAND = "per_hand"
REQ_BROADCAST = "broadcast"
REQ_MODES = (REQ_PER_HAND, REQ_BROADCAST)


@dataclass
class Thresholds:
    min_duration_s: float = 300.0
    loop_rate_hz: float = 500.0
    p99_fraction: float = 1 / 1.1  # D89: period = p99 x 1.1
    timeout_rate_max: float = 1e-3
    # Half-width of the UNDECIDED band around the threshold count, in binomial
    # standard deviations sqrt(n p (1 - p)) of the threshold rate: 1.65 sigma is
    # ±20 counts at 150 000 cycles (130..170), ±1.6 at 1 000 writes.
    undecided_sigmas: float = 1.65
    # How far the CSV's median tick period may sit from 1 / loop_rate_hz before
    # (c) refuses to judge against that threshold.
    period_tolerance: float = 0.05
    estop_window_ms: float = 2.0
    # A silent REQ streak of this many is the link-fail budget: 100 ms at
    # 500 Hz, the shipped link_fail_timeout_ms (a streak that long would have
    # tripped the node's own link_ok, which is why the two must agree). The
    # CLI derives it from --link-fail-timeout-ms and the run's rate
    # (silent_req_budget) unless --silent-req-max is given.
    silent_req_max: int = 50

    @property
    def period_us(self) -> float:
        return 1e6 / self.loop_rate_hz

    @property
    def p99_limit_us(self) -> float:
        return self.p99_fraction * self.period_us


def _p99_rule(fraction: float) -> str:
    """The (c) limit as written: ``period / 1.1`` for the default (D89)."""
    return f"period / {1 / fraction:g}" if fraction > 0 else f"{fraction:g} x period"


@dataclass
class Check:
    key: str
    verdict: str
    value: str
    threshold: str
    note: str = ""


@dataclass
class TimingSummary:
    rows: int = 0
    span_s: float = math.nan
    run_ids: list[int] = field(default_factory=list)
    tick_gaps: int = 0
    period_us_median: float = math.nan
    p50_total_us: float = math.nan
    p99_total_us: float = math.nan
    max_total_us: float = math.nan
    over_limit: int = 0
    over_period: int = 0
    p99_jitter_us: float = math.nan
    max_jitter_us: float = math.nan


@dataclass
class EstopEvent:
    t_cmd1: float
    t_cmd0: float
    t_echo1: float | None
    t_echo0: float | None
    cmd_frames_after: int

    @property
    def delta_ms(self) -> float:
        if self.t_echo0 is None or self.t_echo1 is None:
            return math.nan
        return (max(self.t_echo0, self.t_echo1) - self.t_cmd1) * 1e3


@dataclass
class BusSummary:
    frames: int = 0
    unparsed_lines: int = 0
    error_frames: int = 0
    cmd: int = 0
    echo: int = 0
    req: int = 0
    resp: int = 0
    other: int = 0
    nonzero_dlc_req: int = 0
    complete_sets: int = 0
    incomplete_sets: int = 0
    max_silent_req_streak: int = 0
    write_pairs: int = 0  # normal Frame 0 -> Frame 1 command pairs seen on the bus
    rtt_req_f2_ms: list[float] = field(default_factory=list)
    rtt_cmd_echo_ms: list[float] = field(default_factory=list)
    # Write tick, pipelined (#919): CMD frame 0 -> RESP frame 2 of the REQ that
    # follows the pair within `cmd_req_max_s`.
    rtt_cmd_f2_ms: list[float] = field(default_factory=list)
    # REQ -> RESP frame 2 of the ticks that wrote nothing: the hand's bare
    # round trip, comparable with the read-only baseline (H1-F06 Run B-1).
    rtt_req_f2_readonly_ms: list[float] = field(default_factory=list)
    estop: EstopEvent | None = None
    estop_candidates: int = 0


HAND_NODES = (NODE_RIGHT_HAND, NODE_LEFT_HAND)
HAND_NAMES = {NODE_RIGHT_HAND: "right", NODE_LEFT_HAND: "left"}


@dataclass
class Violation:
    line_no: int
    intruder: str  # the hand that sent the frame
    victim: str  # the hand whose transaction was open
    kind: str


@dataclass
class BroadcastSummary:
    """The two-hand bus check (x) of a ``req_mode: broadcast`` run; see the module note.

    A window runs from one broadcast REQ to the next (or to the E-Stop cut);
    its tick part ends at the first host CMD more than `cmd_req_max_s` after
    the REQ, and RESP frames after that are late (counted, not judged).
    When the cut leaves a set of the last window incomplete, that window is
    checked for (1) and (2) but not judged for (3): its set was cut short, not
    lost."""

    windows: int = 0  # judged windows
    truncated: int = 0  # the last window, when the E-Stop cut left a set incomplete (0 or 1)
    complete: int = 0  # every hand's set complete
    one_hand_missing: int = 0  # exactly one hand's set incomplete — the judged count ((3))
    all_missing: int = 0  # every hand's set incomplete — (d) judges these per hand
    missing: dict[str, int] = field(default_factory=dict)  # per hand, any incomplete window
    one_hand_missing_by_hand: dict[str, int] = field(default_factory=dict)
    duplicates: int = 0  # (1) a hand's RESP idx twice in one tick part
    cmd_in_burst: int = 0  # (2) a host CMD of this tick before the tick part's last RESP
    # RESP frames after the next tick's write, before the next REQ: the host had
    # given up on the set (a set timeout, judged by (d)); per hand, reported only.
    late_resp: dict[str, int] = field(default_factory=dict)
    order: dict[str, int] = field(default_factory=dict)  # RESP pattern (R/L per frame) -> windows
    # REQ (0xFF) -> each hand's own RESP frame 2: the later hand's figure
    # includes the other hand's burst.
    req_f2_ms: dict[str, list[float]] = field(default_factory=dict)
    # Write ticks: first CMD frame 0 before the REQ -> the last RESP frame 2.
    cmd_f2_ms: list[float] = field(default_factory=list)


@dataclass
class OverlapSummary:
    """The two-hand bus check (x); see the module note."""

    hands_on_bus: list[str] = field(default_factory=list)
    transactions: dict[str, int] = field(default_factory=dict)
    # per_hand: intrusions; broadcast: (1) duplicate RESP and (2) CMD in the burst.
    violations: list[Violation] = field(default_factory=list)
    # Candump line of the first E-Stop zero frame; the check stops there.
    estop_line: int | None = None
    # Each hand's E-Stop Frame 1 zeros time (D78: right -> left gap, reported).
    estop_f1_t: dict[str, float] = field(default_factory=dict)
    # Set in broadcast mode only: the per-window check replaces the transactions.
    broadcast: BroadcastSummary | None = None

    @property
    def two_hands(self) -> bool:
        return len(self.hands_on_bus) >= 2

    @property
    def by_pair(self) -> dict[str, int]:
        out: dict[str, int] = {}
        for v in self.violations:
            key = f"{v.intruder} -> {v.victim}"
            out[key] = out.get(key, 0) + 1
        return out

    @property
    def estop_gap_ms(self) -> float:
        r = self.estop_f1_t.get("right")
        left = self.estop_f1_t.get("left")
        if r is None or left is None:
            return math.nan
        return (left - r) * 1e3


def silent_req_budget(
    link_fail_timeout_ms: float, loop_rate_hz: float, comm_decimation: int = 1
) -> int:
    """The node's link-fail cycle count (LinkFailCyclesFromTimeoutMs): REQs per budget.

    Rounded half away from zero like std::lround (Python's round() takes 12.5
    to 12), decimation clamped to >= 1 like ClampCommDecimation."""
    dec = max(1, comm_decimation)
    return max(1, math.floor(link_fail_timeout_ms / 1000.0 * loop_rate_hz / dec + 0.5))


# ── timing CSV ───────────────────────────────────────────────────────────────


def load_timing_csv(path: Path) -> list[dict[str, int]]:
    """Rows with the integer columns the gate reads (t_wall_ns, tick_count, run_id, t_total_us, jitter_us)."""
    wanted = ("t_wall_ns", "tick_count", "run_id", "t_total_us", "jitter_us")
    rows: list[dict[str, int]] = []
    with path.open(newline="") as fh:
        reader = csv.DictReader(fh)
        missing = [c for c in wanted if c not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f"{path}: timing CSV lacks column(s) {missing}")
        for raw in reader:
            try:
                rows.append({c: _as_int(raw[c]) for c in wanted})
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{path}: non-numeric row {raw!r}") from exc
    return rows


def _as_int(text: str) -> int:
    """Exact for the integer columns (t_wall_ns exceeds a double's 53-bit mantissa)."""
    try:
        return int(text)
    except ValueError:
        return int(float(text))


def summarise_timing(
    rows: list[dict[str, int]], limit_us: float, period_us: float
) -> TimingSummary:
    s = TimingSummary(rows=len(rows))
    s.run_ids = sorted({r["run_id"] for r in rows})
    if not rows:
        return s
    t_wall = [r["t_wall_ns"] for r in rows]
    s.span_s = (max(t_wall) - min(t_wall)) / 1e9
    ticks = [r["tick_count"] for r in rows]
    s.tick_gaps = sum(1 for a, b in zip(ticks, ticks[1:], strict=False) if b - a != 1)
    if len(t_wall) > 1:
        dts = sorted((b - a) / 1e3 for a, b in zip(t_wall, t_wall[1:], strict=False))
        s.period_us_median = dts[len(dts) // 2]
    total = [float(r["t_total_us"]) for r in rows]
    s.p50_total_us = quantile(total, 0.50)
    s.p99_total_us = quantile(total, 0.99)
    s.max_total_us = max(total)
    s.over_limit = sum(1 for v in total if v >= limit_us)
    s.over_period = sum(1 for v in total if v >= period_us)
    jitter = [float(r["jitter_us"]) for r in rows]
    s.p99_jitter_us = quantile(jitter, 0.99)
    s.max_jitter_us = max(jitter)
    return s


# ── stats JSON ───────────────────────────────────────────────────────────────


def load_comm_stats(path: Path) -> dict:
    with path.open() as fh:
        doc = json.load(fh)
    try:
        comm = doc["comm_stats"]
    except (KeyError, TypeError) as exc:
        raise ValueError(f"{path}: no comm_stats block") from exc
    if comm.get("transport") != "canfd":
        raise ValueError(f"{path}: transport is {comm.get('transport')!r}, not a CAN FD run")
    return comm


def stage_attempts(comm: dict, kind: str) -> int:
    """Attempts of stage `kind` whose outcome was counted once (see the module note)."""
    stage = comm.get("per_request", {}).get(kind, {})
    return sum(int(stage.get(c, 0)) for c in ATTEMPT_COUNTERS)


def stage_timeouts(comm: dict, kind: str) -> int:
    return int(comm.get("per_request", {}).get(kind, {}).get("timeout", 0))


# ── bus (candump) ────────────────────────────────────────────────────────────


def analyse_bus(
    frames: list[Frame],
    node: int,
    unparsed_lines: int = 0,
    cmd_req_max_s: float = 1e-3,
    *,
    broadcast_req: bool = False,
) -> BusSummary:
    """Per-hand counts, REQ -> RESP completeness, round trips and the E-Stop event.

    `cmd_req_max_s` bounds how long after CMD frame 0 the write tick's own REQ
    may follow (half a period from the CLI): a tick whose REQ never left
    (send failure) must not borrow the next read-only tick's REQ.
    `broadcast_req` (``req_mode: broadcast``) makes the broadcast REQ this
    hand's REQ, so the sets, the write ticks and the E-Stop order read as they
    do per hand."""
    s = BusSummary(frames=len(frames), unparsed_lines=unparsed_lines)
    kinds = [classify(f, node, broadcast_req=broadcast_req) for f in frames]
    for f, k in zip(frames, kinds, strict=True):
        if k is Kind.ERROR:
            s.error_frames += 1
        elif k is Kind.CMD:
            s.cmd += 1
        elif k is Kind.ECHO:
            s.echo += 1
        elif k is Kind.REQ:
            s.req += 1
            if f.data:
                s.nonzero_dlc_req += 1
        elif k is Kind.RESP:
            s.resp += 1
        else:
            s.other += 1

    # REQ -> RESP sets. A set is complete when frames 0, 1 and 2 all arrived
    # before the next REQ. The RTT is REQ -> frame 2, the last of the set. The
    # echoes of a pipelined write tick (#919) interleave with the burst on the
    # bus; they are another Kind and do not touch the set. Each REQ's frame-2
    # time is kept (by REQ index) for the write-tick figure below.
    open_req: Frame | None = None
    open_req_index = -1
    seen: set[int] = set()
    t_f2: float | None = None
    silent = 0
    f2_by_req: dict[int, tuple[float, float]] = {}  # REQ index -> (t_req, t_f2)

    def close_set() -> None:
        nonlocal silent
        if open_req is None:
            return
        if seen >= {0, 1, 2}:
            s.complete_sets += 1
            silent = 0
            if t_f2 is not None:
                s.rtt_req_f2_ms.append((t_f2 - open_req.t) * 1e3)
                f2_by_req[open_req_index] = (open_req.t, t_f2)
        else:
            s.incomplete_sets += 1
            silent += 1
            s.max_silent_req_streak = max(s.max_silent_req_streak, silent)

    for i, (f, k) in enumerate(zip(frames, kinds, strict=True)):
        if k is Kind.REQ:
            close_set()
            open_req, open_req_index, seen, t_f2 = f, i, set(), None
        elif k is Kind.RESP and open_req is not None:
            seen.add(f.frame_idx)
            if f.frame_idx == 2:
                t_f2 = f.t
    close_set()

    # CMD pairs and their echoes. Normal write: frame 0 then frame 1 (both
    # echoed, D41). E-Stop: frame 1 then frame 0, both zero (D26 · D33). Two
    # consecutive normal writes also put a frame 1 next to a frame 0 — but a
    # REQ always lies between them (every comm cycle sends its REQ after the
    # CMD pair, pipelined since #919 or sequential before), whereas the E-Stop
    # pair is sent back to back with no REQ (the loop exits), so "no REQ in
    # between" is what separates the two. Echoes are searched up to the next
    # command frame after the pair, never into the following write; the REQ,
    # the RESP burst and the echoes of one tick all lie in that window
    # whatever their interleaving.
    cmd_frames = [
        (i, f) for i, (f, k) in enumerate(zip(frames, kinds, strict=True)) if k is Kind.CMD
    ]
    req_indices = [i for i, k in enumerate(kinds) if k is Kind.REQ]

    def req_between(lo: int, hi: int) -> bool:
        # req_indices is ascending (frame order): two bisections, not a scan —
        # a 5 min run that writes every cycle has ~300k pairs x ~150k REQs.
        return bisect.bisect_right(req_indices, lo) < bisect.bisect_left(req_indices, hi)

    def first_req_in(lo: int, hi: int) -> int | None:
        j = bisect.bisect_right(req_indices, lo)
        return req_indices[j] if j < len(req_indices) and req_indices[j] < hi else None

    write_reqs: set[int] = set()
    for n, ((i_a, a), (i_b, b)) in enumerate(zip(cmd_frames, cmd_frames[1:], strict=False)):
        window_end = cmd_frames[n + 2][0] if n + 2 < len(cmd_frames) else len(frames)
        if a.frame_idx == 0 and b.frame_idx == 1 and not req_between(i_a, i_b):
            s.write_pairs += 1
            t0, t1 = _echoes_between(frames, kinds, i_a, window_end)
            if t0 is not None and t1 is not None:
                s.rtt_cmd_echo_ms.append((max(t0, t1) - a.t) * 1e3)
            # The write tick's REQ is the first after the pair, before the next
            # command and within cmd_req_max_s of CMD frame 0 (not the next
            # tick's); its frame 2 (when the set completed) closes the tick's
            # hand-side round trip.
            i_req = first_req_in(i_b, window_end)
            if i_req is not None and frames[i_req].t - a.t <= cmd_req_max_s:
                write_reqs.add(i_req)
                if i_req in f2_by_req:
                    s.rtt_cmd_f2_ms.append((f2_by_req[i_req][1] - a.t) * 1e3)
        elif (
            a.frame_idx == 1
            and b.frame_idx == 0
            and a.all_zero
            and b.all_zero
            and not req_between(i_a, i_b)
        ):
            s.estop_candidates += 1
            t0, t1 = _echoes_between(frames, kinds, i_a, window_end)
            s.estop = EstopEvent(
                t_cmd1=a.t,
                t_cmd0=b.t,
                t_echo1=t1,
                t_echo0=t0,
                cmd_frames_after=len(cmd_frames) - (n + 2),
            )
    s.rtt_req_f2_readonly_ms = [
        (t_f2 - t_req) * 1e3 for i, (t_req, t_f2) in f2_by_req.items() if i not in write_reqs
    ]
    return s


def _echoes_between(
    frames: list[Frame], kinds: list[Kind], start: int, end: int
) -> tuple[float | None, float | None]:
    """Timestamps of the first echo of frame 0 and of frame 1 in frames (start, end)."""
    t0: float | None = None
    t1: float | None = None
    for f, k in zip(frames[start + 1 : end], kinds[start + 1 : end], strict=True):
        if k is Kind.ECHO:
            if f.frame_idx == 0 and t0 is None:
                t0 = f.t
            elif f.frame_idx == 1 and t1 is None:
                t1 = f.t
            if t0 is not None and t1 is not None:
                break
    return t0, t1


# ── two hands on one bus (x) ────────────────────────────────────────────────


Role = tuple[frozenset[int], Kind]
_NO_HAND: frozenset[int] = frozenset()


def _hand_of(frame: Frame) -> Role:
    """(hand nodes, kind) of a hand-protocol frame, else (empty, OTHER).

    One node for every frame but the broadcast REQ, which is a REQ to every
    hand (#956)."""
    if not frame.is_error and frame.can_id == broadcast_req_id():
        return frozenset(HAND_NODES), Kind.REQ
    for node in HAND_NODES:
        k = classify(frame, node)
        if k in (Kind.CMD, Kind.ECHO, Kind.REQ, Kind.RESP):
            return frozenset((node,)), k
        if k is Kind.ERROR:
            return _NO_HAND, k
    return _NO_HAND, Kind.OTHER


def _estop_rows(frames: list[Frame], roles: list[Role]) -> dict[int, int]:
    """Each hand's first E-Stop pair: row of CMD Frame 1 zeros, when CMD Frame 0
    zeros is the next command frame to that hand with no REQ to it in between
    (a broadcast REQ is a REQ to every hand)."""
    out: dict[int, int] = {}
    pending: dict[int, int] = {}  # hand -> row of a zero F1 awaiting its F0
    for i, ((nodes, k), f) in enumerate(zip(roles, frames, strict=True)):
        if k is Kind.REQ:
            for n in nodes:
                pending.pop(n, None)
            continue
        if len(nodes) != 1:
            continue
        (node,) = nodes
        if node in out:
            continue
        if k is Kind.CMD:
            if f.frame_idx == 1 and f.all_zero:
                pending[node] = i
            elif f.frame_idx == 0 and f.all_zero and node in pending:
                out[node] = pending.pop(node)
            else:
                pending.pop(node, None)
    return out


def analyse_overlap(
    frames: list[Frame], req_mode: str = REQ_PER_HAND, cmd_req_max_s: float = 1e-3
) -> OverlapSummary:
    """Frames of one hand inside the other hand's open transaction (module note).

    In ``broadcast`` mode the per-window check (`BroadcastSummary`) replaces
    the transactions; `cmd_req_max_s` bounds its write-tick figure as in
    `analyse_bus`."""
    s = OverlapSummary()
    roles = [_hand_of(f) for f in frames]
    # A hand is on the bus when a frame to or from it alone is: the broadcast
    # REQ names every hand whether it is fitted or not.
    on_bus = {n for nodes, _ in roles if len(nodes) == 1 for n in nodes}
    s.hands_on_bus = [HAND_NAMES[n] for n in HAND_NODES if n in on_bus]
    estops = _estop_rows(frames, roles)
    for node, row in estops.items():
        s.estop_f1_t[HAND_NAMES[node]] = frames[row].t
    cut = min(estops.values()) if estops else len(frames)
    if estops:
        s.estop_line = frames[cut].line_no

    if req_mode == REQ_BROADCAST:
        hands = [n for n in HAND_NODES if n in on_bus]
        s.broadcast = _analyse_broadcast(
            frames, roles, hands, cut, bool(estops), cmd_req_max_s, s.violations
        )
        return s

    latest: int | None = None  # hand of the most recently opened transaction
    is_open = False
    has_req = False
    has_f1 = False
    for f, (nodes, k) in zip(frames[:cut], roles[:cut], strict=True):
        if len(nodes) != 1:
            continue
        (node,) = nodes
        if k in (Kind.CMD, Kind.REQ):
            starts = (
                latest != node
                or (k is Kind.CMD and f.frame_idx == 0)
                or (k is Kind.REQ and has_req)
                or (k is Kind.CMD and f.frame_idx == 1 and has_f1)
            )
            if starts:
                latest, is_open, has_req, has_f1 = node, True, False, False
                name = HAND_NAMES[node]
                s.transactions[name] = s.transactions.get(name, 0) + 1
            has_req = has_req or k is Kind.REQ
            has_f1 = has_f1 or (k is Kind.CMD and f.frame_idx == 1)
            continue
        # A frame from a hand (echo or RESP).
        if latest is not None and is_open and node != latest:
            s.violations.append(Violation(f.line_no, HAND_NAMES[node], HAND_NAMES[latest], str(k)))
        elif node == latest and k is Kind.RESP and f.frame_idx == 2:
            is_open = False
    return s


def _analyse_broadcast(
    frames: list[Frame],
    roles: list[Role],
    hands: list[int],
    cut: int,
    cut_by_estop: bool,
    cmd_req_max_s: float,
    violations: list[Violation],
) -> BroadcastSummary:
    """Per broadcast-REQ window: (1) duplicate RESP, (2) host CMD in the burst,
    (3) one hand's set missing while the other's is complete (module note)."""
    b = BroadcastSummary(
        missing={HAND_NAMES[n]: 0 for n in hands},
        one_hand_missing_by_hand={HAND_NAMES[n]: 0 for n in hands},
        late_resp={HAND_NAMES[n]: 0 for n in hands},
        req_f2_ms={HAND_NAMES[n]: [] for n in hands},
    )
    bcast = broadcast_req_id()
    req_rows = [i for i in range(cut) if roles[i][1] is Kind.REQ and frames[i].can_id == bcast]
    letter = {NODE_RIGHT_HAND: "R", NODE_LEFT_HAND: "L"}

    def first_f0(lo: int, hi: int) -> float | None:
        for j in range(lo, hi):
            if roles[j][1] is Kind.CMD and frames[j].frame_idx == 0:
                return frames[j].t
        return None

    # The write CMDs of a tick go out before its REQ: before the first REQ, and
    # after the previous window's last RESP for every later one.
    pre_f0 = first_f0(0, req_rows[0]) if req_rows else None
    for w, lo in enumerate(req_rows):
        hi = req_rows[w + 1] if w + 1 < len(req_rows) else cut
        cut_short = cut_by_estop and w + 1 == len(req_rows)
        t_req = frames[lo].t
        # The tick part ends at the next tick's write: the first host CMD more
        # than cmd_req_max_s after this REQ.
        tick_end = next(
            (
                j
                for j in range(lo + 1, hi)
                if roles[j][1] is Kind.CMD and frames[j].t - t_req > cmd_req_max_s
            ),
            hi,
        )
        for j in range(tick_end, hi):
            nodes, k = roles[j]
            if k is Kind.RESP and len(nodes) == 1 and next(iter(nodes)) in hands:
                b.late_resp[HAND_NAMES[next(iter(nodes))]] += 1
        seen: dict[int, set[int]] = {n: set() for n in hands}
        t_f2: dict[int, float] = {}
        pattern: list[str] = []
        last_resp = None
        for j in range(lo + 1, tick_end):
            nodes, k = roles[j]
            if k is not Kind.RESP or len(nodes) != 1:
                continue
            (node,) = nodes
            if node not in seen:
                continue
            last_resp = j
            idx = frames[j].frame_idx
            pattern.append(letter[node])
            if idx in seen[node]:
                b.duplicates += 1
                name = HAND_NAMES[node]
                violations.append(Violation(frames[j].line_no, name, name, "duplicate_resp"))
            seen[node].add(idx)
            if idx == 2 and node not in t_f2:
                t_f2[node] = frames[j].t
        # (2) a host CMD of this tick (all CMDs before tick_end are within
        # cmd_req_max_s) before the tick part's last RESP; the ones after it are
        # the next tick's write, sent ahead of the next REQ.
        next_lo = lo + 1 if last_resp is None else last_resp + 1
        for j in range(lo + 1, next_lo):
            nodes, k = roles[j]
            if k is Kind.CMD:
                b.cmd_in_burst += 1
                victim = HAND_NAMES[next(iter(nodes))]
                violations.append(Violation(frames[j].line_no, "host", victim, "cmd_in_burst"))
        this_f0, pre_f0 = pre_f0, first_f0(next_lo, hi)
        incomplete = [n for n in hands if not seen[n] >= {0, 1, 2}]
        if cut_short and incomplete:
            b.truncated += 1
            continue
        b.windows += 1
        key = "".join(pattern) or "-"
        b.order[key] = b.order.get(key, 0) + 1
        for n in incomplete:
            b.missing[HAND_NAMES[n]] += 1
        if not incomplete:
            b.complete += 1
        elif len(incomplete) == len(hands):
            b.all_missing += 1
        elif len(incomplete) == 1:
            b.one_hand_missing += 1
            b.one_hand_missing_by_hand[HAND_NAMES[incomplete[0]]] += 1
        for n, t in t_f2.items():
            b.req_f2_ms[HAND_NAMES[n]].append((t - t_req) * 1e3)
        if this_f0 is not None and t_req - this_f0 <= cmd_req_max_s and len(t_f2) == len(hands):
            b.cmd_f2_ms.append((max(t_f2.values()) - this_f0) * 1e3)
    return b


# ── side files ───────────────────────────────────────────────────────────────


def count_needle(path: Path | None, needle: str) -> int | None:
    if path is None:
        return None
    with path.open(encoding="utf-8", errors="replace") as fh:
        return sum(1 for line in fh if needle in line)


def failure_reasons(path: Path | None) -> list[str] | None:
    """Reasons the node logged as `Hand failure detected: <reason> (...)`, in order."""
    if path is None:
        return None
    out: list[str] = []
    with path.open(encoding="utf-8", errors="replace") as fh:
        for line in fh:
            i = line.find(FAILURE_NEEDLE)
            if i < 0:
                continue
            rest = line[i + len(FAILURE_NEEDLE) :].strip()
            out.append(rest.split(" (", 1)[0].split()[0] if rest else "?")
    return out


# ── judgement ────────────────────────────────────────────────────────────────


def _cmd_ack(comm: dict) -> str:
    """``canfd.cmd_ack`` of the stats JSON; archives that predate it are echo mode."""
    return comm.get("canfd", {}).get("cmd_ack", "echo")


def resolve_req_mode(requested: str, comm: dict) -> str:
    """``--req-mode``, or with ``auto`` the stats JSON ``canfd.req_mode``;
    archives that predate the key are per_hand. Raises ValueError on a value
    the gate does not know or an explicit mode the JSON contradicts."""
    recorded = comm.get("canfd", {}).get("req_mode")
    if recorded is not None and recorded not in REQ_MODES:
        raise ValueError(f"stats JSON canfd.req_mode {recorded!r} is not one of {REQ_MODES}")
    if requested == "auto":
        return recorded or REQ_PER_HAND
    if recorded is not None and recorded != requested:
        raise ValueError(
            f"req_mode mismatch: --req-mode {requested} but the stats JSON says {recorded}"
        )
    return requested


def req_mode_mismatch(frames: list[Frame], req_mode: str) -> str | None:
    """Why the bus contradicts `req_mode`, or None (a REQ of the other mode is on it)."""
    per_hand_ids = {req_id(n) for n in HAND_NODES}
    bcast = broadcast_req_id()
    n_bcast = sum(1 for f in frames if not f.is_error and f.can_id == bcast)
    n_per_hand = sum(1 for f in frames if not f.is_error and f.can_id in per_hand_ids)
    if req_mode == REQ_PER_HAND and n_bcast:
        return (
            f"req_mode mismatch: judged as per_hand but the bus carries {n_bcast} broadcast "
            f"REQ(s) {bcast:08X} — pass --req-mode broadcast"
        )
    if req_mode == REQ_BROADCAST and n_per_hand:
        return (
            f"req_mode mismatch: judged as broadcast but the bus carries {n_per_hand} per-hand "
            f"REQ(s) — pass --req-mode per_hand"
        )
    return None


def _rate_check(key: str, count: int, denominator: int, th: Thresholds, what: str) -> Check:
    if denominator <= 0:
        return Check(
            key,
            NOT_EVALUATED,
            f"NOT_EVALUATED(no {what} in this run)",
            f"< {th.timeout_rate_max:.1%}",
        )
    rate = count / denominator
    p = th.timeout_rate_max
    limit_count = p * denominator
    band = th.undecided_sigmas * math.sqrt(denominator * p * (1.0 - p))
    value = f"{count} / {denominator} = {rate:.4%}"
    threshold = f"< {p:.1%} (= {limit_count:.1f} ± {band:.1f} of {denominator})"
    if count == 0 or count < limit_count - band:
        return Check(key, PASS, value, threshold)
    if count > limit_count + band:
        return Check(key, FAIL, value, threshold)
    return Check(
        key,
        UNDECIDED,
        value,
        threshold,
        f"within ±{th.undecided_sigmas:g} sigma of the threshold count — repeat the run once",
    )


def judge(
    timing: TimingSummary,
    comm: dict,
    bus: BusSummary,
    th: Thresholds,
    *,
    link_down_lines: int | None,
    link_status_false: int | None,
    failure_reasons: list[str] | None = None,
    overlap: OverlapSummary | None = None,
) -> list[Check]:
    checks: list[Check] = []
    none_mode = _cmd_ack(comm) == "none"

    # (a) duration
    span = timing.span_s
    checks.append(
        Check(
            "a.duration",
            PASS if span >= th.min_duration_s else FAIL,
            f"{span:.1f} s, {timing.rows} rows",
            f">= {th.min_duration_s:.0f} s",
        )
    )
    checks.append(
        Check(
            "a.single_run",
            PASS if len(timing.run_ids) == 1 else FAIL,
            f"run_id {timing.run_ids}",
            "exactly one run_id",
            "" if len(timing.run_ids) == 1 else "pass --run-id to select one restart",
        )
    )
    decimation = int(comm.get("comm_decimation", 1))
    if decimation == 1:
        checks.append(
            Check(
                "a.tick_gaps",
                PASS if timing.tick_gaps == 0 else FAIL,
                str(timing.tick_gaps),
                "0",
                "" if timing.tick_gaps == 0 else "rows missing: SPSC drop or suppressed ticks",
            )
        )
    else:
        checks.append(
            Check(
                "a.tick_gaps",
                NOT_EVALUATED,
                f"NOT_EVALUATED(comm_decimation={decimation}: skipped ticks leave gaps by design)",
                "0",
                f"{timing.tick_gaps} gap(s) seen",
            )
        )

    # (b) link never down — four independent witnesses must agree. The failure
    # detector's latch counts only when its reason is about the link: a sensor
    # heuristic (all-zero / duplicate payload) trips on every sensorless hand
    # and is reported, not failed. Attribution needs the node log.
    link_ok = bool(comm.get("link_ok", False))
    failed = bool(comm.get("failure_detected", False))
    verdict = PASS if link_ok else FAIL
    note = ""
    if failed:
        if failure_reasons is None:
            verdict = FAIL
            note = "detector latched; pass --node-log to attribute the reason"
        elif failure_reasons and all(r in SENSOR_HEURISTIC_REASONS for r in failure_reasons):
            note = f"detector latched on {sorted(set(failure_reasons))}: sensor heuristic, not the link"
        else:
            verdict = FAIL
            note = f"detector latched on {sorted(set(failure_reasons or ['?']))}"
    checks.append(
        Check(
            "b.json_link_ok",
            verdict,
            f"link_ok={link_ok} failure_detected={failed}",
            "link_ok; a detector latch only for a link reason",
            note,
        )
    )
    if link_down_lines is None:
        checks.append(Check("b.log_link_down", NOT_EVALUATED, "NOT_EVALUATED(no --node-log)", "0"))
    else:
        checks.append(
            Check(
                "b.log_link_down",
                PASS if link_down_lines == 0 else FAIL,
                f"{link_down_lines} line(s)",
                "0",
            )
        )
    checks.append(
        Check(
            "b.bus_silent_req",
            PASS if bus.max_silent_req_streak < th.silent_req_max else FAIL,
            f"max {bus.max_silent_req_streak} consecutive REQ without a complete RESP set",
            f"< {th.silent_req_max}",
        )
    )
    if link_status_false is None:
        checks.append(
            Check("b.topic_false", NOT_EVALUATED, "NOT_EVALUATED(no --link-status)", "0")
        )
    else:
        checks.append(
            Check(
                "b.topic_false",
                PASS if link_status_false == 0 else FAIL,
                f"{link_status_false} false sample(s)",
                "0",
            )
        )

    # (c) cycle p99, nothing excluded — but only against the period the run
    # actually ticked at.
    p99 = timing.p99_total_us
    period_note = ""
    period_ok = True
    if not math.isnan(timing.period_us_median):
        rel = abs(timing.period_us_median - th.period_us) / th.period_us
        if rel > th.period_tolerance:
            period_ok = False
            period_note = (
                f"median tick period {timing.period_us_median:.0f} us is not the "
                f"--loop-rate-hz period {th.period_us:.0f} us: re-run with the right rate"
            )
    checks.append(
        Check(
            "c.cycle_p99",
            PASS if period_ok and p99 < th.p99_limit_us else FAIL,
            f"{p99:.0f} us (max {timing.max_total_us:.0f}, >= limit {timing.over_limit}, "
            f">= period {timing.over_period})",
            f"< {th.p99_limit_us:.0f} us ({_p99_rule(th.p99_fraction)}, "
            f"period {th.period_us:.0f} us)",
            period_note,
        )
    )

    # (d) per-stage timeout rates
    total_cycles = int(comm.get("total_cycles", 0))
    checks.append(
        _rate_check(
            "d.resp_set_timeout",
            stage_timeouts(comm, "joint_read"),
            total_cycles,
            th,
            "comm cycles",
        )
    )
    if none_mode:
        checks.append(
            Check(
                "d.echo_timeout",
                NOT_EVALUATED,
                "NOT_EVALUATED(cmd_ack none: no echo stage)",
                f"< {th.timeout_rate_max:.1%}",
            )
        )
    else:
        checks.append(
            _rate_check(
                "d.echo_timeout",
                stage_timeouts(comm, "write_echo"),
                stage_attempts(comm, "write_echo"),
                th,
                "writes",
            )
        )

    # (e) E-Stop on the bus
    e = bus.estop
    estop_threshold = (
        f"1 pair, F1 -> F0 zeros, < {th.estop_window_ms:g} ms, no CMD after"
        if none_mode
        else f"1 pair, both echoes, < {th.estop_window_ms:g} ms"
    )
    if e is None:
        checks.append(
            Check("e.estop", FAIL, "no Frame 1 zeros -> Frame 0 zeros pair", estop_threshold)
        )
    else:
        notes = []
        ok = True
        if bus.estop_candidates != 1:
            ok = False
            notes.append(f"{bus.estop_candidates} candidate pairs (expected 1)")
        if e.cmd_frames_after != 0:
            ok = False
            notes.append(
                f"{e.cmd_frames_after} command frame(s) after the pair — loop did not stop"
            )
        if none_mode:
            delta_ms = (e.t_cmd0 - e.t_cmd1) * 1e3
            value = f"Δt(CMD F1 zeros -> CMD F0 zeros) = {delta_ms:.3f} ms, candump-to-candump"
            if not (e.t_cmd0 > e.t_cmd1 and delta_ms < th.estop_window_ms):
                ok = False
        else:
            value = f"Δt(first zero CMD -> last echo) = {e.delta_ms:.3f} ms, candump-to-candump"
            if e.t_echo0 is None or e.t_echo1 is None:
                ok = False
                notes.append("echo missing")
            elif not e.delta_ms < th.estop_window_ms:
                ok = False
        checks.append(
            Check("e.estop", PASS if ok else FAIL, value, estop_threshold, "; ".join(notes))
        )

    # (x) two hands on one bus
    if overlap is not None and overlap.two_hands:
        if overlap.broadcast is not None:
            checks.append(_broadcast_check(overlap.broadcast, th))
        else:
            checks.append(_overlap_check(overlap))
    return checks


def _overlap_check(o: OverlapSummary) -> Check:
    threshold = "0 frames of a hand inside the other hand's transaction"
    tx = ", ".join(f"{k} {v}" for k, v in sorted(o.transactions.items()))
    if len(o.transactions) < 2:
        return Check(
            "x.no_overlap",
            NOT_EVALUATED,
            f"NOT_EVALUATED(transactions before the E-Stop: {tx or 'none'})",
            threshold,
        )
    n = len(o.violations)
    value = f"{n} intrusion(s) over transactions {tx}"
    note = ""
    if n:
        pairs = ", ".join(f"{k} x{v}" for k, v in sorted(o.by_pair.items()))
        note = f"intruder -> victim: {pairs}; first at line {o.violations[0].line_no}"
    return Check("x.no_overlap", PASS if n == 0 else FAIL, value, threshold, note)


def _broadcast_check(b: BroadcastSummary, th: Thresholds) -> Check:
    """x.broadcast_sets: (1) + (2) must be 0; (3) is a rate judged like (d)."""
    key = "x.broadcast_sets"
    rate = _rate_check(key, b.one_hand_missing, b.windows, th, "broadcast windows")
    hard = b.duplicates + b.cmd_in_burst
    value = (
        f"duplicate RESP {b.duplicates}, CMD in burst {b.cmd_in_burst}; "
        f"one-hand-missing {rate.value}"
    )
    threshold = f"duplicate 0, CMD in burst 0; one-hand-missing {rate.threshold}"
    by_hand = ", ".join(f"{k} {v}" for k, v in sorted(b.one_hand_missing_by_hand.items()))
    notes = [f"one-hand-missing by hand: {by_hand}"] if b.one_hand_missing else []
    if rate.note:
        notes.append(rate.note)
    if hard:
        return Check(key, FAIL, value, threshold, "; ".join(notes))
    return Check(key, rate.verdict, value, threshold, "; ".join(notes))


def overall(checks: list[Check]) -> str:
    verdicts = {c.verdict for c in checks}
    if FAIL in verdicts:
        return FAIL
    if UNDECIDED in verdicts:
        return UNDECIDED
    return PASS


# ── report ───────────────────────────────────────────────────────────────────


def _dist(values: list[float]) -> str:
    if not values:
        return "n=0"
    return (
        f"n={len(values)} p50={quantile(values, 0.5):.3f} p99={quantile(values, 0.99):.3f} "
        f"max={max(values):.3f} ms"
    )


def render_markdown(
    checks: list[Check],
    timing: TimingSummary,
    comm: dict,
    bus: BusSummary,
    th: Thresholds,
    overlap: OverlapSummary | None = None,
    req_mode: str | None = None,
) -> str:
    not_evaluated = [c.key for c in checks if c.verdict == NOT_EVALUATED]
    tail = f" — not evaluated: {', '.join(not_evaluated)}" if not_evaluated else ""
    gate = "G-5" if overlap is not None and overlap.two_hands else "G-3"
    out = [f"## {gate} verdict: **{overall(checks)}**{tail}", ""]
    out += ["| check | verdict | value | threshold | note |", "|---|---|---|---|---|"]
    for c in checks:
        out.append(f"| `{c.key}` | {c.verdict} | {c.value} | {c.threshold} | {c.note} |")
    out += ["", "### Supporting figures", ""]
    canfd = comm.get("canfd", {})
    asm = canfd.get("assembler", {})
    total_cycles = int(comm.get("total_cycles", 0))
    rt = int(comm.get("recv_timeout", 0))
    re_ = int(comm.get("recv_error", 0))
    bound = (rt + re_) / total_cycles if total_cycles else math.nan
    none_mode = _cmd_ack(comm) == "none"
    d42 = "D42: 2 x T < period (nominal; no echo stage)" if none_mode else "D42: 2 x T < period"
    recv_expect = (
        "unused under broadcast (D89: window = slot - broadcast_tail_guard_ms)"
        if req_mode == REQ_BROADCAST
        else d42
    )
    echo_handling = (
        "shares its window with the CMD sends since #919"
        if none_mode
        else "includes the echo handling since #919"
    )
    out += [
        "| figure | value | expected |",
        "|---|---|---|",
        f"| protocol / transport | {comm.get('protocol_version')} / {comm.get('transport')} | 2 / canfd |",
        *(
            [f"| req_mode (--req-mode, else JSON canfd.req_mode) | {req_mode} | — |"]
            if req_mode is not None
            else []
        ),
        f"| recv_timeout_ms (as run) | {comm.get('recv_timeout_ms')} | {recv_expect} |",
        f"| total_cycles / timing rows | {total_cycles} / {timing.rows} | about equal |",
        f"| median tick period | {timing.period_us_median:.0f} us | {th.period_us:.0f} us |",
        f"| t_total_us p50 / p99 / max | {timing.p50_total_us:.0f} / {timing.p99_total_us:.0f} / "
        f"{timing.max_total_us:.0f} | — |",
        f"| jitter_us p99 / max | {timing.p99_jitter_us:.0f} / {timing.max_jitter_us:.0f} | — |",
        f"| aggregate (recv_timeout + recv_error) / total_cycles | ({rt} + {re_}) / {total_cycles} "
        f"= {bound:.4%} | upper bound only (up to 2 per cycle) |",
        f"| canfd.tx_eagain / tx_enobufs / tx_enetdown / tx_other | {canfd.get('tx_eagain')} / "
        f"{canfd.get('tx_enobufs')} / {canfd.get('tx_enetdown')} / {canfd.get('tx_other')} | 0 |",
        f"| canfd.bus_off / error_frames | {canfd.get('bus_off')} / {canfd.get('error_frames')} "
        f"| 0 (bus errors are outside error_mask) |",
        f"| canfd.echo_payload_mismatch | {canfd.get('echo_payload_mismatch')} | 0 |",
        f"| assembler sets_completed / sets_abandoned | {asm.get('sets_completed')} / "
        f"{asm.get('sets_abandoned')} | abandoned ~ timeouts |",
        f"| assembler bad_index / bad_length / duplicate / outside_set | {asm.get('bad_index')} / "
        f"{asm.get('bad_length')} / {asm.get('duplicate')} / {asm.get('outside_set')} | 0 |",
        f"| stale_drained / cmd_mismatch / mode_mismatch | {comm.get('stale_drained')} / "
        f"{comm.get('cmd_mismatch')} / {comm.get('mode_mismatch')} | 0 |",
        f"| feedforward_size_mismatch / nonfinite | {comm.get('feedforward_size_mismatch')} / "
        f"{comm.get('feedforward_nonfinite')} | 0 |",
        f"| bus frames (parsed / unparsed lines / error frames) | {bus.frames} / "
        f"{bus.unparsed_lines} / {bus.error_frames} | errors 0 |",
        f"| bus CMD / ECHO / REQ / RESP / other-node | {bus.cmd} / {bus.echo} / {bus.req} / "
        f"{bus.resp} / {bus.other} | {'ECHO 0 (cmd_ack none)' if none_mode else '—'} |",
        f"| REQ with DLC > 0 | {bus.nonzero_dlc_req} | 0 (D47: DLC 0) |",
        f"| RESP sets complete / incomplete | {bus.complete_sets} / {bus.incomplete_sets} | — |",
        f"| REQ -> RESP frame 2 round trip, all ticks (bus, lower bound; a write tick's "
        f"{echo_handling}) | {_dist(bus.rtt_req_f2_ms)} | — |",
        f"| REQ -> RESP frame 2 round trip, read-only ticks (bus, lower bound; the H1-F06 "
        f"baseline) | {_dist(bus.rtt_req_f2_readonly_ms)} | — |",
        f"| CMD frame 0 -> RESP frame 2, write ticks (bus, lower bound) | "
        f"{_dist(bus.rtt_cmd_f2_ms)} | — |",
        f"| write attempts (JSON once-per-attempt counters) / bus write pairs | "
        f"{stage_attempts(comm, 'write_echo')} / {bus.write_pairs} | about equal |",
    ]
    if not none_mode:
        out.insert(
            out.index(next(r for r in out if r.startswith("| CMD frame 0 -> RESP frame 2"))),
            f"| CMD frame 0 -> last echo round trip (bus, lower bound) | "
            f"{_dist(bus.rtt_cmd_echo_ms)} | — |",
        )
    if "echo_unexpected" in canfd:
        out.insert(
            out.index(next(r for r in out if r.startswith("| canfd.echo_payload_mismatch"))) + 1,
            f"| echo_unexpected | {canfd.get('echo_unexpected')} | 0 |",
        )
    if bus.estop is not None:
        e = bus.estop
        echoes = (
            ""
            if none_mode
            else f", echo F1 {_rel(e.t_echo1, e.t_cmd1)}, echo F0 {_rel(e.t_echo0, e.t_cmd1)}"
        )
        out.append(
            f"| E-Stop frames (t rel. to CMD F1) | CMD F0 +{(e.t_cmd0 - e.t_cmd1) * 1e3:.3f} ms"
            f"{echoes} | — |"
        )
    if overlap is not None and overlap.two_hands:
        gap = overlap.estop_gap_ms
        gap_text = "missing" if math.isnan(gap) else f"{gap:+.3f} ms"
        if overlap.broadcast is not None:
            out += _broadcast_rows(overlap.broadcast)
        else:
            tx = " / ".join(str(overlap.transactions.get(h, 0)) for h in ("right", "left"))
            pairs = ", ".join(f"{k} x{v}" for k, v in sorted(overlap.by_pair.items())) or "none"
            out += [
                f"| transactions right / left (before the E-Stop) | {tx} | about equal |",
                f"| overlap intrusions (intruder -> victim) | {pairs} | none |",
            ]
        out.append(
            f"| E-Stop zero-write gap, right CMD F1 -> left CMD F1 | {gap_text} "
            f"| reported only (D78) |"
        )
    return "\n".join(out) + "\n"


def _order_text(order: dict[str, int]) -> str:
    return ", ".join(f"{k} ×{v}" for k, v in sorted(order.items(), key=lambda kv: -kv[1]))


def _broadcast_rows(b: BroadcastSummary) -> list[str]:
    missing = " / ".join(str(b.missing.get(h, 0)) for h in ("right", "left"))
    one = " / ".join(str(b.one_hand_missing_by_hand.get(h, 0)) for h in ("right", "left"))
    late = " / ".join(str(b.late_resp.get(h, 0)) for h in ("right", "left"))
    rows = [
        f"| broadcast windows judged / cut by the E-Stop | {b.windows} / {b.truncated} | — |",
        f"| windows complete / one hand missing / every hand missing | {b.complete} / "
        f"{b.one_hand_missing} / {b.all_missing} | (x) judges the middle, (d) the last |",
        f"| windows with the set missing, right / left (one hand missing: {one}) | {missing} "
        f"| reported only |",
        f"| duplicate RESP / host CMD in the burst | {b.duplicates} / {b.cmd_in_burst} | 0 / 0 |",
        f"| late RESP after the next tick's write, right / left (set already timed out) "
        f"| {late} | reported only |",
        f"| RESP order (R right, L left frame, per window) | {_order_text(b.order) or 'none'} "
        f"| reported only |",
    ]
    for h in ("right", "left"):
        if h in b.req_f2_ms:
            rows.append(
                f"| REQ(0xFF) -> {h} RESP frame 2 (bus, lower bound; the later hand's figure "
                f"includes the other hand's burst) | {_dist(b.req_f2_ms[h])} | — |"
            )
    rows.append(
        f"| first CMD frame 0 -> last RESP frame 2, write ticks (bus, lower bound) | "
        f"{_dist(b.cmd_f2_ms)} | — |"
    )
    return rows


def _json_safe(obj):
    """NaN -> null (json.dumps would emit a bare NaN token, which is not JSON)."""
    if isinstance(obj, float):
        return None if math.isnan(obj) else obj
    if isinstance(obj, dict):
        return {k: _json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_json_safe(v) for v in obj]
    if isinstance(obj, Path):
        return str(obj)
    return obj


def _rel(t: float | None, t0: float) -> str:
    return "missing" if t is None else f"+{(t - t0) * 1e3:.3f} ms"


# ── CLI ──────────────────────────────────────────────────────────────────────


def _artifacts(session: Path, suffix: str, timing_suffix: str | None = None) -> tuple[Path, Path]:
    """Resolve the (timing CSV, comm-stats JSON) pair of one run.

    `timing_suffix` (default: `suffix`) names the CSV on its own — a two-hand
    run writes one unsuffixed CSV beside a suffixed JSON per hand (D72).

    The pre-F08 names (#905, PRE_F08_HAND_STEMS) are used only when BOTH exist
    and NEITHER current file does; every other mix resolves to the current
    names, so a session holding one run's CSV next to another run's JSON is an
    exit-3 "missing file" (naming what the node writes today), never a verdict
    assembled from two runs."""
    tsuffix = suffix if timing_suffix is None else timing_suffix
    current = (
        session / "timing" / f"{TIMING_STEM}{tsuffix}.csv",
        session / "device" / f"{STATS_STEM}{suffix}.json",
    )
    legacy = (
        session / "timing" / f"{PRE_F08_HAND_STEMS[TIMING_STEM]}{tsuffix}.csv",
        session / "device" / f"{PRE_F08_HAND_STEMS[STATS_STEM]}{suffix}.json",
    )
    if all(p.exists() for p in legacy) and not any(p.exists() for p in current):
        return legacy
    return current


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("session", type=Path, help="session directory (device/ + timing/)")
    parser.add_argument("candump", type=Path, help="candump -L log of the run")
    parser.add_argument("--hand-side", choices=("right", "left"), default="right")
    parser.add_argument(
        "--req-mode",
        choices=("auto", *REQ_MODES),
        default="auto",
        help="the node's req_mode; auto = stats JSON canfd.req_mode, per_hand when absent",
    )
    parser.add_argument(
        "--suffix",
        default=None,
        help="artifact suffix, default '' (or '_<side>' with --explicit-side)",
    )
    parser.add_argument(
        "--explicit-side",
        action="store_true",
        help="hand_side was set explicitly on the node: artifacts carry _<side> (D51)",
    )
    parser.add_argument(
        "--timing-suffix",
        default=None,
        help="timing CSV suffix when it differs from the JSON's: '' for a hand_sides run (D72)",
    )
    parser.add_argument("--node-log", type=Path, default=None, help="node console log")
    parser.add_argument(
        "--link-status", type=Path, default=None, help="`ros2 topic echo .../link_status` capture"
    )
    parser.add_argument("--run-id", type=int, default=None, help="timing CSV run_id to judge")
    parser.add_argument("--loop-rate-hz", type=float, default=Thresholds.loop_rate_hz)
    parser.add_argument("--min-duration-s", type=float, default=Thresholds.min_duration_s)
    parser.add_argument("--p99-fraction", type=float, default=Thresholds.p99_fraction)
    parser.add_argument("--timeout-rate-max", type=float, default=Thresholds.timeout_rate_max)
    parser.add_argument("--undecided-sigmas", type=float, default=Thresholds.undecided_sigmas)
    parser.add_argument("--period-tolerance", type=float, default=Thresholds.period_tolerance)
    parser.add_argument("--estop-window-ms", type=float, default=Thresholds.estop_window_ms)
    parser.add_argument(
        "--link-fail-timeout-ms",
        type=float,
        default=100.0,
        help="the node's link_fail_timeout_ms; with the rate it sets --silent-req-max",
    )
    parser.add_argument(
        "--silent-req-max",
        type=int,
        default=None,
        help="silent REQ streak that fails (b); default link_fail_timeout_ms x rate / 1000",
    )
    parser.add_argument(
        "--json-out", type=Path, default=None, help="write checks + figures as JSON"
    )
    args = parser.parse_args(argv)
    if not args.p99_fraction > 0:
        parser.error(f"--p99-fraction must be > 0, got {args.p99_fraction:g}")

    th = Thresholds(
        min_duration_s=args.min_duration_s,
        loop_rate_hz=args.loop_rate_hz,
        p99_fraction=args.p99_fraction,
        timeout_rate_max=args.timeout_rate_max,
        undecided_sigmas=args.undecided_sigmas,
        period_tolerance=args.period_tolerance,
        estop_window_ms=args.estop_window_ms,
    )
    suffix = args.suffix
    if suffix is None:
        suffix = f"_{args.hand_side}" if args.explicit_side else ""
    csv_path, json_path = _artifacts(args.session, suffix, args.timing_suffix)

    try:
        rows = load_timing_csv(csv_path)
        comm = load_comm_stats(json_path)
        frames, unparsed = read_candump(args.candump)
        link_down = count_needle(args.node_log, LINK_DOWN_NEEDLE)
        reasons = failure_reasons(args.node_log)
        link_false = count_needle(args.link_status, "data: false")
        req_mode = resolve_req_mode(args.req_mode, comm)
    except (OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 3
    mismatch = req_mode_mismatch(frames, req_mode)
    if mismatch is not None:
        print(f"error: {mismatch}", file=sys.stderr)
        return 3

    if args.run_id is not None:
        rows = [r for r in rows if r["run_id"] == args.run_id]
        if not rows:
            print(f"error: no rows with run_id {args.run_id} in {csv_path}", file=sys.stderr)
            return 3
    th.silent_req_max = (
        args.silent_req_max
        if args.silent_req_max is not None
        else silent_req_budget(
            args.link_fail_timeout_ms, th.loop_rate_hz, int(comm.get("comm_decimation", 1))
        )
    )
    timing = summarise_timing(rows, th.p99_limit_us, th.period_us)
    cmd_req_max_s = th.period_us * 0.5e-6
    overlap = analyse_overlap(frames, req_mode, cmd_req_max_s)
    bus = analyse_bus(
        frames,
        node_id_of(args.hand_side),
        unparsed,
        cmd_req_max_s=cmd_req_max_s,
        broadcast_req=req_mode == REQ_BROADCAST,
    )
    checks = judge(
        timing,
        comm,
        bus,
        th,
        link_down_lines=link_down,
        link_status_false=link_false,
        failure_reasons=reasons,
        overlap=overlap,
    )

    print(f"session : {args.session}")
    print(f"timing  : {csv_path}")
    print(f"stats   : {json_path}")
    print(f"candump : {args.candump}")
    print()
    print(render_markdown(checks, timing, comm, bus, th, overlap, req_mode))

    if args.json_out is not None:
        payload = {
            "verdict": overall(checks),
            "req_mode": req_mode,
            "checks": [asdict(c) for c in checks],
            "thresholds": asdict(th),
            "timing": asdict(timing),
            "bus": {
                **{
                    k: v
                    for k, v in asdict(bus).items()
                    if k
                    not in (
                        "rtt_req_f2_ms",
                        "rtt_cmd_echo_ms",
                        "rtt_cmd_f2_ms",
                        "rtt_req_f2_readonly_ms",
                    )
                },
                "rtt_req_f2_ms": _dist(bus.rtt_req_f2_ms),
                "rtt_req_f2_readonly_ms": _dist(bus.rtt_req_f2_readonly_ms),
                "rtt_cmd_echo_ms": _dist(bus.rtt_cmd_echo_ms),
                "rtt_cmd_f2_ms": _dist(bus.rtt_cmd_f2_ms),
            },
        }
        if overlap.two_hands:
            payload["overlap"] = {
                "hands_on_bus": overlap.hands_on_bus,
                "transactions": overlap.transactions,
                "violations": len(overlap.violations),
                "by_pair": overlap.by_pair,
                "first_violations": [asdict(v) for v in overlap.violations[:20]],
                "estop_line": overlap.estop_line,
                "estop_gap_ms": overlap.estop_gap_ms,
            }
            b = overlap.broadcast
            if b is not None:
                payload["overlap"]["broadcast"] = {
                    **{k: v for k, v in asdict(b).items() if k not in ("req_f2_ms", "cmd_f2_ms")},
                    "req_f2_ms": {h: _dist(v) for h, v in b.req_f2_ms.items()},
                    "cmd_f2_ms": _dist(b.cmd_f2_ms),
                }
        args.json_out.write_text(json.dumps(_json_safe(payload), indent=2) + "\n")

    verdict = overall(checks)
    return {PASS: 0, UNDECIDED: 2, FAIL: 1}[verdict]


if __name__ == "__main__":
    sys.exit(main())
