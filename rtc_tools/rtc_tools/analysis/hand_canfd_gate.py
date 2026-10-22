"""G-3 gate judge for the CAN FD hand link (epic #895 D55, H1-F06 #904).

One standalone run of ``udp_hand.launch.py`` (protocol_version 2) against the
real hand must satisfy, all at once:

  (a) duration  — the timing CSV spans >= 300 s, one run_id, no tick gap
  (b) link      — never down: stats JSON link_ok and not failure_detected, zero
                  "Hand link DOWN" lines in the node log, no silent REQ streak on
                  the bus that reaches the link-fail budget, no `false` on
                  /…/link_status
  (c) cycle p99 — t_total_us p99 < 0.8 x period (1600 us at 500 Hz), nothing
                  excluded
  (d) timeouts  — RESP-set timeouts / comm cycles < 0.1 % AND echo timeouts /
                  write attempts < 0.1 %, each on its own denominator
  (e) E-Stop    — on the bus: Frame 1 zeros -> Frame 0 zeros (the D26 order,
                  which is the REVERSE of a normal write) -> both echoes, within
                  2 ms of the first zero frame, and no command after it

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
  * (d) is a proportion estimated from one run. Near the threshold the run
    cannot decide it: a count within ``undecided_sigmas`` binomial standard
    deviations of the threshold count (±20 counts at 150 000 cycles) is
    UNDECIDED (repeat once) instead of PASS or FAIL by luck. The band scales
    with the denominator, so a small write count cannot hide a gross failure.
  * The per-stage ``ok`` / ``timeout`` / ``error`` counters fire once per
    attempt; ``cmd_mismatch`` fires once per unexpected FRAME (up to the echo
    budget per attempt) and an attempt that exhausts that budget is counted
    nowhere. The attempt denominator therefore sums only the once-per-attempt
    counters — it can only undercount attempts, i.e. overstate the rate — and
    the bus write-pair count is reported beside it as the cross-check.
  * (c) is judged against ``--loop-rate-hz``; the stats JSON carries no rate,
    so the CSV's own median tick period must agree with it or (c) FAILS with
    the reason instead of judging against the wrong threshold.
  * candump stamps the loopback copy of a transmitted frame at TX-done, so
    every bus-side round trip here is a lower bound on what the node saw.

Inputs are the forensic artifacts the node writes under the session directory
(``device/hand_udp_stats[_<side>].json``, ``timing/hand_udp_timing_log[_<side>].csv``,
D51 suffix when ``hand_side`` was explicit), a ``candump -L`` log, and
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
    Frame,
    Kind,
    classify,
    node_id_of,
    read_candump,
)
from rtc_tools.analysis.hand_close import quantile

PASS = "PASS"
FAIL = "FAIL"
UNDECIDED = "UNDECIDED"
NOT_EVALUATED = "NOT_EVALUATED"

TIMING_STEM = "hand_udp_timing_log"
STATS_STEM = "hand_udp_stats"
LINK_DOWN_NEEDLE = "Hand link DOWN"
FAILURE_NEEDLE = "Hand failure detected: "
# Failure-detector reasons that judge the SENSOR payload, not the link: a hand
# whose sensors read zero (none fitted, or motors off) latches these within
# the first 100 ms of every run, which says nothing about link-down 0 (D55).
SENSOR_HEURISTIC_REASONS = ("hand_sensor_all_zero", "hand_sensor_duplicate")
# The per-request-kind outcome counters that fire ONCE per attempt (fixture
# udp_hand_driver/test/fixtures/hand_udp_stats_keys_proto2.txt lists the full
# set; cmd_mismatch / mode_mismatch are per frame and excluded on purpose).
ATTEMPT_COUNTERS = ("ok", "timeout", "error", "short_or_decode")


@dataclass
class Thresholds:
    min_duration_s: float = 300.0
    loop_rate_hz: float = 500.0
    p99_fraction: float = 0.8
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
    # tripped the node's own link_ok, which is why the two must agree).
    silent_req_max: int = 50

    @property
    def period_us(self) -> float:
        return 1e6 / self.loop_rate_hz

    @property
    def p99_limit_us(self) -> float:
        return self.p99_fraction * self.period_us


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
    estop: EstopEvent | None = None
    estop_candidates: int = 0


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


def analyse_bus(frames: list[Frame], node: int, unparsed_lines: int = 0) -> BusSummary:
    """Per-hand counts, REQ -> RESP completeness, round trips and the E-Stop event."""
    s = BusSummary(frames=len(frames), unparsed_lines=unparsed_lines)
    kinds = [classify(f, node) for f in frames]
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
    # before the next REQ. The RTT is REQ -> frame 2, the last of the set.
    open_req: Frame | None = None
    seen: set[int] = set()
    t_f2: float | None = None
    silent = 0

    def close_set() -> None:
        nonlocal silent
        if open_req is None:
            return
        if seen >= {0, 1, 2}:
            s.complete_sets += 1
            silent = 0
            if t_f2 is not None:
                s.rtt_req_f2_ms.append((t_f2 - open_req.t) * 1e3)
        else:
            s.incomplete_sets += 1
            silent += 1
            s.max_silent_req_streak = max(s.max_silent_req_streak, silent)

    for f, k in zip(frames, kinds, strict=True):
        if k is Kind.REQ:
            close_set()
            open_req, seen, t_f2 = f, set(), None
        elif k is Kind.RESP and open_req is not None:
            seen.add(f.frame_idx)
            if f.frame_idx == 2:
                t_f2 = f.t
    close_set()

    # CMD pairs and their echoes. Normal write: frame 0 then frame 1 (both
    # echoed, D41). E-Stop: frame 1 then frame 0, both zero (D26 · D33). Two
    # consecutive normal writes also put a frame 1 next to a frame 0 — but a
    # REQ always lies between them (every comm cycle reads after it writes),
    # whereas the E-Stop pair is sent back to back, so "no REQ in between" is
    # what separates the two. Echoes are searched up to the next command frame
    # after the pair, never into the following write.
    cmd_frames = [
        (i, f) for i, (f, k) in enumerate(zip(frames, kinds, strict=True)) if k is Kind.CMD
    ]
    req_indices = [i for i, k in enumerate(kinds) if k is Kind.REQ]

    def req_between(lo: int, hi: int) -> bool:
        # req_indices is ascending (frame order): two bisections, not a scan —
        # a 5 min run that writes every cycle has ~300k pairs x ~150k REQs.
        return bisect.bisect_right(req_indices, lo) < bisect.bisect_left(req_indices, hi)

    for n, ((i_a, a), (i_b, b)) in enumerate(zip(cmd_frames, cmd_frames[1:], strict=False)):
        window_end = cmd_frames[n + 2][0] if n + 2 < len(cmd_frames) else len(frames)
        if a.frame_idx == 0 and b.frame_idx == 1 and not req_between(i_a, i_b):
            s.write_pairs += 1
            t0, t1 = _echoes_between(frames, kinds, i_a, window_end)
            if t0 is not None and t1 is not None:
                s.rtt_cmd_echo_ms.append((max(t0, t1) - a.t) * 1e3)
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
) -> list[Check]:
    checks: list[Check] = []

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
            f"< {th.p99_limit_us:.0f} us ({th.p99_fraction:g} x {th.period_us:.0f} us)",
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
    if e is None:
        checks.append(
            Check(
                "e.estop",
                FAIL,
                "no Frame 1 zeros -> Frame 0 zeros pair",
                f"1 pair, both echoes, < {th.estop_window_ms:g} ms",
            )
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
        if e.t_echo0 is None or e.t_echo1 is None:
            ok = False
            notes.append("echo missing")
        elif not e.delta_ms < th.estop_window_ms:
            ok = False
        checks.append(
            Check(
                "e.estop",
                PASS if ok else FAIL,
                f"Δt(first zero CMD -> last echo) = {e.delta_ms:.3f} ms, candump-to-candump",
                f"1 pair, both echoes, < {th.estop_window_ms:g} ms",
                "; ".join(notes),
            )
        )
    return checks


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
    checks: list[Check], timing: TimingSummary, comm: dict, bus: BusSummary, th: Thresholds
) -> str:
    not_evaluated = [c.key for c in checks if c.verdict == NOT_EVALUATED]
    tail = f" — not evaluated: {', '.join(not_evaluated)}" if not_evaluated else ""
    out = [f"## G-3 verdict: **{overall(checks)}**{tail}", ""]
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
    out += [
        "| figure | value | expected |",
        "|---|---|---|",
        f"| protocol / transport | {comm.get('protocol_version')} / {comm.get('transport')} | 2 / canfd |",
        f"| recv_timeout_ms (as run) | {comm.get('recv_timeout_ms')} | D42: 2 x T < period |",
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
        f"| assembler sets_completed / sets_abandoned / restarted | {asm.get('sets_completed')} / "
        f"{asm.get('sets_abandoned')} / {asm.get('restarted')} | abandoned ~ timeouts |",
        f"| assembler bad_index / bad_length / duplicate / outside_set | {asm.get('bad_index')} / "
        f"{asm.get('bad_length')} / {asm.get('duplicate')} / {asm.get('outside_set')} | 0 |",
        f"| stale_drained / cmd_mismatch / mode_mismatch | {comm.get('stale_drained')} / "
        f"{comm.get('cmd_mismatch')} / {comm.get('mode_mismatch')} | 0 |",
        f"| feedforward_size_mismatch / nonfinite | {comm.get('feedforward_size_mismatch')} / "
        f"{comm.get('feedforward_nonfinite')} | 0 |",
        f"| bus frames (parsed / unparsed lines / error frames) | {bus.frames} / "
        f"{bus.unparsed_lines} / {bus.error_frames} | errors 0 |",
        f"| bus CMD / ECHO / REQ / RESP / other-node | {bus.cmd} / {bus.echo} / {bus.req} / "
        f"{bus.resp} / {bus.other} | — |",
        f"| REQ with DLC > 0 | {bus.nonzero_dlc_req} | 0 (D47: DLC 0) |",
        f"| RESP sets complete / incomplete | {bus.complete_sets} / {bus.incomplete_sets} | — |",
        f"| REQ -> RESP frame 2 round trip (bus, lower bound) | {_dist(bus.rtt_req_f2_ms)} | — |",
        f"| CMD frame 0 -> last echo round trip (bus, lower bound) | {_dist(bus.rtt_cmd_echo_ms)} "
        f"| — |",
        f"| write attempts (JSON once-per-attempt counters) / bus write pairs | "
        f"{stage_attempts(comm, 'write_echo')} / {bus.write_pairs} | about equal |",
    ]
    if bus.estop is not None:
        e = bus.estop
        out.append(
            f"| E-Stop frames (t rel. to CMD F1) | CMD F0 +{(e.t_cmd0 - e.t_cmd1) * 1e3:.3f} ms, "
            f"echo F1 {_rel(e.t_echo1, e.t_cmd1)}, echo F0 {_rel(e.t_echo0, e.t_cmd1)} | — |"
        )
    return "\n".join(out) + "\n"


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


def _artifact(session: Path, sub: str, stem: str, suffix: str, ext: str) -> Path:
    return session / sub / f"{stem}{suffix}{ext}"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("session", type=Path, help="session directory (device/ + timing/)")
    parser.add_argument("candump", type=Path, help="candump -L log of the run")
    parser.add_argument("--hand-side", choices=("right", "left"), default="right")
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
    parser.add_argument("--silent-req-max", type=int, default=Thresholds.silent_req_max)
    parser.add_argument(
        "--json-out", type=Path, default=None, help="write checks + figures as JSON"
    )
    args = parser.parse_args(argv)

    th = Thresholds(
        min_duration_s=args.min_duration_s,
        loop_rate_hz=args.loop_rate_hz,
        p99_fraction=args.p99_fraction,
        timeout_rate_max=args.timeout_rate_max,
        undecided_sigmas=args.undecided_sigmas,
        period_tolerance=args.period_tolerance,
        estop_window_ms=args.estop_window_ms,
        silent_req_max=args.silent_req_max,
    )
    suffix = args.suffix
    if suffix is None:
        suffix = f"_{args.hand_side}" if args.explicit_side else ""
    csv_path = _artifact(args.session, "timing", TIMING_STEM, suffix, ".csv")
    json_path = _artifact(args.session, "device", STATS_STEM, suffix, ".json")

    try:
        rows = load_timing_csv(csv_path)
        comm = load_comm_stats(json_path)
        frames, unparsed = read_candump(args.candump)
        link_down = count_needle(args.node_log, LINK_DOWN_NEEDLE)
        reasons = failure_reasons(args.node_log)
        link_false = count_needle(args.link_status, "data: false")
    except (OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 3

    if args.run_id is not None:
        rows = [r for r in rows if r["run_id"] == args.run_id]
        if not rows:
            print(f"error: no rows with run_id {args.run_id} in {csv_path}", file=sys.stderr)
            return 3
    timing = summarise_timing(rows, th.p99_limit_us, th.period_us)
    bus = analyse_bus(frames, node_id_of(args.hand_side), unparsed)
    checks = judge(
        timing,
        comm,
        bus,
        th,
        link_down_lines=link_down,
        link_status_false=link_false,
        failure_reasons=reasons,
    )

    print(f"session : {args.session}")
    print(f"timing  : {csv_path}")
    print(f"stats   : {json_path}")
    print(f"candump : {args.candump}")
    print()
    print(render_markdown(checks, timing, comm, bus, th))

    if args.json_out is not None:
        payload = {
            "verdict": overall(checks),
            "checks": [asdict(c) for c in checks],
            "thresholds": asdict(th),
            "timing": asdict(timing),
            "bus": {
                **{
                    k: v
                    for k, v in asdict(bus).items()
                    if k not in ("rtt_req_f2_ms", "rtt_cmd_echo_ms")
                },
                "rtt_req_f2_ms": _dist(bus.rtt_req_f2_ms),
                "rtt_cmd_echo_ms": _dist(bus.rtt_cmd_echo_ms),
            },
        }
        args.json_out.write_text(json.dumps(_json_safe(payload), indent=2) + "\n")

    verdict = overall(checks)
    return {PASS: 0, UNDECIDED: 2, FAIL: 1}[verdict]


if __name__ == "__main__":
    sys.exit(main())
