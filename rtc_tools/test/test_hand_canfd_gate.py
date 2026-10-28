"""G-3 gate judge (epic #895 D55, H1-F06).

Every threshold is pinned with a boundary PAIR — the last value that passes
and the first that fails — on synthetic inputs whose verdict is known by
construction, so a green here is evidence about the judge, not the fixture.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from rtc_tools.analysis.candump_log import (
    broadcast_req_id,
    cmd_id,
    echo_id,
    node_id_of,
    read_candump,
    req_id,
    resp_id,
)
from rtc_tools.analysis.hand_canfd_gate import (
    FAIL,
    NOT_EVALUATED,
    PASS,
    STATS_STEM,
    TIMING_STEM,
    UNDECIDED,
    Thresholds,
    _artifacts,
    _broadcast_check,
    _overlap_check,
    analyse_bus,
    analyse_overlap,
    judge,
    load_comm_stats,
    load_timing_csv,
    main,
    overall,
    render_markdown,
    req_mode_mismatch,
    resolve_req_mode,
    silent_req_budget,
    summarise_timing,
)
from rtc_tools.utils.session_dir import PRE_F08_HAND_STEMS

NODE = node_id_of("right")
TH = Thresholds()
PERIOD_US = 2000.0
LIMIT_US = PERIOD_US / 1.1  # 1818.18 us


# ── fixture builders ─────────────────────────────────────────────────────────


def timing_rows(
    n: int, total_us: list[int] | None = None, run_id: int = 0, period_ns: int = 2_000_000
):
    total_us = total_us if total_us is not None else [500] * n
    return [
        {
            "t_wall_ns": 1_000_000_000 + i * period_ns,
            "tick_count": i,
            "run_id": run_id,
            "t_total_us": total_us[i],
            "jitter_us": 3,
        }
        for i in range(n)
    ]


def comm_stats(
    total_cycles: int = 150_000,
    joint_timeouts: int = 0,
    write_ok: int = 0,
    write_timeouts: int = 0,
    link_ok: bool = True,
    failure_detected: bool = False,
    decimation: int = 1,
    cmd_ack: str | None = None,
) -> dict:
    def stage(ok: int, timeout: int) -> dict:
        return {
            "ok": ok,
            "timeout": timeout,
            "error": 0,
            "short_or_decode": 0,
            "cmd_mismatch": 0,
            "mode_mismatch": 0,
        }

    return {
        "protocol_version": "2",
        "transport": "canfd",
        "recv_timeout_ms": 0.9,
        "total_cycles": total_cycles,
        "comm_decimation": decimation,
        "recv_timeout": joint_timeouts + write_timeouts,
        "recv_error": 0,
        "link_ok": link_ok,
        "failure_detected": failure_detected,
        "per_request": {
            "joint_read": stage(total_cycles - joint_timeouts, joint_timeouts),
            "write_echo": stage(write_ok, write_timeouts),
        },
        "canfd": {
            "tx_eagain": 0,
            "tx_enobufs": 0,
            "tx_enetdown": 0,
            "tx_other": 0,
            "bus_off": 0,
            "error_frames": 0,
            "echo_payload_mismatch": 0,
            "assembler": {
                "sets_completed": total_cycles - joint_timeouts,
                "sets_abandoned": joint_timeouts,
                "bad_index": 0,
                "bad_length": 0,
                "duplicate": 0,
                "outside_set": 0,
            },
        },
    }


def none_comm_stats(**kw) -> dict:
    """A cmd_ack none run: the key, no unexpected echoes, no echo timeouts."""
    comm = comm_stats(write_ok=kw.pop("write_ok", 1000), **kw)
    comm["canfd"]["cmd_ack"] = "none"
    comm["canfd"]["echo_unexpected"] = 0
    return comm


def line(t: float, can_id: int, data_hex: str = "") -> str:
    return f"({t:.6f}) can0 {can_id:08X}##1{data_hex}"


ZERO44 = "00" * 44
NONZERO44 = "01" * 44


def req_resp(t: float, answered: bool = True) -> list[str]:
    out = [line(t, req_id(NODE))]
    if answered:
        out += [line(t + 0.0003 + 0.0001 * i, resp_id(NODE, i), "11" * 8) for i in range(3)]
    return out


def normal_write(t: float, data_hex: str = NONZERO44) -> list[str]:
    """Frame 0 then frame 1, both echoed — a WriteCommandWithEcho."""
    return [
        line(t, cmd_id(NODE, 0), data_hex),
        line(t + 0.0001, cmd_id(NODE, 1), data_hex),
        line(t + 0.0003, echo_id(NODE, 0), data_hex),
        line(t + 0.0004, echo_id(NODE, 1), data_hex),
    ]


def pipelined_write(
    t: float, data_hex: str = NONZERO44, resp_first: bool = False, complete: bool = True
) -> list[str]:
    """One #919 write tick: CMD 0 · CMD 1 · REQ back to back, then the echoes and
    the RESP burst sharing the window (RESP 0 ahead of the echoes when
    `resp_first`); frame 2 withheld when not `complete`."""
    out = [
        line(t, cmd_id(NODE, 0), data_hex),
        line(t + 0.00004, cmd_id(NODE, 1), data_hex),
        line(t + 0.00008, req_id(NODE)),
    ]
    echoes = [
        line(t + 0.0005, echo_id(NODE, 0), data_hex),
        line(t + 0.00077, echo_id(NODE, 1), data_hex),
    ]
    resp = [line(t + 0.0004 + 0.00013 * i, resp_id(NODE, i), "11" * 8) for i in range(3)]
    if not complete:
        resp = resp[:2]
    if resp_first:
        out += resp[:1] + echoes + resp[1:]
    else:
        out += echoes + resp
    return out


def estop(t: float, echo_delay_s: float = 0.0015, with_echoes: bool = True) -> list[str]:
    """Frame 1 zeros then frame 0 zeros (D26), both echoed `echo_delay_s` after the first."""
    out = [line(t, cmd_id(NODE, 1), ZERO44), line(t + 0.0001, cmd_id(NODE, 0), ZERO44)]
    if with_echoes:
        out += [
            line(t + echo_delay_s - 0.0001, echo_id(NODE, 1), ZERO44),
            line(t + echo_delay_s, echo_id(NODE, 0), ZERO44),
        ]
    return out


def bus_from(lines: list[str], tmp_path: Path):
    log = tmp_path / "g3.log"
    log.write_text("\n".join(lines) + "\n")
    frames, unparsed = read_candump(log)
    return analyse_bus(frames, NODE, unparsed)


def good_bus_lines(n_req: int = 20) -> list[str]:
    lines: list[str] = []
    for i in range(n_req):
        lines += req_resp(1.0 + i * 0.002)
    lines += estop(1.0 + n_req * 0.002)
    return lines


def by_key(checks):
    return {c.key: c for c in checks}


def run_judge(tmp_path, rows=None, comm=None, lines=None, th=TH, **kw):
    rows = rows if rows is not None else timing_rows(150_001)
    comm = comm if comm is not None else comm_stats()
    lines = lines if lines is not None else good_bus_lines()
    timing = summarise_timing(rows, th.p99_limit_us, th.period_us)
    bus = bus_from(lines, tmp_path)
    kw.setdefault("link_down_lines", 0)
    kw.setdefault("link_status_false", 0)
    return by_key(judge(timing, comm, bus, th, **kw))


# ── (a) duration ─────────────────────────────────────────────────────────────


def test_duration_boundary(tmp_path):
    # 150_001 rows at 2 ms span exactly 300.000 s; one row fewer is 299.998 s.
    assert run_judge(tmp_path, rows=timing_rows(150_001))["a.duration"].verdict == PASS
    assert run_judge(tmp_path, rows=timing_rows(150_000))["a.duration"].verdict == FAIL


def test_multiple_run_ids_fail_single_run(tmp_path):
    rows = timing_rows(100_000, run_id=0) + timing_rows(60_000, run_id=1)
    assert run_judge(tmp_path, rows=rows)["a.single_run"].verdict == FAIL
    assert run_judge(tmp_path)["a.single_run"].verdict == PASS


def test_tick_gap_fails_unless_decimated(tmp_path):
    rows = timing_rows(150_001)
    del rows[1000]
    assert run_judge(tmp_path, rows=rows)["a.tick_gaps"].verdict == FAIL
    gaps = run_judge(tmp_path, rows=rows, comm=comm_stats(decimation=2))["a.tick_gaps"]
    assert gaps.verdict == NOT_EVALUATED
    assert gaps.value.startswith("NOT_EVALUATED(")
    assert run_judge(tmp_path)["a.tick_gaps"].verdict == PASS


# ── (b) link ─────────────────────────────────────────────────────────────────


def test_json_link_flags(tmp_path):
    assert run_judge(tmp_path, comm=comm_stats(link_ok=False))["b.json_link_ok"].verdict == FAIL
    assert run_judge(tmp_path)["b.json_link_ok"].verdict == PASS
    latched = comm_stats(failure_detected=True)
    # No node log: the latch cannot be attributed, so it counts against the link.
    assert run_judge(tmp_path, comm=latched)["b.json_link_ok"].verdict == FAIL
    # A sensor heuristic (every sensorless hand latches it) is reported, not failed.
    c = run_judge(tmp_path, comm=latched, failure_reasons=["hand_sensor_all_zero"])[
        "b.json_link_ok"
    ]
    assert c.verdict == PASS
    assert "sensor heuristic" in c.note
    # Anything else the detector latched on is a link failure.
    c = run_judge(tmp_path, comm=latched, failure_reasons=["hand_link_down"])["b.json_link_ok"]
    assert c.verdict == FAIL
    c = run_judge(
        tmp_path, comm=latched, failure_reasons=["hand_sensor_all_zero", "hand_rate_low"]
    )["b.json_link_ok"]
    assert c.verdict == FAIL


def test_failure_reasons_are_parsed_from_the_node_log(tmp_path):
    from rtc_tools.analysis.hand_canfd_gate import failure_reasons

    log = tmp_path / "node.log"
    log.write_text(
        "[udp_hand_node-1] [INFO] [1.0] [hand.node]: UdpHandNode activated\n"
        "[udp_hand_node-1] [ERROR] [1.1] [hand.node]: Hand failure detected: "
        "hand_sensor_all_zero (count=5)\n"
        "[udp_hand_node-1] [WARN] [1.2] [hand.fail]: Sensor duplicate detected (count=6)\n"
    )
    assert failure_reasons(log) == ["hand_sensor_all_zero"]
    assert failure_reasons(None) is None


def test_log_and_topic_witnesses(tmp_path):
    assert run_judge(tmp_path, link_down_lines=1)["b.log_link_down"].verdict == FAIL
    assert run_judge(tmp_path, link_status_false=1)["b.topic_false"].verdict == FAIL
    c = run_judge(tmp_path, link_down_lines=None, link_status_false=None)
    assert c["b.log_link_down"].verdict == NOT_EVALUATED
    assert c["b.topic_false"].verdict == NOT_EVALUATED


def test_silent_req_streak_boundary(tmp_path):
    def lines_with_streak(k: int) -> list[str]:
        out = req_resp(1.0)
        for i in range(k):
            out += req_resp(1.1 + i * 0.002, answered=False)
        out += req_resp(2.0)
        out += estop(2.1)
        return out

    assert run_judge(tmp_path, lines=lines_with_streak(49))["b.bus_silent_req"].verdict == PASS
    assert run_judge(tmp_path, lines=lines_with_streak(50))["b.bus_silent_req"].verdict == FAIL


def test_streak_is_longest_consecutive_not_total(tmp_path):
    out: list[str] = []
    t = 1.0
    for _ in range(3):
        for _i in range(30):
            out += req_resp(t, answered=False)
            t += 0.002
        out += req_resp(t)
        t += 0.002
    out += estop(t)
    bus = bus_from(out, tmp_path)
    assert bus.incomplete_sets == 90
    assert bus.max_silent_req_streak == 30


# ── (c) cycle p99 ────────────────────────────────────────────────────────────


def test_p99_boundary_is_strict(tmp_path):
    n = 150_001
    # Nearest-rank p99 of n samples is the ceil(0.99 n)-th order statistic; put
    # exactly the top 1 % at the probe value so that statistic equals it.
    top = n - (int(0.99 * n + 0.999999) - 1)
    for probe, verdict in ((1818, PASS), (1819, FAIL)):
        total = [500] * (n - top) + [probe] * top
        assert run_judge(tmp_path, rows=timing_rows(n, total))["c.cycle_p99"].verdict == verdict


def test_p99_excludes_nothing(tmp_path):
    # One outlier does not move p99, but it IS reported in max / over counts.
    total = [500] * 150_000 + [9000]
    s = summarise_timing(timing_rows(150_001, total), LIMIT_US, PERIOD_US)
    assert s.p99_total_us == 500
    assert s.max_total_us == 9000
    assert s.over_limit == 1
    assert s.over_period == 1


# ── (d) timeout rates ────────────────────────────────────────────────────────


@pytest.mark.parametrize(
    ("timeouts", "verdict"),
    [(0, PASS), (129, PASS), (130, UNDECIDED), (150, UNDECIDED), (170, UNDECIDED), (171, FAIL)],
)
def test_resp_set_timeout_band_at_150k_cycles(tmp_path, timeouts, verdict):
    c = run_judge(tmp_path, comm=comm_stats(joint_timeouts=timeouts))["d.resp_set_timeout"]
    assert c.verdict == verdict, c


def test_echo_timeout_uses_write_attempts_as_denominator(tmp_path):
    # 1000 writes: threshold count 1.0, band 1.65 sigma = 1.65 -> 0 PASS,
    # 1..2 UNDECIDED, 3+ FAIL (a fixed ±20 would have absorbed a 2 % rate).
    assert run_judge(tmp_path, comm=comm_stats(write_ok=1000))["d.echo_timeout"].verdict == PASS
    c = run_judge(tmp_path, comm=comm_stats(write_ok=999, write_timeouts=1))["d.echo_timeout"]
    assert c.verdict == UNDECIDED
    c = run_judge(tmp_path, comm=comm_stats(write_ok=998, write_timeouts=2))["d.echo_timeout"]
    assert c.verdict == UNDECIDED
    c = run_judge(tmp_path, comm=comm_stats(write_ok=997, write_timeouts=3))["d.echo_timeout"]
    assert c.verdict == FAIL
    assert "3 / 1000" in c.value


def test_no_writes_is_not_evaluated_not_passed(tmp_path):
    c = run_judge(tmp_path)["d.echo_timeout"]
    assert c.verdict == NOT_EVALUATED
    assert "no writes" in c.value


def test_aggregate_recv_timeout_is_not_the_rate(tmp_path):
    # 300 aggregate timeouts of which 150 are echo-stage: the per-stage judge
    # sees 150 RESP-set timeouts (UNDECIDED band), not a 0.2 % FAIL.
    comm = comm_stats(joint_timeouts=150, write_ok=149_850, write_timeouts=150)
    assert comm["recv_timeout"] == 300
    assert run_judge(tmp_path, comm=comm)["d.resp_set_timeout"].verdict == UNDECIDED


# ── (e) E-Stop ───────────────────────────────────────────────────────────────


def test_estop_window_boundary(tmp_path):
    # 0.1 ms either side of the 2 ms window: the candump timestamps are parsed
    # from 6-decimal text, so an exact-2.000 probe would hinge on rounding.
    ok = good_bus_lines()[:-4] + estop(1.04, echo_delay_s=0.0019)
    assert run_judge(tmp_path, lines=ok)["e.estop"].verdict == PASS
    late = good_bus_lines()[:-4] + estop(1.04, echo_delay_s=0.0021)
    c = run_judge(tmp_path, lines=late)["e.estop"]
    assert c.verdict == FAIL
    assert "2.100 ms" in c.value


def test_estop_requires_both_echoes(tmp_path):
    lines = good_bus_lines()[:-4] + estop(1.04, with_echoes=False)
    c = run_judge(tmp_path, lines=lines)["e.estop"]
    assert c.verdict == FAIL
    assert "echo missing" in c.note


def test_normal_zero_writes_are_not_estops(tmp_path):
    # Run B-2: zeros commanded all along in the NORMAL order (F0 then F1).
    lines: list[str] = []
    for i in range(10):
        lines += normal_write(1.0 + i * 0.002, ZERO44)
        lines += req_resp(1.0005 + i * 0.002)
    no_estop = bus_from(lines, tmp_path)
    assert no_estop.estop is None
    assert no_estop.estop_candidates == 0
    assert len(no_estop.rtt_cmd_echo_ms) == 10
    assert run_judge(tmp_path, lines=lines)["e.estop"].verdict == FAIL

    lines += estop(1.1)
    bus = bus_from(lines, tmp_path)
    assert bus.estop is not None
    assert bus.estop_candidates == 1
    assert bus.estop.cmd_frames_after == 0
    assert run_judge(tmp_path, lines=lines)["e.estop"].verdict == PASS


def test_commands_after_the_estop_fail(tmp_path):
    lines = good_bus_lines() + normal_write(2.0)
    c = run_judge(tmp_path, lines=lines)["e.estop"]
    assert c.verdict == FAIL
    assert "loop did not stop" in c.note


def test_bus_counts_and_rtt(tmp_path):
    bus = bus_from(good_bus_lines(20), tmp_path)
    assert (bus.req, bus.resp, bus.cmd, bus.echo, bus.other, bus.error_frames) == (
        20,
        60,
        2,
        2,
        0,
        0,
    )
    assert bus.complete_sets == 20
    assert bus.nonzero_dlc_req == 0
    assert len(bus.rtt_req_f2_ms) == 20
    assert bus.rtt_req_f2_ms[0] == pytest.approx(0.5, abs=1e-6)


def test_other_hand_traffic_is_ignored(tmp_path):
    left = node_id_of("left")
    lines = good_bus_lines() + [line(5.0, req_id(left)), line(5.0005, resp_id(left, 0))]
    bus = bus_from(lines, tmp_path)
    assert bus.other == 2
    assert bus.req == 20


def test_dlc0_command_pair_is_not_an_estop(tmp_path):
    lines = good_bus_lines()[:-4] + [line(1.04, cmd_id(NODE, 1)), line(1.0401, cmd_id(NODE, 0))]
    bus = bus_from(lines, tmp_path)
    assert bus.estop is None
    assert bus.estop_candidates == 0


def test_write_pairs_are_counted_for_the_denominator_cross_check(tmp_path):
    lines: list[str] = []
    for i in range(7):
        lines += normal_write(1.0 + i * 0.002)
        lines += req_resp(1.0005 + i * 0.002)
    lines += estop(1.1)
    assert bus_from(lines, tmp_path).write_pairs == 7


# ── cmd_ack none (firmware without CMD echoes) ───────────────────────────────


def none_bus_lines(n_req: int = 20) -> list[str]:
    lines: list[str] = []
    for i in range(n_req):
        lines += req_resp(1.0 + i * 0.002)
    return lines + estop(1.0 + n_req * 0.002, with_echoes=False)


def test_none_mode_echo_timeout_not_evaluated_set_timeout_still_judged(tmp_path):
    c = run_judge(tmp_path, comm=none_comm_stats(), lines=none_bus_lines())
    assert c["d.echo_timeout"].verdict == NOT_EVALUATED
    assert c["d.echo_timeout"].value == "NOT_EVALUATED(cmd_ack none: no echo stage)"
    # Even with writes counted, the echo row is never judged in none mode.
    assert run_judge(tmp_path, comm=none_comm_stats(write_ok=0))["d.echo_timeout"].verdict == (
        NOT_EVALUATED
    )
    for timeouts, verdict in ((0, PASS), (150, UNDECIDED), (171, FAIL)):
        comm = none_comm_stats(joint_timeouts=timeouts)
        c = run_judge(tmp_path, comm=comm, lines=none_bus_lines())
        assert c["d.resp_set_timeout"].verdict == verdict


def test_none_mode_estop_pair_passes_without_echoes(tmp_path):
    c = run_judge(tmp_path, comm=none_comm_stats(), lines=none_bus_lines())["e.estop"]
    assert c.verdict == PASS, c
    assert "CMD F1 zeros -> CMD F0 zeros" in c.value
    assert "0.100 ms" in c.value
    assert "both echoes" not in c.threshold
    # The same bus judged as an echo run is missing its echoes.
    assert run_judge(tmp_path, lines=none_bus_lines())["e.estop"].verdict == FAIL


def test_none_mode_estop_fails_on_wrong_order_or_late_f0_or_cmd_after(tmp_path):
    comm = none_comm_stats()
    head = none_bus_lines()[:-2]  # drop the E-Stop pair
    # Wrong order: F0 zeros then F1 zeros is a normal write, not an E-Stop.
    wrong = head + [line(1.04, cmd_id(NODE, 0), ZERO44), line(1.0401, cmd_id(NODE, 1), ZERO44)]
    assert run_judge(tmp_path, comm=comm, lines=wrong)["e.estop"].verdict == FAIL
    # F0 zeros 2.1 ms after F1 zeros: outside the window; 1.9 ms is inside.
    for gap, verdict in ((0.0019, PASS), (0.0021, FAIL)):
        pair = [line(1.04, cmd_id(NODE, 1), ZERO44), line(1.04 + gap, cmd_id(NODE, 0), ZERO44)]
        c = run_judge(tmp_path, comm=comm, lines=head + pair)["e.estop"]
        assert c.verdict == verdict, c
    # A command after the pair: the loop did not stop.
    after = none_bus_lines() + [
        line(2.0, cmd_id(NODE, 0), NONZERO44),
        line(2.0001, cmd_id(NODE, 1), NONZERO44),
    ]
    c = run_judge(tmp_path, comm=comm, lines=after)["e.estop"]
    assert c.verdict == FAIL
    assert "loop did not stop" in c.note


def _render(tmp_path, comm, lines):
    timing = summarise_timing(timing_rows(150_001), TH.p99_limit_us, TH.period_us)
    bus = bus_from(lines, tmp_path)
    checks = judge(timing, comm, bus, TH, link_down_lines=0, link_status_false=0)
    return render_markdown(checks, timing, comm, bus, TH)


def test_none_mode_report_rows(tmp_path):
    md = _render(tmp_path, none_comm_stats(), none_bus_lines())
    assert "| echo_unexpected | 0 | 0 |" in md
    assert "ECHO 0 (cmd_ack none)" in md
    assert "CMD frame 0 -> last echo round trip" not in md
    assert "D42: 2 x T < period (nominal; no echo stage)" in md
    assert "includes the echo handling" not in md
    assert "echo F1" not in md
    assert "NOT_EVALUATED(cmd_ack none: no echo stage)" in md


def test_missing_cmd_ack_key_reads_as_echo_and_prints_no_none(tmp_path):
    comm = comm_stats(write_ok=1000)
    assert "cmd_ack" not in comm["canfd"]
    md = _render(tmp_path, comm, good_bus_lines())
    assert "cmd_ack" not in md
    assert "echo_unexpected" not in md
    assert "CMD frame 0 -> last echo round trip" in md
    assert "D42: 2 x T < period |" in md
    assert "includes the echo handling" in md
    c = run_judge(tmp_path, comm=comm)
    assert c["d.echo_timeout"].verdict == PASS
    assert c["e.estop"].verdict == PASS


# ── pipelined write ticks (#919, D49 (b) · D59) ──────────────────────────────


def test_pipelined_write_ticks_keep_pairs_sets_and_the_estop_apart(tmp_path):
    # 6 write ticks (3 with RESP 0 ahead of the echoes), 4 read-only ticks, an
    # incomplete write tick, then the E-Stop. Interleaved echoes do not break
    # the REQ -> RESP set, the pair count or the E-Stop detection.
    lines: list[str] = []
    t = 1.0
    for i in range(6):
        lines += pipelined_write(t, resp_first=(i % 2 == 1))
        t += 0.002
    for _ in range(4):
        lines += req_resp(t)
        t += 0.002
    lines += pipelined_write(t, complete=False)
    t += 0.002
    lines += estop(t)
    bus = bus_from(lines, tmp_path)
    assert bus.write_pairs == 7
    assert (bus.cmd, bus.echo, bus.req) == (16, 16, 11)
    assert bus.complete_sets == 10
    assert bus.incomplete_sets == 1
    assert bus.estop_candidates == 1
    assert bus.estop is not None and bus.estop.cmd_frames_after == 0
    assert len(bus.rtt_cmd_echo_ms) == 7
    assert bus.rtt_cmd_echo_ms[0] == pytest.approx(0.77, abs=1e-6)
    # REQ -> F2 for every complete set (write and read-only ticks alike) ...
    assert len(bus.rtt_req_f2_ms) == 10
    assert bus.rtt_req_f2_ms[0] == pytest.approx((0.0004 + 0.00026 - 0.00008) * 1e3, abs=1e-6)
    # ... the read-only ticks' alone (the baseline figure) ...
    assert len(bus.rtt_req_f2_readonly_ms) == 4
    assert all(v == pytest.approx(0.5, abs=1e-6) for v in bus.rtt_req_f2_readonly_ms)
    # ... and CMD 0 -> F2 only for the write ticks whose set completed.
    assert len(bus.rtt_cmd_f2_ms) == 6
    assert all(v == pytest.approx(0.66, abs=1e-6) for v in bus.rtt_cmd_f2_ms)
    assert run_judge(tmp_path, lines=lines)["e.estop"].verdict == PASS


def test_sequential_write_ticks_have_no_cmd_f2_without_a_following_req(tmp_path):
    # Pre-#919 order (write, then a separate REQ) still yields the figure — the
    # REQ follows the pair before the next command — but a pair with no REQ
    # before the next CMD (E-Stop right after a write) does not.
    lines = normal_write(1.0) + req_resp(1.0005) + normal_write(1.002) + estop(1.0025)
    bus = bus_from(lines, tmp_path)
    assert bus.write_pairs == 2
    assert len(bus.rtt_cmd_f2_ms) == 1
    assert bus.rtt_cmd_f2_ms[0] == pytest.approx((1.0005 + 0.0005 - 1.0) * 1e3, abs=1e-6)


def test_write_tick_without_its_req_does_not_borrow_the_next_ticks(tmp_path):
    # The write tick's REQ never left (send failure): the next tick's REQ, 2 ms
    # later, must not become this tick's CMD 0 -> F2 sample, and that tick stays
    # a read-only one for the baseline row.
    lines = pipelined_write(1.0)[:2] + pipelined_write(1.0)[3:5]  # CMD 0 · CMD 1 · echoes, no REQ
    lines += req_resp(1.002) + estop(1.004)
    bus = bus_from(lines, tmp_path)
    assert bus.write_pairs == 1
    assert bus.rtt_cmd_f2_ms == []
    assert len(bus.rtt_req_f2_readonly_ms) == 1
    # With the bound relaxed past a period the sample would be taken (the
    # bound is what keeps it out).
    frames, unparsed = read_candump(tmp_path / "g3.log")
    assert len(analyse_bus(frames, NODE, unparsed, cmd_req_max_s=0.01).rtt_cmd_f2_ms) == 1


def test_report_renders_the_cmd_f2_distribution(tmp_path):
    lines = pipelined_write(1.0) + estop(1.002)
    bus = bus_from(lines, tmp_path)
    assert len(bus.rtt_cmd_f2_ms) == 1
    md = render_markdown([], summarise_timing([], LIMIT_US, PERIOD_US), comm_stats(), bus, TH)
    assert "CMD frame 0 -> RESP frame 2, write ticks" in md
    assert "n=1 p50=0.660" in md
    assert "read-only ticks" in md and "n=0" in md  # no read-only tick in this log


GOLDEN_ECHO_RENDER = Path(__file__).parent / "data" / "hand_canfd_gate_echo_render.md"


def _echo_mode_render(tmp_path) -> str:
    """Echo-mode report on a fixed synthetic session: sequential + pipelined
    writes, read-only ticks, an incomplete set and the E-Stop pair."""
    lines: list[str] = []
    t = 1.0
    for i in range(5):
        lines += normal_write(t)
        lines += req_resp(t + 0.0005)
        t += 0.002
        lines += pipelined_write(t, resp_first=bool(i % 2))
        t += 0.002
        lines += req_resp(t)
        t += 0.002
    lines += pipelined_write(t, complete=False)
    t += 0.002
    lines += estop(t)
    comm = comm_stats(total_cycles=150_000, joint_timeouts=3, write_ok=10, write_timeouts=0)
    timing = summarise_timing(timing_rows(150_001), TH.p99_limit_us, TH.period_us)
    bus = bus_from(lines, tmp_path)
    checks = judge(timing, comm, bus, TH, link_down_lines=0, link_status_false=0)
    return render_markdown(checks, timing, comm, bus, TH)


def test_echo_mode_render_is_unchanged(tmp_path):
    # Golden file: echo-mode output must stay byte-identical across cmd_ack changes.
    assert _echo_mode_render(tmp_path) == GOLDEN_ECHO_RENDER.read_text()


def test_period_mismatch_refuses_to_judge_p99(tmp_path):
    # A 250 Hz run (4 ms ticks) judged with the 500 Hz default threshold.
    rows = timing_rows(75_001, period_ns=4_000_000)
    c = run_judge(tmp_path, rows=rows)["c.cycle_p99"]
    assert c.verdict == FAIL
    assert "re-run with the right rate" in c.note
    c = run_judge(tmp_path, rows=rows, th=Thresholds(loop_rate_hz=250.0))["c.cycle_p99"]
    assert c.verdict == PASS


def test_t_wall_ns_is_read_exactly(tmp_path):
    csv_path = tmp_path / "t.csv"
    csv_path.write_text(
        "t_wall_ns,tick_count,run_id,t_state_us,t_compute_us,t_publish_us,t_total_us,jitter_us\n"
        "1697000000123456789,0,0,1,1,1,500,2\n"
    )
    assert load_timing_csv(csv_path)[0]["t_wall_ns"] == 1697000000123456789


# ── overall / CLI ────────────────────────────────────────────────────────────


def test_overall_precedence(tmp_path):
    checks = run_judge(tmp_path)
    assert overall(list(checks.values())) == PASS  # d.echo_timeout is NOT_EVALUATED
    checks = run_judge(tmp_path, comm=comm_stats(joint_timeouts=150))
    assert overall(list(checks.values())) == UNDECIDED
    checks = run_judge(tmp_path, comm=comm_stats(joint_timeouts=150, link_ok=False))
    assert overall(list(checks.values())) == FAIL


def write_session(
    tmp_path: Path, suffix: str = "", rows=None, comm=None, legacy_names: bool = False
) -> Path:
    session = tmp_path / "session"
    (session / "timing").mkdir(parents=True, exist_ok=True)
    (session / "device").mkdir(exist_ok=True)
    timing_stem = PRE_F08_HAND_STEMS[TIMING_STEM] if legacy_names else TIMING_STEM
    stats_stem = PRE_F08_HAND_STEMS[STATS_STEM] if legacy_names else STATS_STEM
    rows = rows if rows is not None else timing_rows(150_001)
    with (session / "timing" / f"{timing_stem}{suffix}.csv").open("w") as fh:
        fh.write(
            "t_wall_ns,tick_count,run_id,t_state_us,t_compute_us,t_publish_us,t_total_us,jitter_us\n"
        )
        for r in rows:
            fh.write(
                f"{r['t_wall_ns']},{r['tick_count']},{r['run_id']},400,50,0,"
                f"{r['t_total_us']},{r['jitter_us']}\n"
            )
    (session / "device" / f"{stats_stem}{suffix}.json").write_text(
        json.dumps({"comm_stats": comm if comm is not None else comm_stats(), "timing_stats": {}})
    )
    return session


def _names(pair) -> tuple[str, str]:
    return tuple(p.name for p in pair)


def test_artifacts_fall_back_to_the_pre_f08_pair_only_as_a_pair(tmp_path):
    # Pre-F08 (#905) session: both artifacts under the old names, D51 suffix kept.
    legacy = write_session(tmp_path / "legacy", suffix="_left", legacy_names=True)
    assert _names(_artifacts(legacy, "_left")) == (
        "hand_udp_timing_log_left.csv",
        "hand_udp_stats_left.json",
    )
    # Neither name present: the current names, so the exit-3 message shows them.
    assert _names(_artifacts(legacy, "")) == ("hand_timing_log.csv", "hand_stats.json")
    # Two complete pairs (a pre- and a post-rename binary restarted into the
    # same minute's session): the current pair.
    both = write_session(tmp_path / "both", legacy_names=True)
    write_session(tmp_path / "both")
    assert _names(_artifacts(both, "")) == ("hand_timing_log.csv", "hand_stats.json")
    # A mixed session (old run's CSV, new run's JSON) must not be judged as one
    # run: it resolves to the current pair, whose CSV is missing -> exit 3.
    mixed = write_session(tmp_path / "mixed", legacy_names=True)
    write_session(tmp_path / "mixed")
    (mixed / "timing" / "hand_timing_log.csv").unlink()
    (mixed / "device" / "hand_udp_stats.json").unlink()
    assert _names(_artifacts(mixed, "")) == ("hand_timing_log.csv", "hand_stats.json")
    log = tmp_path / "g3.log"
    log.write_text("\n".join(good_bus_lines()) + "\n")
    assert main([str(mixed), str(log)]) == 3


def test_main_reads_a_pre_f08_session(tmp_path):
    session = write_session(tmp_path, legacy_names=True, comm=comm_stats(joint_timeouts=150))
    log = tmp_path / "g3.log"
    log.write_text("\n".join(good_bus_lines()) + "\n")
    assert main([str(session), str(log)]) == 2  # judged, not exit 3


def test_main_end_to_end_pass(tmp_path, capsys):
    session = write_session(tmp_path)
    log = tmp_path / "g3.log"
    log.write_text("\n".join(good_bus_lines()) + "\n")
    node_log = tmp_path / "node.log"
    node_log.write_text("[INFO] Hand link UP\n")
    link = tmp_path / "link.log"
    link.write_text("data: true\n---\ndata: true\n---\n")
    out_json = tmp_path / "g3.json"
    rc = main(
        [
            str(session),
            str(log),
            "--node-log",
            str(node_log),
            "--link-status",
            str(link),
            "--json-out",
            str(out_json),
        ]
    )
    assert rc == 0
    text = capsys.readouterr().out
    assert "## G-3 verdict: **PASS**" in text
    assert "not evaluated: d.echo_timeout" in text
    payload = json.loads(out_json.read_text())  # strict JSON: no NaN tokens
    assert payload["verdict"] == PASS
    assert {c["key"] for c in payload["checks"]} >= {"a.duration", "e.estop", "d.resp_set_timeout"}


def test_main_exit_codes_and_suffix(tmp_path):
    session = write_session(tmp_path, suffix="_left", comm=comm_stats(joint_timeouts=150))
    log = tmp_path / "g3.log"
    log.write_text("\n".join(good_bus_lines()) + "\n")
    # Explicit side picks the D51 suffix; the right-hand node id would see no frames.
    assert main([str(session), str(log), "--hand-side", "left", "--explicit-side"]) == 1
    # Same artifacts judged as the right hand (frames are right-hand ids): UNDECIDED from (d).
    assert main([str(session), str(log), "--suffix", "_left"]) == 2
    # Missing artifacts are a usage error, not a verdict.
    assert main([str(tmp_path / "nowhere"), str(log)]) == 3


def test_p99_fraction_must_be_positive(tmp_path):
    # D89 renders (c) as "period / 1.1"; a zero fraction is a usage error, not a crash.
    log = tmp_path / "g3.log"
    log.write_text("\n".join(good_bus_lines()) + "\n")
    for bad in ("0", "-0.5"):
        with pytest.raises(SystemExit) as exc:
            main([str(tmp_path), str(log), "--p99-fraction", bad])
        assert exc.value.code == 2
    c = run_judge(tmp_path, th=Thresholds(p99_fraction=0.0))["c.cycle_p99"]
    assert c.verdict == FAIL


def test_run_id_selection(tmp_path):
    rows = timing_rows(10, run_id=0) + timing_rows(150_001, run_id=1)
    session = write_session(tmp_path, rows=rows)
    log = tmp_path / "g3.log"
    log.write_text("\n".join(good_bus_lines()) + "\n")
    assert main([str(session), str(log)]) == 1  # two run_ids -> a.single_run FAIL
    assert main([str(session), str(log), "--run-id", "1"]) == 0
    assert main([str(session), str(log), "--run-id", "7"]) == 3


def test_loaders_reject_the_wrong_inputs(tmp_path):
    bad = tmp_path / "x.csv"
    bad.write_text("t_wall_ns,tick_count\n1,2\n")
    with pytest.raises(ValueError):
        load_timing_csv(bad)
    udp = tmp_path / "udp.json"
    udp.write_text(json.dumps({"comm_stats": {"transport": "udp"}}))
    with pytest.raises(ValueError):
        load_comm_stats(udp)


# ── two hands on one bus (x, G-5 / epic #906) ───────────────────────────────

RIGHT = node_id_of("right")
LEFT = node_id_of("left")


def hand_tx(
    node: int, t: float, write: bool = False, echo: bool = True, complete: bool = True
) -> list[str]:
    """One comm cycle of `node` in row order: [CMD 0 · CMD 1] · REQ · [echoes] · RESP."""
    out: list[str] = []
    if write:
        out += [line(t, cmd_id(node, 0), NONZERO44), line(t + 4e-5, cmd_id(node, 1), NONZERO44)]
    out.append(line(t + 8e-5, req_id(node)))
    if write and echo:
        out += [
            line(t + 5e-4, echo_id(node, 0), NONZERO44),
            line(t + 7.7e-4, echo_id(node, 1), NONZERO44),
        ]
    resp = [line(t + 4e-4 + 1.3e-4 * i, resp_id(node, i), "11" * 8) for i in range(3)]
    return out + (resp if complete else resp[:2])


def two_hand_estop(t: float, with_echoes: bool = True) -> list[str]:
    """OnLoopAborted: right F1 · F0 zeros, then left — echoes arrive after both pairs."""
    out = [
        line(t, cmd_id(RIGHT, 1), ZERO44),
        line(t + 4e-5, cmd_id(RIGHT, 0), ZERO44),
        line(t + 2e-4, cmd_id(LEFT, 1), ZERO44),
        line(t + 2.4e-4, cmd_id(LEFT, 0), ZERO44),
    ]
    if with_echoes:
        out += [
            line(t + 5e-4, echo_id(RIGHT, 1), ZERO44),
            line(t + 6e-4, echo_id(RIGHT, 0), ZERO44),
            line(t + 7e-4, echo_id(LEFT, 1), ZERO44),
            line(t + 8e-4, echo_id(LEFT, 0), ZERO44),
        ]
    return out


def two_hand_lines(n_ticks: int = 20, echo: bool = True) -> list[str]:
    """250 Hz ticks, right then left; every other tick writes both hands."""
    lines: list[str] = []
    for i in range(n_ticks):
        t = 1.0 + i * 0.004
        lines += hand_tx(RIGHT, t, write=i % 2 == 0, echo=echo)
        lines += hand_tx(LEFT, t + 0.0015, write=i % 2 == 0, echo=echo)
    return lines + two_hand_estop(1.0 + n_ticks * 0.004, with_echoes=echo)


def overlap_of(lines: list[str], tmp_path: Path):
    log = tmp_path / "g5.log"
    log.write_text("\n".join(lines) + "\n")
    frames, _ = read_candump(log)
    return analyse_overlap(frames)


def test_two_hands_back_to_back_have_no_overlap(tmp_path):
    o = overlap_of(two_hand_lines(), tmp_path)
    assert o.two_hands
    assert o.transactions == {"right": 20, "left": 20}
    assert o.violations == []
    # D78: the left pair follows the right one, reported in ms.
    assert o.estop_gap_ms == pytest.approx(0.2, abs=1e-6)
    checks = run_judge(tmp_path, lines=two_hand_lines(), overlap=o)
    assert checks["x.no_overlap"].verdict == PASS


def test_another_hands_resp_inside_a_transaction_is_one_intrusion(tmp_path):
    lines = hand_tx(RIGHT, 1.0)
    lines.insert(2, line(1.00045, resp_id(LEFT, 0), "11" * 8))  # REQ · RESP 0 · stray · …
    lines += hand_tx(LEFT, 1.0015)
    o = overlap_of(lines, tmp_path)
    assert [(v.intruder, v.victim, v.kind) for v in o.violations] == [("left", "right", "resp")]
    c = _overlap_check(o)
    assert c.verdict == FAIL
    assert "left -> right x1" in c.note


def test_a_req_before_the_set_completes_counts_the_late_tail(tmp_path):
    # The host sent the left REQ while the right set was still arriving: the
    # right hand's late frames fall into the left window and count against it.
    right = hand_tx(RIGHT, 1.0)  # REQ, RESP 0, 1, 2
    lines = right[:2] + [line(1.0006, req_id(LEFT))] + right[2:]
    lines += [line(1.0012 + 1.3e-4 * i, resp_id(LEFT, i), "11" * 8) for i in range(3)]
    o = overlap_of(lines, tmp_path)
    assert o.by_pair == {"right -> left": 2}


def test_a_late_echo_in_the_next_hands_write_is_the_late_hands(tmp_path):
    right = hand_tx(RIGHT, 1.0, write=True)  # CMD0 CMD1 REQ E0 E1 R0 R1 R2
    late_echo = right.pop(4)  # E1 lands after the left tick started
    left = hand_tx(LEFT, 1.0015, write=True)
    lines = right + left[:1] + [late_echo] + left[1:]
    o = overlap_of(lines, tmp_path)
    assert [(v.intruder, v.victim, v.kind) for v in o.violations] == [("right", "left", "echo")]


def test_a_set_without_frame_2_then_the_next_hand_is_no_overlap(tmp_path):
    lines = hand_tx(RIGHT, 1.0, complete=False) + hand_tx(LEFT, 1.0015)
    lines += hand_tx(RIGHT, 1.004) + hand_tx(LEFT, 1.0055)
    assert overlap_of(lines, tmp_path).violations == []


def test_cmd_ack_none_two_hands_have_no_overlap(tmp_path):
    o = overlap_of(two_hand_lines(echo=False), tmp_path)
    assert o.violations == []
    assert o.transactions == {"right": 20, "left": 20}


def test_the_estop_echoes_after_the_cut_are_not_judged(tmp_path):
    # The four zero frames go out back to back, every echo after them.
    o = overlap_of(two_hand_lines(n_ticks=2), tmp_path)
    assert o.violations == []
    assert o.estop_line is not None
    assert set(o.estop_f1_t) == {"right", "left"}


def test_one_hand_on_the_bus_reads_as_before(tmp_path):
    lines = good_bus_lines()
    o = overlap_of(lines, tmp_path)
    assert not o.two_hands
    timing = summarise_timing(timing_rows(150_001), TH.p99_limit_us, TH.period_us)
    bus = bus_from(lines, tmp_path)
    comm = comm_stats()
    plain = judge(timing, comm, bus, TH, link_down_lines=0, link_status_false=0)
    with_o = judge(timing, comm, bus, TH, link_down_lines=0, link_status_false=0, overlap=o)
    assert [c.key for c in with_o] == [c.key for c in plain]
    assert render_markdown(with_o, timing, comm, bus, TH, o) == render_markdown(
        plain, timing, comm, bus, TH
    )


def test_two_hands_on_the_bus_but_one_transacting_is_not_evaluated(tmp_path):
    lines: list[str] = []
    for i in range(5):
        lines += hand_tx(RIGHT, 1.0 + i * 0.004)
    lines += two_hand_estop(1.03)  # the left hand appears only after the cut
    o = overlap_of(lines, tmp_path)
    assert o.two_hands
    assert _overlap_check(o).verdict == NOT_EVALUATED


def test_silent_req_budget_follows_the_rate():
    assert silent_req_budget(100.0, 500.0) == TH.silent_req_max == 50
    assert silent_req_budget(100.0, 250.0) == 25
    assert silent_req_budget(100.0, 500.0, comm_decimation=2) == 25
    assert silent_req_budget(0.1, 500.0) == 1
    # Half-way counts round up, as std::lround does in the node.
    assert silent_req_budget(100.0, 125.0) == 13
    assert silent_req_budget(10.0, 250.0) == 3
    assert silent_req_budget(100.0, 500.0, comm_decimation=0) == 50


def test_main_judges_one_hand_of_a_two_hand_session(tmp_path, capsys):
    # hand_sides run: one unsuffixed CSV at 250 Hz, a suffixed JSON per hand.
    session = write_session(tmp_path, rows=timing_rows(75_001, period_ns=4_000_000))
    for side in ("right", "left"):
        (session / "device" / f"{STATS_STEM}_{side}.json").write_text(
            json.dumps({"comm_stats": comm_stats(total_cycles=75_000), "timing_stats": {}})
        )
    log = tmp_path / "g5.log"
    log.write_text("\n".join(two_hand_lines()) + "\n")
    out_json = tmp_path / "g5.json"
    args = [str(session), str(log), "--explicit-side", "--timing-suffix", ""]
    args += ["--loop-rate-hz", "250", "--json-out", str(out_json)]
    for side in ("right", "left"):
        assert main([*args, "--hand-side", side]) == 0, side
        text = capsys.readouterr().out
        assert f"{STATS_STEM}_{side}.json" in text
        assert "## G-5 verdict: **PASS**" in text
        assert "| `x.no_overlap` | PASS |" in text
        assert "| < 25 |" in text  # silent REQ budget at 250 Hz
        assert "E-Stop zero-write gap, right CMD F1 -> left CMD F1 | +0.200 ms" in text
        payload = json.loads(out_json.read_text())
        assert payload["overlap"]["violations"] == 0
        assert payload["thresholds"]["silent_req_max"] == 25


# ── req_mode broadcast (#956) ────────────────────────────────────────────────

BCAST_REQ = broadcast_req_id()


def bcast_tick(
    t: float,
    write: bool = False,
    order: str = "RRRLLL",
    drop: tuple[tuple[int, int], ...] = (),
    data_hex: str = NONZERO44,
) -> list[str]:
    """One broadcast tick: [right CMD 0 · 1, left CMD 0 · 1] · REQ(0xFF) · both
    RESP sets in `order` (R / L = the next frame of that hand); (node, idx) in
    `drop` never arrive. cmd_ack none: no echoes."""
    out: list[str] = []
    if write:
        out += [
            line(t, cmd_id(RIGHT, 0), data_hex),
            line(t + 4e-5, cmd_id(RIGHT, 1), data_hex),
            line(t + 8e-5, cmd_id(LEFT, 0), data_hex),
            line(t + 1.2e-4, cmd_id(LEFT, 1), data_hex),
        ]
    out.append(line(t + 1.6e-4, BCAST_REQ))
    nxt = {RIGHT: 0, LEFT: 0}
    for k, c in enumerate(order):
        node = RIGHT if c == "R" else LEFT
        idx = nxt[node]
        nxt[node] += 1
        if (node, idx) not in drop:
            out.append(line(t + 4e-4 + 1.3e-4 * k, resp_id(node, idx), "11" * 8))
    return out


def bcast_lines(n_ticks: int = 20, estop_end: bool = True, **per_tick) -> list[str]:
    """250 Hz broadcast ticks, every other one a write; `per_tick[i]` overrides tick i."""
    lines: list[str] = []
    for i in range(n_ticks):
        kw = {"write": i % 2 == 0, **per_tick.get(f"t{i}", {})}
        lines += bcast_tick(1.0 + i * 0.004, **kw)
    if estop_end:
        lines += two_hand_estop(1.0 + n_ticks * 0.004, with_echoes=False)
    return lines


def frames_of(lines: list[str], tmp_path: Path):
    log = tmp_path / "bcast.log"
    log.write_text("\n".join(lines) + "\n")
    return read_candump(log)[0]


def bcast_overlap(lines: list[str], tmp_path: Path):
    return analyse_overlap(frames_of(lines, tmp_path), "broadcast", cmd_req_max_s=0.002)


def write_two_hand_session(tmp_path: Path, lines: list[str], req_mode: str | None) -> Path:
    """hand_sides run at 250 Hz, cmd_ack none; `req_mode` None = an archive without the key."""
    session = write_session(tmp_path, rows=timing_rows(75_001, period_ns=4_000_000))
    comm = none_comm_stats(total_cycles=75_000)
    if req_mode is not None:
        comm["canfd"]["req_mode"] = req_mode
    for side in ("right", "left"):
        (session / "device" / f"{STATS_STEM}_{side}.json").write_text(
            json.dumps({"comm_stats": comm, "timing_stats": {}})
        )
    (tmp_path / "bus.log").write_text("\n".join(lines) + "\n")
    return session


def run_main(tmp_path: Path, session: Path, side: str, *extra: str) -> tuple[int, dict | None]:
    out_json = tmp_path / f"gate_{side}.json"
    out_json.unlink(missing_ok=True)
    args = [str(session), str(tmp_path / "bus.log"), "--explicit-side", "--timing-suffix", ""]
    args += ["--loop-rate-hz", "250", "--json-out", str(out_json), "--hand-side", side, *extra]
    rc = main(args)
    return rc, json.loads(out_json.read_text()) if out_json.exists() else None


def test_broadcast_req_is_every_hands_req_only_when_asked(tmp_path):
    assert BCAST_REQ == 0x007FC000
    frames = frames_of(bcast_lines(10), tmp_path)
    for node in (RIGHT, LEFT):
        bus = analyse_bus(frames, node, broadcast_req=True)
        assert (bus.req, bus.resp, bus.complete_sets, bus.incomplete_sets) == (10, 30, 10, 0)
        assert bus.write_pairs == 5
        assert len(bus.rtt_cmd_f2_ms) == 5
        assert len(bus.rtt_req_f2_readonly_ms) == 5
        assert bus.estop_candidates == 1
        assert bus.estop is not None and bus.estop.cmd_frames_after == 0
        # Default (per_hand): the broadcast REQ is another node's frame.
        assert analyse_bus(frames, node).req == 0


def test_complete_broadcast_run_passes_end_to_end(tmp_path, capsys):
    order = {f"t{i}": {"order": "LRRRLL"} for i in range(0, 20, 4)}
    lines = bcast_lines(20, **order)
    session = write_two_hand_session(tmp_path, lines, "broadcast")
    for side in ("right", "left"):
        rc, payload = run_main(tmp_path, session, side)  # auto: from canfd.req_mode
        text = capsys.readouterr().out
        assert rc == 0, text
        assert "## G-5 verdict: **PASS**" in text
        assert "| `x.broadcast_sets` | PASS |" in text
        assert "x.no_overlap" not in text
        assert "| req_mode (--req-mode, else JSON canfd.req_mode) | broadcast |" in text
        assert "RRRLLL ×15, LRRRLL ×5" in text
        assert "the later hand's figure includes the other hand's burst" in text
        assert payload["req_mode"] == "broadcast"
        assert payload["overlap"]["violations"] == 0
        assert payload["overlap"]["estop_gap_ms"] == pytest.approx(0.2, abs=1e-6)
        b = payload["overlap"]["broadcast"]
        assert (b["windows"], b["truncated"], b["complete"]) == (20, 0, 20)
        assert b["order"] == {"RRRLLL": 15, "LRRRLL": 5}
        assert b["req_f2_ms"]["right"].startswith("n=20 ")
        assert b["req_f2_ms"]["left"].startswith("n=20 ")
        assert b["cmd_f2_ms"].startswith("n=10 ")
        assert payload["bus"]["req"] == 20
        assert payload["bus"]["complete_sets"] == 20


@pytest.mark.parametrize(
    ("n_missing", "verdict"), [(0, PASS), (4, PASS), (10, UNDECIDED), (16, FAIL)]
)
def test_one_hand_missing_rate_uses_the_d_band(tmp_path, n_missing, verdict):
    # 200 windows at a 5 % threshold: limit 10, band 1.65 x sqrt(200 x 0.05 x 0.95) = 5.09
    # -> 0..4 PASS, 5..15 UNDECIDED, 16+ FAIL. Two more windows lose BOTH sets:
    # (d) judges those per hand, they are not one-hand-missing.
    per_tick = {f"t{i}": {"drop": ((LEFT, 1),)} for i in range(n_missing)}
    per_tick["t196"] = per_tick["t197"] = {"drop": ((RIGHT, 2), (LEFT, 2))}
    lines = bcast_lines(200, **per_tick)
    o = bcast_overlap(lines, tmp_path)
    b = o.broadcast
    assert (b.windows, b.one_hand_missing, b.all_missing) == (200, n_missing, 2)
    assert b.missing == {"right": 2, "left": n_missing + 2}
    assert b.one_hand_missing_by_hand == {"right": 0, "left": n_missing}
    if n_missing:
        assert b.order["RRRLL"] == n_missing
    c = _broadcast_check(b, Thresholds(timeout_rate_max=0.05))
    assert c.verdict == verdict, c
    assert f"{n_missing} / 200" in c.value
    # The same windows are the left hand's incomplete sets for (b) / (d).
    left = analyse_bus(frames_of(lines, tmp_path), LEFT, broadcast_req=True)
    assert left.incomplete_sets == n_missing + 2


def test_duplicate_resp_in_a_window_fails(tmp_path):
    assert bcast_overlap(bcast_lines(10), tmp_path).broadcast.duplicates == 0
    tick = bcast_tick(1.012)  # REQ R0 R1 R2 L0 L1 L2
    tick.insert(5, tick[4])  # left RESP 0 twice
    lines = bcast_lines(3, estop_end=False) + tick + two_hand_estop(1.02, with_echoes=False)
    o = bcast_overlap(lines, tmp_path)
    assert o.broadcast.duplicates == 1
    assert [(v.intruder, v.victim, v.kind) for v in o.violations] == [
        ("left", "left", "duplicate_resp")
    ]
    c = _broadcast_check(o.broadcast, TH)
    assert c.verdict == FAIL
    assert "duplicate RESP 1" in c.value


def test_host_cmd_inside_the_burst_fails_and_the_next_write_does_not(tmp_path):
    # Next tick's write after the last RESP is fine (every write tick in bcast_lines).
    assert bcast_overlap(bcast_lines(10), tmp_path).broadcast.cmd_in_burst == 0
    tick = bcast_tick(1.040)  # REQ R0 R1 R2 L0 L1 L2
    tick.insert(4, line(1.04085, cmd_id(LEFT, 0), NONZERO44))  # after R2, before L0
    lines = bcast_lines(10, estop_end=False) + tick + two_hand_estop(1.05, with_echoes=False)
    o = bcast_overlap(lines, tmp_path)
    assert o.broadcast.cmd_in_burst == 1
    assert [(v.intruder, v.victim, v.kind) for v in o.violations] == [
        ("host", "left", "cmd_in_burst")
    ]
    assert _broadcast_check(o.broadcast, TH).verdict == FAIL


def test_a_resp_after_the_next_ticks_write_is_late_not_a_violation(tmp_path):
    # Left RESP 2 of tick 10 arrives after tick 11's CMD pair (the host timed
    # out and moved on): reported late, the window is one-hand-missing, no FAIL.
    tick = bcast_tick(1.040, drop=((LEFT, 2),))
    nxt = bcast_tick(1.044, write=True)  # CMD R0 R1 L0 L1 · REQ · RESP x6
    late = line(1.04413, resp_id(LEFT, 2), "11" * 8)
    lines = bcast_lines(10, estop_end=False) + tick + nxt[:4] + [late] + nxt[4:]
    lines += bcast_tick(1.048) + two_hand_estop(1.052, with_echoes=False)
    o = bcast_overlap(lines, tmp_path)
    b = o.broadcast
    assert o.violations == []
    assert (b.duplicates, b.cmd_in_burst) == (0, 0)
    assert b.late_resp == {"right": 0, "left": 1}
    assert b.one_hand_missing == 1
    assert b.one_hand_missing_by_hand == {"right": 0, "left": 1}
    assert b.windows == 13
    assert "duplicate RESP 0, CMD in burst 0; one-hand-missing 1 / 13" in (
        _broadcast_check(b, TH).value
    )


@pytest.mark.parametrize(("gap_s", "in_burst", "late"), [(0.0019, 1, 0), (0.0021, 0, 1)])
def test_a_cmd_within_half_a_period_before_a_resp_is_in_the_burst(tmp_path, gap_s, in_burst, late):
    # A host CMD 1.9 ms after the REQ (half period 2 ms at 250 Hz) followed by a
    # RESP is the host sending during its own tick's burst; at 2.1 ms it is the
    # next tick's write and the RESP after it is late.
    tick = bcast_tick(1.040)  # REQ R0 R1 R2 L0 L1 L2
    t_req = 1.04016
    lines = bcast_lines(10, estop_end=False) + tick[:6]
    lines += [
        line(t_req + gap_s, cmd_id(LEFT, 0), NONZERO44),
        line(t_req + gap_s + 1e-5, resp_id(LEFT, 2), "11" * 8),
    ]
    lines += bcast_tick(1.044) + two_hand_estop(1.048, with_echoes=False)
    b = bcast_overlap(lines, tmp_path).broadcast
    assert b.cmd_in_burst == in_burst
    assert b.late_resp["left"] == late
    assert b.one_hand_missing == late
    if in_burst:
        assert _broadcast_check(b, TH).verdict == FAIL


def test_estop_before_the_left_resp_is_cut_not_judged(tmp_path):
    tick = bcast_tick(1.040)  # REQ R0 R1 R2 L0 L1 L2
    lines = bcast_lines(10, estop_end=False) + tick[:4]
    lines += two_hand_estop(1.0409, with_echoes=False)
    lines += [line(1.0412 + 1e-5 * i, resp_id(LEFT, i), "11" * 8) for i in range(3)]
    o = bcast_overlap(lines, tmp_path)
    b = o.broadcast
    assert o.estop_line is not None
    assert (b.windows, b.truncated, b.cmd_in_burst, b.one_hand_missing) == (10, 1, 0, 0)
    assert o.violations == []
    assert _broadcast_check(b, TH).verdict == PASS


def test_left_zero_writes_with_broadcast_reqs_are_not_estops(tmp_path):
    # Every tick writes zeros (F0 F1 | REQ(0xFF) | F0 F1 ...) and no E-Stop follows.
    lines: list[str] = []
    for i in range(10):
        lines += bcast_tick(1.0 + i * 0.004, write=True, data_hex=ZERO44)
    frames = frames_of(lines, tmp_path)
    left = analyse_bus(frames, LEFT, broadcast_req=True)
    assert left.estop_candidates == 0
    assert left.estop is None
    assert left.write_pairs == 10
    o = analyse_overlap(frames, "broadcast")
    assert o.estop_line is None
    assert o.estop_f1_t == {}
    # Read per hand, the same bus has no REQ between the pairs: E-Stop look-alikes.
    assert analyse_bus(frames, LEFT).estop_candidates == 9


def test_per_hand_two_hand_run_is_judged_as_before(tmp_path, capsys):
    session = write_two_hand_session(tmp_path, two_hand_lines(echo=False), None)
    for side in ("right", "left"):
        rc_auto, auto = run_main(tmp_path, session, side)
        text = capsys.readouterr().out
        rc_explicit, explicit = run_main(tmp_path, session, side, "--req-mode", "per_hand")
        capsys.readouterr()
        assert rc_auto == rc_explicit == 0
        assert auto == explicit
        assert auto["req_mode"] == "per_hand"
        assert "| `x.no_overlap` | PASS |" in text
        assert "x.broadcast_sets" not in text
        overlap = dict(auto["overlap"])
        assert overlap.pop("estop_gap_ms") == pytest.approx(0.2, abs=1e-6)
        assert overlap.pop("estop_line") is not None
        assert overlap == {
            "hands_on_bus": ["right", "left"],
            "transactions": {"right": 20, "left": 20},
            "violations": 0,
            "by_pair": {},
            "first_violations": [],
        }


def test_req_mode_resolution():
    assert resolve_req_mode("auto", comm_stats()) == "per_hand"  # archive without the key
    comm = comm_stats()
    comm["canfd"]["req_mode"] = "broadcast"
    assert resolve_req_mode("auto", comm) == "broadcast"
    assert resolve_req_mode("broadcast", comm) == "broadcast"
    with pytest.raises(ValueError, match="req_mode mismatch"):
        resolve_req_mode("per_hand", comm)
    comm["canfd"]["req_mode"] = "round_robin"
    with pytest.raises(ValueError):
        resolve_req_mode("auto", comm)


def test_per_hand_mode_on_a_broadcast_bus_is_exit_3(tmp_path, capsys):
    # An archive without canfd.req_mode reads as per_hand: the broadcast bus contradicts it.
    session = write_two_hand_session(tmp_path, bcast_lines(5), None)
    rc, payload = run_main(tmp_path, session, "right")
    assert rc == 3
    assert payload is None
    assert "req_mode mismatch" in capsys.readouterr().err
    # Told it is broadcast, the same archive is judged.
    rc, payload = run_main(tmp_path, session, "right", "--req-mode", "broadcast")
    assert rc != 3
    assert payload["req_mode"] == "broadcast"


def test_broadcast_mode_on_a_per_hand_bus_is_exit_3(tmp_path, capsys):
    session = write_two_hand_session(tmp_path, two_hand_lines(n_ticks=5, echo=False), None)
    rc, payload = run_main(tmp_path, session, "left", "--req-mode", "broadcast")
    assert rc == 3
    assert payload is None
    assert "req_mode mismatch" in capsys.readouterr().err
    frames = frames_of(two_hand_lines(n_ticks=5), tmp_path)
    assert req_mode_mismatch(frames, "per_hand") is None
