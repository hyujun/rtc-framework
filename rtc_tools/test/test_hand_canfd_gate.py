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
    analyse_bus,
    judge,
    load_comm_stats,
    load_timing_csv,
    main,
    overall,
    render_markdown,
    summarise_timing,
)
from rtc_tools.utils.session_dir import PRE_F08_HAND_STEMS

NODE = node_id_of("right")
TH = Thresholds()
PERIOD_US = 2000.0
LIMIT_US = 1600.0


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
    for probe, verdict in ((1599, PASS), (1600, FAIL)):
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
