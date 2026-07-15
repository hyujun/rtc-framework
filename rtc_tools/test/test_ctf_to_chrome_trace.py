"""Tests for rtc_tools.conversion.ctf_to_chrome_trace.build_trace.

CTF reader 자체 (bt2 binding / babeltrace2 CLI) 는 외부 의존성이라 mocking
대신 build_trace 에 직접 ``(ts_ns, name, payload, cpu_id)`` 튜플 iterator 를
주입해 변환 로직만 검증한다.

검증 대상:
- 구조화 이벤트 (callback B/E, sched_switch, irq_handler_*) 는 항상 출력
- Unknown event 는 기본 drop, ``_dropped_event_counts`` 에 집계
- ``keep_events`` 화이트리스트로 특정 unknown event 만 복구 가능
- ``keep_all=True`` 는 legacy 동작 (모든 unknown event → instant marker)
- Thread label = ``"<procname>-<vtid>"`` 로 등록 (callback path + sched_switch
  의 prev/next_comm 모두에서 잡힘)
- Cpu lane = sched_switch 의 cpu_id 별로 분리, idle (next_tid==0) skip
"""

from __future__ import annotations

from rtc_tools.conversion.ctf_to_chrome_trace import (
    CPU_PID,
    IRQ_PID,
    STRUCTURED_EVENTS,
    THREAD_PID,
    _parse_bt2_cli,
    _report_capture_gaps,
    build_trace,
)

# ─────────────────────────────────────────────────────────────────────────────
# Fixtures — synthetic event streams matching the bt2 generator's tuple shape.
# ─────────────────────────────────────────────────────────────────────────────


def _callback_pair(ts_start_ns, ts_end_ns, *, vtid, callback, symbol, procname="rt_control"):
    return [
        (
            ts_start_ns,
            "ros2:callback_start",
            {"vtid": vtid, "procname": procname, "callback": callback, "symbol": symbol},
            None,
        ),
        (
            ts_end_ns,
            "ros2:callback_end",
            {"vtid": vtid, "procname": procname, "callback": callback},
            None,
        ),
    ]


def _sched_switch(ts_ns, *, cpu, prev_tid, next_tid, prev_comm="", next_comm=""):
    return (
        ts_ns,
        "sched_switch",
        {
            "prev_tid": prev_tid,
            "next_tid": next_tid,
            "prev_comm": prev_comm,
            "next_comm": next_comm,
        },
        cpu,
    )


def _irq_pair(ts_start_ns, ts_end_ns, *, cpu, irq, name):
    return [
        (ts_start_ns, "irq_handler_entry", {"irq": irq, "name": name}, cpu),
        (ts_end_ns, "irq_handler_exit", {"irq": irq}, cpu),
    ]


# ─────────────────────────────────────────────────────────────────────────────
# Structured-event coverage
# ─────────────────────────────────────────────────────────────────────────────


def test_callback_pair_emits_b_and_e_on_thread_lane():
    events = _callback_pair(1_000_000, 1_500_000, vtid=42, callback=0xABCD, symbol="WBC::tick")
    out = build_trace(iter(events))
    slices = [ev for ev in out["traceEvents"] if ev.get("cat") == "callback"]
    assert [ev["ph"] for ev in slices] == ["B", "E"]
    b = slices[0]
    assert b["pid"] == THREAD_PID
    assert b["tid"] == 42
    assert b["name"] == "WBC::tick"
    assert b["args"]["callback"] == "0xabcd"


def test_sched_switch_emits_b_e_on_cpu_lane_and_skips_idle():
    events = [
        _sched_switch(1_000_000, cpu=2, prev_tid=0, next_tid=100, next_comm="rt_control"),
        _sched_switch(2_000_000, cpu=2, prev_tid=100, next_tid=0, prev_comm="rt_control"),
    ]
    out = build_trace(iter(events))
    run_slices = [ev for ev in out["traceEvents"] if ev.get("cat") == "run"]
    # B for next_tid=100 at t=0us, E for prev_tid at t=1000us when next_tid=0.
    # The idle next_tid=0 does NOT open a new B (idle is skipped).
    phases = [ev["ph"] for ev in run_slices]
    assert phases == ["B", "E"]
    assert run_slices[0]["pid"] == CPU_PID
    assert run_slices[0]["tid"] == 2
    assert run_slices[0]["name"] == "rt_control-100"


def test_irq_pair_emits_b_e_on_irq_lane():
    events = _irq_pair(1_000_000, 1_010_000, cpu=4, irq=42, name="net0")
    out = build_trace(iter(events))
    irq_slices = [ev for ev in out["traceEvents"] if ev.get("cat") == "irq"]
    assert [ev["ph"] for ev in irq_slices] == ["B", "E"]
    assert irq_slices[0]["pid"] == IRQ_PID
    assert irq_slices[0]["tid"] == 4


def _rtc_span(ts_begin_ns, ts_end_ns, *, vtid, name, procname="integrated_rt_c"):
    return [
        (ts_begin_ns, "rtc:span_begin", {"vtid": vtid, "procname": procname, "name": name}, 2),
        (ts_end_ns, "rtc:span_end", {"vtid": vtid}, 2),
    ]


def test_rtc_span_emits_b_e_on_thread_lane_with_name():
    events = _rtc_span(1_000_000, 1_500_000, vtid=55, name="DemoWbcController::ComputeControl")
    out = build_trace(iter(events))
    spans = [ev for ev in out["traceEvents"] if ev.get("cat") == "rtc"]
    assert [ev["ph"] for ev in spans] == ["B", "E"]
    b = spans[0]
    assert b["pid"] == THREAD_PID
    assert b["tid"] == 55
    assert b["name"] == "DemoWbcController::ComputeControl"


def test_rtc_spans_nest_in_stack_order():
    # RT tick → CM::Compute → ComputeControl → sub-step, closed inside-out.
    # Perfetto reconstructs the flame stack from B/E order on the tid lane, so
    # the converter must preserve emission order without reordering.
    labels = [
        "rt_control_tick",
        "CM::Compute",
        "DemoWbcController::ComputeControl",
        "ComputeKinematicWbc",
    ]
    events = []
    for i, lbl in enumerate(labels):  # nested begins
        events.append((1_000_000 + i * 1000, "rtc:span_begin", {"vtid": 9, "name": lbl}, 2))
    for i in range(len(labels)):  # ends, inside-out
        events.append((2_000_000 + i * 1000, "rtc:span_end", {"vtid": 9}, 2))
    out = build_trace(iter(events))
    spans = [ev for ev in out["traceEvents"] if ev.get("cat") == "rtc"]
    begins = [ev["name"] for ev in spans if ev["ph"] == "B"]
    assert begins == labels
    assert sum(1 for ev in spans if ev["ph"] == "E") == len(labels)


def test_rtc_span_survives_empty_keep():
    events = _rtc_span(1_000_000, 1_500_000, vtid=1, name="x")
    out = build_trace(iter(events), keep_events=frozenset())
    assert {ev.get("cat") for ev in out["traceEvents"]} & {"rtc"}
    assert out["_dropped_event_counts"] == {}


# ─────────────────────────────────────────────────────────────────────────────
# Drop policy
# ─────────────────────────────────────────────────────────────────────────────


def test_unknown_events_dropped_by_default():
    events = [
        (1_000_000, "ros2:rclcpp_publish", {"vtid": 10, "procname": "mpc_main"}, None),
        (1_100_000, "sched_wakeup", {"vtid": 10}, None),
        (1_200_000, "ros2:rcl_init", {}, None),
    ]
    out = build_trace(iter(events))
    # Only metadata records should remain — no instant markers.
    instants = [ev for ev in out["traceEvents"] if ev.get("ph") == "i"]
    assert instants == []
    assert out["_dropped_event_counts"] == {
        "ros2:rclcpp_publish": 1,
        "sched_wakeup": 1,
        "ros2:rcl_init": 1,
    }


def test_keep_events_whitelist_restores_specific_names():
    events = [
        (1_000_000, "ros2:rclcpp_publish", {"vtid": 10, "procname": "mpc_main"}, None),
        (1_100_000, "sched_wakeup", {"vtid": 10}, None),
    ]
    out = build_trace(iter(events), keep_events=frozenset({"ros2:rclcpp_publish"}))
    instants = [ev for ev in out["traceEvents"] if ev.get("ph") == "i"]
    assert len(instants) == 1
    assert instants[0]["name"] == "ros2:rclcpp_publish"
    # sched_wakeup still dropped.
    assert out["_dropped_event_counts"] == {"sched_wakeup": 1}


def test_keep_all_emits_every_non_structured_event():
    events = [
        (1_000_000, "ros2:rclcpp_publish", {"vtid": 10, "procname": "mpc_main"}, None),
        (1_100_000, "sched_wakeup", {"vtid": 10}, None),
    ]
    out = build_trace(iter(events), keep_all=True)
    instants = [ev for ev in out["traceEvents"] if ev.get("ph") == "i"]
    assert {ev["name"] for ev in instants} == {"ros2:rclcpp_publish", "sched_wakeup"}
    assert out["_dropped_event_counts"] == {}


def test_structured_events_never_dropped_even_with_empty_keep():
    # Callback / sched_switch / IRQ must survive regardless of keep_events.
    events = _callback_pair(1_000_000, 1_500_000, vtid=1, callback=0x1, symbol="x")
    events += [_sched_switch(2_000_000, cpu=0, prev_tid=0, next_tid=1, next_comm="x")]
    events += _irq_pair(3_000_000, 3_010_000, cpu=0, irq=1, name="x")
    out = build_trace(iter(events), keep_events=frozenset())
    cats = {ev.get("cat") for ev in out["traceEvents"]}
    assert {"callback", "run", "irq"}.issubset(cats)
    assert out["_dropped_event_counts"] == {}


def test_structured_events_set_matches_documented_list():
    # Guards against accidental removal of an event from the protected set.
    assert (
        frozenset(
            {
                "ros2:callback_start",
                "ros2:callback_end",
                "sched_switch",
                "irq_handler_entry",
                "irq_handler_exit",
                "rtc:span_begin",
                "rtc:span_end",
            }
        )
        == STRUCTURED_EVENTS
    )


# ─────────────────────────────────────────────────────────────────────────────
# Thread / CPU labelling
# ─────────────────────────────────────────────────────────────────────────────


def test_thread_name_picked_up_from_callback_payload():
    events = _callback_pair(
        1_000_000, 1_500_000, vtid=77, callback=0x1, symbol="x", procname="mpc_main"
    )
    out = build_trace(iter(events))
    thread_meta = [
        ev
        for ev in out["traceEvents"]
        if ev["ph"] == "M" and ev["name"] == "thread_name" and ev["pid"] == THREAD_PID
    ]
    labels = {ev["tid"]: ev["args"]["name"] for ev in thread_meta}
    assert labels[77] == "mpc_main-77"


def test_sched_only_threads_get_no_thread_row_but_keep_cpu_label():
    # Spec change (workspace-focus display): a thread visible only through
    # sched_switch used to get an (empty) row in the flat Threads group —
    # one per system thread, drowning the workspace threads. It now gets no
    # thread row at all; its activity stays visible on the Cpu lane, where
    # the slice keeps the comm-derived label.
    events = [
        _sched_switch(1_000_000, cpu=3, prev_tid=0, next_tid=200, next_comm="hand_udp_drv"),
        _sched_switch(2_000_000, cpu=3, prev_tid=200, next_tid=0, prev_comm="hand_udp_drv"),
    ]
    out = build_trace(iter(events))
    thread_meta = [
        ev
        for ev in out["traceEvents"]
        if ev["ph"] == "M" and ev["name"] == "thread_name" and ev["pid"] not in (CPU_PID, IRQ_PID)
    ]
    assert thread_meta == []
    run_b = [ev for ev in out["traceEvents"] if ev.get("cat") == "run" and ev["ph"] == "B"]
    assert run_b[0]["name"] == "hand_udp_drv-200"


def test_sched_switch_comm_overrides_stale_ust_procname():
    # pthread_setname_np scenario: a thread first emits UST events
    # while its comm is still inherited from the main thread, then later
    # sched_switch shows the renamed comm. Latest sched_switch wins.
    events = [
        # UST event emitted before pthread_setname_np — procname is parent.
        (
            1_000_000,
            "ros2:callback_start",
            {
                "vtid": 27798,
                "procname": "integrated_rt_c",
                "callback": 0x1,
                "symbol": "boot",
            },
            None,
        ),
        (
            1_010_000,
            "ros2:callback_end",
            {"vtid": 27798, "procname": "integrated_rt_c", "callback": 0x1},
            None,
        ),
        # After pthread_setname_np("rt_callback"), kernel reflects new comm.
        _sched_switch(2_000_000, cpu=1, prev_tid=0, next_tid=27798, next_comm="rt_callback"),
        _sched_switch(3_000_000, cpu=1, prev_tid=27798, next_tid=0, prev_comm="rt_callback"),
    ]
    out = build_trace(iter(events))
    thread_meta = [
        ev
        for ev in out["traceEvents"]
        if ev["ph"] == "M" and ev["name"] == "thread_name" and ev["pid"] == THREAD_PID
    ]
    labels = {ev["tid"]: ev["args"]["name"] for ev in thread_meta}
    # 27798 (int) must resolve to the renamed comm, not the stale parent.
    assert labels[27798] == "rt_callback-27798"
    # The stale "integrated_rt_c-27798" label must not coexist as a
    # duplicate string-keyed lane.
    str_keyed = [ev for ev in thread_meta if isinstance(ev["tid"], str)]
    assert str_keyed == []


def test_callback_tid_is_int_not_string():
    # Perfetto tolerates string tid but our metadata loop keys by int —
    # if vtid sneaks through as a string the lane label and the slice
    # silently land on separate tracks. Guard against the regression.
    events = _callback_pair(
        1_000_000, 1_500_000, vtid=42, callback=0x1, symbol="x", procname="rt_control"
    )
    out = build_trace(iter(events))
    for ev in out["traceEvents"]:
        if ev.get("cat") == "callback":
            assert isinstance(ev["tid"], int), f"callback tid not int: {ev!r}"


def test_cpu_lanes_emitted_for_every_cpu_seen():
    events = [
        _sched_switch(1_000_000, cpu=0, prev_tid=0, next_tid=1, next_comm="a"),
        _sched_switch(2_000_000, cpu=5, prev_tid=0, next_tid=2, next_comm="b"),
        _sched_switch(3_000_000, cpu=11, prev_tid=0, next_tid=3, next_comm="c"),
    ]
    out = build_trace(iter(events))
    cpu_meta = [
        ev
        for ev in out["traceEvents"]
        if ev["ph"] == "M" and ev["name"] == "thread_name" and ev["pid"] == CPU_PID
    ]
    cpus = {ev["tid"] for ev in cpu_meta}
    assert cpus == {0, 5, 11}


def _process_names(out):
    return {
        ev["pid"]: ev["args"]["name"]
        for ev in out["traceEvents"]
        if ev["ph"] == "M" and ev["name"] == "process_name"
    }


def test_swimlane_groups_labelled():
    # Context-less capture (no vpid) falls back to the flat thread group.
    out = build_trace(iter(_callback_pair(0, 1000, vtid=1, callback=0x1, symbol="x")))
    assert _process_names(out)[THREAD_PID] == "Threads (by TID)"


def test_cpus_group_only_advertised_when_sched_switch_present():
    # Spec change: the Cpus group used to be emitted unconditionally, so a
    # capture without kernel events rendered an empty "Cpus" group — visually
    # identical to a converter bug, while the real cause is capture-side.
    # No sched_switch => no group at all; the census warning explains why.
    out = build_trace(iter(_callback_pair(0, 1000, vtid=1, callback=0x1, symbol="x")))
    assert CPU_PID not in _process_names(out)

    out = build_trace(
        iter([_sched_switch(1_000_000, cpu=2, prev_tid=0, next_tid=100, next_comm="rt_control")])
    )
    assert _process_names(out)[CPU_PID] == "Cpus"


# ─────────────────────────────────────────────────────────────────────────────
# babeltrace2 CLI fallback parser (_parse_bt2_cli) — text-line fixtures
# ─────────────────────────────────────────────────────────────────────────────

_UST_LINE = (
    "[14:30:42.123456789] (+0.000002112) myhost ros2:callback_start: "
    "{ cpu_id = 4 }, { vpid = 123, vtid = 124 }, "
    "{ callback = 0x7F12, is_intra_process = 0 }"
)
_SCHED_LINE = (
    "[14:30:42.223456789] (+0.100000000) myhost sched_switch: "
    '{ cpu_id = 2 }, { prev_comm = "rt_control", prev_tid = 100, '
    "prev_prio = 20, prev_state = 0, "
    'next_comm = "swapper/2", next_tid = 0, next_prio = 20 }'
)
_IRQ_LINE = (
    "[14:30:42.323456789] (+0.100000000) myhost irq_handler_entry: "
    '{ cpu_id = 2 }, { irq = 27, name = "eth0" }'
)


def test_cli_parser_parses_ust_line():
    events = list(_parse_bt2_cli([_UST_LINE]))
    assert len(events) == 1
    _ts, name, payload, cpu_id = events[0]
    assert name == "ros2:callback_start"
    assert payload["vtid"] == 124
    assert payload["callback"] == 0x7F12
    assert cpu_id == 4


def test_cli_parser_parses_kernel_sched_switch_line():
    # Regression guard: kernel events carry no "provider:" prefix — a
    # colon-mandatory name pattern silently dropped every sched_switch,
    # leaving the Cpus swimlane empty on CLI-fallback hosts.
    events = list(_parse_bt2_cli([_SCHED_LINE]))
    assert len(events) == 1
    _ts, name, payload, cpu_id = events[0]
    assert name == "sched_switch"
    assert payload["prev_comm"] == "rt_control"
    assert payload["prev_tid"] == 100
    assert payload["next_comm"] == "swapper/2"
    assert payload["next_tid"] == 0
    assert cpu_id == 2


def test_cli_parser_parses_irq_line():
    events = list(_parse_bt2_cli([_IRQ_LINE]))
    assert len(events) == 1
    _ts, name, payload, cpu_id = events[0]
    assert name == "irq_handler_entry"
    assert payload["irq"] == 27
    assert payload["name"] == "eth0"
    assert cpu_id == 2


def test_cli_parser_skips_non_event_lines():
    lines = [
        "",
        "[warning] Some babeltrace2 diagnostic without an event",
        _UST_LINE,
    ]
    events = list(_parse_bt2_cli(lines))
    assert [name for _ts, name, _p, _c in events] == ["ros2:callback_start"]


def test_cli_parser_handles_missing_hostname():
    line = (
        "[14:30:42.123456789] (+0.000002112) sched_switch: "
        '{ cpu_id = 0 }, { prev_comm = "a", prev_tid = 1, '
        'next_comm = "b", next_tid = 2 }'
    )
    events = list(_parse_bt2_cli([line]))
    assert len(events) == 1
    assert events[0][1] == "sched_switch"


def test_cli_parsed_kernel_lines_reach_cpu_lane():
    # End-to-end for the fallback path: text lines → parser → build_trace
    # must yield Cpu-lane run slices and IRQ-lane slices.
    out = build_trace(_parse_bt2_cli([_SCHED_LINE, _IRQ_LINE]))
    cats = {ev.get("cat") for ev in out["traceEvents"]}
    assert "irq" in cats
    # next_tid == 0 (idle) closes the lane without opening a slice; feed a
    # non-idle switch to observe a "run" B slice.
    sched_busy = _SCHED_LINE.replace(
        'next_comm = "swapper/2", next_tid = 0', 'next_comm = "rt_control", next_tid = 100'
    )
    out = build_trace(_parse_bt2_cli([sched_busy]))
    run_b = [ev for ev in out["traceEvents"] if ev.get("cat") == "run" and ev["ph"] == "B"]
    assert len(run_b) == 1
    assert run_b[0]["pid"] == CPU_PID
    assert run_b[0]["tid"] == 2


# ─────────────────────────────────────────────────────────────────────────────
# Callback symbol resolution via ros2:rclcpp_callback_register
# ─────────────────────────────────────────────────────────────────────────────


def _callback_register(ts_ns, *, callback, symbol, vtid=1, procname="rt_control"):
    return (
        ts_ns,
        "ros2:rclcpp_callback_register",
        {"vtid": vtid, "procname": procname, "callback": callback, "symbol": symbol},
        None,
    )


def _callback_pair_no_symbol(ts_start_ns, ts_end_ns, *, vtid, callback):
    # Real ros2:callback_start payloads carry only the address (+
    # is_intra_process) — no symbol field. Mirrors tracetools tp_call.h.
    return [
        (ts_start_ns, "ros2:callback_start", {"vtid": vtid, "callback": callback}, None),
        (ts_end_ns, "ros2:callback_end", {"vtid": vtid, "callback": callback}, None),
    ]


def test_callback_symbol_resolved_from_register_event():
    events = [_callback_register(500_000, callback=0xABCD, symbol="WbcController::Tick()")]
    events += _callback_pair_no_symbol(1_000_000, 1_500_000, vtid=42, callback=0xABCD)
    out = build_trace(iter(events))
    b = [ev for ev in out["traceEvents"] if ev.get("cat") == "callback" and ev["ph"] == "B"]
    assert len(b) == 1
    assert b[0]["name"] == "WbcController::Tick()"
    assert b[0]["args"]["symbol"] == "WbcController::Tick()"


def test_callback_symbol_falls_back_to_address_without_register():
    events = _callback_pair_no_symbol(1_000_000, 1_500_000, vtid=42, callback=0xABCD)
    out = build_trace(iter(events))
    b = [ev for ev in out["traceEvents"] if ev.get("cat") == "callback" and ev["ph"] == "B"]
    assert b[0]["name"] == "callback@0xabcd"


def test_register_event_not_counted_as_dropped():
    events = [_callback_register(500_000, callback=0x1, symbol="x")]
    out = build_trace(iter(events))
    assert out["_dropped_event_counts"] == {}
    # Consumed silently: no slice, no instant marker in default mode.
    assert [ev for ev in out["traceEvents"] if ev["ph"] != "M"] == []


def test_register_event_emitted_as_marker_under_keep_all():
    events = [_callback_register(500_000, callback=0xABCD, symbol="x")]
    events += _callback_pair_no_symbol(1_000_000, 1_500_000, vtid=42, callback=0xABCD)
    out = build_trace(iter(events), keep_all=True)
    instants = [ev for ev in out["traceEvents"] if ev["ph"] == "i"]
    assert {ev["name"] for ev in instants} == {"ros2:rclcpp_callback_register"}
    # Consumption and marker emission are not mutually exclusive: the
    # symbol map must still label the callback slice.
    b = [ev for ev in out["traceEvents"] if ev.get("cat") == "callback" and ev["ph"] == "B"]
    assert b[0]["name"] == "x"


# ─────────────────────────────────────────────────────────────────────────────
# Focus tiering — real-pid process groups, external-process summarization
# ─────────────────────────────────────────────────────────────────────────────


def _callback_pair_pid(ts_start_ns, ts_end_ns, *, vpid, vtid, procname, callback=0x1, symbol="cb"):
    return [
        (
            ts_start_ns,
            "ros2:callback_start",
            {
                "vpid": vpid,
                "vtid": vtid,
                "procname": procname,
                "callback": callback,
                "symbol": symbol,
            },
            None,
        ),
        (
            ts_end_ns,
            "ros2:callback_end",
            {"vpid": vpid, "vtid": vtid, "procname": procname, "callback": callback},
            None,
        ),
    ]


def _rtc_span_pid(ts_begin_ns, ts_end_ns, *, vpid, vtid, procname, name):
    return [
        (
            ts_begin_ns,
            "rtc:span_begin",
            {"vpid": vpid, "vtid": vtid, "procname": procname, "name": name},
            2,
        ),
        (ts_end_ns, "rtc:span_end", {"vpid": vpid, "vtid": vtid, "procname": procname}, 2),
    ]


def test_vpid_context_groups_slices_by_real_pid():
    events = _callback_pair_pid(1_000_000, 1_500_000, vpid=500, vtid=501, procname="mpc_main")
    out = build_trace(iter(events))
    slices = [ev for ev in out["traceEvents"] if ev.get("cat") == "callback"]
    assert {ev["pid"] for ev in slices} == {500}
    proc_meta = {
        ev["pid"]: ev["args"]["name"]
        for ev in out["traceEvents"]
        if ev["ph"] == "M" and ev["name"] == "process_name"
    }
    assert proc_meta[500] == "mpc_main"
    thread_meta = [
        ev
        for ev in out["traceEvents"]
        if ev["ph"] == "M" and ev["name"] == "thread_name" and ev["pid"] == 500
    ]
    assert [(ev["tid"], ev["args"]["name"]) for ev in thread_meta] == [(501, "mpc_main-501")]


def test_external_process_collapses_into_async_summary_lane():
    # pid 100 emits rtc spans (workspace, focus) — keeps sync B/E per-thread.
    # pid 200 emits only ros2 callbacks (external driver) — collapses into
    # async b/e slices on its process lane, no thread rows.
    events = _rtc_span_pid(
        1_000_000,
        1_400_000,
        vpid=100,
        vtid=100,
        procname="integrated_rt_c",
        name="rt_control_tick",
    )
    # Two ur_driver threads with time-overlapping callbacks — the stream is
    # time-ordered like a real trace, so the b/b/e/e interleaving is what
    # forces the async (id-matched) representation.
    pair_a = _callback_pair_pid(1_100_000, 1_600_000, vpid=200, vtid=201, procname="ur_driver")
    pair_b = _callback_pair_pid(
        1_200_000, 1_700_000, vpid=200, vtid=202, procname="ur_driver", callback=0x2
    )
    events += [pair_a[0], pair_b[0], pair_a[1], pair_b[1]]
    out = build_trace(iter(events))

    focus_slices = [ev for ev in out["traceEvents"] if ev.get("pid") == 100 and "cat" in ev]
    assert [ev["ph"] for ev in focus_slices] == ["B", "E"]

    ext = [ev for ev in out["traceEvents"] if ev.get("pid") == 200 and ev.get("cat") == "callback"]
    assert [ev["ph"] for ev in ext] == ["b", "b", "e", "e"]
    assert all("id" in ev and ev["name"] for ev in ext)
    # Each e must reuse its opening b's id (per-tid stacks, LIFO).
    b_ids = {ev["id"] for ev in ext if ev["ph"] == "b"}
    e_ids = {ev["id"] for ev in ext if ev["ph"] == "e"}
    assert b_ids == e_ids and len(b_ids) == 2

    proc_meta = {
        ev["pid"]: ev["args"]["name"]
        for ev in out["traceEvents"]
        if ev["ph"] == "M" and ev["name"] == "process_name"
    }
    assert proc_meta[100] == "integrated_rt_c"
    assert proc_meta[200] == "ur_driver (summary)"
    thread_meta_ext = [
        ev
        for ev in out["traceEvents"]
        if ev["ph"] == "M" and ev["name"] == "thread_name" and ev["pid"] == 200
    ]
    assert thread_meta_ext == []

    sort_index = {
        ev["pid"]: ev["args"]["sort_index"]
        for ev in out["traceEvents"]
        if ev["ph"] == "M" and ev["name"] == "process_sort_index"
    }
    # Workspace process above the Cpu lanes, summary group below them.
    assert sort_index[100] < sort_index[CPU_PID] < sort_index[200]


def test_orphan_end_in_summary_process_is_dropped():
    # Capture started mid-callback: an end with no begin must not survive
    # as an unmatched async "e".
    events = [
        (
            900_000,
            "ros2:callback_end",
            {"vpid": 200, "vtid": 201, "procname": "ur_driver", "callback": 0x9},
            None,
        ),
    ]
    events += _callback_pair_pid(1_000_000, 1_500_000, vpid=200, vtid=201, procname="ur_driver")
    events += _rtc_span_pid(
        1_000_000, 1_400_000, vpid=100, vtid=100, procname="integrated_rt_c", name="tick"
    )
    out = build_trace(iter(events))
    ext = [ev for ev in out["traceEvents"] if ev.get("pid") == 200 and ev.get("cat") == "callback"]
    assert [ev["ph"] for ev in ext] == ["b", "e"]


def test_no_spans_falls_back_to_per_thread_detail():
    # Capture from a build without RTC_ENABLE_TRACING: no rtc:* spans, so
    # there is no signal to classify — everything stays per-thread.
    events = _callback_pair_pid(1_000_000, 1_500_000, vpid=200, vtid=201, procname="ur_driver")
    out = build_trace(iter(events))
    ext = [ev for ev in out["traceEvents"] if ev.get("pid") == 200 and ev.get("cat") == "callback"]
    assert [ev["ph"] for ev in ext] == ["B", "E"]
    proc_meta = {
        ev["pid"]: ev["args"]["name"]
        for ev in out["traceEvents"]
        if ev["ph"] == "M" and ev["name"] == "process_name"
    }
    assert proc_meta[200] == "ur_driver"


def test_focus_proc_substring_promotes_process_without_spans():
    events = _callback_pair_pid(1_000_000, 1_500_000, vpid=200, vtid=201, procname="ur_driver")
    events += _callback_pair_pid(1_100_000, 1_600_000, vpid=300, vtid=301, procname="foo_node")
    out = build_trace(iter(events), focus_procs=frozenset({"foo"}))
    foo = [ev for ev in out["traceEvents"] if ev.get("pid") == 300 and ev.get("cat") == "callback"]
    assert [ev["ph"] for ev in foo] == ["B", "E"]
    ur = [ev for ev in out["traceEvents"] if ev.get("pid") == 200 and ev.get("cat") == "callback"]
    assert [ev["ph"] for ev in ur] == ["b", "e"]


def test_all_threads_disables_summarization():
    events = _rtc_span_pid(
        1_000_000, 1_400_000, vpid=100, vtid=100, procname="integrated_rt_c", name="tick"
    )
    events += _callback_pair_pid(1_100_000, 1_600_000, vpid=200, vtid=201, procname="ur_driver")
    out = build_trace(iter(events), all_threads=True)
    ext = [ev for ev in out["traceEvents"] if ev.get("pid") == 200 and ev.get("cat") == "callback"]
    assert [ev["ph"] for ev in ext] == ["B", "E"]
    thread_meta_ext = [
        ev
        for ev in out["traceEvents"]
        if ev["ph"] == "M" and ev["name"] == "thread_name" and ev["pid"] == 200
    ]
    assert [(ev["tid"], ev["args"]["name"]) for ev in thread_meta_ext] == [(201, "ur_driver-201")]


def test_census_counts_input_events_and_vpid_coverage():
    events = _rtc_span_pid(
        1_000_000, 1_400_000, vpid=100, vtid=100, procname="integrated_rt_c", name="tick"
    )
    events += [_sched_switch(2_000_000, cpu=2, prev_tid=0, next_tid=100, next_comm="rt")]
    out = build_trace(iter(events))
    census = out["_census"]
    assert census["rtc:span_begin"] == 1
    assert census["sched_switch"] == 1
    assert census["_with_vpid"] == 2  # both span events carry vpid


def test_capture_gap_warnings_name_the_capture_side_cause():
    # A trace with callbacks but no spans and no sched_switch is exactly what
    # a non---tracing build + kernel-events-off capture produces. The tool
    # must say so, since both render as "the group is empty".
    lines = "\n".join(_report_capture_gaps({"ros2:callback_start": 5, "_with_vpid": 5}))
    assert "0 rtc:* spans" in lines
    assert "--tracing" in lines
    assert "0 sched_switch" in lines
    assert "tracing" in lines  # group-membership hint for kernel events

    # Healthy capture: census line only, no warnings.
    lines = _report_capture_gaps(
        {
            "rtc:span_begin": 10,
            "ros2:callback_start": 5,
            "sched_switch": 100,
            "_with_vpid": 15,
        }
    )
    assert len(lines) == 1
    assert "capture census" in lines[0]


def test_missing_vpid_context_warns_only_when_ust_present():
    assert any("no vpid context" in ln for ln in _report_capture_gaps({"ros2:callback_start": 5}))
    # Kernel-only capture has no UST events — the vpid note would be noise.
    assert not any("no vpid context" in ln for ln in _report_capture_gaps({"sched_switch": 10}))


def test_cpu_lane_label_gets_process_prefix():
    # A worker thread whose pid is known from the UST vpid context gets its
    # process name prefixed on the Cpu lane; the main thread (tid == pid)
    # keeps its bare comm — a "proc/proc-tid" prefix would be noise.
    events = _callback_pair_pid(
        1_000_000, 1_100_000, vpid=100, vtid=100, procname="integrated_rt_c"
    )
    events += _callback_pair_pid(
        1_200_000, 1_300_000, vpid=100, vtid=101, procname="integrated_rt_c"
    )
    events += [
        _sched_switch(2_000_000, cpu=2, prev_tid=0, next_tid=101, next_comm="dds_worker"),
        _sched_switch(2_500_000, cpu=2, prev_tid=101, next_tid=100, next_comm="integrated_rt_c"),
        _sched_switch(3_000_000, cpu=2, prev_tid=100, next_tid=0, prev_comm="integrated_rt_c"),
    ]
    out = build_trace(iter(events))
    run_b = [ev for ev in out["traceEvents"] if ev.get("cat") == "run" and ev["ph"] == "B"]
    labels = [ev["name"] for ev in run_b]
    assert labels == ["integrated_rt_c/dds_worker-101", "integrated_rt_c-100"]
