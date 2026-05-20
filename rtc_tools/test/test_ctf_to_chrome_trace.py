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


def test_thread_name_picked_up_from_sched_switch_comm():
    # UST-only thread (no callback) — name must still appear via sched_switch.
    events = [
        _sched_switch(1_000_000, cpu=3, prev_tid=0, next_tid=200, next_comm="hand_udp_drv"),
        _sched_switch(2_000_000, cpu=3, prev_tid=200, next_tid=0, prev_comm="hand_udp_drv"),
    ]
    out = build_trace(iter(events))
    thread_meta = [
        ev
        for ev in out["traceEvents"]
        if ev["ph"] == "M" and ev["name"] == "thread_name" and ev["pid"] == THREAD_PID
    ]
    labels = {ev["tid"]: ev["args"]["name"] for ev in thread_meta}
    assert labels[200] == "hand_udp_drv-200"


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


def test_swimlane_groups_labelled():
    # The two process_name metadata records must label "Threads" and "Cpus".
    out = build_trace(iter(_callback_pair(0, 1000, vtid=1, callback=0x1, symbol="x")))
    proc_meta = {
        ev["pid"]: ev["args"]["name"]
        for ev in out["traceEvents"]
        if ev["ph"] == "M" and ev["name"] == "process_name"
    }
    assert proc_meta[THREAD_PID] == "Threads (by TID)"
    assert proc_meta[CPU_PID] == "Cpus"
