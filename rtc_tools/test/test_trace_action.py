"""Tests for rtc_tools.launch.trace_action UST-event assembly.

make_trace_action() needs a launch context, so the full action is exercised in
launch integration; here we unit-test the pure event-list helper that decides
which UST events the LTTng session enables — the runtime gate that determines
whether rtc:* spans get recorded at all.
"""

from rtc_tools.launch.trace_action import RTC_UST_EVENTS, _with_rtc_events


def test_rtc_events_appended_to_ros_defaults():
    base = ["ros2:callback_start", "ros2:callback_end"]
    out = _with_rtc_events(base)
    # ros2 events preserved, in order, up front.
    assert out[: len(base)] == base
    # rtc span events appended so the session records them.
    assert "rtc:span_begin" in out
    assert "rtc:span_end" in out


def test_rtc_events_not_duplicated():
    base = ["ros2:callback_start", "rtc:span_begin"]
    out = _with_rtc_events(base)
    assert out.count("rtc:span_begin") == 1
    assert out.count("rtc:span_end") == 1


def test_rtc_events_appended_to_empty_base():
    assert _with_rtc_events([]) == list(RTC_UST_EVENTS)


def test_rtc_ust_events_are_the_span_pair():
    # Guards against accidentally enabling the wrong provider names — these must
    # match the TRACEPOINT_EVENT names in rtc_base/tracing/rtc_tracepoints.hpp.
    assert set(RTC_UST_EVENTS) == {"rtc:span_begin", "rtc:span_end"}
