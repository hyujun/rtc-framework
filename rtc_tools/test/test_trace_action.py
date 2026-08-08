"""Tests for rtc_tools.launch.trace_action event assembly.

Two axes:

* the pure UST event-list helper that decides which UST events the LTTng
  session enables — the runtime gate that determines whether rtc:* spans get
  recorded at all;
* ``make_trace_action`` itself, driven with a real ``LaunchContext`` and a
  recording stand-in for ``tracetools_launch.action.Trace``.

The second axis exists because of issue #190. On kernels where no compatible
``lttng-modules`` build exists, ``trace_events_kernel:=`` (empty) is the *only*
way to capture anything at all, so that branch carries the whole UST-only
policy. The module docstring here used to claim the action was "exercised in
launch integration" — there was no such test, and the branch that the policy
rests on had never been executed by any sensor.
"""

import sys
import types

import pytest

from rtc_tools.launch.trace_action import (
    KERNEL_EVENTS_OFF,
    RTC_UST_EVENTS,
    _with_rtc_events,
    make_trace_action,
)


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


# ── make_trace_action: kernel-event wiring (issue #190) ─────────────────────


class _RecordingTrace:
    """Stand-in for tracetools_launch.action.Trace that captures its kwargs.

    Constructing the real Trace pulls in an LTTng session setup path we do not
    want in a unit test; what this test owns is the *kwargs* make_trace_action
    hands over, which is exactly where the UST-only decision is expressed.
    """

    last_kwargs: dict = {}

    def __init__(self, **kwargs):
        type(self).last_kwargs = dict(kwargs)


@pytest.fixture
def recording_trace(monkeypatch):
    """Make the lazy `from tracetools_launch.action import Trace` resolve to the recorder."""
    mod = types.ModuleType("tracetools_launch.action")
    mod.Trace = _RecordingTrace
    pkg = types.ModuleType("tracetools_launch")
    pkg.action = mod
    monkeypatch.setitem(sys.modules, "tracetools_launch", pkg)
    monkeypatch.setitem(sys.modules, "tracetools_launch.action", mod)
    _RecordingTrace.last_kwargs = {}
    return _RecordingTrace


def _context(**configs):
    from launch import LaunchContext

    ctx = LaunchContext()
    base = {
        "enable_tracing": "true",
        "trace_session_name": "",
        "trace_events_ust": "",
        "trace_events_kernel": "",
    }
    base.update(configs)
    for key, value in base.items():
        ctx.launch_configurations[key] = value
    return ctx


# The literal a user is told to type. Kept as a module constant so the assertion
# below is about *the documented invocation*, not about a value that only a
# launch-file default can produce — the distinction that let a broken command
# ship: the first version of this test asserted "" and passed, while
# `trace_events_kernel:=` (what docs, the installer warning and the RT check all
# instructed) was rejected by ros2launch before any of this code ran.
DOCUMENTED_KERNEL_OFF = "none"


def test_documented_off_token_yields_empty_event_list(recording_trace, tmp_path):
    """The token docs tell users to type must reach Trace as [] — the UST-only path.

    A falsy-but-absent value would let ros2_tracing fall back to its own kernel
    default, which needs the lttng-modules kernel module; on a host where that
    module cannot be built the session then fails to start instead of degrading
    to UST-only. This assertion is what pins issue #190's chosen policy.
    """
    ctx = _context(trace_events_kernel=DOCUMENTED_KERNEL_OFF)
    actions = make_trace_action(ctx, session_dir=str(tmp_path))

    assert len(actions) == 1
    assert recording_trace.last_kwargs["events_kernel"] == []
    # UST side must stay populated — UST-only means "no kernel events", not
    # "no events".
    assert set(RTC_UST_EVENTS) <= set(recording_trace.last_kwargs["events_ust"])


def test_documented_off_token_is_a_recognised_sentinel():
    """Guards the docs↔code seam: the documented token must be in the off set.

    Without this, renaming the sentinel in code leaves every doc, the installer
    warning and the check_rt_setup fix hint pointing at a value that silently
    becomes a bogus kernel-event name instead of disabling kernel tracing.
    """
    assert DOCUMENTED_KERNEL_OFF in KERNEL_EVENTS_OFF


def test_empty_value_still_works_from_launch_file_defaults(recording_trace, tmp_path):
    """Empty is unreachable from the CLI but remains valid as a launch-file default."""
    actions = make_trace_action(_context(trace_events_kernel=""), session_dir=str(tmp_path))

    assert len(actions) == 1
    assert recording_trace.last_kwargs["events_kernel"] == []


def test_off_sentinels_are_not_treated_as_event_names(recording_trace, tmp_path):
    """`none`/`off`/... must disable, not become an event named "none"."""
    for token in sorted(KERNEL_EVENTS_OFF) + ["NONE", " none "]:
        make_trace_action(_context(trace_events_kernel=token), session_dir=str(tmp_path))
        assert recording_trace.last_kwargs["events_kernel"] == [], token


def test_kernel_events_are_split_when_given(recording_trace, tmp_path):
    ctx = _context(trace_events_kernel="sched_switch, irq_handler_entry ,")
    make_trace_action(ctx, session_dir=str(tmp_path))

    assert recording_trace.last_kwargs["events_kernel"] == [
        "sched_switch",
        "irq_handler_entry",
    ]


def test_disabled_tracing_builds_no_action(recording_trace, tmp_path):
    actions = make_trace_action(_context(enable_tracing="false"), session_dir=str(tmp_path))

    assert actions == []
    assert recording_trace.last_kwargs == {}
