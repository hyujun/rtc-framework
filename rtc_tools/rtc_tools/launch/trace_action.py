"""trace_action.py — robot-agnostic launch helper that wraps ``ros2_tracing``.

Bringup packages declare their own LaunchArguments
(``enable_tracing`` / ``trace_session_name`` / ``trace_events_ust`` /
``trace_events_kernel``) and call :func:`make_trace_action` from inside
an ``OpaqueFunction`` once the session directory is known.

The helper:
    * imports what it needs from ros2_tracing lazily and under a single
      guard — ``tracetools_launch.action.Trace`` and
      ``tracetools_trace.tools.names.DEFAULT_EVENTS_ROS`` (so the launch file
      can be parsed even when ros2_tracing is not installed)
    * writes traces to ``<session_dir>/tracing/<session_name>/`` — the same
      session tree as the CSV timing logs, so trace and CSV cross-reference
      1:1 (not the ``~/.ros/tracing/`` ros2_tracing default). The
      session_name defaults to ``"trace"``; the surrounding session_dir
      already carries the YYMMDD_HHMM stamp.
    * is a no-op when ``enable_tracing:=false`` (returns ``[]``)
    * declines to act with a clear message when ros2_tracing is not on the
      Python path (so ``./install.sh --tracing`` is suggested instead of
      a cryptic ImportError)

Lifecycle: ``Trace`` registers an ``OnShutdown`` handler internally that
calls ``lttng.lttng_fini()`` to flush and close the session — no extra
cleanup needed on the caller side.
"""

from __future__ import annotations

import os

from launch.substitutions import LaunchConfiguration

# rtc_base's RTC_TRACE_SCOPE UST provider. Always enabled alongside the ros2:*
# events so RT-tick spans are captured whenever tracing is on. Harmless if the
# binary was built without -DRTC_ENABLE_TRACING (no provider registers these
# names → LTTng simply matches nothing, no error). See docs/tracing.md.
RTC_UST_EVENTS = ("rtc:span_begin", "rtc:span_end")


# Values that mean "no kernel events" (UST-only capture).
#
# The obvious spelling — an empty value — is **unreachable from the command
# line**: ros2launch rejects `name:=` outright (`ros2launch.api.api` raises
# "malformed launch argument" when the token ends with ':='), so
# `trace_events_kernel:=` can never be typed even though docs instructed it for
# a long while. An empty string still arrives from launch-file defaults, so both
# spellings have to work; only these sentinels are typable (issue #190).
KERNEL_EVENTS_OFF = frozenset({"none", "off", "false", "0"})


def _split_csv(value: str) -> list[str]:
    """Comma-separated string → stripped non-empty list."""
    return [item.strip() for item in value.split(",") if item.strip()]


def _kernel_events(raw: str) -> list[str]:
    """Kernel-event list from the raw launch argument, honouring the off sentinels."""
    if raw.strip().lower() in KERNEL_EVENTS_OFF:
        return []
    return _split_csv(raw) if raw else []


def _with_rtc_events(base: list[str]) -> list[str]:
    """Append the rtc:* span events to `base`, preserving order, de-duped."""
    out = list(base)
    for ev in RTC_UST_EVENTS:
        if ev not in out:
            out.append(ev)
    return out


def make_trace_action(
    context,
    *,
    session_dir: str,
    enable_arg: str = "enable_tracing",
    session_name_arg: str = "trace_session_name",
    events_ust_arg: str = "trace_events_ust",
    events_kernel_arg: str = "trace_events_kernel",
):
    """Build the ros2_tracing capture action list to extend onto a launch description.

    Args:
        context: Launch context (from inside an ``OpaqueFunction``).
        session_dir: Logging-data session directory (e.g.
            ``<ws>/logging_data/260520_1430``). Its basename becomes the
            default ``session_name`` so the LTTng trace lives at
            ``~/.ros/tracing/260520_1430/`` and lines up with the CSV
            timing logs under that session.
        enable_arg: LaunchArgument that gates the whole capture.
        session_name_arg: LaunchArgument carrying the LTTng session name
            (also becomes the leaf directory under ``<session_dir>/tracing/``).
            Empty = ``"trace"``.
        events_ust_arg: LaunchArgument carrying a comma-separated list of
            UST events to enable. Empty = ros2_tracing's DEFAULT_EVENTS_ROS
            (broad ros2:* coverage). The rtc:* span events (RTC_UST_EVENTS)
            are always appended regardless, so RT-tick spans are captured
            whenever tracing is on.
        events_kernel_arg: LaunchArgument carrying a comma-separated list
            of kernel events. ``none`` (or an empty value, which only a
            launch-file default can supply) disables kernel tracing — common
            choices are ``sched_switch,sched_waking,sched_wakeup,
            irq_handler_entry,irq_handler_exit``.

    Returns:
        ``list`` of launch actions (empty if disabled or ros2_tracing
        is not installed).
    """
    enabled = LaunchConfiguration(enable_arg).perform(context)
    if enabled.lower() not in ("true", "1", "yes"):
        return []

    # Lazy import — declining gracefully if ros2_tracing is not installed.
    #
    # Both modules ship in the ros2_tracing stack (`ros-<distro>-tracetools-launch`
    # depends on `tracetools-trace`), but they are imported under one guard rather
    # than assuming co-installation: a partial install — or a test that stubs only
    # one of them — otherwise escapes this branch and dies on a raw ImportError
    # deeper in, past the point where the graceful message could still be printed.
    try:
        from tracetools_launch.action import Trace
        from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS
    except ImportError:
        print(
            "[trace_action] ros2_tracing not installed — "
            "run ./install.sh --tracing to enable ros2_tracing capture. "
            "Skipping trace; launch will continue."
        )
        return []

    # session_name = LTTng session identifier. Default "trace" keeps the
    # directory short; the surrounding session_dir already carries the
    # YYMMDD_HHMM stamp, so the LTTng dir does not need to repeat it.
    session_name = LaunchConfiguration(session_name_arg).perform(context) or "trace"

    # base_path = <logging_data>/<YYMMDD_HHMM>/tracing → the CTF trace lands
    # at <session_dir>/tracing/<session_name>/ alongside CSV timing logs,
    # not in ~/.ros/tracing/. session_dir's "tracing" subdir is pre-created
    # by create_session_dir (_SESSION_SUBDIRS).
    base_path = os.path.join(session_dir, "tracing")

    trace_kwargs: dict = {"session_name": session_name, "base_path": base_path}

    # events_ust = (user list if given, else ros2_tracing's DEFAULT_EVENTS_ROS)
    # + rtc:* spans. We always pass an explicit list — the previous "omit kwarg
    # to inherit the default" path would silently drop rtc:* (the default set is
    # ros2:* only), so the RT-tick spans never got recorded.
    events_ust_raw = LaunchConfiguration(events_ust_arg).perform(context)
    base_ust = _split_csv(events_ust_raw) if events_ust_raw else list(DEFAULT_EVENTS_ROS)
    trace_kwargs["events_ust"] = _with_rtc_events(base_ust)

    events_kernel_raw = LaunchConfiguration(events_kernel_arg).perform(context)
    trace_kwargs["events_kernel"] = _kernel_events(events_kernel_raw)

    trace = Trace(**trace_kwargs)

    return [trace]
