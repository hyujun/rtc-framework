"""trace_action.py — robot-agnostic launch helper that wraps ``ros2_tracing``.

Bringup packages declare their own LaunchArguments
(``enable_tracing`` / ``trace_session_name`` / ``trace_events_ust`` /
``trace_events_kernel``) and call :func:`make_trace_action` from inside
an ``OpaqueFunction`` once the session directory is known.

The helper:
    * imports ``tracetools_launch.action.Trace`` lazily (so the launch file
      can be parsed even when ros2_tracing is not installed)
    * writes traces to ``~/.ros/tracing/<session_name>/`` — the
      ros2_tracing default. The session_name defaults to the logging_data
      session directory's basename so traces cross-reference cleanly with
      CSV timing logs.
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


def _split_csv(value: str) -> list[str]:
    """Comma-separated string → stripped non-empty list."""
    return [item.strip() for item in value.split(",") if item.strip()]


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
            (broad ros2:* coverage).
        events_kernel_arg: LaunchArgument carrying a comma-separated list
            of kernel events. Empty list disables kernel tracing — common
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
    try:
        from tracetools_launch.action import Trace
    except ImportError:
        print(
            "[trace_action] tracetools_launch not installed — "
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

    # Empty events_ust → omit the kwarg so tracetools_launch.Trace falls back
    # to its default ros2:* set. Passing events_ust=None explicitly crashes
    # Trace.__init__ with 'NoneType is not iterable' (Jazzy 8.2.5).
    trace_kwargs: dict = {"session_name": session_name, "base_path": base_path}

    events_ust_raw = LaunchConfiguration(events_ust_arg).perform(context)
    if events_ust_raw:
        trace_kwargs["events_ust"] = _split_csv(events_ust_raw)

    events_kernel_raw = LaunchConfiguration(events_kernel_arg).perform(context)
    trace_kwargs["events_kernel"] = _split_csv(events_kernel_raw) if events_kernel_raw else []

    trace = Trace(**trace_kwargs)

    return [trace]
