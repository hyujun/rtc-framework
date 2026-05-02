"""perf_action.py — robot-agnostic launch helper that wraps ``perf record``.

Bringup packages declare their own LaunchArguments
(``enable_perf`` / ``perf_targets`` / ``perf_duration`` /
``perf_start_delay`` / ``perf_stack_size`` / ``perf_frequency``)
and call :func:`make_perf_action` from inside an ``OpaqueFunction``
once the session directory is known.

The helper:
    * resolves ``perf_record.sh`` from the installed ``repo_scripts`` share
    * writes ``<session_dir>/perf/perf.data``
    * is a no-op when ``enable_perf:=false`` (returns ``[]``)
    * starts perf immediately when ``perf_start_delay:=0`` (no TimerAction wrap)
"""

from __future__ import annotations

import os

from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration


def _resolve_perf_record_script() -> str | None:
    """Return absolute path to ``perf_record.sh`` from the installed share, or None."""
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory("repo_scripts")
    except Exception:
        return None

    # repo_scripts CMakeLists installs scripts to lib/repo_scripts/, not share/.
    # share returns .../install/repo_scripts/share/repo_scripts; lib is sibling.
    candidates = [
        os.path.join(
            os.path.dirname(os.path.dirname(share)), "lib", "repo_scripts", "perf_record.sh"
        ),
        os.path.join(share, "scripts", "perf_record.sh"),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path
    return None


def _read_float_arg(context, name: str, default: float) -> float:
    """LaunchConfiguration → float, falling back on parse failure."""
    raw = LaunchConfiguration(name).perform(context)
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        print(f"[perf_action] {name}={raw!r} is not numeric — using {default}")
        return default


def make_perf_action(
    context,
    *,
    session_dir: str,
    targets_arg: str = "perf_targets",
    duration_arg: str = "perf_duration",
    enable_arg: str = "enable_perf",
    start_delay_arg: str = "perf_start_delay",
    stack_size_arg: str = "perf_stack_size",
    frequency_arg: str = "perf_frequency",
    event_arg: str = "perf_event",
    output_filename: str = "perf.data",
):
    """Build the perf-capture action list to extend onto a launch description.

    Args:
        context: Launch context (from inside an ``OpaqueFunction``).
        session_dir: Session directory (created with ``perf/`` subdir already).
        targets_arg: LaunchArgument name carrying a regex of process names
            to attach to (e.g. ``"my_rt_controller|my_worker"``).
        duration_arg: LaunchArgument name carrying capture duration in seconds.
            Empty/0 = run until launch SIGINT.
        enable_arg: LaunchArgument that gates the whole capture.
        start_delay_arg: LaunchArgument carrying seconds to wait before invoking
            perf. ``0`` = start immediately (default — captures lifecycle bring-up).
        stack_size_arg: LaunchArgument carrying DWARF unwind stack size in bytes.
            Smaller is faster to load in Hotspot at the cost of deeper truncation
            risk (4096 covers typical C++ stacks; 8192/16384 for deep templates).
        frequency_arg: LaunchArgument carrying perf sampling frequency in Hz.
        event_arg: LaunchArgument carrying the perf event spec (e.g. ``cycles:P``,
            ``task-clock``). cycles:P samples on CPU cycles (good for hot CPU
            users); task-clock samples on wall-clock CPU time (better for
            burst-then-sleep RT loops where cycles:P under-samples).
        output_filename: Filename written under ``<session_dir>/perf/``.

    Returns:
        ``list`` of launch actions (empty if disabled or unresolvable).
    """
    enabled = LaunchConfiguration(enable_arg).perform(context)
    if enabled.lower() not in ("true", "1", "yes"):
        return []

    targets = LaunchConfiguration(targets_arg).perform(context)
    if not targets:
        print(f"[perf_action] {targets_arg} is empty — perf capture disabled")
        return []

    script = _resolve_perf_record_script()
    if script is None:
        print(
            "[perf_action] repo_scripts/scripts/perf_record.sh not found — "
            "is repo_scripts built and installed? Skipping perf capture."
        )
        return []

    output_path = os.path.join(session_dir, "perf", output_filename)

    cmd = [script, output_path, targets]

    duration = LaunchConfiguration(duration_arg).perform(context)
    if duration and duration not in ("0", "0.0", ""):
        cmd.extend(["--duration", duration])

    frequency = LaunchConfiguration(frequency_arg).perform(context)
    if frequency:
        cmd.extend(["--frequency", frequency])

    stack_size = LaunchConfiguration(stack_size_arg).perform(context)
    if stack_size:
        cmd.extend(["--stack-size", stack_size])

    event = LaunchConfiguration(event_arg).perform(context)
    if event:
        cmd.extend(["--event", event])

    perf_proc = ExecuteProcess(
        cmd=cmd,
        output="screen",
        # Allow generous flush time on SIGINT so perf can finalise the trace.
        sigterm_timeout="15",
        sigkill_timeout="20",
    )

    start_delay = _read_float_arg(context, start_delay_arg, 0.0)
    if start_delay <= 0.0:
        return [perf_proc]
    return [TimerAction(period=start_delay, actions=[perf_proc])]
