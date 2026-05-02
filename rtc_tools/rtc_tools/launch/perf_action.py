"""perf_action.py — robot-agnostic launch helper that wraps ``perf record``.

Bringup packages declare their own ``enable_perf`` / ``perf_targets`` /
``perf_duration`` LaunchArguments and call :func:`make_perf_action` from inside
an ``OpaqueFunction`` once the session directory is known.

The helper:
    * resolves ``perf_record.sh`` from the installed ``repo_scripts`` share
    * writes ``<session_dir>/perf/perf.data``
    * is a no-op when ``enable_perf:=false`` (returns ``[]``)
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


def make_perf_action(
    context,
    *,
    session_dir: str,
    targets_arg: str = "perf_targets",
    duration_arg: str = "perf_duration",
    enable_arg: str = "enable_perf",
    start_delay_sec: float = 5.0,
    output_filename: str = "perf.data",
):
    """Build the perf-capture action list to extend onto a launch description.

    Args:
        context: Launch context (from inside an ``OpaqueFunction``).
        session_dir: Session directory (created with ``perf/`` subdir already).
        targets_arg: LaunchArgument name carrying a regex of process names
            to attach to (e.g. ``"ur5e_rt_controller|mpc_main"``).
        duration_arg: LaunchArgument name carrying capture duration in seconds.
            Empty/0 = run until launch SIGINT.
        enable_arg: LaunchArgument that gates the whole capture.
        start_delay_sec: Wait this long after launch start before invoking perf,
            so that lifecycle nodes have transitioned to ``active`` and RT
            threads have spawned.
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

    perf_proc = ExecuteProcess(
        cmd=cmd,
        output="screen",
        # Allow generous flush time on SIGINT so perf can finalise the trace.
        sigterm_timeout="15",
        sigkill_timeout="20",
    )

    return [TimerAction(period=start_delay_sec, actions=[perf_proc])]
