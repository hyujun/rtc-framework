"""``demo_controller_sequence``: one hand's shipped sequence, looped headless.

    ros2 run integrated_bringup demo_controller_sequence --robot p2 --mode loop --duration 9h

It plays what git ships for the robot (``config/<robot>/demo_gui/`` of the
installed package), so a field PC needs only pull, build and this command.

Arguments and exit codes live here; the run is ``orchestrator``, its inputs
``files``, the ROS side ``ros_io``. Operator guide:
docs/demo-controller-sequence.md.

Exit codes: 0 done (duration over or Ctrl-C, at a cycle end) · 1 aborted ·
2 arguments or files · 3 the stack is not ready · 4 the final summary could not
be written · 130 stopped at once (Ctrl-C twice, SIGTERM, SIGHUP).
"""

from __future__ import annotations

import argparse
import os
import re
import sys

from . import env
from .files import FileProblem, load_run_inputs
from .gates import (
    DEFAULT_HOLD_ABORT_AFTER_S,
    MAX_HOLD_ABORT_AFTER_S,
    MIN_HOLD_ABORT_AFTER_S,
)
from .orchestrator import EXIT_USAGE

ROBOTS = ("p2", "p2_left")
MIN_DURATION_S = 60.0
MAX_DURATION_S = 10 * 3600.0
SHORT_DURATION_WARN_S = 8 * 3600.0
DEFAULT_STARTUP_TIMEOUT_S = 60.0

_DURATION = re.compile(r"^\s*(\d+(?:\.\d+)?)\s*([smh]?)\s*$")


def parse_duration(text: str) -> float:
    """'9h' · '90m' · '45s' · '3600' (seconds) → seconds."""
    match = _DURATION.match(text)
    if not match:
        raise argparse.ArgumentTypeError(f"'{text}' is not a duration (e.g. 9h, 90m, 600s)")
    value, unit = float(match.group(1)), match.group(2) or "s"
    return value * {"s": 1.0, "m": 60.0, "h": 3600.0}[unit]


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="demo_controller_sequence",
        description="Loop one hand's saved posture sequence without the GUI.",
    )
    p.add_argument("--robot", required=True, choices=ROBOTS)
    p.add_argument("--mode", required=True, choices=("loop",))
    p.add_argument("--duration", required=True, type=parse_duration, help="1m .. 10h")
    p.add_argument(
        "--startup-timeout",
        type=parse_duration,
        default=DEFAULT_STARTUP_TIMEOUT_S,
        help="how long to wait for the stack before exit 3 (default 60s)",
    )
    p.add_argument(
        "--hold-abort-after",
        type=parse_duration,
        default=DEFAULT_HOLD_ABORT_AFTER_S,
        help="a dwell drift lasting longer aborts (default 1s, 0.1s .. 60s)",
    )
    p.add_argument("--yes", action="store_true", help="start without typing 'start'")
    return p


def check_args(args: argparse.Namespace) -> str:
    if not MIN_DURATION_S <= args.duration <= MAX_DURATION_S:
        return "--duration must be between 1m and 10h"
    if not MIN_HOLD_ABORT_AFTER_S <= args.hold_abort_after <= MAX_HOLD_ABORT_AFTER_S:
        return "--hold-abort-after must be between 0.1s and 60s"
    if args.startup_timeout <= 0.0:
        return "--startup-timeout must be positive"
    if not args.yes and not sys.stdin.isatty():
        return "no terminal to type 'start' in (nohup, a service, a pipe) — pass --yes"
    return ""


def shipped_config_dir() -> str:
    """The installed package's ``config``: where the shipped postures are read."""
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

    try:
        return os.path.join(get_package_share_directory("integrated_bringup"), "config")
    except PackageNotFoundError:
        raise FileProblem("integrated_bringup is not installed — source the workspace") from None


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if problem := check_args(args):
        print(f"error: {problem}", file=sys.stderr)
        return EXIT_USAGE
    if args.duration < SHORT_DURATION_WARN_S:
        print(f"note: --duration {args.duration / 3600:.2f} h is under 8 h", file=sys.stderr)
    # Where the run writes (its summary, the per-robot lock) and the disk it
    # watches: the stack's session folders go there too.
    root, problem = env.logging_root()
    if problem:
        print(f"error: {problem}", file=sys.stderr)
        return EXIT_USAGE
    for line in env.align_rmw(os.environ):
        print(line, file=sys.stderr)
    # rclpy is imported only now: the RMW above must be in place first, and the
    # argument / file errors need no ROS at all.
    from ..demo_gui.discovery import ROBOT_PROFILES

    try:
        inputs = load_run_inputs(shipped_config_dir(), args.robot)
    except FileProblem as exc:
        print(f"error: {exc}", file=sys.stderr)
        return EXIT_USAGE
    from .ros_io import run

    return run(args, ROBOT_PROFILES[args.robot], inputs, root)
