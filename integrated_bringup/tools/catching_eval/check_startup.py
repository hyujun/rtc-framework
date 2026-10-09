#!/usr/bin/env python3
"""Has the catching controller come up, or has the bringup failed? (#747)

``check_startup.py <launch.log>`` exits 0 when the log has the line the controller
writes once it can throw (``supervisor: trials enabled``), 1 when the bringup has
failed, 2 when neither is there yet (``run_unit.sh`` polls it once a second).

A failure is one of ``Config load failed`` / ``bring_up_failed`` / ``refus…`` on a line
that is NOT an ``[INFO]`` line. The level matters: a healthy controller says "… is
refused without the IK (too_far)" at INFO while it configures, a few tens of
milliseconds before the ready line, and a poll that fell between the two used to end
the unit as ``FAIL:startup`` before a throw. The lines that do mean a failed bringup
are WARN / ERROR / FATAL lines (the controller's ``refusing to configure`` / ``will
refuse to activate``, the manager's ``Config load failed``) or carry no level at all
(the launch's own output). The checks are functions so a test can feed them
synthetic logs.
"""

from __future__ import annotations

import re
import sys

READY, FAILED, WAITING = "ready", "failed", "waiting"
READY_LINE = "supervisor: trials enabled"
_FAILURE = re.compile(r"Config load failed|bring_up_failed|refus")
_INFO = re.compile(r"\[INFO\]")


def failure_lines(log_text: str) -> list[str]:
    """The lines of the log that say the bringup failed."""
    return [
        line for line in log_text.splitlines() if _FAILURE.search(line) and not _INFO.search(line)
    ]


def startup_state(log_text: str) -> str:
    """``ready`` once the ready line is there (whatever else is), else ``failed`` on a
    failure line, else ``waiting``."""
    if READY_LINE in log_text:
        return READY
    return FAILED if failure_lines(log_text) else WAITING


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    try:
        with open(argv[1], encoding="utf-8", errors="replace") as handle:
            text = handle.read()
    except OSError:
        return 2  # the launch has not written its log yet
    state = startup_state(text)
    if state == FAILED:
        print(failure_lines(text)[0], file=sys.stderr)
    return {READY: 0, FAILED: 1, WAITING: 2}[state]


if __name__ == "__main__":
    sys.exit(main(sys.argv))
