#!/usr/bin/env python3
"""Which segment mode did the controller log at configure? (#711)

``check_segment_mode.py <launch.log> <mpc|closed_form>`` exits 0 when the controller's
startup lines say exactly the expected mode, 1 otherwise (the reason on stderr). Both
modes are said — ``segment mode: mpc — …`` and ``segment mode: closed_form — …`` — so
the expected line has to be PRESENT: the absence of the other one proves nothing, a
controller that never configured is just as silent. ``run_unit.sh`` calls this; the
check is a function so a test can feed it synthetic logs.
"""

from __future__ import annotations

import re
import sys

MODES = ("mpc", "closed_form")
_LINE = re.compile(r"segment mode: (mpc|closed_form)\b")


def logged_modes(log_text: str) -> set[str]:
    """The segment modes the log's ``segment mode: <mode>`` lines name."""
    return set(_LINE.findall(log_text))


def check(log_text: str, expected: str) -> str | None:
    """``None`` when the log names ``expected`` and no other mode, else why not."""
    if expected not in MODES:
        return f"expected mode {expected!r} is not one of {MODES}"
    seen = logged_modes(log_text)
    if expected not in seen:
        return f"no 'segment mode: {expected}' line in the log (modes logged: {sorted(seen)})"
    if seen - {expected}:
        return f"the log also names {sorted(seen - {expected})}, expected only {expected}"
    return None


def main(argv: list[str]) -> int:
    if len(argv) != 3:
        print(f"usage: {argv[0]} <launch.log> <mpc|closed_form>", file=sys.stderr)
        return 2
    try:
        with open(argv[1], encoding="utf-8", errors="replace") as handle:
            text = handle.read()
    except OSError as err:
        print(f"cannot read {argv[1]}: {err}", file=sys.stderr)
        return 1
    why = check(text, argv[2])
    if why:
        print(why, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
