#!/usr/bin/env python3
"""Which segment mode did the controller log at configure? (#711)

``check_segment_mode.py <launch.log> <mpc|closed_form|mpc_docking> [--search <grid|nlp>]``
exits 0 when the controller's startup lines say exactly the expected mode, 1 otherwise
(the reason on stderr). Every mode is said — ``segment mode: mpc — …``,
``segment mode: closed_form — …``, ``segment mode: mpc_docking — …`` — so the expected
line has to be PRESENT: the absence of the other ones proves nothing, a controller that
never configured is just as silent. With ``--search`` the ``search mode: <grid|nlp> — …``
line is checked the same way, besides the segment line. ``run_unit.sh`` calls this; the
checks are functions so a test can feed them synthetic logs.
"""

from __future__ import annotations

import re
import sys

MODES = ("mpc", "closed_form", "mpc_docking")
SEARCH_MODES = ("grid", "nlp")
_LINE = re.compile(r"segment mode: (mpc_docking|mpc|closed_form)\b")
_SEARCH_LINE = re.compile(r"search mode: (grid|nlp)\b")


def logged_modes(log_text: str) -> set[str]:
    """The segment modes the log's ``segment mode: <mode>`` lines name."""
    return set(_LINE.findall(log_text))


def logged_search_modes(log_text: str) -> set[str]:
    """The search modes the log's ``search mode: <mode>`` lines name."""
    return set(_SEARCH_LINE.findall(log_text))


def check_search(log_text: str, expected: str) -> str | None:
    """``None`` when the log names search mode ``expected`` and no other, else why not."""
    if expected not in SEARCH_MODES:
        return f"expected search mode {expected!r} is not one of {SEARCH_MODES}"
    seen = logged_search_modes(log_text)
    if expected not in seen:
        return (
            f"no 'search mode: {expected}' line in the log (search modes logged: {sorted(seen)})"
        )
    if seen - {expected}:
        return f"the log also names {sorted(seen - {expected})}, expected only {expected}"
    return None


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
    args = list(argv)
    search = None
    if len(args) == 5 and args[3] == "--search":
        search = args[4]
        args = args[:3]
    if len(args) != 3:
        print(
            f"usage: {argv[0]} <launch.log> <mpc|closed_form|mpc_docking> [--search <grid|nlp>]",
            file=sys.stderr,
        )
        return 2
    try:
        with open(argv[1], encoding="utf-8", errors="replace") as handle:
            text = handle.read()
    except OSError as err:
        print(f"cannot read {argv[1]}: {err}", file=sys.stderr)
        return 1
    why = check(text, args[2])
    if why is None and search is not None:
        why = check_search(text, search)
    if why:
        print(why, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
