#!/usr/bin/env python3
"""What a unit's EXPECT_KV says of the approach grid it must start with (#822).

    expect_kv.py grid <expect_mode> <expect_kv> <expect_npre|-> <expect_dtpre|->
    expect_kv.py overlay <overlay.yaml> [<expect_kv>]

``run_unit.sh`` checks the segment planner's approach grid twice: the mirror's
``approach.n_pre_max`` against EXPECT_NPRE, and the startup line "… up to <n> x
<dt> s" against EXPECT_NPRE and EXPECT_DTPRE (``%.3f``, as the controller prints
it). An overlay that moves the grid had to set the two by hand beside its
EXPECT_KV — four places for one value, and a unit lost before a throw when one
was missed (#819's FAIL:overlay).

``grid`` prints ``<n_pre> <dt_pre>`` — each the EXPECT_* given, else what EXPECT_KV
sets for the planner of ``<expect_mode>`` (``planner.segment.<mode>.approach.n_pre_max``
/ ``.dt_pre_s``), else ``-`` (the caller keeps its default). A key of another
planner says nothing of this unit's grid and is not read. An EXPECT_* given that
EXPECT_KV contradicts is refused (exit 2): the unit could not pass both checks.

``overlay`` prints the EXPECT_KV the unit checks: the caller's items, then those
``mk_overlay.py --set`` wrote into the overlay's header (``# EXPECT_KV=…``) that the
caller does not name. Both are checked: a caller's EXPECT_KV that only added a key
once left the overlay's own values unread and unrecorded. A key both set to other
values is refused (exit 2).
"""

import sys
from pathlib import Path

HEADER = "# EXPECT_KV="
NONE = "-"


def parse(expect_kv: str) -> dict[str, str]:
    """``key=value;key=value`` as a dict (an empty item is skipped)."""
    out = {}
    for item in expect_kv.split(";"):
        if not item.strip():
            continue
        key, sep, value = item.partition("=")
        if not sep:
            raise ValueError(f"EXPECT_KV item {item!r} is not key=value")
        out[key.strip()] = value.strip()
    return out


def dt_text(value: str) -> str:
    """A grid step as the controller's startup line prints it (``%.3f``)."""
    return f"{float(value):.3f}"


def approach_grid(
    mode: str, expect_kv: str, npre: str | None, dtpre: str | None
) -> tuple[str | None, str | None]:
    """``(n_pre, dt_pre)`` the unit must start with (module docstring); None for one
    neither EXPECT_* nor EXPECT_KV says. ``ValueError`` when the two contradict."""
    kv = parse(expect_kv)
    prefix = f"planner.segment.{mode}.approach."
    set_n = kv.get(prefix + "n_pre_max")
    set_dt = kv.get(prefix + "dt_pre_s")
    if set_n is not None and npre is not None and int(set_n) != int(npre):
        raise ValueError(f"EXPECT_NPRE {npre}, but EXPECT_KV sets {prefix}n_pre_max={set_n}")
    if set_dt is not None and dtpre is not None and dt_text(set_dt) != dt_text(dtpre):
        raise ValueError(f"EXPECT_DTPRE {dtpre}, but EXPECT_KV sets {prefix}dt_pre_s={set_dt}")
    n = npre if npre is not None else set_n
    dt = dtpre if dtpre is not None else (None if set_dt is None else dt_text(set_dt))
    return n, dt


def same_value(a: str, b: str) -> bool:
    """Two EXPECT_KV values for one key: equal as numbers when both are, else as text."""
    try:
        return float(a) == float(b)
    except ValueError:
        return a == b


def merged(caller: str, header: str) -> str:
    """The caller's EXPECT_KV with the overlay header's items it does not name
    appended. ``ValueError`` for a key the two set to other values."""
    mine = parse(caller)
    theirs = parse(header)
    for key, value in theirs.items():
        if key in mine and not same_value(mine[key], value):
            raise ValueError(f"EXPECT_KV sets {key}={mine[key]}, the overlay's header {value}")
    items = [f"{k}={v}" for k, v in mine.items()]
    items += [f"{k}={v}" for k, v in theirs.items() if k not in mine]
    return ";".join(items)


def overlay_expect_kv(path: Path) -> str:
    """The EXPECT_KV an overlay's header states, or ``""``."""
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if line.startswith(HEADER):
            return line[len(HEADER) :].strip()
        if not line.startswith("#"):
            break
    return ""


def main(argv: list[str]) -> int:
    if argv[:1] == ["overlay"] and len(argv) in (2, 3):
        caller = argv[2] if len(argv) == 3 else ""
        try:
            print(merged(caller, overlay_expect_kv(Path(argv[1]))))
        except ValueError as exc:
            print(f"expect_kv: {exc}", file=sys.stderr)
            return 2
        return 0
    if argv[:1] == ["grid"] and len(argv) == 5:
        mode, expect_kv, npre, dtpre = argv[1:]
        try:
            n, dt = approach_grid(
                mode, expect_kv, None if npre == NONE else npre, None if dtpre == NONE else dtpre
            )
        except ValueError as exc:
            print(f"expect_kv: {exc}", file=sys.stderr)
            return 2
        print(f"{n or NONE} {dt or NONE}")
        return 0
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
