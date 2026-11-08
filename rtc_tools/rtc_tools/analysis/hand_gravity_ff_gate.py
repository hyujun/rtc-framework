"""PASS / FAIL judge for a hand gravity-feedforward A/B/A run (``joint_ff.csv``).

The producer (HandJointController, config_key ``hand_joint_controller``) writes
one row per tick to ``<session>/controllers/<config_key>/joint_ff.csv``; the
column contract is the C++ header writer in
``integrated_bringup/.../logging/joint_feedforward_log_pod.hpp``. This judge
reads, per row, ``t_relative_s`` · ``ff_enabled`` · ``readable`` · ``estop`` ·
``ff_finite`` · ``held`` · ``held_ticks`` and per joint ``q_<joint>`` /
``q_ref_<joint>``. The joint list and its order are the ``tau_ff_<joint>``
block of the header (``q_`` is also the prefix of ``q_ref_``, so it cannot
name the joints on its own).

Pre-registered gate — the thresholds below are the contract, not a tuning
knob; every one is a CLI flag so a deviation is visible on the command line.

  Segments   the rows are split by every change of ``ff_enabled`` into
             contiguous segments (an A/B/A run is on, off, on). A segment
             shorter than ``--min-segment-s`` in ``t_relative_s`` is ignored.
             At least one usable ``off`` AND one usable ``on`` segment are
             required, otherwise the per-joint checks are UNDECIDED.
  Window     per segment, statistics are taken over the LAST ``--window-s``
             seconds of it, rows with ``readable == 1`` and ``estop == 0`` only.
  Metric     per joint ``e_j = |q_ref_j - q_j|`` [rad]; the statistic is the
             window mean (the max is reported). With several on / off segments
             the worst (largest) on-mean and the smallest off-mean are judged.
  Big joint  premise ``off_mean >= --big-off-min-rad`` (else UNDECIDED: the
             plant does not sag enough without feedforward to discriminate);
             ``on_mean <= --big-on-max-rad`` and
             ``on_mean <= --big-ratio-max x off_mean``.
  Small      premise ``off_mean >= --small-off-min-rad`` (else UNDECIDED);
             ``on_mean <= --small-on-max-rad``.
  Excluded   reported, never judged.
  Global     every on-window row has ``ff_finite == 1``; every window row
             (on and off) has ``held == 0``.

``--loop-discriminator`` names the joint whose row is annotated as the
loop-consistency discriminator (a closed-chain joint whose gravity term is
right only if the projection is); it changes no verdict.

Verdict: FAIL if any check FAILs, else UNDECIDED if any is UNDECIDED, else
PASS. Exit code 0 / 1 (FAIL) / 2 (UNDECIDED) / 3 (unusable input: no or
several ``joint_ff.csv`` without ``--csv``, missing columns, a joint-list
name that matches no joint, or one joint in both lists).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path

PASS = "PASS"
FAIL = "FAIL"
UNDECIDED = "UNDECIDED"
NOT_EVALUATED = "NOT_EVALUATED"

# kJointFeedforwardLogInstance in the POD header.
JOINT_FF_STEM = "joint_ff"
# The per-joint column prefixes, in the header writer's order.
Q_PREFIX = "q_"
Q_REF_PREFIX = "q_ref_"
G_PREFIX = "g_"
TAU_FF_PREFIX = "tau_ff_"
REQUIRED_COLUMNS = (
    "t_relative_s",
    "tick",
    "estop",
    "readable",
    "ff_enabled",
    "ff_finite",
    "held",
    "held_ticks",
)

# The joint classes of the shipped hand, matched by suffix against the joints
# the file names (see `classify`). Overridden per run with --big / --small /
# --exclude.
DEFAULT_BIG = (
    "cmc_01_joint",
    "cmc_02_joint",
    "index_mcp_joint",
    "middle_mcp_joint",
    "ring_mcp_joint",
)
DEFAULT_SMALL = (
    "thumb_mcp_joint",
    "thumb_pip_link02_joint",
    "index_pip_joint",
    "middle_pip_joint",
)
DEFAULT_EXCLUDE = ("index_mcp_rotate_joint",)
DEFAULT_LOOP_DISCRIMINATOR = "ring_mcp_joint"

CLASS_BIG = "big"
CLASS_SMALL = "small"
CLASS_EXCLUDED = "excluded"
CLASS_UNLISTED = "unlisted"


@dataclass
class Thresholds:
    min_segment_s: float = 10.0
    window_s: float = 3.0
    big_off_min_rad: float = 0.010
    big_on_max_rad: float = 0.001
    big_ratio_max: float = 0.05
    small_off_min_rad: float = 0.003
    small_on_max_rad: float = 0.001


@dataclass
class Check:
    key: str
    verdict: str
    value: str
    threshold: str
    note: str = ""


# ── input ────────────────────────────────────────────────────────────────────


def joint_names_of(columns) -> list[str]:
    """The joints a joint_ff header names, in device order.

    Read off the ``tau_ff_`` block: ``q_`` is a prefix of ``q_ref_`` too, so the
    ``q_`` block cannot be told apart from the reference block by prefix alone.
    """
    return [c[len(TAU_FF_PREFIX) :] for c in columns if c.startswith(TAU_FF_PREFIX)]


@dataclass
class Table:
    path: Path
    columns: list[str]
    joints: list[str]
    rows: list[dict[str, float]]


def _float(text: str) -> float:
    try:
        return float(text)
    except ValueError:
        return math.nan


def load_joint_ff_csv(path: Path) -> Table:
    """Read one ``joint_ff.csv``. Raises ValueError on a header this judge cannot use."""
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            raise ValueError(f"{path}: empty file")
        missing = [c for c in REQUIRED_COLUMNS if c not in header]
        if missing:
            raise ValueError(f"{path}: not a joint_ff.csv — missing columns {missing}")
        joints = joint_names_of(header)
        if not joints:
            raise ValueError(f"{path}: no {TAU_FF_PREFIX}<joint> columns")
        for j in joints:
            for prefix in (Q_PREFIX, Q_REF_PREFIX):
                if f"{prefix}{j}" not in header:
                    raise ValueError(f"{path}: joint {j} has no {prefix}{j} column")
        rows = []
        for raw in reader:
            if len(raw) != len(header):
                continue  # a line cut mid-write
            rows.append({k: _float(v) for k, v in zip(header, raw, strict=True)})
    if not rows:
        raise ValueError(f"{path}: no data rows")
    return Table(path, header, joints, rows)


def find_joint_ff_csv(target: Path, explicit: Path | None) -> Path:
    """Resolve the positional argument to one CSV file.

    A file is used as is. A directory is searched for ``**/joint_ff.csv``; one
    hit is used, none or several is an error unless ``--csv`` names the file
    (absolute, or relative to the directory).
    """
    if explicit is not None:
        candidate = explicit if explicit.is_absolute() else target / explicit
        if not candidate.is_file():
            candidate = explicit
        if not candidate.is_file():
            raise ValueError(f"--csv {explicit}: no such file")
        return candidate
    if target.is_file():
        return target
    if not target.is_dir():
        raise ValueError(f"{target}: no such file or directory")
    hits = sorted(target.rglob(f"{JOINT_FF_STEM}.csv"))
    if not hits:
        raise ValueError(f"{target}: no {JOINT_FF_STEM}.csv under it")
    if len(hits) > 1:
        listed = ", ".join(str(h.relative_to(target)) for h in hits)
        raise ValueError(
            f"{target}: {len(hits)} {JOINT_FF_STEM}.csv files ({listed}) — pass --csv"
        )
    return hits[0]


# ── segments and windows ─────────────────────────────────────────────────────


@dataclass
class Segment:
    index: int
    ff_on: bool
    rows: list[dict[str, float]] = field(default_factory=list)

    @property
    def t_start(self) -> float:
        return self.rows[0]["t_relative_s"]

    @property
    def t_end(self) -> float:
        return self.rows[-1]["t_relative_s"]

    @property
    def span_s(self) -> float:
        return self.t_end - self.t_start


def split_segments(rows: list[dict[str, float]]) -> list[Segment]:
    """Contiguous runs of equal ``ff_enabled``, in file order."""
    segments: list[Segment] = []
    for row in rows:
        on = row["ff_enabled"] == 1
        if not segments or segments[-1].ff_on != on:
            segments.append(Segment(len(segments), on))
        segments[-1].rows.append(row)
    return segments


def window_rows(segment: Segment, window_s: float) -> list[dict[str, float]]:
    """The last ``window_s`` seconds of a segment, readable and not E-STOP rows only."""
    t_from = segment.t_end - window_s
    return [
        r
        for r in segment.rows
        if r["t_relative_s"] >= t_from and r["readable"] == 1 and r["estop"] == 0
    ]


def _mean(values: list[float]) -> float:
    return math.fsum(values) / len(values) if values else math.nan


@dataclass
class SegmentSummary:
    index: int
    ff_on: bool
    t_start: float
    t_end: float
    span_s: float
    rows: int
    window_rows: int
    usable: bool
    # joint -> (mean, max) of |q_ref - q| over the window
    error: dict[str, tuple[float, float]] = field(default_factory=dict)
    ff_finite_violations: int = 0
    held_rows: int = 0
    held_ticks_max: int = 0


@dataclass
class JointStat:
    joint: str
    on_mean: float = math.nan  # worst (largest) over the usable on segments
    on_max: float = math.nan
    off_mean: float = math.nan  # smallest over the usable off segments
    off_max: float = math.nan

    @property
    def ratio(self) -> float:
        if math.isnan(self.on_mean) or math.isnan(self.off_mean) or self.off_mean == 0:
            return math.nan
        return self.on_mean / self.off_mean


@dataclass
class Summary:
    segments: list[SegmentSummary]
    joints: dict[str, JointStat]
    usable_on: int
    usable_off: int
    on_window_rows: int
    ff_finite_violations: int  # on-window rows with ff_finite != 1
    window_rows: int  # on + off windows
    held_rows: int  # window rows with held == 1
    held_ticks_max: int


def summarise(table: Table, th: Thresholds) -> Summary:
    segments = split_segments(table.rows)
    out: list[SegmentSummary] = []
    stats = {j: JointStat(j) for j in table.joints}
    ff_finite_violations = on_window_rows = window_total = held_rows = held_ticks_max = 0
    usable_on = usable_off = 0
    for seg in segments:
        usable = seg.span_s >= th.min_segment_s
        win = window_rows(seg, th.window_s) if usable else []
        s = SegmentSummary(
            seg.index,
            seg.ff_on,
            seg.t_start,
            seg.t_end,
            seg.span_s,
            len(seg.rows),
            len(win),
            usable,
        )
        if usable:
            if seg.ff_on:
                usable_on += 1
            else:
                usable_off += 1
        for j in table.joints:
            e = [abs(r[f"{Q_REF_PREFIX}{j}"] - r[f"{Q_PREFIX}{j}"]) for r in win]
            e = [v for v in e if not math.isnan(v)]
            mean, peak = _mean(e), (max(e) if e else math.nan)
            s.error[j] = (mean, peak)
            if usable and not math.isnan(mean):
                st = stats[j]
                if seg.ff_on and (math.isnan(st.on_mean) or mean > st.on_mean):
                    st.on_mean, st.on_max = mean, peak
                if not seg.ff_on and (math.isnan(st.off_mean) or mean < st.off_mean):
                    st.off_mean, st.off_max = mean, peak
        if usable:
            s.held_rows = sum(1 for r in win if r["held"] == 1)
            s.held_ticks_max = int(max((r["held_ticks"] for r in win), default=0))
            held_rows += s.held_rows
            held_ticks_max = max(held_ticks_max, s.held_ticks_max)
            window_total += len(win)
            if seg.ff_on:
                s.ff_finite_violations = sum(1 for r in win if r["ff_finite"] != 1)
                ff_finite_violations += s.ff_finite_violations
                on_window_rows += len(win)
        out.append(s)
    return Summary(
        out,
        stats,
        usable_on,
        usable_off,
        on_window_rows,
        ff_finite_violations,
        window_total,
        held_rows,
        held_ticks_max,
    )


# ── joint classes ────────────────────────────────────────────────────────────


def _matches(joint: str, name: str) -> bool:
    return joint == name or joint.endswith(name)


def classify(
    joints: list[str],
    big: tuple[str, ...] | list[str],
    small: tuple[str, ...] | list[str],
    exclude: tuple[str, ...] | list[str],
) -> dict[str, str]:
    """joint -> class. A list name that matches no joint, or a joint claimed by
    two lists, is a ValueError (a typo must not quietly judge nothing)."""
    classes: dict[str, str] = {}
    for label, names in ((CLASS_BIG, big), (CLASS_SMALL, small), (CLASS_EXCLUDED, exclude)):
        for name in names:
            hits = [j for j in joints if _matches(j, name)]
            if not hits:
                raise ValueError(f"--{label} {name!r} matches no joint in {joints}")
            for j in hits:
                if j in classes and classes[j] != label:
                    raise ValueError(f"joint {j} is both --{classes[j]} and --{label}")
                classes[j] = label
    for j in joints:
        classes.setdefault(j, CLASS_UNLISTED)
    return classes


def parse_list(text: str) -> tuple[str, ...]:
    return tuple(t.strip() for t in text.split(",") if t.strip())


# ── judge ────────────────────────────────────────────────────────────────────


def _rad(x: float) -> str:
    return "n/a" if math.isnan(x) else f"{x:.5f} rad"


def judge(
    summary: Summary,
    classes: dict[str, str],
    th: Thresholds,
    loop_discriminator: str | None = DEFAULT_LOOP_DISCRIMINATOR,
) -> list[Check]:
    checks: list[Check] = []
    have_pair = summary.usable_on >= 1 and summary.usable_off >= 1
    ignored = sum(1 for s in summary.segments if not s.usable)
    checks.append(
        Check(
            "a.segments",
            PASS if have_pair else UNDECIDED,
            f"on {summary.usable_on}, off {summary.usable_off} usable "
            f"({len(summary.segments)} total, {ignored} shorter than the minimum)",
            f">= 1 on and >= 1 off, each >= {th.min_segment_s:g} s",
            "" if have_pair else "no on/off pair to compare — record a longer A/B/A run",
        )
    )

    for joint, st in summary.joints.items():
        cls = classes.get(joint, CLASS_UNLISTED)
        tag = ""
        if loop_discriminator and _matches(joint, loop_discriminator):
            tag = "loop-consistency discriminator"
        value = f"on {_rad(st.on_mean)} (max {_rad(st.on_max)}), off {_rad(st.off_mean)}"
        if cls in (CLASS_EXCLUDED, CLASS_UNLISTED):
            why = "excluded by --exclude" if cls == CLASS_EXCLUDED else "in no joint list"
            checks.append(Check(f"{joint}.info", NOT_EVALUATED, value, "—", _join(why, tag)))
            continue
        off_min = th.big_off_min_rad if cls == CLASS_BIG else th.small_off_min_rad
        on_max = th.big_on_max_rad if cls == CLASS_BIG else th.small_on_max_rad
        if not have_pair:
            reason = "no usable on/off segment pair"
            keys = ("off_floor", "on_abs") + (("on_ratio",) if cls == CLASS_BIG else ())
            for key in keys:
                checks.append(
                    Check(f"{joint}.{key}", NOT_EVALUATED, value, "—", _join(reason, tag))
                )
            continue
        no_on = math.isnan(st.on_mean)
        no_off = math.isnan(st.off_mean)
        premise = (not no_off) and st.off_mean >= off_min
        if no_off:
            premise_note = "no usable rows in any off window"
        elif premise:
            premise_note = ""
        else:
            premise_note = (
                f"{cls} joint sags less than {off_min:g} rad without FF — cannot discriminate"
            )
        checks.append(
            Check(
                f"{joint}.off_floor",
                PASS if premise else UNDECIDED,
                f"off {_rad(st.off_mean)} (max {_rad(st.off_max)})",
                f">= {off_min:g} rad ({cls})",
                _join(premise_note, tag),
            )
        )
        if no_on or not premise:
            note = "no usable rows in any on window" if no_on else "premise not met"
            checks.append(
                Check(
                    f"{joint}.on_abs",
                    UNDECIDED,
                    f"on {_rad(st.on_mean)}",
                    f"<= {on_max:g} rad ({cls})",
                    _join(note, tag),
                )
            )
            if cls == CLASS_BIG:
                checks.append(
                    Check(
                        f"{joint}.on_ratio",
                        UNDECIDED,
                        f"on / off = {st.ratio:.4f}" if not math.isnan(st.ratio) else "n/a",
                        f"<= {th.big_ratio_max:g}",
                        _join(note, tag),
                    )
                )
            continue
        checks.append(
            Check(
                f"{joint}.on_abs",
                PASS if st.on_mean <= on_max else FAIL,
                f"on {_rad(st.on_mean)} (max {_rad(st.on_max)})",
                f"<= {on_max:g} rad ({cls})",
                tag,
            )
        )
        if cls == CLASS_BIG:
            limit = th.big_ratio_max * st.off_mean
            checks.append(
                Check(
                    f"{joint}.on_ratio",
                    PASS if st.on_mean <= limit else FAIL,
                    f"on / off = {st.ratio:.4f}",
                    f"<= {th.big_ratio_max:g} (= {limit:.5f} rad here)",
                    tag,
                )
            )

    if summary.on_window_rows:
        checks.append(
            Check(
                "g.ff_finite",
                PASS if summary.ff_finite_violations == 0 else FAIL,
                f"{summary.ff_finite_violations} / {summary.on_window_rows} on-window rows "
                "without a finite feedforward",
                "0",
            )
        )
    else:
        checks.append(
            Check("g.ff_finite", NOT_EVALUATED, "no on-window rows", "0", "no usable on window")
        )
    if summary.window_rows:
        checks.append(
            Check(
                "g.held",
                PASS if summary.held_rows == 0 else FAIL,
                f"{summary.held_rows} / {summary.window_rows} window rows held",
                "0",
                f"held_ticks max {summary.held_ticks_max}" if summary.held_rows else "",
            )
        )
    else:
        checks.append(Check("g.held", NOT_EVALUATED, "no window rows", "0", "no usable segment"))
    return checks


def _join(*parts: str) -> str:
    return "; ".join(p for p in parts if p)


def overall(checks: list[Check]) -> str:
    verdicts = {c.verdict for c in checks}
    if FAIL in verdicts:
        return FAIL
    if UNDECIDED in verdicts:
        return UNDECIDED
    return PASS


# ── report ───────────────────────────────────────────────────────────────────


def render_markdown(
    checks: list[Check], summary: Summary, classes: dict[str, str], th: Thresholds
) -> str:
    not_evaluated = [c.key for c in checks if c.verdict == NOT_EVALUATED]
    tail = f" — not evaluated: {', '.join(not_evaluated)}" if not_evaluated else ""
    out = [f"## hand gravity FF verdict: **{overall(checks)}**{tail}", ""]
    out += ["| check | verdict | value | threshold | note |", "|---|---|---|---|---|"]
    for c in checks:
        out.append(f"| `{c.key}` | {c.verdict} | {c.value} | {c.threshold} | {c.note} |")
    out += ["", "### Segments (split on `ff_enabled`)", ""]
    out += [
        f"| # | ff | t_start | t_end | span | rows | window rows (last {th.window_s:g} s, "
        "readable, no E-STOP) | used |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for s in summary.segments:
        used = "yes" if s.usable else f"no (< {th.min_segment_s:g} s)"
        out.append(
            f"| {s.index} | {'on' if s.ff_on else 'off'} | {s.t_start:.3f} | {s.t_end:.3f} | "
            f"{s.span_s:.3f} s | {s.rows} | {s.window_rows} | {used} |"
        )
    out += ["", "### Per joint, |q_ref − q| over the judged windows", ""]
    out += [
        "| joint | class | off mean | off max | on mean (worst) | on max | on / off |",
        "|---|---|---|---|---|---|---|",
    ]
    for joint, st in summary.joints.items():
        ratio = "n/a" if math.isnan(st.ratio) else f"{st.ratio:.4f}"
        out.append(
            f"| `{joint}` | {classes.get(joint, CLASS_UNLISTED)} | {_rad(st.off_mean)} | "
            f"{_rad(st.off_max)} | {_rad(st.on_mean)} | {_rad(st.on_max)} | {ratio} |"
        )
    return "\n".join(out) + "\n"


def _json_safe(obj):
    """NaN -> null (json.dumps would emit a bare NaN token, which is not JSON)."""
    if isinstance(obj, float):
        return None if math.isnan(obj) else obj
    if isinstance(obj, dict):
        return {str(k): _json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_json_safe(v) for v in obj]
    if isinstance(obj, Path):
        return str(obj)
    return obj


# ── CLI ──────────────────────────────────────────────────────────────────────


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "target", type=Path, help=f"a {JOINT_FF_STEM}.csv, or a session directory holding one"
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=None,
        help="the CSV to judge when the directory holds several (absolute or relative to it)",
    )
    parser.add_argument("--big", default=",".join(DEFAULT_BIG), help="big joints (suffix match)")
    parser.add_argument(
        "--small", default=",".join(DEFAULT_SMALL), help="small joints (suffix match)"
    )
    parser.add_argument(
        "--exclude", default=",".join(DEFAULT_EXCLUDE), help="joints reported, never judged"
    )
    parser.add_argument(
        "--loop-discriminator",
        default=DEFAULT_LOOP_DISCRIMINATOR,
        help="joint annotated as the loop-consistency discriminator ('' for none)",
    )
    parser.add_argument("--min-segment-s", type=float, default=Thresholds.min_segment_s)
    parser.add_argument("--window-s", type=float, default=Thresholds.window_s)
    parser.add_argument("--big-off-min-rad", type=float, default=Thresholds.big_off_min_rad)
    parser.add_argument("--big-on-max-rad", type=float, default=Thresholds.big_on_max_rad)
    parser.add_argument("--big-ratio-max", type=float, default=Thresholds.big_ratio_max)
    parser.add_argument("--small-off-min-rad", type=float, default=Thresholds.small_off_min_rad)
    parser.add_argument("--small-on-max-rad", type=float, default=Thresholds.small_on_max_rad)
    parser.add_argument(
        "--json-out", type=Path, default=None, help="write checks + figures as JSON"
    )
    args = parser.parse_args(argv)
    th = Thresholds(
        min_segment_s=args.min_segment_s,
        window_s=args.window_s,
        big_off_min_rad=args.big_off_min_rad,
        big_on_max_rad=args.big_on_max_rad,
        big_ratio_max=args.big_ratio_max,
        small_off_min_rad=args.small_off_min_rad,
        small_on_max_rad=args.small_on_max_rad,
    )
    try:
        csv_path = find_joint_ff_csv(args.target, args.csv)
        table = load_joint_ff_csv(csv_path)
        classes = classify(
            table.joints, parse_list(args.big), parse_list(args.small), parse_list(args.exclude)
        )
    except (OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 3

    summary = summarise(table, th)
    checks = judge(summary, classes, th, args.loop_discriminator or None)

    print(f"csv    : {csv_path}")
    print(f"joints : {', '.join(table.joints)}")
    print()
    print(render_markdown(checks, summary, classes, th))

    if args.json_out is not None:
        payload = {
            "verdict": overall(checks),
            "csv": csv_path,
            "joints": table.joints,
            "classes": classes,
            "checks": [asdict(c) for c in checks],
            "thresholds": asdict(th),
            "segments": [asdict(s) for s in summary.segments],
            "per_joint": {
                j: {**asdict(st), "ratio": st.ratio} for j, st in summary.joints.items()
            },
        }
        args.json_out.write_text(json.dumps(_json_safe(payload), indent=2) + "\n")

    return {PASS: 0, FAIL: 1, UNDECIDED: 2}[overall(checks)]


if __name__ == "__main__":
    sys.exit(main())
