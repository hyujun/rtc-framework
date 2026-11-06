"""Standing-still metrics of a floating-base robot, from recorded base + foot samples.

The plant gate of the walking work (WG-0) asks four numbers of a robot that
stands under its position servos for T seconds:

* ``stance_hold_s``        — how long the judged window lasted,
* ``foot_slip_max``        — the largest xy excursion of any foot centre from
  where it was when the window opened [m],
* ``base_height_dev_max``  — the largest |z − z_ref| of the base [m], z_ref being
  the start pose's height (or the window's first sample),
* ``base_tilt_max``        — the largest angle between the base z axis and the
  world z axis [rad],

plus whether every foot carried load throughout (``steps_without_all_feet``,
the count of samples where some foot's normal load was at or under the
threshold). The thresholds are not here: this module measures, the caller
judges (the Sprint Contract numbers live with the issue that set them).

Two more numbers say whether the samples themselves can be trusted:
``sample_dt_median_s`` (the sensor period the recording ran at) and
``sample_gap_max_s`` (the longest gap between two samples in the window). A gap
of several periods means samples were dropped on the way — a per-sample count
such as ``steps_without_all_feet`` can then miss a one-sample event, and the
reader should know before believing a 0.

**The window opens after ``settle_s``** of the recording and runs to its end.
Slip and height references are taken at the FIRST sample of the window (or
``z_ref`` for the height), so a transient before the window — a servo mode
switch, the first contact — does not count as slip or sag. This is the same
rule the model-level stance test applies to its own first second.

Pure Python: no ROS, so the unit tests run under any interpreter. The samples
come from :mod:`rtc_tools.analysis.stance_metrics_recorder` (a CSV written at
the simulator's sensor rate) or from anything else that produces the schema in
:data:`COLUMNS`.

CSV schema (one row per sample, N feet):

    t, px, py, pz, qw, qx, qy, qz, foot0_x, foot0_y, foot0_z, foot0_load, foot1_x, ...

``t`` [s]; ``p*`` the base position [m]; ``q*`` the base orientation as a
Hamilton quaternion (w, x, y, z — MuJoCo's framequat order); ``foot<i>_*`` the
foot centre [m] and its normal load [N] (positive when the foot presses the
floor — the recorder flips the contact sensor's sign).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections.abc import Iterable, Sequence
from dataclasses import asdict, dataclass
from pathlib import Path

BASE_COLUMNS = ("t", "px", "py", "pz", "qw", "qx", "qy", "qz")
FOOT_FIELDS = ("x", "y", "z", "load")


def columns(num_feet: int) -> list[str]:
    """The CSV header for ``num_feet`` feet."""
    cols = list(BASE_COLUMNS)
    for i in range(num_feet):
        cols.extend(f"foot{i}_{f}" for f in FOOT_FIELDS)
    return cols


@dataclass(frozen=True)
class Sample:
    t: float
    pos: tuple[float, float, float]
    quat: tuple[float, float, float, float]  # w, x, y, z
    feet: tuple[tuple[float, float, float], ...]  # centre per foot
    loads: tuple[float, ...]  # normal load per foot, > 0 pressing the floor


@dataclass(frozen=True)
class StanceMetrics:
    stance_hold_s: float
    foot_slip_max: float
    base_height_dev_max: float
    base_tilt_max: float
    steps_without_all_feet: int
    num_samples: int  # samples inside the window
    num_feet: int
    z_ref: float
    sample_dt_median_s: float  # 0 when the window has one sample
    sample_gap_max_s: float  # 0 when the window has one sample


def tilt_rad(quat: Sequence[float]) -> float:
    """Angle between the body z axis and the world z axis for a (w, x, y, z) quaternion.

    The world-z component of the rotated z axis is ``1 − 2 (x² + y²)`` (the
    quaternion need not be normalised exactly: the value is clamped).
    """
    _, x, y, _ = quat
    cos_tilt = 1.0 - 2.0 * (x * x + y * y)
    return math.acos(max(-1.0, min(1.0, cos_tilt)))


def compute(
    samples: Iterable[Sample],
    *,
    settle_s: float,
    force_threshold_n: float,
    z_ref: float | None = None,
) -> StanceMetrics:
    """Metrics over the samples whose ``t − t_first >= settle_s``.

    Raises ``ValueError`` when the window is empty — a recording shorter than
    its settle time has measured nothing, and a zero metric would read as a
    pass.
    """
    rows = list(samples)
    if not rows:
        raise ValueError("no samples")
    t0 = rows[0].t
    window = [s for s in rows if s.t - t0 >= settle_s]
    if not window:
        raise ValueError(
            f"no sample after the settle time ({settle_s} s) — the recording lasted "
            f"{rows[-1].t - t0:.3f} s"
        )
    first = window[0]
    num_feet = len(first.feet)
    ref_z = first.pos[2] if z_ref is None else z_ref
    slip_max = 0.0
    height_dev_max = 0.0
    tilt_max = 0.0
    without_all = 0
    for s in window:
        if len(s.feet) != num_feet or len(s.loads) != num_feet:
            raise ValueError("samples disagree on the number of feet")
        for k in range(num_feet):
            dx = s.feet[k][0] - first.feet[k][0]
            dy = s.feet[k][1] - first.feet[k][1]
            slip_max = max(slip_max, math.hypot(dx, dy))
        height_dev_max = max(height_dev_max, abs(s.pos[2] - ref_z))
        tilt_max = max(tilt_max, tilt_rad(s.quat))
        if not all(load > force_threshold_n for load in s.loads):
            without_all += 1
    gaps = sorted(b.t - a.t for a, b in zip(window, window[1:], strict=False))
    return StanceMetrics(
        stance_hold_s=window[-1].t - first.t,
        foot_slip_max=slip_max,
        base_height_dev_max=height_dev_max,
        base_tilt_max=tilt_max,
        steps_without_all_feet=without_all,
        num_samples=len(window),
        num_feet=num_feet,
        z_ref=ref_z,
        sample_dt_median_s=gaps[len(gaps) // 2] if gaps else 0.0,
        sample_gap_max_s=gaps[-1] if gaps else 0.0,
    )


def to_row(sample: Sample) -> list[float]:
    row = [sample.t, *sample.pos, *sample.quat]
    for centre, load in zip(sample.feet, sample.loads, strict=True):
        row.extend([*centre, load])
    return row


def from_row(row: dict[str, str]) -> Sample:
    feet = []
    loads = []
    i = 0
    while f"foot{i}_x" in row:
        feet.append((float(row[f"foot{i}_x"]), float(row[f"foot{i}_y"]), float(row[f"foot{i}_z"])))
        loads.append(float(row[f"foot{i}_load"]))
        i += 1
    return Sample(
        t=float(row["t"]),
        pos=(float(row["px"]), float(row["py"]), float(row["pz"])),
        quat=(float(row["qw"]), float(row["qx"]), float(row["qy"]), float(row["qz"])),
        feet=tuple(feet),
        loads=tuple(loads),
    )


def read_csv(path: Path) -> list[Sample]:
    with path.open(newline="") as handle:
        return [from_row(r) for r in csv.DictReader(handle)]


def write_csv(path: Path, samples: Iterable[Sample], num_feet: int) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(columns(num_feet))
        for s in samples:
            writer.writerow(to_row(s))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("csv", type=Path, help="recording written by run_stance_metrics")
    parser.add_argument("--settle", type=float, default=1.0, help="window opens after this [s]")
    parser.add_argument(
        "--force-threshold", type=float, default=5.0, help="a foot carries load above this [N]"
    )
    parser.add_argument(
        "--z-ref", type=float, default=None, help="height reference [m]; default = window's first"
    )
    args = parser.parse_args(argv)
    try:
        metrics = compute(
            read_csv(args.csv),
            settle_s=args.settle,
            force_threshold_n=args.force_threshold,
            z_ref=args.z_ref,
        )
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(asdict(metrics), indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
