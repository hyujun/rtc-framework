"""Decode and summarise the ball_perception prediction lane (dynamic_catching S3.4).

Plan: ``docs/dynamic_catching/IMPLEMENTATION_PLAN.md`` §4.4 S3.4 and D-4. S3.4
**measures only** — every policy question (how to treat a rewinding
``snapshot_sequence``, whether to accept partial ``validity``) belongs to S5.2.
This module turns what ``sim_estimator_node`` publishes into numbers those
decisions can be made from, and it does so **by field name**, which is the
whole of D-4: the ``prediction/trajectory`` PointCloud2 layout is a promise
written in ball_perception's README, not a compile-time type, so a consumer
checks name, offset, datatype, count, ``point_step`` and endianness and
**refuses** a message that differs. A decoder that indexed by fixed offset would
read garbage as a trajectory the day one field moved, and every downstream
number would still look plausible.

Two halves, deliberately separable:

* ``decode_trajectory`` and ``check_layout`` take primitives (field tuples,
  ``point_step``, a ``bytes`` buffer) rather than a ROS message, so the
  contract is unit-testable without rclpy and so the same code decodes a
  message and a recorded buffer identically.
* The ``summarise_*`` functions read the CSVs the probe writes
  (``vision_lane_probe``) and answer the S3.4 questions: publish rate, N,
  horizon (TBD-VIS-04), frame (TBD-VIS-06), what ``validity`` does after the
  measurement stops (TBD-VIS-07, the ghost-track question) and whether a
  best-effort subscription loses anything a reliable one gets (TBD-VIS-08).

Nothing here decides a threshold. The report says what was observed.
"""

from __future__ import annotations

import bisect
import csv
import math
import struct
import sys
from collections import Counter
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path

# sensor_msgs/PointField datatype codes. Spelled out here so the decoder has no
# message dependency; the probe maps msg.fields onto these.
UINT8 = 2
UINT32 = 6
FLOAT64 = 8

_DATATYPE_STRUCT = {UINT8: "B", UINT32: "I", FLOAT64: "d"}

# The layout ball_perception_sim/README.md "prediction/trajectory layout"
# promises. (name, offset, datatype, count). uint64 values are carried as two
# UINT32 [low, high] because PointField has no 64-bit integer.
EXPECTED_FIELDS: tuple[tuple[str, int, int, int], ...] = (
    ("x", 0, FLOAT64, 1),
    ("y", 8, FLOAT64, 1),
    ("z", 16, FLOAT64, 1),
    ("vx", 24, FLOAT64, 1),
    ("vy", 32, FLOAT64, 1),
    ("vz", 40, FLOAT64, 1),
    ("ax", 48, FLOAT64, 1),
    ("ay", 56, FLOAT64, 1),
    ("az", 64, FLOAT64, 1),
    ("covariance", 72, FLOAT64, 36),
    ("snapshot_sequence", 360, UINT32, 2),
    ("generation", 368, UINT32, 2),
    ("horizon_ns", 376, UINT32, 1),
    ("validity", 380, UINT8, 1),
)
EXPECTED_POINT_STEP = 384

VALIDITY_NOT_EVALUATED = 0
VALIDITY_VALID = 1
VALIDITY_NAMES = {VALIDITY_NOT_EVALUATED: "NOT_EVALUATED", VALIDITY_VALID: "VALID"}


class LayoutMismatch(ValueError):
    """The message does not carry the promised layout. Refuse, do not guess."""


def check_layout(
    fields: list[tuple[str, int, int, int]], point_step: int, is_bigendian: bool
) -> str | None:
    """None when the layout is exactly the promised one, else the first difference.

    Exact, not "contains": an extra field that shifted nothing would be harmless,
    but one that moved ``validity`` is not, and the cheap way to be sure is to
    require the whole table. The README says a consumer that finds a difference
    drops the message, and that is what the decoder does with this reason.
    """
    if is_bigendian:
        return "is_bigendian is set; the layout is promised little-endian"
    if point_step != EXPECTED_POINT_STEP:
        return f"point_step {point_step} != {EXPECTED_POINT_STEP}"
    by_name = {f[0]: f for f in fields}
    for name, offset, datatype, count in EXPECTED_FIELDS:
        got = by_name.get(name)
        if got is None:
            return f"field '{name}' missing"
        if got[1] != offset or got[2] != datatype or got[3] != count:
            return (
                f"field '{name}' is (offset {got[1]}, datatype {got[2]}, count {got[3]}), "
                f"expected ({offset}, {datatype}, {count})"
            )
    extra = set(by_name) - {f[0] for f in EXPECTED_FIELDS}
    if extra:
        return f"unexpected field(s) {sorted(extra)}"
    return None


@dataclass(frozen=True)
class TrajectoryPoint:
    position_m: tuple[float, float, float]
    velocity_m_s: tuple[float, float, float]
    acceleration_m_s2: tuple[float, float, float]
    covariance: tuple[float, ...]  # 36, row-major px..pz,vx..vz; NaN = unknown
    snapshot_sequence: int
    generation: int
    horizon_ns: int
    validity: int


@dataclass(frozen=True)
class TrajectorySnapshot:
    stamp_ns: int
    frame_id: str
    points: tuple[TrajectoryPoint, ...]

    @property
    def n_points(self) -> int:
        return len(self.points)

    @property
    def validities(self) -> set[int]:
        return {p.validity for p in self.points}

    @property
    def horizon_last_ns(self) -> int | None:
        return self.points[-1].horizon_ns if self.points else None


def _read(fmt: str, data: bytes, offset: int, count: int):
    return struct.unpack_from("<" + fmt * count, data, offset)


def decode_trajectory(
    *,
    stamp_ns: int,
    frame_id: str,
    fields: list[tuple[str, int, int, int]],
    point_step: int,
    is_bigendian: bool,
    width: int,
    height: int,
    data: bytes,
) -> TrajectorySnapshot:
    """Decode one prediction snapshot. Raises LayoutMismatch rather than guessing.

    ``width`` is the point count N; ``height`` must be 1. An INVALID snapshot is
    published with zero points and the field table intact — that decodes to an
    empty snapshot, not an error, because "no prediction right now" is a value.
    """
    reason = check_layout(fields, point_step, is_bigendian)
    if reason is not None:
        raise LayoutMismatch(reason)
    if height != 1:
        raise LayoutMismatch(f"height {height} != 1")
    if len(data) != width * point_step:
        raise LayoutMismatch(f"data length {len(data)} != width {width} * point_step {point_step}")

    points = []
    for i in range(width):
        base = i * point_step
        x, y, z, vx, vy, vz, ax, ay, az = _read("d", data, base + 0, 9)
        cov = _read("d", data, base + 72, 36)
        seq_lo, seq_hi = _read("I", data, base + 360, 2)
        gen_lo, gen_hi = _read("I", data, base + 368, 2)
        (horizon,) = _read("I", data, base + 376, 1)
        (validity,) = _read("B", data, base + 380, 1)
        points.append(
            TrajectoryPoint(
                position_m=(x, y, z),
                velocity_m_s=(vx, vy, vz),
                acceleration_m_s2=(ax, ay, az),
                covariance=cov,
                snapshot_sequence=seq_lo | (seq_hi << 32),
                generation=gen_lo | (gen_hi << 32),
                horizon_ns=horizon,
                validity=validity,
            )
        )
    return TrajectorySnapshot(stamp_ns=stamp_ns, frame_id=frame_id, points=tuple(points))


# ── CSV contract shared with the probe ──────────────────────────────────────

PREDICTION_COLUMNS = (
    "recv_ns",  # steady clock at receipt, probe-side
    "sub",  # which subscription received it: best_effort | reliable
    "stamp_ns",  # header.stamp (= origin time of the snapshot)
    "frame_id",
    "n_points",
    "validity",  # "VALID" | "NOT_EVALUATED" | "MIXED" | "" (no points)
    "snapshot_sequence",
    "generation",
    "horizon_first_ns",
    "horizon_last_ns",
    "x0",
    "y0",
    "z0",  # first predicted point, for a sanity plot
    "cov_nan",  # 1 if any covariance entry of the first point is NaN
)

CAMERA_COLUMNS = ("recv_ns", "stamp_ns", "frame_id", "x", "y", "z")
TRUTH_COLUMNS = (
    "recv_ns",
    "stamp_ns",
    "frame_id",
    "child_frame_id",
    "x",
    "y",
    "z",
    "vx",
    "vy",
    "vz",
)
DIAG_COLUMNS = ("recv_ns", "status", "key", "value")


def snapshot_row(recv_ns: int, sub: str, snap: TrajectorySnapshot) -> dict[str, object]:
    vals = snap.validities
    if not vals:
        validity = ""
    elif len(vals) > 1:
        validity = "MIXED"
    else:
        validity = VALIDITY_NAMES.get(next(iter(vals)), str(next(iter(vals))))
    first = snap.points[0] if snap.points else None
    return {
        "recv_ns": recv_ns,
        "sub": sub,
        "stamp_ns": snap.stamp_ns,
        "frame_id": snap.frame_id,
        "n_points": snap.n_points,
        "validity": validity,
        "snapshot_sequence": first.snapshot_sequence if first else "",
        "generation": first.generation if first else "",
        "horizon_first_ns": first.horizon_ns if first else "",
        "horizon_last_ns": snap.horizon_last_ns if first else "",
        "x0": first.position_m[0] if first else "",
        "y0": first.position_m[1] if first else "",
        "z0": first.position_m[2] if first else "",
        "cov_nan": int(any(math.isnan(c) for c in first.covariance)) if first else "",
    }


# ── Summaries ───────────────────────────────────────────────────────────────


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def quantile(values: list[float], q: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, int(math.ceil(q * len(ordered)) - 1)))
    return ordered[idx]


@dataclass
class LaneSummary:
    messages: int = 0
    by_sub: Counter = field(default_factory=Counter)
    frame_ids: Counter = field(default_factory=Counter)
    n_points: Counter = field(default_factory=Counter)
    validity: Counter = field(default_factory=Counter)
    horizon_last_s: Counter = field(default_factory=Counter)
    horizon_first_s: Counter = field(default_factory=Counter)
    # Inter-arrival on ONE subscription, seconds, only while snapshots are
    # non-empty (an empty INVALID snapshot after a flight is not "the rate").
    period_s: list[float] = field(default_factory=list)
    sequence_rewinds: int = 0
    generation_changes: int = 0
    cov_nan_snapshots: int = 0

    def rate_hz(self) -> tuple[float, float, float]:
        if not self.period_s:
            return (math.nan, math.nan, math.nan)
        return (
            1.0 / quantile(self.period_s, 0.5),
            1.0 / quantile(self.period_s, 0.05),
            1.0 / quantile(self.period_s, 0.95),
        )


def summarise_prediction(path: Path, sub: str = "best_effort") -> LaneSummary:
    rows = _read_csv(path)
    out = LaneSummary()
    prev_recv = None
    prev_seq = None
    prev_gen = None
    for row in rows:
        out.messages += 1
        out.by_sub[row["sub"]] += 1
        if row["sub"] != sub:
            continue
        out.frame_ids[row["frame_id"]] += 1
        n = int(row["n_points"])
        out.n_points[n] += 1
        out.validity[row["validity"] or "(empty)"] += 1
        if n > 0:
            out.horizon_last_s[round(int(row["horizon_last_ns"]) * 1e-9, 3)] += 1
            out.horizon_first_s[round(int(row["horizon_first_ns"]) * 1e-9, 3)] += 1
            recv = int(row["recv_ns"])
            if prev_recv is not None:
                out.period_s.append((recv - prev_recv) * 1e-9)
            prev_recv = recv
            seq = int(row["snapshot_sequence"])
            gen = int(row["generation"])
            if prev_seq is not None and seq < prev_seq:
                out.sequence_rewinds += 1
            if prev_gen is not None and gen != prev_gen:
                out.generation_changes += 1
            prev_seq, prev_gen = seq, gen
            if row["cov_nan"] == "1":
                out.cov_nan_snapshots += 1
        else:
            # A gap in the flight resets the period baseline.
            prev_recv = None
    return out


@dataclass
class GhostSummary:
    """What the lane does after the measurement stops (TBD-VIS-07).

    For every prediction snapshot, the age of the newest camera message that
    preceded it. A VALID snapshot published long after the last measurement is
    a ghost track: fresh stamp, small innovation, and nothing behind it.
    """

    valid_after_loss_max_s: float = math.nan
    valid_after_loss_p95_s: float = math.nan
    valid_snapshots: int = 0
    # Time from the last camera message of a loss episode to the first
    # NOT-VALID (empty or NOT_EVALUATED) snapshot — how long the ghost lives.
    loss_to_invalid_s: list[float] = field(default_factory=list)


def summarise_ghost(
    prediction_csv: Path, camera_csv: Path, sub: str = "best_effort", loss_gap_s: float = 0.3
) -> GhostSummary:
    preds = [r for r in _read_csv(prediction_csv) if r["sub"] == sub]
    cams = _read_csv(camera_csv)
    cam_recv = sorted(int(r["recv_ns"]) for r in cams)
    out = GhostSummary()
    if not cam_recv or not preds:
        return out

    ages_valid: list[float] = []
    for row in preds:
        recv = int(row["recv_ns"])
        i = bisect.bisect_right(cam_recv, recv)
        if i == 0:
            continue
        age = (recv - cam_recv[i - 1]) * 1e-9
        if row["validity"] == "VALID":
            ages_valid.append(age)
    out.valid_snapshots = len(ages_valid)
    if ages_valid:
        out.valid_after_loss_max_s = max(ages_valid)
        out.valid_after_loss_p95_s = quantile(ages_valid, 0.95)

    # Loss episodes: a gap between consecutive camera messages longer than
    # loss_gap_s. For each, find the first non-VALID prediction after the gap's
    # start and record the delay.
    pred_sorted = sorted(preds, key=lambda r: int(r["recv_ns"]))
    for a, b in zip(cam_recv, cam_recv[1:], strict=False):
        if (b - a) * 1e-9 <= loss_gap_s:
            continue
        for row in pred_sorted:
            recv = int(row["recv_ns"])
            if recv <= a:
                continue
            if recv >= b:
                break
            if row["validity"] != "VALID":
                out.loss_to_invalid_s.append((recv - a) * 1e-9)
                break
    # Also the tail after the last camera message.
    last = cam_recv[-1]
    for row in pred_sorted:
        recv = int(row["recv_ns"])
        if recv > last and row["validity"] != "VALID":
            out.loss_to_invalid_s.append((recv - last) * 1e-9)
            break
    return out


@dataclass
class ReliabilitySummary:
    """Did a best-effort KEEP_LAST(1) subscription miss anything (TBD-VIS-08)?

    Compared by (stamp_ns, snapshot_sequence) identity, not by count: two
    subscriptions can receive the same number of messages and different ones.
    """

    reliable: int = 0
    best_effort: int = 0
    only_reliable: int = 0
    only_best_effort: int = 0


def summarise_reliability(path: Path) -> ReliabilitySummary:
    rows = _read_csv(path)
    keys = {"reliable": set(), "best_effort": set()}
    for row in rows:
        keys[row["sub"]].add((row["stamp_ns"], row["snapshot_sequence"], row["n_points"]))
    out = ReliabilitySummary()
    out.reliable = len(keys["reliable"])
    out.best_effort = len(keys["best_effort"])
    out.only_reliable = len(keys["reliable"] - keys["best_effort"])
    out.only_best_effort = len(keys["best_effort"] - keys["reliable"])
    return out


@dataclass
class DiagSummary:
    clock_reset_seen: bool = False
    max_dropped_after_clock_reset: int = 0
    max_input_rejected_after_clock_reset: int = 0
    generations: set[int] = field(default_factory=set)
    snapshot_sequence_min: int | None = None
    snapshot_sequence_max: int | None = None
    keys: Counter = field(default_factory=Counter)


def summarise_diagnostics(path: Path) -> DiagSummary:
    out = DiagSummary()
    for row in _read_csv(path):
        key, value = row["key"], row["value"]
        out.keys[key] += 1
        if key == "clock_reset" and value == "true":
            out.clock_reset_seen = True
        elif key == "dropped_after_clock_reset":
            out.max_dropped_after_clock_reset = max(out.max_dropped_after_clock_reset, int(value))
        elif key == "input_rejected_after_clock_reset":
            out.max_input_rejected_after_clock_reset = max(
                out.max_input_rejected_after_clock_reset, int(value)
            )
        elif key == "generation":
            out.generations.add(int(value))
        elif key == "snapshot_sequence":
            v = int(value)
            out.snapshot_sequence_min = (
                v if out.snapshot_sequence_min is None else min(out.snapshot_sequence_min, v)
            )
            out.snapshot_sequence_max = (
                v if out.snapshot_sequence_max is None else max(out.snapshot_sequence_max, v)
            )
    return out


def truth_trajectory(path: Path) -> list[tuple[float, float, float]]:
    """Positions in publish order — the e2e gate compares two of these."""
    return [(float(r["x"]), float(r["y"]), float(r["z"])) for r in _read_csv(path)]


def first_flight(
    positions: list[tuple[float, float, float]], recv_ns: list[int], gap_s: float = 0.3
) -> list[tuple[float, float, float]]:
    """The positions up to the first receive gap longer than gap_s."""
    out = []
    for i, p in enumerate(positions):
        if i > 0 and (recv_ns[i] - recv_ns[i - 1]) * 1e-9 > gap_s:
            break
        out.append(p)
    return out


@dataclass(frozen=True)
class DetectionLatency:
    """One flight's T_det: launch → the first VALID prediction (plan §7.3, S3.6).

    Two axes, each computed WITHIN one clock, never across the two (D-2):

    - ``t_det_recv_s`` — probe-side steady receive of the first ground-truth
      sample of the flight → probe-side steady receive of the first VALID
      prediction. One clock by construction (the probe's), transport included.
    - ``t_det_stamp_s`` — the two messages' own header stamps. Transport is
      excluded, so the pair brackets T_det. This axis is ONE clock only under
      the sim rig's precondition: the simulator stamps ground truth and the
      camera sample in the same statement (``PublishProjectileBall``), and the
      estimator copies that capture stamp onto the prediction it produces
      (``stamp_is_capture_time``, measured in S3.4). If a producer ever stamps
      predictions with its own clock instead, this axis becomes a cross-clock
      subtraction and must not be read — the recv axis stays valid either way.

    The launch instant is the flight's FIRST ground-truth sample because the
    simulator publishes nothing while the ball is parked
    (``PublishProjectileBall``), so the truth lane's silence separates flights.
    Its quantisation is one ball sample period (``publish.sample_rate_hz``).

    ``first_valid_*`` are NaN/-1 for a flight in which no VALID prediction
    arrived — that flight is reported, not dropped, because a silent flight is
    the observation that matters most.
    """

    flight_index: int
    launch_recv_ns: int
    launch_stamp_ns: int
    first_valid_recv_ns: int
    first_valid_stamp_ns: int
    t_det_recv_s: float
    t_det_stamp_s: float
    predictions_before_valid: int


def flight_spans(recv_ns: Sequence[int], gap_s: float = 0.3) -> list[tuple[int, int]]:
    """Index ranges [start, end] of each flight, split at receive gaps > gap_s."""
    spans: list[tuple[int, int]] = []
    start = 0
    for i in range(1, len(recv_ns)):
        if (recv_ns[i] - recv_ns[i - 1]) * 1e-9 > gap_s:
            spans.append((start, i - 1))
            start = i
    if recv_ns:
        spans.append((start, len(recv_ns) - 1))
    return spans


def detection_latencies(
    truth_csv: Path, pred_csv: Path, sub: str = "best_effort", gap_s: float = 0.3
) -> list[DetectionLatency]:
    """T_det per flight. Flights come from the truth lane, predictions from one sub."""
    truth = _read_csv(truth_csv)
    if not truth:
        return []
    truth_recv = [int(r["recv_ns"]) for r in truth]
    preds = [r for r in _read_csv(pred_csv) if r["sub"] == sub]
    generations = [(int(r["recv_ns"]), r["generation"]) for r in preds if r["generation"] != ""]
    spans = flight_spans(truth_recv, gap_s)
    out: list[DetectionLatency] = []
    for k, (start, _end) in enumerate(spans):
        launch_recv = truth_recv[start]
        launch_stamp = int(truth[start]["stamp_ns"])
        # The next launch bounds this flight: a prediction that arrived after it
        # belongs to the next flight, not to a late detection of this one.
        limit = truth_recv[spans[k + 1][0]] if k + 1 < len(spans) else None
        # A track that was already VALID before this launch is a ghost (TBD-VIS-07),
        # not a detection of this flight, and it carries the generation it had then.
        # Requiring a generation never seen before the launch is what separates the
        # two; a new flight forces a new track, so it always has one.
        seen = {g for recv, g in generations if recv < launch_recv}
        before = 0
        hit = None
        for r in preds:
            recv = int(r["recv_ns"])
            if recv < launch_recv or (limit is not None and recv >= limit):
                continue
            # Both comparisons stay inside one clock; a message stamped before
            # this launch is the previous flight's tail however late it arrived.
            if int(r["stamp_ns"]) < launch_stamp:
                continue
            if r["validity"] == "VALID" and r["generation"] not in seen:
                hit = r
                break
            before += 1
        out.append(
            DetectionLatency(
                flight_index=k,
                launch_recv_ns=launch_recv,
                launch_stamp_ns=launch_stamp,
                first_valid_recv_ns=int(hit["recv_ns"]) if hit else -1,
                first_valid_stamp_ns=int(hit["stamp_ns"]) if hit else -1,
                t_det_recv_s=(int(hit["recv_ns"]) - launch_recv) * 1e-9 if hit else math.nan,
                t_det_stamp_s=(int(hit["stamp_ns"]) - launch_stamp) * 1e-9 if hit else math.nan,
                predictions_before_valid=before,
            )
        )
    return out


def max_abs_difference(
    a: list[tuple[float, float, float]], b: list[tuple[float, float, float]]
) -> tuple[int, float]:
    """(compared length, max |a-b| over that length). Length mismatch is reported
    by the caller; comparing the overlap alone would hide a shorter flight."""
    n = min(len(a), len(b))
    worst = 0.0
    for i in range(n):
        for k in range(3):
            worst = max(worst, abs(a[i][k] - b[i][k]))
    return n, worst


# ── CLI ─────────────────────────────────────────────────────────────────────


def _fmt_counter(c: Counter, limit: int = 8) -> str:
    items = sorted(c.items(), key=lambda kv: (-kv[1], str(kv[0])))[:limit]
    return ", ".join(f"{k}×{v}" for k, v in items) or "(none)"


def main(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("prefix", help="CSV prefix the probe wrote (<prefix>_prediction.csv …)")
    parser.add_argument("--sub", default="best_effort", choices=("best_effort", "reliable"))
    parser.add_argument("--loss-gap-s", type=float, default=0.3)
    parser.add_argument(
        "--flight-gap-s",
        type=float,
        default=0.3,
        help="truth-lane silence that separates two flights (T_det); not the camera-loss gap",
    )
    args = parser.parse_args(argv)

    prefix = Path(args.prefix)
    pred_csv = Path(f"{prefix}_prediction.csv")
    cam_csv = Path(f"{prefix}_camera.csv")
    diag_csv = Path(f"{prefix}_diag.csv")
    if not pred_csv.exists():
        print(f"missing {pred_csv}", file=sys.stderr)
        return 2

    lane = summarise_prediction(pred_csv, args.sub)
    rate_p50, rate_lo, rate_hi = lane.rate_hz()
    print(f"prediction lane : {pred_csv}  ({lane.messages} rows, by sub {dict(lane.by_sub)})")
    print(f"  summarised sub          : {args.sub}")
    print(f"  frame_id                : {_fmt_counter(lane.frame_ids)}   (TBD-VIS-06)")
    print(f"  N (points)              : {_fmt_counter(lane.n_points)}   (TBD-VIS-04)")
    print(
        f"  horizon first / last [s]: {_fmt_counter(lane.horizon_first_s)} / "
        f"{_fmt_counter(lane.horizon_last_s)}"
    )
    print(
        f"  rate while non-empty    : p50 {rate_p50:.1f} Hz  (p05 {rate_lo:.1f}, p95 {rate_hi:.1f})"
    )
    print(f"  validity                : {_fmt_counter(lane.validity)}")
    print(f"  covariance NaN snapshots: {lane.cov_nan_snapshots}")
    print(
        f"  snapshot_sequence rewinds / generation changes: "
        f"{lane.sequence_rewinds} / {lane.generation_changes}"
    )

    rel = summarise_reliability(pred_csv)
    print(
        f"  reliability (TBD-VIS-08): reliable {rel.reliable}, best_effort {rel.best_effort}, "
        f"only-reliable {rel.only_reliable}, only-best_effort {rel.only_best_effort}"
    )

    if cam_csv.exists():
        ghost = summarise_ghost(pred_csv, cam_csv, args.sub, args.loss_gap_s)
        print(
            f"  ghost (TBD-VIS-07)      : VALID snapshots {ghost.valid_snapshots}; age of newest "
            f"camera msg at a VALID snapshot max {ghost.valid_after_loss_max_s * 1e3:.1f} ms, "
            f"p95 {ghost.valid_after_loss_p95_s * 1e3:.1f} ms"
        )
        if ghost.loss_to_invalid_s:
            print(
                f"    loss → first non-VALID  : "
                f"{', '.join(f'{v * 1e3:.0f}' for v in ghost.loss_to_invalid_s)} ms"
            )
        else:
            print("    loss → first non-VALID  : (no loss episode observed)")

    if diag_csv.exists():
        diag = summarise_diagnostics(diag_csv)
        print(
            f"  diagnostics             : clock_reset seen {diag.clock_reset_seen}; "
            f"dropped_after_clock_reset max {diag.max_dropped_after_clock_reset}; "
            f"input_rejected_after_clock_reset max {diag.max_input_rejected_after_clock_reset}; "
            f"generations {sorted(diag.generations)}; snapshot_sequence "
            f"{diag.snapshot_sequence_min}..{diag.snapshot_sequence_max}"
        )
    truth_csv = Path(f"{prefix}_truth.csv")
    if truth_csv.exists():
        lat = detection_latencies(truth_csv, pred_csv, args.sub, args.flight_gap_s)
        seen = [d.t_det_recv_s for d in lat if not math.isnan(d.t_det_recv_s)]
        stamped = [d.t_det_stamp_s for d in lat if not math.isnan(d.t_det_stamp_s)]
        silent = [d.flight_index for d in lat if math.isnan(d.t_det_recv_s)]
        print()
        print(f"T_det (plan §7.3)  : {len(lat)} flights, {len(seen)} with a VALID prediction")
        if seen:
            print(
                f"  recv axis  [s]          : min {min(seen):.3f}  p50 {quantile(seen, 0.5):.3f}  "
                f"p95 {quantile(seen, 0.95):.3f}  max {max(seen):.3f}"
            )
            print(
                f"  stamp axis [s]          : min {min(stamped):.3f}  p50 "
                f"{quantile(stamped, 0.5):.3f}  p95 {quantile(stamped, 0.95):.3f}  "
                f"max {max(stamped):.3f}"
            )
            print(
                "  predictions before VALID: "
                f"{_fmt_counter(Counter(d.predictions_before_valid for d in lat if not math.isnan(d.t_det_recv_s)))}"
            )
        if silent:
            print(f"  flights with NO VALID   : {silent}")
        print(
            "  launch instant = the flight's first ground-truth sample (the ball lane is "
            "silent while parked);"
        )
        print("  its quantisation is one ball sample period (publish.sample_rate_hz).")
    print()
    print("  This is a MEASUREMENT (S3.4). No threshold is applied here; the policy for")
    print("  rewinds, partial validity and ghost tracks is S5.2's (plan §4.4, L1 §10).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
