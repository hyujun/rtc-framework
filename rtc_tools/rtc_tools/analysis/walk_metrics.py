"""Walking metrics of one locomotion session, from its ``locomotion_diag.csv``.

A walking run follows a velocity command script whose segments are known in
length but not in time: the log's own command column places them. The
segment table (:data:`WALK_SCRIPT`) is a ramp, a hold, a yaw step, a hold and a
stop; ``a`` is the first tick whose ``cmd_vx`` exceeds ``start_vx`` and every
segment up to the stop is ``a`` plus the lengths before it; the stop starts at
the first tick after ``a`` whose ``cmd_vx`` falls back below ``stop_vx`` (its
offset from the scheduled stop is the filter's delay, reported).

Every quantity is on the controller's time axis (``t_relative_s``: the
simulator's time in a synchronised run). The measured planar velocity is the
gait scheduler's own (:func:`planar_velocity`): the base-frame linear velocity
rotated to the world and expressed on the yaw-aligned axes, and the world yaw
rate of the base-frame angular velocity — so the gait switches and these
numbers see the same quantity.

The columns read: ``t_relative_s``, ``tick``, ``mode``, ``fault_latched``,
``fault_count``, ``fault_cause``, ``pose_guard_mask``, ``source_status``,
``health``, ``tau_valid``, ``qp_status``, ``rows_dropped``, ``eom_residual_inf``,
the ``margin_*`` columns with ``margins_stale``, ``base_p*``, ``base_q*``,
``base_v*``, ``base_w*``, ``com_x``, ``com_y``, ``ref_source``,
``plausibility_rejects``, ``contact_mask``, ``policy_age_us``, ``wbc_tick_us``,
``cmd_vx``, ``cmd_vy``, ``cmd_wz``, ``cmd_valid``, ``gait_latest``,
``ref_base_v{x,y,z}``, ``meas_contact_valid`` and ``meas_fz_<contact>`` (contact
order = the bits of ``contact_mask``, read from the header).

Pure Python plus numpy: no ROS, no robot constant (the weight is the run's own
standing load, the gait order is the caller's).
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from rtc_tools.analysis.locomotion_diag import (
    FAULT_CAUSE_NAMES,
    HEALTH_NAMES,
    MARGINS,
    MODE_NAMES,
    REFERENCE_SOURCE_NAMES,
)

Table = dict[str, np.ndarray]

# The log's codes, from the name tables the C++ enums are pinned to.
REF_SOURCE_IDLE = REFERENCE_SOURCE_NAMES.index("idle")
REF_SOURCE_STAND = REFERENCE_SOURCE_NAMES.index("stand")
MODE_DEGRADED = MODE_NAMES.index("degraded")
HEALTH_FAULT = HEALTH_NAMES.index("fault")
FAULT_CAUSE_PLAN_LOSS = FAULT_CAUSE_NAMES.index("plan loss")
QP_INFEASIBLE_STATUSES = (2, 3)  # proxsuite QPSolverOutput PRIMAL_INFEASIBLE, DUAL_INFEASIBLE


@dataclass(frozen=True)
class Segment:
    name: str
    length_s: float
    vx0: float  # v_x at the segment's start; linear to vx1 at its end
    vx1: float
    wz: float


# The walking command script [s, m/s, rad/s]: ramp 0 → 0.5 m/s, hold, a yaw
# rate step, hold, stop.
WALK_SCRIPT = (
    Segment("A", 2.0, 0.0, 0.5, 0.0),
    Segment("B", 8.0, 0.5, 0.5, 0.0),
    Segment("C", 4.0, 0.5, 0.5, 0.3),
    Segment("D", 2.0, 0.5, 0.5, 0.0),
    Segment("E", 10.0, 0.0, 0.0, 0.0),
)


def _turn(wz: float) -> tuple[Segment, ...]:
    # Turning in place: +wz, -wz, twice, 4 s each (16 s of walking, as in
    # WALK_SCRIPT, net heading 0), then the same 10 s stop.
    return (
        Segment("A", 4.0, 0.0, 0.0, wz),
        Segment("B", 4.0, 0.0, 0.0, -wz),
        Segment("C", 4.0, 0.0, 0.0, wz),
        Segment("D", 4.0, 0.0, 0.0, -wz),
        Segment("E", 10.0, 0.0, 0.0, 0.0),
    )


def _curve(wz: float) -> tuple[Segment, ...]:
    # Walking with a left and a right turn: the WALK_SCRIPT ramp, a hold, +wz
    # and -wz for 3 s each at 0.5 m/s, a hold, the same 10 s stop.
    return (
        Segment("A", 2.0, 0.0, 0.5, 0.0),
        Segment("B", 4.0, 0.5, 0.5, 0.0),
        Segment("C", 3.0, 0.5, 0.5, wz),
        Segment("D", 3.0, 0.5, 0.5, -wz),
        Segment("E", 4.0, 0.5, 0.5, 0.0),
        Segment("F", 10.0, 0.0, 0.0, 0.0),
    )


# The command scripts a run can publish, by name. Every one ends in a 10 s
# stop and is as long as WALK_SCRIPT, so a run's length and stop window do not
# depend on the script. walk_summary judges WALK_SCRIPT only (its segments by
# name); the others are inputs for the estimator evaluation (WE-F05c — a
# turn in place and a walk with turns both ways; the "_slow" pair is its
# pre-registered fallback should the 0.4 rad/s turns not survive).
COMMAND_SCRIPTS: dict[str, tuple[Segment, ...]] = {
    "walk": WALK_SCRIPT,
    "turn": _turn(0.4),
    "curve": _curve(0.4),
    "turn_slow": _turn(0.3),
    "curve_slow": _curve(0.3),
}


def script_length(script: Sequence[Segment] = WALK_SCRIPT) -> float:
    """The script's length [s]."""
    return sum(s.length_s for s in script)


def segment_at(script: Sequence[Segment], t: float) -> str | None:
    """The segment `t` seconds after the script's start; None outside it."""
    if t < 0.0:
        return None
    start = 0.0
    for seg in script:
        if t < start + seg.length_s:
            return seg.name
        start += seg.length_s
    return None


def command_at(script: Sequence[Segment], t: float) -> tuple[float, float, float]:
    """(v_x, v_y, ω_z) of the script `t` seconds after its start; 0 past its end."""
    if t < 0.0:
        return 0.0, 0.0, 0.0
    start = 0.0
    for seg in script:
        if t < start + seg.length_s:
            f = (t - start) / seg.length_s if seg.length_s > 0.0 else 1.0
            return seg.vx0 + f * (seg.vx1 - seg.vx0), 0.0, seg.wz
        start += seg.length_s
    return 0.0, 0.0, 0.0


# ── kinematics ────────────────────────────────────────────────────────────


def rotation_matrices(qx, qy, qz, qw) -> np.ndarray:
    """(n, 3, 3) rotations of (x, y, z, w) quaternions, normalised first."""
    q = np.stack([np.asarray(a, dtype=float) for a in (qx, qy, qz, qw)], axis=-1)
    norm = np.linalg.norm(q, axis=-1, keepdims=True)
    q = np.divide(q, norm, out=np.full_like(q, math.nan), where=norm > 0.0)
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    r = np.empty((*x.shape, 3, 3))
    r[..., 0, 0] = 1 - 2 * (y * y + z * z)
    r[..., 0, 1] = 2 * (x * y - z * w)
    r[..., 0, 2] = 2 * (x * z + y * w)
    r[..., 1, 0] = 2 * (x * y + z * w)
    r[..., 1, 1] = 1 - 2 * (x * x + z * z)
    r[..., 1, 2] = 2 * (y * z - x * w)
    r[..., 2, 0] = 2 * (x * z - y * w)
    r[..., 2, 1] = 2 * (y * z + x * w)
    r[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return r


def yaw(qx, qy, qz, qw) -> np.ndarray:
    """atan2(R10, R00) — the heading the scheduler's yaw-aligned axes use."""
    r = rotation_matrices(qx, qy, qz, qw)
    return np.arctan2(r[..., 1, 0], r[..., 0, 0])


def planar_velocity(q_xyzw, v_lin_b, w_b) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The gait scheduler's measured velocity, per row.

    ``v_world = R v_b``, ``ψ = atan2(R10, R00)``, (v_x, v_y) = v_world on the
    axes rotated by ψ, ω_z = (R ω_b).z. A zero quaternion gives 0 (as the
    scheduler does); a non-finite one gives NaN.
    """
    qx, qy, qz, qw = (np.atleast_1d(np.asarray(a, dtype=float)) for a in q_xyzw)
    zero = (qx * qx + qy * qy + qz * qz + qw * qw) == 0.0
    r = rotation_matrices(qx, qy, qz, qw)
    v = np.stack([np.atleast_1d(np.asarray(a, dtype=float)) for a in v_lin_b], axis=-1)
    w = np.stack([np.atleast_1d(np.asarray(a, dtype=float)) for a in w_b], axis=-1)
    v_world = np.einsum("nij,nj->ni", r, v)
    w_world = np.einsum("nij,nj->ni", r, w)
    psi = np.arctan2(r[:, 1, 0], r[:, 0, 0])
    c, s = np.cos(psi), np.sin(psi)
    vx = c * v_world[:, 0] + s * v_world[:, 1]
    vy = -s * v_world[:, 0] + c * v_world[:, 1]
    wz = w_world[:, 2]
    for a in (vx, vy, wz):
        a[zero] = 0.0
    return vx, vy, wz


def wrap(angle):
    return (np.asarray(angle) + math.pi) % (2.0 * math.pi) - math.pi


def circular_mean(angles: np.ndarray) -> float:
    a = angles[np.isfinite(angles)]
    return float(math.atan2(np.mean(np.sin(a)), np.mean(np.cos(a)))) if a.size else math.nan


# ── segments ──────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class Segments:
    a: float  # the ramp's start (cmd_vx > start_vx)
    e: float  # the stop's start (cmd_vx back below stop_vx)
    bounds: dict[str, tuple[float, float]]  # name → [t0, t1)


def find_segments(
    t: np.ndarray,
    cmd_vx: np.ndarray,
    script: Sequence[Segment] = WALK_SCRIPT,
    start_vx: float = 0.01,
    stop_vx: float = 0.25,
) -> Segments | None:
    """The script's segments on the log's time axis (module docstring); None if absent."""
    above = np.flatnonzero(cmd_vx > start_vx)
    if above.size == 0:
        return None
    ia = int(above[0])
    a = float(t[ia])
    peaked = np.flatnonzero(cmd_vx[ia:] >= stop_vx)
    if peaked.size == 0:
        return None
    ip = ia + int(peaked[0])
    below = np.flatnonzero(cmd_vx[ip:] < stop_vx)
    if below.size == 0:
        return None
    e = float(t[ip + int(below[0])])
    bounds: dict[str, tuple[float, float]] = {}
    start = a
    for seg in script[:-1]:
        bounds[seg.name] = (start, start + seg.length_s)
        start += seg.length_s
    last = script[-1]
    bounds[last.name] = (e, e + last.length_s)
    return Segments(a=a, e=e, bounds=bounds)


def in_window(t: np.ndarray, t0: float, t1: float) -> np.ndarray:
    """[t0, t1) — half-open, so back-to-back segments share no row
    (locomotion_diag.window_mask is closed: a plot window, not a partition)."""
    return (t >= t0) & (t < t1)


# ── contacts ──────────────────────────────────────────────────────────────


def contact_names(header: Sequence[str]) -> list[str]:
    """The ``meas_fz_<c>`` block's names, in column order (= contact order)."""
    return [h[len("meas_fz_") :] for h in header if h.startswith("meas_fz_")]


def mask_bits(mask: np.ndarray, n: int) -> np.ndarray:
    """(rows, n) booleans of `mask` bit c; a NaN mask reads as all clear."""
    m = np.where(np.isfinite(mask), mask, 0).astype(np.int64)
    return np.stack([(m >> c) & 1 == 1 for c in range(n)], axis=1)


def edges(state: np.ndarray, t: np.ndarray) -> list[tuple[float, bool]]:
    """(time, rising) of every change of a boolean series."""
    s = state.astype(np.int8)
    idx = np.flatnonzero(np.diff(s) != 0) + 1
    return [(float(t[i]), bool(state[i])) for i in idx]


def measured_contact(normal: np.ndarray, on_n: float, off_n: float) -> np.ndarray:
    """In contact above `on_n`, out below `off_n`, the last state between (a NaN holds)."""
    out = np.zeros(len(normal), dtype=bool)
    readable = np.flatnonzero(~np.isnan(normal))
    state = bool(normal[readable[0]] > on_n) if readable.size else False
    for i, f in enumerate(normal):
        if not math.isnan(f):
            if f > on_n:
                state = True
            elif f < off_n:
                state = False
        out[i] = state
    return out


@dataclass
class EdgeMatch:
    dt_s: list[float]  # measured − planned, per matched pair
    missed: int  # planned edges with no measured edge of the same direction near them
    extra: int  # measured edges no planned edge claimed


def match_edges(
    planned: list[tuple[float, bool]], measured: list[tuple[float, bool]], window_s: float
) -> EdgeMatch:
    """1:1 pairing in time order: each planned edge takes the nearest unclaimed
    measured edge of the same direction within ±`window_s`."""
    used = [False] * len(measured)
    dts: list[float] = []
    missed = 0
    for tp, rising in planned:
        best, best_d = -1, math.inf
        for j, (tm, r) in enumerate(measured):
            if used[j] or r != rising:
                continue
            d = abs(tm - tp)
            if d <= window_s and d < best_d:
                best, best_d = j, d
        if best < 0:
            missed += 1
        else:
            used[best] = True
            dts.append(measured[best][0] - tp)
    return EdgeMatch(dt_s=dts, missed=missed, extra=used.count(False))


def steps(bits: np.ndarray, t: np.ndarray, t0: float, t1: float, hold_ticks: int = 3) -> int:
    """Touch-downs: per contact, 0 → 1 edges in [t0, t1) whose 1 lasts ≥ `hold_ticks` rows."""
    n = 0
    for c in range(bits.shape[1]):
        b = bits[:, c]
        for i in np.flatnonzero(~b[:-1] & b[1:]) + 1:
            if t0 <= t[i] < t1 and i + hold_ticks <= len(b) and b[i : i + hold_ticks].all():
                n += 1
    return n


# ── the summary ───────────────────────────────────────────────────────────


# The reductions leave out NaN only: an infinite value reaches the caller's
# check (and fails it) instead of vanishing with the NaNs.


def _rms(x: np.ndarray) -> float:
    f = x[~np.isnan(x)]
    return float(np.sqrt(np.mean(f * f))) if f.size else math.nan


def _mean(x: np.ndarray) -> float:
    f = x[~np.isnan(x)]
    return float(np.mean(f)) if f.size else math.nan


def _pct(x: np.ndarray, q: float) -> float | None:
    f = x[~np.isnan(x)]
    return float(np.percentile(f, q)) if f.size else None


def _finite_fraction(x: np.ndarray) -> float:
    return float(np.mean(np.isfinite(x))) if x.size else 0.0


def _bad(ok: np.ndarray) -> np.ndarray:
    """The rows a predicate does not pass — a NaN compares false, so it is one."""
    return ~ok


def _increase(col: np.ndarray, mask: np.ndarray) -> int:
    """How much a running total grew over the rows of `mask`."""
    v = col[mask & np.isfinite(col)]
    return int(v.max() - v.min()) if v.size else 0


# The tracking and heading windows in walk cycles: v_x over the last cycles of
# B, the yaw rate over the last of C, the heading reference at B's start / end.
VX_WINDOW_CYCLES = 4
YAW_WINDOW_CYCLES = 2
HEADING_REF_CYCLES = 1
# The two heading references sit in B with the v_x window's length to spare, so
# fitting the v_x window fits them (windows_fit checks only v_x and yaw).
assert 2 * HEADING_REF_CYCLES <= VX_WINDOW_CYCLES


def window_lengths(walk_period_s: float) -> tuple[float, float, float]:
    """(v_x window, yaw-rate window, heading reference) [s] for a walk cycle of `walk_period_s`."""
    if not (math.isfinite(walk_period_s) and walk_period_s > 0.0):
        raise ValueError(f"walk_period_s must be finite and > 0, got {walk_period_s}")
    return (
        VX_WINDOW_CYCLES * walk_period_s,
        YAW_WINDOW_CYCLES * walk_period_s,
        HEADING_REF_CYCLES * walk_period_s,
    )


def windows_fit(walk_period_s: float, script: Sequence[Segment] = WALK_SCRIPT) -> str | None:
    """None when every window of `walk_period_s` fits its script segment, else why not.

    A window longer than its segment would read the segment before it: the v_x
    window (and with it the two heading references) in B, the yaw-rate window in C.
    """
    vx, yaw, ref = window_lengths(walk_period_s)
    length = {seg.name: seg.length_s for seg in script}
    if vx > length["B"] or yaw > length["C"]:
        return (
            f"a walk cycle of {walk_period_s} s gives windows ({vx}, {yaw}, {ref}) s "
            f"longer than B ({length['B']} s) or C ({length['C']} s)"
        )
    return None


def walk_summary(
    header: Sequence[str],
    diag: Table,
    gaits: Sequence[str],
    walk_gait: str = "walk",
    stance_gait: str | None = None,
    script: Sequence[Segment] = WALK_SCRIPT,
    *,
    walk_period_s: float,
    settle_hold_s: float = 1.0,
    settle_speed: float = 0.05,
    settle_yaw_rate: float = 0.05,
    contact_on: float = 0.05,
    contact_off: float = 0.02,
    edge_window_s: float = 0.2,
    event_window_s: float = 0.01,
    walk_entry_s: float = 5.0,
) -> dict:
    """Every walking number of one run (module docstring); `gaits` = the gait table order.

    `walk_period_s` is the walk gait's cycle [s] (its last switching time): the
    tracking and heading windows are whole cycles of it (:func:`window_lengths`),
    so the step's own velocity and yaw-rate swing averages out of them.
    """
    vx_window_s, yaw_window_s, heading_ref_s = window_lengths(walk_period_s)
    misfit = windows_fit(walk_period_s, script)
    if misfit is not None:
        return {"error": misfit}
    t = diag["t_relative_s"]
    seg = find_segments(t, diag["cmd_vx"], script)
    if seg is None:
        return {"error": "the command script was not found in cmd_vx"}
    b = seg.bounds
    dt = float(np.median(np.diff(t))) if len(t) > 1 else math.nan
    run = in_window(t, seg.a, b[script[-1].name][1])  # the script's rows
    b0, d1 = b["B"][0], b["D"][1]
    walking = in_window(t, b0, d1)
    out: dict = {
        "segments": {k: list(v) for k, v in b.items()},
        "script_length_s": seg.e - seg.a,
        "script_length_nominal_s": script_length(script[:-1]),
        # The publisher's own end: the ramp's start plus the whole script (the
        # stop as scheduled, not as the filtered command shows it).
        "script_end_scheduled_s": seg.a + script_length(script),
        "dt_s": dt,
        "rows_expected": int(round((t[run][-1] - t[run][0]) / dt)) + 1 if run.any() else 0,
        "rows": int(run.sum()),
    }

    # Counters over the script.
    latched = _bad(diag["fault_latched"] < 0.5)
    non_idle = np.flatnonzero(diag["ref_source"] != REF_SOURCE_IDLE)
    after_plan = np.zeros(len(t), dtype=bool)
    if non_idle.size:
        after_plan[int(non_idle[0]) :] = True
    out["counters"] = {
        "fault_latched_rows": int((latched & run).sum()),
        "fault_count_increase": _increase(diag["fault_count"], run),
        "pose_guard_rows": int((_bad(diag["pose_guard_mask"] < 0.5) & run).sum()),
        "plan_loss_rows": int(
            (
                run
                & (
                    (after_plan & (diag["ref_source"] == REF_SOURCE_IDLE))
                    | (diag["fault_cause"] == FAULT_CAUSE_PLAN_LOSS)
                )
            ).sum()
        ),
        "plausibility_reject_increase": _increase(diag["plausibility_rejects"], run),
        "qp_infeasible_rows": int(
            (
                (np.isin(diag["qp_status"], QP_INFEASIBLE_STATUSES) | np.isnan(diag["qp_status"]))
                & run
            ).sum()
        ),
        "health_fault_rows": int((_bad(diag["health"] < HEALTH_FAULT) & run).sum()),
        "rows_dropped_rows": int((_bad(diag["rows_dropped"] <= 0) & run).sum()),
        "source_not_ok_rows": int((_bad(diag["source_status"] == 0) & run).sum()),
        "cmd_invalid_rows_b_to_d": int((_bad(diag["cmd_valid"] > 0.5) & walking).sum()),
        "degraded_fraction": float(
            np.mean((diag["mode"][run] == MODE_DEGRADED) | np.isnan(diag["mode"][run]))
        )
        if run.any()
        else math.nan,
    }

    # Gaits.
    gait = diag["gait_latest"]
    walk_idx = list(gaits).index(walk_gait)
    is_walk = gait == walk_idx
    entered = np.flatnonzero(is_walk & (t >= seg.a))
    t_walk = float(t[entered[0]]) if entered.size else None
    ramp_end = b["A"][1]
    since_walk = in_window(t, t_walk, d1) if t_walk is not None else np.zeros(len(t), bool)
    g_bd = gait[walking & np.isfinite(gait)]
    changes = np.diff(g_bd) if g_bd.size > 1 else np.zeros(0)
    out["gait"] = {
        "walk_entry_s_after_ramp_end": (t_walk - ramp_end) if t_walk is not None else None,
        "walk_entry_limit_s": walk_entry_s,
        "walk_fraction_from_entry": float(np.mean(is_walk[since_walk]))
        if since_walk.any()
        else 0.0,
        "changes_b_to_d": int(np.count_nonzero(changes)),
        "down_changes_b_to_d": int(np.count_nonzero(changes < 0)),
    }

    # The measured planar velocity (the scheduler's).
    vx, vy, wz = planar_velocity(
        (diag["base_qx"], diag["base_qy"], diag["base_qz"], diag["base_qw"]),
        (diag["base_vx"], diag["base_vy"], diag["base_vz"]),
        (diag["base_wx"], diag["base_wy"], diag["base_wz"]),
    )
    q_norm2 = sum(diag[k] ** 2 for k in ("base_qx", "base_qy", "base_qz", "base_qw"))
    no_pose = _bad(q_norm2 > 0.0)
    for a in (vx, vy, wz):
        a[no_pose] = math.nan
    vx_win = in_window(t, b["B"][1] - vx_window_s, b["B"][1])
    yaw_win = in_window(t, b["C"][1] - yaw_window_s, b["C"][1])
    seg_b = next(s for s in script if s.name == "B")
    seg_c = next(s for s in script if s.name == "C")
    out["tracking"] = {
        "vx_err_rms": _rms(vx[vx_win] - diag["cmd_vx"][vx_win]),
        "vx_mean": _mean(vx[vx_win]),
        "vx_mean_err": abs(_mean(vx[vx_win]) - seg_b.vx1),
        "yaw_rate_err_rms": _rms(wz[yaw_win] - diag["cmd_wz"][yaw_win]),
        "yaw_rate_mean": _mean(wz[yaw_win]),
        "yaw_rate_mean_err": abs(_mean(wz[yaw_win]) - seg_c.wz),
        "vx_window_finite_fraction": _finite_fraction(vx[vx_win] - diag["cmd_vx"][vx_win]),
        "yaw_window_finite_fraction": _finite_fraction(wz[yaw_win] - diag["cmd_wz"][yaw_win]),
    }

    # Lateral and heading drift over B.
    psi = yaw(diag["base_qx"], diag["base_qy"], diag["base_qz"], diag["base_qw"])
    in_b = in_window(t, *b["B"])
    ref = in_window(t, b["B"][0], b["B"][0] + heading_ref_s)
    last = in_window(t, b["B"][1] - heading_ref_s, b["B"][1])
    psi0 = circular_mean(psi[ref])
    p0 = np.array([_mean(diag["base_px"][ref]), _mean(diag["base_py"][ref])])
    normal = np.array([-math.sin(psi0), math.cos(psi0)])
    lateral = (diag["base_px"][in_b] - p0[0]) * normal[0] + (
        diag["base_py"][in_b] - p0[1]
    ) * normal[1]
    out["drift"] = {
        "lateral_drift_m": float(np.nanmax(np.abs(lateral))) if lateral.size else math.nan,
        "heading_drift_rad": abs(float(wrap(circular_mean(psi[last]) - psi0))),
    }

    # The stop.
    e0, e1 = b[script[-1].name]
    still = (
        (np.hypot(vx, vy) < settle_speed)
        & (np.abs(wz) < settle_yaw_rate)
        & (diag["ref_source"] == REF_SOURCE_STAND)
    )
    settle = None
    i0 = int(np.searchsorted(t, e0))
    run_start = None
    i1 = int(np.searchsorted(t, e1))
    for i in range(i0, i1):
        if still[i]:
            if run_start is None:
                run_start = i
            if t[i] - t[run_start] >= settle_hold_s - 1e-9:
                settle = float(t[run_start] - e0)
                break
        else:
            run_start = None
    stance_idx = list(gaits).index(stance_gait) if stance_gait is not None else 0
    to_stance = np.flatnonzero((gait == stance_idx) & (t >= e0))
    out["stop"] = {
        "stop_settle_s": settle,
        "stop_to_stance_gait_s": float(t[to_stance[0]] - e0) if to_stance.size else None,
        "filter_delay_s": seg.e - (seg.a + out["script_length_nominal_s"]),
    }

    # Contacts: the planned mask's edges against the measured normal force.
    names = contact_names(header)
    nc = len(names)
    out["contacts"] = {"names": names}
    if nc:
        bits = mask_bits(diag["contact_mask"], nc)
        normal_n = np.stack([-diag[f"meas_fz_{c}"] for c in names], axis=1)
        valid = mask_bits(diag["meas_contact_valid"], nc)
        normal_n = np.where(valid, normal_n, math.nan)
        stand_rows = (diag["ref_source"] == REF_SOURCE_STAND) & (t < seg.a) & valid.all(axis=1)
        weight = _mean(normal_n[stand_rows].sum(axis=1)) if stand_rows.any() else math.nan
        dts: list[float] = []
        missed = extra = 0
        slot_fail = single = 0
        planned_edges = [edges(bits[run, c], t[run]) for c in range(nc)]
        for c in range(nc):
            planned = planned_edges[c]
            meas_state = measured_contact(
                normal_n[run, c], contact_on * weight, contact_off * weight
            )
            m = match_edges(planned, edges(meas_state, t[run]), edge_window_s)
            dts += m.dt_s
            missed += m.missed
            extra += m.extra
        # Slot self-check: mid single support, the swinging contact unloaded and
        # the supporting one carrying more than half the weight.
        one = bits.sum(axis=1) == 1
        idx = np.flatnonzero(run)
        if idx.size:
            runs = np.split(idx, np.flatnonzero(np.diff(one[idx].astype(np.int8)) != 0) + 1)
            for r in runs:
                if len(r) < 3 or not one[r[0]]:
                    continue
                single += 1
                mid = r[len(r) // 2]
                sup = int(np.flatnonzero(bits[mid])[0])
                swing_ok = all(
                    normal_n[mid, c] < contact_on * weight for c in range(nc) if c != sup
                )
                if not (swing_ok and normal_n[mid, sup] > 0.5 * weight):
                    slot_fail += 1
        ad = np.abs(np.asarray(dts)) * 1e3
        out["contacts"].update(
            {
                "weight_n": weight,
                "matched_edges": len(dts),
                "missed_edges": missed,
                "extra_edges": extra,
                "mismatch_abs_p95_ms": _pct(ad, 95),
                "mismatch_abs_max_ms": float(ad.max()) if ad.size else None,
                "mismatch_signed_mean_ms": float(np.mean(dts) * 1e3) if dts else None,
                "single_support_phases": single,
                "slot_check_failures": slot_fail,
                "invalid_force_rows": int((~valid[run]).any(axis=1).sum()),
                "force_valid_fraction": float(np.mean(valid[run].all(axis=1)))
                if run.any()
                else 0.0,
            }
        )
        out["steps_completed"] = steps(bits, t, b0, d1)

        # Event continuity: the rows within ±event_window_s of a planned edge.
        near = np.zeros(len(t), dtype=bool)
        for planned in planned_edges:
            for te, _ in planned:
                near |= np.abs(t - te) <= event_window_s + 1e-12
        stale = _bad(diag["margins_stale"] < 0.5)
        solved = ~np.isnan(diag["eom_residual_inf"])
        judged = near & ~stale & solved
        worst_margin = {m: _min(diag[m][judged]) for m in MARGINS}
        out["events"] = {
            "rows": int(near.sum()),
            "excluded_rows": int((near & ~judged).sum()),
            "eom_residual_inf_max": _max(diag["eom_residual_inf"][judged]),
            "margin_min": worst_margin,
            "tau_invalid_rows": int((near & _bad(diag["tau_valid"] > 0.5)).sum()),
        }

    # Reported, not judged.
    first_swing = np.flatnonzero(run & (bits.sum(axis=1) < nc)) if nc else np.zeros(0, int)
    com_adv = None
    if first_swing.size:
        ia, isw = int(np.searchsorted(t, seg.a)), int(first_swing[0])
        dx = diag["com_x"][isw] - diag["com_x"][ia]
        dy = diag["com_y"][isw] - diag["com_y"][ia]
        com_adv = float(math.cos(psi[ia]) * dx + math.sin(psi[ia]) * dy)
    pvx, _, _ = planar_velocity(
        (diag["base_qx"], diag["base_qy"], diag["base_qz"], diag["base_qw"]),
        (diag["ref_base_vx"], diag["ref_base_vy"], diag["ref_base_vz"]),
        (np.zeros(len(t)),) * 3,
    )
    age = diag["policy_age_us"]
    age_p99 = _pct(age[run & (age >= 0)], 99)
    out["report"] = {
        "com_advance_before_first_swing_m": com_adv,
        "first_swing_s_after_a": float(t[first_swing[0]] - seg.a) if first_swing.size else None,
        "plan_vx_bias": _mean(pvx[vx_win] - diag["cmd_vx"][vx_win]),
        "policy_age_p99_ms": age_p99 * 1e-3 if age_p99 is not None else None,
        "wbc_tick_us_p99": _pct(diag["wbc_tick_us"][run], 99),
    }
    return out


def _min(x: np.ndarray) -> float | None:
    f = x[~np.isnan(x)]
    return float(f.min()) if f.size else None


def _max(x: np.ndarray) -> float | None:
    f = x[~np.isnan(x)]
    return float(f.max()) if f.size else None
