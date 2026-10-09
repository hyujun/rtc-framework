"""DECEL stretch metrics of catching sim trials (MPC · dual-arm plan E0-F02, #625).

The A/B of plan gate G-1 compares two ways of stopping the arm after the catch:
v1's closed-form virtual target (L7 §4.3) and the MPC tail. This tool measures
the stop itself, per trial and per unit, from what a run already logs — the
catching diag (``mode``, ``q_cmd_*``, ``q_meas_*``, ``ref_xd_*``) and the arm's
device lane (``effort_*``) — so the same numbers can be read off either arm of
the comparison.

**The window is the arm's, not the mode's.** v1 leaves ``DECEL`` the tick its
virtual target stops, while the measured arm (a servo lag behind) still moves;
an MPC stretch has its own length. Judging both by the ``DECEL`` rows would
give each a different question. The window therefore runs

* from ``k0``, the trial's first ``DECEL`` tick,
* to ``k1``, the first tick of the first stretch of ``rest_s`` seconds in which
  the measured catch frame rests in its TASK pose: its linear speed below
  ``rest_speed`` and the angular rate of its approach axis below
  ``rest_axis_rate``,

and never past the first ``RETREAT`` tick (the arm then moves on purpose): a
trial that does not rest before it has ``rest_reached = False`` and ends there.
The mode-based length is reported beside it as a reference.

Rest is judged on the hand's task pose, not on the joints. The catching
controller adds a null-space motion that ascends the manipulability measure,
so the joints keep moving while the task pose stands still; a joint-speed
threshold then reads the length of ``HOLD`` instead of the stop. The task is
the catch frame's position and its approach axis (the frame's +z): the roll
about that axis is not a task row, so it belongs to the null space too and
does not count. Beside the verdict the tool reports, as references,
``joint_rest_reached`` (every joint below ``joint_rest_speed`` for ``rest_s``)
and the full angular speed of the frame, roll included.

Per trial, inside the window:

* joint acceleration and jerk peaks (max over joints and ticks), for the
  command and for the measurement. Both are ``np.gradient`` derivatives on the
  tick grid, box-averaged over ``smooth_ticks`` — v1's reference acceleration
  steps at entry and at the target's stop, so an unsmoothed jerk is a function
  of the tick length; it is reported as ``jerk_cmd_raw_peak`` for reference
  only. The derivatives are taken over the window padded by ``pad_ticks`` on
  both sides, then cut, so the entry step is inside;
* stop distance of the catch frame — displacement ``‖p(k1) − p(k0)‖`` and path
  length, measured — next to the closed form ``‖ẋ_s‖² / (2 a_dec)`` of the
  entry reference speed;
* stop time ``t(k1) − t(k0)``;
* limit margins: the smallest distance to a position limit, the largest
  ``|q̇_meas| / q̇_max`` and the largest ``|effort| / τ_max``. A violation is a
  negative position margin or a ratio above 1.

Per unit and pooled: p50 / p95 / max of the per-trial values, all trials and
split by truth success (``catching_trials.csv`` of the unit).

``--a`` / ``--b`` name two sets of units that threw the SAME throws (one
throw list, or one seeded series): trials are paired by
:func:`rtc_tools.analysis.catching_throw_list.throw_key` — a throw-list trial
on (the list's sha256, ``throw_id``), a seeded one on ``(kind, seed,
sample_idx)`` — and the 2×2 table of
truth success is reported with the discordance rate ψ, its Wilson interval and
the exact McNemar p. Each set has its own pooled block; the two are pooled
TOGETHER only with ``--same-arm`` (two runs of one arm — two arms of a
comparison are not one population). ψ between two runs of the same arm is what the paired
non-inferiority test of G-1 needs for its sample size
(:func:`paired_noninferiority_n`).

Between two arms (MPC E1-F06, #632) the pairs also carry the test itself, per
``--margin``: Tango's score test of ``H0: p_b − p_a ≤ −margin``
(:func:`tango_noninferiority`), its score interval (:func:`tango_score_ci`),
the Wald interval (:func:`paired_difference`) and the exact power and size of
the test (:func:`paired_noninferiority_power`); both intervals are at the
level ``1 − 2·alpha`` the one-sided test matches. ``--success hold`` counts a
throw as caught only when it is a truth success AND reached the HOLD verdict
without an ``ABORT_SAFE`` — ``catching_trials``' ``hold_verdict`` and
``abort_in_window`` columns, read over the window its truth columns use.

Every limit comes from the run's profile, never from code (ARCH-1): joints from
the diag's columns, ratings from the device roster, ``a_dec`` from the
controller YAML with ``sim.yaml`` and the ``--overlay`` files laid over it.
"""

from __future__ import annotations

import argparse
import csv
import functools
import json
import math
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from statistics import NormalDist

import numpy as np

from rtc_tools.analysis import catching_arm_budget as ab, catching_trials as ct
from rtc_tools.analysis.catching_hand_near import _is_true as _truth_cell
from rtc_tools.analysis.catching_throw_list import throw_key
from rtc_tools.utils.catching_keys import reject_renamed_keys

TOOL = "catching_decel"
SMOOTH_TICKS = ab.ACCEL_SMOOTH_TICKS
REST_SPEED_M_S = 0.02
# 0.02 m/s at a 0.1 m lever: the two thresholds ask the same of a point on the hand.
REST_AXIS_RATE_RAD_S = 0.2
APPROACH_AXIS = 2  # the catch frame's +z (the column of its rotation)
JOINT_REST_SPEED_RAD_S = 0.01
REST_S = 0.05
PAD_TICKS = 25
SESSION_DIRS = ("session", "session_copy")
PEAK_KEYS = (
    "qdd_cmd_peak",
    "qdd_meas_peak",
    "jerk_cmd_peak",
    "jerk_meas_peak",
    "jerk_cmd_raw_peak",
)
SUMMARY_KEYS = (
    *PEAK_KEYS,
    "stop_time_s",
    "mode_decel_s",
    "stop_distance_mm",
    "stop_path_mm",
    "stop_distance_closed_form_mm",
    "entry_speed_m_s",
    "hand_speed_entry_m_s",
    "hand_speed_peak_m_s",
    "axis_rate_peak_rad_s",
    "angular_speed_peak_rad_s",
    "angular_speed_at_rest_rad_s",
    "position_margin_rad",
    "velocity_ratio",
    "torque_ratio",
)


# ── Pure numerics ────────────────────────────────────────────────────────────
box_smooth = ab.box_smooth  # one kernel for the budget's q̈ and the stop's q̈ and jerk


def derivatives(q: np.ndarray, dt: float, smooth_ticks: int = SMOOTH_TICKS) -> dict:
    """q̇, smoothed q̈, smoothed jerk and the unsmoothed jerk of a joint trajectory."""
    qd = np.gradient(q, dt, axis=0)
    qdd = np.gradient(qd, dt, axis=0)
    qdd_s = box_smooth(qdd, smooth_ticks)
    return {
        "qd": qd,
        "qdd": qdd_s,
        "jerk": box_smooth(np.gradient(qdd_s, dt, axis=0), smooth_ticks),
        "jerk_raw": np.gradient(qdd, dt, axis=0),
    }


def decel_entries(mode: np.ndarray) -> list[tuple[int, int, int]]:
    """``(k0, k_mode_end, k_limit)`` of every DECEL stretch.

    ``k_mode_end`` is the first tick after the stretch, ``k_limit`` the first
    ``RETREAT`` tick after it — or the next stretch's ``k0``, or the end of the
    log, whichever comes first (a stretch that aborts never retreats).
    """
    is_decel = (np.asarray(mode) == ct.MODE_DECEL).astype(int)
    edges = np.flatnonzero(np.diff(np.concatenate(([0], is_decel, [0]))))
    starts, ends = edges[::2], edges[1::2]
    retreat = np.flatnonzero(np.asarray(mode) == ct.MODE_RETREAT)
    out = []
    for i, (k0, ke) in enumerate(zip(starts, ends, strict=True)):
        limit = int(starts[i + 1]) if i + 1 < len(starts) else len(mode)
        after = retreat[retreat >= ke]
        if after.size and after[0] < limit:
            limit = int(after[0])
        out.append((int(k0), int(ke), limit))
    return out


def angular_speed(rotations: np.ndarray, dt: float) -> np.ndarray:
    """Angular speed [rad/s] of a sequence of rotation matrices (central differences)."""
    n = len(rotations)
    out = np.zeros(n)
    for k in range(n):
        a, b = max(0, k - 1), min(n - 1, k + 1)
        if a == b:
            continue
        rel = rotations[a].T @ rotations[b]
        sin = 0.5 * np.linalg.norm(
            [rel[2, 1] - rel[1, 2], rel[0, 2] - rel[2, 0], rel[1, 0] - rel[0, 1]]
        )
        out[k] = math.atan2(sin, 0.5 * (np.trace(rel) - 1.0)) / ((b - a) * dt)
    return out


def rest_tick(
    speed: np.ndarray, k0: int, k_limit: int, rest_speed: float, rest_ticks: int
) -> tuple[int, bool]:
    """``(k1, rest_reached)``: the first tick of the first ``rest_ticks`` run below ``rest_speed``.

    ``speed`` is one non-negative value per tick. Without such a run inside
    ``[k0, k_limit)`` the window ends at ``k_limit`` and the flag is False.
    """
    below = speed[k0:k_limit] < rest_speed
    run = 0
    for i, b in enumerate(below):
        run = run + 1 if b else 0
        if run >= rest_ticks:
            return k0 + i - rest_ticks + 1, True
    return int(k_limit), False


def window_metrics(
    q_cmd: np.ndarray,
    q_meas: np.ndarray,
    dt: float,
    k0: int,
    k_mode_end: int,
    k_limit: int,
    fk: Callable[[np.ndarray], tuple[np.ndarray, np.ndarray]],
    *,
    position_lower: Sequence[float],
    position_upper: Sequence[float],
    max_velocity: Sequence[float],
    entry_speed: float = math.nan,
    a_dec: float = math.nan,
    torque_ratio: np.ndarray | None = None,
    smooth_ticks: int = SMOOTH_TICKS,
    rest_speed: float = REST_SPEED_M_S,
    rest_axis_rate: float = REST_AXIS_RATE_RAD_S,
    joint_rest_speed: float = JOINT_REST_SPEED_RAD_S,
    rest_s: float = REST_S,
    pad_ticks: int = PAD_TICKS,
) -> dict:
    """The module docstring's per-trial numbers for one DECEL stretch.

    ``torque_ratio`` is ``|effort| / τ_max`` per diag tick and joint (NaN rows
    where the lane had no matching tick), or None without a lane. ``fk`` maps
    a posture to the catch frame's ``(position, rotation 3×3)``.
    """
    n = len(q_cmd)
    # A stretch the log ends in has no tick at its limit: its window ends at
    # the last one.
    k_limit = min(k_limit, n - 1)
    lo = max(0, k0 - pad_ticks)
    hi = min(n, k_limit + pad_ticks)
    cmd = derivatives(q_cmd[lo:hi], dt, smooth_ticks)
    meas = derivatives(q_meas[lo:hi], dt, smooth_ticks)
    poses = [fk(q) for q in q_meas[lo:hi]]
    p_all = np.array([p for p, _ in poses])
    r_all = np.array([r for _, r in poses])
    hand_speed = np.linalg.norm(np.gradient(p_all, dt, axis=0), axis=1)
    axis_rate = np.linalg.norm(np.gradient(r_all[:, :, APPROACH_AXIS], dt, axis=0), axis=1)
    turn = angular_speed(r_all, dt)
    rest_ticks = max(1, round(rest_s / dt))
    # One measure for both rows of the task: the larger of the two ratios to
    # its threshold, at rest below 1.
    task_motion = np.maximum(hand_speed / rest_speed, axis_rate / rest_axis_rate)
    k1_local, rested = rest_tick(task_motion, k0 - lo, k_limit - lo, 1.0, rest_ticks)
    _, joints_rested = rest_tick(
        np.abs(meas["qd"]).max(axis=1), k0 - lo, k_limit - lo, joint_rest_speed, rest_ticks
    )
    k1 = lo + k1_local
    # The window includes k1 itself: a trial that rests at once still has a row.
    sel = slice(k0 - lo, k1_local + 1)

    def peak(a: np.ndarray) -> float:
        a = np.abs(a[sel])
        return float(a.max()) if a.size else math.nan

    p = p_all[sel]
    steps = np.linalg.norm(np.diff(p, axis=0), axis=1)
    q_win = q_meas[k0 : k1 + 1]
    margin = np.minimum(
        q_win - np.asarray(position_lower, dtype=float),
        np.asarray(position_upper, dtype=float) - q_win,
    )
    out = {
        "k0": int(k0),
        "k1": int(k1),
        "rest_reached": bool(rested),
        "joint_rest_reached": bool(joints_rested),
        "hand_speed_entry_m_s": float(hand_speed[k0 - lo]),
        "hand_speed_peak_m_s": float(hand_speed[sel].max()),
        "axis_rate_peak_rad_s": float(axis_rate[sel].max()),
        "angular_speed_peak_rad_s": float(turn[sel].max()),
        "angular_speed_at_rest_rad_s": float(turn[k1_local]),
        "qdd_cmd_peak": peak(cmd["qdd"]),
        "qdd_meas_peak": peak(meas["qdd"]),
        "jerk_cmd_peak": peak(cmd["jerk"]),
        "jerk_meas_peak": peak(meas["jerk"]),
        "jerk_cmd_raw_peak": peak(cmd["jerk_raw"]),
        "stop_time_s": float((k1 - k0) * dt),
        "mode_decel_s": float((k_mode_end - k0) * dt),
        "stop_distance_mm": float(1e3 * np.linalg.norm(p[-1] - p[0])),
        "stop_path_mm": float(1e3 * steps.sum()),
        "entry_speed_m_s": float(entry_speed),
        "stop_distance_closed_form_mm": (
            float(1e3 * entry_speed**2 / (2.0 * a_dec))
            if math.isfinite(entry_speed) and math.isfinite(a_dec) and a_dec > 0.0
            else math.nan
        ),
        "position_margin_rad": float(margin.min()),
        "velocity_ratio": float(
            (np.abs(meas["qd"][sel]) / np.asarray(max_velocity, dtype=float)).max()
        ),
        "torque_ratio": math.nan,
    }
    if torque_ratio is not None:
        tr = torque_ratio[k0 : k1 + 1]
        tr = tr[np.isfinite(tr).all(axis=1)]
        if tr.size:
            out["torque_ratio"] = float(tr.max())
    out["violation_position"] = bool(out["position_margin_rad"] < 0.0)
    out["violation_velocity"] = bool(out["velocity_ratio"] > 1.0)
    out["violation_torque"] = bool(
        math.isfinite(out["torque_ratio"]) and out["torque_ratio"] > 1.0
    )
    return out


def paired_noninferiority_n(
    psi: float, margin: float, *, alpha: float = 0.025, power: float = 0.8, diff: float = 0.0
) -> int:
    """Pairs a one-sided paired non-inferiority test of two proportions needs.

    Normal approximation on the paired difference d = p_B − p_A, whose
    variance per pair is ``ψ − d²`` (ψ = discordance rate): H0 ``d ≤ −margin``
    against the truth ``d = diff``. ``n = (z_α + z_β)² (ψ − diff²) / (margin + diff)²``.
    """
    if not (0.0 < psi <= 1.0) or margin + diff <= 0.0 or psi <= diff * diff:
        raise ValueError(f"psi {psi}, margin {margin}, diff {diff}: no finite sample size")
    z = NormalDist().inv_cdf(1.0 - alpha) + NormalDist().inv_cdf(power)
    return math.ceil(z * z * (psi - diff * diff) / (margin + diff) ** 2)


def pair_table(a: Mapping[tuple, bool], b: Mapping[tuple, bool]) -> dict:
    """2×2 table of two outcome maps over their common keys, with ψ and McNemar."""
    keys = sorted(set(a) & set(b))
    both = sum(1 for k in keys if a[k] and b[k])
    only_a = sum(1 for k in keys if a[k] and not b[k])
    only_b = sum(1 for k in keys if b[k] and not a[k])
    n = len(keys)
    lo, hi = ct.wilson_interval(only_a + only_b, n) if n else (math.nan, math.nan)
    return {
        "n_pairs": n,
        "unpaired_a": len(set(a) - set(b)),
        "unpaired_b": len(set(b) - set(a)),
        "both": both,
        "only_a": only_a,
        "only_b": only_b,
        "neither": n - both - only_a - only_b,
        "success_a": both + only_a,
        "success_b": both + only_b,
        "discordance": (only_a + only_b) / n if n else math.nan,
        "discordance_ci95": [lo, hi],
        "mcnemar_p": ct.mcnemar_exact(only_a, only_b) if only_a + only_b else 1.0,
    }


def paired_difference(pairs: Mapping, z: float = 1.96) -> dict:
    """``p_b − p_a`` of a :func:`pair_table` with a Wald interval.

    Paired proportions: with b = only_a, c = only_b over n pairs the difference
    is ``(c − b) / n`` and its variance ``((b + c) − (c − b)² / n) / n²``.
    ``ci95`` is the interval at ``z`` (95 % only at the default).
    """
    n = pairs["n_pairs"]
    if not n:
        return {"diff": math.nan, "ci95": [math.nan, math.nan]}
    b, c = pairs["only_a"], pairs["only_b"]
    d = (c - b) / n
    se = math.sqrt(max((b + c) - (c - b) ** 2 / n, 0.0)) / n
    return {"diff": d, "ci95": [d - z * se, d + z * se]}


def tango_restricted_q(only_a, only_b, n, delta):
    """The MLE of the only-a cell probability under ``p_b − p_a = delta`` (Tango 1998).

    With ``q_b = q_a + delta`` the constrained likelihood peaks at the larger
    root of ``A q² + B q + C``: ``A = 2n``, ``B = −x_b − x_a + (2n − x_b + x_a) δ``,
    ``C = −x_a δ (1 − δ)`` (x_a = only_a, x_b = only_b). Takes arrays too.
    """
    xa, xb = np.asarray(only_a, dtype=float), np.asarray(only_b, dtype=float)
    a = 2.0 * n
    b = -xb - xa + (2.0 * n - xb + xa) * delta
    c = -xa * delta * (1.0 - delta)
    return (-b + np.sqrt(np.maximum(b * b - 4.0 * a * c, 0.0))) / (2.0 * a)


def tango_z(only_a, only_b, n, delta):
    """Tango's score statistic for ``H0: p_b − p_a = delta`` (large = b better than delta).

    ``Z = (x_b − x_a − n δ) / sqrt(n [2 q̃_a + δ (1 − δ)])``. Where the variance
    vanishes (``x_a = x_b = 0`` at δ = 0, or δ = ±1) a zero numerator is 0 and
    any other is ±inf. Takes arrays too.
    """
    xa, xb = np.asarray(only_a, dtype=float), np.asarray(only_b, dtype=float)
    num = xb - xa - n * delta
    var = n * (2.0 * tango_restricted_q(xa, xb, n, delta) + delta * (1.0 - delta))
    with np.errstate(divide="ignore", invalid="ignore"):
        z = np.where(var > 0.0, num / np.sqrt(np.maximum(var, 0.0)), np.sign(num) * np.inf)
    z = np.where((var <= 0.0) & (num == 0.0), 0.0, z)
    return float(z) if z.ndim == 0 else z


def tango_noninferiority(only_a: int, only_b: int, n: int, margin: float, *, alpha=0.025) -> dict:
    """One-sided paired non-inferiority of b to a: ``H0: p_b − p_a ≤ −margin``.

    Tango's score test (:func:`tango_z` at ``δ0 = −margin``); b is
    non-inferior when ``p = 1 − Φ(Z) < alpha``.
    """
    if n <= 0 or only_a < 0 or only_b < 0 or only_a + only_b > n:
        raise ValueError(f"only_a {only_a}, only_b {only_b}, n {n}: not a paired table")
    delta = -float(margin)
    z = tango_z(only_a, only_b, n, delta)
    p = 1.0 - NormalDist().cdf(z) if math.isfinite(z) else (0.0 if z > 0 else 1.0)
    return {
        "margin": float(margin),
        "alpha": alpha,
        "z": z,
        "p": p,
        "reject": bool(p < alpha),
        "q_a_restricted": float(tango_restricted_q(only_a, only_b, n, delta)),
    }


def tango_score_ci(only_a: int, only_b: int, n: int, level: float = 0.95) -> list[float]:
    """Tango's score interval of ``p_b − p_a``: ``{δ : |Z(δ)| < z}``, by bisection on [−1, 1].

    Z falls as δ rises, so each end is the root on its side of the point
    estimate; an estimate of ±1 is its own end on that side.
    """
    if n <= 0 or only_a < 0 or only_b < 0 or only_a + only_b > n:
        raise ValueError(f"only_a {only_a}, only_b {only_b}, n {n}: not a paired table")
    zc = NormalDist().inv_cdf(0.5 + level / 2.0)
    d = (only_b - only_a) / n

    def root(lo: float, hi: float, target: float) -> float:
        # Z(lo) ≥ target ≥ Z(hi) on a falling Z.
        for _ in range(200):
            mid = 0.5 * (lo + hi)
            if tango_z(only_a, only_b, n, mid) >= target:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    lower = -1.0 if d <= -1.0 or tango_z(only_a, only_b, n, -1.0) < zc else root(-1.0, d, zc)
    upper = 1.0 if d >= 1.0 or tango_z(only_a, only_b, n, 1.0) > -zc else root(d, 1.0, -zc)
    return [lower, upper]


def paired_noninferiority_power(
    n: int, psi: float, diff: float, margin: float, *, alpha: float = 0.025
) -> float:
    """Exact rejection probability of :func:`tango_noninferiority` over ``n`` pairs.

    Sums the trinomial over every table (only_a, only_b, the rest) with cell
    probabilities ``q_a = (ψ − d)/2``, ``q_b = (ψ + d)/2`` and ``1 − ψ``. At
    ``diff = −margin`` it is the test's size.
    """
    q_a, q_b = (psi - diff) / 2.0, (psi + diff) / 2.0
    if n <= 0 or not (0.0 <= psi <= 1.0) or q_a < 0.0 or q_b < 0.0:
        raise ValueError(f"n {n}, psi {psi}, diff {diff}: no such paired table")
    xa, xb, rest, log_coef = _rejection_region(int(n), float(margin), float(alpha))

    def xlogq(x: np.ndarray, q: float) -> np.ndarray:
        # 0 · log 0 = 0: a cell of probability 0 holds 0 pairs with certainty.
        return np.where(x > 0, x * math.log(q), 0.0) if q > 0.0 else np.where(x > 0, -np.inf, 0.0)

    logp = log_coef + xlogq(xa, q_a) + xlogq(xb, q_b) + xlogq(rest, 1.0 - psi)
    return float(np.exp(logp).sum())


@functools.lru_cache(maxsize=16)
def _rejection_region(n: int, margin: float, alpha: float) -> tuple[np.ndarray, ...]:
    """The tables (only_a, only_b, the rest) the test rejects at, with log n!/(a! b! r!).

    Depends on ``(n, margin, alpha)`` only, so every power row of one test
    shares it. The arrays are read-only (they are cached).
    """
    xa, xb = (g.ravel() for g in np.mgrid[0 : n + 1, 0 : n + 1])
    keep = xa + xb <= n
    xa, xb = xa[keep], xb[keep]
    reject = tango_z(xa, xb, n, -margin) > NormalDist().inv_cdf(1.0 - alpha)
    xa, xb = xa[reject], xb[reject]
    rest = n - xa - xb
    log_fact = np.concatenate(([0.0], np.cumsum(np.log(np.arange(1, n + 1)))))
    out = (xa, xb, rest, log_fact[n] - log_fact[xa] - log_fact[xb] - log_fact[rest])
    for a in out:
        a.flags.writeable = False
    return out


# ── One unit ─────────────────────────────────────────────────────────────────
def parse_unit_arg(value: str) -> tuple[Path, Path]:
    """``<unit>[:<session>]`` — the session defaults to ``<unit>/session`` (or ``session_copy``)."""
    unit, _, session = value.partition(":")
    u = Path(unit)
    if session:
        return u, Path(session)
    for name in SESSION_DIRS:
        if (u / name).is_dir():
            return u, u / name
    return u, u / SESSION_DIRS[0]


def composed_a_dec(
    node: Mapping, config_dir: Path, controller: str, overlays: Sequence[Path]
) -> tuple[float, str]:
    """``supervisor.decel.a_dec`` as the launch composes it, and which layer had the last word."""
    catching = dict(node.get("catching") or {})
    layers = [("profile", None)]
    sim_yaml = Path(config_dir) / "sim.yaml"
    if sim_yaml.is_file():
        layers.append(("sim.yaml", sim_yaml))
    layers += [(p.name, p) for p in overlays]
    source = "profile"
    for name, path in layers:
        if path is None:
            continue
        tree = ab._overlay_catching(path, controller)
        reject_renamed_keys(tree, source=str(path))
        if ((tree.get("supervisor") or {}).get("decel") or {}).get("a_dec") is not None:
            source = name
        catching = ab._deep_merge(catching, tree)
    value = ((catching.get("supervisor") or {}).get("decel") or {}).get("a_dec")
    return (math.nan if value is None else float(value)), source


def _torque_ratio(
    session: Path,
    profile: ct.CatchingProfile,
    joints: Sequence[str],
    tau_max: Sequence[float],
    t: np.ndarray,
    dt: float,
) -> tuple[np.ndarray | None, str | None]:
    """``|effort| / τ_max`` on the diag's ticks (rows matched by time, NaN when unmatched)."""
    log = profile.device_logs.get(profile.arm_device)
    if not log:
        return None, None
    path = session / "controllers" / profile.controller / f"{log}.csv"
    found = ct._exists(path)
    if found is None:
        return None, None
    cols = [f"effort_{j}" for j in joints]
    header = ct._csv_header(found)
    if any(c not in header for c in cols) or "t_relative_s" not in header:
        return None, f"{path.name}: no effort_* / t_relative_s columns"
    lane = ct._read_csv(path, usecols=["t_relative_s", *cols])
    t_lane = lane["t_relative_s"].to_numpy(dtype=float)
    if len(t_lane) == 0:
        return None, f"{path.name}: no lane row (header only)"
    idx = np.clip(np.searchsorted(t_lane, t), 0, len(t_lane) - 1)
    left = np.clip(idx - 1, 0, len(t_lane) - 1)
    take = np.where(np.abs(t_lane[left] - t) < np.abs(t_lane[idx] - t), left, idx)
    ok = np.abs(t_lane[take] - t) <= 0.5 * dt
    ratio = np.abs(lane[cols].to_numpy(dtype=float)[take]) / np.asarray(tau_max, dtype=float)
    ratio[~ok] = np.nan
    return ratio, path.name


def _trial_table(unit: Path, ct_dir: Path, extra: Sequence[str] = ()) -> list[dict]:
    """The unit's ``catching_trials.csv`` rows joined with the runner's throw identity.

    ``extra`` names further columns to carry over as they are in the CSV
    (strings, ``""`` when absent) — the caller converts what it needs.
    """
    path = ct_dir / "catching_trials.csv"
    if not path.is_file():
        raise SystemExit(f"{path}: run catching_trials on the unit first")
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    _, info = ct.load_trials(unit / "trials")
    by_idx = {int(r["idx"]): r for r in info["records"]}
    # The list the unit threw, if it threw one (run_meta.json's throws_file).
    list_sha = (info["meta"].get("throws_file") or {}).get("sha256")
    out = []
    for r in rows:
        rec = by_idx.get(int(r["idx"]), {})

        def num(key: str, row=r) -> float:
            try:
                return float(row.get(key, ""))
            except ValueError:
                return math.nan

        hold, abort = (_bool_cell(r.get(k)) for k in ("hold_verdict", "abort_in_window"))
        out.append(
            {
                "idx": int(r["idx"]),
                "kind": r.get("kind", ""),
                "seed": rec.get("seed"),
                "sample_idx": rec.get("sample_idx"),
                "throw_id": rec.get("throw_id"),
                "throws_file_sha256": list_sha,
                "t_launch": num("t_launch"),
                "t_end": num("t_end"),
                "invalid_reason": r.get("invalid_reason", ""),
                "truth_success": _truth_cell(r.get("truth_success")),
                # None: catching_trials judged no window (or predates the columns).
                "hold_no_abort": None if hold is None or abort is None else hold and not abort,
                "supervisor": r.get("supervisor", ""),
                **{k: r.get(k, "") for k in extra},
            }
        )
    return out


def analyse_unit(
    unit: Path,
    session: Path,
    config_dir: Path,
    *,
    controller: str | None = None,
    overlays: Sequence[Path] = (),
    urdf: Path | None = None,
    ct_dir: Path | None = None,
    smooth_ticks: int = SMOOTH_TICKS,
    rest_speed: float = REST_SPEED_M_S,
    rest_axis_rate: float = REST_AXIS_RATE_RAD_S,
    joint_rest_speed: float = JOINT_REST_SPEED_RAD_S,
    rest_s: float = REST_S,
) -> dict:
    """Per-trial DECEL rows and the unit summary (module docstring)."""
    from rtc_tools.analysis.derive_accel_limits import resolve_urdf_text  # noqa: PLC0415

    unit, session, config_dir = Path(unit), Path(session), Path(config_dir)
    run_meta = unit / "trials" / "run_meta.json"
    meta = ct.load_run_meta(run_meta) if run_meta.is_file() else {}
    profile = ct.load_profile(config_dir, controller, session)
    node = ct._catching_controllers(config_dir)[profile.controller]
    diag_path = (
        session / "controllers" / profile.controller / f"{profile.diag_log or 'catching_diag'}.csv"
    )
    header = ct._csv_header(ct._exists(diag_path) or diag_path)
    joints = ct.arm_joints_from_diag(header)
    ref_cols = [f"ref_xd_{ax}" for ax in "xyz"]
    missing = [c for c in ("t_relative_s", "mode") if c not in header]
    if missing:
        raise SystemExit(f"catching diag lacks column(s) {missing}")
    # The reference speed only feeds the closed-form stop distance (a reference
    # value): a diag without it still has every measured number.
    has_ref = all(c in header for c in ref_cols)
    cols = ["t_relative_s", "mode", *(ref_cols if has_ref else [])]
    cols += [f"q_cmd_{j}" for j in joints] + [f"q_meas_{j}" for j in joints]
    diag = ct._read_csv(diag_path, usecols=cols)
    t = diag["t_relative_s"].to_numpy(dtype=float)
    mode = diag["mode"].to_numpy()
    dt = float(
        (meta.get("controller_mirror") or {}).get("control.dt")
        or np.median(np.diff(t[: min(len(t), 2000)]))
    )
    q_cmd = diag[[f"q_cmd_{j}" for j in joints]].to_numpy(dtype=float)
    q_meas = diag[[f"q_meas_{j}" for j in joints]].to_numpy(dtype=float)
    xd_ref = (
        np.linalg.norm(diag[ref_cols].to_numpy(dtype=float), axis=1)
        if has_ref
        else np.full(len(t), math.nan)
    )

    limits, lim_src = ab._device_limits(config_dir, profile.arm_device)
    n = len(joints)
    need = ("position_lower", "position_upper", "max_velocity", "max_torque")
    for key in need:
        if len(limits.get(key) or []) != n:
            raise SystemExit(
                f"devices.{profile.arm_device}.joint_limits.{key} must have {n} entries "
                f"(the diag's arm joints), got {limits.get(key)}"
            )
    a_dec, a_dec_src = composed_a_dec(node, config_dir, profile.controller, overlays)
    ratio, torque_src = _torque_ratio(session, profile, joints, limits["max_torque"], t, dt)
    urdf_text, _ = resolve_urdf_text(profile.robot_params, urdf)
    fk = ct.CatchFrameFk(urdf_text, joints, profile).pose_world

    trials = _trial_table(unit, ct_dir or unit / "ct")
    launches = np.array([r["t_launch"] for r in trials], dtype=float)
    entries = decel_entries(mode)
    rows = []
    claimed: set[int] = set()
    for k0, k_mode_end, k_limit in entries:
        # The stretch belongs to the latest launch at or before its entry.
        before = np.flatnonzero(np.isfinite(launches) & (launches <= t[k0]))
        if not before.size:
            continue
        i = int(before[np.argmax(launches[before])])
        if i in claimed:  # a trial enters DECEL once; a second stretch is not its stop
            continue
        claimed.add(i)
        m = window_metrics(
            q_cmd,
            q_meas,
            dt,
            k0,
            k_mode_end,
            k_limit,
            fk,
            position_lower=limits["position_lower"],
            position_upper=limits["position_upper"],
            max_velocity=limits["max_velocity"],
            entry_speed=float(xd_ref[k0]),
            a_dec=a_dec,
            torque_ratio=ratio,
            smooth_ticks=smooth_ticks,
            rest_speed=rest_speed,
            rest_axis_rate=rest_axis_rate,
            joint_rest_speed=joint_rest_speed,
            rest_s=rest_s,
        )
        rows.append({**trials[i], "t_decel": float(t[k0]), **m})
    rows.sort(key=lambda r: r["idx"])
    valid = [r for r in trials if not r["invalid_reason"]]
    summary = {
        "unit": str(unit),
        "arm": meta.get("arm"),
        "controller": profile.controller,
        "joints": list(joints),
        "dt": dt,
        "a_dec": a_dec,
        "sources": {
            "a_dec": a_dec_src,
            "limits": {k: lim_src.get(k) for k in need},
            "torque_lane": torque_src,
            "entry_speed": "diag ref_xd_*" if has_ref else None,
        },
        "settings": {
            "smooth_ticks": smooth_ticks,
            "rest_speed_m_s": rest_speed,
            "rest_axis_rate_rad_s": rest_axis_rate,
            "joint_rest_speed_rad_s": joint_rest_speed,
            "rest_s": rest_s,
            "pad_ticks": PAD_TICKS,
        },
        "limits": {k: [float(v) for v in limits[k]] for k in need},
        "host_watch": meta.get("host_watch"),
        "n_trials": len(trials),
        "n_valid": len(valid),
        "truth_success": sum(1 for r in valid if r["truth_success"]),
        "hold_success": _hold_success(valid),
        "unclaimed_decel_stretches": len(entries) - len(rows),
        **summarise(rows),
    }
    return {"summary": summary, "trials": rows, "all_trials": trials}


def _stats(values: Sequence[float], qs: Sequence[float] = (50, 95)) -> dict:
    """``n``, the percentiles ``qs`` (as ``p50`` …), ``max`` and ``min`` of the finite values."""
    v = np.asarray([x for x in values if x is not None and math.isfinite(x)], dtype=float)
    out = {"n": int(v.size)}
    for q in qs:
        out[f"p{q:g}"] = float(np.percentile(v, q)) if v.size else math.nan
    out["max"] = float(v.max()) if v.size else math.nan
    out["min"] = float(v.min()) if v.size else math.nan
    return out


def _bool_cell(value) -> bool | None:
    """A CSV bool cell; ``None`` when the column is absent or the cell is empty."""
    if value is None or str(value).strip() == "":
        return None
    return _truth_cell(value)


def _hold_success(valid: Sequence[Mapping]) -> int | None:
    """Valid trials that count under ``--success hold``; ``None`` when one has no verdict."""
    if any(r["hold_no_abort"] is None for r in valid):
        return None
    return sum(1 for r in valid if r["truth_success"] and r["hold_no_abort"])


def summarise(rows: Sequence[Mapping]) -> dict:
    """p50 / p95 / max of the per-trial values: all DECEL trials, and by truth success."""

    def block(sel: Sequence[Mapping]) -> dict:
        return {
            "n": len(sel),
            "rest_reached": sum(1 for r in sel if r["rest_reached"]),
            "joint_rest_reached": sum(1 for r in sel if r["joint_rest_reached"]),
            "violations": {
                k: sum(1 for r in sel if r[f"violation_{k}"])
                for k in ("position", "velocity", "torque")
            },
            **{k: _stats([r[k] for r in sel]) for k in SUMMARY_KEYS},
        }

    return {
        "decel": block(rows),
        "decel_success": block([r for r in rows if r["truth_success"]]),
        "decel_failure": block([r for r in rows if not r["truth_success"]]),
    }


def pool(units: Sequence[dict]) -> dict:
    """The pooled block over several units of one robot and ONE arm of a comparison."""
    rows = [r for u in units for r in u["trials"]]
    valid = [r for u in units for r in u["all_trials"] if not r["invalid_reason"]]
    total = sum(len(u["all_trials"]) for u in units)
    k = sum(1 for r in valid if r["truth_success"])
    itt = ct.wilson_interval(k, total) if total else (math.nan, math.nan)
    return {
        "units": [Path(u["summary"]["unit"]).name for u in units],
        "n_trials": total,
        "n_valid": len(valid),
        "truth_success": k,
        "hold_success": _hold_success(valid),
        "truth_ci95": list(ct.wilson_interval(k, len(valid))) if valid else [math.nan] * 2,
        "itt_ci95": list(itt),
        "invalid_reasons": _count(
            r["invalid_reason"] for u in units for r in u["all_trials"] if r["invalid_reason"]
        ),
        **summarise(rows),
    }


def _count(values) -> dict:
    out: dict = {}
    for v in values:
        out[v] = out.get(v, 0) + 1
    return out


SUCCESS_DEFS = ("truth", "hold")


def outcome_map(units: Sequence[dict], success: str = "truth") -> dict[tuple, bool]:
    """``throw_key → success`` over the valid trials of ``units``.

    The key is :func:`rtc_tools.analysis.catching_throw_list.throw_key`: the
    list's sha256 and ``throw_id`` of a throw-list trial, ``(kind, seed,
    sample_idx)`` of a seeded one. A trial with neither pairs with nothing.

    ``success``: ``truth`` is ``truth_success``; ``hold`` also asks the trial
    to reach the HOLD verdict without an ``ABORT_SAFE`` (:func:`hold_no_abort`).
    """
    if success not in SUCCESS_DEFS:
        raise ValueError(f"success {success!r}: one of {SUCCESS_DEFS}")
    out: dict[tuple, bool] = {}
    for u in units:
        for r in u["all_trials"]:
            key = throw_key(r)
            if r["invalid_reason"] or key is None:
                continue
            if key in out:
                raise SystemExit(
                    f"{u['summary']['unit']}: throw {key} appears twice in one set — a set is "
                    "one run of each throw"
                )
            ok = bool(r["truth_success"])
            if success == "hold":
                if r["hold_no_abort"] is None:
                    raise SystemExit(
                        f"{u['summary']['unit']}: trial {r['idx']} has no mode-path verdict — "
                        "--success hold needs catching_trials.csv with hold_verdict and "
                        "abort_in_window (re-run catching_trials)"
                    )
                ok = ok and r["hold_no_abort"]
            out[key] = ok
    return out


# ── CLI ──────────────────────────────────────────────────────────────────────
def _fmt_stats(s: Mapping, scale: float = 1.0, nd: int = 1) -> str:
    if not s["n"]:
        return "—"
    return "/".join(f"{scale * s[k]:.{nd}f}" for k in ("p50", "p95", "max"))


def report_block(name: str, b: Mapping) -> list[str]:
    d = b["decel"]
    lines = [
        f"[{name}] DECEL trials {d['n']} · task pose rested before RETREAT {d['rest_reached']} "
        f"(joints {d['joint_rest_reached']})"
    ]
    if "n_valid" in b:
        ci = b.get("truth_ci95")
        tail = f" (Wilson 95 % [{ci[0]:.3f}, {ci[1]:.3f}])" if ci else ""
        lines[0] += f" · truth success {b['truth_success']}/{b['n_valid']} valid{tail}"
        if b.get("hold_success") is not None:
            lines[0] += f" · hold success {b['hold_success']}/{b['n_valid']}"
    lines.append(
        "  q̈ peak rad/s² (p50/p95/max): "
        f"cmd {_fmt_stats(d['qdd_cmd_peak'])} · meas {_fmt_stats(d['qdd_meas_peak'])}"
    )
    lines.append(
        "  jerk peak rad/s³: "
        f"cmd {_fmt_stats(d['jerk_cmd_peak'], nd=0)} · meas {_fmt_stats(d['jerk_meas_peak'], nd=0)}"
        f" · cmd unsmoothed {_fmt_stats(d['jerk_cmd_raw_peak'], nd=0)} (reference)"
    )
    lines.append(
        f"  stop distance mm: {_fmt_stats(d['stop_distance_mm'])} · path "
        f"{_fmt_stats(d['stop_path_mm'])} · closed form {_fmt_stats(d['stop_distance_closed_form_mm'])}"
        f" · entry speed m/s ref {_fmt_stats(d['entry_speed_m_s'], nd=2)} hand "
        f"{_fmt_stats(d['hand_speed_entry_m_s'], nd=2)} (peak {_fmt_stats(d['hand_speed_peak_m_s'], nd=2)})"
    )
    lines.append(
        f"  stop time s: {_fmt_stats(d['stop_time_s'], nd=3)} · DECEL mode "
        f"{_fmt_stats(d['mode_decel_s'], nd=3)} · approach axis rate peak rad/s "
        f"{_fmt_stats(d['axis_rate_peak_rad_s'], nd=2)} · frame angular speed at rest "
        f"{_fmt_stats(d['angular_speed_at_rest_rad_s'], nd=3)} (roll included, reference)"
    )
    pm = d["position_margin_rad"]
    lines.append(
        f"  margins: position min {pm['min']:.3f} rad · velocity ratio "
        f"{_fmt_stats(d['velocity_ratio'], nd=3)} · torque ratio {_fmt_stats(d['torque_ratio'], nd=3)}"
        f" · violations {d['violations']}"
    )
    for key, label in (("decel_success", "success"), ("decel_failure", "failure")):
        s = b[key]
        lines.append(
            f"  {label} n {s['n']}: q̈ meas {_fmt_stats(s['qdd_meas_peak'])} · jerk meas "
            f"{_fmt_stats(s['jerk_meas_peak'], nd=0)} · stop {_fmt_stats(s['stop_distance_mm'])} mm"
        )
    return lines


def report(units: Sequence[dict], pooled: Mapping | None, pairs: Mapping | None) -> str:
    lines = [f"{TOOL}: {len(units)} unit(s)"]
    for u in units:
        s = u["summary"]
        lines += report_block(f"{s['arm']} {Path(s['unit']).name}", s)
    if pooled:
        lines += report_block("pooled", pooled)
        lines.append(
            f"  ITT {pooled['truth_success']}/{pooled['n_trials']} Wilson 95 % "
            f"[{pooled['itt_ci95'][0]:.3f}, {pooled['itt_ci95'][1]:.3f}] · invalid "
            f"{pooled['invalid_reasons'] or 0}"
        )
    if pairs:
        ci = pairs["discordance_ci95"]
        lines.append(
            f"[a × b] pairs {pairs['n_pairs']} (unpaired a {pairs['unpaired_a']} b "
            f"{pairs['unpaired_b']}) · {pairs.get('success', 'truth')} success a "
            f"{pairs['success_a']} b {pairs['success_b']} · both "
            f"{pairs['both']} only a {pairs['only_a']} only b {pairs['only_b']} neither "
            f"{pairs['neither']} · ψ {pairs['discordance']:.3f} [{ci[0]:.3f}, {ci[1]:.3f}] · "
            f"McNemar p {pairs['mcnemar_p']:.3g}"
        )
        if not pairs["n_pairs"]:
            lines.append(
                "  no throw is in both sets (list · throw_id, or seed · sample_idx): nothing to pair"
            )
        w = pairs.get("wald")
        if w and pairs["n_pairs"]:
            lines.append(
                f"  d̂ {w['diff']:+.4f} · Wald {100 * w['level']:g} % "
                f"[{w['ci'][0]:+.5f}, {w['ci'][1]:+.5f}]"
            )
        for b in pairs.get("noninferiority") or []:
            if b is None:
                continue
            s = b["score_ci"]
            verdict = "non-inferior" if b["reject"] else "non-inferiority not shown"
            lines.append(
                f"  Tango non-inferiority margin {b['margin']:.2f} (one-sided α {b['alpha']}): "
                f"Z {b['z']:.4f} · p {b['p']:.4g} → {verdict} · score {100 * b['ci_level']:g} % "
                f"[{s[0]:+.5f}, {s[1]:+.5f}]"
            )
            for key, label in (("power_design", "design"), ("power_observed", "observed")):
                rows = b[key] or []
                if rows:
                    cells = " · ".join(
                        f"d {r['diff']:+.3f} → "
                        + ("N/A" if r["power"] is None else f"{r['power']:.3f}")
                        for r in rows
                    )
                    lines.append(
                        f"    power at ψ {rows[0]['psi']:.3f} ({label}, n {rows[0]['n']}): {cells}"
                    )
            size = "N/A" if b["size"] is None else f"{b['size']:.4f}"
            lines.append(f"    exact size at the H0 boundary (ψ̂): {size}")
        for row in pairs.get("sample_size", []):
            at, upper = (
                "— (no discordant pair seen)" if row[k] is None else row[k]
                for k in ("n_at_psi", "n_at_psi_upper")
            )
            lines.append(
                f"  paired non-inferiority n (α {row['alpha']}, power {row['power']}, true "
                f"difference 0): margin {row['margin']:.2f} → ψ̂ {at} · ψ upper {upper}"
            )
    return "\n".join(lines)


def noninferiority_block(
    pairs: Mapping,
    margin: float,
    alpha: float,
    design_psi: float | None,
    power_diffs: Sequence[float],
    design_n: int | None = None,
) -> dict | None:
    """Tango's test of b against a at ``margin``, its score interval and power (MPC E1-F06).

    The score interval is at ``1 − 2·alpha``, the level whose lower end
    clears ``−margin`` exactly when the one-sided test rejects. Power rows are
    exact (:func:`paired_noninferiority_power`): at the design ψ over
    ``design_n`` pairs (the planned count; the valid pairs when not given) and
    at the observed ψ̂ over the valid pairs, each at ``power_diffs``; the
    observed ones add the estimate d̂ (a function of the same data as p, not
    new evidence). A difference larger than ψ in size has no table and is
    ``None``. ``size`` is the test's exact rejection rate on the H0 boundary
    at ψ̂. The point estimate and the Wald interval are the pair table's
    (``pairs["wald"]``), not repeated per margin. ``None`` without a pair.
    """
    n = pairs["n_pairs"]
    if not n:
        return None
    only_a, only_b = pairs["only_a"], pairs["only_b"]
    psi, d_hat = pairs["discordance"], paired_difference(pairs)["diff"]
    level = 1.0 - 2.0 * alpha

    def power(at_n: int, at_psi: float, diff: float) -> float | None:
        if abs(diff) > at_psi:
            return None
        return paired_noninferiority_power(at_n, at_psi, diff, margin, alpha=alpha)

    n_design = design_n or n
    return {
        **tango_noninferiority(only_a, only_b, n, margin, alpha=alpha),
        "n": n,
        "ci_level": level,
        "score_ci": tango_score_ci(only_a, only_b, n, level),
        "power_design": (
            None
            if design_psi is None
            else [
                {
                    "n": n_design,
                    "psi": design_psi,
                    "diff": d,
                    "power": power(n_design, design_psi, d),
                }
                for d in power_diffs
            ]
        ),
        "power_observed": [
            {"n": n, "psi": psi, "diff": d, "power": power(n, psi, d)}
            for d in [*power_diffs, d_hat]
        ],
        "size": power(n, psi, -margin),
    }


def write_outputs(
    units: Sequence[dict], pooled: Mapping | None, pairs: Mapping | None, out_dir: Path
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    doc = {"units": [u["summary"] for u in units], "pooled": pooled, "pairs": pairs}
    (out_dir / "decel_summary.json").write_text(
        json.dumps(doc, indent=1, default=ab._json_default)
    )
    rows = [{"unit": Path(u["summary"]["unit"]).name, **r} for u in units for r in u["trials"]]
    fields = list(dict.fromkeys(k for r in rows for k in r))
    with (out_dir / "decel_trials.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, restval="")
        w.writeheader()
        w.writerows(rows)


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog=TOOL, description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "units",
        nargs="*",
        default=[],
        help="<unit>[:<session>] — session defaults to <unit>/session (or session_copy)",
    )
    ap.add_argument("--a", nargs="+", default=[], help="units of set a (paired with --b)")
    ap.add_argument("--b", nargs="+", default=[], help="units of set b: the same throws as --a")
    ap.add_argument(
        "--same-arm",
        action="store_true",
        help="sets a and b are two runs of ONE arm (replicates): pool them together too. "
        "Without it the pooled block takes the plain units only — two arms of a "
        "comparison are not one population",
    )
    ap.add_argument(
        "--config-dir",
        type=Path,
        required=True,
        help="robot profile directory (integrated_bringup config/<robot>)",
    )
    ap.add_argument("--controller", help="catching controller name when the profile has several")
    ap.add_argument(
        "--overlay",
        type=Path,
        action="append",
        default=[],
        help="sim overlay YAML the units ran with (read for supervisor.decel.a_dec)",
    )
    ap.add_argument("--urdf", type=Path, help="expanded URDF (default: urdf.package/path)")
    ap.add_argument("--smooth-ticks", type=int, default=SMOOTH_TICKS)
    ap.add_argument("--rest-speed", type=float, default=REST_SPEED_M_S, help="catch frame, m/s")
    ap.add_argument(
        "--rest-axis-rate",
        type=float,
        default=REST_AXIS_RATE_RAD_S,
        help="approach axis of the catch frame, rad/s",
    )
    ap.add_argument(
        "--joint-rest-speed", type=float, default=JOINT_REST_SPEED_RAD_S, help="rad/s (reference)"
    )
    ap.add_argument("--rest-s", type=float, default=REST_S)
    ap.add_argument(
        "--margin",
        type=float,
        nargs="*",
        default=[0.05, 0.10],
        help="non-inferiority margins (absolute) for the sample-size rows of --a/--b",
    )
    ap.add_argument("--alpha", type=float, default=0.025, help="one-sided")
    ap.add_argument("--power", type=float, default=0.8)
    ap.add_argument(
        "--success",
        choices=SUCCESS_DEFS,
        default="truth",
        help="what a paired trial's success is: truth_success, or (hold) truth_success "
        "AND the HOLD verdict reached AND no ABORT_SAFE in [t_launch, t_end]",
    )
    ap.add_argument(
        "--design-psi",
        type=float,
        help="discordance the comparison was planned at: adds the design power rows",
    )
    ap.add_argument(
        "--design-n",
        type=int,
        help="pairs the comparison was planned at, for the design power rows "
        "(default: the valid pairs)",
    )
    ap.add_argument(
        "--power-diff",
        type=float,
        nargs="*",
        default=[0.0, -0.05],
        help="true differences p_b − p_a the power rows are computed at (the "
        "estimate is added to the observed rows)",
    )
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)
    if bool(args.a) != bool(args.b):
        ap.error("--a and --b go together")
    if not (args.units or args.a):
        ap.error("no units")
    if not 0.0 < args.alpha < 0.5:
        ap.error(f"--alpha {args.alpha}: one-sided, in (0, 0.5)")
    if args.design_psi is not None and not 0.0 <= args.design_psi <= 1.0:
        ap.error(f"--design-psi {args.design_psi}: a discordance rate, in [0, 1]")
    if args.design_n is not None and args.design_n < 1:
        ap.error(f"--design-n {args.design_n}: at least one pair")

    def load(values: Sequence[str]) -> list[dict]:
        return [
            analyse_unit(
                *parse_unit_arg(v),
                args.config_dir,
                controller=args.controller,
                overlays=args.overlay,
                urdf=args.urdf,
                smooth_ticks=args.smooth_ticks,
                rest_speed=args.rest_speed,
                rest_axis_rate=args.rest_axis_rate,
                joint_rest_speed=args.joint_rest_speed,
                rest_s=args.rest_s,
            )
            for v in values
        ]

    plain, set_a, set_b = load(args.units), load(args.a), load(args.b)
    units = plain + set_a + set_b
    pooled_over = units if args.same_arm else plain
    pooled = pool(pooled_over) if len(pooled_over) > 1 else None
    pairs = None
    if set_a:
        pairs = pair_table(outcome_map(set_a, args.success), outcome_map(set_b, args.success))
        pairs["success"] = args.success
        pairs["pooled_a"], pairs["pooled_b"] = pool(set_a), pool(set_b)
        level = 1.0 - 2.0 * args.alpha
        wald = paired_difference(pairs, z=NormalDist().inv_cdf(1.0 - args.alpha))
        pairs["wald"] = {"diff": wald["diff"], "level": level, "ci": wald["ci95"]}
        pairs["noninferiority"] = [
            noninferiority_block(
                pairs, m, args.alpha, args.design_psi, args.power_diff, args.design_n
            )
            for m in args.margin
        ]

        def pairs_needed(psi: float, margin: float) -> int | None:
            # No pair, or no discordant one: ψ says nothing about a sample size,
            # and 0 would read as "no pair needed".
            if not (math.isfinite(psi) and psi > 0.0):
                return None
            return paired_noninferiority_n(psi, margin, alpha=args.alpha, power=args.power)

        pairs["sample_size"] = [
            {
                "margin": m,
                "alpha": args.alpha,
                "power": args.power,
                "n_at_psi": pairs_needed(pairs["discordance"], m),
                "n_at_psi_upper": pairs_needed(pairs["discordance_ci95"][1], m),
            }
            for m in args.margin
        ]
    write_outputs(units, pooled, pairs, args.out)
    print(report(units, pooled, pairs))
    if pairs:
        for name in ("pooled_a", "pooled_b"):
            print("\n".join(report_block(name, pairs[name])))
    print(f"\n-> {args.out / 'decel_summary.json'}\n-> {args.out / 'decel_trials.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
