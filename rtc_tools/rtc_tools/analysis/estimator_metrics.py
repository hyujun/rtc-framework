"""Open-loop metrics of the contact-aided InEKF, from an ``estimator_replay`` run.

``ros2 run integrated_bringup estimator_replay`` replays a run's logs through the
inekf state source and writes ``<replay>.csv`` (one row per tick, 17 digits)
and ``<replay>.csv.json`` (counts, the IMU extrinsic ``R_BI`` / ``p_BI``, the
contact names, gravity). This module reads the pair and computes, per run:

* **attitude / velocity / position** against the simulator's truth (T tier, so
  every name ends in ``_truth``): the tilt ∠(Rᵀe_z, R̂ᵀe_z), the yaw of
  R R̂ᵀ, the SO(3) geodesic angle, the base-frame velocity error, and the
  horizontal position drift per touchdown / per second (with and without the
  part the yaw error explains).
* **consistency**: NEES on the observable 5 dof (world tilt x, y and the IMU
  velocity) of the right-invariant error X = Exp(ξ) X̂ — ξ_φ = Log(R R̂ᵀ),
  ξ_v = J_l(ξ_φ)⁻¹ (v − R R̂ᵀ v̂) — against the φ·v block of P; NIS of every
  requested contact update (used or gated), 3 dof each.
* **contact**: the truth contact (the normal force over a fraction of the
  weight, ``walk_metrics.measured_contact`` hysteresis, a dwell) against the
  estimate (``est_mask``: stance ∪ unknown) and the planned mask as a
  baseline — precision / recall with and without the ticks near a truth edge,
  touchdown / liftoff latency, slip false positives, the time in each mode.
* **accelerometer check**: the measured specific force against the one the
  truth IMU-point trajectory implies (lever arm included).
* **walking and standing parts**: the run splits at the first planned
  lift-off (the first row a contact of the full planned stance is off). The
  NEES is also averaged over each part, and the z gyro bias estimate is held
  against the run's measured offset (the mean of the measured z rate minus the
  truth one at the IMU — a constant bias) over the walking part.
* **resets and slip cycling**: the resets after the first started row (its
  own reset, the activation's, is not one) and, per foot, the entries into slip
  within a trailing :data:`SLIP_WINDOW_S` window — a *cycle* is that count at
  :data:`SLIP_CYCLE_ENTRIES` or more for :data:`SLIP_CYCLE_S` or longer
  (WD-359). On an inekf-driven run the same counts of the logged online
  estimate (``online_source``) are what the controller ran on.
* **online comparison** (an inekf-driven run): the replay against the logged
  online estimate — the first tick a discrete field differs, the continuous
  difference before it, and whether those rows cover the scored ones.

The gauge (yaw and translation are unobservable) is aligned at the start of
every state epoch (a reset starts the estimate over at a new origin): the yaw
ψ₀ = atan2(E₁₀, E₀₀) of E = R R̂ᵀ and the translation that puts the base on
the truth at the epoch's first truth row. Ticks
are scored where the estimate has started and a truth row exists, up to
:func:`cut_row` (a fault, a fall: the rows after it are not scored, so a
reset or a slip that follows a fall is not counted); NEES skips
held ticks (``periods`` ≠ ≥ 1). A K-run summary (:func:`aggregate`) judges the
run-level ANEES / ANIS against the χ²(K·dof)/(K·dof) 95 % band — each run's
time average taken as one sample, a band no narrower than the true one — for
the whole run and for its walking and standing parts.

Pure numpy; no robot constant (weights, extrinsic and names come from the
sidecar and the file).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

from rtc_tools.analysis.locomotion_diag import read_table
from rtc_tools.analysis.walk_metrics import (
    edges,
    first_lift_row,
    mask_bits,
    match_edges,
    measured_contact,
    rotation_matrices,
    steps,
)

Table = dict[str, np.ndarray]

# rtc::estimation::ContactMode / ContactUpdateResult codes.
MODE_NAMES = ("swing", "candidate", "stance", "slip", "unknown")
MODE_STANCE = 2
MODE_SLIP = 3
UPDATE_USED = 2
UPDATE_GATED = 3

NEES_DOF = 5
NIS_DOF = 3
BAND = 0.95

# The truth contact (WE-F03's definition): on above 3 % of the weight, off
# below 2 %, a state kept at least 20 ms; ticks within 60 ms (plus the
# planned edge's own offset) of a truth edge are the "near an edge" ones.
CONTACT_ON = 0.03
CONTACT_OFF = 0.02
CONTACT_DWELL_S = 0.020
EDGE_MARGIN_S = 0.060
EDGE_MATCH_S = 0.5
LATENCY_WINDOW_S = 0.3
SLIP_SPEED_MPS = 0.02

# Slip cycling (WD-359): the entries into slip of one foot on the rows of a
# trailing window; a cycle holds the count at ≥ SLIP_CYCLE_ENTRIES for
# ≥ SLIP_CYCLE_S. The stretch at ≥ SLIP_REPORT_ENTRIES is reported beside it.
SLIP_WINDOW_S = 0.5
SLIP_CYCLE_ENTRIES = 12
SLIP_CYCLE_S = 1.0
SLIP_REPORT_ENTRIES = 8
# The online estimate's velocity error is also reported over the last second before the cut.
PRE_CUT_S = 1.0

# Online comparison bounds before the first discrete divergence (WD-305 (ii)).
ONLINE_ROT_RAD = 1e-4
ONLINE_VEL_MPS = 1e-4
ONLINE_POS_M = 1e-3
DISCRETE = ("initialized", "reset_count", "reset_reason", "periods", "hold_ticks",
            "step_status", "report_valid", "est_mask")  # fmt: skip


# ── SO(3) ──────────────────────────────────────────────────────────────────


def skew(w: np.ndarray) -> np.ndarray:
    """(n, 3, 3) [w]× of (n, 3) vectors."""
    s = np.zeros((*w.shape[:-1], 3, 3))
    s[..., 0, 1], s[..., 0, 2] = -w[..., 2], w[..., 1]
    s[..., 1, 0], s[..., 1, 2] = w[..., 2], -w[..., 0]
    s[..., 2, 0], s[..., 2, 1] = -w[..., 1], w[..., 0]
    return s


def _vee_asym(r: np.ndarray) -> np.ndarray:
    return np.stack(
        [r[..., 2, 1] - r[..., 1, 2], r[..., 0, 2] - r[..., 2, 0], r[..., 1, 0] - r[..., 0, 1]],
        axis=-1,
    )


def angle_of(r: np.ndarray) -> np.ndarray:
    """Rotation angles, atan2(‖vee(R − Rᵀ)‖/2, (tr R − 1)/2) — exact near 0 too."""
    return np.arctan2(
        np.linalg.norm(_vee_asym(r), axis=-1) / 2.0,
        (np.trace(r, axis1=-2, axis2=-1) - 1.0) / 2.0,
    )


def so3_log(r: np.ndarray) -> np.ndarray:
    """(n, 3) rotation vectors of (n, 3, 3) rotations (angle < π)."""
    theta = angle_of(r)
    vee = _vee_asym(r)
    small = theta < 1e-6
    scale = np.where(
        small, 0.5 + theta**2 / 12.0, theta / (2.0 * np.sin(np.where(small, 1.0, theta)))
    )
    return vee * scale[..., None]


def left_jacobian_inverse(phi: np.ndarray) -> np.ndarray:
    """(n, 3, 3) J_l(φ)⁻¹ = I − ½[φ]× + (1/θ² − (1 + cos θ)/(2θ sin θ)) [φ]×²."""
    theta = np.linalg.norm(phi, axis=-1)
    k = skew(phi)
    small = theta < 1e-4
    t = np.where(small, 1.0, theta)
    coef = np.where(small, 1.0 / 12.0 + theta**2 / 720.0,
                    1.0 / t**2 - (1.0 + np.cos(t)) / (2.0 * t * np.sin(t)))  # fmt: skip
    eye = np.broadcast_to(np.eye(3), k.shape)
    return eye - 0.5 * k + coef[..., None, None] * (k @ k)


def rot_z(psi: np.ndarray) -> np.ndarray:
    c, s = np.cos(psi), np.sin(psi)
    r = np.zeros((*np.shape(psi), 3, 3))
    r[..., 0, 0], r[..., 0, 1], r[..., 1, 0], r[..., 1, 1] = c, -s, s, c
    r[..., 2, 2] = 1.0
    return r


def yaw_of(r: np.ndarray) -> np.ndarray:
    return np.arctan2(r[..., 1, 0], r[..., 0, 0])


# ── χ² quantiles (no scipy) ────────────────────────────────────────────────


def _gamma_p(a: float, x: float) -> float:
    """Regularised lower incomplete gamma P(a, x) (series / continued fraction)."""
    if x <= 0.0:
        return 0.0
    gln = math.lgamma(a)
    if x < a + 1.0:
        term = total = 1.0 / a
        n = a
        for _ in range(10000):
            n += 1.0
            term *= x / n
            total += term
            if abs(term) < abs(total) * 4e-16:
                break
        return total * math.exp(-x + a * math.log(x) - gln)
    b = x + 1.0 - a
    c = 1.0 / 1e-300
    d = 1.0 / b
    h = d
    for i in range(1, 10000):
        an = -i * (i - a)
        b += 2.0
        d = an * d + b
        d = 1e-300 if abs(d) < 1e-300 else d
        c = b + an / c
        c = 1e-300 if abs(c) < 1e-300 else c
        d = 1.0 / d
        delta = d * c
        h *= delta
        if abs(delta - 1.0) < 4e-16:
            break
    return 1.0 - math.exp(-x + a * math.log(x) - gln) * h


def chi2_quantile(p: float, dof: int) -> float:
    """x with P(χ²_dof ≤ x) = p, by bisection on the CDF."""
    lo, hi = 0.0, max(10.0, 10.0 * dof)
    while _gamma_p(dof / 2.0, hi / 2.0) < p:
        hi *= 2.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if _gamma_p(dof / 2.0, mid / 2.0) < p:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def mean_band(dof_total: int, level: float = BAND) -> tuple[float, float]:
    """The two-sided band of χ²(n)/n."""
    a = (1.0 - level) / 2.0
    return chi2_quantile(a, dof_total) / dof_total, chi2_quantile(1.0 - a, dof_total) / dof_total


# ── reading ────────────────────────────────────────────────────────────────


def load_replay(path: Path) -> tuple[Table, dict]:
    """The replay CSV's columns and its sidecar."""
    _, cols = read_table(Path(path))
    meta = json.loads(Path(f"{path}.json").read_text())
    return cols, meta


def _quat(cols: Table, prefix: str) -> np.ndarray:
    return rotation_matrices(*(cols[f"{prefix}q{a}"] for a in "xyzw"))


def _vec(cols: Table, prefix: str, axes: str = "xyz") -> np.ndarray:
    return np.stack([cols[f"{prefix}{a}"] for a in axes], axis=-1)


# ── the truth contact ──────────────────────────────────────────────────────


def debounce(state: np.ndarray, t: np.ndarray, min_s: float) -> np.ndarray:
    """A state that lasts less than `min_s` reads as the one before it."""
    out = state.copy()
    if len(out) == 0:
        return out
    start = 0
    for i in range(1, len(out) + 1):
        if i == len(out) or out[i] != out[start]:
            if start > 0 and t[min(i, len(t) - 1)] - t[start] < min_s and i < len(out):
                out[start:i] = out[start - 1]
            start = i
    return out


def truth_contact(fz: np.ndarray, t: np.ndarray, weight_n: float) -> np.ndarray:
    """(n,) truth contact of one foot from its floor-on-foot normal force."""
    return debounce(measured_contact(fz, CONTACT_ON * weight_n, CONTACT_OFF * weight_n), t,
                    CONTACT_DWELL_S)  # fmt: skip


def near_edges(gt: np.ndarray, planned: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Ticks within EDGE_MARGIN_S + |planned − truth edge| of a truth edge."""
    near = np.zeros(len(t), dtype=bool)
    p_edges = edges(planned, t)
    for te, rising in edges(gt, t):
        best = 0.0
        cand = [abs(tp - te) for tp, r in p_edges if r == rising and abs(tp - te) <= EDGE_MATCH_S]
        if cand:
            best = min(cand)
        near |= np.abs(t - te) <= EDGE_MARGIN_S + best
    return near


def _pr(est: np.ndarray, gt: np.ndarray) -> dict:
    tp = int(np.sum(est & gt))
    fp = int(np.sum(est & ~gt))
    fn = int(np.sum(~est & gt))
    return {
        "precision": tp / (tp + fp) if tp + fp else None,
        "recall": tp / (tp + fn) if tp + fn else None,
        "ticks": int(len(est)),
    }


def _stats(x) -> dict | None:
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return None
    return {"median": float(np.median(x)), "p95": float(np.percentile(np.abs(x), 95)),
            "n": int(x.size)}  # fmt: skip


def _rms(x: np.ndarray) -> float | None:
    x = x[np.isfinite(x)]
    return float(math.sqrt(np.mean(x * x))) if x.size else None


# ── one run ────────────────────────────────────────────────────────────────


def epochs(cols: Table) -> list[np.ndarray]:
    """Row indices of each started state epoch (in log order)."""
    on = cols["initialized"] == 1
    epoch = cols["state_epoch"]
    out = []
    i = 0
    n = len(epoch)
    while i < n:
        if not on[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and on[j + 1] and epoch[j + 1] == epoch[i]:
            j += 1
        out.append(np.arange(i, j + 1))
        i = j + 1
    return out


def slip_entries(mode: np.ndarray, t: np.ndarray, window_s: float = SLIP_WINDOW_S) -> np.ndarray:
    """(n,) entries into slip on the rows in (t − `window_s`, t] — an entry is a slip
    row after a row in another mode (the first row is none)."""
    entry = np.zeros(len(mode))
    if len(mode) > 1:
        entry[1:] = (mode[1:] == MODE_SLIP) & (mode[:-1] != MODE_SLIP)
    total = np.concatenate([[0.0], np.cumsum(entry)])
    first = np.searchsorted(t, t - window_s, side="right")
    return total[1:] - total[first]


def longest_span(t: np.ndarray, on: np.ndarray) -> float:
    """The longest t[last] − t[first] over a stretch of consecutive `on` rows (0 without one)."""
    on = np.asarray(on, dtype=bool)
    if not on.any():
        return 0.0
    d = np.diff(np.concatenate([[0], on.astype(np.int8), [0]]))
    starts, ends = np.flatnonzero(d == 1), np.flatnonzero(d == -1) - 1
    return float(np.max(t[ends] - t[starts]))


def slip_cycle(cols: Table, names: list[str], prefix: str = "mode_") -> dict:
    """Per foot: the window's most entries, the longest stretch at ≥ SLIP_CYCLE_ENTRIES
    (``cycle_span_s``) and at ≥ SLIP_REPORT_ENTRIES (``report_span_s``), whether it
    cycled; ``feet_cycling`` counts the feet that did. `prefix` names the mode columns."""
    t = cols["t_relative_s"]
    feet = {}
    for c in names:
        count = slip_entries(cols[f"{prefix}{c}"], t)
        span = longest_span(t, count >= SLIP_CYCLE_ENTRIES)
        feet[c] = {
            "window_max": int(count.max()) if count.size else 0,
            "cycle_span_s": span,
            "report_span_s": longest_span(t, count >= SLIP_REPORT_ENTRIES),
            # The rows sit on a tick grid: a stretch of exactly SLIP_CYCLE_S may read 1 ulp short.
            "cycle": bool(span >= SLIP_CYCLE_S - 1e-9),
        }
    return {"feet": feet, "feet_cycling": sum(f["cycle"] for f in feet.values())}


def resets_after_start(cols: Table, prefix: str = "") -> int | None:
    """Resets after the first started row — that row's own reset (the activation's) is
    not one; None when the estimate never started. `prefix` names the columns."""
    started = np.flatnonzero(cols[f"{prefix}initialized"] == 1)
    if not started.size:
        return None
    count = cols[f"{prefix}reset_count"][started[0] :]
    return int(np.nanmax(count) - count[0])


# A fall: the truth base below this fraction of its height at the first truth row.
FALL_HEIGHT_FRACTION = 0.6


def cut_row(cols: Table) -> tuple[int, str]:
    """The first row not scored: a latched fault, a fall, or the end (WD-306)."""
    n = len(cols["tick"])
    cut, why = n, "end"
    if "online_fault_latched" in cols:
        f = np.flatnonzero(cols["online_fault_latched"] == 1)
        if f.size and f[0] < cut:
            cut, why = int(f[0]), "fault"
    z = cols.get("truth_pz")
    if z is not None:
        ok = np.flatnonzero(np.isfinite(z))
        if ok.size:
            low = np.flatnonzero(z < FALL_HEIGHT_FRACTION * z[ok[0]])
            if low.size and low[0] < cut:
                cut, why = int(low[0]), "fall"
    return cut, why


def run_metrics(cols: Table, meta: dict) -> dict:
    """Every metric of one replayed run (module docstring), up to :func:`cut_row`."""
    cut, why = cut_row(cols)
    cols = {k: v[:cut] for k, v in cols.items()}
    names = list(meta["contacts"])
    nc = len(names)
    t = cols["t_relative_s"]
    n = len(t)
    r_bi = np.asarray(meta["R_BI"], dtype=float).reshape(3, 3)
    p_bi = np.asarray(meta["p_BI"], dtype=float)
    g = float(meta["gravity"])
    out: dict = {"rows": n, "cut": why, "contacts": names}
    for k in ("diag_rows", "unmatched_rows", "gaps", "missing_ticks", "runs",
              "approx_joint_ticks", "update_ns"):  # fmt: skip
        out[k] = meta.get(k)

    started = cols["initialized"] == 1
    have_truth = cols["truth_valid"] == 1
    out["started_ticks"] = int(started.sum())
    first = np.flatnonzero(started)
    out["start_tick"] = int(cols["tick"][first[0]]) if first.size else None
    out["start_t_s"] = float(t[first[0]]) if first.size else None
    out["resets"] = int(np.nanmax(cols["reset_count"])) if n else 0
    out["resets_after_start"] = resets_after_start(cols)
    lift = first_lift_row(cols["planned_mask"], nc) if n else None
    walking = np.zeros(n, dtype=bool)
    if lift is not None:
        walking[lift:] = True
    out["lift_t_s"] = float(t[lift]) if lift is not None else None

    # ── the gauge, per epoch ──────────────────────────────────────────────
    r_hat = _quat(cols, "base_")  # R_WB estimate
    p_hat = _vec(cols, "base_p")
    v_hat_b = _vec(cols, "base_v")
    r_true = _quat(cols, "truth_")
    p_true = _vec(cols, "truth_p")
    v_true_b = _vec(cols, "truth_v")
    w_true_b = _vec(cols, "truth_w")
    psi = np.full(n, math.nan)
    trans = np.full((n, 3), math.nan)
    epoch_rows = []
    for rows in epochs(cols):
        rows = rows[have_truth[rows]]
        if rows.size == 0:
            continue
        k0 = rows[0]
        psi0 = yaw_of(r_true[k0] @ r_hat[k0].T)
        trans0 = p_true[k0] - rot_z(psi0) @ p_hat[k0]
        psi[rows] = psi0
        trans[rows] = trans0
        epoch_rows.append(rows)
    scored = np.isfinite(psi)
    out["scored_ticks"] = int(scored.sum())
    rz = rot_z(np.where(scored, psi, 0.0))
    r_al = rz @ r_hat  # the estimate on the truth's gauge
    p_al = np.einsum("nij,nj->ni", rz, p_hat) + np.where(scored[:, None], trans, 0.0)

    # ── attitude, velocity ────────────────────────────────────────────────
    ez = np.array([0.0, 0.0, 1.0])
    up_true = np.einsum("nji,j->ni", r_true, ez)
    up_hat = np.einsum("nji,j->ni", r_hat, ez)
    tilt = np.arccos(np.clip(np.sum(up_true * up_hat, axis=1), -1.0, 1.0))
    rel = r_true @ np.swapaxes(r_al, 1, 2)
    yaw_err = np.arctan2(rel[:, 1, 0], rel[:, 0, 0])
    geo = angle_of(rel)
    vel = np.linalg.norm(v_true_b - v_hat_b, axis=1)
    s = scored
    out["tilt_rms_truth"] = _rms(tilt[s])
    out["tilt_max_truth"] = float(np.max(tilt[s])) if s.any() else None
    out["yaw_rms_truth"] = _rms(yaw_err[s])
    out["att_err_rms_truth"] = _rms(geo[s])
    out["vel_err_rms_truth"] = _rms(vel[s])
    out["vel_err_max_truth"] = float(np.max(vel[s])) if s.any() else None

    # ── the truth contact ─────────────────────────────────────────────────
    fz = np.stack([cols[f"truth_fz_{c}"] for c in names], axis=1)
    planned = mask_bits(cols["planned_mask"], nc)
    est = mask_bits(cols["est_mask"], nc)
    both = np.isfinite(fz).all(axis=1) & planned.all(axis=1)
    weight = float(np.median(fz[both].sum(axis=1))) if both.any() else float(meta["weight_n"])
    out["weight_n"] = weight
    gt = np.stack([truth_contact(fz[:, c], t, weight) for c in range(nc)], axis=1)
    defined = np.isfinite(fz)

    # ── position drift ────────────────────────────────────────────────────
    drift = drift_noyaw = duration = 0.0
    touchdowns = 0
    for rows in epoch_rows:
        k0, k1 = rows[0], rows[-1]
        e0 = np.linalg.norm((p_true[k0] - p_al[k0])[:2])
        e1 = np.linalg.norm((p_true[k1] - p_al[k1])[:2])
        drift += e1 - e0
        d_true = p_true[k1] - p_true[k0]
        d_hat = rot_z(yaw_err[k1]) @ (p_al[k1] - p_al[k0])
        drift_noyaw += float(np.linalg.norm((d_true - d_hat)[:2]))
        duration += float(t[k1] - t[k0])
        touchdowns += steps(gt[rows], t[rows], float(t[k0]), float(t[k1]) + 1e-9)
    out["pos_drift_m_truth"] = drift
    out["pos_drift_noyaw_m_truth"] = drift_noyaw
    out["touchdowns_truth"] = touchdowns
    out["scored_s"] = duration
    out["pos_drift_per_s_truth"] = drift / duration if duration > 0 else None
    out["pos_drift_noyaw_per_s_truth"] = drift_noyaw / duration if duration > 0 else None
    out["pos_drift_per_step_truth"] = drift / touchdowns if touchdowns else None
    out["pos_drift_noyaw_per_step_truth"] = drift_noyaw / touchdowns if touchdowns else None

    # ── NEES (IMU point, the filter's own gauge) ──────────────────────────
    r_wi_true = r_true @ r_bi
    v_wi_true = np.einsum("nij,nj->ni", r_true, v_true_b + np.cross(w_true_b, p_bi))
    rz_t = np.swapaxes(rz, 1, 2)
    r_t = rz_t @ r_wi_true  # truth on the filter's gauge
    v_t = np.einsum("nij,nj->ni", rz_t, v_wi_true)
    r_imu = _quat(cols, "imu_")
    v_imu = _vec(cols, "imu_v")
    nees_rows = scored & (cols["periods"] >= 1)
    nees = np.full(n, math.nan)
    if nees_rows.any():
        idx = np.flatnonzero(nees_rows)
        e_r = r_t[idx] @ np.swapaxes(r_imu[idx], 1, 2)
        xi_phi = so3_log(e_r)
        xi_v = np.einsum("nij,nj->ni", left_jacobian_inverse(xi_phi),
                         v_t[idx] - np.einsum("nij,nj->ni", e_r, v_imu[idx]))  # fmt: skip
        e = np.concatenate([xi_phi[:, :2], xi_v], axis=1)
        p = np.zeros((idx.size, 6, 6))
        for i in range(6):
            for j in range(i, 6):
                p[:, i, j] = p[:, j, i] = cols[f"P_{i}_{j}"][idx]
        keep = [0, 1, 3, 4, 5]
        s5 = p[:, keep][:, :, keep]
        # A block that is not symmetric positive definite has no NEES (counted).
        spd = np.all(np.isfinite(s5), axis=(1, 2))
        spd[spd] = np.linalg.eigvalsh(s5[spd]).min(axis=1) > 0.0
        good = idx[spd]
        nees[good] = np.einsum(
            "ni,ni->n", e[spd], np.linalg.solve(s5[spd], e[spd][..., None])[..., 0]
        )
        out["nees_non_spd_ticks"] = int((~spd).sum())
    lo5, hi5 = chi2_quantile(0.025, NEES_DOF), chi2_quantile(0.975, NEES_DOF)
    nv = nees[np.isfinite(nees)]
    out["nees_ticks"] = int(nv.size)
    out["nees_mean_per_dof_truth"] = float(np.mean(nv) / NEES_DOF) if nv.size else None
    out["nees_in_band_fraction_truth"] = (
        float(np.mean((nv >= lo5) & (nv <= hi5))) if nv.size else None
    )
    for part, rows in (("walk", walking), ("stand", ~walking)):
        v = nees[rows & np.isfinite(nees)]
        out[f"nees_{part}_ticks"] = int(v.size)
        out[f"nees_{part}_mean_per_dof_truth"] = float(np.mean(v) / NEES_DOF) if v.size else None

    # ── z gyro bias against the run's measured offset (a constant bias) ───
    out["gyro_bias_z_truth"] = out["gyro_bias_z_dev_max_truth"] = None
    if "bg_z" in cols and "in_imu_gz" in cols:
        ok = have_truth & (cols["in_imu_valid"] == 1)
        if ok.any():
            w_imu_z = np.einsum("j,nj->n", r_bi[:, 2], w_true_b)  # (R_BIᵀ ω_B)_z
            bias = float(np.mean((cols["in_imu_gz"] - w_imu_z)[ok]))
            sel = walking & started & np.isfinite(cols["bg_z"])
            out["gyro_bias_z_truth"] = bias
            out["gyro_bias_z_dev_max_truth"] = (
                float(np.max(np.abs(cols["bg_z"][sel] - bias))) if sel.any() else None
            )

    # ── NIS (every requested update: used or gated) ───────────────────────
    nis: list[float] = []
    rep = cols["report_valid"] == 1
    for c in names:
        upd = cols[f"update_{c}"]
        sel = rep & ((upd == UPDATE_USED) | (upd == UPDATE_GATED))
        nis += list(cols[f"m2_{c}"][sel])
    nis_a = np.asarray(nis, dtype=float)
    nis_a = nis_a[np.isfinite(nis_a)]
    lo3, hi3 = chi2_quantile(0.025, NIS_DOF), chi2_quantile(0.975, NIS_DOF)
    out["nis_updates"] = int(nis_a.size)
    out["nis_mean_per_dof"] = float(np.mean(nis_a) / NIS_DOF) if nis_a.size else None
    out["nis_in_band_fraction"] = (
        float(np.mean((nis_a >= lo3) & (nis_a <= hi3))) if nis_a.size else None
    )

    # ── contact ───────────────────────────────────────────────────────────
    contact: dict = {}
    for ci, c in enumerate(names):
        d = defined[:, ci] & started
        near = near_edges(gt[:, ci], planned[:, ci], t)
        far = d & ~near
        mode = cols[f"mode_{c}"]
        speed = np.linalg.norm(np.stack([cols[f"truth_v_{c}_{a}"] for a in "xyz"], axis=1), axis=1)
        still = d & gt[:, ci] & planned[:, ci] & (speed < SLIP_SPEED_MPS)
        dt_td = match_edges([e for e in edges(gt[d, ci], t[d]) if e[1]],
                            [e for e in edges(est[d, ci], t[d]) if e[1]], LATENCY_WINDOW_S)  # fmt: skip
        dt_lo = match_edges([e for e in edges(gt[d, ci], t[d]) if not e[1]],
                            [e for e in edges(est[d, ci], t[d]) if not e[1]], LATENCY_WINDOW_S)  # fmt: skip
        contact[c] = {
            "estimate_all": _pr(est[d, ci], gt[d, ci]),
            "estimate_far_from_edges": _pr(est[far, ci], gt[far, ci]),
            "planned_all": _pr(planned[d, ci], gt[d, ci]),
            "planned_far_from_edges": _pr(planned[far, ci], gt[far, ci]),
            "touchdown_latency_s": _stats(dt_td.dt_s),
            "touchdown_missed": dt_td.missed,
            "touchdown_extra": dt_td.extra,
            "liftoff_latency_s": _stats(dt_lo.dt_s),
            "liftoff_missed": dt_lo.missed,
            "liftoff_extra": dt_lo.extra,
            "still_ticks": int(still.sum()),
            "slip_false_positive": float(np.mean(mode[still] == MODE_SLIP))
            if still.any()
            else None,
            "out_of_stance_when_still": (
                float(np.mean(mode[still] != MODE_STANCE)) if still.any() else None
            ),
            "mode_fraction": {
                m: float(np.mean(mode[started] == i)) if started.any() else None
                for i, m in enumerate(MODE_NAMES)
            },
        }
    out["contact"] = contact
    out["slip_cycle"] = slip_cycle(cols, names)

    # ── the online estimate the controller ran on (an inekf-driven run) ───
    if "online_initialized" in cols and bool((cols["online_initialized"] == 1).any()):
        on = (cols["online_initialized"] == 1) & have_truth
        last = on & (t >= t[-1] - PRE_CUT_S)
        dv = np.linalg.norm(v_true_b - _vec(cols, "online_base_v"), axis=1)
        out["online_source"] = {
            "resets_after_start": resets_after_start(cols, "online_"),
            "slip_cycle": slip_cycle(cols, names, "online_mode_"),
            "vel_err_rms_truth": _rms(dv[on]),
            "vel_err_rms_last_s_truth": _rms(dv[last]),
        }

    # ── accelerometer: measured vs the truth IMU point's specific force ───
    out["accel_check"] = accel_check(cols, r_wi_true, v_wi_true, g)
    return out


def accel_check(cols: Table, r_wi: np.ndarray, v_wi: np.ndarray, g: float) -> dict | None:
    """Residual of the raw accelerometer against R_WIᵀ (a_WI − g_W), a_WI by central differences."""
    t = cols["t_relative_s"]
    n = len(t)
    if n < 3:
        return None
    ok = np.zeros(n, dtype=bool)
    ok[1:-1] = (
        (cols["truth_valid"][:-2] == 1) & (cols["truth_valid"][1:-1] == 1) & (cols["truth_valid"][2:] == 1)
        & (cols["gap"][1:-1] == 0) & (cols["gap"][2:] == 0) & (cols["in_imu_valid"][1:-1] == 1)
    )  # fmt: skip
    idx = np.flatnonzero(ok)
    if idx.size == 0:
        return None
    dt = t[idx + 1] - t[idx - 1]
    good = dt > 0
    idx, dt = idx[good], dt[good]
    a = (v_wi[idx + 1] - v_wi[idx - 1]) / dt[:, None]
    g_w = np.array([0.0, 0.0, -g])
    pred = np.einsum("nji,nj->ni", r_wi[idx], a - g_w)
    meas = _vec(cols, "in_imu_a")[idx]
    res = meas - pred
    return {
        "ticks": int(idx.size),
        "residual_mean": [float(x) for x in np.mean(res, axis=0)],
        "residual_rms": [float(x) for x in np.sqrt(np.mean(res * res, axis=0))],
        "residual_norm_p99": float(np.percentile(np.linalg.norm(res, axis=1), 99)),
        "measured_norm_p99": float(np.percentile(np.linalg.norm(meas, axis=1), 99)),
    }


def compare_online(cols: Table, meta: dict) -> dict:
    """The replay against the logged online estimate (an inekf-driven run)."""
    names = list(meta["contacts"])
    n = len(cols["tick"])
    stop = n
    reason = "end"
    gap = np.flatnonzero(cols["gap"][1:] > 0)
    if gap.size:
        stop = int(gap[0]) + 1
        reason = "gap"
    fields = [*DISCRETE, *(f"mode_{c}" for c in names), *(f"update_{c}" for c in names)]
    diverge = None
    for i in range(stop):
        bad = [f for f in fields if cols[f][i] != cols[f"online_{f}"][i]]
        if bad:
            diverge = {"row": i, "tick": int(cols["tick"][i]), "fields": bad}
            break
    end = diverge["row"] if diverge else stop
    rows = np.arange(end)
    rows = rows[cols["initialized"][rows] == 1]
    # The rows the run's metrics score (up to its cut) are the ones the comparison must cover.
    cut, _ = cut_row(cols)
    out: dict = {"compared_rows": int(end), "stop": reason if diverge is None else "diverged",
                 "first_divergence": diverge, "covers_scored_rows": bool(end >= cut)}  # fmt: skip
    if rows.size:
        r_a = _quat(cols, "base_")[rows]
        r_b = _quat(cols, "online_base_")[rows]
        rot = angle_of(r_a @ np.swapaxes(r_b, 1, 2))
        dv = np.abs(_vec(cols, "base_v")[rows] - _vec(cols, "online_base_v")[rows]).max(axis=1)
        dp = np.abs(_vec(cols, "base_p")[rows] - _vec(cols, "online_base_p")[rows]).max(axis=1)
        out["max_rot_rad"] = float(rot.max())
        out["max_vel_mps"] = float(dv.max())
        out["max_pos_m"] = float(dp.max())
        out["within_bounds"] = bool(
            rot.max() <= ONLINE_ROT_RAD and dv.max() <= ONLINE_VEL_MPS and dp.max() <= ONLINE_POS_M
        )
    return out


def aggregate(runs: list[dict]) -> dict:
    """The K-run consistency verdict (module docstring).

    Every ANEES / ANIS entry holds its run mean, the χ² band at its K and
    ``pass`` (inside the band); ``below_upper`` is the one-sided half, the
    verdict where a value under the band only means a conservative filter.
    """
    out: dict = {"runs": len(runs)}
    for key, dof, name in (("nees_mean_per_dof_truth", NEES_DOF, "anees_truth"),
                           ("nees_walk_mean_per_dof_truth", NEES_DOF, "anees_walk_truth"),
                           ("nees_stand_mean_per_dof_truth", NEES_DOF, "anees_stand_truth"),
                           ("nis_mean_per_dof", NIS_DOF, "anis")):  # fmt: skip
        vals = [r[key] for r in runs if r.get(key) is not None]
        if not vals:
            out[name] = None
            continue
        k = len(vals)
        lo, hi = mean_band(k * dof)
        mean = float(np.mean(vals))
        out[name] = {"value": mean, "band": [lo, hi], "k": k, "pass": bool(lo <= mean <= hi),
                     "below_upper": bool(mean <= hi)}  # fmt: skip
    out["runs_cycling"] = sum(bool((r.get("slip_cycle") or {}).get("feet_cycling")) for r in runs)
    for key in ("tilt_rms_truth", "yaw_rms_truth", "vel_err_rms_truth", "pos_drift_per_step_truth",
                "pos_drift_per_s_truth"):  # fmt: skip
        vals = [r[key] for r in runs if r.get(key) is not None]
        out[key] = {"mean": float(np.mean(vals)), "max": float(np.max(vals))} if vals else None
    return out


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("replays", nargs="+", type=Path, help="estimator_replay CSVs")
    p.add_argument("--out", type=Path, help="write the summary JSON here")
    p.add_argument("--compare-online", action="store_true",
                   help="also compare the replay with the logged online estimate")  # fmt: skip
    args = p.parse_args(argv)
    runs = []
    for path in args.replays:
        cols, meta = load_replay(path)
        r = run_metrics(cols, meta)
        r["file"] = str(path)
        if args.compare_online:
            r["online"] = compare_online(cols, meta)
        runs.append(r)
    summary = {"runs": runs, "aggregate": aggregate(runs)}
    text = json.dumps(summary, indent=2)
    if args.out:
        args.out.write_text(text + "\n")
    print(json.dumps(summary["aggregate"], indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
