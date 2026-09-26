#!/usr/bin/env python3
"""Vision-side S8-E gates: G8-B (prediction NEES) and G8-C2 (A⊥B), per trial.

Plan: ``docs/dynamic_catching/IMPLEMENTATION_PLAN.md`` §4.4 D-S8-16 ②b/②c and
L8 §9.1 (G8-B, G8-C2). Pure functions over the files the S8-E capture leaves
behind — no ROS, no pinocchio — called by
:mod:`rtc_tools.analysis.catching_trials` (per session, one row per trial) and
:mod:`rtc_tools.analysis.catching_pool` (per arm, from the pooled per-trial
columns). Frames and time axes are the caller's business; the contract here is:

* every time is on the ball's **stamp axis** [ns] — the sim ball stamps are
  launch-anchored (``rtc_mujoco_sim`` ``ProjectileBallStampSteadyNs``: the
  flight's sim time laid onto the wall from the launch instant), and the
  estimator's predictions, ``sim_capture_evaluate`` samples and the probe dump
  all carry them;
* every position is in the frame of the probe dump / capture (``world`` — the
  SIM world; the caller maps the controller's model-world ``p_c`` into it).

**G8-B** (:func:`nees_by_trial`, :func:`g8b_summary`). A ``sim_capture_evaluate``
prediction sample is bound to a trial when its origin stamp is at or after the
launch and its TARGET stamp (origin + horizon) is at or before the trial's first
contact: after that the evaluator compares against the ball in the hand or
reset, which says nothing about the prediction. Per horizon the pooled mean
NEES is ``Σ NEES / Σ n`` over trials (exact from the per-trial means and
counts), its CI a trial-cluster bootstrap; PASS when the CI contains the
position dof 3 (D-S8-16 ②c).

**G8-C2** (:func:`plan_point_stamp`, :func:`p_hat_at`, :func:`c2_summary`).
``A = p_true(t_c) − p̂_live(t_c)``, ``B = p̂_live(t_c) − p_c`` at the plan's catch
instant, where p̂_live is the snapshot the RT tick committed on (diag
``input_snapshot_sequence`` / ``input_generation``, joined exactly to the dump's
``snapshot_sequence`` / ``generation``).
"""

from __future__ import annotations

import math
import re
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from rtc_tools.analysis.table_cells import num as _num

POSITION_DOF = 3
CHI2_3_95 = 7.814727903251178  # scipy.stats.chi2.ppf(0.95, 3)
NEES_NAN_MAX_SHARE = 0.10  # D-S8-16 ②c: a horizon bin with more NaN NEES is not evaluated
LAUNCH_TOL_NS = 1_000_000  # a sample stamped at the launch instant (float round-off)
C2_N_MIN = 100  # L8 §9.1 G8-C2: n ≥ 100
PLAN_POINT_TOL_M = 1e-3  # p_c is a trajectory sample of the plan's snapshot — exact up to I/O
DUMP_SUB_PREFERENCE = ("reliable", "best_effort")
_NEES_COL = re.compile(r"^nees_h(.+)ms_n$")


def horizon_label(horizon_ns: float) -> str:
    """``100000000`` → ``"100"`` (ms, no trailing ``.0``)."""
    ms = float(horizon_ns) / 1e6
    return f"{ms:g}"


# ══ G8-B ══════════════════════════════════════════════════════════════════════


def load_eval_samples(path: Path):
    """``sim_capture_evaluate`` ``eval_samples.csv``: the PREDICTION rows, sorted by origin.

    ``state`` rows (horizon 0, the filter's own estimate) are not G8-B's
    subject and are dropped.
    """
    import pandas as pd  # noqa: PLC0415

    df = pd.read_csv(path)
    need = {"record", "time_ns", "horizon_ns", "position_nees"}
    missing = need - set(df.columns)
    if missing:
        raise SystemExit(f"{path}: missing column(s) {sorted(missing)}")
    df = df[df["record"] == "prediction"].copy()
    df["time_ns"] = df["time_ns"].astype(np.int64)
    df["horizon_ns"] = df["horizon_ns"].astype(np.int64)
    return df.sort_values("time_ns").reset_index(drop=True)


def nees_by_trial(samples, lo_ns: float, hi_ns: float) -> dict:
    """Per-horizon NEES statistics of the samples bound to one trial.

    Bound = origin ``time_ns`` ≥ ``lo_ns`` (the launch, less
    :data:`LAUNCH_TOL_NS`) and target ``time_ns + horizon_ns`` ≤ ``hi_ns`` (the
    first contact). Columns per horizon ``h`` [ms]: ``nees_h<h>ms_n`` (bound
    samples), ``_nan`` (NaN NEES among them), ``_mean`` / ``_cov95`` (mean and
    share ≤ χ²₃(0.95) over the finite ones), ``err_h<h>ms_{x,y,z}`` (mean signed
    position error over the samples with a finite error).
    """
    out: dict = {}
    horizons = np.unique(samples["horizon_ns"].to_numpy()) if len(samples) else []
    if not (np.isfinite(lo_ns) and np.isfinite(hi_ns)):
        sel = samples.iloc[0:0]
    else:
        t = samples["time_ns"].to_numpy()
        a = np.searchsorted(t, lo_ns - LAUNCH_TOL_NS, side="left")
        b = np.searchsorted(t, hi_ns, side="right")
        sel = samples.iloc[a:b]
        sel = sel[(sel["time_ns"] + sel["horizon_ns"]).to_numpy() <= hi_ns]
    err_cols = [f"position_error_{a}_m" for a in "xyz"]
    for h in horizons:
        lab = horizon_label(h)
        hs = sel[sel["horizon_ns"] == h]
        nees = hs["position_nees"].to_numpy(float)
        finite = np.isfinite(nees)
        out[f"nees_h{lab}ms_n"] = len(hs)
        out[f"nees_h{lab}ms_nan"] = int((~finite).sum())
        out[f"nees_h{lab}ms_mean"] = float(nees[finite].mean()) if finite.any() else math.nan
        out[f"nees_h{lab}ms_cov95"] = (
            float(np.mean(nees[finite] <= CHI2_3_95)) if finite.any() else math.nan
        )
        for axis, col in zip("xyz", err_cols, strict=True):
            e = hs[col].to_numpy(float) if col in hs.columns else np.array([])
            e = e[np.isfinite(e)]
            out[f"err_h{lab}ms_{axis}"] = float(e.mean()) if e.size else math.nan
    return out


def g8b_horizons(rows: Sequence[Mapping]) -> list[str]:
    labels = set()
    for r in rows:
        for key in r:
            m = _NEES_COL.match(key)
            if m:
                labels.add(m.group(1))
    return sorted(labels, key=float)


def g8b_summary(
    rows: Sequence[Mapping], n_boot: int, seed: int, dof: int = POSITION_DOF
) -> dict | str:
    """G8-B verdict per horizon over ``rows`` (one per trial, :func:`nees_by_trial` columns).

    The pooled mean is exact — ``Σ mean_i·m_i / Σ m_i`` with ``m_i`` the trial's
    finite NEES count — and its 95 % CI resamples whole trials. PASS when the
    CI contains ``dof``; ``NOT_EVALUATED(NaN > 10 %)`` when the bin's NaN share
    exceeds :data:`NEES_NAN_MAX_SHARE`. Coverage and the signed bias are
    reported, not judged (a gravity-only estimator has an expected long-horizon
    bias, L8 §9.1).
    """
    labels = g8b_horizons(rows)
    if not labels:
        return "NOT_EVALUATED(no eval samples bound to a trial)"
    out: dict = {"dof": dof, "chi2_95": CHI2_3_95, "horizons": {}}
    for lab in labels:
        n = np.array([_num(r.get(f"nees_h{lab}ms_n")) for r in rows])
        nan = np.array([_num(r.get(f"nees_h{lab}ms_nan")) for r in rows])
        mean = np.array([_num(r.get(f"nees_h{lab}ms_mean")) for r in rows])
        cov = np.array([_num(r.get(f"nees_h{lab}ms_cov95")) for r in rows])
        err = np.array(
            [[_num(r.get(f"err_h{lab}ms_{a}")) for a in "xyz"] for r in rows], float
        ).reshape(-1, 3)
        keep = np.isfinite(n) & (n > 0)
        n, nan, mean, cov, err = n[keep], nan[keep], mean[keep], cov[keep], err[keep]
        nan = np.where(np.isfinite(nan), nan, 0.0)
        m = n - nan
        s = np.where(m > 0, mean * m, 0.0)
        c = np.where(m > 0, cov * m, 0.0)
        total_n = float(n.sum())
        block: dict = {
            "n_trials": int(keep.sum()),
            "n_samples": int(total_n),
            "n_nan": int(nan.sum()),
            "nan_share": float(nan.sum() / total_n) if total_n else math.nan,
        }
        if total_n == 0:
            block["verdict"] = "NOT_EVALUATED(no sample whose target precedes the first contact)"
            out["horizons"][lab] = block
            continue
        if m.sum() <= 0:
            block["verdict"] = "NOT_EVALUATED(no finite NEES)"
            out["horizons"][lab] = block
            continue
        pooled = float(s.sum() / m.sum())
        rng = np.random.default_rng(seed)
        pick = rng.integers(0, len(m), size=(n_boot, len(m)))
        denom = m[pick].sum(axis=1)
        boot = s[pick].sum(axis=1)[denom > 0] / denom[denom > 0]
        lo, hi = (float(v) for v in np.percentile(boot, [2.5, 97.5]))
        has_err = np.all(np.isfinite(err), axis=1)
        bias = (
            (err[has_err] * n[has_err, None]).sum(axis=0) / n[has_err].sum()
            if has_err.any()
            else np.full(3, math.nan)
        )
        block.update(
            mean_nees=pooled,
            ci95=[lo, hi],
            coverage_95=float(c.sum() / m.sum()),
            bias_m=[float(v) for v in bias],
        )
        if block["nan_share"] > NEES_NAN_MAX_SHARE:
            block["verdict"] = "NOT_EVALUATED(NaN > 10 %)"
        else:
            block["verdict"] = "PASS" if lo <= dof <= hi else "FAIL"
        out["horizons"][lab] = block
    return out


def eval_report_horizons(path: Path) -> dict:
    """The UNRESTRICTED per-horizon numbers of ``sim_capture_evaluate``'s report.

    They include every sample of the capture — post-contact ground truth (the
    ball in the hand, or reset) included — so they are echoed for reference,
    never judged.
    """
    import json  # noqa: PLC0415

    doc = json.loads(Path(path).read_text())
    out = {"note": "unrestricted — includes post-contact ground truth; not a verdict"}
    for h in doc.get("prediction", {}).get("horizons", []):
        out[horizon_label(h["horizon_ns"])] = {
            k: h.get(k)
            for k in (
                "evaluated",
                "excluded_no_ground_truth",
                "position_nees_coverage_95",
                "mean_position_error_m",
                "nees_failures",
            )
        }
    return out


# ══ G8-C2 ═════════════════════════════════════════════════════════════════════


@dataclass
class ProbeDump:
    """``vision_lane_probe --dump`` predictions, one subscription, by snapshot."""

    points: dict  # (snapshot_sequence, generation) → (n, 10) [t_ns, x,y,z, vx,vy,vz, ax,ay,az]
    recv_ns: np.ndarray  # sorted probe receipt steady instants, one per snapshot
    recv_keys: list  # the (sequence, generation) received at each recv_ns
    frame_id: str
    sub: str


def load_probe_dump(path: Path) -> ProbeDump:
    """Read the dump; keep one subscription (both carry the same messages)."""
    import pandas as pd  # noqa: PLC0415

    cols = [
        "recv_ns",
        "sub",
        "stamp_ns",
        "frame_id",
        "snapshot_sequence",
        "generation",
        "horizon_ns",
        "x",
        "y",
        "z",
        "vx",
        "vy",
        "vz",
        "ax",
        "ay",
        "az",
    ]
    df = pd.read_csv(path, usecols=cols)
    subs = list(dict.fromkeys(df["sub"].astype(str)))
    sub = next((s for s in DUMP_SUB_PREFERENCE if s in subs), subs[0] if subs else "")
    df = df[df["sub"].astype(str) == sub]
    frames = set(df["frame_id"].astype(str))
    if len(frames) > 1:
        raise SystemExit(f"{path}: predictions in several frames {sorted(frames)}")
    points = {}
    first_recv = []
    for (seq, gen), g in df.groupby(["snapshot_sequence", "generation"], sort=False):
        g = g.sort_values("horizon_ns")
        t = g["stamp_ns"].to_numpy(np.int64) + g["horizon_ns"].to_numpy(np.int64)
        arr = np.column_stack(
            [t.astype(float)]
            + [g[c].to_numpy(float) for c in ("x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az")]
        )
        key = (int(seq), int(gen))
        points[key] = (arr, t)
        first_recv.append((int(g["recv_ns"].min()), key))
    first_recv.sort()
    return ProbeDump(
        points=points,
        recv_ns=np.array([r for r, _ in first_recv], dtype=np.int64),
        recv_keys=[k for _, k in first_recv],
        frame_id=next(iter(frames)) if frames else "",
        sub=sub,
    )


def p_hat_at(dump: ProbeDump, key, t_ns: float) -> np.ndarray:
    """A snapshot's predicted position at stamp ``t_ns`` (NaN when outside it).

    From the nearest horizon point with its own velocity and acceleration
    (``p + v·dt + ½·a·dt²``, |dt| ≤ half the point spacing) — linear
    interpolation between 50 ms points would err by g·dt²/8 ≈ 3 mm.
    """
    entry = dump.points.get(key)
    if entry is None or not np.isfinite(t_ns):
        return np.full(3, math.nan)
    arr, t_int = entry
    spacing = float(np.median(np.diff(t_int))) if len(t_int) >= 2 else 0.0
    i = int(np.argmin(np.abs(t_int - t_ns)))
    dt = (t_ns - t_int[i]) * 1e-9
    if abs(dt) * 1e9 > max(spacing / 2.0, 1.0) + 1.0:
        return np.full(3, math.nan)
    p, v, a = arr[i, 1:4], arr[i, 4:7], arr[i, 7:10]
    return p + v * dt + 0.5 * a * dt * dt


def plan_point_stamp(dump: ProbeDump, key, p_c_world: np.ndarray) -> tuple[float, float]:
    """The stamp of the planner snapshot's sample that IS ``p_c``, and its distance.

    The planner picks ``p_c`` among the trajectory samples of the snapshot it
    planned on (``planner_events.csv`` ``snapshot_sequence`` /
    ``track_generation``), so one dump point equals it — in the dump's frame,
    once the caller has mapped ``p_c`` there. A distance above
    :data:`PLAN_POINT_TOL_M` is a frame or join error: the stamp is NaN.
    """
    entry = dump.points.get(key)
    if entry is None:
        return math.nan, math.nan
    arr, t_int = entry
    d = np.linalg.norm(arr[:, 1:4] - np.asarray(p_c_world, float), axis=1)
    i = int(np.argmin(d))
    return (float(t_int[i]) if d[i] <= PLAN_POINT_TOL_M else math.nan), float(d[i])


def last_snapshot_before(dump: ProbeDump, steady_ns: float):
    """The key of the last snapshot the probe received at or before ``steady_ns``."""
    if not np.isfinite(steady_ns) or dump.recv_ns.size == 0:
        return None
    i = int(np.searchsorted(dump.recv_ns, steady_ns, side="right")) - 1
    return dump.recv_keys[i] if i >= 0 else None


def c2_values(p_true: np.ndarray, p_live: np.ndarray, p_c: np.ndarray) -> dict:
    a = np.asarray(p_true, float) - np.asarray(p_live, float)
    b = np.asarray(p_live, float) - np.asarray(p_c, float)
    out = {f"c2_A_{ax}": float(a[i]) for i, ax in enumerate("xyz")}
    out.update({f"c2_B_{ax}": float(b[i]) for i, ax in enumerate("xyz")})
    return out


def c2_arrays(rows: Sequence[Mapping]) -> tuple[np.ndarray, np.ndarray, list[int]]:
    a = np.array([[_num(r.get(f"c2_A_{x}")) for x in "xyz"] for r in rows], float).reshape(-1, 3)
    b = np.array([[_num(r.get(f"c2_B_{x}")) for x in "xyz"] for r in rows], float).reshape(-1, 3)
    ok = np.all(np.isfinite(a), axis=1) & np.all(np.isfinite(b), axis=1)
    return a[ok], b[ok], [i for i, flag in enumerate(ok) if flag]


def c2_summary(
    rows: Sequence[Mapping],
    n_boot: int,
    seed: int,
    clusters: Sequence | None = None,
    n_min: int = C2_N_MIN,
) -> dict | str:
    """G8-C2 over ``rows`` (their ``c2_A_*`` / ``c2_B_*`` columns, metres).

    The orthogonality identity ``E|A+B|² = E|A|² + E|B|² + 2E[A·B]`` is reported
    with its cross term, and :func:`catching_trials.independence_test` (whitened
    cross-covariance, cluster bootstrap) judges A⊥B — below ``n_min`` trials it is
    ``NOT_EVALUATED``.
    """
    a, b, used = c2_arrays(rows)
    joins = [str(rows[i].get("c2_join")) for i in used]
    if len(a) == 0:
        return "NOT_EVALUATED(no trial with A and B)"
    ab = a + b
    out = {
        "n": len(a),
        "n_exact": joins.count("exact"),
        "n_approx": joins.count("approx"),
        "A_norm_mm_median": float(np.median(np.linalg.norm(a, axis=1)) * 1e3),
        "B_norm_mm_median": float(np.median(np.linalg.norm(b, axis=1)) * 1e3),
        "E_A_plus_B_sq_mm2": float(np.mean(np.sum(ab * ab, axis=1)) * 1e6),
        "E_A_sq_plus_E_B_sq_mm2": float(
            (np.mean(np.sum(a * a, axis=1)) + np.mean(np.sum(b * b, axis=1))) * 1e6
        ),
        "two_E_A_dot_B_mm2": float(2.0 * np.mean(np.sum(a * b, axis=1)) * 1e6),
        "frame": "sim world (the probe dump's `world`)",
    }
    if len(a) < n_min:
        out["independence"] = f"NOT_EVALUATED(n < {n_min})"
        return out
    from rtc_tools.analysis.catching_trials import independence_test  # noqa: PLC0415

    labels = None if clusters is None else [clusters[i] for i in used]
    try:
        res = independence_test(a, b, labels, n_boot=n_boot, seed=seed)
    except ValueError as exc:  # singular covariance (e.g. B identically 0)
        out["independence"] = f"NOT_EVALUATED({exc})"
        return out
    out["independence"] = {
        "passed": res.passed,
        "cross": res.cross,
        "ci_low": res.ci_low,
        "ci_high": res.ci_high,
        "alpha": res.alpha,
        "n_clusters": res.n_clusters,
    }
    return out
