"""Hand-near throw analysis (dynamic_catching S8-F, #537).

The S8-F question — how fast a ball passing within 0.2 m of the waiting hand
can still be caught — has two parts the planner answers separately: did it
find a plan it could reach in time (COMMIT), and given a plan, did the hand
hold the ball (CATCH). The two are fitted separately and multiplied; a single
success rate would fold "never reached" into "reached and dropped".

Inputs are one or more UNITS, each the output of one launch::

    <unit>/trials/trial_results.json   catching_sim_trials --dist hand_*  (factors, aim error)
    <unit>/trials/run_meta.json        the arm label and the hand geometry
    <unit>/ct/catching_trials.csv      catching_trials (truth success, commit, contact v_rel)
    <planner_events.csv>               optional, the controller session's planner log

Per trial the factors are the throw's design values (speed v, flight time T,
offset r, offset angle ψ, incidence offset α) and the outcomes are ``committed``
(the controller froze a plan) and ``truth_success`` (the ground-truth verdict
of catching_trials). Fits are logistic GLMs by IRLS on numpy alone (no
statsmodels on the workspace), with trial-bootstrap confidence intervals.

Robot-agnostic: nothing here names a robot, a joint or a frame — the geometry
is read from ``run_meta.json`` and reported, not assumed.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

TOOL = "catching_hand_near"
FACTORS = ("speed_m_s", "flight_time_s", "offset_m", "offset_angle_deg", "incidence_offset_deg")
# The design centre the conditional v50(r) is quoted at (#537 S8-F-1): mid
# flight time, face-on. ψ enters as (sin, cos) and is marginalised at 0.
CENTRE_FLIGHT_TIME_S = 0.7
CENTRE_INCIDENCE_OFFSET_DEG = 0.0
V50_OFFSETS_M = (0.0, 0.05, 0.10, 0.15, 0.20)
V_SEARCH_M_S = (0.0, 20.0)
DEFAULT_N_BOOT = 500
DEFAULT_SEED = 0
DEFAULT_AIM_TOL_MM = 2.0
Z_95 = 1.959963984540054


def _num(value) -> float:
    """A CSV / JSON cell as a float (empty, missing or non-numeric → NaN)."""
    if value is None:
        return math.nan
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def _is_true(value) -> bool:
    """A CSV / JSON truth cell: ``True`` or the strings ``true`` / ``1`` (a NaN or empty cell is not a success)."""
    if isinstance(value, bool | np.bool_):
        return bool(value)
    return str(value).strip().lower() in ("true", "1")


def _int0(value) -> int:
    """A count cell as an int (empty / NaN → 0)."""
    n = _num(value)
    return int(n) if math.isfinite(n) else 0


def _read_csv_rows(path: Path) -> list[dict]:
    """All rows of a CSV, plain or ``.gz`` beside it; SystemExit when neither exists."""
    if path.is_file():
        with path.open(newline="") as f:
            return list(csv.DictReader(f))
    alt = path.with_name(path.name + ".gz")
    if alt.is_file():
        import gzip  # noqa: PLC0415

        with gzip.open(alt, "rt", newline="") as f:
            return list(csv.DictReader(f))
    raise SystemExit(f"missing {path}")


# ── Statistics ────────────────────────────────────────────────────────────────


def _sigmoid(x: float) -> float:
    return 1.0 / (1.0 + math.exp(-max(-500.0, min(500.0, x))))


def wilson(k: int, n: int, z: float = Z_95) -> tuple[float, float, float]:
    """(p̂, lower, upper) — Wilson score interval, two-sided ``z``. NaN for n 0."""
    if n <= 0:
        return math.nan, math.nan, math.nan
    p = k / n
    denom = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return p, max(0.0, centre - half), min(1.0, centre + half)


def fit_logistic(
    x: np.ndarray, y: np.ndarray, *, ridge: float = 1e-4, max_iter: int = 100
) -> np.ndarray | None:
    """IRLS logistic regression, ``y`` in {0, 1}; ``x`` (n, k) with its own intercept column.

    A small ridge on every coefficient but the first keeps a separable design
    (all successes below some speed, none above — the cliff the experiment is
    looking for) from diverging; it biases the slope towards 0 by a negligible
    amount at the sample sizes here. Returns ``None`` when the iteration does
    not converge or the design is degenerate (one class only).
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    n, k = x.shape
    if n < k or y.min() == y.max():
        return None
    penalty = np.full(k, ridge)
    penalty[0] = 0.0
    beta = np.zeros(k)
    for _ in range(max_iter):
        eta = x @ beta
        p = 1.0 / (1.0 + np.exp(-np.clip(eta, -35.0, 35.0)))
        w = p * (1.0 - p)
        grad = x.T @ (y - p) - penalty * beta
        hess = (x * w[:, None]).T @ x + np.diag(penalty) + 1e-12 * np.eye(k)
        try:
            step = np.linalg.solve(hess, grad)
        except np.linalg.LinAlgError:
            return None
        beta = beta + step
        if not np.all(np.isfinite(beta)):
            return None
        if np.max(np.abs(step)) < 1e-8:
            return beta
    return None


def design_matrix(v, r, flight_time_s, alpha_deg, psi_deg) -> np.ndarray:
    """[1, v, r, r², T − T_centre, α − α_centre, v·r, sin ψ, cos ψ]."""
    v = np.asarray(v, dtype=float)
    r = np.asarray(r, dtype=float)
    t = np.asarray(flight_time_s, dtype=float) - CENTRE_FLIGHT_TIME_S
    a = np.asarray(alpha_deg, dtype=float) - CENTRE_INCIDENCE_OFFSET_DEG
    psi = np.radians(np.asarray(psi_deg, dtype=float))
    return np.column_stack([np.ones_like(v), v, r, r * r, t, a, v * r, np.sin(psi), np.cos(psi)])


DESIGN_COLUMNS = ("intercept", "v", "r", "r2", "dT", "alpha", "v_r", "sin_psi", "cos_psi")


def logit_at(beta: np.ndarray, v: float, r: float) -> float:
    """The fitted logit at speed ``v`` and offset ``r``, other factors at the design centre."""
    return float(
        beta
        @ design_matrix([v], [r], [CENTRE_FLIGHT_TIME_S], [CENTRE_INCIDENCE_OFFSET_DEG], [0.0])[0]
        - beta[8]
    )


def v50_of(beta: np.ndarray | None, r: float) -> float:
    """Speed where the fitted probability crosses 0.5 at offset ``r`` (design centre), NaN if none in range."""
    if beta is None:
        return math.nan
    # Linear in v at fixed r: β0 + β_v v + β_r r + β_rr r² + β_vr v r = 0.
    slope = beta[1] + beta[6] * r
    if abs(slope) < 1e-12:
        return math.nan
    v = -(beta[0] + beta[2] * r + beta[3] * r * r) / slope
    return float(v) if V_SEARCH_M_S[0] <= v <= V_SEARCH_M_S[1] else math.nan


def product_v50(beta_plan: np.ndarray | None, beta_catch: np.ndarray | None, r: float) -> float:
    """Speed where P(plan)·P(catch | plan) crosses 0.5 (bisection), NaN if it never does."""
    if beta_plan is None or beta_catch is None:
        return math.nan

    def prob(v: float) -> float:
        return _sigmoid(logit_at(beta_plan, v, r)) * _sigmoid(logit_at(beta_catch, v, r))

    lo, hi = V_SEARCH_M_S
    if (prob(lo) - 0.5) * (prob(hi) - 0.5) > 0:
        return math.nan
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if (prob(lo) - 0.5) * (prob(mid) - 0.5) <= 0:
            hi = mid
        else:
            lo = mid
    return 0.5 * (lo + hi)


def mcnemar_exact(b: int, c: int) -> float:
    """Two-sided exact McNemar p-value from the discordant counts (b: A only, c: B only)."""
    from scipy.stats import binomtest  # noqa: PLC0415

    n = b + c
    if n == 0:
        return 1.0
    return float(binomtest(min(b, c), n, 0.5, alternative="two-sided").pvalue)


def bootstrap_ci(values: Sequence[float]) -> tuple[float, float, int]:
    """(2.5 %, 97.5 %, n finite) of bootstrap replicates, NaN when fewer than 20 are finite."""
    finite = np.asarray([v for v in values if math.isfinite(v)], dtype=float)
    if finite.size < 20:
        return math.nan, math.nan, int(finite.size)
    return float(np.percentile(finite, 2.5)), float(np.percentile(finite, 97.5)), int(finite.size)


# ── Loading ───────────────────────────────────────────────────────────────────


@dataclass
class Trial:
    unit: str
    arm: str
    kind: str
    idx: int
    seed: int
    sample_idx: int
    factors: dict[str, float]
    derived: dict[str, float]
    accepted: bool
    invalid_reason: str
    committed: bool
    truth_success: bool
    first_plan_s: float
    plan_valid_ratio: float
    contact_v_rel: float
    gamma_f_planned: float
    ball_speed_tc: float
    d_min_mm: float
    rtf_trial_min: float
    aim_error_mm: float
    aim_pass_speed_m_s: float

    @property
    def valid(self) -> bool:
        return self.accepted and not self.invalid_reason

    def row(self) -> dict:
        return {
            "unit": self.unit,
            "arm": self.arm,
            "kind": self.kind,
            "idx": self.idx,
            "seed": self.seed,
            "sample_idx": self.sample_idx,
            **self.factors,
            **self.derived,
            "accepted": self.accepted,
            "invalid_reason": self.invalid_reason,
            "committed": self.committed,
            "truth_success": self.truth_success,
            "first_plan_s": self.first_plan_s,
            "plan_valid_ratio": self.plan_valid_ratio,
            "contact_v_rel": self.contact_v_rel,
            "gamma_f_planned": self.gamma_f_planned,
            "ball_speed_tc": self.ball_speed_tc,
            "d_min_mm": self.d_min_mm,
            "rtf_trial_min": self.rtf_trial_min,
            "aim_error_mm": self.aim_error_mm,
            "aim_pass_speed_m_s": self.aim_pass_speed_m_s,
        }


@dataclass
class Unit:
    path: Path
    arm: str
    geometry: dict | None
    trials: list[Trial]
    planner: dict | None = None  # rank-gate summary of the session's planner_events


DERIVED = (
    "incidence_deg",
    "release_height_offset_m",
    "horizontal_distance_m",
    "release_speed_m_s",
    "release_elevation_deg",
    "apex_z_m",
)


def load_unit(unit_dir: Path, planner_events: Path | None = None, *, ct_dir: str = "ct") -> Unit:
    """Join a unit's trial records with catching_trials' per-trial rows on ``idx``."""
    unit_dir = Path(unit_dir)
    trials_dir = unit_dir / "trials"
    records = json.loads((trials_dir / "trial_results.json").read_text())
    meta_path = trials_dir / "run_meta.json"
    meta = json.loads(meta_path.read_text()) if meta_path.is_file() else {}
    arm = str(meta.get("arm") or unit_dir.name)
    ct_path = unit_dir / ct_dir / "catching_trials.csv"
    if not ct_path.is_file():
        raise SystemExit(f"{unit_dir}: missing {ct_path} — run catching_trials on the unit first")
    with ct_path.open() as f:
        ct_rows = {int(_num(row["idx"])): row for row in csv.DictReader(f)}
    trials = []
    for rec in records:
        if rec.get("target_m") is None:
            raise SystemExit(
                f"{unit_dir}: trial {rec.get('idx')} is a {rec.get('kind')!r} throw, not a hand-near one "
                "(no target_m) — this tool reads --dist hand_* units only"
            )
        idx = int(rec["idx"])
        ct = ct_rows.get(idx)
        if ct is None:
            raise SystemExit(f"{unit_dir}: trial {idx} has no catching_trials row")
        aim_error = rec.get("aim_error_m")
        trials.append(
            Trial(
                unit=str(unit_dir),
                arm=arm,
                kind=str(rec["kind"]),
                idx=idx,
                seed=int(rec.get("seed", -1)),
                sample_idx=int(rec.get("sample_idx", idx)),
                factors={k: float(rec[k]) for k in FACTORS},
                derived={k: float(rec[k]) for k in DERIVED if k in rec},
                accepted=bool(rec.get("accepted", False)),
                invalid_reason=str(ct.get("invalid_reason") or ""),
                committed=math.isfinite(_num(ct.get("t_commit"))),
                truth_success=_is_true(ct.get("truth_success")),
                first_plan_s=_num(ct.get("first_plan_s")),
                plan_valid_ratio=_num(ct.get("plan_valid_ratio")),
                contact_v_rel=_num(ct.get("contact_v_rel")),
                gamma_f_planned=_num(ct.get("gamma_f_planned")),
                ball_speed_tc=_num(ct.get("ball_speed_tc")),
                d_min_mm=_num(ct.get("d_min_mm")),
                rtf_trial_min=_num(ct.get("rtf_trial_min")),
                aim_error_mm=1e3 * float(aim_error) if aim_error is not None else math.nan,
                aim_pass_speed_m_s=_num(rec.get("aim_pass_speed_m_s")),
            )
        )
    planner = planner_events_summary(planner_events) if planner_events else None
    return Unit(
        path=unit_dir, arm=arm, geometry=meta.get("hand_geometry"), trials=trials, planner=planner
    )


def planner_events_summary(path: Path) -> dict:
    """Rank-gate bits of the PUBLISHED plans and the judgement rejects, over a whole session.

    ``rej_workspace`` is the sensor that the widened catch box took effect: a
    session whose box was the shipped one rejects hand-near candidates there.
    """
    path = Path(path)
    rows_read = _read_csv_rows(path)
    published = 0
    reach = gamma = 0
    rejects = dict.fromkeys(
        ("rej_input", "rej_ik", "rej_manipulability", "rej_workspace", "rej_not_evaluated"), 0
    )
    rows = 0
    for row in rows_read:
        rows += 1
        for k in rejects:
            rejects[k] += _int0(row.get(k))
        if _is_true(row.get("plan_valid")):
            published += 1
            reach += int(_is_true(row.get("rank_reach")))
            gamma += int(_is_true(row.get("rank_gamma")))
    return {
        "path": str(path),
        "rows": rows,
        "plans_published": published,
        "rank_reach_rate": reach / published if published else math.nan,
        "rank_gamma_rate": gamma / published if published else math.nan,
        "judge_rejects": rejects,
    }


# ── Analyses ──────────────────────────────────────────────────────────────────


def aim_check(trials: Iterable[Trial], tol_mm: float) -> dict:
    errs = [t.aim_error_mm for t in trials if math.isfinite(t.aim_error_mm)]
    over = [t.idx for t in trials if math.isfinite(t.aim_error_mm) and t.aim_error_mm > tol_mm]
    return {
        "tol_mm": tol_mm,
        "n": len(errs),
        "max_mm": max(errs) if errs else math.nan,
        "p50_mm": float(np.median(errs)) if errs else math.nan,
        "over_tol_idx": over,
        "pass": bool(errs) and not over,
    }


def speed_table(trials: Sequence[Trial]) -> list[dict]:
    """Per-speed counts with Wilson intervals — the model-free view of a grid."""
    speeds = sorted({round(t.factors["speed_m_s"], 6) for t in trials})
    out = []
    for v in speeds:
        rows = [t for t in trials if round(t.factors["speed_m_s"], 6) == v and t.valid]
        n = len(rows)
        k_plan = sum(t.committed for t in rows)
        k_catch = sum(t.truth_success for t in rows)
        p, lo, hi = wilson(k_catch, n)
        pp, plo, phi = wilson(k_plan, n)
        pc, clo, chi = wilson(k_catch, k_plan)
        out.append(
            {
                "speed_m_s": v,
                "n": n,
                "committed": k_plan,
                "caught": k_catch,
                "p_catch": p,
                "p_catch_lo": lo,
                "p_catch_hi": hi,
                "p_plan": pp,
                "p_plan_lo": plo,
                "p_plan_hi": phi,
                "p_catch_given_plan": pc,
                "p_catch_given_plan_lo": clo,
                "p_catch_given_plan_hi": chi,
                "contact_v_rel_p50": float(
                    np.median([t.contact_v_rel for t in rows if math.isfinite(t.contact_v_rel)])
                )
                if any(math.isfinite(t.contact_v_rel) for t in rows)
                else math.nan,
            }
        )
    return out


def _xy(trials: Sequence[Trial], outcome: str) -> tuple[np.ndarray, np.ndarray]:
    f = [t.factors for t in trials]
    x = design_matrix(
        [d["speed_m_s"] for d in f],
        [d["offset_m"] for d in f],
        [d["flight_time_s"] for d in f],
        [d["incidence_offset_deg"] for d in f],
        [d["offset_angle_deg"] for d in f],
    )
    y = np.array([float(getattr(t, outcome)) for t in trials])
    return x, y


@dataclass
class StageFit:
    outcome: str
    n: int
    events: int
    beta: list[float] | None
    v50: dict[str, float] = field(default_factory=dict)  # by offset, as strings "0.05"
    v50_ci: dict[str, list] = field(default_factory=dict)

    def as_dict(self) -> dict:
        return {
            "outcome": self.outcome,
            "n": self.n,
            "events": self.events,
            "beta": None
            if self.beta is None
            else dict(zip(DESIGN_COLUMNS, self.beta, strict=True)),
            "v50_by_offset_m": self.v50,
            "v50_ci95_by_offset_m": self.v50_ci,
        }


def two_stage(
    trials: Sequence[Trial], *, n_boot: int, seed: int, offsets: Sequence[float] = V50_OFFSETS_M
) -> dict:
    """P(plan), P(catch | plan) and their product as logistic GLMs on the design factors.

    v50(r) for each is the speed where the fitted probability crosses 0.5 at
    the design centre (T 0.7 s, face-on, ψ marginal); the CI is a trial
    bootstrap (refit on resampled trials, percentiles of the v50 replicates).
    """
    valid = [t for t in trials if t.valid]
    committed = [t for t in valid if t.committed]
    rng = np.random.default_rng(seed)
    keys = [f"{r:.2f}" for r in offsets]

    def fit_all(
        sample: Sequence[Trial],
    ) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None]:
        if len(sample) < 12:
            return None, None, None
        xp, yp = _xy(sample, "committed")
        beta_plan = fit_logistic(xp, yp)
        com = [t for t in sample if t.committed]
        beta_catch = None
        if len(com) >= 12:
            xc, yc = _xy(com, "truth_success")
            beta_catch = fit_logistic(xc, yc)
        xa, ya = _xy(sample, "truth_success")
        beta_all = fit_logistic(xa, ya)
        return beta_plan, beta_catch, beta_all

    beta_plan, beta_catch, beta_all = fit_all(valid)
    plan = StageFit(
        "committed",
        len(valid),
        sum(t.committed for t in valid),
        None if beta_plan is None else beta_plan.tolist(),
    )
    catch = StageFit(
        "truth_success | committed",
        len(committed),
        sum(t.truth_success for t in committed),
        None if beta_catch is None else beta_catch.tolist(),
    )
    overall = StageFit(
        "truth_success",
        len(valid),
        sum(t.truth_success for t in valid),
        None if beta_all is None else beta_all.tolist(),
    )
    product: dict[str, float] = {}
    for key, r in zip(keys, offsets, strict=True):
        plan.v50[key] = v50_of(beta_plan, r)
        catch.v50[key] = v50_of(beta_catch, r)
        overall.v50[key] = v50_of(beta_all, r)
        product[key] = product_v50(beta_plan, beta_catch, r)
    reps: dict[str, dict[str, list[float]]] = {
        s: {k: [] for k in keys} for s in ("plan", "catch", "overall", "product")
    }
    if valid:
        for _ in range(n_boot):
            pick = rng.integers(0, len(valid), len(valid))
            sample = [valid[i] for i in pick]
            bp, bc, ba = fit_all(sample)
            for key, r in zip(keys, offsets, strict=True):
                reps["plan"][key].append(v50_of(bp, r))
                reps["catch"][key].append(v50_of(bc, r))
                reps["overall"][key].append(v50_of(ba, r))
                reps["product"][key].append(product_v50(bp, bc, r))
    for stage, fit in (("plan", plan), ("catch", catch), ("overall", overall)):
        for key in keys:
            fit.v50_ci[key] = list(bootstrap_ci(reps[stage][key]))
    return {
        "n_valid": len(valid),
        "n_committed": len(committed),
        "n_caught": sum(t.truth_success for t in valid),
        "plan": plan.as_dict(),
        "catch_given_plan": catch.as_dict(),
        "overall": overall.as_dict(),
        "product_v50_by_offset_m": product,
        "product_v50_ci95_by_offset_m": {k: list(bootstrap_ci(reps["product"][k])) for k in keys},
        "n_boot": n_boot,
        "design_centre": {
            "flight_time_s": CENTRE_FLIGHT_TIME_S,
            "incidence_offset_deg": CENTRE_INCIDENCE_OFFSET_DEG,
        },
    }


def v_rel_fit(trials: Sequence[Trial], *, n_boot: int, seed: int) -> dict:
    """P(catch | committed) against the MEASURED contact relative speed — the fly-in tolerance in closed loop."""
    rows = [t for t in trials if t.valid and t.committed and math.isfinite(t.contact_v_rel)]
    if len(rows) < 12:
        return {
            "n": len(rows),
            "beta": None,
            "v_rel50": math.nan,
            "v_rel50_ci95": [math.nan, math.nan, 0],
        }
    rng = np.random.default_rng(seed)

    def fit(sample):
        x = np.column_stack([np.ones(len(sample)), [t.contact_v_rel for t in sample]])
        y = np.array([float(t.truth_success) for t in sample])
        beta = fit_logistic(x, y)
        if beta is None or abs(beta[1]) < 1e-12:
            return None, math.nan
        v = -beta[0] / beta[1]
        return beta, float(v) if 0.0 <= v <= 20.0 else math.nan

    beta, v50 = fit(rows)
    reps = []
    for _ in range(n_boot):
        pick = rng.integers(0, len(rows), len(rows))
        reps.append(fit([rows[i] for i in pick])[1])
    return {
        "n": len(rows),
        "beta": None if beta is None else {"intercept": float(beta[0]), "v_rel": float(beta[1])},
        "v_rel50": v50,
        "v_rel50_ci95": list(bootstrap_ci(reps)),
        "contact_v_rel_p50": float(np.median([t.contact_v_rel for t in rows])),
    }


def wilson_cells(
    trials: Sequence[Trial],
    r_edges: Sequence[float] = (0.0, 0.05, 0.10, 0.15, 0.2001),
    v_edges: Sequence[float] = (3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 7.001),
) -> list[dict]:
    """(r bin × v bin) counts with Wilson intervals — the model-free check of the v50(r) map."""
    out = []
    valid = [t for t in trials if t.valid]
    for i in range(len(r_edges) - 1):
        for j in range(len(v_edges) - 1):
            cell = [
                t
                for t in valid
                if r_edges[i] <= t.factors["offset_m"] < r_edges[i + 1]
                and v_edges[j] <= t.factors["speed_m_s"] < v_edges[j + 1]
            ]
            n = len(cell)
            k_plan = sum(t.committed for t in cell)
            k = sum(t.truth_success for t in cell)
            p, lo, hi = wilson(k, n)
            pp, plo, phi = wilson(k_plan, n)
            out.append(
                {
                    "r_lo": r_edges[i],
                    "r_hi": min(r_edges[i + 1], 0.2),
                    "v_lo": v_edges[j],
                    "v_hi": min(v_edges[j + 1], 7.0),
                    "n": n,
                    "committed": k_plan,
                    "caught": k,
                    "p_catch": p,
                    "p_catch_lo": lo,
                    "p_catch_hi": hi,
                    "p_plan": pp,
                    "p_plan_lo": plo,
                    "p_plan_hi": phi,
                }
            )
    return out


def paired_arms(a: Sequence[Trial], b: Sequence[Trial]) -> dict:
    """A/B on the same throws (paired by (seed, sample_idx)): McNemar on catch and on commit."""
    key = lambda t: (t.seed, t.sample_idx)  # noqa: E731
    ma = {key(t): t for t in a if t.valid}
    mb = {key(t): t for t in b if t.valid}
    common = sorted(set(ma) & set(mb))
    if not common:
        return {"pairs": 0}

    def table(attr: str) -> dict:
        both = only_a = only_b = neither = 0
        for k in common:
            xa, xb = getattr(ma[k], attr), getattr(mb[k], attr)
            both += xa and xb
            only_a += xa and not xb
            only_b += xb and not xa
            neither += (not xa) and (not xb)
        return {
            "both": both,
            "a_only": only_a,
            "b_only": only_b,
            "neither": neither,
            "p_a": (both + only_a) / len(common),
            "p_b": (both + only_b) / len(common),
            "mcnemar_p": mcnemar_exact(only_a, only_b),
        }

    return {
        "pairs": len(common),
        "unpaired_a": len(ma) - len(common),
        "unpaired_b": len(mb) - len(common),
        "truth_success": table("truth_success"),
        "committed": table("committed"),
    }


# ── Driver ────────────────────────────────────────────────────────────────────


def analyse(
    units: Sequence[Unit], *, n_boot: int, seed: int, aim_tol_mm: float, ab: tuple[str, str] | None
) -> dict:
    trials = [t for u in units for t in u.trials]
    by_kind: dict[str, list[Trial]] = {}
    for t in trials:
        by_kind.setdefault(t.kind, []).append(t)
    by_arm: dict[str, list[Trial]] = {}
    for t in trials:
        by_arm.setdefault(t.arm, []).append(t)
    summary: dict = {
        "tool": TOOL,
        "units": [
            {
                "path": str(u.path),
                "arm": u.arm,
                "n_trials": len(u.trials),
                "n_valid": sum(t.valid for t in u.trials),
                "kinds": sorted({t.kind for t in u.trials}),
                "hand_geometry": u.geometry,
                "planner_events": u.planner,
            }
            for u in units
        ],
        "aim": aim_check(trials, aim_tol_mm),
        "arms": {},
    }
    for arm, rows in sorted(by_arm.items()):
        grids = {k: speed_table(v) for k, v in _group(rows, "kind").items() if k != "hand_lhs"}
        lhs = [t for t in rows if t.kind == "hand_lhs"]
        arm_out: dict = {
            "n_valid": sum(t.valid for t in rows),
            "n_committed": sum(t.committed for t in rows if t.valid),
            "n_caught": sum(t.truth_success for t in rows if t.valid),
            "grids": grids,
            "grid_v50": {},
        }
        for kind, grid_rows in _group(rows, "kind").items():
            if kind == "hand_lhs":
                continue
            arm_out["grid_v50"][kind] = grid_v50(
                [t for t in grid_rows if t.valid], n_boot=n_boot, seed=seed
            )
        if lhs:
            arm_out["lhs"] = two_stage(lhs, n_boot=n_boot, seed=seed)
            arm_out["lhs_wilson_cells"] = wilson_cells(lhs)
        arm_out["v_rel"] = v_rel_fit(rows, n_boot=n_boot, seed=seed)
        summary["arms"][arm] = arm_out
    if ab is not None:
        a, b = ab
        if a not in by_arm or b not in by_arm:
            raise SystemExit(f"--ab names arms {ab} but the units carry {sorted(by_arm)}")
        summary["ab"] = {"a": a, "b": b, **paired_arms(by_arm[a], by_arm[b])}
    return summary


def grid_v50(trials: Sequence[Trial], *, n_boot: int, seed: int) -> dict:
    """Logistic in speed alone on a grid arm: v50 for catch, plan and catch | plan, with bootstrap CIs."""
    rng = np.random.default_rng(seed)

    def fit(sample: Sequence[Trial], outcome: str) -> float:
        if len(sample) < 8:
            return math.nan
        x = np.column_stack([np.ones(len(sample)), [t.factors["speed_m_s"] for t in sample]])
        y = np.array([float(getattr(t, outcome)) for t in sample])
        beta = fit_logistic(x, y)
        if beta is None or abs(beta[1]) < 1e-12:
            return math.nan
        v = -beta[0] / beta[1]
        return float(v) if V_SEARCH_M_S[0] <= v <= V_SEARCH_M_S[1] else math.nan

    out = {}
    for name, rows, outcome in (
        ("catch", trials, "truth_success"),
        ("plan", trials, "committed"),
        ("catch_given_plan", [t for t in trials if t.committed], "truth_success"),
    ):
        point = fit(rows, outcome)
        reps = []
        for _ in range(n_boot):
            if not rows:
                break
            pick = rng.integers(0, len(rows), len(rows))
            reps.append(fit([rows[i] for i in pick], outcome))
        out[name] = {"n": len(rows), "v50": point, "ci95": list(bootstrap_ci(reps))}
    return out


def _group(trials: Sequence[Trial], attr: str) -> dict[str, list[Trial]]:
    out: dict[str, list[Trial]] = {}
    for t in trials:
        out.setdefault(getattr(t, attr), []).append(t)
    return dict(sorted(out.items()))


def write_outputs(summary: dict, units: Sequence[Unit], out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = [t.row() for u in units for t in u.trials]
    if rows:
        with (out_dir / "hand_near_trials.csv").open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
    (out_dir / "hand_near_summary.json").write_text(
        json.dumps(summary, indent=2, default=_json_default)
    )
    v50_rows = []
    for arm, a in summary["arms"].items():
        lhs = a.get("lhs")
        if not lhs:
            continue
        for key in lhs["plan"]["v50_by_offset_m"]:
            v50_rows.append(
                {
                    "arm": arm,
                    "offset_m": float(key),
                    "v50_plan": lhs["plan"]["v50_by_offset_m"][key],
                    "v50_plan_lo": lhs["plan"]["v50_ci95_by_offset_m"][key][0],
                    "v50_plan_hi": lhs["plan"]["v50_ci95_by_offset_m"][key][1],
                    "v50_catch_given_plan": lhs["catch_given_plan"]["v50_by_offset_m"][key],
                    "v50_catch_given_plan_lo": lhs["catch_given_plan"]["v50_ci95_by_offset_m"][
                        key
                    ][0],
                    "v50_catch_given_plan_hi": lhs["catch_given_plan"]["v50_ci95_by_offset_m"][
                        key
                    ][1],
                    "v50_overall": lhs["overall"]["v50_by_offset_m"][key],
                    "v50_overall_lo": lhs["overall"]["v50_ci95_by_offset_m"][key][0],
                    "v50_overall_hi": lhs["overall"]["v50_ci95_by_offset_m"][key][1],
                    "v50_product": lhs["product_v50_by_offset_m"][key],
                    "v50_product_lo": lhs["product_v50_ci95_by_offset_m"][key][0],
                    "v50_product_hi": lhs["product_v50_ci95_by_offset_m"][key][1],
                }
            )
    if v50_rows:
        with (out_dir / "v50_map.csv").open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(v50_rows[0]))
            w.writeheader()
            w.writerows(v50_rows)
    cell_rows = [
        {"arm": arm, **c}
        for arm, a in summary["arms"].items()
        for c in a.get("lhs_wilson_cells", [])
    ]
    if cell_rows:
        with (out_dir / "wilson_cells.csv").open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(cell_rows[0]))
            w.writeheader()
            w.writerows(cell_rows)


def _json_default(o):
    if isinstance(o, np.integer | np.bool_):
        return o.item()
    if isinstance(o, np.floating):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, Path):
        return str(o)
    raise TypeError(f"not JSON serialisable: {type(o)}")


def parse_unit_arg(value: str) -> tuple[Path, Path | None]:
    """``<unit dir>[:<planner_events.csv>]``."""
    if ":" in value:
        unit, events = value.split(":", 1)
        return Path(unit), Path(events)
    return Path(value), None


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument(
        "units",
        nargs="+",
        help="unit dirs (<unit>/trials + <unit>/ct), optionally <unit>:<planner_events.csv>",
    )
    ap.add_argument("--out", type=Path, required=True, help="output dir")
    ap.add_argument("--n-boot", type=int, default=DEFAULT_N_BOOT)
    ap.add_argument("--seed", type=int, default=DEFAULT_SEED)
    ap.add_argument(
        "--aim-tol-mm",
        type=float,
        default=DEFAULT_AIM_TOL_MM,
        help="aim error every trial must stay under",
    )
    ap.add_argument(
        "--ab", nargs=2, metavar=("ARM_A", "ARM_B"), help="paired McNemar between two arm labels"
    )
    args = ap.parse_args(argv)
    units = [load_unit(unit, events) for unit, events in map(parse_unit_arg, args.units)]
    summary = analyse(
        units,
        n_boot=args.n_boot,
        seed=args.seed,
        aim_tol_mm=args.aim_tol_mm,
        ab=tuple(args.ab) if args.ab else None,
    )
    write_outputs(summary, units, args.out)
    aim = summary["aim"]
    print(f"{TOOL}: {sum(len(u.trials) for u in units)} trials in {len(units)} units → {args.out}")
    print(
        f"  aim: max {aim['max_mm']:.3f} mm (tol {aim['tol_mm']}), over: {aim['over_tol_idx'] or 'none'}"
    )
    for arm, a in summary["arms"].items():
        print(f"  {arm}: valid {a['n_valid']} committed {a['n_committed']} caught {a['n_caught']}")
        for kind, g in a["grid_v50"].items():
            print(f"    {kind}: v50 catch {g['catch']['v50']:.2f} plan {g['plan']['v50']:.2f}")
        if "lhs" in a:
            p = a["lhs"]["product_v50_by_offset_m"]
            print("    lhs v50(product): " + ", ".join(f"r {k}: {v:.2f}" for k, v in p.items()))
    if "ab" in summary and summary["ab"].get("pairs"):
        ab = summary["ab"]
        print(
            f"  A/B {ab['a']} vs {ab['b']}: pairs {ab['pairs']}, catch {ab['truth_success']['p_a']:.3f} vs "
            f"{ab['truth_success']['p_b']:.3f} (McNemar p {ab['truth_success']['mcnemar_p']:.3g})"
        )
    return 0 if aim["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
