"""One-sided two-arm permutation tests: a median difference and a restricted mean survival time.

The companion of :mod:`binomial_stats` for numbers that are not counts. A run
of a closed-loop batch survives some seconds after its first step (censored
when it reaches the end of its script) and has a tracking ratio; two arms of
a few dozen runs are compared without a distribution assumed:

* :func:`km_rmst` — the restricted mean survival time ∫₀^τ S(t) dt of the
  Kaplan–Meier curve (events before censorings at a tie), for many member
  sets at once.
* :func:`permutation_one_sided` — P(statistic ≥ observed) over random
  relabellings of the pooled runs, the statistic being arm a's value minus
  arm b's. As in :func:`binomial_stats.fisher_one_sided` the caller names
  before the runs which arm is expected to be the better one (arm a): a small
  p rejects "arm b is no worse". The p value is the Monte Carlo one,
  (1 + #{relabellings at least as extreme}) / (1 + n), which never falls
  under the exact p's expectation; a fixed seed makes it reproducible.
* :func:`median_test` and :func:`rmst_test` — that test for the two
  statistics.

numpy only.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence

import numpy as np

# Relabellings evaluated per block (memory: block × runs booleans and floats).
BLOCK = 2000


def km_rmst(
    time: Sequence[float], event: Sequence[bool], tau: float, member: np.ndarray
) -> np.ndarray:
    """(m,) restricted mean survival time up to `tau` of each row of `member` ((m, n) bool).

    ``time`` [s] ≥ 0 and ``event`` (True: the run ended by the event, False:
    censored) describe the n pooled runs. Past a member set's last time the
    curve keeps its last value (0 after an event).
    """
    time = np.asarray(time, dtype=float)
    event = np.asarray(event, dtype=bool)
    member = np.atleast_2d(np.asarray(member, dtype=bool))
    if time.shape != event.shape or member.shape[1] != time.size:
        raise ValueError("time, event and member disagree on the number of runs")
    if not (np.isfinite(time).all() and (time >= 0).all()):
        raise ValueError("survival times must be finite and ≥ 0")
    if not (np.isfinite(tau) and tau > 0):
        raise ValueError(f"tau must be > 0, got {tau}")
    order = np.lexsort((~event, time))  # time ascending; at a tie the events first
    t, d, m = time[order], event[order], member[:, order]
    # Tied events one row at a time: (1 − 1/n)(1 − 1/(n − 1))… = (n − d)/n, the KM factor.
    at_risk = np.cumsum(m[:, ::-1], axis=1)[:, ::-1]
    factor = np.where(m & d, 1.0 - 1.0 / np.maximum(at_risk, 1), 1.0)
    surv = np.concatenate([np.ones((m.shape[0], 1)), np.cumprod(factor, axis=1)], axis=1)
    widths = np.diff(np.minimum(np.concatenate([[0.0], t, [tau]]), tau))
    return surv @ widths


def _memberships(n: int, n_a: int, count: int, rng: np.random.Generator) -> np.ndarray:
    """(count, n) bool: `n_a` of the n runs drawn into arm a, uniformly, per row."""
    keys = rng.random((count, n))
    ranks = np.argsort(np.argsort(keys, axis=1), axis=1)
    return ranks < n_a


def permutation_one_sided(
    n_a: int,
    n_b: int,
    statistic: Callable[[np.ndarray], np.ndarray],
    n_perm: int = 10_000,
    seed: int = 0,
) -> dict:
    """The one-sided permutation test of `statistic` (module docstring).

    ``statistic(member)`` takes an (m, n_a + n_b) bool array — True for the
    runs labelled arm a, the pooled runs in the order the caller gave (arm a's
    first) — and returns the (m,) statistic, arm a minus arm b. An empty arm
    tests nothing: p = 1.
    """
    if n_a <= 0 or n_b <= 0:
        return {"p": 1.0, "observed": None, "n_perm": 0, "seed": seed}
    n = n_a + n_b
    observed_member = np.zeros((1, n), dtype=bool)
    observed_member[0, :n_a] = True
    observed = float(statistic(observed_member)[0])
    # A relabelling equal to the observed one in exact arithmetic may read an ulp lower.
    level = observed - 1e-12 * max(1.0, abs(observed))
    rng = np.random.default_rng(seed)
    extreme = 0
    for start in range(0, n_perm, BLOCK):
        count = min(BLOCK, n_perm - start)
        extreme += int(np.sum(statistic(_memberships(n, n_a, count, rng)) >= level))
    return {
        "p": (1 + extreme) / (1 + n_perm),
        "observed": observed,
        "n_perm": n_perm,
        "seed": seed,
    }


def _masked_median(x: np.ndarray, member: np.ndarray) -> np.ndarray:
    return np.nanmedian(np.where(member, x[None, :], np.nan), axis=1)


def median_test(
    a: Sequence[float], b: Sequence[float], n_perm: int = 10_000, seed: int = 0
) -> dict:
    """median(a) − median(b), one-sided: a small p says arm b's median is lower than a's
    by more than a relabelling explains. Non-finite values are dropped."""
    a = np.asarray([v for v in a if v is not None and np.isfinite(v)], dtype=float)
    b = np.asarray([v for v in b if v is not None and np.isfinite(v)], dtype=float)
    x = np.concatenate([a, b])

    def stat(member: np.ndarray) -> np.ndarray:
        return _masked_median(x, member) - _masked_median(x, ~member)

    out = permutation_one_sided(a.size, b.size, stat, n_perm, seed)
    out.update(n_a=int(a.size), n_b=int(b.size),
               median_a=float(np.median(a)) if a.size else None,
               median_b=float(np.median(b)) if b.size else None)  # fmt: skip
    return out


def rmst_test(
    a_time: Sequence[float],
    a_event: Sequence[bool],
    b_time: Sequence[float],
    b_event: Sequence[bool],
    n_perm: int = 10_000,
    seed: int = 0,
    tau: float | None = None,
) -> dict:
    """RMST(a) − RMST(b) up to `tau`, one-sided (module docstring).

    ``tau`` defaults to the earliest censoring time of the pooled runs — the
    end of observation (a script's end) — so no curve is carried past a run
    still at risk; with nothing censored, to the latest time (both curves are 0
    after their last event). Neither default can be 0 unless every time is,
    and then the arms cannot differ: p = 1.
    """
    if len(a_time) != len(a_event) or len(b_time) != len(b_event):
        raise ValueError("each arm needs one event flag per time")
    time = np.asarray([*a_time, *b_time], dtype=float)
    event = np.asarray([*a_event, *b_event], dtype=bool)
    n_a = len(a_time)
    if tau is None:
        censored = time[~event]
        early = float(censored.min()) if censored.size else 0.0
        tau = early if early > 0.0 else float(time.max(initial=0.0))
    tau = float(tau)
    if tau <= 0.0 and not (time > 0).any():
        return {"p": 1.0, "observed": 0.0, "n_perm": 0, "seed": seed, "tau": tau,
                "n_a": n_a, "n_b": len(b_time), "rmst_a": 0.0, "rmst_b": 0.0}  # fmt: skip

    def stat(member: np.ndarray) -> np.ndarray:
        return km_rmst(time, event, tau, member) - km_rmst(time, event, tau, ~member)

    out = permutation_one_sided(n_a, len(b_time), stat, n_perm, seed)
    if n_a and len(b_time):
        ref = np.zeros((1, time.size), dtype=bool)
        ref[0, :n_a] = True
        out["rmst_a"] = float(km_rmst(time, event, tau, ref)[0])
        out["rmst_b"] = float(km_rmst(time, event, tau, ~ref)[0])
    out.update(tau=tau, n_a=n_a, n_b=len(b_time))
    return out
