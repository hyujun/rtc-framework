"""Small-sample statistics of success counts: an interval, a two-arm test and its power.

Used where a run either completes or does not and an arm is a handful of runs:
the interval says how little a count pins the rate down, the test is the exact
one for two counts, and the power says what a "no difference" from that test is
worth at the arm size in hand.

Pure Python: no numpy, no scipy.
"""

from __future__ import annotations

import math


def wilson_interval(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    """Wilson score interval for k successes in n trials.

    ``z`` is stated, never implied: 1.96 is the two-sided 95 % interval, whose
    lower bound is the 97.5 % one-sided bound the truth verdict uses.
    """
    if n <= 0:
        return (math.nan, math.nan)
    if not 0 <= k <= n:
        raise ValueError(f"k={k} outside [0, n={n}]")
    p = k / n
    z2 = z * z
    denom = 1.0 + z2 / n
    centre = (p + z2 / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z2 / (4 * n * n)) / denom
    return (max(0.0, centre - half), min(1.0, centre + half))


def fisher_one_sided(a_k: int, a_n: int, b_k: int, b_n: int) -> float:
    """P(arm a has ≥ `a_k` successes | both arms share one rate) — Fisher's exact test.

    The hypergeometric upper tail of arm a's count given the pooled count: small
    when arm a did better than arm b by more than the split of a common rate
    explains. One-sided on purpose — the caller names which arm is expected to
    be the better one before the runs. An empty arm tests nothing: 1.0.
    """
    for k, n in ((a_k, a_n), (b_k, b_n)):
        if n < 0 or not 0 <= k <= n:
            raise ValueError(f"k={k} outside [0, n={n}]")
    if a_n == 0 or b_n == 0:
        return 1.0
    pooled, total = a_k + b_k, a_n + b_n
    tail = sum(
        math.comb(a_n, x) * math.comb(b_n, pooled - x)
        for x in range(a_k, min(a_n, pooled) + 1)
        if 0 <= pooled - x <= b_n
    )
    return tail / math.comb(total, pooled)


def _binomial_pmf(n: int, p: float) -> list[float]:
    return [math.comb(n, k) * p**k * (1.0 - p) ** (n - k) for k in range(n + 1)]


def fisher_power(p_a: float, p_b: float, n: int, alpha: float = 0.05) -> float:
    """P(:func:`fisher_one_sided` < `alpha`) with `n` runs per arm at true rates `p_a` > `p_b`.

    Exact: every pair of counts weighted by its binomial probability. At
    ``p_a == p_b`` it is the test's actual size (below `alpha`: the test is
    conservative).
    """
    if not (0.0 <= p_a <= 1.0 and 0.0 <= p_b <= 1.0):
        raise ValueError(f"rates must be in [0, 1], got {p_a}, {p_b}")
    pmf_a, pmf_b = _binomial_pmf(n, p_a), _binomial_pmf(n, p_b)
    power = 0.0
    for a_k in range(n + 1):
        for b_k in range(n + 1):
            if fisher_one_sided(a_k, n, b_k, n) < alpha:
                power += pmf_a[a_k] * pmf_b[b_k]
    return power
