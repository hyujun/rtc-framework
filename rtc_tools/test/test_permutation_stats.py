"""rtc_tools.analysis.permutation_stats: hand-computed RMSTs, the p value's form, A/A size."""

from __future__ import annotations

import math

import numpy as np
import pytest

from rtc_tools.analysis import permutation_stats as ps

TOL = 1e-12


def rmst(time, event, tau):
    return float(ps.km_rmst(time, event, tau, np.ones((1, len(time)), dtype=bool))[0])


def test_without_censoring_the_rmst_is_the_mean_of_the_clipped_times():
    t = [0.5, 2.0, 3.0, 7.0]
    assert rmst(t, [True] * 4, 4.0) == pytest.approx(np.mean(np.minimum(t, 4.0)), abs=TOL)


def test_the_kaplan_meier_rmst_by_hand():
    # S = 3/4 on [1, 3) (the censoring at 2 leaves 2 at risk), 3/8 on [3, 4), 0 after.
    assert rmst([1, 2, 3, 4], [1, 0, 1, 1], 4.0) == pytest.approx(1 + 2 * 0.75 + 0.375, abs=TOL)
    # Two events tied at 1 of 3 at risk: S = 1/3 after — the tie in one step or two is the same.
    assert rmst([1, 1, 2], [1, 1, 0], 2.0) == pytest.approx(1 + 1 / 3, abs=TOL)
    # An event and a censoring tied: the event first, so the censored run is still at risk.
    assert rmst([1, 1], [0, 1], 2.0) == pytest.approx(1 + 0.5, abs=TOL)
    # Everything censored at the end: S = 1 throughout.
    assert rmst([5, 5, 5], [0, 0, 0], 5.0) == pytest.approx(5.0, abs=TOL)


def test_member_sets_are_independent_rows():
    time, event = [1.0, 2.0, 3.0, 4.0], [True, True, False, True]
    member = np.array([[1, 1, 0, 0], [0, 0, 1, 1], [1, 0, 1, 0]], dtype=bool)
    got = ps.km_rmst(time, event, 3.0, member)
    for row, want in zip(member, got, strict=True):
        t = [x for x, m in zip(time, row, strict=True) if m]
        e = [x for x, m in zip(event, row, strict=True) if m]
        assert rmst(t, e, 3.0) == pytest.approx(want, abs=TOL)


@pytest.mark.parametrize(
    ("time", "event", "tau"),
    [([1.0, -0.1], [1, 1], 1.0), ([1.0, math.nan], [1, 1], 1.0), ([1.0], [1], 0.0)],
)
def test_bad_inputs_are_refused(time, event, tau):
    with pytest.raises(ValueError):
        rmst(time, event, tau)


def test_a_separation_no_relabelling_matches_has_the_smallest_p():
    # Distinct values: only the observed labelling reaches its difference (with ties, a
    # majority of the high values in arm a would already give the same median).
    a, b = [100.0 + i for i in range(12)], [float(i) for i in range(12)]
    out = ps.median_test(a, b, n_perm=999, seed=3)
    assert out["observed"] == 100.0
    assert out["p"] == pytest.approx(1 / 1000, abs=TOL)  # C(24, 12) ≈ 2.7e6 labellings
    assert (out["median_a"], out["median_b"]) == (105.5, 5.5)


def test_the_test_is_one_sided_and_reproducible():
    rng = np.random.default_rng(0)
    a, b = rng.normal(1.0, 0.3, 20), rng.normal(0.0, 0.3, 20)
    good = ps.median_test(a, b, n_perm=2000, seed=7)
    assert good["p"] < 0.01
    assert ps.median_test(b, a, n_perm=2000, seed=7)["p"] > 0.99  # b is not the better arm
    assert ps.median_test(a, b, n_perm=2000, seed=7) == good


def test_identical_arms_give_a_p_of_at_least_one_half():
    x = [0.1, 0.2, 0.3, 0.4, 0.5]
    assert ps.median_test(x, x, n_perm=2000, seed=1)["p"] >= 0.5


def test_an_empty_arm_tests_nothing_and_non_finite_values_drop():
    assert ps.median_test([], [1.0], n_perm=10)["p"] == 1.0
    out = ps.median_test([1.0, None, math.nan], [0.0], n_perm=10)
    assert (out["n_a"], out["n_b"]) == (1, 1)


def test_the_rmst_horizon_is_the_earliest_censoring_time():
    out = ps.rmst_test([30.0, 32.0, 10.0], [0, 0, 1], [5.0, 20.0], [1, 1], n_perm=500, seed=0)
    assert out["tau"] == 30.0
    assert out["rmst_a"] == pytest.approx((30 + 30 + 10) / 3, abs=TOL)
    assert out["rmst_b"] == pytest.approx((5.0 + 20.0) / 2, abs=TOL)  # 0 after its last event
    assert out["observed"] == pytest.approx(out["rmst_a"] - out["rmst_b"], abs=TOL)
    # Nothing censored: the latest time.
    assert ps.rmst_test([3.0, 4.0], [1, 1], [1.0], [1], n_perm=10)["tau"] == 4.0
    with pytest.raises(ValueError):
        ps.rmst_test([1.0], [1, 0], [1.0], [1])


def test_an_arm_that_collapses_early_is_rejected_whatever_its_times():
    # Every candidate run ends at about the same early time: the horizon stays at the
    # control's censoring, so the collapse is a large RMST difference.
    a_t, a_e = [40.0] * 6 + [8.0 + i for i in range(6)], [0] * 6 + [1] * 6
    b_t, b_e = [5.0 + 0.01 * i for i in range(12)], [1] * 12
    out = ps.rmst_test(a_t, a_e, b_t, b_e, n_perm=999, seed=0)
    assert out["tau"] == 40.0 and out["p"] < 0.01
    # Every candidate run fell before its first step (survival 0): no crash, still rejected.
    out = ps.rmst_test(a_t, a_e, [0.0] * 12, [1] * 12, n_perm=999, seed=0)
    assert out["rmst_b"] == 0.0 and out["p"] < 0.01
    assert ps.rmst_test([0.0], [1], [0.0], [1], n_perm=10)["p"] == 1.0  # nothing to compare


def test_the_size_under_the_null_is_at_most_alpha():
    """A/A: both arms from one survival law (censored at its end) and one ratio law."""
    rng = np.random.default_rng(20261121)
    trials, n, alpha, end = 300, 24, 0.05, 30.0
    rejects = {"rmst": 0, "median": 0}
    for k in range(trials):
        t = np.minimum(rng.exponential(15.0, 2 * n), end)
        e = t < end
        if ps.rmst_test(t[:n], e[:n], t[n:], e[n:], n_perm=499, seed=k)["p"] < alpha:
            rejects["rmst"] += 1
        v = rng.normal(0.2, 0.04, 2 * n)
        if ps.median_test(v[:n], v[n:], n_perm=499, seed=k)["p"] < alpha:
            rejects["median"] += 1
    slack = 3 * math.sqrt(alpha * (1 - alpha) / trials)
    for name, count in rejects.items():
        assert count / trials <= alpha + slack, name
