"""rtc_tools.analysis.binomial_stats against hand-checked values."""

from __future__ import annotations

import math

import pytest

from rtc_tools.analysis import binomial_stats as bs, catching_trials


def test_the_wilson_interval_of_a_half():
    lo, hi = bs.wilson_interval(9, 18)
    assert (lo, hi) == pytest.approx((0.2903, 0.7097), abs=5e-5)
    assert lo + hi == pytest.approx(1.0)
    assert bs.wilson_interval(0, 10)[0] == 0.0 and bs.wilson_interval(10, 10)[1] == 1.0
    assert all(math.isnan(x) for x in bs.wilson_interval(0, 0))
    with pytest.raises(ValueError):
        bs.wilson_interval(3, 2)


def test_the_catching_module_keeps_the_same_function():
    assert catching_trials.wilson_interval is bs.wilson_interval


def test_fisher_is_the_hypergeometric_upper_tail():
    # 2 x 2 with every margin 2: P(a = 2) = 1/6, P(a >= 1) = 5/6.
    assert bs.fisher_one_sided(2, 2, 0, 2) == pytest.approx(1 / 6)
    assert bs.fisher_one_sided(1, 2, 1, 2) == pytest.approx(5 / 6)
    assert bs.fisher_one_sided(0, 2, 2, 2) == 1.0
    # Tea tasting (Fisher 1935): 3 of 4 right both ways, p = 17/70.
    assert bs.fisher_one_sided(3, 4, 1, 4) == pytest.approx(17 / 70)


def test_fisher_is_one_sided():
    assert bs.fisher_one_sided(22, 42, 23, 46) == pytest.approx(0.496163, abs=1e-6)
    better, worse = bs.fisher_one_sided(15, 18, 5, 18), bs.fisher_one_sided(5, 18, 15, 18)
    assert better < 0.01 and worse > 0.99


def test_fisher_refuses_impossible_counts_and_ignores_an_empty_arm():
    with pytest.raises(ValueError):
        bs.fisher_one_sided(5, 4, 1, 4)
    assert bs.fisher_one_sided(0, 0, 3, 4) == 1.0


@pytest.mark.parametrize(
    ("low", "n", "power"),
    [(0.3, 18, 0.26), (0.3, 54, 0.61), (0.3, 93, 0.85), (0.2, 18, 0.50), (0.2, 54, 0.93)],
)
def test_the_power_at_the_arm_sizes_in_use(low, n, power):
    assert bs.fisher_power(0.5, low, n) == pytest.approx(power, abs=0.006)


def test_the_test_is_conservative_at_equal_rates():
    assert bs.fisher_power(0.5, 0.5, 18) < 0.05
