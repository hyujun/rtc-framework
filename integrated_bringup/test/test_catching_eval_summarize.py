"""tools/catching_eval/summarize.py: the mechanical parts of the G-1 verdict, on planted inputs.

Every input is built in the test — no recorded unit is read.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "catching_eval"))
import summarize as sm  # noqa: E402

from rtc_tools.analysis import catching_trials as ct  # noqa: E402

A, C, CL, D, H = (
    ct.MODE_APPROACH,
    ct.MODE_COMMITTED,
    ct.MODE_CLOSING,
    ct.MODE_DECEL,
    ct.MODE_HOLD,
)
R, AR, T, AB = ct.MODE_RETREAT, ct.MODE_ARMED, ct.MODE_TRACKING, ct.MODE_ABORT_SAFE


def test_nearest_rank_is_the_ceil_rank():
    v = list(range(1, 101))  # 1..100
    assert sm.nearest_rank(v, 0.99) == 99
    assert sm.nearest_rank(v, 0.5) == 50
    assert sm.nearest_rank([7.0], 0.99) == 7.0
    assert sm.nearest_rank(list(range(1, 201)), 0.99) == 198
    assert sm.nearest_rank([], 0.99) is None
    assert sm.nearest_rank([float("nan"), 3.0], 0.99) == 3.0


@pytest.mark.parametrize(
    ("log", "verdict"),
    [
        (
            [
                [0, "TRACKING"],
                [1, "APPROACH"],
                [2, "DECEL"],
                [3, "HOLD"],
                [4, "RETREAT"],
            ],
            True,
        ),
        (
            [[0, "APPROACH"], [1, "ABORT_SAFE"], [2, "RETREAT"]],
            False,
        ),  # RETREAT not from HOLD
        (
            [[0, "APPROACH"], [1, "ABORT_SAFE"], [2, "HOLD"], [3, "RETREAT"]],
            False,
        ),  # an abort
        ([[0, "APPROACH"], [1, "DECEL"], [2, "HOLD"]], False),  # no RETREAT
        ([], False),
    ],
)
def test_mode_log_verdict(log, verdict):
    hold, abort = sm.mode_log_verdict([[*m, 0] for m in log])
    assert (hold and not abort) == verdict


@pytest.mark.parametrize(
    ("modes", "window"),
    [
        ([T, A, C, CL, D, D, H, R], (1, 5)),  # to the last DECEL tick
        (
            [T, A, C, D, H, D, H, R],
            (1, 5),
        ),  # the LAST DECEL, not the first stretch's end
        ([T, A, C, AB, AB, R, AR], (1, 4)),  # no DECEL: up to RETREAT, abort included
        ([T, A, C, T, A], (1, 2)),  # back to TRACKING
        ([T, T, AR], None),  # never in APPROACH
        ([T, A, C, CL], (1, 3)),  # log ends inside
    ],
)
def test_follow_window(modes, window):
    assert sm.follow_window(np.array(modes)) == window


def _row(seed, i, d4, invalid=""):
    return {
        "kind": "s35b",
        "seed": seed,
        "sample_idx": i,
        "invalid_reason": invalid,
        "d4": d4,
        "truth": d4,
    }


def test_pairs_join_on_seed_and_sample_not_position():
    cf = [_row(631, i, i < 3) for i in range(5)]
    mpc = [_row(631, i, i >= 2) for i in reversed(range(5))]  # same throws, other order
    pr = sm.pair(cf, mpc)
    tab = sm.table(pr["valid"], "d4")
    assert (tab["both"], tab["mpc_only"], tab["cf_only"], tab["neither"]) == (
        1,
        2,
        2,
        0,
    )


def test_an_invalid_trial_drops_its_pair_and_is_tabled_by_arm_and_reason():
    cf = [_row(631, i, True, "lane_drop" if i == 0 else "") for i in range(8)]
    mpc = [_row(631, i, True, "sim_stall" if i in (0, 1) else "") for i in range(8)]
    cf += [_row(632, i, True) for i in range(3)]
    mpc += [_row(632, i, False) for i in range(3)]
    pr = sm.pair(cf, mpc)
    assert pr["dropped_n"] == 2
    assert pr["dropped"] == {"both/lane_drop/sim_stall": 1, "mpc_only/-/sim_stall": 1}
    assert len(pr["valid"]) == 9
    assert pr["seeds_to_rerun"] == []
    tab = sm.table(pr["valid"], "d4")
    assert (tab["both"], tab["cf_only"]) == (6, 3)


def test_six_dropped_pairs_in_a_seed_ask_for_a_rerun():
    cf = [_row(633, i, True, "lane_drop" if i < 6 else "") for i in range(10)]
    mpc = [_row(633, i, True) for i in range(10)]
    assert sm.pair(cf, mpc)["seeds_to_rerun"] == [633]
    cf = [_row(633, i, True, "lane_drop" if i < 5 else "") for i in range(10)]
    assert sm.pair(cf, mpc)["seeds_to_rerun"] == []


def test_a_throw_twice_in_one_arm_is_refused():
    with pytest.raises(SystemExit, match="twice"):
        sm.pair([_row(631, 0, True), _row(631, 0, False)], [_row(631, 0, True)])


def test_units_sharing_a_name_are_refused():
    # Rows are kept by the unit's directory name: the same name in both arms
    # would have each arm read the other's rows.
    with pytest.raises(SystemExit, match="p1b_631"):
        sm.refuse_shared_names(["cf/p1b_631", "cf/p1b_632", "mpc/p1b_631"])
    with pytest.raises(SystemExit, match="twice"):
        sm.refuse_shared_names(["u/p1b_631", "u/p1b_631"])
    sm.refuse_shared_names(["u/cf_p1b_631", "u/mpc_p1b_631"])


def test_the_test_block_is_the_repo_tango_on_the_table():
    t = sm.test_block({"n": 200, "both": 85, "mpc_only": 38, "cf_only": 48, "neither": 29})
    assert t["z"] == pytest.approx(1.0826, abs=1e-4)
    assert not t["noninferior"]
    assert t["wald_ci95"] == pytest.approx([-0.14062, 0.04062], abs=1e-5)


def test_worst_case_counts_dropped_pairs_as_mpc_failures():
    tab = {"n": 100, "both": 40, "mpc_only": 10, "cf_only": 10, "neither": 40}
    w = sm.worst_case(tab, 5)
    assert (w["n"], w["cf_only"]) == (105, 15)


def _solves(search=True, first=True, replan=True):
    def blk(p):
        return {"pass": p}

    return (
        {"search": blk(search)},
        {"search": blk(True), "first": blk(first), "replan": blk(replan)},
    )


def _lim(win, bad):
    return {"trials_with_window": win, "violating_trials": bad}


def test_verdict_labels():
    ok = {"noninferior": True}
    cf, mp = _solves()
    assert sm.verdict(ok, _lim(10, 0), cf, mp, "pass")[1] == "PASS"
    assert sm.verdict({"noninferior": False}, _lim(10, 0), cf, mp, "pass")[1] == "FAIL"
    # the control arm's search over budget fails the robot too (MD-87 (3), both arms)
    cf_bad, _ = _solves(search=False)
    assert sm.verdict(ok, _lim(10, 0), cf_bad, mp, "pass")[1] == "FAIL"
    assert sm.verdict(ok, _lim(10, 1), cf, mp, "pass")[1] == "FAIL"
    # an empty sample is not a PASS
    label = sm.verdict(ok, _lim(0, 0), cf, mp, "pass")[1]
    assert label.startswith("PASS 아님") and "2_limits" in label
    _, mp_none = _solves(replan=None)
    label = sm.verdict(ok, _lim(10, 0), cf, mp_none, "pass")[1]
    assert label.startswith("PASS 아님") and "3.replan" in label
    # FAIL outranks not-evaluable
    assert sm.verdict({"noninferior": False}, _lim(0, 0), cf, mp_none, "pass")[1] == "FAIL"
    assert sm.verdict(ok, _lim(10, 0), cf, mp, "unknown")[1].startswith("PASS 아님")


def test_limit_check_is_strict():
    lim = {
        "position_lower": [-1.0],
        "position_upper": [1.0],
        "max_velocity": [2.0],
        "max_torque": [5.0],
    }
    at = sm.limit_check(np.array([[1.0]]), np.array([[2.0]]), np.array([[-5.0]]), lim)
    assert (at["pos"], at["vel"], at["torque"]) == (0, 0, 0)  # equal is not a violation
    over = sm.limit_check(np.array([[1.0001]]), np.array([[-2.0001]]), np.array([[5.0001]]), lim)
    assert (over["pos"], over["vel"], over["torque"]) == (1, 1, 1)
