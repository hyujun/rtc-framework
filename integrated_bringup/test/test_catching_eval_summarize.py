"""tools/catching_eval/summarize.py: the mechanical parts of the G-1 verdict, on planted inputs.

Every input is built in the test — no recorded unit is read.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

TOOLS = Path(__file__).resolve().parents[1] / "tools" / "catching_eval"
sys.path.insert(0, str(TOOLS))
import summarize as sm  # noqa: E402

from integrated_bringup import catching_tc_vector  # noqa: E402
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


# ── Old column names and old tool outputs (#711) ─────────────────────────────


def _read_diag(tmp_path, *, old_names: bool):
    """A diag as ``ct._read_csv`` returns it, summarize's own ``usecols`` included, for a
    file written with the old column names or the new ones."""
    import pandas as pd

    frame = pd.DataFrame(
        {
            "t_relative_s": np.arange(12) * 0.002,
            "mode": [ct.MODE_APPROACH] * 6 + [ct.MODE_HOLD] * 6,
            "segment_event": [0, 4, 4, 5, 4, 0, 0, 4, 5, 0, 0, 0],
            "segment_rho": np.linspace(0.1, 0.9, 12),
        }
    )
    if old_names:
        frame = frame.rename(columns={"segment_event": "decel_event", "segment_rho": "decel_rho"})
    path = tmp_path / ("old" if old_names else "new") / "catching_diag.csv"
    path.parent.mkdir()
    frame.to_csv(path, index=False)
    want = {"t_relative_s", "mode", "segment_event", "segment_rho"}
    return ct._read_csv(path, usecols=lambda c: c in want)


def _ct_frame(**columns):
    import pandas as pd

    return pd.DataFrame({"idx": [0, 1], "rtf_trial_min": [1.0, 1.0], **columns})


def test_the_replan_block_is_the_same_for_a_diag_recorded_with_old_names(tmp_path):
    diags = [_read_diag(tmp_path, old_names=old) for old in (False, True)]
    blocks = [sm.replan_block([{"diag": d, "ct": _ct_frame(segment_aged=[1, 2])}]) for d in diags]
    assert blocks[0] == blocks[1]
    assert blocks[0]["aged"] == 3 and blocks[0]["replan_switches"] == 3


def test_an_old_catching_trials_output_is_refused_not_skipped(tmp_path):
    summ_new = {"arm_joints": [], "medians": {"segment_wait_node0_ms": 1.0}, "segment_lane": {}}
    sm.reject_old_ct_outputs(tmp_path, summ_new, _ct_frame(segment_aged=[0, 0]), None)  # no error
    with pytest.raises(SystemExit) as exc:
        sm.reject_old_ct_outputs(tmp_path, summ_new, _ct_frame(decel_aged=[0, 0]), None)
    msg = str(exc.value)
    assert "catching_trials.csv" in msg and "decel_aged -> segment_aged" in msg
    assert "older catching_trials" in msg and "regenerate" in msg


def test_a_ct_from_before_the_lane_metrics_is_refused_not_counted_as_zero(tmp_path):
    # Neither spelling of the lane metrics: nothing renamed to refuse, and the replan
    # block reads segment_aged — it used to count such a unit's aged segments as 0.
    import pandas as pd

    before = pd.DataFrame({"idx": [0, 1], "caught": [1, 0]}).set_index("idx")
    with pytest.raises(SystemExit, match=r"catching_trials.csv: no segment_aged.*regenerate"):
        sm.reject_old_ct_outputs(tmp_path, {"arm_joints": []}, before, None)


def test_an_old_summary_json_and_an_old_vec_csv_are_refused_too(tmp_path):
    import pandas as pd

    ok = _ct_frame(segment_aged=[0, 0])
    with pytest.raises(SystemExit, match="decel_lane -> segment_lane"):
        sm.reject_old_ct_outputs(tmp_path, {"decel_lane": {}}, ok, None)
    with pytest.raises(SystemExit, match="decel_wait_node0_ms -> segment_wait_node0_ms"):
        sm.reject_old_ct_outputs(tmp_path, {"medians": {"decel_wait_node0_ms": 1.0}}, ok, None)
    vec = pd.DataFrame({"idx": [0], "decel_segments_followed": [2]})
    with pytest.raises(SystemExit, match=r"vec.csv.*older tc_vector.*segment_n_followed"):
        sm.reject_old_ct_outputs(tmp_path, {}, ok, vec)


def test_run_unit_asks_the_startup_check_and_no_longer_greps_for_refus():
    text = (TOOLS / "run_unit.sh").read_text()
    assert 'check_startup.py" "$OUT/launch.log"' in text
    assert "bring_up_failed\\|refus" not in text


def test_a_unit_that_asks_for_the_probe_dump_gets_it_flushed_and_handed_on():
    """PROBE_DUMP=1 (#807): the probe runs in its own group, is stopped BEFORE the
    estimator and the sim (it writes its CSVs on SIGINT), a unit without the dump
    fails, and the analysis hands the dump to catching_trials."""
    text = (TOOLS / "run_unit.sh").read_text()
    assert 'vision_lane_probe "$OUT/probe/lane" --dump' in text
    cleanup = text[text.index("cleanup() {") : text.index("keep_failed_session() {")]
    assert cleanup.index('stop_group "$PPG"') < cleanup.index('stop_group "$EPG"')
    # A probe that died at its start, or that had to be killed, fails the unit too.
    for why in ("probe not running", "probe killed", "no probe dump"):
        assert f'echo "FAIL:{why}" > "$OUT/status"' in text
    assert text.index('echo "FAIL:probe not running"') < text.index("catching_sim_trials")
    assert 'echo "probe_dump: ' in text
    # tc_vector.py is a shim since #821: the analysis it runs is the installed module.
    assert (
        "from integrated_bringup.catching_tc_vector import main"
        in (TOOLS / "tc_vector.py").read_text()
    )
    vector = Path(catching_tc_vector.__file__).read_text()
    assert 'unit / "probe" / "lane_prediction_dump.csv"' in vector
    assert '"--probe-dump", str(dump)' in vector


def test_a_conditions_n_min_is_counted_from_that_robots_own_horizon():
    """make_conditions.py once counted ``io.n_min`` from one constant for both
    robots; a robot whose shipped ``io.horizon_min`` is longer then got overlays
    one sample short of its controller's own check (``ceil(horizon_min / dt) + 1``)."""
    import make_conditions as mc

    assert not hasattr(mc, "HORIZON_MIN_S")
    # 25 ms grid: 0.51 s needs 22 samples, 0.53 s needs 23.
    assert mc.n_min_for(0.51, 0.025, 40) == 22
    assert mc.n_min_for(0.53, 0.025, 40) == 23
    assert mc.n_min_for(0.53, 0.05, 20) == 12
    with pytest.raises(SystemExit, match="n_min 30 <= points 24"):
        mc.n_min_for(0.53, 0.01875, 24)
    assert mc.grid(1.0, 40) == 0.025


# ── #798: units that threw a list pair on (the list's sha256, throw_id) ──────
def _list_row(sha, throw_id, d4, invalid=""):
    return {
        "kind": "lhs",
        "seed": None,
        "sample_idx": None,
        "throw_id": throw_id,
        "throws_file_sha256": sha,
        "invalid_reason": invalid,
        "d4": d4,
        "truth": d4,
    }


def test_list_units_pair_on_throw_id_and_a_seedless_row_does_not_crash():
    sha = "ab" * 32
    cf = [_list_row(sha, 40 + i, i < 3) for i in range(5)]
    mpc = [_list_row(sha, 44 - i, i >= 3) for i in range(5)]  # the same ids, other order
    mpc.append(_list_row("cd" * 32, 40, True))  # another list: pairs with nothing
    mpc.append({**_list_row(None, 41, True), "seed": None})  # a row that lost its list
    pr = sm.pair(cf, mpc)
    assert pr["keys"] == 5 and pr["unpaired_cf"] == 0 and pr["unpaired_mpc"] == 1
    assert pr["unkeyed"] == {"mpc": 1}
    assert [k for k, _, _ in pr["valid"]] == [("list", sha, 40 + i) for i in range(5)]
    # the two arms' rows met by id, not by position: cf holds ids 40–42, mpc 40–41
    # (a join by position would read (T,F) (T,F) (T,F) (F,T) (F,T))
    assert [(ra["d4"], rb["d4"]) for _, ra, rb in pr["valid"]] == [
        (True, True),
        (True, True),
        (True, False),
        (False, False),
        (False, False),
    ]
    assert sm.seed_block(pr["valid"])["per_seed"].keys() == {"list:abababababab"}


def test_a_dropped_list_pair_is_counted_against_its_list():
    sha = "ab" * 32
    cf = [_list_row(sha, i, True) for i in range(8)]
    mpc = [_list_row(sha, i, True, invalid="lane_drop" if i < 6 else "") for i in range(8)]
    pr = sm.pair(cf, mpc)
    assert pr["dropped_n"] == 6 and pr["seed_drops"] == {"list:abababababab": 6}
    assert pr["seeds_to_rerun"] == ["list:abababababab"]
