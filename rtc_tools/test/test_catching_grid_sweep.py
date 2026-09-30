"""catching_grid_sweep — planted units whose every reported number is known by hand.

Each unit is written in the files the runner and catching_trials leave: the
trials dir (``trial_results.json`` + ``run_meta.json`` with the mirror), the
unit's ``ct/catching_trials.csv`` and a session with the catching diag and the
planner's events. Outcomes, message timing, points and cycle times are planted,
so a wrong join, a wrong filter or a wrong statistic moves a number the test
pins.
"""

import csv
import json
import math

import pytest

from rtc_tools.analysis import catching_grid_sweep as gs

CTL = "demo_catching_controller"
GRID_50 = {"prediction.dt_expected": 0.05, "io.n_min": 12, "planner.slice.dt": 0.05}
GRID_25 = {"prediction.dt_expected": 0.025, "io.n_min": 22, "planner.slice.dt": 0.025}


def _write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def make_unit(
    root,
    name,
    outcomes,
    *,
    seed=601,
    grid=GRID_50,
    points=20,
    period_s=0.034,
    search_us=(1000.0,),
    pred_mm=None,
    tc_axis=None,
    rtf=None,
):
    """A unit of ``len(outcomes)`` s35b throws; flight k streams 10 messages from t = 3k s."""
    unit = root / name
    ses = unit / "session" / "controllers" / CTL
    n = len(outcomes)
    diag = []
    for k in range(n):
        t0 = 3.0 * k
        for m in range(10):
            diag.append({"t_relative_s": t0 + m * period_s, "input_new": 1, "input_n": points})
        # a new snapshot with no points (an invalid/ghost message) does not count
        diag.append({"t_relative_s": t0 + 10 * period_s, "input_new": 1, "input_n": 0})
        diag.append({"t_relative_s": t0 + 11 * period_s, "input_new": 0, "input_n": points})
    _write_csv(ses / "catching_diag.csv", diag)
    events = [
        {"search_us": s, "n_in_window": 5, "n_ik": 3, "budget_hit": 0} for s in search_us
    ] + [{"search_us": 99999.0, "n_in_window": 0, "n_ik": 0, "budget_hit": 1}]
    _write_csv(ses / "planner_events.csv", events)
    _write_csv(
        unit / "ct" / "catching_trials.csv",
        [
            {
                "idx": i,
                "kind": "s35b",
                "t_launch": 3.0 * i,
                "invalid_reason": "",
                "truth_success": str(ok),
                "supervisor": "CAPTURED" if ok else "MISSED",
                "pred_mm": (pred_mm or [10.0] * n)[i],
                "total_mm": 20.0,
                "tc_axis": (tc_axis or ["ok"] * n)[i],
                "contact_v_rel": 1.0,
                "approach_plan_switches": 0,
                "rtf_trial_min": (rtf or [1.0] * n)[i],
            }
            for i, ok in enumerate(outcomes)
        ],
    )
    (unit / "trials").mkdir(parents=True)
    (unit / "trials" / "trial_results.json").write_text(
        json.dumps(
            [
                {"idx": i, "kind": "s35b", "seed": seed, "sample_idx": i, "accepted": False}
                for i in range(n)
            ]
        )
    )
    (unit / "trials" / "run_meta.json").write_text(
        json.dumps({"arm": name, "controller_mirror": {"control.dt": 0.002, **grid}})
    )
    return unit


def test_paired_difference_is_the_hand_computed_wald_interval():
    t = {"n_pairs": 100, "only_a": 10, "only_b": 20}
    d = gs.paired_difference(t)
    se = math.sqrt(30 - 10**2 / 100) / 100
    assert d["diff"] == pytest.approx(0.10)
    assert d["ci95"] == pytest.approx([0.10 - 1.96 * se, 0.10 + 1.96 * se])
    # no pairs: nothing to say, not zero
    assert math.isnan(gs.paired_difference({"n_pairs": 0, "only_a": 0, "only_b": 0})["diff"])


def test_holm_is_step_down_and_keeps_the_input_order():
    assert gs.holm([0.01, 0.04, 0.03]) == pytest.approx([0.03, 0.06, 0.06])
    assert gs.holm([0.5, 0.9]) == pytest.approx([1.0, 1.0])
    assert gs.holm([]) == []


def test_the_stream_reads_intervals_within_flights_and_the_points_of_real_messages(tmp_path):
    unit = make_unit(tmp_path, "u", [True, False, True], points=25, period_s=0.034)
    s = gs.message_stream(unit / "session" / "controllers" / CTL / "catching_diag.csv")
    # 10 messages per flight → 9 intervals; the gap to the next flight (≈ 2.6 s) is not one
    assert s["n_messages"] == 30
    assert s["interval_ms"]["n"] == 27
    assert s["interval_ms"]["p50"] == pytest.approx(34.0)
    assert s["interval_ms"]["max"] == pytest.approx(34.0)
    # the empty new snapshots are not messages of this grid
    assert s["points"] == {25: 30} and s["points_mode"] == 25


def test_planner_cycles_count_only_the_cycles_with_a_candidate(tmp_path):
    unit = make_unit(tmp_path, "u", [True], search_us=(1000.0, 3000.0, 2000.0))
    p = gs.planner_cycles(unit / "session" / "controllers" / CTL / "planner_events.csv")
    assert sorted(p["search_us"]) == [1000.0, 2000.0, 3000.0]  # the 99999 idle row is out
    assert p["budget_hit"] == 0 and p["n_in_window_max"] == 5 and p["n_ik_max"] == 3


def test_an_arm_reports_its_rate_grid_message_and_errors(tmp_path):
    u1 = make_unit(
        tmp_path,
        "a1",
        [True, True, False, True],
        grid=GRID_25,
        points=40,
        pred_mm=[10.0, 20.0, 30.0, 1000.0],
        tc_axis=["ok", "ok", "ok", "shifted"],
        rtf=[1.0, 0.9, 1.0, 1.0],
        search_us=(1000.0, 30000.0),
    )
    u2 = make_unit(tmp_path, "a2", [False, True], seed=602, grid=GRID_25, points=40)
    units = [gs.analyse_unit(*gs.cd.parse_unit_arg(str(u))) for u in (u1, u2)]
    s = gs.summarise_arm("L-25", units, budget_s=0.020)
    assert (s["truth_success"], s["n_valid"]) == (4, 6)
    assert s["grid"] == GRID_25
    assert s["input_n_mode"] == 40 and s["message_bytes"] == 40 * gs.POINT_STEP
    # the shifted trial's 1000 mm is not a prediction error at t_c
    assert s["tc_axis_shifted"] == 1
    assert s["pred_mm"]["max"] == pytest.approx(30.0) and s["pred_mm"]["n"] == 5
    assert s["rtf_below_min"] == 1
    assert s["planner_search_us"]["max"] == pytest.approx(30000.0)
    assert s["planner_over_budget"] is True  # p99 of {1000, 30000, 1000} > 20 ms


def test_units_of_one_arm_that_ran_different_grids_are_refused(tmp_path):
    u1 = make_unit(tmp_path, "a1", [True], grid=GRID_25)
    u2 = make_unit(tmp_path, "a2", [True], seed=602, grid=GRID_50)
    units = [gs.analyse_unit(*gs.cd.parse_unit_arg(str(u))) for u in (u1, u2)]
    with pytest.raises(SystemExit, match="different grids"):
        gs.summarise_arm("mixed", units)


def test_comparisons_pair_the_same_throws_and_adjust_within_the_family(tmp_path):
    ref = [True] * 6 + [False] * 4
    # B: two of ref's successes fail, one of its failures succeeds → only_a 2, only_b 1
    b = [False, False] + [True] * 4 + [True] + [False] * 3
    # C: identical to ref
    arms = {
        "ref": [gs.analyse_unit(*gs.cd.parse_unit_arg(str(make_unit(tmp_path, "r", ref))))],
        "B": [gs.analyse_unit(*gs.cd.parse_unit_arg(str(make_unit(tmp_path, "b", b))))],
        "C": [gs.analyse_unit(*gs.cd.parse_unit_arg(str(make_unit(tmp_path, "c", ref))))],
    }
    rows = gs.compare(arms, [("ref", "B"), ("ref", "C")])
    rb, rc = rows
    assert (rb["n_pairs"], rb["only_a"], rb["only_b"]) == (10, 2, 1)
    assert rb["diff"] == pytest.approx(-0.1)
    assert (rc["only_a"], rc["only_b"], rc["diff"]) == (0, 0, 0.0)
    assert rb["mcnemar_p"] == pytest.approx(1.0)  # 3 discordant, 2:1
    assert rb["holm_p"] >= rb["mcnemar_p"]


def test_a_throw_of_another_seed_is_not_paired(tmp_path):
    arms = {
        "ref": [gs.analyse_unit(*gs.cd.parse_unit_arg(str(make_unit(tmp_path, "r", [True] * 3))))],
        "B": [
            gs.analyse_unit(
                *gs.cd.parse_unit_arg(str(make_unit(tmp_path, "b", [False] * 3, seed=999)))
            )
        ],
    }
    (row,) = gs.compare(arms, [("ref", "B")])
    assert row["n_pairs"] == 0 and row["unpaired_a"] == 3
    assert math.isnan(row["diff"])


def test_cli_writes_the_summary_and_the_trials(tmp_path, capsys):
    r = make_unit(tmp_path, "r", [True, False, True])
    b = make_unit(tmp_path, "b", [True, True, False], grid=GRID_25, points=40)
    out = tmp_path / "out"
    rc = gs.main(
        [
            "--arm",
            "L-50",
            str(r),
            "--arm",
            "L-25",
            str(b),
            "--ref",
            "L-50",
            "--pair",
            "L-25:L-50",
            "--out",
            str(out),
        ]
    )
    assert rc == 0
    doc = json.loads((out / "grid_sweep_summary.json").read_text())
    assert [a["arm"] for a in doc["arms"]] == ["L-50", "L-25"]
    assert set(doc["comparisons"]) == {"vs L-50", "pairs"}
    with (out / "grid_sweep_trials.csv").open() as f:
        assert len(list(csv.DictReader(f))) == 6
    assert "L-25 − L-50" in capsys.readouterr().out


def test_cli_refuses_an_unknown_reference(tmp_path):
    r = make_unit(tmp_path, "r", [True])
    with pytest.raises(SystemExit):
        gs.main(["--arm", "L-50", str(r), "--ref", "nope", "--out", str(tmp_path / "o")])
