"""tools/catching_eval: the record of N arms (#746), on planted inputs.

``summarize.py --unit`` reads each unit's arm off the unit, joins the arms throw by
throw inside one repetition and describes the tables — it judges nothing. ``run_all.sh``
carries the arm's search and ball in a plan line and retries a rig failure only;
``mk_overlay.py`` states the sim's ball. Every input is built in the test.
"""

import os
import shutil
import subprocess
import sys
from pathlib import Path

import pandas as pd
import pytest
import yaml

TOOLS = Path(__file__).resolve().parents[1] / "tools" / "catching_eval"
sys.path.insert(0, str(TOOLS))
import mk_overlay  # noqa: E402
import summarize as sm  # noqa: E402

from rtc_tools.analysis import catching_decel as cd, catching_trials as ct  # noqa: E402

A = sm.Arm("grid", "closed_form", "tennis")
B = sm.Arm("grid", "mpc", "tennis")
C = sm.Arm("nlp", "mpc_docking", "tennis")
C2 = sm.Arm("nlp", "mpc_docking", "beanbag")
SHA1, SHA2 = "ab" * 32, "cd" * 32


# ── the arm is read off the unit ─────────────────────────────────────────────
def _unit(tmp_path, name, arm, *, expect=None, seed="3011", sha=SHA1, mirror_drop=()):
    unit = tmp_path / name
    unit.mkdir()
    mirror = {
        "planner.search.mode": f"String value is: {arm.search}",
        "planner.segment.mode": f"String value is: {arm.segment}",
        "ball_type": f"String value is: {arm.ball}",
    }
    (unit / "mirror.txt").write_text(
        "".join(f"{k}: {v}\n" for k, v in mirror.items() if k not in mirror_drop)
    )
    cond = {"robot": "ur5e_p1b", "overlay": "/x/o.yaml", "throws_file_sha256": sha}
    if seed is not None:
        cond["seed"] = seed
    cond.update(
        {"expect_search": arm.search, "expect_mode": arm.segment, "expect_ball": arm.ball}
        if expect is None
        else expect
    )
    (unit / "conditions.txt").write_text("".join(f"{k}: {v}\n" for k, v in cond.items()))
    return unit


def test_a_units_arm_is_its_mirror_and_its_repetition_its_seed_and_list(tmp_path):
    got = sm.unit_identity(_unit(tmp_path, "u", C2))
    assert got["arm"] == C2 and got["seed"] == "3011" and got["list"] == SHA1
    assert got["overlay"] == "/x/o.yaml" and got["robot"] == "ur5e_p1b"
    assert sm.arm_label(C2) == "nlp×mpc_docking·beanbag"


def test_a_unit_told_to_expect_another_arm_is_refused(tmp_path):
    # the positive control of reading the mirror: each field alone, wrong in conditions.txt
    for field, value in (
        ("expect_search", "grid"),
        ("expect_mode", "mpc"),
        ("expect_ball", "tennis"),
    ):
        told = {"expect_search": "nlp", "expect_mode": "mpc_docking", "expect_ball": "beanbag"}
        told[field] = value
        with pytest.raises(SystemExit, match=rf"{field}: {value}, the mirror says"):
            sm.unit_identity(_unit(tmp_path, f"u_{field}", C2, expect=told))


def test_a_unit_from_before_the_arm_lines_is_read_by_its_mirror_alone(tmp_path):
    old = _unit(tmp_path, "old", C, expect={"expect_mode": "mpc_docking"})
    assert sm.unit_identity(old)["arm"] == C


def test_a_unit_whose_mirror_or_seed_is_missing_is_refused(tmp_path):
    with pytest.raises(SystemExit, match="mirror.txt has no ball_type"):
        sm.unit_identity(_unit(tmp_path, "noball", C, mirror_drop=("ball_type",)))
    with pytest.raises(SystemExit, match="no seed"):
        sm.unit_identity(_unit(tmp_path, "noseed", C, seed=None))


def test_a_string_mirror_is_not_read_as_a_number():
    mirror = {"planner.search.mode": "String value is: nlp", "x": "Parameter not set"}
    assert sm.mirror_text(mirror, "planner.search.mode") == "nlp"
    assert sm.mirror_text(mirror, "x") is None and sm.mirror_text(mirror, "absent") is None


# ── which arms are tabled against each other ─────────────────────────────────
def test_the_pairs_differ_in_the_planner_or_in_the_ball_never_in_both():
    pairs = sm.record_pairs([C2, C, B, A])  # any order in
    assert [(a, b, what) for a, b, what in pairs] == [
        (A, B, "planner"),
        (A, C, "planner"),
        (B, C, "planner"),
        (C, C2, "ball"),
    ]
    everything = sm.record_pairs([A, B, C, C2], all_pairs=True)
    assert len(everything) == 6
    assert (A, C2, "planner+ball") in everything and (B, C2, "planner+ball") in everything


# ── a table, described ───────────────────────────────────────────────────────
def test_a_table_is_described_not_judged():
    tab = {"n": 180, "both": 4, "a_only": 12, "b_only": 3, "neither": 161}
    d = sm.describe(tab)
    assert (d["k_a"], d["k_b"]) == (16, 7)
    assert d["p_a"] == pytest.approx(16 / 180) and d["p_b"] == pytest.approx(7 / 180)
    assert d["wilson95_a"] == pytest.approx(ct.wilson_interval(16, 180))
    assert d["wilson95_b"] == pytest.approx(ct.wilson_interval(7, 180))
    assert d["diff"] == pytest.approx((3 - 12) / 180)  # b − a
    assert d["score_ci95"] == pytest.approx(cd.tango_score_ci(12, 3, 180))
    assert d["mcnemar_exact_p"] == pytest.approx(ct.mcnemar_exact(12, 3))
    # The Tango row is the G-1 verdict's own statistic on the same table — and here it
    # is a reference: the record carries no verdict of it.
    g1 = sm.test_block({"n": 180, "both": 4, "cf_only": 12, "mpc_only": 3, "neither": 161})
    assert d["tango_reference"]["z"] == pytest.approx(g1["z"])
    assert "reference" in d["tango_reference"]["note"]
    assert not {"noninferior", "verdict", "pass", "reject"} & set(d)
    assert not {"reject", "noninferior"} & set(d["tango_reference"])


def test_a_table_of_no_catch_at_all_is_described_and_an_empty_one_is_left_as_it_is():
    # D-1: nobody catches. Tango alone would read 0 vs 0 as "non-inferior" (Z 3.8) —
    # which is why it is a reference row beside the rates, not the record.
    d = sm.describe({"n": 130, "both": 0, "a_only": 0, "b_only": 0, "neither": 130})
    assert d["p_a"] == d["p_b"] == 0.0 and d["diff"] == 0.0
    assert d["wilson95_a"][0] == 0.0 and 0.02 < d["wilson95_a"][1] < 0.03
    assert d["tango_reference"]["z"] > 3.0
    empty = {"n": 0, "both": 0, "a_only": 0, "b_only": 0, "neither": 0}
    assert sm.describe(empty) == empty


def _row(sha, throw_id, ok, invalid="", d4=None):
    return {
        "kind": "lhs",
        "seed": None,
        "sample_idx": None,
        "throw_id": throw_id,
        "throws_file_sha256": sha,
        "invalid_reason": invalid,
        "truth": ok,
        "d4": ok if d4 is None else d4,
    }


def test_an_arms_rates_are_over_its_valid_throws():
    rows = [_row(SHA1, i, i < 3, d4=i < 2) for i in range(10)]
    rows.append(_row(SHA1, 10, True, invalid="lane_drop"))
    r = sm.rates(rows)
    assert (r["thrown"], r["valid"]) == (11, 10)
    assert (r["truth"]["k"], r["d4"]["k"]) == (3, 2)
    assert r["truth"]["wilson95"] == pytest.approx(ct.wilson_interval(3, 10))
    none = sm.rates([_row(SHA1, 0, True, invalid="sim_stall")])
    assert none["valid"] == 0 and none["truth"]["p"] is None


# ── units joined inside one repetition ───────────────────────────────────────
def _units(spec):
    """``{(seed, sha): (name, rows)}`` → the ``units`` and ``rows`` pair_record takes."""
    units = {key: {"name": name} for key, (name, _) in spec.items()}
    rows = dict(spec.values())
    return units, rows


def test_a_list_thrown_under_two_seeds_is_two_strata_tabled_apart_and_pooled():
    n = 6
    ua, ra = _units(
        {
            ("3011", SHA1): ("a_1", [_row(SHA1, i, i < 2) for i in range(n)]),
            ("3012", SHA1): ("a_2", [_row(SHA1, i, i < 3) for i in range(n)]),
            ("3011", SHA2): ("a_3", [_row(SHA2, i, False) for i in range(n)]),
        }
    )
    ub, rb = _units(
        {
            ("3011", SHA1): ("b_1", [_row(SHA1, i, i in (1, 4)) for i in range(n)]),
            ("3012", SHA1): ("b_2", [_row(SHA1, i, False) for i in range(n)]),
            ("3012", SHA2): ("b_4", [_row(SHA2, i, True) for i in range(n)]),
        }
    )
    rows = {**ra, **rb}
    # The positive control: one join of both seeds meets every throw twice.
    with pytest.raises(SystemExit, match="twice in one arm"):
        sm.pair_rows(rows["a_1"] + rows["a_2"], rows["b_1"] + rows["b_2"])
    rec = sm.pair_record(ua, ub, rows)
    assert rec["strata"] == 2
    assert rec["strata_only_a"] == ["seed 3011 · list:cdcdcdcdcdcd"]
    assert rec["strata_only_b"] == ["seed 3012 · list:cdcdcdcdcdcd"]
    s1 = rec["by_stratum"]["seed 3011 · list:abababababab"]
    assert s1["units"] == ["a_1", "b_1"] and s1["pairs"]["keys"] == n
    t = s1["truth"]
    assert (t["both"], t["a_only"], t["b_only"], t["neither"]) == (1, 1, 1, 3)
    t = rec["by_stratum"]["seed 3012 · list:abababababab"]["truth"]
    assert (t["both"], t["a_only"], t["b_only"], t["neither"]) == (0, 3, 0, 3)
    pooled = rec["pooled"]["truth"]
    assert pooled["n"] == 2 * n  # a throw counts once per repetition
    assert (pooled["both"], pooled["a_only"], pooled["b_only"]) == (1, 4, 1)
    assert rec["by_list"].keys() == {"list:abababababab"}
    assert rec["by_list"]["list:abababababab"]["truth"]["n"] == 2 * n


def test_an_invalid_throw_drops_its_pair_in_its_own_stratum_only():
    ua, ra = _units(
        {
            ("3011", SHA1): (
                "a_1",
                [_row(SHA1, i, True, invalid="lane_drop" if i == 0 else "") for i in range(4)],
            ),
            ("3012", SHA1): ("a_2", [_row(SHA1, i, True) for i in range(4)]),
        }
    )
    ub, rb = _units(
        {
            ("3011", SHA1): ("b_1", [_row(SHA1, i, True) for i in range(4)]),
            ("3012", SHA1): ("b_2", [_row(SHA1, i, True) for i in range(4)]),
        }
    )
    rec = sm.pair_record(ua, ub, {**ra, **rb})
    first = rec["by_stratum"]["seed 3011 · list:abababababab"]
    assert first["pairs"]["dropped"] == {"a_only/lane_drop/-": 1} and first["truth"]["n"] == 3
    assert rec["by_stratum"]["seed 3012 · list:abababababab"]["truth"]["n"] == 4
    assert rec["pooled"]["truth"]["n"] == 7


def test_an_arm_against_itself_shows_what_a_repetition_moves():
    units, rows = _units(
        {
            ("3011", SHA1): ("c_1", [_row(SHA1, i, i in (0, 1, 2)) for i in range(8)]),
            ("3012", SHA1): ("c_2", [_row(SHA1, i, i in (2, 3)) for i in range(8)]),
            ("3011", SHA2): (
                "c_3",
                [_row(SHA2, i, True) for i in range(8)],
            ),  # one seed: no repeat
        }
    )
    rep = sm.repeat_record(units, rows)
    assert rep.keys() == {"list:abababababab · seed 3011 (a) vs 3012 (b)"}
    t = rep["list:abababababab · seed 3011 (a) vs 3012 (b)"]["truth"]
    assert (t["both"], t["a_only"], t["b_only"], t["neither"]) == (1, 2, 1, 4)


# ── what the planner did, per arm ────────────────────────────────────────────
def _ct(rows):
    return pd.DataFrame(rows)


def test_the_plan_block_counts_verdicts_over_valid_throws_and_catches_under_each():
    u = {
        "ct": _ct(
            [
                {"invalid_reason": "", "plan_verdict": "published", "plan_reject": "",
                 "search_valid_cycles": 3, "first_solve_cut": 0, "truth_success": True},
                {"invalid_reason": "", "plan_verdict": "published", "plan_reject": "",
                 "search_valid_cycles": 1, "first_solve_cut": 1, "truth_success": False},
                {"invalid_reason": "", "plan_verdict": "withheld", "plan_reject": "segment:catch_error",
                 "search_valid_cycles": 2, "first_solve_cut": 0, "truth_success": False},
                {"invalid_reason": float("nan"), "plan_verdict": "no_plan", "plan_reject": "search:too_far",
                 "search_valid_cycles": 0, "first_solve_cut": 0, "truth_success": "False"},
                {"invalid_reason": "sim_stall", "plan_verdict": "published", "plan_reject": "",
                 "search_valid_cycles": 9, "first_solve_cut": 9, "truth_success": True},
            ]
        )
    }  # fmt: skip
    p = sm.arm_plan([u])
    assert p["valid"] == 4
    assert p["plan_verdict"] == {"published": 2, "withheld": 1, "no_plan": 1}
    assert p["reject_most_frequent"] == {"segment:catch_error": 1, "search:too_far": 1}
    assert p["search_accepted"] == 3 and p["throws_with_a_first_solve_cut"] == 1
    assert p["caught_by_plan_verdict"] == {"published": 1, "withheld": 0, "no_plan": 0}
    assert "reject_last" not in p  # a column the tool did not write is left out


def test_the_entrance_block_measures_from_rho_ref_on_published_throws():
    def throw(x, y, caught, verdict="published", margin=-1.0):
        return {"invalid_reason": "", "plan_verdict": verdict, "ent_x_mm": x, "ent_y_mm": y,
                "ent_lateral_margin_mm": margin, "truth_success": caught, "cf_tot_s_mm": 20.0}  # fmt: skip

    u = {
        "summ": {"catch_frame": {"rho_ref_mm": [-5.0, -5.0]}},
        "ct": _ct(
            [
                throw(-5.0, 1.0, True, margin=2.0),  # 6 mm from rho_ref
                throw(0.0, 8.0, False),  # 8 mm from the origin, 13.9 mm from rho_ref
                throw(float("nan"), float("nan"), False),  # never crossed
                throw(-5.0, -5.0, True, verdict="withheld"),  # not published: left out
            ]
        ),
    }
    e = sm.arm_catch_frame([u])
    assert (e["published"], e["caught"], e["crossed_entrance"]) == (3, 1, 2)
    assert (e["within_near_mm_of_rho_ref"], e["caught_of_those"]) == (1, 1)
    assert e["inside_lateral_set"] == 1
    assert e["medians"]["ent_lat_mm"] == pytest.approx((6.0 + (5.0**2 + 13.0**2) ** 0.5) / 2)
    # a hand without a docking set says nothing here, and neither does an older tool
    assert sm.arm_catch_frame([{"summ": {}, "ct": u["ct"]}])["crossed_entrance"] == 0
    bare = _ct([{"invalid_reason": "", "truth_success": True}])
    assert sm.arm_catch_frame([{"summ": {}, "ct": bare}]) is None


def _event(kind, outcome, reason, us, **extra):
    return {"n_in_window": 4, "search_us": 900, "budget_hit": 0, "segment_kind": kind,
            "segment_outcome": outcome, "segment_core_reason": reason, "segment_solve_us": us,
            **extra}  # fmt: skip


def _solve_unit(events, **mirror):
    return {
        "pe": pd.DataFrame(events),
        "ptiming": None,
        "cond": {"planner_budget_s": "0.040"},
        "mirror": {k: f"Double value is: {v}" for k, v in mirror.items()},
    }


_BUDGETS = {
    "planner.segment.mpc.budget.first_s": 0.2,
    "planner.segment.mpc.budget.replan_s": 0.1,
    "planner.segment.mpc_docking.budget.first_s": 0.035,
    "planner.segment.mpc_docking.budget.replan_s": 0.025,
}


def test_a_solve_cut_at_the_deadline_is_a_count_and_a_share_never_a_time():
    events = [_event("first", "published", "converged", us) for us in (10_000, 12_000, 30_000)]
    events += [_event("first", "budget", "deadline", 35_000) for _ in range(5)]
    events += [_event("same", "published", "converged", 4_000)]
    events += [_event("none", "up_to_date", "none", 0)]
    s = sm.arm_solves([_solve_unit(events, **_BUDGETS)], C)
    first = s["segment"]["first"]
    assert (first["n"], first["n_done"], first["n_cut"]) == (8, 3, 5)
    assert first["cut_share"] == pytest.approx(5 / 8)
    # the ended solves alone: the 35 ms of a cut solve is in no quantile of them
    assert first["done"]["max"] == 30.0 and first["done"]["p99"] == 30.0
    assert first["p99"] == (35.0, True)  # over both kinds: a lower bound, flagged
    assert first["budget_ms"] == 35.0 and first["done_p99_within_budget"] is True
    assert first["outcomes"] == {"budget": 5, "published": 3}
    assert s["segment"]["replan"]["budget_ms"] == 25.0
    assert "repl_first" not in s["segment"] and "nlp" not in s


def test_an_arms_solves_are_held_against_its_own_planners_budget():
    events = [_event("first", "published", "converged", 30_000)]
    unit = _solve_unit(events, **_BUDGETS)
    # the same solves under the two segment planners: 30 ms is inside 200 ms, not 25 ms
    assert sm.arm_solves([unit], B)["segment"]["first"]["budget_ms"] == 200.0
    docking = sm.arm_solves([unit], C)["segment"]["first"]
    assert docking["budget_ms"] == 35.0 and docking["done_p99_within_budget"] is True
    tight = dict(_BUDGETS)
    tight["planner.segment.mpc_docking.budget.first_s"] = 0.025
    over = sm.arm_solves([_solve_unit(events, **tight)], C)["segment"]["first"]
    assert over["done_p99_within_budget"] is False
    # the closed-form law has no segment planner: no budget to hold a solve against
    cf = sm.arm_solves([unit], A)["segment"]["first"]
    assert cf["budget_ms"] is None and cf["done_p99_within_budget"] is None
    with pytest.raises(SystemExit, match="differs across the units of one arm"):
        sm.arm_solves([unit, _solve_unit(events, **tight)], C)


def test_a_withheld_replacements_first_solve_runs_under_the_first_budget():
    event = _event("same", "solve_failed", "infeasible", 5_000,
                   replacement_solve_us=36_000, replacement_core_reason="deadline",
                   replacement_outcome="budget")  # fmt: skip
    s = sm.arm_solves([_solve_unit([event], **_BUDGETS)], C)
    repl = s["segment"]["repl_first"]
    assert (repl["n"], repl["n_cut"], repl["done"]) == (1, 1, None)
    assert repl["budget_ms"] == 35.0 and repl["done_p99_within_budget"] is None


def test_the_nlp_searchs_wakes_are_counted_with_their_budget():
    events = [
        _event("first", "published", "converged", 20, nlp_ran=1, nlp_reason="none",
               nlp_solve_us_max=15_000, nlp_rej_deadline=0),
        _event("none", "no_ball", "none", 0, nlp_ran=1, nlp_reason="deadline",
               nlp_solve_us_max=24_000, nlp_rej_deadline=1),
        _event("none", "no_ball", "none", 0, nlp_ran=0, nlp_reason="",
               nlp_solve_us_max=0, nlp_rej_deadline=0),
    ]  # fmt: skip
    nlp_budget = {
        "planner.search.nlp.budget.budget_s": 0.04,
        "planner.search.nlp.budget.solve_s": 0.024,
        "planner.search.nlp.budget.max_solves": 1,
    }
    nlp = sm.arm_solves([_solve_unit(events, **_BUDGETS, **nlp_budget)], C)["nlp"]
    assert nlp["wakes"] == 2 and nlp["rejects"] == {"deadline": 1}
    assert nlp["budget"] == {"wake_s": 0.04, "solve_s": 0.024, "max_solves": 1.0}
    assert nlp["wakes_with_a_deadline_reject_share"] == 0.5
    # a unit recorded before the budget was mirrored says None, not a number
    before = _solve_unit(events, **_BUDGETS)
    old = sm.arm_solves([before], C)["nlp"]
    assert old["budget"] == {"wake_s": None, "solve_s": None, "max_solves": None}
    # … and beside a unit that did record it, the arm has the recorded value: the
    # finished unit cannot be mirrored again, so it must not stop the record
    after = _solve_unit(events, **_BUDGETS, **nlp_budget)
    assert sm.arm_solves([before, after], C)["nlp"]["budget"]["solve_s"] == 0.024
    other = _solve_unit(
        events, **_BUDGETS, **{**nlp_budget, "planner.search.nlp.budget.solve_s": 0.018}
    )
    with pytest.raises(SystemExit, match=r"nlp\.budget\.solve_s differs"):
        sm.arm_solves([before, after, other], C)
    # a segment budget is in every unit's mirror: one that lacks it is not waved through
    lacking = _solve_unit(
        events, **{k: v for k, v in _BUDGETS.items() if "docking.budget.first" not in k}
    )
    with pytest.raises(SystemExit, match="differs across the units of one arm"):
        sm.arm_solves([after, lacking], C)


def _arm_block(arm, units, rows):
    """What ``arm_record`` assembles for one arm, from planted units: the blocks that
    read a real session (FK, limits of a state log, the RT tick) are left out of it."""
    loaded = [units[k] for k in sorted(units)]
    arm_rows = [r for u in loaded for r in rows[u["name"]]]
    return {
        "search": arm.search,
        "segment": arm.segment,
        "ball": arm.ball,
        "units": {f"seed {k[0]} · {sm.list_tag(k[1])}": units[k]["name"] for k in sorted(units)},
        "rates": {
            "all": sm.rates(arm_rows),
            "by_list": {sm.list_tag(SHA1): sm.rates(arm_rows)},
            "by_unit": {u["name"]: sm.rates(rows[u["name"]]) for u in loaded},
        },
        "plan": sm.arm_plan(loaded),
        "catch_frame": sm.arm_catch_frame(loaded),
        "solves": sm.arm_solves(loaded, arm),
        "limits": sm.limits_block(arm_rows),
        "failure_modes": sm.failure_modes(arm_rows),
        "repeat": sm.repeat_record(units, rows),
    }


def test_a_record_prints_and_dumps_and_carries_no_verdict(tmp_path):
    import json

    def unit(name, caught):
        frame = _ct(
            [
                {"invalid_reason": "", "plan_verdict": "published", "plan_reject": "",
                 "ent_x_mm": -5.0, "ent_y_mm": 1.0, "ent_lateral_margin_mm": 1.0,
                 "truth_success": i in caught}
                for i in range(6)
            ]
        )  # fmt: skip
        events = [_event("first", "published", "converged", 9_000)]
        return {"name": name, "ct": frame, "summ": {"catch_frame": {"rho_ref_mm": [-5.0, -5.0]}},
                **_solve_unit(events, **_BUDGETS)}  # fmt: skip

    def arm_units(tag, caught_by_seed):
        units = {
            (seed, SHA1): unit(f"{tag}_{seed}", caught) for seed, caught in caught_by_seed.items()
        }
        rows = {
            u["name"]: [_row(SHA1, i, i in caught_by_seed[seed]) for i in range(6)]
            for (seed, _), u in units.items()
        }
        return units, rows

    ub, rb = arm_units("b", {"3011": {0, 1}, "3012": {1}})
    uc, rc = arm_units("c", {"3011": {1, 2, 3}})  # one repetition only
    rows = {**rb, **rc}
    res = {
        "record": "planted",
        "robot": "ur5e_p1b",
        "arms": {sm.arm_label(B): _arm_block(B, ub, rows), sm.arm_label(C): _arm_block(C, uc, rows)},
        "pairs": {
            "b | c": {"a": sm.arm_label(B), "b": sm.arm_label(C), "differs_in": "planner",
                      **sm.pair_record(ub, uc, rows)}
        },
        "mode_log_mismatch": [],
        "regression": "unknown",
    }  # fmt: skip
    text = "\n".join(sm.format_record(res))
    # b caught throws 0 and 1, c caught 1, 2 and 3: one both, one b-arm only, two c-arm only
    assert "truth: n 6 · both 1 · a only 1 · b only 2 · neither 2" in text
    assert "thrown by one arm only a 0 b 0" in text
    assert "strata of one arm only: a ['seed 3012 · list:abababababab'] · b []" in text
    assert "repeat list:abababababab · seed 3011 (a) vs 3012 (b):" in text
    assert "arm grid×mpc·tennis: 2 units · thrown 12 · valid 12" in text
    assert "segment first: 2 solves" in text and "budget 200.0 ms" in text
    assert "— reference)" in text
    dumped = json.loads(json.dumps(res, default=sm._json))
    assert dumped["pairs"]["b | c"]["pooled"]["truth"]["b_only"] == 2

    # nothing in a record is a verdict: no key says pass / fail / noninferior anywhere
    def keys(node):
        if isinstance(node, dict):
            for k, v in node.items():
                yield str(k)
                yield from keys(v)
        elif isinstance(node, list):
            for v in node:
                yield from keys(v)

    assert not {"pass", "fail", "verdict", "noninferior", "reject"} & set(keys(dumped))
    assert dumped["arms"]["grid×mpc·tennis"]["solves"]["search"]["p99_within_budget"] is True


# ── the two modes of the command line ────────────────────────────────────────
def test_the_record_and_the_verdict_do_not_mix_on_one_command_line(tmp_path):
    run = [sys.executable, str(TOOLS / "summarize.py"), "--cfg", str(tmp_path)]
    both = subprocess.run([*run, "--unit", "u1", "--cf", "u2"], capture_output=True, text=True)
    assert both.returncode == 2 and "takes no --cf" in both.stderr
    half = subprocess.run([*run, "--cf", "u1", "--mpc", "u2"], capture_output=True, text=True)
    assert half.returncode == 2 and "(missing: --overlay-cf --overlay-mpc)" in half.stderr
    # --all-pairs belongs to the record: with every flag of the verdict present it is
    # still refused, and nothing is named missing
    verdict = [
        *run,
        "--cf",
        "u1",
        "--mpc",
        "u2",
        "--overlay-cf",
        "a.yaml",
        "--overlay-mpc",
        "b.yaml",
    ]
    stray = subprocess.run([*verdict, "--all-pairs"], capture_output=True, text=True)
    assert stray.returncode == 2 and "(missing: -)" in stray.stderr


def test_units_of_one_arm_seed_and_list_twice_are_refused(tmp_path):
    for name in ("c_1", "c_1_again"):
        _unit(tmp_path, name, C)
    with pytest.raises(SystemExit, match="one unit per arm, seed and list"):
        sm.record(tmp_path, [str(tmp_path / "c_1"), str(tmp_path / "c_1_again")])
    other = _unit(tmp_path, "leap", C, seed="3111")
    (other / "conditions.txt").write_text(
        (other / "conditions.txt").read_text().replace("ur5e_p1b", "iiwa7_leap")
    )
    with pytest.raises(SystemExit, match="one robot's"):
        sm.record(tmp_path, [str(tmp_path / "c_1"), str(other)])
    gone = _unit(tmp_path, "c_2", C, seed="3012")
    with pytest.raises(SystemExit, match=r"c_2: the overlay its conditions.txt names is gone"):
        sm.record(tmp_path, [str(gone)])


# ── run_all.sh: the arm in a plan line, and which failures are retried ───────
_STUB = """#!/bin/bash
OUT=$1; NAME=$(basename "$OUT"); mkdir -p "$OUT"
echo "$NAME|$2|$3|$5|$ARM|$EXPECT_MODE|$EXPECT_SEARCH|$EXPECT_BALL|$EXPECT_KV" >> "$DATA/calls.log"
N=$(grep -c "^$NAME|" "$DATA/calls.log")
sed -n "${N}p" "$DATA/script_$NAME" > "$OUT/status"
"""


def _run_all(tmp_path, plan, scripts, **env):
    tools = tmp_path / "tools"
    tools.mkdir()
    shutil.copy(TOOLS / "run_all.sh", tools / "run_all.sh")
    (tools / "run_unit.sh").write_text(_STUB)
    (tools / "run_unit.sh").chmod(0o755)
    data = tmp_path / "data"
    data.mkdir()
    for name, statuses in scripts.items():
        (data / f"script_{name}").write_text("".join(s + "\n" for s in statuses))
    (tmp_path / "plan.txt").write_text(plan)
    full = {**os.environ, "DATA": str(data), "PLAN": str(tmp_path / "plan.txt"),
            "IDLE_GRACE": "0", "IDLE_MAX_S": "0", **env}  # fmt: skip
    for name in ("EXPECT_SEARCH", "EXPECT_BALL", "ONLY", "MAX_TRY", "RETRY_ON"):
        if name not in env:
            full.pop(name, None)
    done = subprocess.run(
        ["bash", str(tools / "run_all.sh")], env=full, capture_output=True, text=True
    )
    assert done.returncode == 0, done.stderr
    calls = [line.split("|") for line in (data / "calls.log").read_text().splitlines()]
    return data, calls


def test_a_plan_line_carries_the_arms_search_and_ball(tmp_path):
    plan = (
        "# a comment\n"
        "c2 p1b /o/nd_bb.yaml 1 3011 mpc_docking - nlp beanbag\n"
        "b p1b /o/gm.yaml 1 3011 mpc planner.freeze.T_freeze=0.39 grid\n"
        "a p1b /o/cf.yaml 1 3011 closed_form\n"
    )
    _, calls = _run_all(tmp_path, plan, {n: ["DONE"] for n in ("c2", "b", "a")})
    assert calls == [
        ["c2", "p1b", "/o/nd_bb.yaml", "3011", "mpc_docking", "mpc_docking", "nlp", "beanbag", ""],
        ["b", "p1b", "/o/gm.yaml", "3011", "mpc", "mpc", "grid", "tennis",
         "planner.freeze.T_freeze=0.39"],
        ["a", "p1b", "/o/cf.yaml", "3011", "closed_form", "closed_form", "grid", "tennis", ""],
    ]  # fmt: skip


def test_a_plan_line_without_the_arm_columns_takes_the_callers_as_before(tmp_path):
    plan = "old p1b /o/nd.yaml 1 2081 mpc_docking\nnew p1b /o/cf.yaml 1 2081 closed_form - grid\n"
    scripts = {"old": ["DONE"], "new": ["DONE"]}
    _, calls = _run_all(tmp_path, plan, scripts, EXPECT_SEARCH="nlp", EXPECT_BALL="beanbag")
    assert [c[6:8] for c in calls] == [["nlp", "beanbag"], ["grid", "beanbag"]]


def test_only_a_rig_failure_is_retried(tmp_path):
    plan = "".join(
        f"{n} p1b /o.yaml 1 3011 mpc\n" for n in ("busy", "est", "start", "ovl", "rc", "stuck")
    )
    scripts = {
        "busy": ["FAIL:host_busy", "DONE"],
        "est": ["FAIL:estimator not activated", "DONE"],
        "start": ["FAIL:startup", "FAIL:startup", "DONE"],
        "ovl": ["FAIL:overlay mode_mirror", "DONE"],  # the DONE is never reached
        "rc": ["FAIL:trials rc=124", "DONE"],
        "stuck": ["FAIL:host_busy"] * 5,
    }
    data, calls = _run_all(tmp_path, plan, scripts)
    tries = {n: sum(1 for c in calls if c[0] == n) for n in scripts}
    assert tries == {"busy": 2, "est": 2, "start": 3, "ovl": 1, "rc": 1, "stuck": 3}
    status = {n: (data / n / "status").read_text().strip() for n in scripts}
    assert status == {
        "busy": "DONE",
        "est": "DONE",
        "start": "DONE",
        "ovl": "FAIL:overlay mode_mirror",
        "rc": "FAIL:trials rc=124",
        "stuck": "FAIL:host_busy",
    }
    # a retried attempt is kept beside the unit; an unretried failure stays where it failed
    assert (data / "busy.fail1" / "status").read_text().strip() == "FAIL:host_busy"
    assert (data / "start.fail2").is_dir() and not (data / "ovl.fail1").exists()
    log = (data / "progress.log").read_text()
    assert "ovl not retried: 'FAIL:overlay mode_mirror' is not a rig failure" in log
    assert "rc not retried" in log and "busy not retried" not in log
    # the batch went on after the failure it did not retry
    assert [c[0] for c in calls].index("rc") > [c[0] for c in calls].index("ovl")


def test_a_relaunch_runs_a_failed_unit_anew_and_skips_a_done_one(tmp_path):
    plan = "ovl p1b /o.yaml 1 3011 mpc\nok p1b /o.yaml 1 3011 mpc\n"
    data, calls = _run_all(tmp_path, plan, {"ovl": ["FAIL:overlay x", "DONE"], "ok": ["DONE"]})
    assert [c[0] for c in calls] == ["ovl", "ok"]
    env = {**os.environ, "DATA": str(data), "PLAN": str(tmp_path / "plan.txt"),
           "IDLE_GRACE": "0", "IDLE_MAX_S": "0"}  # fmt: skip
    subprocess.run(["bash", str(tmp_path / "tools" / "run_all.sh")], env=env, check=True)
    calls = [line.split("|")[0] for line in (data / "calls.log").read_text().splitlines()]
    assert calls == ["ovl", "ok", "ovl"]
    assert (data / "ovl" / "status").read_text().strip() == "DONE"
    assert (data / "ovl.fail1" / "status").read_text().strip() == "FAIL:overlay x"


# ── mk_overlay.py: the ball; run_unit.sh: the arm in conditions.txt ──────────
def test_mk_overlay_states_the_sims_ball_only_when_given(tmp_path):
    out = tmp_path / "bb.yaml"
    mk_overlay.main([str(out), "p1b", "mpc_docking", "nlp", "beanbag"])
    doc = yaml.safe_load(out.read_text())
    assert doc["mujoco_simulator"]["ros__parameters"]["projectile_ball"] == {
        "ball_type": "beanbag"
    }
    planner = doc["integrated_rt_controller"]["ros__parameters"]["demo_catching_controller"][
        "catching"
    ]["planner"]
    assert (planner["search"]["mode"], planner["segment"]["mode"]) == ("nlp", "mpc_docking")
    assert "projectile_ball.ball_type" in out.read_text().splitlines()[0]
    plain = tmp_path / "plain.yaml"
    mk_overlay.main([str(plain), "leap", "mpc_docking", "nlp"])
    assert "mujoco_simulator" not in yaml.safe_load(plain.read_text())
    tennis = tmp_path / "tennis.yaml"
    mk_overlay.main([str(tennis), "leap", "closed_form", "grid", "tennis"])
    said = yaml.safe_load(tennis.read_text())["mujoco_simulator"]["ros__parameters"]
    assert said["projectile_ball"]["ball_type"] == "tennis"


def test_mk_overlay_refuses_a_ball_the_simulator_does_not_have(tmp_path):
    out = tmp_path / "o.yaml"
    with pytest.raises(SystemExit):
        mk_overlay.main([str(out), "p1b", "mpc", "grid", "golf"])
    with pytest.raises(SystemExit):
        mk_overlay.main([str(out), "p1b", "mpc", "grid", "beanbag", "extra"])
    assert not out.exists()


def test_run_unit_records_the_arm_it_was_told_to_expect():
    text = (TOOLS / "run_unit.sh").read_text()
    block = text[text.index('echo "date_start:') : text.index('} > "$OUT/conditions.txt"')]
    assert 'echo "expect_search: ${EXPECT_SEARCH:-grid}"' in block
    assert 'echo "expect_ball: $EXPECT_BALL"' in block
    # EXPECT_BALL has its default before the block that writes it
    assert text.index("EXPECT_BALL=${EXPECT_BALL:-tennis}") < text.index('echo "date_start:')
    for key in ("budget_s", "solve_s", "max_solves"):
        assert f"planner.search.nlp.budget.{key}" in text
