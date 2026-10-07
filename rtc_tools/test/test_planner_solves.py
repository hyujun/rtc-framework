"""rtc_tools.analysis.planner_solves — solves that ended and solves that were cut, kept apart.

Every frame is built here; no recorded session is read.
"""

import math

import numpy as np
import pandas as pd
import pytest

from rtc_tools.analysis import planner_solves as ps


def _frame(rows):
    """``(kind, outcome, core_reason, solve_us[, extra])`` rows as a planner_events frame."""
    out = []
    for kind, outcome, reason, us, *extra in rows:
        row = {
            "segment_kind": kind,
            "segment_outcome": outcome,
            "segment_core_reason": reason,
            "segment_solve_us": us,
            "segment_iterations": 10,
        }
        row.update(extra[0] if extra else {})
        out.append(row)
    return pd.DataFrame(out)


class TestCutAtDeadline:
    def test_the_core_reason_decides_not_the_outcome(self):
        df = _frame(
            [
                ("first", "budget", "deadline", 35_050),  # cut
                ("first", "budget", "infeasible", 36_000),  # ended past the budget
                ("first", "budget", "none", 21_000),  # a core without a deadline
                ("first", "solve_failed", "infeasible", 20_000),
                ("first", "published", "converged", 9_000),
            ]
        )
        assert ps.cut_at_deadline(df).tolist() == [True, False, False, False, False]
        assert ps.solved_rows(df).all()

    def test_a_step_that_solved_nothing_is_neither(self):
        df = _frame(
            [
                ("first", "too_late", "none", 0),
                ("none", "off", "none", 0),
                # A refusal recorded with the deadline's name but no time is not a solve.
                ("first", "budget", "deadline", 0),
            ]
        )
        assert not ps.solved_rows(df).any()
        assert not ps.cut_at_deadline(df).any()

    def test_a_log_without_the_reason_column_has_no_cut_solve(self):
        df = _frame([("first", "budget", "deadline", 35_050)]).drop(columns="segment_core_reason")
        assert ps.solved_rows(df).tolist() == [True]
        assert ps.cut_at_deadline(df).tolist() == [False]
        assert not ps.solved_rows(pd.DataFrame({"wake_ns": [1, 2]})).any()


class TestQuantileWithBound:
    def test_nearest_rank_is_an_observed_value(self):
        v = list(range(1, 101))
        assert ps.nearest_rank(v, 0.99) == 99
        assert ps.nearest_rank(v, 0.5) == 50
        assert ps.nearest_rank([7.0], 0.99) == 7.0
        assert math.isnan(ps.nearest_rank([], 0.5))

    def test_exact_below_the_earliest_cut_and_a_bound_from_it_on(self):
        # 90 solves that ended in 1..9 ms, 10 cut at 35 ms.
        values = [float(1 + i % 9) for i in range(90)] + [35.0] * 10
        cut = [False] * 90 + [True] * 10
        p50, bound = ps.quantile_with_bound(values, cut, 0.5)
        assert (p50, bound) == (5.0, False)
        p99, bound = ps.quantile_with_bound(values, cut, 0.99)
        assert (p99, bound) == (35.0, True)

    def test_a_solve_that_ended_after_the_earliest_cut_is_still_a_bound(self):
        # One solve ended at 36 ms (past the budget), others were cut at 35 ms:
        # where the cut ones would have ended relative to it is unknown.
        values = [36.0, 35.0, 35.0, 1.0]
        cut = [False, True, True, False]
        assert ps.quantile_with_bound(values, cut, 1.0) == (36.0, True)
        assert ps.quantile_with_bound(values, cut, 0.25) == (1.0, False)

    def test_nothing_cut_is_never_a_bound(self):
        assert ps.quantile_with_bound([3.0, 1.0, 2.0], [False] * 3, 0.99) == (3.0, False)
        value, bound = ps.quantile_with_bound([], [], 0.5)
        assert math.isnan(value) and bound is False


class TestSummariseTimes:
    def test_the_two_kinds_are_reported_apart(self):
        s = ps.summarise_times([10.0, 20.0, 30.0, 35.1, 40.0], [False, False, False, True, True])
        assert (s["n"], s["n_done"], s["n_cut"]) == (5, 3, 2)
        assert s["done"] == {"min": 10.0, "p50": 20.0, "p90": 30.0, "p99": 30.0, "max": 30.0}
        assert s["cut_min_ms"] == 35.1 and s["cut_max_ms"] == 40.0

    def test_all_cut_has_no_distribution(self):
        s = ps.summarise_times([35.05, 39.0, 698.0], [True] * 3)
        assert s["done"] is None
        assert s["n_cut"] == 3 and s["cut_min_ms"] == 35.05
        assert s["p50"] == (39.0, True)

    def test_none_cut(self):
        s = ps.summarise_times([1.0, 2.0], [False, False])
        assert s["cut_min_ms"] is None and s["n_cut"] == 0
        assert s["p99"] == (2.0, False)


class TestSolveGroups:
    def test_groups_by_what_the_solve_was_and_how_it_ended(self):
        df = _frame(
            [
                ("first", "budget", "deadline", 35_050),
                ("first", "budget", "deadline", 39_000),
                ("first", "budget", "infeasible", 36_000),
                ("first", "solve_failed", "infeasible", 20_000),
                ("first", "too_late", "none", 0),
            ]
        )
        groups = ps.solve_groups(df)
        assert [(g["outcome"], g["core_reason"], g["n"]) for g in groups] == [
            ("budget", "deadline", 2),
            ("budget", "infeasible", 1),
            ("solve_failed", "infeasible", 1),
        ]
        cut = groups[0]["time_ms"]
        assert cut["done"] is None and cut["n_cut"] == 2 and cut["cut_min_ms"] == 35.05
        past = groups[1]["time_ms"]
        assert past["n_cut"] == 0 and past["done"]["max"] == 36.0
        # A log from before the docking columns: the counts it cannot give are None.
        assert groups[0]["qp_solves_p50"] is None
        assert groups[0]["qp_share_p50"] is None
        assert groups[0]["infeasible_group"] == ""
        assert groups[0]["violation"] == {}

    def test_the_docking_columns_split_and_fill_the_groups(self):
        def docking(group, lateral, qp_us):
            return {
                "segment_infeasible_group": group,
                "segment_qp_solves": 20,
                "segment_qp_iterations": 400,
                "segment_start_us": 100.0,
                "segment_linearize_us": 200.0,
                "segment_assemble_us": 100.0,
                "segment_qp_us": qp_us,
                "segment_merit_us": 100.0,
                "segment_viol_lateral": lateral,
                "segment_viol_torque": 0.0,
            }

        df = _frame(
            [
                ("first", "solve_failed", "infeasible", 20_000, docking("lateral", 0.02, 9500.0)),
                ("first", "solve_failed", "infeasible", 22_000, docking("lateral", 0.04, 9500.0)),
                ("first", "solve_failed", "infeasible", 21_000, docking("timing", 0.0, 4500.0)),
            ]
        )
        groups = ps.solve_groups(df)
        assert [(g["infeasible_group"], g["n"]) for g in groups] == [("lateral", 2), ("timing", 1)]
        lateral = groups[0]
        assert lateral["qp_solves_p50"] == 20
        assert lateral["qp_iterations_per_qp_p50"] == 20
        assert lateral["qp_share_p50"] == pytest.approx(0.95)
        # Only a group some solve violated is listed.
        assert lateral["violation"] == {"lateral": {"p50": pytest.approx(0.03), "max": 0.04}}
        assert groups[1]["violation"] == {}
        assert groups[1]["qp_share_p50"] == pytest.approx(0.9)

    def test_no_solve_no_group(self):
        assert ps.solve_groups(_frame([("none", "off", "none", 0)])) == []
        assert ps.solve_groups(pd.DataFrame({"wake_ns": [1]})) == []

    def test_text_never_prints_a_cut_time_as_a_solve_time(self):
        df = _frame(
            [
                ("first", "budget", "deadline", 35_050),
                ("first", "budget", "deadline", 39_000),
                ("first", "published", "converged", 9_000),
            ]
        )
        lines = ps.format_solve_groups(ps.solve_groups(df))
        cut_line = next(line for line in lines if " deadline " in line)
        # 0 solves ended, no distribution; the quantiles over both carry '>='.
        assert " - " in cut_line and ">=37" not in cut_line
        assert ">=35.05" in cut_line or ">=39.00" in cut_line
        assert "35.05" in cut_line  # the earliest cut instant
        assert any("lower bound" in line for line in lines)
        done_line = next(line for line in lines if " converged " in line)
        assert ">=" not in done_line and "9.00/9.00/9.00" in done_line
        assert ps.format_solve_groups([]) == ["  (no solve recorded)"]


class TestNlpSummary:
    def _nlp(self):
        return pd.DataFrame(
            {
                "nlp_ran": [1, 1, 1, 0],
                "nlp_reason": ["speed_window", "none", "deadline", "off"],
                "nlp_n_lattice": [18, 18, 16, 0],
                "nlp_n_screened": [0, 4, 3, 0],
                "nlp_n_solved": [0, 2, 3, 0],
                "nlp_n_valid": [0, 1, 0, 0],
                "nlp_rej_speed_window": [18, 10, 9, 0],
                "nlp_rej_deadline": [0, 0, 2, 0],
                "nlp_rej_hard_row": [0, 1, 1, 0],
                "nlp_solve_us_max": [0.0, 9000.0, 12000.0, float("nan")],
            }
        )

    def test_counts_only_the_wakes_the_search_ran_on(self):
        s = ps.nlp_summary(self._nlp())
        assert s["wakes"] == 3
        assert s["reasons"] == {"speed_window": 1, "none": 1, "deadline": 1}
        assert s["funnel"]["nlp_n_lattice"] == pytest.approx(52 / 3)
        assert s["rejects"] == {"speed_window": 37, "deadline": 2, "hard_row": 2}

    def test_a_wake_that_cut_a_candidate_has_a_cut_time(self):
        times = ps.nlp_summary(self._nlp())["solve_ms_max"]
        # The wake that solved nothing (0) is not a solve; of the two that did,
        # one cut a candidate.
        assert (times["n"], times["n_done"], times["n_cut"]) == (2, 1, 1)
        assert times["done"]["max"] == 9.0
        assert times["cut_min_ms"] == 12.0

    def test_none_without_the_columns_or_without_a_wake(self):
        assert ps.nlp_summary(pd.DataFrame({"wake_ns": [1]})) is None
        assert ps.nlp_summary(pd.DataFrame({"nlp_ran": [0, 0]})) is None


class TestReplaceSummary:
    def test_counts_where_the_attempts_ended(self):
        df = pd.DataFrame(
            {"replace_step": ["none", "too_late_followed", "too_late_followed", "published"]}
        )
        assert ps.replace_summary(df) == {"too_late_followed": 2, "published": 1}
        assert ps.replace_summary(pd.DataFrame({"replace_step": ["none"]})) == {}
        assert ps.replace_summary(pd.DataFrame({"wake_ns": [1]})) is None


def test_the_name_tables_match_the_cpp_headers():
    """The column suffixes this module reads are the C++ names, in enum order."""
    import re
    from pathlib import Path

    root = Path(__file__).resolve().parents[2]
    core = root / "rtc_controllers/src/catching/mpc_docking_segment_core.cpp"
    stats = root / "rtc_controllers/include/rtc_controllers/catching/search_stats.hpp"
    cycle = root / "rtc_controllers/include/rtc_controllers/catching/planner_cycle.hpp"
    csv = root / "integrated_bringup/include/integrated_bringup/logging/planner_events_csv.hpp"
    if not all(p.exists() for p in (core, stats, cycle, csv)):  # pragma: no cover
        pytest.skip("C++ sources not present")

    def names(text, function):
        body = text.split(function, 1)[1].split("\n}", 1)[0]
        return re.findall(r'return "([a-z_]+)";', body)

    groups = names(core.read_text(), "DockingRowGroupName(DockingRowGroup group) noexcept {")
    assert tuple(g for g in groups if g != "unknown") == ps.DOCKING_ROW_GROUPS
    rejects = names(stats.read_text(), "NlpRejectName(NlpReject r) noexcept {")
    rejects = [n for n in rejects if n != "unknown"]
    # none, the candidate reasons, then the three wake-only ones.
    assert tuple(rejects[1:-3]) == ps.NLP_REJECT_REASONS
    steps = names(cycle.read_text(), "ReplaceStepName(ReplaceStep s) noexcept {")
    assert tuple(s for s in steps if s != "unknown") == ps.REPLACE_STEPS
    header = csv.read_text()
    for group in ps.DOCKING_ROW_GROUPS:
        assert f"segment_viol_{group}" in header
    for group in ps.DOCKING_ELASTIC_GROUPS:
        assert f"segment_elastic_{group}" in header
    for stage in ps.SOLVE_STAGES:
        assert f"segment_{stage}_us" in header
    for reason in ps.NLP_REJECT_REASONS:
        assert f"nlp_rej_{reason}" in header
    assert np.all([f"segment_elastic_{g}" not in header for g in ps.DOCKING_ROW_GROUPS[7:]])
