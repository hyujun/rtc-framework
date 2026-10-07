"""catching_trials.plan_verdict_window — one throw's verdict and why it got no plan.

Every frame is built here; no recorded session is read.
"""

import re
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

from rtc_tools.analysis import catching_trials as ct


def _events(rows):
    """Wake rows (dicts) with the defaults of a wake that searched and found nothing."""
    base = {
        "outcome": "published",
        "decision": "no_current",
        "search_valid": 0,
        "plan_valid": 0,
        "plan_reason": 0,
        "segment_kind": "none",
        "segment_outcome": "off",
        "segment_core_reason": "none",
        "segment_solve_us": 0,
    }
    return pd.DataFrame([{**base, **r} for r in rows])


def _verdict(rows, t=None, window=(0.0, 10.0)):
    ev = _events(rows)
    t = np.arange(len(ev), dtype=float) if t is None else np.asarray(t, float)
    return ct.plan_verdict_window(t, ev["plan_valid"].to_numpy(float), ev, *window)


class TestVerdict:
    def test_published_when_any_wake_stored_a_valid_plan(self):
        v = _verdict(
            [
                {"plan_reason": 1},  # "no plan" published: uncertainty
                {"search_valid": 1, "plan_valid": 1},
                {"outcome": "held", "search_valid": 1, "decision": "held_hysteresis"},
            ]
        )
        assert v["plan_verdict"] == "published"
        assert v["plan_reject"] == "" and v["plan_reject_last"] == ""

    def test_withheld_names_the_first_segment_that_did_not_go_out(self):
        cut = {
            "outcome": "held",
            "search_valid": 1,
            "segment_kind": "first",
            "segment_outcome": "budget",
            "segment_core_reason": "deadline",
            "segment_solve_us": 35_050,
        }
        infeasible = {
            **cut,
            "segment_outcome": "solve_failed",
            "segment_core_reason": "infeasible",
            "segment_solve_us": 20_000,
        }
        v = _verdict([{"plan_reason": 1}, cut, cut, infeasible])
        assert v["plan_verdict"] == "withheld"
        # Of the wakes whose search FOUND a plan — the "no plan" wake is not one.
        assert v["plan_reject"] == "segment:budget/deadline"
        assert v["plan_reject_last"] == "segment:solve_failed/infeasible"
        assert v["first_solve_cut"] == 2

    def test_a_refusal_without_a_core_reason_is_the_outcome_alone(self):
        late = {
            "outcome": "held",
            "search_valid": 1,
            "segment_kind": "first",
            "segment_outcome": "too_late",
        }
        assert _verdict([late])["plan_reject"] == "segment:too_late"

    def test_no_plan_takes_the_search_reason(self):
        v = _verdict([{"plan_reason": 1}, {"plan_reason": 7}, {"plan_reason": 7}])
        assert v["plan_verdict"] == "no_plan"
        assert v["plan_reject"] == "search:stopping_distance"
        assert v["plan_reject_last"] == "search:stopping_distance"
        assert v["first_solve_cut"] == 0

    def test_the_nlp_reason_is_read_before_the_folded_plan_reason(self):
        # workspace and speed_window fold to different PlanReason codes; the
        # NLP search's own word is the one reported.
        v = _verdict(
            [
                {"plan_reason": 6, "nlp_reason": "speed_window"},
                {"plan_reason": 6, "nlp_reason": "speed_window"},
                {"plan_reason": 12, "nlp_reason": "deadline"},
            ]
        )
        assert v["plan_reject"] == "search:speed_window"
        assert v["plan_reject_last"] == "search:deadline"
        # `off` (no NLP search ran on that wake) falls back to the plan reason.
        assert (
            _verdict([{"plan_reason": 2, "nlp_reason": "off"}])["plan_reject"]
            == "search:ik_failed"
        )

    def test_ties_go_to_the_later_reason(self):
        v = _verdict([{"plan_reason": 1}, {"plan_reason": 7}])
        assert v["plan_reject"] == "search:stopping_distance"

    def test_a_found_plan_the_cycle_dropped_is_the_cycles_reason(self):
        v = _verdict([{"outcome": "superseded", "search_valid": 1}])
        assert v["plan_verdict"] == "withheld"
        assert v["plan_reject"] == "cycle:superseded"

    def test_no_search_when_no_wake_of_the_throw_searched(self):
        v = _verdict([{"outcome": "idle"}, {"outcome": "no_input"}])
        assert v == {
            "plan_verdict": "no_search",
            "plan_reject": "",
            "plan_reject_last": "",
            "first_solve_cut": 0,
        }

    def test_only_the_wakes_of_the_throw_count(self):
        rows = [{"search_valid": 1, "plan_valid": 1}, {"plan_reason": 1}, {"plan_reason": 1}]
        assert _verdict(rows, t=[0.0, 5.0, 6.0], window=(4.0, 7.0))["plan_verdict"] == "no_plan"
        assert _verdict(rows, t=[0.0, 5.0, 6.0], window=(8.0, 9.0))["plan_verdict"] == "no_search"

    def test_replacement_counts_only_on_a_log_that_has_the_column(self):
        assert "replace_attempts" not in _verdict([{"plan_reason": 1}])
        v = _verdict(
            [
                {"search_valid": 1, "plan_valid": 1, "replace_step": "none"},
                {"outcome": "held", "search_valid": 1, "replace_step": "too_late_followed"},
                {"search_valid": 1, "plan_valid": 1, "replace_step": "published"},
            ]
        )
        assert (v["replace_attempts"], v["replace_published"]) == (2, 1)

    def test_an_older_log_gives_what_its_columns_allow(self):
        # Before search_valid and the segment columns: a published valid plan
        # is the only "found" there is, and nothing can be said about solves.
        ev = pd.DataFrame({"outcome": ["published", "published"], "plan_valid": [0, 0]})
        v = ct.plan_verdict_window(
            np.array([0.0, 1.0]), ev["plan_valid"].to_numpy(float), ev, 0, 9
        )
        assert v["plan_verdict"] == "no_plan"
        assert "first_solve_cut" not in v and "replace_attempts" not in v


def test_plan_reason_names_match_the_cpp_enum():
    hdr = (
        Path(__file__).resolve().parents[2]
        / "rtc_controllers/include/rtc_controllers/catching/trajectory.hpp"
    )
    if not hdr.exists():  # pragma: no cover
        pytest.skip(f"C++ header not present at {hdr}")
    body = hdr.read_text().split("enum class PlanReason : std::uint8_t {", 1)[1].split("};", 1)[0]
    cpp = re.findall(r"^\s*k([A-Za-z]+)", body, flags=re.MULTILINE)
    snake = [re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower() for name in cpp]
    assert tuple(snake) == ct.PLAN_REASON_NAMES
