"""tools/catching_eval/unit_report.py: one unit's report, on a unit planted in a temp directory.

Every input is built in the test — no recorded unit is read.
"""

import json
import sys
from pathlib import Path

import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "catching_eval"))
import unit_report as ur  # noqa: E402

CTRL = "demo_catching_controller"


def _plant(tmp_path, *, events, diag=None, timing=None, trials=None, status="DONE"):
    unit = tmp_path / "u1"
    ctl = unit / "session" / "controllers" / CTRL
    ctl.mkdir(parents=True)
    (unit / "status").write_text(status + "\n")
    pd.DataFrame(events).to_csv(ctl / "planner_events.csv", index=False)
    if diag is not None:
        pd.DataFrame(diag).to_csv(ctl / "catching_diag.csv", index=False)
    if timing is not None:
        (unit / "session" / "timing").mkdir(parents=True)
        pd.DataFrame(timing).to_csv(unit / "session" / "timing" / "cm_timing_log.csv", index=False)
    if trials is not None:
        (unit / "trials").mkdir()
        (unit / "trials" / "trial_results.json").write_text(json.dumps(trials))
    return unit


def _solve(kind, segment_outcome, reason, us, **extra):
    return {
        "outcome": "held",
        "decision": "no_current",
        "segment_kind": kind,
        "segment_outcome": segment_outcome,
        "segment_core_reason": reason,
        "segment_solve_us": us,
        "segment_iterations": 12,
        "replace_step": "none",
        **extra,
    }


def test_throws_counts_the_modes_each_mode_log_names():
    trials = [
        {"mode_log": [[0.0, "TRACKING"], [0.4, "APPROACH"], [0.9, "HOLD"]], "outcome": "CAPTURED"},
        {"mode_log": "[(0.0, 'TRACKING'), (0.5, 'ABORT_SAFE')]", "final_outcome": "ABORTED"},
        {"mode_log": None},
    ]
    assert ur.throws(trials) == {
        "n": 3,
        "hold": 1,
        "abort_safe": 1,
        "outcomes": {"CAPTURED": 1, "ABORTED": 1, "?": 1},
    }
    assert ur.throws({"trials": trials})["n"] == 3


def test_a_cut_solve_is_counted_and_never_given_a_solve_time(tmp_path, capsys):
    events = [
        _solve("first", "budget", "deadline", 35_050),
        _solve("first", "budget", "deadline", 41_000),
        _solve("first", "budget", "infeasible", 36_000),
        _solve("first", "published", "converged", 9_000, outcome="published"),
        _solve("none", "off", "none", 0, replace_step="too_late_followed"),
    ]
    unit = _plant(tmp_path, events=events)
    r = ur.report(unit)
    assert r["status"] == "DONE"
    by = {(g["outcome"], g["core_reason"]): g for g in r["planner"]["solves"]}
    cut = by[("budget", "deadline")]["time_ms"]
    assert cut["n_cut"] == 2 and cut["done"] is None and cut["cut_min_ms"] == 35.05
    past = by[("budget", "infeasible")]["time_ms"]
    assert past["n_cut"] == 0 and past["done"]["max"] == 36.0
    assert r["planner"]["replace"] == {"too_late_followed": 1}
    assert r["planner"]["outcome"] == {"held": 4, "published": 1}
    assert r["planner"]["nlp"] is None
    assert "throws" not in r and "lane_events" not in r

    assert ur.main([str(unit), "--json", str(tmp_path / "out.json")]) == 0
    text = capsys.readouterr().out
    assert "== u1 (DONE)" in text
    assert "replacement attempts ended: too_late_followed 1" in text
    cut_line = next(line for line in text.splitlines() if " deadline " in line)
    # A count and the earliest cut instant — no distribution of cut times.
    assert cut_line.split()[-5:] == ["0", "-", "2", "35.05", "-"]
    written = json.loads((tmp_path / "out.json").read_text())
    assert written[0]["unit"] == "u1"
    assert written[0]["planner"]["solves"][0]["time_ms"]["p50"] == [35.05, True]


def test_lane_events_and_the_law_ticks_compute_time(tmp_path):
    diag = {
        "tick": [1, 2, 3, 4, 5, 6],
        "mode_name": ["tracking", "approach", "approach", "committed", "hold", "retreat"],
        "segment_event": [0, 11, 12, 4, 0, 5],
    }
    timing = {"tick_count": [1, 2, 3, 4, 5, 6, 7], "t_compute_us": [900, 10, 20, 30, 40, 800, 700]}
    unit = _plant(tmp_path, events=[_solve("none", "off", "none", 0)], diag=diag, timing=timing)
    r = ur.report(unit)
    events = r["lane_events"]
    assert events["pair_admitted"] == 1 and events["plan_switched"] == 1
    assert sum(events.values()) == 4
    # Ticks 2-5 are in a mode that runs the segment law; 1, 6 and 7 are not.
    assert r["rt_tick_us"] == {
        "n": 4,
        "law_ticks": 4,
        "p50": 20.0,
        "p99": 40.0,
        "p999": 40.0,
        "max": 40.0,
    }
    text = "\n".join(ur.format_report(r))
    assert "RT tick t_compute_us on law ticks (n=4)" in text
    assert "(no solve recorded)" in text


def test_the_nlp_wakes_are_reported_when_the_log_has_them(tmp_path):
    events = [
        {
            **_solve("none", "off", "none", 0),
            "nlp_ran": 1,
            "nlp_reason": "speed_window",
            "nlp_n_lattice": 18,
            "nlp_n_screened": 0,
            "nlp_n_solved": 0,
            "nlp_n_valid": 0,
            "nlp_rej_speed_window": 18,
            "nlp_rej_deadline": 0,
            "nlp_solve_us_max": 0,
        }
    ]
    r = ur.report(_plant(tmp_path, events=events))
    assert r["planner"]["nlp"]["reasons"] == {"speed_window": 1}
    text = "\n".join(ur.format_report(r))
    assert "nlp search wakes: 1 | reason: speed_window 1" in text
    assert "nlp candidates removed, by reason: speed_window 18" in text


def test_a_unit_without_logs_reports_its_status_alone(tmp_path):
    unit = tmp_path / "empty"
    unit.mkdir()
    r = ur.report(unit)
    assert r == {"unit": "empty", "status": "?"}
    assert ur.format_report(r) == ["== empty (?)"]
