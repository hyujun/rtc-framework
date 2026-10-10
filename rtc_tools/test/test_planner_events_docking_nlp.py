"""planner_events plotter — the E1-F18 panels and the cut-solve statistics.

The columns are read out of the C++ header, so a frame here has exactly the
names the controller writes; a recording from before the columns is the same
frame cut at ``segment_source_seq``. No recorded session is read.
"""

import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # headless: must precede the pyplot import in the plotters

import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
import pytest  # noqa: E402

from rtc_tools.plotting.io.csv_loader import load_log_csv  # noqa: E402
from rtc_tools.plotting.io.log_type import detect_log_type_by_columns  # noqa: E402
from rtc_tools.plotting.plotters import planner_events as pe  # noqa: E402

_STRINGS = {
    "outcome": "held",
    "decision": "no_current",
    "segment_outcome": "off",
    "segment_core_reason": "none",
    "segment_kind": "none",
    "nlp_reason": "off",
    "segment_infeasible_group": "none",
    "segment_cut_site": "none",
    "replace_step": "none",
    "replacement_outcome": "off",
    "replacement_core_reason": "none",
}


def _header_columns():
    hdr = (
        Path(__file__).resolve().parents[2]
        / "integrated_bringup/include/integrated_bringup/logging/planner_events_csv.hpp"
    )
    if not hdr.exists():  # pragma: no cover - 단독 배포 시
        pytest.skip(f"C++ header not present at {hdr}")
    body = hdr.read_text().split("WritePlannerEventsHeader(std::ostream& os) {", 1)[1]
    body = body.split("\n}", 1)[0]
    joined = "".join(re.findall(r'"([^"]*)"', body)).replace("\\n", "")
    return [c for c in joined.strip().split(",") if c]


def _old_columns():
    cols = _header_columns()
    return cols[: cols.index("segment_source_seq") + 1]


def _is_computed_value(name):
    """Columns the writer leaves NaN on a wake that did not compute them."""
    if name in _STRINGS or name.startswith(("nlp_rej_", "nlp_n_")):
        return False
    return name.startswith("nlp_") or name in _docking_values()


def _docking_values():
    cols = _header_columns()
    block = cols[cols.index("segment_start_us") : cols.index("replace_step")]
    return {c for c in block if c not in ("segment_infeasible_group", "segment_c_guarded")}


def _row(i, columns, **over):
    row = {}
    for name in columns:
        if name in _STRINGS:
            row[name] = _STRINGS[name]
        elif _is_computed_value(name) and name not in ("nlp_ran", "nlp_x0_clamped"):
            row[name] = float("nan")
        else:
            row[name] = 0
    row["wake_ns"] = 1_000_000_000 + i * 20_000_000
    row.update(over)
    return row


def _load(tmp_path, rows, columns):
    path = tmp_path / "planner_events.csv"
    pd.DataFrame(rows, columns=columns).to_csv(path, index=False, na_rep="nan")
    return load_log_csv(str(path), "planner_events")


def _docking_solve(i, columns, *, reason, us, group="none", **over):
    values = {
        "segment_kind": "first",
        "segment_outcome": "budget" if reason == "deadline" else "solve_failed",
        "segment_core_reason": reason,
        "segment_infeasible_group": group,
        "segment_solve_us": us,
        "segment_iterations": 10,
        "segment_qp_solves": 20,
        "segment_qp_iterations": 400,
        "segment_start_us": 10.0,
        "segment_linearize_us": 200.0,
        "segment_assemble_us": 100.0,
        "segment_qp_us": us - 400.0,
        "segment_merit_us": 90.0,
        "segment_kkt_residual": 40.0,
        "segment_c_catch": 0.6,
        "segment_sigma_s": 0.03,
        "segment_sigma_t": 0.045,
        "segment_chance_lateral": -0.07,
        "segment_chance_timing": 0.01,
    }
    for g in ("torque", "gap", "entrance", "lateral", "timing", "velocity_set", "impact"):
        values[f"segment_viol_{g}"] = 0.0
        values[f"segment_elastic_{g}"] = 0.0
    values["segment_viol_box"] = 3e-14  # rounding at an iterate that satisfies the row
    values["segment_viol_terminal"] = 0.0
    values["segment_viol_lateral"] = 0.07
    values.update(over)
    return _row(i, columns, **values)


def _nlp_wake(i, columns, *, reason, **over):
    values = {
        "outcome": "published",
        "nlp_ran": 1,
        "nlp_reason": reason,
        "nlp_n_lattice": 18,
        "nlp_n_screened": 4,
        "nlp_n_solved": 2,
        "nlp_n_valid": 1 if reason == "none" else 0,
        "nlp_screen_us": 700,
        "nlp_solve_us_max": 9000,
    }
    if reason == "none":
        values.update(nlp_phi=3.5, nlp_j_reference=2.5, nlp_lead_s=0.3)
    values.update(over)
    return _row(i, columns, **values)


class TestTheLogIsStillRecognised:
    def test_the_wider_header_is_the_same_channel(self):
        columns = _header_columns()
        assert len(columns) > len(_old_columns())
        assert detect_log_type_by_columns(columns) == "planner_events"
        assert detect_log_type_by_columns(_old_columns()) == "planner_events"

    def test_the_name_columns_stay_strings_when_every_row_says_off(self, tmp_path):
        """A session of another search and planner writes `off` / `none` in every
        row: a column outside the loader's string set would come back all-NaN."""
        columns = _header_columns()
        df = _load(tmp_path, [_row(i, columns) for i in range(5)], columns)
        for name, value in _STRINGS.items():
            assert df[name].iloc[0] == value, name


class TestPanels:
    def test_an_older_recording_gets_no_new_panel(self, tmp_path):
        columns = _old_columns()
        rows = [
            _row(
                i,
                columns,
                segment_kind="first",
                segment_outcome="published",
                segment_solve_us=7000,
            )
            for i in range(6)
        ]
        df = _load(tmp_path, rows, columns)
        assert pe._segment_panels(df) == ["outcome", "solve", "catch"]
        assert pe._nlp_panels(df) == []
        pe.plot_planner_events(df, save_dir=str(tmp_path))
        assert (tmp_path / "planner_events.png").exists()
        pe.print_planner_events_statistics(df)

    def test_a_session_of_another_planner_has_the_columns_and_no_panel(self, tmp_path):
        columns = _header_columns()
        rows = [
            _row(
                i,
                columns,
                segment_kind="first",
                segment_outcome="published",
                segment_solve_us=7000,
            )
            for i in range(6)
        ]
        df = _load(tmp_path, rows, columns)
        assert pe._segment_panels(df) == ["outcome", "solve", "catch"]
        assert pe._nlp_panels(df) == []

    def test_docking_solves_add_the_rows_and_crossing_panels(self, tmp_path):
        columns = _header_columns()
        rows = [_docking_solve(i, columns, reason="deadline", us=35_050 + i) for i in range(4)]
        rows.append(_docking_solve(9, columns, reason="infeasible", us=20_000, group="lateral"))
        df = _load(tmp_path, rows, columns)
        assert pe._segment_panels(df) == ["outcome", "solve", "catch", "rows", "crossing"]
        pe.plot_planner_events(df, save_dir=str(tmp_path))
        assert (tmp_path / "planner_events.png").exists()

    def test_nlp_wakes_add_their_panels(self, tmp_path):
        columns = _header_columns()
        screened_out = [
            _nlp_wake(i, columns, reason="speed_window", nlp_n_solved=0, nlp_solve_us_max=0)
            for i in range(4)
        ]
        df = _load(tmp_path, screened_out, columns)
        # Nothing was solved or chosen: no cost panel to draw.
        assert pe._nlp_panels(df) == ["nlp_funnel", "nlp_reason"]
        rows = [*screened_out, _nlp_wake(5, columns, reason="none")]
        rows.append(_nlp_wake(6, columns, reason="deadline", nlp_rej_deadline=1))
        df = _load(tmp_path, rows, columns)
        assert pe._nlp_panels(df) == ["nlp_funnel", "nlp_reason", "nlp_chosen"]
        pe.plot_planner_events(df, save_dir=str(tmp_path))
        assert (tmp_path / "planner_events.png").exists()


class TestCutSolvesAreDrawnAndCountedApart:
    def _frame(self, tmp_path):
        columns = _header_columns()
        rows = [
            _docking_solve(i, columns, reason="deadline", us=35_050 + 100 * i) for i in range(3)
        ]
        rows += [
            _docking_solve(5, columns, reason="infeasible", us=20_000, group="lateral"),
            _docking_solve(6, columns, reason="infeasible", us=22_000, group="lateral"),
        ]
        return _load(tmp_path, rows, columns)

    def test_the_solve_panel_marks_them(self, tmp_path):
        import matplotlib.pyplot as plt

        df = self._frame(tmp_path)
        fig, ax = plt.subplots()
        pe._draw_segment_solve(ax, df, pe._time_axis(df))
        by_label = {c.get_label(): c for c in ax.collections}
        assert len(by_label["first"].get_offsets()) == 2
        cut = by_label["first: cut at deadline (≥)"]
        assert len(cut.get_offsets()) == 3
        # Hollow: no face colour.
        assert cut.get_facecolors().size == 0 or np.all(cut.get_facecolors()[:, 3] == 0)
        plt.close(fig)

    def test_the_statistics_keep_them_out_of_the_distribution(self, tmp_path, capsys):
        pe.print_planner_events_statistics(self._frame(tmp_path))
        out = capsys.readouterr().out
        line = next(text for text in out.splitlines() if text.startswith("Segment kind first"))
        assert "solve [ms] (n=2): p50 20.00  p99 22.00  max 22.00" in line
        assert "cut at the deadline: 3 (>= 35.05 ms, not a solve time)" in line
        assert "Segment solves by kind / outcome / core reason / infeasible group:" in out
        # The violated group is listed with its numbers; rounding in `box` is not.
        violation = next(text for text in out.splitlines() if "violation at the returned" in text)
        assert "lateral p50 0.07 max 0.07" in violation and "box" not in violation

    def test_the_rows_panel_draws_violated_rows_only(self, tmp_path):
        import matplotlib.pyplot as plt

        df = self._frame(tmp_path)
        fig, ax = plt.subplots()
        pe._draw_segment_rows(ax, df, pe._time_axis(df))
        drawn = {line.get_label() for line in ax.lines}
        assert drawn == {"lateral"}
        # The two solves that ended infeasible in `lateral` are ringed.
        assert sum(len(c.get_offsets()) for c in ax.collections) == 2
        plt.close(fig)


def test_the_reach_pre_filter_reason_reaches_the_statistics(tmp_path, capsys):
    columns = _header_columns()
    rows = [
        _nlp_wake(0, columns, reason="too_far", nlp_rej_too_far=18, nlp_solve_us_max=0),
        _nlp_wake(1, columns, reason="none"),
    ]
    pe.print_planner_events_statistics(_load(tmp_path, rows, columns))
    out = capsys.readouterr().out
    assert "NLP search wakes: 2 | reason: too_far×1, none×1" in out
    assert "NLP candidates removed, by reason: too_far×18" in out


def test_replacement_steps_and_nlp_reasons_reach_the_statistics(tmp_path, capsys):
    columns = _header_columns()
    rows = [
        _nlp_wake(0, columns, reason="speed_window", nlp_rej_speed_window=18, nlp_solve_us_max=0),
        _nlp_wake(1, columns, reason="none"),
        _row(2, columns, replace_step="too_late_followed"),
        _row(3, columns, replace_step="published"),
    ]
    pe.print_planner_events_statistics(_load(tmp_path, rows, columns))
    out = capsys.readouterr().out
    assert "NLP search wakes: 2 | reason: speed_window×1, none×1" in out
    assert "NLP candidates removed, by reason: speed_window×18" in out
    assert "Replacement attempts ended: too_late_followed×1, published×1" in out
