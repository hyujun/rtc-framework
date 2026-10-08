"""plot_rtc_log on the multi-frame CLIK diagnostics log and a many-joint state log.

Two sources of columns, on purpose:

  - a RECORDED header. ``data/g1_dualarm_review_pairs`` is 120 consecutive rows
    (20 before the first goal, 100 after) cut from a g1_p1b sim session of
    demo_dualarm_controller — two tasks, 17 body joints. Rows were cut, never
    thinned or edited: a thinned file has tick gaps, and a tick gap is what this
    log uses to say a row was dropped. This is the header the column fallback
    used to classify as a WBC device log. Stored gzipped (the repo ignores
    `*.csv`) and unpacked per test under the file names the session had.
  - SYNTHETIC headers with other task and joint counts, because the point of the
    pipeline is that it reads both off the header.
"""

from __future__ import annotations

import csv
import gzip
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import pytest  # noqa: E402

from rtc_tools.plotting.columns import (  # noqa: E402
    detect_joint_columns,
    detect_task_prefixes,
    has_goal_counter_activity,
    invalidate_column_cache,
    task_counter_columns,
)
from rtc_tools.plotting.io import detect_log_type  # noqa: E402
from rtc_tools.plotting.io.csv_loader import load_log_csv  # noqa: E402
from rtc_tools.plotting.io.log_type import (  # noqa: E402
    detect_log_type_by_columns,
    peek_csv_header,
)
from rtc_tools.plotting.pipelines.registry import PIPELINES, STATS_PRINTERS  # noqa: E402
from rtc_tools.plotting.plotters import (  # noqa: E402
    dualarm as dualarm_plots,  # noqa: E402
    plot_robot_positions,
    print_dualarm_diag_statistics,
)

FIXTURE = (
    Path(__file__).parent
    / "data"
    / "g1_dualarm_review_pairs"
    / "controllers"
    / "demo_dualarm_controller"
)


def _unpack(tmp_path, stem):
    """`<stem>.csv` of the recorded session, unpacked under a session-shaped path."""
    out_dir = tmp_path / "session" / "controllers" / "demo_dualarm_controller"
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / f"{stem}.csv"
    out.write_bytes(gzip.decompress((FIXTURE / f"{stem}.csv.gz").read_bytes()))
    return out


@pytest.fixture
def diag_csv(tmp_path):
    return _unpack(tmp_path, "dualarm_diag")


@pytest.fixture
def state_csv(tmp_path):
    return _unpack(tmp_path, "g1_state")


ALL_FIGURES = [entry.name for entry in PIPELINES["dualarm_diag"]]


@pytest.fixture(autouse=True)
def _clear_cache():
    invalidate_column_cache()
    yield
    invalidate_column_cache()
    plt.close("all")


# ── Synthetic header (column order of the POD's header writer) ───────────────

_FIXED = [
    "t_relative_s",
    "tick",
    "hold",
    "clik_ran",
    "estop",
    "fault_latched",
    "fault_cause",
    "body_readable",
    "hand_readable",
    "reseeded",
    "reached_solve",
    "converged",
    "rejected_input",
    "non_finite",
    "command_mismatch",
    "accel_rows_violated",
    "brake_box_empty",
    "status",
    "iterations",
    "solve_time_us",
    "accel_rows",
    "accel_rows_binding",
    "fb_saturated",
    "rot_near_pi",
    "brake_active",
    "brake_static_infeasible",
    "qp_fail_streak",
    "track_err",
    "group_goal_rejects",
]
_POSE = ("x", "y", "z", "qw", "qx", "qy", "qz")
_COUNTERS = (
    "goals_accepted",
    "reject_goal_type",
    "reject_non_finite",
    "reject_unknown_frame",
    "drop_stale",
    "drop_unusable",
    "drop_held",
)


def _task_columns(task):
    cols = [f"{task}_{f}" for f in ("valid", "traj_active", "goal_sequence", "err_lin", "err_ang")]
    cols += [f"{task}_ref_{a}" for a in _POSE]
    cols += [f"{task}_cmd_{a}" for a in _POSE]
    cols += [f"{task}_meas_valid"]
    cols += [f"{task}_meas_{a}" for a in _POSE]
    cols += [f"{task}_{c}" for c in _COUNTERS]
    return cols


def _header(tasks, joints):
    cols = list(_FIXED)
    for task in tasks:
        cols += _task_columns(task)
    cols += [f"q_cmd_{j}" for j in joints]
    return cols


def _row(header, i, overrides):
    """One solved tick: identity quaternions, small errors, everything else 0."""
    row = []
    for col in header:
        if col in overrides:
            value = overrides[col]
        elif col == "t_relative_s":
            value = 0.002 * i
        elif col == "tick":
            value = i
        elif col in (
            "clik_ran",
            "converged",
            "reached_solve",
            "body_readable",
            "hand_readable",
        ) or col.endswith(("_valid",)):
            value = 1
        elif col.endswith("_qw"):
            value = 1.0
        elif col == "solve_time_us":
            value = 12.5 + i
        elif col.endswith("_err_lin"):
            value = 1e-4 * (i + 1)
        elif col.startswith("q_cmd_"):
            value = 0.01 * i
        else:
            value = 0
        row.append(value)
    return row


def _write(path, header, rows):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def _synthetic(tmp_path, tasks, joints, n=8, per_row=None, name="dualarm_diag.csv"):
    header = _header(tasks, joints)
    rows = [_row(header, i, (per_row or (lambda _i: {}))(i)) for i in range(n)]
    path = tmp_path / name
    _write(path, header, rows)
    return load_log_csv(str(path), "dualarm_diag")


def _render_all(df, out_dir):
    for entry in PIPELINES["dualarm_diag"]:
        if entry.available(df):
            entry.fn(df, str(out_dir))
    return sorted(p.stem for p in Path(out_dir).glob("*.png"))


# ── Detection ────────────────────────────────────────────────────────────────


class TestDetection:
    def test_filename(self):
        assert detect_log_type("/s/controllers/some_controller/dualarm_diag.csv") == "dualarm_diag"

    def test_recorded_file_in_its_session_path(self, diag_csv):
        assert detect_log_type(str(diag_csv)) == "dualarm_diag"

    def test_filename_with_prefix(self):
        assert detect_log_type("/tmp/run3_dualarm_diag.csv") == "dualarm_diag"

    def test_recorded_header_by_columns(self, diag_csv):
        """The recorded header carries `accel_rows*`, which the `accel_` branch
        of the fallback claims for the WBC device log. Classified as that, the
        file plotted two meaningless figures and exited 0."""
        header = peek_csv_header(str(diag_csv))
        assert any(c.startswith("accel_") for c in header)  # the collision is real
        assert detect_log_type_by_columns(header) == "dualarm_diag"

    def test_renamed_copy_reaches_the_column_fallback(self, tmp_path, diag_csv):
        """What plot_rtc_log.main does with a file whose name says nothing."""
        copy = tmp_path / "copy_of_run.csv"
        copy.write_text(diag_csv.read_text())
        assert detect_log_type(str(copy)) == "unknown"
        assert detect_log_type_by_columns(peek_csv_header(str(copy))) == "dualarm_diag"

    @pytest.mark.parametrize(
        ("tasks", "joints"),
        [
            (["tool"], ["j1", "j2"]),
            (["a", "b", "c"], [f"q{i}" for i in range(9)]),
            (["one", "two", "three", "four"], []),
        ],
    )
    def test_any_task_and_joint_count_by_columns(self, tasks, joints):
        assert detect_log_type_by_columns(_header(tasks, joints)) == "dualarm_diag"

    def test_one_marker_alone_is_not_enough(self):
        """Two columns, so a single rename is a detection failure and not a
        near-miss into another pipeline."""
        header = _header(["tool"], ["j1"])
        without_marker = [c for c in header if c != "clik_ran"]
        without_task = [c for c in header if not c.endswith("_err_lin")]
        assert detect_log_type_by_columns(without_marker) != "dualarm_diag"
        assert detect_log_type_by_columns(without_task) != "dualarm_diag"

    def test_other_logs_keep_their_type(self, state_csv):
        """The new branch sits above the WBC and state-log ones; it must not
        take their files."""
        assert detect_log_type_by_columns(["t_relative_s", "accel_a1", "actual_pos_a1"]) == (
            "wbc_log"
        )
        assert detect_log_type_by_columns(peek_csv_header(str(state_csv))) == "state_log"
        # catching_diag also has `clik_ran`; it has no `_err_lin`.
        assert detect_log_type_by_columns(["clik_ran", "track_err_rad", "ref_gamma"]) == (
            "catching_diag"
        )


# ── Columns that follow the run ──────────────────────────────────────────────


class TestColumnsFollowTheRun:
    def test_recorded_tasks_and_joints(self, diag_csv):
        df = load_log_csv(str(diag_csv), "dualarm_diag")
        assert detect_task_prefixes(df) == ["right_hand", "left_hand"]
        cols, names = detect_joint_columns(df, "q_cmd_")
        assert len(cols) == 17
        assert names[0] == "waist_yaw_joint" and names[-1] == "right_wrist_yaw_joint"

    def test_task_order_is_header_order(self, tmp_path):
        df = _synthetic(tmp_path, ["zeta", "alpha", "mid"], ["j1"])
        assert detect_task_prefixes(df) == ["zeta", "alpha", "mid"]

    def test_task_name_with_underscores(self, tmp_path):
        df = _synthetic(tmp_path, ["left_tool_tip"], ["j1"])
        assert detect_task_prefixes(df) == ["left_tool_tip"]

    def test_lone_err_lin_column_is_not_a_task(self, tmp_path):
        df = _synthetic(tmp_path, ["tool"], ["j1"])
        df["stray_err_lin"] = 0.0
        assert detect_task_prefixes(df) == ["tool"]

    def test_counters_are_found_by_prefix(self, tmp_path):
        """A reason the producer names differently is still a counter."""
        df = _synthetic(tmp_path, ["tool"], ["j1"])
        df = df.rename(columns={"tool_drop_unusable": "tool_drop_some_new_reason"})
        counters = task_counter_columns(df, "tool")
        assert counters[0] == "tool_goals_accepted"
        assert "tool_drop_some_new_reason" in counters
        assert len(counters) == len(_COUNTERS)


# ── Figures ──────────────────────────────────────────────────────────────────


class TestFigures:
    def test_recorded_session_renders_every_figure(self, tmp_path, diag_csv):
        df = load_log_csv(str(diag_csv), "dualarm_diag")
        assert _render_all(df, tmp_path) == sorted(ALL_FIGURES)

    @pytest.mark.parametrize(
        ("tasks", "joints"),
        [(["tool"], ["j1", "j2"]), (["a", "b", "c"], [f"q{i}" for i in range(9)])],
    )
    def test_task_figures_grow_a_row_per_task(self, tmp_path, monkeypatch, tasks, joints):
        df = _synthetic(tmp_path, tasks, joints)
        shapes = {}
        real_subplots = plt.subplots

        def spy(nrows=1, ncols=1, **kwargs):
            shapes["last"] = (nrows, ncols)
            return real_subplots(nrows, ncols, **kwargs)

        monkeypatch.setattr(dualarm_plots.plt, "subplots", spy)
        dualarm_plots.plot_dualarm_diag_task_error(df, str(tmp_path))
        assert shapes["last"] == (len(tasks), 1)
        dualarm_plots.plot_dualarm_diag_task_pose(df, str(tmp_path))
        assert shapes["last"] == (len(tasks), 4)

    def test_no_joint_columns_skips_only_the_joint_figure(self, tmp_path, capsys):
        df = _synthetic(tmp_path, ["tool"], [])
        out = tmp_path / "png"
        out.mkdir()
        rendered = _render_all(df, out)
        assert "dualarm_diag_joint_cmd" not in rendered
        assert "dualarm_diag_task_error" in rendered
        assert "Skipping CLIK joint command plot" in capsys.readouterr().out

    def test_goal_figure_is_gated_on_counter_activity(self, tmp_path):
        quiet = _synthetic(tmp_path, ["tool"], ["j1"], name="quiet_dualarm_diag.csv")
        assert has_goal_counter_activity(quiet) is False
        busy = _synthetic(
            tmp_path,
            ["tool"],
            ["j1"],
            per_row=lambda i: {"tool_reject_unknown_frame": int(i >= 4)},
            name="busy_dualarm_diag.csv",
        )
        assert has_goal_counter_activity(busy) is True
        lane = _synthetic(
            tmp_path,
            ["tool"],
            ["j1"],
            per_row=lambda i: {"group_goal_rejects": int(i >= 4)},
            name="lane_dualarm_diag.csv",
        )
        assert has_goal_counter_activity(lane) is True

    def test_held_ticks_are_not_drawn_as_solve_data(self, tmp_path):
        """A held tick writes zeros for what it did not compute. Those zeros
        must not reach the solve-time line or a task's error line."""

        def per_row(i):
            if 3 <= i < 6:
                return {"hold": 2, "clik_ran": 0, "solve_time_us": 0, "tool_valid": 0}
            return {}

        df = _synthetic(tmp_path, ["tool"], ["j1"], n=9, per_row=per_row)
        solved = dualarm_plots._solved(df)
        assert list(solved) == [True] * 3 + [False] * 3 + [True] * 3
        solve_time = dualarm_plots._masked(df, "solve_time_us", solved)
        assert solve_time.isna().tolist() == [False] * 3 + [True] * 3 + [False] * 3
        err = dualarm_plots._masked(df, "tool_err_lin", dualarm_plots._task_valid(df, "tool"))
        assert err.isna().sum() == 3
        assert dualarm_plots._runs(df["hold"].to_numpy() == 2) == [(3, 6)]
        # and the figures still render with a held stretch in the file
        out = tmp_path / "png"
        out.mkdir()
        assert "dualarm_diag_solver" in _render_all(df, out)

    def test_feedback_mask_bits_index_tasks_by_header_position(self, tmp_path):
        """Bit 2k = task k linear, 2k+1 = task k angular."""
        df = _synthetic(tmp_path, ["a", "b"], ["j1"], per_row=lambda _i: {"fb_saturated": 0b1000})
        assert not dualarm_plots._bit(df["fb_saturated"], 0).any()  # a linear
        assert not dualarm_plots._bit(df["fb_saturated"], 2).any()  # b linear
        assert dualarm_plots._bit(df["fb_saturated"], 3).all()  # b angular

    def test_cmd_meas_gap_is_geodesic_and_blank_where_not_computed(self, tmp_path):
        def per_row(i):
            row = {"tool_meas_x": 0.003, "tool_meas_y": 0.004}  # 5 mm from cmd at 0
            if i == 0:  # −q is the same rotation as q
                row.update({"tool_meas_qw": -1.0})
            if i == 1:
                row.update({"tool_meas_valid": 0})
            return row

        df = _synthetic(tmp_path, ["tool"], ["j1"], n=4, per_row=per_row)
        dist, angle = dualarm_plots._cmd_meas_gap(df, "tool")
        assert dist[0] == pytest.approx(0.005)
        assert angle[0] == pytest.approx(0.0, abs=1e-9)
        assert dist[1] != dist[1] and angle[1] != angle[1]  # NaN: meas not valid


# ── Statistics ───────────────────────────────────────────────────────────────


class TestStatistics:
    def test_registry_runs_the_printer(self):
        assert [e.name for e in STATS_PRINTERS["dualarm_diag"]] == ["print_dualarm_diag_stats"]

    def test_recorded_session(self, capsys, diag_csv):
        print_dualarm_diag_statistics(load_log_csv(str(diag_csv), "dualarm_diag"))
        out = capsys.readouterr().out
        assert "Rows: 120" in out
        assert "Dropped rows (tick gaps): 0" in out
        assert "Ticks that ran the solve: 120/120" in out
        assert "Tasks (2): right_hand, left_hand" in out
        assert "goal sequence reached 1" in out

    def test_dropped_rows_are_counted_from_tick_gaps(self, tmp_path, capsys):
        # ticks 0 1 2 | 5 6 | 9 → 2 + 2 rows missing
        ticks = [0, 1, 2, 5, 6, 9]
        df = _synthetic(
            tmp_path, ["tool"], ["j1"], n=len(ticks), per_row=lambda i: {"tick": ticks[i]}
        )
        print_dualarm_diag_statistics(df)
        assert "Dropped rows (tick gaps): 4" in capsys.readouterr().out

    def test_holds_and_fault_cause_are_named(self, tmp_path, capsys):
        def per_row(i):
            if i >= 5:
                return {"hold": 2, "clik_ran": 0, "fault_latched": 1, "fault_cause": 3}
            return {}

        print_dualarm_diag_statistics(_synthetic(tmp_path, ["tool"], ["j1"], n=8, per_row=per_row))
        out = capsys.readouterr().out
        assert "Ticks that ran the solve: 5/8" in out
        assert "fault latch: 3" in out
        assert "seed outside box" in out

    def test_unknown_codes_print_as_numbers(self, tmp_path, capsys):
        """A code this file has no name for is reported, not dropped or crashed on."""
        print_dualarm_diag_statistics(
            _synthetic(
                tmp_path,
                ["tool"],
                ["j1"],
                per_row=lambda _i: {"hold": 9, "clik_ran": 0},
            )
        )
        assert "code 9: 8" in capsys.readouterr().out

    def test_solve_time_ignores_held_ticks(self, tmp_path, capsys):
        def per_row(i):
            if i < 4:
                return {"hold": 1, "clik_ran": 0, "solve_time_us": 0}
            return {"solve_time_us": 20.0}

        print_dualarm_diag_statistics(_synthetic(tmp_path, ["tool"], ["j1"], n=8, per_row=per_row))
        assert "p50=20.0 p99=20.0 max=20.0" in capsys.readouterr().out

    def test_brake_mask_bits_are_listed(self, tmp_path, capsys):
        print_dualarm_diag_statistics(
            _synthetic(
                tmp_path,
                ["tool"],
                ["j1"],
                per_row=lambda i: {"brake_active": 0b101 if i == 2 else 0},
            )
        )
        out = capsys.readouterr().out
        assert "brake_active — model velocity indices ever set: [0, 2]" in out
        assert "brake_static_infeasible — model velocity indices ever set: none" in out


# ── Many-joint state log (second device group of the same session) ───────────


class TestManyJointStateLog:
    def test_recorded_state_log_detects_and_loads(self, state_csv):
        assert detect_log_type(str(state_csv)) == "unknown"  # the stem says nothing
        df = load_log_csv(str(state_csv), "state_log")
        cols, _ = detect_joint_columns(df, "actual_pos_")
        assert len(cols) == 17

    def test_unused_grid_cells_are_hidden(self, tmp_path, monkeypatch, state_csv):
        """17 joints land on a 4 x 5 grid; the three spare cells used to stay
        on the figure as empty axes."""
        df = load_log_csv(str(state_csv), "state_log")
        captured = {}
        real_savefig = plt.savefig

        def spy(*args, **kwargs):
            captured["axes"] = plt.gcf().get_axes()
            return real_savefig(*args, **kwargs)

        monkeypatch.setattr(plt, "savefig", spy)
        plot_robot_positions(df, str(tmp_path))
        visible = [ax.get_visible() for ax in captured["axes"]]
        assert len(visible) == 20
        assert visible == [True] * 17 + [False] * 3
