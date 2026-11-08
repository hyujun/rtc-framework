"""plot_rtc_log on the gravity-feedforward joint log (`joint_ff.csv`).

Synthetic headers only: the column order is the POD's header writer (pinned to
the C++ text at the bottom), and the joint list is whatever the test names,
because the point of the pipeline is that it reads the joints off the header.
"""

from __future__ import annotations

import csv
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pytest  # noqa: E402

from rtc_tools.plotting.columns import invalidate_column_cache  # noqa: E402
from rtc_tools.plotting.io import detect_log_type  # noqa: E402
from rtc_tools.plotting.io.csv_loader import load_log_csv  # noqa: E402
from rtc_tools.plotting.io.log_type import (  # noqa: E402
    detect_log_type_by_columns,
    peek_csv_header,
)
from rtc_tools.plotting.pipelines.registry import PIPELINES, STATS_PRINTERS  # noqa: E402
from rtc_tools.plotting.plotters import (  # noqa: E402
    joint_ff as joint_ff_plots,  # noqa: E402
    print_joint_ff_statistics,
)

ALL_FIGURES = [entry.name for entry in PIPELINES["joint_ff"]]

# The shipped hand, device order (one plausible run; nothing here depends on it).
HAND_JOINTS = [
    "cmc_01_joint",
    "cmc_02_joint",
    "thumb_mcp_joint",
    "thumb_pip_link02_joint",
    "index_mcp_rotate_joint",
    "index_mcp_joint",
    "index_pip_joint",
    "middle_mcp_joint",
    "middle_pip_joint",
    "ring_mcp_joint",
]


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
    "estop",
    "readable",
    "seeded",
    "traj_active",
    "ff_enabled",
    "mode",
    "ff_finite",
    "model_fresh",
    "held",
    "held_ticks",
    "singular",
    "closure_error",
    "clamped_mask",
    "num_joints",
]
_PREFIXES = ("q_", "q_ref_", "g_", "tau_ff_")


def _header(joints, with_mode=True):
    cols = [c for c in _FIXED if with_mode or c != "mode"]
    for prefix in _PREFIXES:
        cols += [f"{prefix}{j}" for j in joints]
    return cols


def _row(header, i, joints, overrides):
    """One feedforward-on tick: q a hair under q_ref, tau_ff = g, flags healthy."""
    row = []
    for col in header:
        if col in overrides:
            value = overrides[col]
        elif col == "t_relative_s":
            value = 0.002 * i
        elif col == "tick":
            value = i
        elif col in ("readable", "seeded", "ff_enabled", "ff_finite", "model_fresh"):
            value = 1
        elif col == "num_joints":
            value = len(joints)
        elif col.startswith("q_ref_"):
            value = 0.5
        elif col.startswith("q_"):
            value = 0.4995
        elif col.startswith(("g_", "tau_ff_")):
            value = 0.02
        else:
            value = 0
        row.append(value)
    return row


def _write(path, header, rows):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def _synthetic(tmp_path, joints, n=8, per_row=None, name="joint_ff.csv", with_mode=True):
    header = _header(joints, with_mode)
    rows = [_row(header, i, joints, (per_row or (lambda _i: {}))(i)) for i in range(n)]
    path = tmp_path / name
    _write(path, header, rows)
    return load_log_csv(str(path), "joint_ff")


def _render_all(df, out_dir):
    for entry in PIPELINES["joint_ff"]:
        if entry.available(df):
            entry.fn(df, str(out_dir))
    return sorted(p.stem for p in Path(out_dir).glob("*.png"))


# ── Detection ────────────────────────────────────────────────────────────────


class TestDetection:
    def test_filename(self):
        assert detect_log_type("/s/controllers/hand_joint_controller/joint_ff.csv") == "joint_ff"

    def test_filename_with_prefix(self):
        assert detect_log_type("/tmp/run3_joint_ff.csv") == "joint_ff"

    @pytest.mark.parametrize("joints", [["j1"], HAND_JOINTS, [f"q{i}" for i in range(20)]])
    def test_any_joint_count_by_columns(self, joints):
        assert detect_log_type_by_columns(_header(joints)) == "joint_ff"

    def test_renamed_copy_reaches_the_column_fallback(self, tmp_path):
        """What plot_rtc_log.main does with a file whose name says nothing."""
        _synthetic(tmp_path, HAND_JOINTS, name="copy_of_run.csv")
        copy = tmp_path / "copy_of_run.csv"
        assert detect_log_type(str(copy)) == "unknown"
        assert detect_log_type_by_columns(peek_csv_header(str(copy))) == "joint_ff"

    def test_one_marker_alone_is_not_enough(self):
        """Two columns, so a single rename is a detection failure and not a
        near-miss into another pipeline."""
        header = _header(["j1"])
        without_flag = [c for c in header if c != "ff_enabled"]
        without_torque = [c for c in header if not c.startswith("tau_ff_")]
        assert detect_log_type_by_columns(without_flag) != "joint_ff"
        assert detect_log_type_by_columns(without_torque) != "joint_ff"

    def test_q_ref_is_not_the_fingerprint(self):
        """A future reference block named `q_ref_*` must not be read as this log."""
        assert detect_log_type_by_columns(["t_relative_s", "q_j1", "q_ref_j1"]) == "unknown"
        assert detect_log_type_by_columns(["t_relative_s", "q_ref_j1", "actual_pos_j1"]) == (
            "state_log"
        )

    def test_a_sensor_token_in_a_joint_name_stays_joint_ff(self):
        """The branch sits above the sensor_log fallback: a joint whose name
        carries `_raw_` must not reach it."""
        header = _header(["finger_raw_joint"])
        assert any("_raw_" in c for c in header)  # the collision is real
        assert detect_log_type_by_columns(header) == "joint_ff"

    def test_other_logs_keep_their_type(self):
        assert detect_log_type_by_columns(["t_relative_s", "accel_a1", "actual_pos_a1"]) == (
            "wbc_log"
        )
        assert detect_log_type_by_columns(["t_relative_s", "actual_pos_a1"]) == "state_log"
        assert detect_log_type_by_columns(["t_relative_s", "tip_raw_0"]) == "sensor_log"


# ── Columns that follow the run ──────────────────────────────────────────────


class TestColumnsFollowTheRun:
    def test_joints_are_header_order(self, tmp_path):
        df = _synthetic(tmp_path, ["zeta", "alpha", "mid"])
        assert joint_ff_plots.joints_of(df) == ["zeta", "alpha", "mid"]

    def test_joints_come_from_the_tau_ff_block_not_the_q_prefix(self, tmp_path):
        """`q_` is a prefix of `q_ref_`; counting `q_*` would double the joints."""
        df = _synthetic(tmp_path, HAND_JOINTS)
        assert len([c for c in df.columns if c.startswith("q_")]) == 2 * len(HAND_JOINTS)
        assert joint_ff_plots.joints_of(df) == HAND_JOINTS

    def test_a_joint_missing_its_position_block_is_dropped(self, tmp_path):
        df = _synthetic(tmp_path, ["a", "b"])
        df = df.drop(columns=["q_ref_b"])
        assert joint_ff_plots.joints_of(df) == ["a"]


# ── Figures ──────────────────────────────────────────────────────────────────


class TestFigures:
    def test_registered_in_both_dispatch_tables(self):
        assert "joint_ff" in PIPELINES and "joint_ff" in STATS_PRINTERS
        assert ALL_FIGURES == ["joint_ff_tracking", "joint_ff_torque", "joint_ff_status"]
        assert [e.name for e in STATS_PRINTERS["joint_ff"]] == ["print_joint_ff_stats"]

    @pytest.mark.parametrize("joints", [["j1"], HAND_JOINTS])
    def test_renders_every_figure(self, tmp_path, joints):
        df = _synthetic(tmp_path, joints)
        out = tmp_path / "png"
        out.mkdir()
        assert _render_all(df, out) == sorted(ALL_FIGURES)

    @pytest.mark.parametrize("joints", [["j1", "j2"], [f"q{i}" for i in range(9)], HAND_JOINTS])
    def test_joint_figures_grow_a_panel_per_joint(self, tmp_path, monkeypatch, joints):
        df = _synthetic(tmp_path, joints)
        captured = {}
        real_savefig = plt.savefig

        def spy(*args, **kwargs):
            axes = plt.gcf().get_axes()
            captured["visible"] = sum(1 for ax in axes if ax.get_visible())
            return real_savefig(*args, **kwargs)

        monkeypatch.setattr(plt, "savefig", spy)
        joint_ff_plots.plot_joint_ff_tracking(df, str(tmp_path))
        assert captured["visible"] == len(joints)
        joint_ff_plots.plot_joint_ff_torque(df, str(tmp_path))
        assert captured["visible"] == len(joints)

    def test_no_torque_columns_skips_the_joint_figures_only(self, tmp_path, capsys):
        df = _synthetic(tmp_path, ["j1"])
        df = df.drop(columns=["tau_ff_j1"])
        out = tmp_path / "png"
        out.mkdir()
        rendered = _render_all(df, out)
        assert rendered == ["joint_ff_status"]
        text = capsys.readouterr().out
        assert "Skipping joint FF tracking plot" in text
        assert "Skipping joint FF torque plot" in text

    def test_off_segment_is_shaded_on_every_joint_panel(self, tmp_path, monkeypatch):
        """An A/B/A run: the middle (off) stretch is one band, on each panel."""

        def per_row(i):
            return {"ff_enabled": int(not 3 <= i < 6)}

        df = _synthetic(tmp_path, ["a", "b"], n=9, per_row=per_row)
        bands = {}
        real_savefig = plt.savefig

        def spy(*args, **kwargs):
            bands["per_axis"] = [len(ax.collections) for ax in plt.gcf().get_axes()]
            bands["extent"] = plt.gcf().get_axes()[0].collections[0].get_paths()[0].get_extents()
            return real_savefig(*args, **kwargs)

        monkeypatch.setattr(plt, "savefig", spy)
        joint_ff_plots.plot_joint_ff_tracking(df, str(tmp_path))
        assert bands["per_axis"] == [1, 1]
        t = df["timestamp"].to_numpy()
        assert (bands["extent"].x0, bands["extent"].x1) == pytest.approx((t[3], t[6]))

    def test_mode_stretches_are_shaded_in_their_own_colours(self, tmp_path, monkeypatch):
        """Tracking, damping, damping_feedforward: two bands, none for tracking,
        on each joint panel, next to (not instead of) the feedforward-off band."""

        def per_row(i):
            return {"mode": 1 if 2 <= i < 4 else 2 if 5 <= i < 7 else 0}

        df = _synthetic(tmp_path, ["a", "b"], n=9, per_row=per_row)
        seen = {}
        real_savefig = plt.savefig

        def spy(*args, **kwargs):
            ax = plt.gcf().get_axes()[0]
            seen["per_axis"] = [len(a.collections) for a in plt.gcf().get_axes()]
            seen["extents"] = [c.get_paths()[0].get_extents() for c in ax.collections]
            seen["colors"] = [tuple(c.get_facecolor()[0][:3]) for c in ax.collections]
            return real_savefig(*args, **kwargs)

        monkeypatch.setattr(plt, "savefig", spy)
        joint_ff_plots.plot_joint_ff_tracking(df, str(tmp_path))
        assert seen["per_axis"] == [2, 2]
        t = df["timestamp"].to_numpy()
        spans = [(e.x0, e.x1) for e in seen["extents"]]
        assert spans[0] == pytest.approx((t[2], t[4]))
        assert spans[1] == pytest.approx((t[5], t[7]))
        assert seen["colors"][0] != seen["colors"][1]

    def test_mode_shading_tolerates_the_nan_rows_of_a_line_break(self, tmp_path):
        ticks = [0, 1, 2, 500, 501]
        df = _synthetic(
            tmp_path,
            ["a"],
            n=len(ticks),
            per_row=lambda i: {
                "tick": ticks[i],
                "t_relative_s": 0.002 * ticks[i],
                "mode": 1,
            },
        )
        broken = joint_ff_plots._with_line_breaks(df)
        assert broken["mode"].isna().any()
        _, ax = plt.subplots()
        handles = joint_ff_plots._shade_modes([ax], broken)
        assert [h.get_label() for h in handles] == ["damping"]
        assert len(ax.collections) == 1

    def test_all_tracking_has_no_mode_band_and_status_gets_a_mode_lane(self, tmp_path):
        df = _synthetic(tmp_path, ["a"])
        _, ax = plt.subplots()
        assert joint_ff_plots._shade_modes([ax], df) == []
        out = tmp_path / "png"
        out.mkdir()
        joint_ff_plots.plot_joint_ff_status(df, str(out))
        assert (out / "joint_ff_status.png").exists()

    def test_mode_is_not_a_status_flag_lane(self):
        assert "mode" not in [col for col, _ in joint_ff_plots.STATUS_LANES]

    def test_a_file_without_mode_still_renders_every_figure(self, tmp_path):
        df = _synthetic(tmp_path, ["a", "b"], with_mode=False)
        assert "mode" not in df.columns
        out = tmp_path / "png"
        out.mkdir()
        assert _render_all(df, out) == sorted(ALL_FIGURES)
        _, ax = plt.subplots()
        assert joint_ff_plots._shade_modes([ax], df) == []

    def test_an_all_on_run_has_no_band(self, tmp_path):
        df = _synthetic(tmp_path, ["a"])
        _, ax = plt.subplots()
        assert joint_ff_plots._shade_off([ax], df) == []
        assert len(ax.collections) == 0

    def test_clamped_bit_k_is_joint_k(self, tmp_path):
        df = _synthetic(tmp_path, ["a", "b", "c"], per_row=lambda i: {"clamped_mask": 0b100})
        assert not joint_ff_plots._bit(df["clamped_mask"], 0).any()
        assert joint_ff_plots._bit(df["clamped_mask"], 2).all()

    def test_lines_break_at_a_tick_gap(self, tmp_path):
        ticks = [0, 1, 2, 500, 501]
        df = _synthetic(
            tmp_path,
            ["a"],
            n=len(ticks),
            per_row=lambda i: {"tick": ticks[i], "t_relative_s": 0.002 * ticks[i]},
        )
        broken = joint_ff_plots._with_line_breaks(df)
        assert len(broken) == len(df) + 1
        assert broken["q_a"].isna().tolist() == [False] * 3 + [True] + [False] * 2
        assert broken["timestamp"].is_monotonic_increasing
        whole = _synthetic(tmp_path, ["a"], name="whole_joint_ff.csv")
        assert joint_ff_plots._with_line_breaks(whole) is whole

    def test_a_file_cut_mid_row_still_plots(self, tmp_path, capsys):
        """A session killed mid-write leaves a short last line; pandas fills the
        missing cells with NaN. The statistics run before every figure."""
        _synthetic(tmp_path, HAND_JOINTS)
        path = tmp_path / "joint_ff.csv"
        lines = path.read_text().splitlines()
        lines[-1] = ",".join(lines[-1].split(",")[:20])
        cut = tmp_path / "cut_joint_ff.csv"
        cut.write_text("\n".join(lines) + "\n")
        df = load_log_csv(str(cut), "joint_ff")
        assert df.iloc[-1].isna().any()
        print_joint_ff_statistics(df)
        assert "Rows: 8" in capsys.readouterr().out
        out = tmp_path / "png"
        out.mkdir()
        assert _render_all(df, out) == sorted(ALL_FIGURES)


# ── Statistics ───────────────────────────────────────────────────────────────


class TestStatistics:
    def test_segments_and_per_joint_error_are_printed(self, tmp_path, capsys):
        def per_row(i):
            if 3 <= i < 6:
                return {"ff_enabled": 0, "q_a": 0.48}  # sags 0.02 rad with FF off
            return {}

        print_joint_ff_statistics(_synthetic(tmp_path, ["a"], n=9, per_row=per_row))
        out = capsys.readouterr().out
        assert "Rows: 9" in out
        assert "Tick gaps: none" in out
        assert "Joints (1): a" in out
        assert "Segments by (ff_enabled, mode) (3):" in out
        assert "[3:5] ff off mode=tracking" in out
        assert "a: |q_ref − q| mean=0.02000 max=0.02000 rad" in out
        assert "ff_enabled: 6/9 ticks" in out
        assert "analyze_hand_gravity_ff" in out

    def test_without_mode_the_output_is_the_old_one(self, tmp_path, capsys):
        def per_row(i):
            return {"ff_enabled": 0, "q_a": 0.48} if 3 <= i < 6 else {}

        print_joint_ff_statistics(
            _synthetic(tmp_path, ["a"], n=9, per_row=per_row, with_mode=False)
        )
        out = capsys.readouterr().out
        assert "Segments by ff_enabled (3):" in out
        assert "[3:5] ff off t=0.01..0.01 s" in out
        assert "drift" not in out
        assert "a: |q_ref − q| mean=0.02000 max=0.02000 rad" in out

    def test_segments_split_by_mode_and_non_tracking_prints_drift(self, tmp_path, capsys):
        def per_row(i):
            if 3 <= i < 6:  # damping: q_ref follows q, q creeps 0.001 rad per 2 ms tick
                q = 0.5 + 0.001 * (i - 3)
                return {"mode": 1, "q_a": q, "q_ref_a": q, "tau_ff_a": 0.0}
            if i >= 6:
                q = 0.5
                return {"mode": 2, "q_a": q, "q_ref_a": q, "tau_ff_a": 0.04}
            return {}

        print_joint_ff_statistics(_synthetic(tmp_path, ["a"], n=9, per_row=per_row))
        out = capsys.readouterr().out
        assert "Segments by (ff_enabled, mode) (3):" in out
        assert "[0:2] ff on  mode=tracking" in out
        assert "[3:5] ff on  mode=damping" in out
        assert "[6:8] ff on  mode=damping_feedforward" in out
        assert "a: drift q(end) − q(start)=+0.00200 rad mean |dq/dt|=0.50000 rad/s" in out
        assert "mean |tau_ff|=0.00000 N·m" in out
        assert "mean |tau_ff|=0.04000 N·m" in out
        # the tracking error is printed for the tracking segment only
        assert out.count("|q_ref − q|") == 1

    def test_error_is_taken_on_readable_non_estop_rows_only(self, tmp_path, capsys):
        def per_row(i):
            if i < 4:
                return {"estop": 1, "ff_finite": 0, "q_a": 0.0}  # held pose, torque zeroed
            return {}

        print_joint_ff_statistics(_synthetic(tmp_path, ["a"], n=8, per_row=per_row))
        out = capsys.readouterr().out
        assert "4/8 readable non-E-STOP rows" in out
        assert "mean=0.00050 max=0.00050 rad" in out  # the 0.5 rad E-STOP rows are not in it

    def test_clamped_joints_are_named(self, tmp_path, capsys):
        print_joint_ff_statistics(
            _synthetic(
                tmp_path,
                ["a", "b", "c"],
                per_row=lambda i: {"clamped_mask": 0b101 if i == 2 else 0},
            )
        )
        assert "Clamped at least once: a, c" in capsys.readouterr().out

    def test_tick_gaps_are_reported(self, tmp_path, capsys):
        ticks = [0, 1, 2, 5, 6, 9]
        print_joint_ff_statistics(
            _synthetic(tmp_path, ["a"], n=len(ticks), per_row=lambda i: {"tick": ticks[i]})
        )
        assert "Tick gaps: 2 (4 ticks; largest 2)" in capsys.readouterr().out


# ── The producer's header is the source — pin the copies here to it ──────────

_LOGGING_DIR = (
    Path(__file__).resolve().parents[2]
    / "integrated_bringup"
    / "include"
    / "integrated_bringup"
    / "logging"
)
_POD_HEADER = _LOGGING_DIR / "joint_feedforward_log_pod.hpp"


def _pod_text():
    if not _POD_HEADER.exists():  # pragma: no cover - rtc_tools shipped alone
        pytest.skip(f"C++ header not present at {_POD_HEADER}")
    return _POD_HEADER.read_text()


class TestPinnedToTheProducer:
    """The synthetic header, the fingerprint and the stem are hand-written
    copies of the C++ writer. Checked against themselves they stay green when
    the producer changes; the pipeline would then misdetect or misread."""

    def test_fixed_columns_follow_the_header_writer(self):
        writer = _pod_text().split("inline void WriteJointFeedforwardLogHeader(", 1)[1]
        fixed = writer.split("for (", 1)[0]  # everything before the per-joint loop
        cols = "".join(re.findall(r'"([^"]*)"', fixed)).split(",")
        assert cols == _FIXED

    def test_per_joint_prefixes_follow_the_header_writer(self):
        writer = _pod_text().split("inline void WriteJointFeedforwardLogHeader(", 1)[1]
        loop = writer.split("for (", 1)[1].split(") {", 1)[0]  # the prefix init-list
        assert tuple(re.findall(r'"([^"]*)"', loop)) == _PREFIXES

    def test_instance_stem_is_the_filename_branch(self):
        match = re.search(r'kJointFeedforwardLogInstance\s*=\s*"([^"]+)"', _pod_text())
        assert match is not None
        assert detect_log_type(f"/s/controllers/x/{match.group(1)}.csv") == "joint_ff"

    def test_the_fingerprint_is_emitted_by_no_other_log(self):
        """The column fallback rests on `ff_enabled` + `tau_ff_` belonging to
        this log alone."""
        if not _LOGGING_DIR.is_dir():  # pragma: no cover - rtc_tools shipped alone
            pytest.skip(f"C++ headers not present at {_LOGGING_DIR}")
        for token in ("ff_enabled", "tau_ff_"):
            emitters = sorted(p.name for p in _LOGGING_DIR.glob("*.hpp") if token in p.read_text())
            assert emitters == [_POD_HEADER.name], token

    def test_flags_are_written_as_0_1_ints(self):
        """The plotters test `== 1`; a writer that printed `true` would break that."""
        row_writer = _pod_text().split("inline void WriteJointFeedforwardLogRow(", 1)[1]
        for flag in ("estop", "readable", "ff_enabled", "ff_finite", "held"):
            assert f"(p.{flag} ? 1 : 0)" in row_writer, flag


def test_status_lanes_exist_in_the_header():
    header = set(_header(["a"]))
    assert all(col in header for col, _ in joint_ff_plots.STATUS_LANES)
    assert np.all([col == label for col, label in joint_ff_plots.STATUS_LANES])
