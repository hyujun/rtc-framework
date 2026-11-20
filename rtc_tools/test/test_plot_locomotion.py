"""plot_rtc_log on the locomotion controller's logs (`locomotion_diag.csv`, `locomotion_truth.csv`).

Synthetic CSVs built in the test (the repo ignores `*.csv`). The diag fixture is
the one `test_locomotion_diag` builds for the standalone entry point, so the two
entry points are checked against the same file; the truth header follows the
POD's header writer, with contact names chosen here (the pipeline must read them
from the header).
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import pytest  # noqa: E402
import test_locomotion_diag as fx  # noqa: E402

from rtc_tools.analysis import locomotion_diag as ld  # noqa: E402
from rtc_tools.plotting.columns import invalidate_column_cache  # noqa: E402
from rtc_tools.plotting.io import detect_log_type  # noqa: E402
from rtc_tools.plotting.io.csv_loader import load_log_csv  # noqa: E402
from rtc_tools.plotting.io.log_type import (  # noqa: E402
    detect_log_type_by_columns,
    peek_csv_header,
)
from rtc_tools.plotting.pipelines.registry import PIPELINES  # noqa: E402

TRUTH_CONTACTS = ["left_sole", "right_sole", "extra_pad"]
N_TRUTH = 12


@pytest.fixture(autouse=True)
def _clear_cache():
    invalidate_column_cache()
    yield
    invalidate_column_cache()
    plt.close("all")


def truth_header(contacts=TRUTH_CONTACTS) -> list[str]:
    cols = ["t_relative_s", "tick", "truth_mask", "truth_base_seq", "truth_base_age_us"]
    cols += [f"truth_base_{a}" for a in ("px", "py", "pz", "qx", "qy", "qz", "qw")]
    cols += [f"truth_base_{a}" for a in ("vx", "vy", "vz", "wx", "wy", "wz")]
    for c in contacts:
        cols += [f"truth_grf_{c}_{a}" for a in ("fx", "fy", "fz", "tx", "ty", "tz")]
        cols += [f"truth_cop_{c}_x", f"truth_cop_{c}_y", f"truth_age_{c}_us", f"truth_seq_{c}"]
    return cols


def write_truth(path: Path, contacts=TRUTH_CONTACTS, moments=True) -> Path:
    header = truth_header(contacts)
    ticks = [t for t in range(N_TRUTH + 2) if t != 5]  # tick 5: no lane fresh, no row
    rows = []
    for i, tick in enumerate(ticks):
        row = dict.fromkeys(header, 0.0)
        row.update(t_relative_s=0.01 * tick, tick=tick, truth_mask=(1 << (len(contacts) + 1)) - 1)
        row.update(truth_base_pz=0.75, truth_base_qw=1.0, truth_base_px=0.001 * i)
        row.update(truth_base_age_us=15.0 + i, truth_base_vx=0.1)
        for k, c in enumerate(contacts):
            row[f"truth_grf_{c}_fz"] = 150.0 + 10 * k
            row[f"truth_cop_{c}_x"] = 0.01 * k
            row[f"truth_age_{c}_us"] = -1.0 if (k == 2 and i < 3) else 12.0
            if moments:
                row[f"truth_grf_{c}_tx"] = 1.0 + k
            else:
                for a in ("tx", "ty", "tz"):
                    row[f"truth_grf_{c}_{a}"] = math.nan
        if i == 4:
            row["truth_mask"] = 0b011  # base + first contact only
            row[f"truth_cop_{contacts[-1]}_x"] = math.nan
        rows.append(row)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows([[repr(float(r[h])) for h in header] for r in rows])
    return path


def render(log_type: str, csv_path: Path, out: Path) -> list[str]:
    df = load_log_csv(str(csv_path), log_type)
    out.mkdir(parents=True, exist_ok=True)  # plot_rtc_log.main does this
    for entry in PIPELINES[log_type]:
        if entry.available(df):
            entry.fn(df, str(out))
    return sorted(p.stem for p in out.glob("*.png"))


@pytest.fixture
def diag_csv(tmp_path: Path) -> Path:
    path = tmp_path / "s" / "controllers" / "locomotion_controller" / ld.DIAG_FILE
    fx.write_csv(path, fx.diag_header(), fx.diag_rows())
    return path


# ── Detection ────────────────────────────────────────────────────────────────


class TestDetection:
    def test_diag_filename(self):
        assert detect_log_type("/s/controllers/locomotion_controller/locomotion_diag.csv") == (
            "locomotion_diag"
        )

    def test_truth_filename(self):
        assert detect_log_type("/s/controllers/locomotion_controller/locomotion_truth.csv") == (
            "locomotion_truth"
        )

    def test_filenames_with_prefix(self):
        assert detect_log_type("/tmp/run3_locomotion_diag.csv") == "locomotion_diag"
        assert detect_log_type("/tmp/run3_locomotion_truth.csv") == "locomotion_truth"

    def test_diag_by_columns_beats_wbc_diag(self):
        """The file carries `solve_time_us` and `lambda_<c>_*`, which fingerprint wbc_diag."""
        header = fx.diag_header()
        assert "solve_time_us" in header
        assert any(c.startswith("lambda_") for c in header)
        assert detect_log_type_by_columns(header) == "locomotion_diag"

    @pytest.mark.parametrize("contacts", [["a"], TRUTH_CONTACTS, ["l_foot", "r_foot", "l_hand"]])
    def test_truth_by_columns_any_contact_count(self, contacts):
        assert detect_log_type_by_columns(truth_header(contacts)) == "locomotion_truth"

    def test_renamed_copies_reach_the_column_fallback(self, tmp_path, diag_csv):
        truth = write_truth(tmp_path / "copy_truth.csv")
        diag = tmp_path / "copy_diag.csv"
        diag.write_bytes(diag_csv.read_bytes())
        for path, kind in ((diag, "locomotion_diag"), (truth, "locomotion_truth")):
            assert detect_log_type(str(path)) == "unknown"
            assert detect_log_type_by_columns(peek_csv_header(str(path))) == kind

    def test_one_marker_alone_is_not_enough(self):
        no_fault = [c for c in fx.diag_header() if c != "fault_cause"]
        assert detect_log_type_by_columns(no_fault) == "wbc_diag"
        no_base = [c for c in truth_header() if not c.startswith("truth_base_")]
        assert detect_log_type_by_columns(no_base) == "unknown"

    def test_neighbouring_pipelines_unchanged(self):
        assert detect_log_type("/s/wbc_diag.csv") == "wbc_diag"
        assert detect_log_type("/s/joint_ff.csv") == "joint_ff"


# ── locomotion_diag ──────────────────────────────────────────────────────────


class TestDiagPipeline:
    def test_figures_are_the_entry_points_files_minus_joint_torque(self, tmp_path, diag_csv):
        stems = render("locomotion_diag", diag_csv, tmp_path / "out")
        expected = sorted(Path(f).stem for f in ld.FIGURE_FILES if "joint_torque" not in f)
        assert stems == expected
        assert [e.name for e in PIPELINES["locomotion_diag"]] == sorted(expected)
        for p in (tmp_path / "out").glob("*.png"):
            assert p.stat().st_size > 0

    def test_same_function_as_the_console_entry(self, tmp_path, diag_csv):
        """Both entry points draw through plotters.locomotion_diag: the files are the same."""
        out_main = tmp_path / "main"
        assert ld.main([str(diag_csv.parents[2]), "--out", str(out_main)]) == 0
        out_pipe = tmp_path / "pipe"
        render("locomotion_diag", diag_csv, out_pipe)
        for name in (
            "01_base_pose.png",
            "05_qp_timeline.png",
            "06_margins.png",
            "07_mode_timeline.png",
        ):
            assert (out_main / name).read_bytes() == (out_pipe / name).read_bytes()

    def test_contacts_follow_the_header(self, tmp_path):
        header = [*fx.diag_header(), "lambda_third_fz"]
        path = tmp_path / ld.DIAG_FILE
        fx.write_csv(path, header, fx.diag_rows())
        df = load_log_csv(str(path), "locomotion_diag")
        from rtc_tools.plotting.plotters.locomotion_diag import session_from_frame

        assert session_from_frame(df).diag_names.contacts == [*fx.CONTACTS, "third"]

    def test_log_without_the_contact_block_skips_that_figure(self, tmp_path):
        header = [h for h in fx.diag_header() if not h.startswith("lambda_")]
        path = tmp_path / ld.DIAG_FILE
        fx.write_csv(path, header, fx.diag_rows())
        stems = render("locomotion_diag", path, tmp_path / "out")
        assert "02_contact_forces" not in stems
        assert "01_base_pose" in stems

    def test_no_save_dir_draws_nothing(self, diag_csv):
        df = load_log_csv(str(diag_csv), "locomotion_diag")
        for entry in PIPELINES["locomotion_diag"]:
            entry.fn(df, None)
        assert plt.get_fignums() == []


# ── locomotion_truth ─────────────────────────────────────────────────────────

ALL_TRUTH = [
    "truth_ages",
    "truth_base_pose",
    "truth_base_twist",
    "truth_cop",
    "truth_grf",
    "truth_grm",
    "truth_mask",
]


class TestTruthPipeline:
    def test_every_figure(self, tmp_path):
        path = write_truth(tmp_path / "locomotion_truth.csv")
        stems = render("locomotion_truth", path, tmp_path / "out")
        assert stems == ALL_TRUTH
        for p in (tmp_path / "out").glob("*.png"):
            assert p.stat().st_size > 0

    def test_rows_follow_the_header_contacts(self, tmp_path, monkeypatch):
        from rtc_tools.plotting.plotters import locomotion_truth as lt

        path = write_truth(tmp_path / "locomotion_truth.csv")
        df = load_log_csv(str(path), "locomotion_truth")
        assert lt.contact_names(df) == TRUTH_CONTACTS
        counts = {}
        real_save = lt.save_figure

        def spy(fig, p):
            counts[Path(p).stem] = len(fig.axes)
            real_save(fig, p)

        monkeypatch.setattr(lt, "save_figure", spy)
        lt.plot_truth_grf(df, tmp_path / "grf.png")
        lt.plot_truth_mask(df, tmp_path / "mask.png")
        # A row per contact in the force figure, a row per lane (base + contacts) in the mask.
        assert counts == {"grf": len(TRUTH_CONTACTS), "mask": len(TRUTH_CONTACTS) + 1}

    def test_moment_figure_needs_a_finite_moment(self, tmp_path):
        path = write_truth(tmp_path / "locomotion_truth.csv", moments=False)
        stems = render("locomotion_truth", path, tmp_path / "out")
        assert "truth_grm" not in stems
        assert "truth_grf" in stems

    def test_single_contact(self, tmp_path):
        path = write_truth(tmp_path / "locomotion_truth.csv", contacts=["only"])
        assert render("locomotion_truth", path, tmp_path / "out") == ALL_TRUTH

    def test_lines_break_at_a_tick_gap(self, tmp_path):
        from rtc_tools.plotting.plotters import locomotion_truth as lt

        path = write_truth(tmp_path / "locomotion_truth.csv")
        df = load_log_csv(str(path), "locomotion_truth")
        x, y = lt._break_gaps(df, df["truth_base_pz"].to_numpy())
        assert len(x) == len(df) + 1  # one gap (tick 5 has no row)
        assert math.isnan(x[5]) and math.isnan(y[5])

    def test_yaw_is_unwrapped(self):
        import numpy as np

        from rtc_tools.plotting.plotters import locomotion_truth as lt

        yaw = np.linspace(0.0, 4.0 * math.pi, 50)
        z, w = np.sin(yaw / 2), np.cos(yaw / 2)
        out = lt.yaw_rad(np.zeros(50), np.zeros(50), z, w)
        assert out[-1] == pytest.approx(4.0 * math.pi)

    def test_age_of_minus_one_is_not_drawn(self, tmp_path):
        from rtc_tools.plotting.plotters import locomotion_truth as lt

        path = write_truth(tmp_path / "locomotion_truth.csv")
        df = load_log_csv(str(path), "locomotion_truth")
        lt.plot_truth_ages(df, tmp_path / "ages.png")
        assert (tmp_path / "ages.png").stat().st_size > 0
