"""catching_keys — the #711 moved-key table and the mirror alias table."""

from __future__ import annotations

import json
import re
from pathlib import Path

import pytest

from rtc_tools.analysis import catching_trials as ct
from rtc_tools.utils import catching_keys as ck

_REPO = Path(__file__).resolve().parents[2]
_PARAMS_HPP = _REPO / "rtc_controllers/include/rtc_controllers/catching/catching_params.hpp"
_LIFECYCLE_CPP = _REPO / "integrated_bringup/src/controllers/catching/lifecycle.cpp"


def _cpp_renamed_keys(header: Path) -> list[tuple[str, str]]:
    """The ``{"old", "new"}`` rows of ``kRenamedCatchingKeys`` in the C++ header."""
    body = header.read_text().split("kRenamedCatchingKeys{{", 1)[1].split("}};", 1)[0]
    return re.findall(r'\{"([^"]+)",\s*"([^"]+)"\}', body)


def test_the_python_moved_key_table_is_the_cpp_one():
    if not _PARAMS_HPP.exists():
        pytest.skip("C++ header is not beside this checkout")
    cpp = _cpp_renamed_keys(_PARAMS_HPP)
    assert len(cpp) == 18
    assert list(ck.RENAMED_CATCHING_KEYS) == cpp


def test_every_renamed_mirror_is_declared_under_its_new_name_by_the_controller():
    if not _LIFECYCLE_CPP.exists():
        pytest.skip("C++ source is not beside this checkout")
    text = _LIFECYCLE_CPP.read_text()
    assert len(ck.RENAMED_MIRRORS) == 45
    declared = set(re.findall(r'declare\(\s*"([^"]+)"', text))
    declared |= set(re.findall(r'^\s*"((?:planner|supervisor)\.[A-Za-z0-9_.]+)",', text, re.M))
    assert set(ck.RENAMED_MIRRORS.values()) <= declared
    assert not set(ck.RENAMED_MIRRORS) & declared  # no old name is declared any more


def test_every_mirror_alias_is_the_key_map_applied_to_the_old_name():
    """A mirror name is a catching key path; the alias target is that path's new spelling."""
    moved = dict(ck.RENAMED_CATCHING_KEYS)

    def rename(path: str) -> str:
        for old, new in moved.items():
            if path == old or path.startswith(old + "."):
                return new + path[len(old) :]
        return path

    for old, new in ck.RENAMED_MIRRORS.items():
        assert rename(old) == new, old


@pytest.mark.parametrize(("old", "new"), ck.RENAMED_CATCHING_KEYS)
def test_each_old_key_is_refused_naming_both_paths(old, new):
    tree: dict = {}
    node = tree
    parts = old.split(".")
    for part in parts[:-1]:
        node = node.setdefault(part, {})
    node[parts[-1]] = {"x": 1}
    with pytest.raises(ck.RenamedCatchingKeyError) as exc:
        ck.reject_renamed_keys(tree, source="p.yaml")
    assert f"catching.{old} → catching.{new}" in str(exc.value)
    assert "p.yaml" in str(exc.value)


@pytest.mark.parametrize("value", [None, 0, {}, "TBD", [1, 2]])
def test_an_old_key_is_refused_with_any_value(value):
    with pytest.raises(ck.RenamedCatchingKeyError):
        ck.reject_renamed_keys({"planner": {"slice": value}})


def test_supervisor_decel_is_judged_leaf_by_leaf():
    ck.reject_renamed_keys({"supervisor": {"decel": {"a_dec": 5.0}}})
    ck.reject_renamed_keys({"supervisor": {"decel": None}})
    with pytest.raises(ck.RenamedCatchingKeyError, match="supervisor.decel.switch_margin"):
        ck.reject_renamed_keys({"supervisor": {"decel": {"a_dec": 5.0, "switch_margin": 1}}})


def test_the_new_layout_and_the_unmoved_keys_pass():
    ck.reject_renamed_keys(
        {
            "reference": {"omega": 10.0},
            "planner": {
                "enabled": True,
                "wait_pose": [0.0],
                "freeze": {"T_freeze": 0.1},
                "segment": {"mode": "mpc", "mpc": {"horizon": {"n_nodes": 7}}},
                "search": {"grid": {"slice": {"dt": 0.05}, "reference": {"v_max": 3.0}}},
            },
            "robot": {"arm": {"qdd_max": [1.0]}},
        }
    )
    ck.reject_renamed_keys(None)
    ck.reject_renamed_keys({"planner": None})
    ck.reject_renamed_keys({"planner": [1]})


def test_the_shipped_profiles_hold_no_old_key():
    from rtc_tools.utils.controller_config import load_controller_config

    for robot in ("ur5e_p1b", "iiwa7_leap"):
        path = (
            _REPO / f"integrated_bringup/config/{robot}/controllers/demo_catching_controller.yaml"
        )
        if not path.exists():
            pytest.skip("integrated_bringup config is not beside this checkout")
        key = "demo_catching_controller"
        ck.reject_renamed_keys(load_controller_config(path, config_key=key)[key]["catching"])


# ── mirror names ────────────────────────────────────────────────────────────
def test_an_old_mirror_name_reads_as_the_new_one_and_other_names_pass():
    old = {
        "planner.decel_mpc.budget.first_s": 0.2,
        "supervisor.decel.mode": "mpc",
        "planner.slice.dt": 0.05,
        "control.dt": 0.002,
        "reference.omega": 10.0,
    }
    assert ck.normalize_mirror(old) == {
        "planner.segment.mpc.budget.first_s": 0.2,
        "planner.segment.mode": "mpc",
        "planner.search.grid.slice.dt": 0.05,
        "control.dt": 0.002,
        "reference.omega": 10.0,
    }


def test_a_new_mirror_name_is_kept_as_it_is():
    new = {"planner.segment.mode": "mpc", "planner.search.grid.gamma.eta_v": 0.9}
    assert ck.normalize_mirror(new) == new


@pytest.mark.parametrize(("old", "new"), sorted(ck.RENAMED_MIRRORS.items()))
def test_a_mirror_with_an_old_name_and_its_new_name_is_refused(old, new):
    with pytest.raises(ck.MixedMirrorNamesError) as exc:
        ck.normalize_mirror({old: 1, new: 2}, source="mirror.txt")
    assert old in str(exc.value) and new in str(exc.value)


def test_an_old_name_beside_another_keys_new_name_is_refused():
    # No controller writes this: one build mirrors every key under the old names,
    # the other under the new ones.
    with pytest.raises(ck.MixedMirrorNamesError) as exc:
        ck.normalize_mirror({"planner.slice.dt": 0.05, "planner.segment.mode": "mpc"})
    assert "planner.slice.dt" in str(exc.value) and "planner.segment.mode" in str(exc.value)
    # A new-only name that has no old spelling does not make a record "new".
    assert ck.normalize_mirror({"planner.slice.dt": 0.05, "control.dt": 0.002}) == {
        "planner.search.grid.slice.dt": 0.05,
        "control.dt": 0.002,
    }


def test_run_meta_is_read_under_the_new_names_and_a_mixed_one_is_refused(tmp_path):
    path = tmp_path / "run_meta.json"
    path.write_text(json.dumps({"arm": "a", "controller_mirror": {"planner.slice.dt": 0.05}}))
    assert ct.load_run_meta(path) == {
        "arm": "a",
        "controller_mirror": {"planner.search.grid.slice.dt": 0.05},
    }
    path.write_text(
        json.dumps(
            {
                "controller_mirror": {
                    "planner.slice.dt": 0.05,
                    "planner.search.grid.slice.dt": 0.05,
                }
            }
        )
    )
    with pytest.raises(ck.MixedMirrorNamesError):
        ct.load_run_meta(path)


# ── The two controller CSVs' renamed columns ────────────────────────────────

_LOGGING = _REPO / "integrated_bringup/include/integrated_bringup/logging"


def _cpp_header_columns(header: Path, function: str) -> list[str]:
    """The column names ``function`` (a header writer) emits, from its string literals.
    Adjacent literals of one statement are one (the compiler concatenates them)."""
    body = header.read_text().split(function, 1)[1].split("\n}", 1)[0]
    statements = re.findall(r'os << ((?:"[^"]*"\s*)+)', body)
    joined = "".join(re.findall(r'"([^"]*)"', "".join(statements))).replace("\\n", "")
    return [c for c in joined.split(",") if c]


def _cpp_segment_columns() -> tuple[list[str], list[str]]:
    diag = _LOGGING / "catching_diag_log_pod.hpp"
    events = _LOGGING / "planner_events_csv.hpp"
    if not diag.exists() or not events.exists():
        pytest.skip("C++ logging headers are not beside this checkout")
    return (
        _cpp_header_columns(diag, "const CatchingDiagLogColumns& cols) {"),
        _cpp_header_columns(events, "WritePlannerEventsHeader(std::ostream& os) {"),
    )


# `segment_*` columns added AFTER the rename (E1-F18, #744: the docking core's
# account of a solve). They were never written under a `decel_*` name, so they
# have no alias — and a `segment_*` column added from here on has to be named
# here for the test below to pass.
_SEGMENT_COLUMNS_ADDED_AFTER_THE_RENAME = (
    "segment_qp_solves",
    "segment_qp_iterations",
    "segment_backtracks",
    "segment_mu_updates",
    "segment_cut_site",
    "segment_start_from_memory",
    "segment_start_us",
    "segment_linearize_us",
    "segment_assemble_us",
    "segment_qp_us",
    "segment_merit_us",
    "segment_kkt_residual",
    "segment_grad_norm",
    "segment_complementarity",
    "segment_infeasible_group",
    "segment_viol_torque",
    "segment_viol_gap",
    "segment_viol_entrance",
    "segment_viol_lateral",
    "segment_viol_timing",
    "segment_viol_velocity_set",
    "segment_viol_impact",
    "segment_viol_box",
    "segment_viol_terminal",
    "segment_elastic_torque",
    "segment_elastic_gap",
    "segment_elastic_entrance",
    "segment_elastic_lateral",
    "segment_elastic_timing",
    "segment_elastic_velocity_set",
    "segment_elastic_impact",
    "segment_c_catch",
    "segment_c_guarded",
    "segment_sigma_s",
    "segment_sigma_t",
    "segment_chance_lateral",
    "segment_chance_timing",
    "segment_cost_reference",
    "segment_cost_stop",
)


def test_the_column_alias_targets_are_the_segment_columns_the_cpp_headers_write():
    diag, events = _cpp_segment_columns()
    cpp_new = {c for c in [*diag, *events] if c.startswith("segment_")}
    added = set(_SEGMENT_COLUMNS_ADDED_AFTER_THE_RENAME)
    assert len(added) == len(_SEGMENT_COLUMNS_ADDED_AFTER_THE_RENAME) == 37
    assert added <= cpp_new
    assert len(ck.RENAMED_COLUMNS) == 49
    # Every renamed column, and nothing else: what the headers write beyond the
    # alias targets is exactly the columns added after the rename.
    assert set(ck.RENAMED_COLUMNS.values()) == cpp_new - added
    # 17 + 33 renamed columns, `segment_seq` is in both files; the events file
    # has the 37 added ones as well.
    assert len([c for c in diag if c.startswith("segment_")]) == 17
    assert len([c for c in events if c.startswith("segment_")]) == 33 + 37
    # No header writes an old name any more; no old name is also a new one.
    assert not [c for c in [*diag, *events] if c in ck.RENAMED_COLUMNS]
    assert not set(ck.RENAMED_COLUMNS) & set(ck.RENAMED_COLUMNS.values())


def test_each_column_alias_is_the_old_name_with_segment_for_decel():
    for old, new in ck.RENAMED_COLUMNS.items():
        expected = (
            "segment_x0_from_segment"
            if old == "decel_from_segment"
            else old.replace("decel_", "segment_")
        )
        assert new == expected


def test_the_trials_output_aliases_are_the_names_catching_trials_writes():
    assert set(ck.RENAMED_TRIALS_COLUMNS.values()) == set(ct.SEGMENT_LANE_KEYS)
    assert len(ck.RENAMED_TRIALS_COLUMNS) == len(ct.SEGMENT_LANE_KEYS) == 12
    assert ck.RENAMED_TRIALS_COLUMNS["decel_segments_followed"] == "segment_n_followed"
    assert not set(ck.RENAMED_TRIALS_COLUMNS) & set(ct.SEGMENT_LANE_KEYS)
    assert ck.RENAMED_TRIALS_SUMMARY_KEYS == {"decel_lane": "segment_lane"}


def test_old_column_names_are_renamed_and_the_others_are_left():
    header = ["t_relative_s", "decel_event", "decel_seq", "q_cmd_a", "decel_from_segment"]
    assert ck.column_renames(header) == {
        "decel_event": "segment_event",
        "decel_seq": "segment_seq",
        "decel_from_segment": "segment_x0_from_segment",
    }
    assert ck.normalize_columns(header) == [
        "t_relative_s",
        "segment_event",
        "segment_seq",
        "q_cmd_a",
        "segment_x0_from_segment",
    ]
    new = ["t_relative_s", "segment_event", "q_cmd_a"]
    assert ck.column_renames(new) == {} and ck.normalize_columns(new) == new


@pytest.mark.parametrize(
    "header",
    [
        ["decel_event", "segment_event"],  # one column under both names
        ["decel_event", "segment_rho"],  # an old column and another column's new name
        ["segment_judged", "decel_seq"],
    ],
)
def test_a_header_with_an_old_and_a_new_column_name_is_refused_naming_both(header):
    with pytest.raises(ck.MixedColumnNamesError) as exc:
        ck.column_renames(header, source="catching_diag.csv")
    msg = str(exc.value)
    assert "catching_diag.csv" in msg
    assert [c for c in header if c.startswith("decel_")][0] in msg
    assert [c for c in header if c.startswith("segment_")][0] in msg
    with pytest.raises(ck.MixedColumnNamesError):
        ck.normalize_columns(header)


def test_usecols_name_the_new_columns_whatever_the_file_has(tmp_path):
    pd = pytest.importorskip("pandas")
    old = tmp_path / "old.csv"
    old.write_text("t,decel_event,decel_rho,q\n0,1,0.5,7\n1,4,0.25,8\n")
    new = tmp_path / "new.csv"
    new.write_text("t,segment_event,segment_rho,q\n0,1,0.5,7\n1,4,0.25,8\n")
    for usecols in (None, ["segment_event", "t"], lambda c: c in {"segment_rho", "q"}):
        a = ck.read_csv_normalized(pd.read_csv, old, usecols=usecols)
        b = ck.read_csv_normalized(pd.read_csv, new, usecols=usecols)
        assert list(a.columns) == list(b.columns)
        assert a.equals(b)
    assert "segment_event" in ck.read_csv_normalized(pd.read_csv, old).columns
    mixed = tmp_path / "mixed.csv"
    mixed.write_text("t,decel_event,segment_rho\n0,1,0.5\n")
    with pytest.raises(ck.MixedColumnNamesError, match="mixed.csv"):
        ck.read_csv_normalized(pd.read_csv, mixed)


def test_an_old_tool_output_is_refused_naming_the_old_and_the_new_name():
    ck.reject_old_output_names(
        ["idx", "segment_aged"], ck.RENAMED_TRIALS_COLUMNS, source="x.csv", tool="catching_trials"
    )
    with pytest.raises(ck.OldToolOutputError) as exc:
        ck.reject_old_output_names(
            ["idx", "decel_aged", "decel_segments_followed"],
            ck.RENAMED_TRIALS_COLUMNS,
            source="x.csv",
            tool="catching_trials",
        )
    msg = str(exc.value)
    assert "x.csv" in msg and "older catching_trials" in msg and "regenerate" in msg
    assert "decel_aged -> segment_aged" in msg
    assert "decel_segments_followed -> segment_n_followed" in msg
