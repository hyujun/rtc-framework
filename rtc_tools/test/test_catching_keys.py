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


def test_a_composed_controller_config_is_judged_under_each_controller():
    ck.reject_renamed_keys_in_config({"ctl": {"catching": {"planner": {"wait_pose": [0.0]}}}})
    with pytest.raises(ck.RenamedCatchingKeyError, match=r"\[ctl\]"):
        ck.reject_renamed_keys_in_config({"ctl": {"catching": {"planner": {"hand": {}}}}})


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
