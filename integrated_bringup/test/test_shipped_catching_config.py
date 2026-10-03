"""The shipped catching config is one tree in four files (MPC plan MD-90).

``demo_catching_controller.yaml`` keeps the QP CLIK step, the bring-up sections
and the key that selects the planner law; ``catching/search_grid.yaml``,
``catching/planner_closed_form.yaml`` and ``catching/planner_mpc.yaml`` hold what
each function is tuned with. The CM merges them before it applies an override.

What can go wrong without an error anywhere:

- a Python tool composes the files differently from the CM, and reads a profile
  the controller does not run with;
- a key lands in the wrong file, and the file stops saying which function it
  belongs to.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest
import yaml

from rtc_tools.utils.controller_config import (
    controller_config_leaf_lines,
    load_controller_config,
)

CONFIG_ROOT = Path(__file__).resolve().parent.parent / "config"
CONTROLLER = "demo_catching_controller"
ROBOTS = ("ur5e_p1b", "iiwa7_leap")
DUMP_ENV = "RTC_CONTROLLER_CONFIG_DUMP"

SEARCH = "catching/search_grid.yaml"
CLOSED_FORM = "catching/planner_closed_form.yaml"
MPC = "catching/planner_mpc.yaml"

# Which keys each fragment owns. A trailing dot is a subtree, anything else is
# one leaf. A key outside every entry belongs to the main file.
FRAGMENT_KEYS = {
    SEARCH: (
        "catching.planner.budget_s",
        "catching.planner.max_ik",
        "catching.planner.n_settle",
        "catching.planner.slice.",
        "catching.planner.time.",
        "catching.planner.unc.",
        "catching.planner.gamma.",
        "catching.planner.rollout.",
        "catching.planner.budget.",
        "catching.planner.score.",
        "catching.planner.workspace.",
        "catching.planner.hand.",
        "catching.planner.ik.",
        "catching.planner.catchability.",
        "catching.robot.arm.qdd_max",
        "catching.robot.arm.qdd_provisional",
    ),
    CLOSED_FORM: (
        "catching.reference.",
        "catching.supervisor.decel.a_dec",
        "catching.planner.switch.",
    ),
    MPC: (
        "catching.planner.decel_mpc.",
        "catching.supervisor.decel.switch_margin",
    ),
}


def _main_path(robot: str) -> Path:
    return CONFIG_ROOT / robot / "controllers" / f"{CONTROLLER}.yaml"


def _leaf_paths(path: Path) -> list[str]:
    """Leaf key paths of ONE file, read on its own (no composition)."""
    tree = yaml.load(path.read_text(), Loader=yaml.BaseLoader)[CONTROLLER]
    return [line.split("\t", 1)[0] for line in controller_config_leaf_lines(tree)]


def _owner(leaf: str) -> str | None:
    for fragment, keys in FRAGMENT_KEYS.items():
        for key in keys:
            if leaf.startswith(key) if key.endswith(".") else leaf == key:
                return fragment
    return None


@pytest.mark.parametrize("robot", ROBOTS)
def test_main_file_includes_the_three_fragments(robot):
    doc = yaml.safe_load(_main_path(robot).read_text())
    assert list(doc) == ["include", CONTROLLER]
    assert doc["include"] == [SEARCH, CLOSED_FORM, MPC]


@pytest.mark.parametrize("robot", ROBOTS)
@pytest.mark.parametrize("fragment", sorted(FRAGMENT_KEYS))
def test_fragment_holds_only_its_functions_keys(robot, fragment):
    leaves = _leaf_paths(_main_path(robot).parent / fragment)
    assert leaves, f"{robot}/{fragment} is empty"
    strays = [leaf for leaf in leaves if _owner(leaf) != fragment]
    assert not strays, f"{robot}/{fragment} holds keys of another function: {strays}"


@pytest.mark.parametrize("robot", ROBOTS)
def test_main_file_holds_no_fragment_key(robot):
    strays = [leaf for leaf in _leaf_paths(_main_path(robot)) if _owner(leaf) is not None]
    assert not strays, f"{robot}: these belong in a fragment: {strays}"


@pytest.mark.parametrize("robot", ROBOTS)
def test_every_fragment_key_group_is_present(robot):
    # An entry of FRAGMENT_KEYS that matches nothing is a group that was renamed
    # or dropped: the two tests above would then pass on a rule nothing obeys.
    for fragment, keys in FRAGMENT_KEYS.items():
        leaves = _leaf_paths(_main_path(robot).parent / fragment)
        for key in keys:
            hit = any(
                leaf.startswith(key) if key.endswith(".") else leaf == key for leaf in leaves
            )
            assert hit, f"{robot}/{fragment}: nothing under '{key}'"


@pytest.mark.parametrize("robot", ROBOTS)
def test_python_composes_the_tree_the_cm_composes(robot):
    dump = os.environ.get(DUMP_ENV)
    if not dump:
        pytest.skip(f"{DUMP_ENV} is set by this package's CMake test registration (colcon test)")
    main = _main_path(robot)
    result = subprocess.run(
        [dump, str(main), CONTROLLER], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stderr
    cpp_lines = result.stdout.splitlines()

    doc = load_controller_config(main, config_key=CONTROLLER, loader=yaml.BaseLoader)
    py_lines = controller_config_leaf_lines(doc[CONTROLLER])

    assert len(cpp_lines) > 100, "the dump is suspiciously small"
    assert py_lines == cpp_lines


@pytest.mark.parametrize("robot", ROBOTS)
def test_the_unit_speed_damping_is_one_number_for_cpp_and_python(robot):
    # The C++ search damps its unit-speed solve with planner.gamma.unit_speed_damping
    # (the shipped-profile C++ test pins it equal to kUnitSpeedDamping). The offline
    # tools must run on the same number: catch_gate_map reads the key from the profile
    # (below), and catch_speed_budget has no profile input at all, so its constant has
    # to equal the shipped value on both robots — drift in either place fails here.
    from rtc_tools.analysis import catch_gate_map, catch_speed_budget  # noqa: PLC0415

    doc = load_controller_config(_main_path(robot), config_key=CONTROLLER, loader=yaml.BaseLoader)
    shipped = float(doc[CONTROLLER]["catching"]["planner"]["gamma"]["unit_speed_damping"])
    assert shipped == 1.0e-3
    assert shipped == catch_speed_budget.DEFAULT_DLS_DAMPING
    assert catch_gate_map.load_unit_speed_damping(_main_path(robot), CONTROLLER) == shipped
