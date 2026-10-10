"""The shipped catching config is one tree in six files (MD-90, E1-F16).

``demo_catching_controller.yaml`` keeps the QP CLIK step, the bring-up sections
(the hand's measured values among them), the arm's acceleration box and the two
keys that select the search and the segment mode; five fragments under
``catching/`` hold what each function is tuned with — the two searches
(``search_grid.yaml``, ``search_nlp.yaml``), the closed_form law
(``planner_closed_form.yaml``) and the two segment planners
(``segment_mpc.yaml``, ``segment_mpc_docking.yaml``). The CM merges them before
it applies an override.

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
SEARCH_NLP = "catching/search_nlp.yaml"
CLOSED_FORM = "catching/planner_closed_form.yaml"
MPC = "catching/segment_mpc.yaml"
MPC_DOCKING = "catching/segment_mpc_docking.yaml"

# Which keys each fragment owns: a function's whole map (#711). A trailing dot
# is a subtree, anything else is one leaf. A key outside every entry belongs to
# the main file — the two selectors (`catching.planner.search.mode`,
# `catching.planner.segment.mode`) and the hand's measured values
# (`catching.robot.hand.*`) among them. The dots matter: `planner.search.` would
# claim the search selector for the grid search, and `planner.segment.mpc`
# without one would hand the docking planner's map to the mpc planner.
FRAGMENT_KEYS = {
    SEARCH: ("catching.planner.search.grid.",),
    SEARCH_NLP: ("catching.planner.search.nlp.",),
    CLOSED_FORM: (
        "catching.reference.",
        "catching.supervisor.decel.a_dec",
    ),
    MPC: ("catching.planner.segment.mpc.",),
    MPC_DOCKING: ("catching.planner.segment.mpc_docking.",),
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
def test_main_file_includes_the_five_fragments(robot):
    doc = yaml.safe_load(_main_path(robot).read_text())
    assert list(doc) == ["include", CONTROLLER]
    assert doc["include"] == [SEARCH, SEARCH_NLP, CLOSED_FORM, MPC, MPC_DOCKING]


@pytest.mark.parametrize("robot", ROBOTS)
def test_the_selectors_and_the_hands_measured_values_are_the_main_files(robot):
    leaves = _leaf_paths(_main_path(robot))
    for key in (
        "catching.planner.search.mode",
        "catching.planner.segment.mode",
        "catching.robot.hand.T_close_e2e",
        "catching.robot.hand.T_close_lead",
        "catching.robot.hand.docking.s_ent",
        "catching.robot.hand.docking.provisional",
    ):
        assert key in leaves, f"{robot}: '{key}' is not in the main file"
    # What is measured on the hand is in no function's fragment: a second copy
    # there is how two functions come to run on two hands.
    for fragment in FRAGMENT_KEYS:
        strays = [
            leaf
            for leaf in _leaf_paths(_main_path(robot).parent / fragment)
            if leaf.split(".")[-1] in HAND_MEASURED or ".robot.hand." in leaf
        ]
        assert not strays, f"{robot}/{fragment} holds the hand's values: {strays}"


# The docking core's fields that are the hand's (robot.hand.docking) or derived
# from it — never keys of a function's `core:` map.
HAND_MEASURED = {
    "s_ent",
    "r_ent",
    "tan_theta",
    "n_faces",
    "faces_a",
    "faces_b",
    "rho_ref",
    "c_min",
    "c_cap_max",
    "c_ent_max",
    "v_perp_max",
    "a_brake",
    "delta_lo",
    "delta_hi",
    "delta_0",
    "sigma_tau",
    "contact_point_hand",
    "restitution",
    "e_max",
    "p_max",
    "m_ball",
}


@pytest.mark.parametrize("robot", ROBOTS)
@pytest.mark.parametrize("fragment", (SEARCH_NLP, MPC_DOCKING))
def test_a_docking_fragment_says_it_is_provisional_and_sim_only(robot, fragment):
    head = []
    for line in (_main_path(robot).parent / fragment).read_text().splitlines():
        if not line.startswith("#"):
            break
        head.append(line)
    text = "\n".join(head)
    assert "PROVISIONAL" in text and "SIM ONLY" in text, f"{robot}/{fragment}"
    assert "robot.hand.docking.provisional" in text, (
        f"{robot}/{fragment}: the hardware condition is not named"
    )


@pytest.mark.parametrize("robot", ROBOTS)
def test_the_two_docking_functions_share_one_grid(robot):
    # Under nlp x mpc_docking the planner republishes the search's solution,
    # which needs one grid: shipped equal, so that pair runs as shipped.
    doc = load_controller_config(_main_path(robot), config_key=CONTROLLER)
    tree = doc[CONTROLLER]["catching"]["planner"]
    nlp, docking = tree["search"]["nlp"], tree["segment"]["mpc_docking"]
    assert nlp["dt_pre_s"] == docking["approach"]["dt_pre_s"]
    assert nlp["stop"] == docking["stop"]
    assert nlp["n_pre"]["max"] <= docking["approach"]["n_pre_max"]
    assert nlp["continuous_tc"] is False
    assert nlp["core"] == docking["core"], f"{robot}: the two cores are tuned apart"


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
    # The C++ search damps its unit-speed solve with
    # planner.search.grid.gamma.unit_speed_damping
    # (the shipped-profile C++ test pins it equal to kUnitSpeedDamping). The offline
    # tools must run on the same number: catch_gate_map reads the key from the profile
    # (below), and catch_speed_budget has no profile input at all, so its constant has
    # to equal the shipped value on both robots — drift in either place fails here.
    from rtc_tools.analysis import catch_gate_map, catch_speed_budget  # noqa: PLC0415

    doc = load_controller_config(_main_path(robot), config_key=CONTROLLER, loader=yaml.BaseLoader)
    grid = doc[CONTROLLER]["catching"]["planner"]["search"]["grid"]
    shipped = float(grid["gamma"]["unit_speed_damping"])
    assert shipped == 1.0e-3
    assert shipped == catch_speed_budget.DEFAULT_DLS_DAMPING
    assert catch_gate_map.load_unit_speed_damping(_main_path(robot), CONTROLLER) == shipped
