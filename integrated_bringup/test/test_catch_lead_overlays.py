"""The S8-B ``catch_lead_*`` sim overlays must write keys the controller READS.

**The failure this exists for is silent.** The controller config is not a ROS
parameter tree: ``ApplyControllerParamOverrides`` (rtc_controller_manager) pokes
every ``demo_catching_controller.*`` parameter into the YAML the controller
loaded, and creates whatever key is not there. An overlay path one level short
(``demo_catching_controller: joint_cmd: ...`` without ``catching:``) is written
to a key nothing reads, the controller configures, and the run is the shipped
profile — measured 2026-09-24: the startup log read ``commit at t_c − 0.360 s``
(shipped) with no warning anywhere. Every lead-ablation number from such a run
would describe the wrong arm.

So every leaf an overlay writes must name a key the shipped
``demo_catching_controller.yaml`` already has, with the same YAML type. The
arm-to-arm differences are pinned too, because the ablation is only an ablation
if lead (or the γ grid) is the one thing that changes.
"""

from __future__ import annotations

import math
import os

import pytest
import yaml

CONFIG_DIR = os.path.join(os.path.dirname(__file__), "..", "config", "ur5e_p1b")
SHIPPED = os.path.join(CONFIG_DIR, "controllers", "demo_catching_controller.yaml")
OVERLAY_DIR = os.path.join(CONFIG_DIR, "sim_overlays")
CONTROLLER = "demo_catching_controller"
ARMS = {
    "catch_lead_on": {"lead_enable": True, "gamma0": False},
    "catch_lead_off": {"lead_enable": False, "gamma0": False},
    "catch_lead_on_gamma0": {"lead_enable": True, "gamma0": True},
    "catch_lead_off_gamma0": {"lead_enable": False, "gamma0": True},
}


def _load(path: str) -> dict:
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def _controller_tree(overlay: dict) -> dict:
    """The subtree the RT node turns into ``demo_catching_controller.*`` params."""
    assert list(overlay) == ["integrated_rt_controller"], "only the RT node's section"
    params = overlay["integrated_rt_controller"]["ros__parameters"]
    assert list(params) == [CONTROLLER], "only the catching controller's keys"
    return params[CONTROLLER]


def _leaves(tree: dict, prefix: tuple[str, ...] = ()) -> dict[tuple[str, ...], object]:
    out: dict[tuple[str, ...], object] = {}
    for key, value in tree.items():
        path = prefix + (str(key),)
        if isinstance(value, dict):
            out.update(_leaves(value, path))
        else:
            out[path] = value
    return out


def _yaml_kind(value: object) -> str:
    # bool first: bool is an int subclass, and a bool written as 1 is exactly
    # the kind of type drift a params file can carry.
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int):
        return "int"
    if isinstance(value, float):
        return "float"
    if isinstance(value, list):
        return "list[" + ",".join(sorted({_yaml_kind(v) for v in value})) + "]"
    return type(value).__name__


def _unread_leaves(tree: dict, shipped: dict) -> list[str]:
    """Overlay leaves that name no shipped key, or a key of a different type."""
    problems = []
    for path, value in _leaves(tree).items():
        node = shipped
        for part in path:
            if not isinstance(node, dict) or part not in node:
                problems.append(".".join(path) + ": not in the shipped YAML")
                break
            node = node[part]
        else:
            if _yaml_kind(node) != _yaml_kind(value):
                problems.append(
                    f"{'.'.join(path)}: shipped {_yaml_kind(node)}, overlay {_yaml_kind(value)}"
                )
    return problems


@pytest.fixture(scope="module")
def shipped() -> dict:
    return _load(SHIPPED)[CONTROLLER]


@pytest.fixture(scope="module")
def arms() -> dict[str, dict]:
    return {
        name: _controller_tree(_load(os.path.join(OVERLAY_DIR, name + ".yaml"))) for name in ARMS
    }


@pytest.mark.parametrize("name", sorted(ARMS))
def test_every_leaf_is_a_shipped_key_of_the_same_type(name, arms, shipped):
    assert _unread_leaves(arms[name], shipped) == []


def test_the_check_catches_a_path_one_level_short(arms, shipped):
    """Positive control: the measured failure (``catching:`` dropped) must be red.

    Without this the leaf check could pass vacuously — e.g. if the shipped tree
    ever grew a top-level ``joint_cmd`` block for another reason.
    """
    shallow = arms["catch_lead_on"]["catching"]
    problems = _unread_leaves(shallow, shipped)
    assert len(problems) == len(_leaves(shallow)), problems


def test_the_check_catches_a_type_change(arms, shipped):
    tree = yaml.safe_load(yaml.safe_dump(arms["catch_lead_on"]))
    tree["catching"]["planner"]["freeze"]["T_freeze"] = 1
    assert _unread_leaves(tree, shipped) == [
        "catching.planner.freeze.T_freeze: shipped float, overlay int"
    ]


def test_lead_and_the_gamma_policy_are_the_only_differences(arms):
    """γ 0 needs the grid AND d_eff (see catch_lead_on_gamma0.yaml): a grid of
    [0.0] alone is replaced by the γ window's floor, which is not 0."""
    base = _leaves(arms["catch_lead_on"])
    lead = ("catching", "joint_cmd", "lag", "lead_enable")
    gamma0 = {
        ("catching", "planner", "gamma", "grid"): [0.0],
        ("catching", "planner", "hand", "d_eff"): 10.0,
    }
    for name, arm in ARMS.items():
        leaves = _leaves(arms[name])
        assert leaves[lead] is arm["lead_enable"], name
        policy = {k: leaves[k] for k in gamma0 if k in leaves}
        assert policy == (gamma0 if arm["gamma0"] else {}), name
        rest = {k: v for k, v in leaves.items() if k != lead and k not in gamma0}
        assert rest == {k: v for k, v in base.items() if k != lead}, name


def test_the_commit_window_is_derived_from_t_arm(arms, shipped):
    """The window keys follow T_arm the way the shipped file derives its own
    (see its ``io.horizon_min`` / ``planner.freeze.T_freeze`` comments):
    T_close,tot = T_close_e2e + h/2, T_freeze >= T_close,tot + T_arm + margin,
    horizon_min >= that + L, n_min = ceil(horizon_min / dt_expected) + 1. A key
    the overlay does not write is the shipped one, and must hold too."""
    catching = arms["catch_lead_on"]["catching"]
    ship = shipped["catching"]

    def effective(*path):
        node = catching
        for part in path:
            if not isinstance(node, dict) or part not in node:
                node = None
                break
            node = node[part]
        if node is not None:
            return node
        node = ship
        for part in path:
            node = node[part]
        return node

    base = _load(os.path.join(CONFIG_DIR, "_base.yaml"))
    h = 1.0 / base["/**"]["ros__parameters"]["control_rate"]
    t_close_tot = ship["robot"]["hand"]["T_close_e2e"] + h / 2
    t_arm = effective("joint_cmd", "lag", "T_arm")
    margin = effective("planner", "time", "margin")
    assert effective("planner", "freeze", "T_freeze") >= t_close_tot + t_arm + margin
    # L is the shipped comment's 0.14 (io.horizon_min: "... + L 0.14").
    horizon = effective("io", "horizon_min")
    assert horizon >= t_close_tot + t_arm + margin + 0.14 - 1e-9
    # The 1e-9 absorbs a quotient landing a hair above an integer in binary.
    dt = effective("prediction", "dt_expected")
    assert effective("io", "n_min") == math.ceil(horizon / dt - 1e-9) + 1
