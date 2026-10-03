"""The catching sim overlays must write keys the controller READS.

**The failure this exists for is silent.** The controller config is not a ROS
parameter tree: ``ApplyControllerParamOverrides`` (rtc_controller_manager) pokes
every ``demo_catching_controller.*`` parameter into the YAML the controller
loaded, and creates whatever key is not there. An overlay path one level short
(``demo_catching_controller: joint_cmd: ...`` without ``catching:``) is written
to a key nothing reads, the controller configures, and the run is the shipped
profile — measured 2026-09-24: the startup log read ``commit at t_c − 0.360 s``
(shipped) with no warning anywhere. Every number from such a run would describe
the wrong arm.

So every leaf an overlay writes must name a key the shipped
``demo_catching_controller.yaml`` already has, with the same YAML type, and the
arm-to-arm differences are pinned so each overlay changes exactly what its
header says. Shipped are the operational arms only (the experiment arms of
S8-B/E/F/G/I were pruned once recorded, 2026-09-27): ur5e_p1b ``catch_lead_on``
(the runner's standard arm), ``s8f_reach_first`` (hand-near throws) and
``catch_wait_pose_current`` (the switched-in pose as the wait pose), iiwa7_leap
``catch_lead_on``. The sim robot config's one controller override (the planner's
envelope box, S8-G R2) is pinned here too.
"""

from __future__ import annotations

import math
import os

import pytest
import yaml

from rtc_tools.utils.controller_config import load_controller_config

CONFIG_ROOT = os.path.join(os.path.dirname(__file__), "..", "config")
CONFIG_DIR = os.path.join(CONFIG_ROOT, "ur5e_p1b")
SHIPPED = os.path.join(CONFIG_DIR, "controllers", "demo_catching_controller.yaml")
OVERLAY_DIR = os.path.join(CONFIG_DIR, "sim_overlays")
CONTROLLER = "demo_catching_controller"
LEAD_ON = "catch_lead_on"
LEAD_ON_LEAVES = {
    ("catching", "joint_cmd", "lag", "T_arm"): 0.05,
    ("catching", "joint_cmd", "lag", "lead_enable"): True,
    ("catching", "planner", "freeze", "T_freeze"): 0.37,
}
# S8-F-1 (#537, 2026-09-26): catch_lead_on with the widened catch box and the
# inverted score weights.
REACH_FIRST = "s8f_reach_first"
S8F_BOX = {
    ("catching", "planner", "workspace", "catch_box", "min"): [-1.1, -1.1, 0.15],
    ("catching", "planner", "workspace", "catch_box", "max"): [1.1, 1.1, 1.2],
}
S8F_SCORE = {
    ("catching", "planner", "score", "w_t"): 5.0,
    ("catching", "planner", "score", "w_gamma"): 1.0,
}
# S8-I (#537 5850509543): the reach-first arm with the wait pose taken from
# the arm's switched-in pose.
WAIT_POSE_CURRENT = "catch_wait_pose_current"
WAIT_POSE_SOURCE = ("catching", "planner", "wait_pose_source")
SIM_CONFIG = os.path.join(CONFIG_DIR, "mujoco_simulator.yaml")
LEAP = "iiwa7_leap"
# Where each profile keeps control_rate (the launch files read the same file).
RATE_FILE = {"ur5e_p1b": "_base.yaml", LEAP: "sim.yaml"}


def _load(path: str) -> dict:
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def _shipped_tree(path: str) -> dict:
    """The shipped controller tree as the CM composes it (main file + ``include:`` fragments)."""
    return load_controller_config(path, config_key=CONTROLLER)[CONTROLLER]


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


def _arm(name: str) -> dict:
    return _controller_tree(_load(os.path.join(OVERLAY_DIR, name + ".yaml")))


@pytest.fixture(scope="module")
def shipped() -> dict:
    return _shipped_tree(SHIPPED)


@pytest.fixture(scope="module")
def arms() -> dict[str, dict]:
    return {name: _arm(name) for name in (LEAD_ON, REACH_FIRST, WAIT_POSE_CURRENT)}


@pytest.fixture(scope="module")
def leap_shipped() -> dict:
    return _shipped_tree(
        os.path.join(CONFIG_ROOT, LEAP, "controllers", "demo_catching_controller.yaml")
    )


LEAP_SCENE = "package://robot_descriptions/robots/iiwa7_leap/mjcf/scene_right.xml"


@pytest.fixture(scope="module")
def leap_arm() -> dict:
    """The leap overlay also swaps the sim scene (its header says why); the
    sim section may carry that and nothing else, the rest is the RT node's."""
    overlay = _load(os.path.join(CONFIG_ROOT, LEAP, "sim_overlays", LEAD_ON + ".yaml"))
    sim = overlay.pop("mujoco_simulator")
    assert sim == {"ros__parameters": {"model_path": LEAP_SCENE}}, sim
    return _controller_tree(overlay)


def test_the_shipped_overlays_are_exactly_the_operational_arms():
    """A new experiment arm belongs outside the repo (README §Scene overlay); a
    shipped overlay is one somebody operates with, and it is pinned below."""
    shipped_names = sorted(f[:-5] for f in os.listdir(OVERLAY_DIR) if f.endswith(".yaml"))
    assert shipped_names == sorted(
        [
            LEAD_ON,
            REACH_FIRST,
            WAIT_POSE_CURRENT,
            "fingertip_grasp",
            "fingertip_grasp_free",
            "inference_pole",
        ]
    )
    leap_dir = os.path.join(CONFIG_ROOT, LEAP, "sim_overlays")
    assert sorted(os.listdir(leap_dir)) == [LEAD_ON + ".yaml"]


@pytest.mark.parametrize("name", (LEAD_ON, REACH_FIRST, WAIT_POSE_CURRENT))
def test_every_leaf_is_a_shipped_key_of_the_same_type(name, arms, shipped):
    assert _unread_leaves(arms[name], shipped) == []


def test_the_check_catches_a_path_one_level_short(arms, shipped):
    """Positive control: the measured failure (``catching:`` dropped) must be red.

    Without this the leaf check could pass vacuously — e.g. if the shipped tree
    ever grew a top-level ``joint_cmd`` block for another reason.
    """
    shallow = arms[LEAD_ON]["catching"]
    problems = _unread_leaves(shallow, shipped)
    assert len(problems) == len(_leaves(shallow)), problems


def test_the_check_catches_a_type_change(arms, shipped):
    tree = yaml.safe_load(yaml.safe_dump(arms[LEAD_ON]))
    tree["catching"]["planner"]["freeze"]["T_freeze"] = 1
    assert _unread_leaves(tree, shipped) == [
        "catching.planner.freeze.T_freeze: shipped float, overlay int"
    ]


def test_lead_on_writes_the_lead_and_its_commit_window_only(arms):
    """The arm is the lead compensation of the sim plant's 0.05 s servo lag and
    the T_freeze that lag moves (catch_lead_on.yaml header) — nothing else."""
    assert _leaves(arms[LEAD_ON]) == LEAD_ON_LEAVES


def _check_commit_window(catching: dict, ship: dict, profile: str) -> None:
    """The window keys follow T_arm the way the shipped file derives its own
    (see its ``io.horizon_min`` / ``planner.freeze.T_freeze`` comments):
    T_close,tot = T_close_e2e + h/2, T_freeze >= T_close,tot + T_arm + margin,
    horizon_min >= that + L, n_min = ceil(horizon_min / dt_expected) + 1. A key
    the overlay does not write is the shipped one, and must hold too."""

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

    base = _load(os.path.join(CONFIG_ROOT, profile, RATE_FILE[profile]))
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


@pytest.mark.parametrize("name", (LEAD_ON, REACH_FIRST, WAIT_POSE_CURRENT))
def test_the_commit_window_is_derived_from_t_arm(name, arms, shipped):
    _check_commit_window(arms[name]["catching"], shipped["catching"], "ur5e_p1b")


# ── ur5e_p1b S8-F-1 (hand-near throws) ───────────────────────────────────────


def test_s8f_reach_first_is_lead_on_plus_the_box_and_the_inverted_weights(arms, shipped):
    """The arm differs from catch_lead_on ONLY by the widened box and the two
    score weights — and the weights really are inverted relative to the shipped
    ones, so the arm measures the priority and nothing else."""
    base = _leaves(arms[LEAD_ON])
    leaves = _leaves(arms[REACH_FIRST])
    extra = {k: v for k, v in leaves.items() if k not in base}
    assert extra == {**S8F_BOX, **S8F_SCORE}
    assert {k: v for k, v in leaves.items() if k in base} == base
    ship = shipped["catching"]["planner"]["score"]
    assert (ship["w_t"], ship["w_gamma"]) == (1.0, 5.0), "the shipped weights the arm inverts"
    assert (
        leaves[("catching", "planner", "score", "w_t")],
        leaves[("catching", "planner", "score", "w_gamma")],
    ) == (
        ship["w_gamma"],
        ship["w_t"],
    )


def test_s8f_box_contains_the_shipped_one_and_clears_the_table(shipped):
    ship = shipped["catching"]["planner"]["workspace"]["catch_box"]
    lo = S8F_BOX[("catching", "planner", "workspace", "catch_box", "min")]
    hi = S8F_BOX[("catching", "planner", "workspace", "catch_box", "max")]
    assert all(a <= b for a, b in zip(lo, ship["min"], strict=True))
    assert all(a >= b for a, b in zip(hi, ship["max"], strict=True))
    # The stopping point may not be reserved below the work table (top z 0.05,
    # mujoco_simulator.yaml) — the box floor sits above it.
    assert lo[2] > 0.05


# ── ur5e_p1b S8-I (the switched-in pose as the wait pose) ────────────────────


def test_wait_pose_current_is_reach_first_plus_the_source_leaf(arms, shipped):
    """ONE leaf on top of the reach-first arm: `wait_pose_source: current`. The
    widened box is not optional (the shipped box is the hull around the SHIPPED
    wait pose — another pose's surroundings are outside it), and the shipped
    source must be `yaml` for the arm to change anything."""
    base = _leaves(arms[REACH_FIRST])
    leaves = _leaves(arms[WAIT_POSE_CURRENT])
    assert {k: v for k, v in leaves.items() if k != WAIT_POSE_SOURCE} == base
    assert leaves[WAIT_POSE_SOURCE] == "current"
    assert shipped["catching"]["planner"]["wait_pose_source"] == "yaml"


# ── ur5e_p1b sim.yaml (S8-G R2: the planner's envelope box) ──────────────────


def test_sim_yaml_overrides_the_planner_box_with_float_keys_of_the_shipped_type(shipped):
    """The sim robot config overrides the box with ONE array and its flag — and
    only those — at shipped keys of the same type. Every element is a float: a
    ROS parameter array has one type, so an integer literal would turn the whole
    override into an integer array. The real robot.yaml must not carry it."""
    sim = _load(os.path.join(CONFIG_DIR, "sim.yaml"))["/**"]["ros__parameters"]
    tree = sim[CONTROLLER]
    arm = ("catching", "robot", "arm")
    assert set(_leaves(tree)) == {arm + ("qdd_max",), arm + ("qdd_provisional",)}
    assert _unread_leaves(tree, shipped) == []
    box = tree["catching"]["robot"]["arm"]["qdd_max"]
    assert box == [20.2531, 30.859, 36.9851, 21.5036, 14.2481, 29.0196]
    assert all(isinstance(v, float) for v in box)
    assert tree["catching"]["robot"]["arm"]["qdd_provisional"] is True
    shipped_box = shipped["catching"]["robot"]["arm"]["qdd_max"]
    assert len(box) == len(shipped_box)
    # The point of the override: the envelope is well above the shipped derived box.
    assert all(e > 2 * s for e, s in zip(box, shipped_box, strict=True))
    robot = _load(os.path.join(CONFIG_DIR, "robot.yaml"))["/**"]["ros__parameters"]
    assert CONTROLLER not in robot, "the real arm keeps the shipped box (S10 measures its own)"


# ── iiwa7_leap (S8-D) ────────────────────────────────────────────────────────


def test_leap_every_leaf_is_a_shipped_key_of_the_same_type(leap_arm, leap_shipped):
    assert _unread_leaves(leap_arm, leap_shipped) == []


def test_leap_check_catches_a_path_one_level_short(leap_arm, leap_shipped):
    shallow = leap_arm["catching"]
    problems = _unread_leaves(shallow, leap_shipped)
    assert len(problems) == len(_leaves(shallow)), problems


def test_leap_commit_window_is_derived_from_t_arm(leap_arm, leap_shipped):
    """The leap overlay writes no T_freeze: the shipped 0.19 must already cover T_arm."""
    _check_commit_window(leap_arm["catching"], leap_shipped["catching"], LEAP)


def test_leap_arm_leads_the_sim_plant(leap_arm):
    assert _leaves(leap_arm) == {
        ("catching", "joint_cmd", "lag", "T_arm"): 0.05,
        ("catching", "joint_cmd", "lag", "lead_enable"): True,
    }
