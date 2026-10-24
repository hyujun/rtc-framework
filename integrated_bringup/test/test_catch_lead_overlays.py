"""A catching override must write keys the controller READS, and the shipped lead is pinned.

**The failure this exists for is silent.** The controller config is not a ROS
parameter tree: ``ApplyControllerParamOverrides`` (rtc_controller_manager) pokes
every ``demo_catching_controller.*`` parameter into the YAML the controller
loaded, and creates whatever key is not there. An override path one level short
(``demo_catching_controller: joint_cmd: ...`` without ``catching:``) is written
to a key nothing reads, the controller configures, and the run is the shipped
profile — measured 2026-09-24: the startup log read ``commit at t_c − 0.360 s``
(shipped) with no warning anywhere. Every number from such a run would describe
the wrong arm. Since #574 the configure warns about such a path
(``[param-override]``) but does not refuse it, so this check stays the gate.

So every leaf an override writes must name a key the shipped
``demo_catching_controller.yaml`` already has, with the same YAML type
(``rtc_tools.utils.controller_config.unread_leaves`` — ``make_conditions.py`` uses
the same rule). What ships of that kind is one thing: the sim
robot config's controller override (the planner's envelope box), pinned
here.

No catching overlay ships any more. The sim arm's lead — the ``catch_lead_on``
overlays of both robots, the runner's standard arm until #821 — is in the
shipped controller YAML, and the tests that read those files went with them
(2026-10-12). What they held is held on the shipped file below: the lag block,
the commit window derived from ``T_arm``, and ur5e_p1b's ``T_freeze``. The
earlier experiment arms were pruned (2026-09-27, 2026-10-09).
"""

from __future__ import annotations

import math
import os

import pytest
import yaml

from rtc_tools.utils.controller_config import (
    load_controller_config,
    tree_leaves,
    unread_leaves,
    yaml_kind,
)

CONFIG_ROOT = os.path.join(os.path.dirname(__file__), "..", "config")
CONFIG_DIR = os.path.join(CONFIG_ROOT, "ur5e_p1b")
SHIPPED = os.path.join(CONFIG_DIR, "controllers", "demo_catching_controller.yaml")
OVERLAY_DIR = os.path.join(CONFIG_DIR, "sim_overlays")
CONTROLLER = "demo_catching_controller"
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


# The leaf rule lives in rtc_tools (make_conditions.py uses it too);
# the names below are what this file's tests and the launch-argument test call it.
_leaves = tree_leaves
_yaml_kind = yaml_kind
_unread_leaves = unread_leaves


@pytest.fixture(scope="module")
def shipped() -> dict:
    return _shipped_tree(SHIPPED)


@pytest.fixture(scope="module")
def sim_override() -> dict:
    """The controller subtree ur5e_p1b's sim robot config overrides (pinned below)."""
    return _load(os.path.join(CONFIG_DIR, "sim.yaml"))["/**"]["ros__parameters"][CONTROLLER]


def test_the_shipped_overlays_are_exactly_the_operational_arms():
    """A new experiment arm belongs outside the repo (README §Scene overlay); a
    shipped overlay is one somebody operates with. No catching overlay is one:
    an arm of the catching controller is two launch arguments (#821)."""
    shipped_names = sorted(f[:-5] for f in os.listdir(OVERLAY_DIR) if f.endswith(".yaml"))
    assert shipped_names == sorted(
        [
            "fingertip_grasp",
            "fingertip_grasp_free",
            "inference_pole",
        ]
    )
    assert not os.path.exists(os.path.join(CONFIG_ROOT, LEAP, "sim_overlays"))


def test_the_check_catches_a_path_one_level_short(sim_override, shipped):
    """Positive control: the measured failure (``catching:`` dropped) must be red.

    Without this the leaf check could pass vacuously — e.g. if the shipped tree
    ever grew a top-level ``robot`` block for another reason.
    """
    shallow = sim_override["catching"]
    problems = _unread_leaves(shallow, shipped)
    assert len(problems) == len(_leaves(shallow)), problems


def test_the_check_catches_a_type_change(sim_override, shipped):
    tree = yaml.safe_load(yaml.safe_dump(sim_override))
    tree["catching"]["robot"]["arm"]["qdd_provisional"] = 1
    assert _unread_leaves(tree, shipped) == [
        "catching.robot.arm.qdd_provisional: shipped bool, overlay int"
    ]


def _check_commit_window(catching: dict, ship: dict, profile: str) -> None:
    """``catching`` is an override's subtree ({} = the shipped file alone), ``ship``
    the shipped one. The window keys follow T_arm the way the shipped file derives its own
    (see its ``io.horizon_min`` / ``planner.freeze.T_freeze`` comments):
    T_close,tot = T_close + h/2, T_freeze >= T_close,tot + T_arm + margin,
    horizon_min >= that + L, n_min = ceil(horizon_min / dt_expected) + 1. Both
    bounds hold for T_close taken as the closure time (T_close_e2e) and as the
    close lead (robot.hand.T_close_lead) — the lead is the one the design bound
    and the grid search's commit-lead rank gate are written in, and either can
    be the longer. A key the overlay does not write is the shipped one, and
    must hold too."""

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
    hand = ship["robot"]["hand"]
    t_close_tot = hand["T_close_e2e"] + h / 2
    t_arm = effective("joint_cmd", "lag", "T_arm")
    margin = effective("planner", "search", "grid", "time", "margin")
    t_freeze = effective("planner", "freeze", "T_freeze")
    assert t_freeze >= t_close_tot + t_arm + margin
    # L is the shipped comment's 0.14 (io.horizon_min: "... + L 0.14").
    horizon = effective("io", "horizon_min")
    assert horizon >= t_close_tot + t_arm + margin + 0.14 - 1e-9
    # The same two in the close lead, which is what the shipped comments derive
    # the values from. Absent, the lead is the closure time (catching_params).
    t_lead_tot = hand.get("T_close_lead", hand["T_close_e2e"]) + h / 2
    assert t_freeze >= t_lead_tot + t_arm + margin - 1e-9
    assert horizon >= t_lead_tot + t_arm + margin + 0.14 - 1e-9
    # The 1e-9 absorbs a quotient landing a hair above an integer in binary.
    dt = effective("prediction", "dt_expected")
    assert effective("io", "n_min") == math.ceil(horizon / dt - 1e-9) + 1


# ── The shipped lead (#821) ──────────────────────────────────────────────────

# What the shipped controller YAML carries since the lead moved into it: the
# values the catch_lead_on overlays wrote. Nothing else pins them.
SHIPPED_LEAD = {
    "ur5e_p1b": {"T_arm": 0.05, "lead_enable": True, "provisional": True, "T_freeze": 0.39},
    LEAP: {"T_arm": 0.05, "lead_enable": True, "provisional": True, "T_freeze": 0.19},
}


def _shipped_catching(profile: str) -> dict:
    path = os.path.join(CONFIG_ROOT, profile, "controllers", "demo_catching_controller.yaml")
    return _shipped_tree(path)["catching"]


@pytest.mark.parametrize("profile", sorted(SHIPPED_LEAD))
def test_the_shipped_controller_leads_the_sim_plant(profile):
    """The shipped lag block is the sim servo's lag with the lead on, and it stays
    provisional: that flag is what parks a real arm until the real arm's own
    T_arm is identified (the validator's kProvisionalOnRealArm), whatever the value here."""
    catching = _shipped_catching(profile)
    expected = SHIPPED_LEAD[profile]
    assert catching["joint_cmd"]["lag"] == {
        key: expected[key] for key in ("provisional", "T_arm", "lead_enable")
    }
    assert catching["planner"]["freeze"]["T_freeze"] == expected["T_freeze"]
    # The shipped keys alone hold the window (the overlays' check, on the shipped file).
    _check_commit_window({}, catching, profile)


@pytest.mark.parametrize("profile", sorted(SHIPPED_LEAD))
def test_the_shipped_t_arm_is_the_sim_servo_lag(profile):
    """T_arm is the lag of the plant the shipped sim runs: a position PD tracks a
    ramp kd/kp behind, on every arm joint (mujoco_simulator.yaml)."""
    sim = _load(os.path.join(CONFIG_ROOT, profile, "mujoco_simulator.yaml"))
    params = sim["mujoco_simulator"]["ros__parameters"]
    assert params["use_yaml_servo_gains"] is True
    lags = [kd / kp for kp, kd in zip(params["servo_kp"], params["servo_kd"], strict=True)]
    t_arm = _shipped_catching(profile)["joint_cmd"]["lag"]["T_arm"]
    assert lags == [pytest.approx(t_arm)] * len(lags)


def test_the_shipped_p1b_freeze_is_the_design_bound_rounded_up():
    """ur5e_p1b's T_freeze moved with T_arm: the lower bound
    T_close_lead + h/2 + T_arm + margin, rounded up to 10 ms."""
    catching = _shipped_catching("ur5e_p1b")
    base = _load(os.path.join(CONFIG_DIR, RATE_FILE["ur5e_p1b"]))
    h = 1.0 / base["/**"]["ros__parameters"]["control_rate"]
    bound = (
        catching["robot"]["hand"]["T_close_lead"]
        + h / 2
        + catching["joint_cmd"]["lag"]["T_arm"]
        + catching["planner"]["search"]["grid"]["time"]["margin"]
    )
    assert bound == pytest.approx(0.383)
    assert catching["planner"]["freeze"]["T_freeze"] == pytest.approx(math.ceil(bound * 100) / 100)


# ── ur5e_p1b sim.yaml (the planner's envelope box) ───────────────────────────


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
