"""The S8-B/S8-D ``catch_lead_*`` sim overlays must write keys the controller READS.

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
if lead (or the γ grid) is the one thing that changes. Both profiles that ship
overlays are covered: ur5e_p1b (S8-B, four ablation arms, plus the S8-E beanbag
arm that changes the simulated ball and nothing on the controller) and
iiwa7_leap (S8-D, a lead-on arm plus its calibration twin with the supervisor
thresholds off).
"""

from __future__ import annotations

import json
import math
import os

import pytest
import yaml

CONFIG_ROOT = os.path.join(os.path.dirname(__file__), "..", "config")
CONFIG_DIR = os.path.join(CONFIG_ROOT, "ur5e_p1b")
SHIPPED = os.path.join(CONFIG_DIR, "controllers", "demo_catching_controller.yaml")
OVERLAY_DIR = os.path.join(CONFIG_DIR, "sim_overlays")
CONTROLLER = "demo_catching_controller"
ARMS = {
    "catch_lead_on": {"lead_enable": True, "gamma0": False},
    "catch_lead_off": {"lead_enable": False, "gamma0": False},
    "catch_lead_on_gamma0": {"lead_enable": True, "gamma0": True},
    "catch_lead_off_gamma0": {"lead_enable": False, "gamma0": True},
}
BEANBAG = "catch_lead_on_beanbag"
SIM_CONFIG = os.path.join(CONFIG_DIR, "mujoco_simulator.yaml")
LEAP = "iiwa7_leap"
LEAP_ARMS = ("catch_lead_on", "catch_lead_on_unbounded")
# Where each profile keeps control_rate (the launch files read the same file).
RATE_FILE = {"ur5e_p1b": "_base.yaml", LEAP: "sim.yaml"}
SUPERVISOR_OFF = ("sat_ticks", "track_err_abort", "stale_committed_max_s")


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


@pytest.fixture(scope="module")
def leap_shipped() -> dict:
    return _load(os.path.join(CONFIG_ROOT, LEAP, "controllers", "demo_catching_controller.yaml"))[
        CONTROLLER
    ]


@pytest.fixture(scope="module")
def leap_arms() -> dict[str, dict]:
    overlay_dir = os.path.join(CONFIG_ROOT, LEAP, "sim_overlays")
    return {name: _controller_tree(_leap_rt_only(overlay_dir, name)) for name in LEAP_ARMS}


LEAP_SCENE = "package://robot_descriptions/robots/iiwa7_leap/mjcf/scene_right.xml"


def _leap_rt_only(overlay_dir: str, name: str) -> dict:
    """The leap overlays also swap the sim scene (their header says why); the
    sim section may carry that and nothing else, the rest is the RT node's."""
    overlay = _load(os.path.join(overlay_dir, name + ".yaml"))
    sim = overlay.pop("mujoco_simulator")
    assert sim == {"ros__parameters": {"model_path": LEAP_SCENE}}, (name, sim)
    return overlay


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


def _beanbag_sections() -> tuple[dict, dict]:
    """(simulator section, the rest) of the beanbag overlay."""
    overlay = _load(os.path.join(OVERLAY_DIR, BEANBAG + ".yaml"))
    sim = overlay.pop("mujoco_simulator")
    return sim, overlay


def test_beanbag_arm_changes_only_the_ball_type():
    """S8-E ball arm (D-S8-11, D-S8-16 ④): the simulator section carries the
    ball preset and nothing else — shape and mass stay the shipped ones, which
    the controller's ball model also assumes — and the key is the one the
    shipped simulator YAML sets (a path one level off would run tennis)."""
    sim, _ = _beanbag_sections()
    assert sim == {"ros__parameters": {"projectile_ball": {"ball_type": "beanbag"}}}
    ship = _load(SIM_CONFIG)["mujoco_simulator"]["ros__parameters"]["projectile_ball"]
    assert ship["ball_type"] == "tennis", "the arm must differ from the shipped ball"


def test_beanbag_arm_runs_the_lead_on_controller(arms):
    """The pair is only a ball comparison if the controller is catch_lead_on's."""
    _, rest = _beanbag_sections()
    assert _controller_tree(rest) == arms["catch_lead_on"]


def test_the_commit_window_is_derived_from_t_arm(arms, shipped):
    _check_commit_window(arms["catch_lead_on"]["catching"], shipped["catching"], "ur5e_p1b")


# ── iiwa7_leap (S8-D) ────────────────────────────────────────────────────────


@pytest.mark.parametrize("name", LEAP_ARMS)
def test_leap_every_leaf_is_a_shipped_key_of_the_same_type(name, leap_arms, leap_shipped):
    assert _unread_leaves(leap_arms[name], leap_shipped) == []


def test_leap_check_catches_a_path_one_level_short(leap_arms, leap_shipped):
    shallow = leap_arms["catch_lead_on_unbounded"]["catching"]
    problems = _unread_leaves(shallow, leap_shipped)
    assert len(problems) == len(_leaves(shallow)), problems


@pytest.mark.parametrize("name", LEAP_ARMS)
def test_leap_commit_window_is_derived_from_t_arm(name, leap_arms, leap_shipped):
    """No leap overlay writes T_freeze: the shipped 0.19 must already cover T_arm."""
    _check_commit_window(leap_arms[name]["catching"], leap_shipped["catching"], LEAP)


def test_leap_arms_lead_the_sim_plant(leap_arms):
    for name, arm in leap_arms.items():
        lag = arm["catching"]["joint_cmd"]["lag"]
        assert lag == {"T_arm": 0.05, "lead_enable": True}, name


def test_leap_calibration_arm_only_lifts_the_supervisor_thresholds(leap_arms, leap_shipped):
    """The calibration twin measures the UNCENSORED streak / error / age
    distributions, so it must differ from the lead-on arm in exactly the three
    thresholds, each set past anything a trial can produce (and inside the
    parser's range — stale_committed_max_s is capped at 1.0)."""
    on = _leaves(leap_arms["catch_lead_on"])
    off = _leaves(leap_arms["catch_lead_on_unbounded"])
    lifted = {("catching", "supervisor", k) for k in SUPERVISOR_OFF}
    assert set(off) - set(on) == lifted
    assert {k: v for k, v in off.items() if k not in lifted} == on
    shipped = leap_shipped["catching"]["supervisor"]
    for key in SUPERVISOR_OFF:
        assert off[("catching", "supervisor", key)] > shipped[key], key
    assert off[("catching", "supervisor", "stale_committed_max_s")] <= 1.0
    # A trial is a few seconds; 10 s at this robot's control rate is a streak
    # no trial can reach, whatever rate the profile ships.
    base = _load(os.path.join(CONFIG_ROOT, LEAP, RATE_FILE[LEAP]))
    rate = base["/**"]["ros__parameters"]["control_rate"]
    assert off[("catching", "supervisor", "sat_ticks")] >= 10 * rate


@pytest.mark.parametrize("profile", ("ur5e_p1b", LEAP))
def test_sim_perception_profile_serves_its_own_controller(profile):
    """Each robot ships its own copy of ``ball_perception_sim_profile.json``,
    and its controller YAML says the two "must match". Pinned here so that
    retuning one robot's copy cannot drift from what its controller expects."""
    ship = _load(
        os.path.join(CONFIG_ROOT, profile, "controllers", "demo_catching_controller.yaml")
    )
    catching = ship[CONTROLLER]["catching"]
    with open(
        os.path.join(CONFIG_ROOT, profile, "ball_perception_sim_profile.json"), encoding="utf-8"
    ) as f:
        vision = json.load(f)
    pred = vision["prediction"]
    assert pred["step_s"] == catching["prediction"]["dt_expected"]
    assert pred["horizon_s"] >= catching["io"]["horizon_min"]
    assert pred["max_points"] >= catching["io"]["n_min"]
    assert vision["time"]["max_future_skew_s"] == catching["sim"]["io"]["future_tol"]
