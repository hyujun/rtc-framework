"""The catching sim-trial driver's non-ROS half: profile resolution and the throw series.

What matters most is that the driver homes the arm to the pose the planner
seeds from. A driver that guessed the device or read the wrong roster would
still produce trials — aligned to the wrong pose — so resolution is checked on
both shipped profiles, whose arms differ in joint count (6 vs 7): a resolver
that picked the wrong device or roster cannot pass both.
"""

import os
import shutil
from types import SimpleNamespace

import pytest
import yaml

from integrated_bringup.catching_sim_trials import (
    _cycle_closed,
    alignment_error,
    load_arm_profile,
    reset_fault_failure,
    reset_fault_name,
    trial_throws,
)
from rtc_tools.utils.controller_config import load_controller_config

CONFIG = os.path.join(os.path.dirname(__file__), "..", "config")


def shipped_wait_pose(profile):
    path = os.path.join(CONFIG, profile, "controllers", "demo_catching_controller.yaml")
    doc = load_controller_config(path, config_key="demo_catching_controller")
    return doc["demo_catching_controller"]["catching"]["planner"]["wait_pose"]


@pytest.mark.parametrize(
    ("profile", "device", "n_joints", "state_topic"),
    [
        ("ur5e_p1b", "ur5e", 6, "/ur5e/joint_states"),
        ("iiwa7_leap", "iiwa7", 7, "/iiwa7/joint_states"),
    ],
)
def test_each_shipped_profile_resolves_to_its_own_arm(profile, device, n_joints, state_topic):
    arm = load_arm_profile(os.path.join(CONFIG, profile))
    assert arm.device == device
    assert len(arm.joint_names) == n_joints
    assert arm.state_topic == state_topic
    assert arm.wait_pose == tuple(shipped_wait_pose(profile))
    assert arm.goal_topic == f"/demo_joint_controller/{device}/joint_goal"


def test_a_wait_pose_of_the_wrong_length_is_refused(tmp_path):
    shutil.copytree(os.path.join(CONFIG, "ur5e_p1b"), tmp_path / "p")
    path = tmp_path / "p" / "controllers" / "demo_catching_controller.yaml"
    doc = yaml.safe_load(path.read_text())
    doc["demo_catching_controller"]["catching"]["planner"]["wait_pose"] = [0.0] * 7
    path.write_text(yaml.safe_dump(doc))
    with pytest.raises(ValueError, match="wait_pose has 7 values"):
        load_arm_profile(str(tmp_path / "p"))


def test_the_alignment_error_is_the_worst_joint_and_needs_matching_lengths():
    assert alignment_error([0.0, 0.3, -0.1], [0.0, -0.02, 0.01], [0.0, 0.0, 0.0]) == (0.3, 0.02)
    with pytest.raises(ValueError):
        alignment_error([0.0, 0.0], [0.0, 0.0], [0.0, 0.0, 0.0])


def test_the_throw_series_is_reference_first_then_seeded_perturbations():
    pos, vel = (1.0, 0.0, 0.2), (-2.375, 0.0, 4.11362)
    throws = trial_throws(3, 4, 42, pos, vel)
    assert [t["kind"] for t in throws] == ["reference"] * 3 + ["varied"] * 4
    assert all(t["vel"] == vel for t in throws[:3])
    for t in throws[3:]:
        assert 0.9 <= t["mag_scale"] <= 1.1
        assert -0.3 <= t["lateral_y"] <= 0.3
        assert t["vel"][2] == pytest.approx(vel[2] * t["mag_scale"])
    # Replayable: the same seed gives the same series; another seed does not.
    assert trial_throws(3, 4, 42, pos, vel) == throws
    assert trial_throws(3, 4, 7, pos, vel) != throws


def test_a_cycle_is_closed_by_a_re_arm_after_retreat_whatever_follows():
    # Measured 2026-09-23: after the re-arm the controller sees the next track
    # and goes on to TRACKING — the cycle is still closed.
    assert _cycle_closed(["TRACKING", "APPROACH", "HOLD", "RETREAT", "ARMED", "TRACKING"])
    assert not _cycle_closed(["TRACKING", "APPROACH", "RETREAT"])
    assert not _cycle_closed(["ARMED", "TRACKING", "APPROACH"])


def test_the_fault_reset_names_the_controller_not_its_config_key():
    # Measured 2026-10-08 (E1-F19): the driver sent the config key, the CM
    # answered "'demo_catching_controller' is not the active controller
    # ('DemoCatchingController')", the latch stayed up and the unit ended at
    # the next trial's homing.
    listed = [
        SimpleNamespace(name="DemoJointController", type="demo_joint_controller"),
        SimpleNamespace(name="DemoCatchingController", type="demo_catching_controller"),
    ]
    assert reset_fault_name(listed) == "DemoCatchingController"
    assert reset_fault_name(listed, "demo_joint_controller") == "DemoJointController"
    # Not listed, or listed without a name: no name to send, and no guess.
    assert reset_fault_name(listed[:1]) is None
    assert reset_fault_name([SimpleNamespace(name="", type="demo_catching_controller")]) is None
    assert reset_fault_name([]) is None


def test_a_fault_reset_that_leaves_the_latch_up_is_named():
    # ok is false only while the latch is still up, and an unanswered call is
    # not a reset either: the driver stops on the CM's own words instead of
    # running into the homing timeout with "not ARMED (mode FAULT)".
    assert reset_fault_failure(SimpleNamespace(ok=True, message="cleared")) is None
    refused = reset_fault_failure(SimpleNamespace(ok=False, message="re-latched: nan_inf"))
    assert refused == "/rtc_cm/reset_fault refused: re-latched: nan_inf"
    assert reset_fault_failure(None) == "/rtc_cm/reset_fault did not answer"
