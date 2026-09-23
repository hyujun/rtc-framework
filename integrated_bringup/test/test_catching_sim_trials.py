"""The catching sim-trial driver's non-ROS half: profile resolution and the throw series.

What matters most is that the driver homes the arm to the pose the planner
seeds from. A driver that guessed the device or read the wrong roster would
still produce trials — aligned to the wrong pose — so resolution is checked on
both shipped profiles, whose arms differ in joint count (6 vs 7): a resolver
that picked the wrong device or roster cannot pass both.
"""

import os
import shutil

import pytest
import yaml

from integrated_bringup.catching_sim_trials import (
    alignment_error,
    load_arm_profile,
    trial_throws,
)

CONFIG = os.path.join(os.path.dirname(__file__), "..", "config")


def shipped_wait_pose(profile):
    path = os.path.join(CONFIG, profile, "controllers", "demo_catching_controller.yaml")
    with open(path) as f:
        return yaml.safe_load(f)["demo_catching_controller"]["catching"]["planner"]["wait_pose"]


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
