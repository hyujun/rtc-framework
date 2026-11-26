"""stand_pose + sim_recover's pure parts (no ROS graph).

The stand pose of the shipped g1_loco / b7_loco profiles is held to their
joint controller YAML's ``estop.arm_safe_position``; the duration rule is
DemoJointController::Compute's (src/controllers/joint/compute.cpp); the
refusals are the preconditions of ``/sim_recover/recover``.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from integrated_bringup.sim_recover import max_abs_diff, refusal
from integrated_bringup.stand_pose import (
    stand_pose,
    trajectory_duration,
    trajectory_limits,
)

CONFIG = Path(__file__).resolve().parents[1] / "config"
CTRL = "demo_joint_controller"


@pytest.mark.parametrize(("profile", "group"), [("g1_loco", "g1"), ("b7_loco", "b7")])
def test_stand_pose_is_the_joint_controllers_safe_position(profile, group):
    raw = yaml.safe_load((CONFIG / profile / "controllers" / f"{CTRL}.yaml").read_text())
    expected = [float(x) for x in raw[CTRL]["estop"]["arm_safe_position"]]
    names, pose = stand_pose(CONFIG / profile, CTRL, group)
    assert pose == expected
    assert len(names) == len(pose) == raw[CTRL]["arm_dof"]
    assert len(set(names)) == len(names)


def test_stand_pose_unknown_group_is_an_error():
    with pytest.raises(ValueError, match="no devices.nope"):
        stand_pose(CONFIG / "g1_loco", CTRL, "nope")


def test_trajectory_limits_of_g1_loco():
    assert trajectory_limits(CONFIG / "g1_loco", CTRL) == (0.78, 3.14)


def test_duration_zero_distance_is_the_floor():
    assert trajectory_duration(0.0, 0.78, 3.14) == 0.01


def test_duration_follows_speed_when_slower():
    # 0.78 rad at 0.78 rad/s = 1 s; the velocity term 1.875*0.78/3.14 = 0.466 s
    assert trajectory_duration(0.78, 0.78, 3.14) == pytest.approx(1.0)


def test_duration_follows_velocity_cap_when_larger():
    # speed term 1.0/10 = 0.1 s; cap term 1.875*1.0/1.0 = 1.875 s
    assert trajectory_duration(1.0, 10.0, 1.0) == pytest.approx(1.875)


def test_duration_without_a_velocity_cap_is_the_speed_term():
    assert trajectory_duration(2.0, 1.0, 0.0) == pytest.approx(2.0)


def test_duration_floors_a_zero_speed_like_the_controller():
    # LoadConfig clamps the speed to >= 1e-6 rad/s: huge, finite, not a division by zero.
    assert trajectory_duration(1.0, 0.0, 0.0) == pytest.approx(1e6)


def test_max_abs_diff():
    assert max_abs_diff([0.0, 1.0], [0.5, 0.0]) == 1.0
    assert max_abs_diff([], []) == 0.0


GOOD = {
    "active": CTRL,
    "estop": False,
    "joint_controller": CTRL,
    "busy": False,
    "have_joints": True,
}


def test_refusal_none_when_ready():
    assert refusal(**GOOD) is None


@pytest.mark.parametrize(
    ("change", "text"),
    [
        ({"busy": True}, "already running"),
        ({"estop": None}, "E-STOP status is unknown"),
        ({"estop": True}, "E-STOP is latched"),
        ({"active": None}, "active controller is unknown"),
        ({"active": "locomotion_controller"}, "locomotion_controller"),
        ({"have_joints": False}, "no joint state"),
        ({"have_sim_time": False}, "no sim time"),
    ],
)
def test_refusals(change, text):
    why = refusal(**{**GOOD, **change})
    assert why is not None and text in why
