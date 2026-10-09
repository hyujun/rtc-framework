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
    END_RULE_RECORD_KEYS,
    MODE_NAMES,
    OPENED_MODES,
    _cycle_closed,
    alignment_error,
    ball_crossing_s,
    decide_throw_end,
    end_reason_counts,
    load_arm_profile,
    parse_args,
    reset_fault_failure,
    reset_fault_name,
    run_meta_args,
)
from rtc_tools.analysis.catching_throw_list import THROW_RECORD_KEYS
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


# ── ball-height end (--end-on-ball-low, #747) ───────────────────────────────
THR, GRACE, CAP = 0.25, 1.0, 12.0
FALL = [(0.0, 1.5), (0.2, 0.9), (0.4, 0.4), (0.6, 0.2), (0.8, 0.1)]  # crosses 0.25 at 0.6


def end_of(modes, truth, *, after=-1.0, wall=1.0, latch=None):
    return decide_throw_end(
        modes,
        truth,
        threshold_m=THR,
        grace_s=GRACE,
        after_stamp_s=after,
        wall_elapsed_s=wall,
        cap_s=CAP,
        ball_low_t_s=latch,
    )


def test_a_cycle_is_open_from_approach_on_in_the_mode_order():
    assert set(OPENED_MODES) == {
        "APPROACH",
        "COMMITTED",
        "CLOSING",
        "DECEL",
        "HOLD",
        "RETREAT",
        "ABORT_SAFE",
        "FAULT",
    }
    assert OPENED_MODES.isdisjoint({"IDLE", "ARMED", "TRACKING"})
    assert set(MODE_NAMES) >= OPENED_MODES


def test_a_missed_ball_ends_on_the_ball_height_once_the_grace_has_run_out():
    modes = ["ARMED", "TRACKING", "ARMED"]
    # Crossing seen, grace not yet run: keep recording, with the crossing latched.
    first = end_of(modes, FALL)
    assert first.reason is None
    assert first.ball_low_t_s == pytest.approx(0.6)
    # Sim time ran 1.0 s past the crossing, no RETREAT: ball_low, outcome missing.
    late = [*FALL, (1.6, 0.05)]
    done = end_of(modes, late, latch=first.ball_low_t_s)
    assert (done.reason, done.outcome_missing) == ("ball_low", True)
    assert done.ball_low_t_s == pytest.approx(0.6)


def test_a_zero_grace_ends_at_the_crossing_itself():
    done = decide_throw_end(
        ["ARMED"],
        FALL,
        threshold_m=THR,
        grace_s=0.0,
        after_stamp_s=-1.0,
        wall_elapsed_s=1.0,
        cap_s=CAP,
    )
    assert (done.reason, done.outcome_missing) == ("ball_low", True)


def test_an_outcome_inside_the_grace_ends_the_throw_with_it():
    # The controller opened a late cycle after the crossing: the outcome on its
    # RETREAT entry arrived inside the grace, so nothing is missing and the
    # throw does not wait for the re-arm.
    latch = 0.6
    modes = ["ARMED", "TRACKING", "APPROACH", "RETREAT"]
    done = end_of(modes, [*FALL, (1.0, 0.05)], latch=latch)
    assert (done.reason, done.outcome_missing) == ("ball_low", False)


def test_a_cycle_that_opened_before_the_crossing_is_not_ended_by_the_ball_height():
    caught = ["ARMED", "TRACKING", "APPROACH", "COMMITTED", "HOLD"]
    late = [*FALL, (2.6, 0.05)]  # far past the grace
    assert end_of(caught, late).reason is None
    # It ends by its close ...
    closed = [*caught, "RETREAT", "ARMED"]
    assert end_of(closed, late).reason == "cycle_closed"
    # ... or by the wall-clock cap.
    assert end_of(caught, late, wall=CAP).reason == "cap"


def test_a_closed_cycle_wins_over_a_ball_that_is_low():
    modes = ["TRACKING", "APPROACH", "HOLD", "RETREAT", "ARMED"]
    assert end_of(modes, FALL).reason == "cycle_closed"
    assert end_of(["TRACKING", "FAULT"], FALL).reason == "fault"


def test_a_stuck_ball_runs_to_the_cap():
    # Never below the threshold (rests on the hand, or is not seen at all).
    hover = [(0.1 * i, 0.9) for i in range(100)]
    modes = ["ARMED", "TRACKING"]
    assert end_of(modes, hover, wall=CAP - 0.1).reason is None
    assert end_of(modes, hover, wall=CAP).reason == "cap"
    assert end_of(modes, [], wall=CAP).reason == "cap"


def test_a_ball_launched_below_the_threshold_has_not_missed():
    # Reference release height 0.2 m, threshold 0.25: rises, then comes down.
    arc = [(0.0, 0.2), (0.1, 0.2), (0.3, 1.5), (1.0, 0.8), (1.4, 0.2)]
    assert ball_crossing_s(arc[:2], THR, -1.0) is None
    assert ball_crossing_s(arc[:3], THR, -1.0) is None
    assert ball_crossing_s(arc, THR, -1.0) == pytest.approx(1.4)


def test_samples_of_the_previous_throw_cannot_end_this_one():
    # A parked ball at the floor, stamped before this launch, preceded by a
    # fall: a rule that did not drop it by stamp would fire on it.
    stale = [(0.0, 1.0), (0.2, 0.1), (0.4, 0.05)]
    mine = [(5.0, 1.2), (5.2, 1.0)]
    assert ball_crossing_s(stale + mine, THR, after_stamp_s=0.4) is None
    assert end_of(["ARMED"], stale + mine, after=0.4).reason is None
    # Judged by sim time: stale samples do not count toward the grace either.
    assert end_of(["ARMED"], [*stale, (5.0, 1.2), (5.5, 0.1)], after=0.4, latch=5.5).reason is None


def test_the_flags_are_off_by_default_and_a_run_without_them_records_nothing_new():
    # a run names its throws (#798); the series is beside the point here
    series = ["--dist", "hand_lhs"]
    args = parse_args(["out", *series])
    assert args.end_on_ball_low is False
    assert (args.ball_low_margin_m, args.ball_low_grace_s) == (0.20, 1.0)
    # run_meta.json's args are what they were before the flag existed ...
    off = run_meta_args(args)
    assert not {"end_on_ball_low", "ball_low_margin_m", "ball_low_grace_s"} & off.keys()
    # ... and with the flag they carry it.
    on = run_meta_args(
        parse_args(["out", *series, "--end-on-ball-low", "--ball-low-grace-s", "2"])
    )
    assert on["end_on_ball_low"] is True
    assert (on["ball_low_margin_m"], on["ball_low_grace_s"]) == (0.20, 2.0)
    with pytest.raises(SystemExit):
        parse_args(["out", *series, "--ball-low-grace-s", "-1"])


def test_the_new_record_keys_cannot_be_smuggled_in_by_a_throw_list():
    assert set(END_RULE_RECORD_KEYS) <= THROW_RECORD_KEYS


def test_the_caps_are_counted_apart_from_the_other_ends():
    results = [
        {"end_reason": "ball_low"},
        {"end_reason": "cap"},
        {"end_reason": "cycle_closed"},
        {"end_reason": "cap"},
        {"accepted": False},
        {"end_reason": None},
    ]
    assert end_reason_counts(results) == {"ball_low": 1, "cap": 2, "cycle_closed": 1}
