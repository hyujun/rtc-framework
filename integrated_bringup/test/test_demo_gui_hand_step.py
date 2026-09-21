"""Hand step panel (dynamic_catching §13 S4).

The panel's whole job is to show the pose the CONTROLLER loaded, so the cases
below are mostly about refusing to show anything else: a profile the controller
did not fully report, a width that disagrees with the device, a step the
controller would refuse anyway, and a rho computed from a short state message.

The rho arithmetic itself is NOT retested here — the panel imports
``rtc_tools.analysis.hand_close.rho`` and ``test_hand_close.py`` owns it. What
is pinned here is that the import is what runs, which the direction case does:
a local copy that dropped the sign would pass every other case in this file.
"""

from __future__ import annotations

import math

import pytest

from integrated_bringup.demo_gui.hand_step import (
    HandStepPanel,
    StepPose,
    profile_from_parameters,
)

JOINTS = ["j0", "j1", "j2"]


def make_values(**overrides) -> dict:
    values = {
        "hand.q_open": [0.0, 0.0, 0.0],
        "hand.q_pre": [0.1, 0.1, 0.1],
        "hand.q_close": [1.1, 1.1, 1.1],
        "hand.caging_mask": [True, True, False],
        "hand.eta_close": 0.9,
        "hand.rho_eps": 0.02,
        "diagnostic.hand_step": True,
    }
    values.update(overrides)
    return values


def test_profile_is_adopted_from_the_controller():
    panel = HandStepPanel()
    assert panel.load(make_values(), JOINTS)
    assert panel.profile is not None
    assert panel.profile.eta_close == 0.9
    assert panel.step_target(StepPose.PRESHAPE) == [0.1, 0.1, 0.1]
    assert panel.step_target(StepPose.CLOSED) == [1.1, 1.1, 1.1]
    assert panel.step_target(StepPose.OPEN) == [0.0, 0.0, 0.0]
    assert panel.last_sent is StepPose.OPEN


def test_a_missing_parameter_is_named_not_padded():
    values = make_values()
    del values["hand.q_close"]
    with pytest.raises(ValueError, match="hand.q_close"):
        profile_from_parameters(values)


def test_a_short_array_is_refused():
    # Padding it would command the missing joint to 0 rad, which on most hands
    # is fully open — a plausible-looking wrong pose.
    with pytest.raises(ValueError, match="hand.q_close"):
        profile_from_parameters(make_values(**{"hand.q_close": [1.1, 1.1]}))


def test_a_device_width_mismatch_refuses_the_load():
    panel = HandStepPanel()
    assert not panel.load(make_values(), ["j0", "j1"])
    assert panel.profile is None
    assert "2 joints" in panel.last_error


def test_a_step_is_refused_when_the_controller_has_the_diagnostic_off():
    # The controller refuses it; the button says so instead of looking sent.
    panel = HandStepPanel()
    assert panel.load(make_values(**{"diagnostic.hand_step": False}), JOINTS)
    with pytest.raises(ValueError, match="hand_step"):
        panel.step_target(StepPose.CLOSED)
    assert "DISABLED" in panel.lines()[0]


def test_a_step_before_any_profile_is_refused():
    panel = HandStepPanel()
    with pytest.raises(ValueError, match="no hand profile"):
        panel.step_target(StepPose.CLOSED)
    assert panel.lines() == ["hand profile: not loaded"]


def test_rho_uses_the_caging_set_only():
    panel = HandStepPanel()
    assert panel.load(make_values(), JOINTS)
    # j2 is outside the caging set: parking it at the preshape must not hold
    # rho at 0 while the two caging joints are halfway.
    assert panel.update_rho([0.6, 0.6, 0.1]) == pytest.approx(0.5)


def test_rho_follows_the_slowest_caging_joint():
    panel = HandStepPanel()
    assert panel.load(make_values(), JOINTS)
    assert panel.update_rho([1.1, 0.35, 1.1]) == pytest.approx(0.25)


def test_rho_of_a_reversed_joint_counts_as_progress():
    # Pins that the imported rho is what runs: a local copy without the sign
    # would return -0.5 here and pass every other case in this file.
    panel = HandStepPanel()
    values = make_values(**{"hand.q_pre": [0.0, 0.0, 0.0], "hand.q_close": [-1.0, -1.0, -1.0]})
    assert panel.load(values, JOINTS)
    assert panel.update_rho([-0.5, -0.5, 0.0]) == pytest.approx(0.5)


def test_a_short_state_message_reports_unknown_not_a_number():
    panel = HandStepPanel()
    assert panel.load(make_values(), JOINTS)
    assert math.isnan(panel.update_rho([0.6, 0.6]))
    assert panel.lines()[2] == "rho: --"


def test_readout_names_the_caging_verdict():
    panel = HandStepPanel()
    assert panel.load(make_values(), JOINTS)
    panel.update_rho([1.1, 1.1, 0.1])
    assert "caged" in panel.lines()[2]
    panel.update_rho([0.6, 0.6, 0.1])
    assert "closing" in panel.lines()[2]


def test_a_not_set_parameter_is_reported_not_raised():
    # A PARAMETER_NOT_SET value arrives from rclpy as None. It passes the "is
    # the key present" check, so the refusal has to come out of float() — as a
    # TypeError, which load() must turn into the panel's error line rather than
    # let escape into the rclpy executor task, where nothing reads it.
    panel = HandStepPanel()
    assert not panel.load(make_values(**{"hand.q_pre": None}), JOINTS)
    assert panel.profile is None
    assert panel.last_error
