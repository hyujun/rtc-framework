"""Unit tests for the demo GUI's Hand Joint panel.

``hand_joint_controller`` (profile p2) declares a ``mode`` parameter —
``tracking`` / ``damping`` / ``damping_feedforward`` — and refuses every
``joint_goal`` while it is not ``tracking``. What these tests hold:

- the pure decisions in ``demo_gui.hand_joint`` (answer parsing, readout text,
  the fail-open send guard) and that the mode tuple agrees with the C++ header,
  read from its source rather than copied,
- the Gains rows (``gravity_ff.*``, ``trajectory_speed``) and that they reach
  GAIN_DEFS only through the profile-scoped registration, bool row included,
- the ``app.py`` wiring pytest can see: the guard in all four hand publish
  paths, the owner-only gate repainted on selection / poll / active change, one
  ``set_parameters_atomically`` per Set Mode, the refusal shown verbatim.

Tk layout is checked at runtime (verify skill), as for every other panel.
"""

from __future__ import annotations

import inspect
import re
import types
from pathlib import Path

import pytest
from rcl_interfaces.msg import ParameterType

from integrated_bringup.demo_gui import config
from integrated_bringup.demo_gui.discovery import RobotProfile
from integrated_bringup.demo_gui.hand_joint import (
    HAND_JOINT_FF_BLOCK,
    HAND_JOINT_FF_LEAVES,
    HAND_JOINT_GAIN_LAYOUT,
    HAND_JOINT_MODE_PARAM,
    HAND_JOINT_MODES,
    HAND_JOINT_OWNERS,
    HAND_JOINT_READOUT,
    format_mode_readout,
    goal_block_reason,
    hand_joint_gain_rows,
    parse_mode,
)

_HEADER = (
    Path(__file__).resolve().parents[1]
    / "include"
    / "integrated_bringup"
    / "controllers"
    / "hand_joint_controller.hpp"
)
_PARAMETERS_CPP = (
    Path(__file__).resolve().parents[1] / "src" / "controllers" / "hand_joint" / "parameters.cpp"
)

_STRING = ParameterType.PARAMETER_STRING


def _value(type_: int, string_value: str = ""):
    return types.SimpleNamespace(type=type_, string_value=string_value)


# ── the C++ side is the owner ────────────────────────────────────────────────


def test_modes_are_the_literals_of_the_cpp_header():
    """ParseHandJointMode / HandJointModeName own the names. Pending on the
    header while it does not spell all three literals."""
    src = _HEADER.read_text()
    for mode in HAND_JOINT_MODES:
        assert f'"{mode}"' in src, f"{_HEADER.name} does not spell the mode '{mode}'"
    assert HAND_JOINT_MODES[0] == "tracking"  # the default and the only goal-accepting mode


def test_ff_names_are_the_literals_of_the_cpp_sources():
    """kHandJointFfBlockKey owns the block name, parameters.cpp the leaves (it
    declares and accepts each as ``key + ".<leaf>"``). The constant is matched,
    not searched for: the header's comments spell the block name too."""
    m = re.search(r'kHandJointFfBlockKey\s*=\s*"([^"]+)"', _HEADER.read_text())
    assert m, f"{_HEADER.name} does not define kHandJointFfBlockKey"
    assert m.group(1) == HAND_JOINT_FF_BLOCK
    src = _PARAMETERS_CPP.read_text()
    for leaf in HAND_JOINT_FF_LEAVES:
        assert f'declare(key + ".{leaf}"' in src, f"parameters.cpp does not declare '{leaf}'"
        assert f'name == key + ".{leaf}"' in src, f"parameters.cpp does not accept '{leaf}'"
    params = {r.param for r in hand_joint_gain_rows()}
    assert {f"{HAND_JOINT_FF_BLOCK}.{leaf}" for leaf in HAND_JOINT_FF_LEAVES} <= params


def test_owner_is_the_profile_extra_controller():
    assert {"hand_joint_controller"} == HAND_JOINT_OWNERS
    assert set(RobotProfile.for_robot("p2").extra_switchable_controllers) >= HAND_JOINT_OWNERS


# ── pure decisions ───────────────────────────────────────────────────────────


@pytest.mark.parametrize("mode", HAND_JOINT_MODES)
def test_parse_mode_reads_each_whitelisted_mode(mode):
    assert parse_mode([_value(_STRING, mode)]) == mode


@pytest.mark.parametrize(
    "answer",
    [
        [],
        [_value(0)],  # PARAMETER_NOT_SET: read before configure
        [_value(_STRING, "fixed")],  # off-whitelist
        [_value(ParameterType.PARAMETER_BOOL, "tracking")],  # wrong type
        [_value(_STRING, "tracking"), _value(_STRING, "tracking")],
    ],
)
def test_parse_mode_returns_unknown_instead_of_a_guess(answer):
    assert parse_mode(answer) is None


def test_readout_spec_reads_mode_from_owners_only():
    assert HAND_JOINT_READOUT.params == (HAND_JOINT_MODE_PARAM,) == ("mode",)
    assert HAND_JOINT_READOUT.owners == HAND_JOINT_OWNERS
    assert HAND_JOINT_READOUT.unknown is None


def test_readout_text_names_each_mode_and_unknown():
    assert format_mode_readout(None) == "current: unknown"
    for mode in HAND_JOINT_MODES:
        assert mode in format_mode_readout(mode)


def test_goal_guard_blocks_only_the_damping_modes():
    assert goal_block_reason("damping") == "mode is damping: joint goals are refused"
    assert "damping_feedforward" in goal_block_reason("damping_feedforward")
    assert goal_block_reason("tracking") == ""


def test_goal_guard_fails_open_on_unknown():
    assert goal_block_reason(None) == ""


# ── Gains rows ───────────────────────────────────────────────────────────────


def test_gain_rows_cover_the_runtime_parameters():
    rows = {r.param: r for r in hand_joint_gain_rows()}
    assert set(rows) == {
        "gravity_ff.enabled",
        "gravity_ff.gain",
        "gravity_ff.inertia_gain",
        "trajectory_speed",
        "max_traj_velocity",
    }
    assert rows["gravity_ff.enabled"].is_bool
    assert not any(r.is_bool for p, r in rows.items() if p != "gravity_ff.enabled")
    assert [p for p, r in rows.items() if r.read_only] == ["max_traj_velocity"]
    groups = {r.group for r in rows.values()}
    assert groups == {g for row in HAND_JOINT_GAIN_LAYOUT for g in row}


def test_no_static_gain_entry_leaks_the_controller_into_every_profile():
    assert "hand_joint_controller" not in config.GAIN_DEFS


def test_registration_builds_the_bool_row_and_setters():
    ctrl = "hand_joint_controller_test_only"
    try:
        config.register_scalar_gain_schema(ctrl, hand_joint_gain_rows(), HAND_JOINT_GAIN_LAYOUT)
        defs = {label: is_bool for label, _n, _d, is_bool, _g in config.GAIN_DEFS[ctrl]}
        assert defs["gravity_ff enabled"] is True
        assert defs["gravity_ff gain [0-2]"] is False
        dispatch = config.GAIN_PARAM_DISPATCH[ctrl]
        name, setter = dispatch["gravity_ff enabled"]
        assert name == "gravity_ff.enabled"
        assert setter([1.0]) == (config.Parameter.Type.BOOL, True)
        assert setter([0.0]) == (config.Parameter.Type.BOOL, False)
        assert dispatch["gravity_ff gain [0-2]"][1] is config._set_double
        assert dispatch["gravity_ff inertia_gain [0-2]"] == (
            "gravity_ff.inertia_gain",
            config._set_double,
        )
        assert dispatch["max_traj_velocity [rad/s]"][1] is config._read_only
    finally:
        for table in (
            config.GAIN_DEFS,
            config.GAIN_ROW_NAMES,
            config.GAIN_PARAM_DISPATCH,
            config.GAIN_GROUP_LAYOUT,
        ):
            table.pop(ctrl, None)


# ── app.py wiring ────────────────────────────────────────────────────────────


def _gui():
    from integrated_bringup.demo_gui.app import DemoControllerGUI

    return DemoControllerGUI


def test_every_hand_publish_leaves_through_the_guarded_choke_point():
    """One `hand_cmd_pub.publish` in the module, inside `_publish_hand`, which
    asks the guard first — a new hand path cannot skip it by forgetting."""
    from integrated_bringup.demo_gui import app as app_module

    gui = _gui()
    assert inspect.getsource(app_module).count("hand_cmd_pub.publish(") == 1
    choke = inspect.getsource(gui._publish_hand)
    assert "hand_cmd_pub.publish(" in choke and "_hand_goal_block" in choke
    for method in (gui._publish_target, gui._send_preset, gui._send_hand_step):
        assert "_publish_hand" in inspect.getsource(method), method.__name__
    # The preset precondition chain asks before anything is built, and the
    # toggle shortcut ends in _send_preset(atomic=True), so it rides that chain.
    assert "_hand_goal_block" in inspect.getsource(gui._send_preset)
    assert "atomic=True" in inspect.getsource(gui._wait_for_toggle_controller)


def test_the_gate_is_repainted_on_selection_poll_and_active_change():
    gui = _gui()
    for method in (
        gui._on_ctrl_radio_change,
        gui._refresh_controller_widgets,
        gui._on_active_controller,
    ):
        assert "self._apply_hand_joint_state()" in inspect.getsource(method), method.__name__


def test_the_gate_is_owner_only_not_inactive_only():
    src = inspect.getsource(_gui()._apply_hand_joint_state)
    assert "HAND_JOINT_OWNERS" in src
    assert "_active_ctrl ==" not in src and "apply_enabled" not in src


def test_set_mode_is_one_atomic_set_with_the_refusal_verbatim():
    gui = _gui()
    src = inspect.getsource(gui._send_hand_joint_mode)
    assert src.count(".set_parameters_atomically(") == 1
    assert 'f"refused: {result.reason}"' in src
    assert "force=True" in inspect.getsource(gui._after_hand_joint_set)
