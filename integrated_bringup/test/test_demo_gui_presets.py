"""Unit tests for demo_controller_gui hand-preset length validation.

Pure-Python (no Tk): exercises ``config.preset_hand_targets`` (the publish
guard) and ``config.default_presets_for`` (robot-appropriate defaults). The
regression guard for issue #137 finding 3: a preset's positions_deg length
must equal the active hand DoF, or the hand target is rejected rather than
publishing a RobotTarget with mismatched joint_names / joint_target lengths.
"""

import math
from pathlib import Path

import pytest
import yaml

from integrated_bringup.demo_gui import config
from integrated_bringup.demo_gui.config import (
    default_presets_for,
    preset_hand_targets,
    read_only_gain_rows,
    superseded_default_presets,
)
from integrated_bringup.demo_gui.discovery import RobotProfile, RobotShape

_CONFIG = Path(__file__).resolve().parents[1] / "config"


def test_default_presets_10dof_lengths():
    presets = default_presets_for(10)
    assert presets  # non-empty
    for name, data in presets.items():
        assert len(data["positions_deg"]) == 10, name


def test_default_presets_16dof_lengths():
    presets = default_presets_for(16)
    assert presets
    for name, data in presets.items():
        assert len(data["positions_deg"]) == 16, name


def test_default_presets_returns_deep_copy():
    a = default_presets_for(10)
    a["open_flat"]["positions_deg"][0] = 99.0
    b = default_presets_for(10)
    # mutating one result must not leak into the module constant / next call
    assert b["open_flat"]["positions_deg"][0] == 0.0


def test_preset_hand_targets_rejects_length_mismatch():
    preset = {"positions_deg": [0.0] * 10}
    # 10-DoF preset against a 16-DoF hand → rejected
    assert preset_hand_targets(preset, 16) is None
    # matching length → radians-ready float list of the right size
    result = preset_hand_targets(preset, 10)
    assert result is not None
    assert len(result) == 10
    assert all(isinstance(v, float) for v in result)


def test_preset_hand_targets_missing_positions_rejected():
    assert preset_hand_targets({}, 10) is None
    assert preset_hand_targets({"grasp_time": 1.0}, 16) is None


def test_default_presets_pass_their_own_guard():
    # every shipped default must survive the publish guard for its target hand
    for hand_dof in (10, 16):
        for name, data in default_presets_for(hand_dof).items():
            assert preset_hand_targets(data, hand_dof) is not None, name


def test_leap_defaults_match_profile_hand_dof():
    # end-to-end: the 16-DoF defaults line up with the iiwa7_leap profile shape
    shape = RobotShape.default_iiwa7_leap()
    for name, data in default_presets_for(shape.hand_dof).items():
        assert preset_hand_targets(data, shape.hand_dof) is not None, name


# ── p2: a 10-joint hand that is not the 10-DoF roster's hand ─────────────────


# Both hands of the pair, one profile each (p2 = the right hand).
@pytest.fixture(params=["p2", "p2_left"])
def p2_robot(request) -> str:
    return request.param


def _p2(robot: str):
    profile = RobotProfile.for_robot(robot)
    return profile.preset_token(), profile.shape.hand_dof


def test_p2_has_its_own_roster_of_the_right_length(p2_robot):
    scope, hand_dof = _p2(p2_robot)
    presets = default_presets_for(hand_dof, scope)
    assert presets != default_presets_for(hand_dof)
    assert {data["type"] for data in presets.values()} == {"open", "close"}
    for name, data in presets.items():
        assert preset_hand_targets(data, hand_dof) is not None, name


def test_p2_mid_pose_is_the_simulators_initial_qpos(p2_robot):
    """The roster's flexed pose is a pose the hand is known to reach, not
    numbers written for another hand."""
    scope, hand_dof = _p2(p2_robot)
    sim = yaml.safe_load((_CONFIG / p2_robot / "mujoco_simulator.yaml").read_text())
    found = []

    def _walk(node):
        if isinstance(node, dict):
            for key, value in node.items():
                if key == "initial_qpos":
                    found.append(value)
                _walk(value)
        elif isinstance(node, list):
            for value in node:
                _walk(value)

    _walk(sim)
    assert len(found) == 1
    mid = default_presets_for(hand_dof, scope)["mid_pose"]["positions_deg"]
    assert [math.radians(d) for d in mid] == [pytest.approx(q, abs=1e-3) for q in found[0]]


def test_p2_roster_never_asks_a_joint_past_the_side_it_flexes_toward(p2_robot):
    """Sign per joint = the sign of the simulator's mid pose (0 = either). The
    10-DoF roster fails this on the joints that flex negative, which is why p2
    may not inherit it once its limits stop being one-sided."""
    scope, hand_dof = _p2(p2_robot)
    presets = default_presets_for(hand_dof, scope)
    mid = presets["mid_pose"]["positions_deg"]
    for name, data in presets.items():
        for value, reference in zip(data["positions_deg"], mid, strict=True):
            assert value * reference >= 0.0, name
    generic = default_presets_for(hand_dof)["power_grasp"]["positions_deg"]
    assert any(v * r < 0.0 for v, r in zip(generic, mid, strict=True))


def test_scope_roster_is_a_deep_copy_and_unknown_scope_falls_back_to_dof(p2_robot):
    scope, hand_dof = _p2(p2_robot)
    a = default_presets_for(hand_dof, scope)
    a["open_flat"]["positions_deg"][0] = 99.0
    assert default_presets_for(hand_dof, scope)["open_flat"]["positions_deg"][0] == 0.0
    assert default_presets_for(10, "ur5e_p1b") == default_presets_for(10)
    assert default_presets_for(16, "") == default_presets_for(16)


def test_only_an_untouched_generic_file_is_replaced(p2_robot):
    scope, hand_dof = _p2(p2_robot)
    generic = default_presets_for(hand_dof)
    assert superseded_default_presets(generic, hand_dof, scope) == default_presets_for(
        hand_dof, scope
    )
    edited = default_presets_for(hand_dof)
    edited["power_grasp"]["grasp_time"] = 3.0
    assert superseded_default_presets(edited, hand_dof, scope) is None
    added = default_presets_for(hand_dof)
    added["mine"] = {"type": "close", "positions_deg": [0.0] * 10, "grasp_time": 1.0}
    assert superseded_default_presets(added, hand_dof, scope) is None
    # The scope's own roster, and any other profile's file, are left alone.
    assert (
        superseded_default_presets(default_presets_for(hand_dof, scope), hand_dof, scope) is None
    )
    assert superseded_default_presets(generic, hand_dof, "ur5e_p1b") is None


# ── Gains panel: read-only rows ──────────────────────────────────────────────


def test_read_only_rows_follow_the_dispatch_row_by_row():
    for ctrl, defs in config.GAIN_DEFS.items():
        flags = read_only_gain_rows(ctrl)
        assert len(flags) == len(defs), ctrl
        expected = [
            config.GAIN_PARAM_DISPATCH[ctrl][label][1] is config._read_only for label, *_ in defs
        ]
        assert flags == expected, ctrl
    caps = [
        label
        for (label, *_), flag in zip(
            config.GAIN_DEFS["demo_joint_controller"],
            read_only_gain_rows("demo_joint_controller"),
            strict=True,
        )
        if flag
    ]
    assert caps == ["robot_max_traj_vel", "hand_max_traj_vel"]
    assert read_only_gain_rows("no_such_controller") == []


def test_apply_does_not_mirror_a_row_it_did_not_send():
    """A read-only cap is skipped on Apply; the mirror must skip it too, or
    "Currently Applied" shows whatever sat in that entry."""
    import inspect

    from integrated_bringup.demo_gui.app import DemoControllerGUI as gui

    apply = inspect.getsource(gui._publish_gains)
    assert "None if row_read_only else" in apply and "read_only_gain_rows(ctrl)" in apply
    mirror = inspect.getsource(gui._show_applied_gains)
    assert "if values is None:" in mirror
    # A refusal reaches the panel, not only the log.
    assert apply.count("self._set_gains_result") >= 4
    assert "refused, nothing applied" in apply


def test_load_gain_fills_the_panel_it_asked_for_and_its_mirror():
    import inspect

    from integrated_bringup.demo_gui.app import DemoControllerGUI as gui

    load = inspect.getsource(gui._request_load_gains)
    assert "self._fill_gains_from_data, ctrl, flat" in load
    fill = inspect.getsource(gui._fill_gains_from_data)
    assert "self._gains_panels.get(ctrl)" in fill
    assert "_set_entry_text(" in fill and "self._show_applied_gains(ctrl, rows)" in fill
