"""Unit tests for demo_controller_gui hand-preset length validation.

Pure-Python (no Tk): exercises ``config.preset_hand_targets`` (the publish
guard) and ``config.default_presets_for`` (robot-appropriate defaults). The
regression guard for issue #137 finding 3: a preset's positions_deg length
must equal the active hand DoF, or the hand target is rejected rather than
publishing a RobotTarget with mismatched joint_names / joint_target lengths.
"""

from integrated_bringup.demo_gui.config import (
    default_presets_for,
    preset_hand_targets,
)
from integrated_bringup.demo_gui.discovery import RobotShape


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
