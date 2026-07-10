"""Unit tests for the demo_controller_gui static robot profiles.

Pure-Python (no rclpy / Tk): exercises RobotShape.default_iiwa7_leap /
default_ur5e_p1b and the RobotProfile.for_robot registry that backs the GUI's
--robot selection.
"""

from integrated_bringup.demo_gui.discovery import (
    ROBOT_PROFILES,
    RobotProfile,
    RobotShape,
)


def test_iiwa7_leap_shape_dof():
    shape = RobotShape.default_iiwa7_leap()
    assert shape.arm_dof == 7
    assert shape.hand_dof == 16
    assert shape.arm_joint_names == ("A1", "A2", "A3", "A4", "A5", "A6", "A7")
    # hand order matches the live /rtc_cm/leap/joint_states wire order
    # (thumb 12-15 first, then index/middle/ring 0-11), so matches_message
    # passes without a spurious mismatch warning.
    assert shape.hand_motor_names == (
        "12",
        "13",
        "14",
        "15",
        "0",
        "1",
        "2",
        "3",
        "4",
        "5",
        "6",
        "7",
        "8",
        "9",
        "10",
        "11",
    )
    # all 16 motor ids "0".."15" present exactly once, regardless of order
    assert set(shape.hand_motor_names) == {str(i) for i in range(16)}


def test_iiwa7_leap_finger_groups_cover_all_motors():
    shape = RobotShape.default_iiwa7_leap()
    labels = [label for label, _ in shape.hand_finger_groups]
    assert labels == ["Thumb", "Index", "Middle", "Ring"]
    flat = [m for _, motors in shape.hand_finger_groups for m in motors]
    # every motor appears exactly once, no "Other" bucket, order preserved
    assert flat == list(shape.hand_motor_names)
    assert len(set(flat)) == 16


def test_iiwa7_leap_name_to_idx_complete():
    shape = RobotShape.default_iiwa7_leap()
    for i in range(7):
        assert shape.arm_name_to_idx[f"A{i + 1}"] == i
    # every motor id is a key, and the index round-trips to its own name
    assert set(shape.hand_name_to_idx) == {str(i) for i in range(16)}
    for name, idx in shape.hand_name_to_idx.items():
        assert shape.hand_motor_names[idx] == name


def test_hand_reorder_is_name_based_not_order_based():
    """The GUI reorders hand positions by name. Feed a wire order that
    differs from the profile order (ascending 0..15 vs the profile's
    thumb-first 12,13,14,15,0,...) and confirm each value still lands at the
    profile slot for its name — proving correctness is order-independent."""
    shape = RobotShape.default_iiwa7_leap()
    idx = shape.hand_name_to_idx
    # ascending wire order, deliberately != profile order; value == motor id
    wire_names = [str(i) for i in range(16)]
    wire_pos = [float(n) for n in wire_names]
    reordered = [0.0] * shape.hand_dof
    for mi, name in enumerate(wire_names):
        if name in idx:
            reordered[idx[name]] = wire_pos[mi]
    # slot k must hold the value of the motor named hand_motor_names[k]
    assert reordered == [float(name) for name in shape.hand_motor_names]


# p1b hand roster the controller republishes on /rtc_cm/p1b/joint_states,
# in wire order — SSoT is config/ur5e_p1b/_base.yaml p1b joint_state_names.
# The profile names must match this exactly or name-based hand feedback
# reordering reads zero (see default_ur5e_p1b docstring).
UR5E_P1B_HAND = (
    "thumb_cmc_aa_joint",
    "thumb_cmc_fe_joint",
    "thumb_mcp_joint",
    "thumb_dip_fe_joint",
    "index_mcp_aa_joint",
    "index_mcp_fe_joint",
    "index_dip_fe_joint",
    "middle_mcp_fe_joint",
    "middle_dip_fe_joint",
    "ring_mcp_fe_joint",
)


def test_ur5e_p1b_shape_dof():
    shape = RobotShape.default_ur5e_p1b()
    assert shape.arm_dof == 6
    assert shape.hand_dof == 10
    assert shape.arm_joint_names == (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    )
    # exact match (names + order) with the wire roster — the correctness
    # invariant behind the whole profile.
    assert shape.hand_motor_names == UR5E_P1B_HAND


def test_ur5e_p1b_finger_groups_cover_all_motors():
    shape = RobotShape.default_ur5e_p1b()
    labels = [label for label, _ in shape.hand_finger_groups]
    assert labels == ["Thumb", "Index", "Middle", "Ring"]
    # p1b splits 4/3/2/1 across the fingers (unlike assm_v1's 3/3/3/1).
    counts = [len(motors) for _, motors in shape.hand_finger_groups]
    assert counts == [4, 3, 2, 1]
    flat = [m for _, motors in shape.hand_finger_groups for m in motors]
    # every motor appears exactly once, no "Other" bucket, order preserved
    assert flat == list(shape.hand_motor_names)
    assert len(set(flat)) == 10


def test_ur5e_p1b_name_to_idx_complete():
    shape = RobotShape.default_ur5e_p1b()
    assert set(shape.hand_name_to_idx) == set(UR5E_P1B_HAND)
    for name, idx in shape.hand_name_to_idx.items():
        assert shape.hand_motor_names[idx] == name


def test_for_robot_known_keys():
    iiwa = RobotProfile.for_robot("iiwa7_leap")
    assert iiwa.shape.arm_dof == 7
    # Controllers broadcast the arm-tip TF with the "_actual" suffix
    # (owned_topics MakeActualChildFrame), matching the ur5e profile below.
    assert (iiwa.tcp_parent, iiwa.tcp_child) == ("link_0", "ee_link_actual")

    ur5e = RobotProfile.for_robot("ur5e_p1a")
    assert ur5e.shape.arm_dof == 6
    assert ur5e.shape.hand_dof == 10
    assert (ur5e.tcp_parent, ur5e.tcp_child) == ("base", "tool0_actual")

    # p1b shares the UR5e arm and TCP frames; only the hand roster differs.
    p1b = RobotProfile.for_robot("ur5e_p1b")
    assert p1b.shape.arm_dof == 6
    assert p1b.shape.hand_dof == 10
    assert p1b.shape.hand_motor_names == UR5E_P1B_HAND
    assert (p1b.tcp_parent, p1b.tcp_child) == ("base", "tool0_actual")


def test_for_robot_unknown_raises():
    try:
        RobotProfile.for_robot("bogus")
    except ValueError as e:
        assert "bogus" in str(e)
        assert "iiwa7_leap" in str(e) and "ur5e_p1a" in str(e)
    else:
        raise AssertionError("expected ValueError for unknown robot key")


def test_registry_keys_match_config_dirs():
    # --robot keys mirror config/<key>/ bringup directory names
    assert set(ROBOT_PROFILES) == {"ur5e_p1a", "ur5e_p1b", "iiwa7_leap"}
