"""Unit tests for demo_controller_gui's ControllerCatalog identity mapping.

Pure-Python (no rclpy Node / Tk): exercises ``catalog.build_entries`` — the
map from a ``/rtc_cm/list_controllers`` response to GUI ``ControllerEntry``
rows. The regression guard for issue #137 finding 1: the config key must come
from ``ControllerState.type`` (== snake_case registry/config key), NOT
``ControllerState.name`` (the C++ class name from ``Name()``).
"""

from integrated_bringup.demo_gui.catalog import build_entries
from integrated_bringup.demo_gui.config import (
    COMPLIANCE_GAIN_RENAMES,
    GAIN_DEFS,
    GAIN_PARAM_DISPATCH,
    GAIN_ROW_NAMES,
    NO_EXTERNAL_COMMAND_CONTROLLERS,
    target_panel_states,
)
from integrated_bringup.demo_gui.discovery import RobotProfile
from rtc_msgs.msg import ControllerState

INFERENCE = "demo_inference_controller"

# What the CM actually publishes: name == PascalCase Name(), type == config key.
# SSoT: rt_controller_node_services.cpp (cs.name = Name(), cs.type =
# controller_types_[i]) + rt_controller_node_params.cpp (controller_types_
# .push_back(entry.config_key)).
_SHIPPED = [
    ("DemoJointController", "demo_joint_controller"),
    ("DemoTaskController", "demo_task_controller"),
    ("DemoWbcController", "demo_wbc_controller"),
    # #469 S2. Listed by hand like the rest: this file builds its states FROM
    # this list, so a controller that is missing here is not a failure, it is
    # simply never tested — which is how the fourth one would have gone
    # unnoticed in the GUI's gain-schema lookup.
    ("DemoComplianceController", "demo_compliance_controller"),
]


def _make_state(name: str, config_key: str, *, is_active: bool = False, groups=()):
    cs = ControllerState()
    cs.name = name
    cs.type = config_key
    cs.state = "active" if is_active else "inactive"
    cs.is_active = is_active
    cs.claimed_groups = list(groups)
    return cs


def test_config_key_from_type_not_name():
    """Every shipped controller maps to its snake_case config key, and that
    key is a GAIN_DEFS key with has_gain_schema True. Keying off cs.name
    (PascalCase) would make all of these fail."""
    states = [_make_state(name, key) for name, key in _SHIPPED]
    entries = build_entries(states, tuple(GAIN_DEFS.keys()))

    assert len(entries) == len(_SHIPPED)
    for entry, (name, key) in zip(entries, _SHIPPED, strict=True):
        assert entry.config_key == key
        assert entry.config_key in GAIN_DEFS
        assert entry.has_gain_schema is True
        # config_key must be the snake_case key, never the PascalCase class name
        assert entry.config_key != name


# ── the switchable / tunable split ──────────────────────────────────────────
#
# _SHIPPED above is the roster the GUI can TUNE. demo_inference_controller is
# reported by the same /rtc_cm/list_controllers response but has no gain panel,
# so it is deliberately absent from that list and tested here instead.


def test_inference_maps_but_carries_no_gain_schema():
    """It is a normal catalog row — the key still comes from cs.type — and
    has_gain_schema is False, which is the honest answer and the reason the
    radio list cannot be built from that flag any more."""
    cs = _make_state("DemoInferenceController", INFERENCE, groups=("ur5e", "p1b"))
    (entry,) = build_entries([cs], tuple(GAIN_DEFS.keys()))
    assert entry.config_key == INFERENCE
    assert entry.has_gain_schema is False
    assert INFERENCE not in GAIN_DEFS
    assert entry.claimed_groups == ("ur5e", "p1b")


def test_radio_filter_keeps_inference_on_p1b_and_drops_it_elsewhere():
    """The selection rule _refresh_controller_widgets applies, exercised over the
    same catalog response every profile receives.

    The CM instantiates every REGISTERED controller regardless of robot, so the
    response is identical on all three profiles — the filter has to come from the
    profile, not the catalog. Before the split this intersected has_gain_schema
    and dropped inference everywhere.
    """
    states = [_make_state(n, k) for n, k in _SHIPPED]
    states.append(_make_state("DemoInferenceController", INFERENCE))
    entries = build_entries(states, tuple(GAIN_DEFS.keys()))

    for profile_key, expect_inference in (
        ("ur5e_p1b", True),
        ("ur5e_p1a", False),
        ("iiwa7_leap", False),
    ):
        switchable = RobotProfile.for_robot(profile_key).switchable_controllers(
            tuple(GAIN_DEFS.keys())
        )
        radio_keys = [e.config_key for e in entries if e.config_key in switchable]
        assert (INFERENCE in radio_keys) is expect_inference, profile_key
        # the tunable four are offered on every profile, unchanged
        for _name, key in _SHIPPED:
            assert key in radio_keys, f"{key} missing on {profile_key}"


def test_no_external_command_controllers_enable_no_target_panel():
    """Both panels off. JOINT_SPACE cannot express this: it is read with a True
    default, so a controller merely absent from it comes up with a live joint
    panel and a Send button that publishes into a topic nobody subscribes to."""
    for key in NO_EXTERNAL_COMMAND_CONTROLLERS:
        assert target_panel_states(key) == (False, False)
    # the tunable roster is unaffected — each still enables exactly one or both
    for _name, key in _SHIPPED:
        assert any(target_panel_states(key)), key


def test_inference_is_in_the_no_external_command_set():
    """Ties the two tables together: a controller offered as a radio but absent
    from GAIN_DEFS has no gain panel, no parameters and no target subscription,
    so it must also be the one the panel/button gates key off."""
    assert INFERENCE in NO_EXTERNAL_COMMAND_CONTROLLERS


def test_type_equals_registry_config_key_contract():
    """The chosen key source (cs.type) equals the registry/config key for the
    shipped demo controllers — the explicit contract acceptance criterion.
    cs.type is the only field the CM fills from entry.config_key."""
    for _name, key in _SHIPPED:
        cs = _make_state(_name, key)
        (entry,) = build_entries([cs], tuple(GAIN_DEFS.keys()))
        assert entry.ctrl_type == key
        assert entry.config_key == entry.ctrl_type


def test_unknown_controller_has_no_schema():
    """A loaded controller without a GUI gain panel is kept but flagged."""
    cs = _make_state("SomeOtherController", "some_other_controller")
    (entry,) = build_entries([cs], tuple(GAIN_DEFS.keys()))
    assert entry.config_key == "some_other_controller"
    assert entry.has_gain_schema is False


def test_label_override_and_prettify_fallback():
    states = [_make_state(name, key) for name, key in _SHIPPED]
    entries = build_entries(
        states,
        tuple(GAIN_DEFS.keys()),
        label_overrides={"demo_wbc_controller": "WBC"},
    )
    labels = {e.config_key: e.display_label for e in entries}
    assert labels["demo_wbc_controller"] == "WBC"  # override wins
    # prettify fallback derives from the snake_case key, not the class name
    assert labels["demo_joint_controller"] == "Demo Joint Controller"


def test_claimed_groups_and_active_preserved():
    cs = _make_state(
        "DemoJointController", "demo_joint_controller", is_active=True, groups=("ur5e", "p1b")
    )
    (entry,) = build_entries([cs], tuple(GAIN_DEFS.keys()))
    assert entry.is_active is True
    assert entry.claimed_groups == ("ur5e", "p1b")


def test_compliance_gain_tables_differ_from_the_sibling_only_by_the_documented_renames():
    """#469 D-A13: demo_compliance's gain tables are written out, not mirrored.

    S2 generated them with a deepcopy loop; the rename to the §7 schema's
    parameter names ended that, and a hand-transcribed table is exactly the kind
    that drifts silently. So the claim is stated as a test instead: same rows, in
    the same order, with the same widths / defaults / groups, and names that
    differ from the sibling's in EXACTLY the three places
    ``COMPLIANCE_GAIN_RENAMES`` documents.

    This is also what protects the flat wire index ``app.py`` uses to find
    grasp_target_force: the widths are pinned row by row, so the rename cannot
    have moved it. S3 adds admittance ROWS, at which point this test fails and
    that sprint states the new relationship — which is the intended way to find
    out that the two controllers have stopped taking the same parameters.
    """
    task = GAIN_DEFS["demo_task_controller"]
    comp = GAIN_DEFS["demo_compliance_controller"]
    assert len(comp) == len(task)

    for t_row, c_row in zip(task, comp, strict=True):
        expected_name = COMPLIANCE_GAIN_RENAMES.get(t_row[0], t_row[0])
        assert c_row[0] == expected_name
        # width, defaults, is_bool, group — everything but the name
        assert c_row[1:] == t_row[1:], f"{c_row[0]}: layout drifted from {t_row[0]}"

    # Every rename must actually be used; a stale entry in the map would let a
    # transcription error read as an intended rename.
    comp_names = {row[0] for row in comp}
    for old, new in COMPLIANCE_GAIN_RENAMES.items():
        assert new in comp_names, f"{new} is declared a rename of {old} but no row uses it"
        assert old not in comp_names, f"{old} survived the rename"

    # The declared ROS parameter each row dispatches to follows the same rule.
    task_dispatch = GAIN_PARAM_DISPATCH["demo_task_controller"]
    comp_dispatch = GAIN_PARAM_DISPATCH["demo_compliance_controller"]
    assert set(comp_dispatch) == comp_names
    for t_key, (t_param, t_builder) in task_dispatch.items():
        c_key = COMPLIANCE_GAIN_RENAMES.get(t_key, t_key)
        c_param, c_builder = comp_dispatch[c_key]
        assert c_param == COMPLIANCE_GAIN_RENAMES.get(t_param, t_param)
        assert c_builder is t_builder

    # Per-axis row labels move with the names they key on.
    assert GAIN_ROW_NAMES["demo_compliance_controller"] == {
        COMPLIANCE_GAIN_RENAMES.get(k, k): v
        for k, v in GAIN_ROW_NAMES["demo_task_controller"].items()
    }
