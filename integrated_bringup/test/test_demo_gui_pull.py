"""Unit tests for the demo_controller_gui pull-estimate panel state machine.

Pure-Python (no Tk mainloop, no rclpy Node, no ROS graph): everything under
test lives in ``demo_gui.pull``, which is why the render state machine was
split out of ``app.py`` in the first place. The acceptance criteria are #234
PR-C 1..6:

1. one immutable snapshot, never a mix of two ticks
2. badge priority SLIP RISK > NO SLIP RISK > UNKNOWN > STALE/UNAVAILABLE
3. pull vector gated on `valid`, contact diagnostics rendered outside it
4. every PullEstimate field stored, plus source / age / frame
5. a rewire drops the previous controller's values
6. peak-hold across the 5 Hz refresh window

The one exception is AC-5, which asserts against ``app.py``'s reset helper —
importing the module is enough (it does not build widgets at import time).
"""

import dataclasses

import pytest

from integrated_bringup.demo_gui.pull import (
    BADGE_NO_SLIP,
    BADGE_SLIP,
    BADGE_STALE,
    BADGE_UNAVAILABLE,
    BADGE_UNKNOWN,
    DIAGNOSTIC_FIELDS,
    PLACEHOLDER,
    PULL_STALE_AFTER_S,
    PULL_UNAVAILABLE,
    SOURCE_FIELDS,
    SOURCE_GRASP,
    SOURCE_WBC,
    VECTOR_FIELDS,
    PullPeakHold,
    PullSnapshot,
    badge_state,
    basis_label,
    build_render,
    format_vec,
    invalid_reason_label,
    mask_label,
)
from rtc_msgs.msg import GraspState, PullEstimate, WbcState

# Fields the GUI adds on top of the wire message.
_GUI_ONLY_FIELDS = {"source", "recv_monotonic", "frame_id"}


def _msg(parent=GraspState, *, frame_id="base_link", **pull_kwargs):
    """Build a parent message carrying a PullEstimate with the given overrides."""
    msg = parent()
    msg.header.frame_id = frame_id
    for key, value in pull_kwargs.items():
        setattr(msg.pull, key, value)
    return msg


def _snap(*, now=100.0, source=SOURCE_GRASP, frame_id="base_link", **pull_kwargs):
    return PullSnapshot.from_message(_msg(frame_id=frame_id, **pull_kwargs), source, now)


# ── AC-1: one immutable snapshot ─────────────────────────────────────────────


def test_snapshot_is_frozen():
    """The renderer holds a reference across a whole frame; the callback must
    not be able to edit it out from under the render."""
    snap = _snap(magnitude=1.0)
    with pytest.raises(dataclasses.FrozenInstanceError):
        snap.magnitude = 2.0


def test_snapshot_never_mixes_two_ticks():
    """Two ticks produce two independent snapshots. The regression this guards
    is the pre-PR-C layout: six loose attributes written one at a time, where a
    render landing mid-write saw tick A's magnitude with tick B's valid flag."""
    tick_a = _snap(now=10.0, magnitude=5.0, valid=True, valid_contact_count=3)
    tick_b = _snap(now=11.0, magnitude=0.0, valid=False, valid_contact_count=0)

    # tick_a is untouched by the later store, and each snapshot is internally
    # consistent — the pairing of magnitude with valid can never cross ticks.
    assert (tick_a.magnitude, tick_a.valid, tick_a.valid_contact_count) == (5.0, True, 3)
    assert (tick_b.magnitude, tick_b.valid, tick_b.valid_contact_count) == (0.0, False, 0)
    assert tick_a.recv_monotonic != tick_b.recv_monotonic


def test_unavailable_is_a_shared_never_received_sentinel():
    assert PullSnapshot.unavailable() is PULL_UNAVAILABLE
    assert PULL_UNAVAILABLE.received is False
    assert PULL_UNAVAILABLE.age_s(123.0) is None
    assert PULL_UNAVAILABLE.is_stale(123.0) is True


# ── AC-4: every wire field is stored ─────────────────────────────────────────


def test_snapshot_covers_every_pull_estimate_field():
    """Storage coverage is asserted against the message definition, not a
    hand-written list: a future PullEstimate field fails this test instead of
    being silently dropped by the GUI (the #234 P-8 regression, where 6 of 17
    fields were kept)."""
    wire_fields = set(PullEstimate.get_fields_and_field_types())
    stored = {f.name for f in dataclasses.fields(PullSnapshot)}
    assert wire_fields <= stored, f"not stored by the GUI: {sorted(wire_fields - stored)}"
    # And nothing extra beyond the deliberate GUI-side additions.
    assert stored - wire_fields == _GUI_ONLY_FIELDS


def test_from_message_round_trips_every_field():
    """Each field arrives with its own value — a copy/paste slip that wired two
    fields to the same source would survive the coverage test above."""
    msg = _msg(
        force=[1.0, 2.0, 3.0],
        force_inplane=[4.0, 5.0],
        plane_normal=[0.0, 0.0, 1.0],
        basis_x=[1.0, 0.0, 0.0],
        basis_source=PullEstimate.BASIS_CARRY,
        invalid_reason=PullEstimate.INVALID_REQUIRED_CONTACT,
        contact_mask=0b0101,
        touch_mask=0b1111,
        magnitude=6.0,
        directional=7.0,
        friction_utilization=0.75,
        leakage_bound=8.0,
        valid_contact_count=2,
        valid=True,
        slip_risk=True,
        any_saturated=True,
        baseline_applied=True,
    )
    snap = PullSnapshot.from_message(msg, SOURCE_WBC, now=42.0)

    assert snap.force == (1.0, 2.0, 3.0)
    assert snap.force_inplane == (4.0, 5.0)
    assert snap.plane_normal == (0.0, 0.0, 1.0)
    assert snap.basis_x == (1.0, 0.0, 0.0)
    assert snap.basis_source == PullEstimate.BASIS_CARRY
    assert snap.invalid_reason == PullEstimate.INVALID_REQUIRED_CONTACT
    assert (snap.contact_mask, snap.touch_mask) == (0b0101, 0b1111)
    assert (snap.magnitude, snap.directional) == (6.0, 7.0)
    assert (snap.friction_utilization, snap.leakage_bound) == (0.75, 8.0)
    assert snap.valid_contact_count == 2
    assert (snap.valid, snap.slip_risk, snap.any_saturated, snap.baseline_applied) == (
        True,
        True,
        True,
        True,
    )
    # GUI-side provenance
    assert (snap.source, snap.recv_monotonic, snap.frame_id) == (SOURCE_WBC, 42.0, "base_link")


def test_both_parent_messages_are_read_identically():
    """GraspState and WbcState embed the same PullEstimate, which is what lets
    one helper serve both callbacks."""
    kwargs = {"magnitude": 3.0, "valid": True, "contact_mask": 0b0011}
    from_grasp = PullSnapshot.from_message(_msg(GraspState, **kwargs), SOURCE_GRASP, 1.0)
    from_wbc = PullSnapshot.from_message(_msg(WbcState, **kwargs), SOURCE_WBC, 1.0)
    assert dataclasses.replace(from_grasp, source=SOURCE_WBC) == from_wbc


def test_age_and_frame_are_rendered():
    snap = _snap(now=100.0, frame_id="ur5e_base_link", valid=True)
    render = build_render(snap, 100.25)
    assert render.values["frame"] == "ur5e_base_link"
    assert "0.25 s" in render.values["source"]
    assert "GraspState" in render.values["source"]

    never = build_render(PULL_UNAVAILABLE, 100.0)
    assert never.values["frame"] == PLACEHOLDER
    assert PLACEHOLDER in never.values["source"]


def test_age_uses_receive_time_and_never_goes_negative():
    """Age is receiver-side by construction: the question is 'is this process
    still hearing from that controller', which the publisher's stamp cannot
    answer. The clamp keeps a caller that passes an earlier `now` from
    rendering a negative age."""
    snap = _snap(now=100.0)
    assert snap.age_s(100.5) == pytest.approx(0.5)
    assert snap.age_s(99.0) == 0.0


# ── AC-2: badge priority ─────────────────────────────────────────────────────

_FRESH = 100.0


@pytest.mark.parametrize(
    ("kwargs", "expected"),
    [
        # fresh + slip_risk wins over everything else on a fresh tick, including
        # an invalid vector — slip is evaluated outside the `valid` gate (P-7).
        ({"slip_risk": True, "valid": True}, BADGE_SLIP),
        ({"slip_risk": True, "valid": False}, BADGE_SLIP),
        ({"slip_risk": False, "valid": True}, BADGE_NO_SLIP),
        ({"slip_risk": False, "valid": False}, BADGE_UNKNOWN),
    ],
)
def test_badge_priority_on_fresh_ticks(kwargs, expected):
    badge = badge_state(_snap(now=_FRESH, **kwargs), _FRESH)
    assert badge.text == expected


def test_badge_stale_outranks_every_fresh_state():
    """A slip flag from a publisher that has gone quiet must not keep alarming;
    conversely a stale `valid` tick must not read as healthy."""
    for kwargs in ({"slip_risk": True}, {"valid": True}, {}):
        snap = _snap(now=_FRESH, **kwargs)
        badge = badge_state(snap, _FRESH + PULL_STALE_AFTER_S + 0.01)
        assert badge.text == BADGE_STALE
        assert "ago" in badge.detail


def test_badge_boundary_is_inclusive_of_the_threshold():
    snap = _snap(now=_FRESH, valid=True)
    assert badge_state(snap, _FRESH + PULL_STALE_AFTER_S).text == BADGE_NO_SLIP
    assert badge_state(snap, _FRESH + PULL_STALE_AFTER_S * 1.001).text == BADGE_STALE


def test_badge_unavailable_when_nothing_received():
    badge = badge_state(PULL_UNAVAILABLE, _FRESH)
    assert badge.text == BADGE_UNAVAILABLE
    assert badge.detail == "no estimate received"


def test_badge_estop_overrides_a_fresh_tick():
    """E-STOP keeps publishing (#234 P-1 guarantees a row every tick), but
    StageEstopPullTick drives the estimator with an all-invalid span and a zero
    normal — so the tick reports INVALID_DEGENERATE_NORMAL. Reporting that as
    'UNKNOWN: degenerate normal' would name a geometry failure that never
    happened."""
    snap = _snap(now=_FRESH, valid=False, invalid_reason=PullEstimate.INVALID_DEGENERATE_NORMAL)
    badge = badge_state(snap, _FRESH, estop_active=True)
    assert badge.text == BADGE_UNAVAILABLE
    assert badge.detail == "E-STOP active"


def test_badge_unknown_carries_the_wire_cause():
    """PR-B put `invalid_reason` on the wire precisely so the UNKNOWN badge does
    not collapse every failure into a bare valid=False."""
    for reason in (
        PullEstimate.INVALID_NOT_INITIALIZED,
        PullEstimate.INVALID_DEGENERATE_NORMAL,
        PullEstimate.INVALID_REQUIRED_CONTACT,
        PullEstimate.INVALID_INSUFFICIENT_CONTACTS,
    ):
        badge = badge_state(_snap(now=_FRESH, valid=False, invalid_reason=reason), _FRESH)
        assert badge.text == BADGE_UNKNOWN
        assert badge.detail == invalid_reason_label(reason)
        assert not badge.detail.startswith("?"), "unlabelled invalid_reason"


def test_badge_key_covers_detail():
    """The Tk dirty-check keys off `key`; a detail-only change (same badge, new
    cause) still has to repaint."""
    a = badge_state(
        _snap(now=_FRESH, invalid_reason=PullEstimate.INVALID_REQUIRED_CONTACT), _FRESH
    )
    b = badge_state(
        _snap(now=_FRESH, invalid_reason=PullEstimate.INVALID_INSUFFICIENT_CONTACTS), _FRESH
    )
    assert a.text == b.text
    assert a.key != b.key


# ── AC-3: valid gate vs partial-contact diagnostics ──────────────────────────


def _all_keys():
    return set(VECTOR_FIELDS) | set(DIAGNOSTIC_FIELDS) | set(SOURCE_FIELDS)


def test_render_emits_exactly_the_declared_fields():
    """The panel builds one widget per emitted key, so a key the render emits
    without a widget is a KeyError at 5 Hz."""
    render = build_render(_snap(now=_FRESH), _FRESH)
    assert set(render.values) == _all_keys()


def test_invalid_tick_blanks_the_vector_but_keeps_diagnostics():
    """The AC-3 core: a required-role dropout invalidates the pull vector while
    the tips still holding can still be slipping. Blanking the diagnostics along
    with the vector would hide exactly the case the panel exists for."""
    snap = _snap(
        now=_FRESH,
        valid=False,
        invalid_reason=PullEstimate.INVALID_REQUIRED_CONTACT,
        magnitude=9.0,
        directional=9.0,
        leakage_bound=9.0,
        force=[9.0, 9.0, 9.0],
        force_inplane=[9.0, 9.0],
        basis_x=[1.0, 0.0, 0.0],
        basis_source=PullEstimate.BASIS_REFERENCE,
        friction_utilization=0.9,
        contact_mask=0b0110,
        touch_mask=0b1110,
        valid_contact_count=2,
        any_saturated=True,
        baseline_applied=True,
        plane_normal=[0.0, 0.0, 1.0],
    )
    render = build_render(snap, _FRESH)

    # Vector fields blank — except `valid` itself, which reports the cause.
    for key in VECTOR_FIELDS:
        if key == "valid":
            continue
        assert render.values[key] == PLACEHOLDER, key
    assert render.values["valid"] == invalid_reason_label(PullEstimate.INVALID_REQUIRED_CONTACT)

    # Diagnostics still rendered, none of them the placeholder.
    for key in DIAGNOSTIC_FIELDS:
        assert render.values[key] != PLACEHOLDER, key
    assert render.values["friction_util"] == "0.90"
    assert render.values["contact_set"].endswith("(2)")
    assert render.values["saturated"] == "YES"


def test_valid_tick_renders_the_vector():
    snap = _snap(
        now=_FRESH,
        valid=True,
        magnitude=1.5,
        directional=-0.25,
        leakage_bound=0.125,
        force=[1.0, 0.0, -1.0],
        force_inplane=[1.0, 0.5],
        basis_x=[1.0, 0.0, 0.0],
        basis_source=PullEstimate.BASIS_REFERENCE,
    )
    render = build_render(snap, _FRESH)
    assert render.vector_ok is True
    assert render.values["magnitude"] == "1.50 N"
    assert render.values["directional"] == "-0.25 N"
    assert render.values["leakage"] == "0.12 N"
    assert render.values["force"] == "(1.00, 0.00, -1.00)"
    assert render.values["inplane"] == "(1.00, 0.50)"
    assert render.values["valid"] == "OK"
    for key in _all_keys():
        assert render.values[key] != PLACEHOLDER, key


def test_plane_normal_survives_an_invalid_tick_but_basis_x_does_not():
    """PullEstimate.msg: a tick can carry a usable normal but no basis (the
    plane was fine, a required tip dropped out). basis_x is zero on any tick
    that was not valid, so gating the two the same way would either hide a good
    normal or render a zero vector as if it meant something."""
    snap = _snap(
        now=_FRESH,
        valid=False,
        invalid_reason=PullEstimate.INVALID_REQUIRED_CONTACT,
        plane_normal=[0.0, 0.0, 1.0],
        basis_x=[0.0, 0.0, 0.0],
    )
    render = build_render(snap, _FRESH)
    assert render.values["plane_normal"] == "(0.00, 0.00, 1.00)"
    assert render.values["basis_x"] == PLACEHOLDER


def test_stale_blanks_everything_except_provenance():
    """When the badge says STALE the numbers are gone — but *which* controller
    went quiet and how long ago is exactly what is still needed."""
    snap = _snap(now=_FRESH, valid=True, magnitude=1.0, friction_utilization=0.5)
    render = build_render(snap, _FRESH + PULL_STALE_AFTER_S + 1.0)
    assert render.trusted is False
    for key in set(VECTOR_FIELDS) | set(DIAGNOSTIC_FIELDS):
        assert render.values[key] == PLACEHOLDER, key
    assert render.values["frame"] == "base_link"
    assert "GraspState" in render.values["source"]


def test_estop_blanks_diagnostics_too():
    """The E-STOP badge and the field gates are derived from the same decision,
    so the panel can never show live-looking numbers under an UNAVAILABLE
    header."""
    render = build_render(_snap(now=_FRESH, friction_utilization=0.4), _FRESH, estop_active=True)
    assert render.trusted is False
    assert render.values["friction_util"] == PLACEHOLDER


def test_gate_is_derived_from_the_badge():
    """Cross-check across the whole state space: `trusted` must be exactly
    'the badge is not STALE/UNAVAILABLE'."""
    cases = [
        (PULL_UNAVAILABLE, _FRESH, False),
        (_snap(now=_FRESH), _FRESH, False),
        (_snap(now=_FRESH), _FRESH + 10.0, False),
        (_snap(now=_FRESH, valid=True), _FRESH, False),
        (_snap(now=_FRESH, slip_risk=True), _FRESH, True),
    ]
    for snap, now, estop in cases:
        render = build_render(snap, now, estop_active=estop)
        blanked = render.badge.text in (BADGE_STALE, BADGE_UNAVAILABLE)
        assert render.trusted is not blanked
        assert render.vector_ok == (render.trusted and snap.valid)


# ── AC-3/4 helpers: labels ───────────────────────────────────────────────────


def test_basis_labels_cover_every_wire_value():
    for value in (
        PullEstimate.BASIS_NONE,
        PullEstimate.BASIS_REFERENCE,
        PullEstimate.BASIS_CARRY,
        PullEstimate.BASIS_SEED,
    ):
        assert not basis_label(value).startswith("?"), value
    # An unmapped value is surfaced, never silently rendered as a known rule.
    assert basis_label(99).startswith("?")


def test_basis_is_shown_as_the_rule_not_an_axis_name():
    """force_inplane[0] is the configured reference component only under
    BASIS_REFERENCE; naming an axis under the fallbacks would be a lie
    (#234 P-11)."""
    for value, expected in (
        (PullEstimate.BASIS_REFERENCE, "reference"),
        (PullEstimate.BASIS_CARRY, "carry"),
        (PullEstimate.BASIS_SEED, "seed"),
    ):
        render = build_render(_snap(now=_FRESH, valid=True, basis_source=value), _FRESH)
        assert render.values["basis"] == expected


def test_mask_label_is_lsb_first():
    """Bit k is contact k in the estimator's configured tip order, so the string
    must read in tip order — the opposite of a binary literal."""
    assert mask_label(0b0001, 4) == "#..."
    assert mask_label(0b1000, 4) == "...#"
    assert mask_label(0b0000, 4) == "...."
    assert mask_label(0b11111111, 8) == "########"


def test_contact_and_touch_masks_render_independently():
    """The two masks are different sets (touch is axis-independent and includes
    the thumb), so 'nothing is touching' stays distinguishable from a
    thumb-only grasp."""
    render = build_render(
        _snap(now=_FRESH, contact_mask=0b0010, touch_mask=0b1011, valid_contact_count=1), _FRESH
    )
    assert render.values["contact_set"] == ".#.. (1)"
    assert render.values["touch_set"] == "##.#"


def test_format_vec_precision():
    assert format_vec((0.0, -0.005, 1.2345)) == "(0.00, -0.01, 1.23)"


# ── AC-6: peak-hold across the refresh window ────────────────────────────────


def test_peak_hold_reports_the_frame_peak_not_the_last_sample():
    """The wire runs at control_rate and the panel at 5 Hz, so sampling the
    newest snapshot shows roughly one tick in a hundred — precisely the ticks
    where a transient slip spike is invisible."""
    hold = PullPeakHold()
    spike = _snap(now=_FRESH, friction_utilization=0.95, slip_risk=True)
    for snap in (
        _snap(now=_FRESH, friction_utilization=0.1),
        spike,
        _snap(now=_FRESH, friction_utilization=0.2),
    ):
        hold.observe(snap)
    latest = _snap(now=_FRESH, friction_utilization=0.2)

    merged = hold.merge_into(latest)
    assert merged.friction_utilization == pytest.approx(0.95)
    assert merged.slip_risk is True
    # Everything else stays tick-coherent with the newest snapshot.
    assert merged.magnitude == latest.magnitude
    assert merged.recv_monotonic == latest.recv_monotonic


def test_peak_hold_is_consumed_each_frame():
    """Not a decaying hold: the value shown belongs to the frame just ended, so
    a spike is visible for one frame and a quiet frame reads quiet."""
    hold = PullPeakHold()
    hold.observe(_snap(now=_FRESH, friction_utilization=0.9, slip_risk=True))
    quiet = _snap(now=_FRESH, friction_utilization=0.1)

    first = hold.merge_into(quiet)
    assert first.friction_utilization == pytest.approx(0.9)
    assert first.slip_risk is True

    hold.observe(quiet)
    second = hold.merge_into(quiet)
    assert second.friction_utilization == pytest.approx(0.1)
    assert second.slip_risk is False


def test_peak_hold_with_no_samples_returns_the_snapshot_unchanged():
    """A frame during which nothing arrived has no peak to report — reporting a
    zero peak would erase the last known value and read as 'no slip'."""
    hold = PullPeakHold()
    snap = _snap(now=_FRESH, friction_utilization=0.6, slip_risk=True)
    assert hold.merge_into(snap) is snap


def test_peak_hold_never_downgrades_the_snapshot():
    """merge_into takes the max / OR, so a frame whose samples were all quieter
    than the newest tick cannot mask that tick's own value."""
    hold = PullPeakHold()
    hold.observe(_snap(now=_FRESH, friction_utilization=0.1, slip_risk=False))
    latest = _snap(now=_FRESH, friction_utilization=0.7, slip_risk=True)
    merged = hold.merge_into(latest)
    assert merged.friction_utilization == pytest.approx(0.7)
    assert merged.slip_risk is True


def test_peak_hold_reset_drops_the_previous_source():
    """A peak belongs to the controller that produced it — a rewire must not
    carry it into the next controller's first frame."""
    hold = PullPeakHold()
    hold.observe(_snap(now=_FRESH, friction_utilization=0.99, slip_risk=True))
    hold.reset()
    assert hold.merge_into(PULL_UNAVAILABLE) is PULL_UNAVAILABLE
    fresh = _snap(now=_FRESH, friction_utilization=0.0)
    hold.observe(fresh)
    merged = hold.merge_into(fresh)
    assert merged.friction_utilization == 0.0
    assert merged.slip_risk is False


def test_peak_hold_is_not_applied_to_an_unavailable_snapshot():
    """Post-rewire the panel must be blank, not blank-with-a-slip-flag."""
    hold = PullPeakHold()
    hold.observe(_snap(now=_FRESH, friction_utilization=0.8, slip_risk=True))
    assert hold.merge_into(PULL_UNAVAILABLE) is PULL_UNAVAILABLE


# ── AC-5: a rewire drops the previous controller's values ────────────────────


def test_rewire_resets_controller_sourced_state():
    """`_rewire_owned_topics` used to recreate the subscription handles and
    leave the values they had fed in place, so a controller switch showed the
    previous controller's pull numbers as if they were live (#234 P-10).

    Driven against the reset helper directly — the surrounding method destroys
    and recreates rclpy handles, which needs a live node.
    """
    from integrated_bringup.demo_gui.app import DemoControllerGUI

    class _State:
        pass

    state = _State()
    state._pull = _snap(now=_FRESH, magnitude=7.0, valid=True, slip_risk=True)
    state._pull_peak = PullPeakHold()
    state._pull_peak.observe(state._pull)
    state._wbc_active = True

    DemoControllerGUI._reset_controller_sourced_state(state)

    assert state._pull is PULL_UNAVAILABLE
    assert state._pull_peak.samples == 0
    assert state._pull_peak.slip_risk is False
    assert state._pull_peak.friction_utilization == 0.0
    # The phase-label source flag goes with it: a stale True decodes the next
    # controller's phase against the WBC enum instead of the Force-PI one.
    assert state._wbc_active is False

    # And the panel renders as never-received rather than as the old values.
    render = build_render(state._pull_peak.merge_into(state._pull), _FRESH)
    assert render.badge.text == BADGE_UNAVAILABLE
    for key in set(VECTOR_FIELDS) | set(DIAGNOSTIC_FIELDS):
        assert render.values[key] == PLACEHOLDER, key


def test_rewire_reset_is_called_by_rewire_owned_topics():
    """Guards the wiring itself: the helper is only useful if the rewire path
    actually calls it."""
    import inspect

    from integrated_bringup.demo_gui.app import DemoControllerGUI

    source = inspect.getsource(DemoControllerGUI._rewire_owned_topics)
    assert "_reset_controller_sourced_state()" in source


def test_panel_declares_a_widget_for_every_rendered_field():
    """The render/widget key contract. ``_refresh_pull_panel`` indexes
    ``self._pull_labels[key]`` for every key ``build_render`` emits, so a
    mismatch is a KeyError five times a second — and neither side alone can
    catch it. Asserted against the panel builder's source so it also holds on a
    headless machine, where the Tk smoke test below skips.
    """
    import inspect
    import re

    from integrated_bringup.demo_gui.app import DemoControllerGUI

    source = inspect.getsource(DemoControllerGUI._build_pull_panel)
    declared = set(re.findall(r'\("[^"]+:",\s*"([a-z_]+)"\)', source))
    emitted = set(build_render(_snap(now=_FRESH), _FRESH).values)
    assert declared == emitted, (
        f"widgets without a render value: {sorted(declared - emitted)}; "
        f"render values without a widget: {sorted(emitted - declared)}"
    )


def test_panel_renders_every_state_against_real_widgets():
    """End-to-end over the actual Tk widgets: build the panel, drive it through
    every badge state, and read the rendered text back. Skips without a display
    (the key contract above is the headless guard)."""
    tk = pytest.importorskip("tkinter")
    from tkinter import font as tkfont

    from integrated_bringup.demo_gui.app import DemoControllerGUI

    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("no display available")
    root.withdraw()
    try:
        gui = DemoControllerGUI.__new__(DemoControllerGUI)
        gui._pull_labels = {}
        gui._prev_pull = {}
        gui._pull = PULL_UNAVAILABLE
        gui._pull_peak = PullPeakHold()
        gui.estop_active = False
        DemoControllerGUI._build_pull_panel(
            gui, tk.Frame(root), tkfont.Font(family="Courier", size=9)
        )

        def texts():
            return {key: widget.cget("text") for key, widget in gui._pull_labels.items()}

        def badge_text():
            return gui._pull_slip_label.cget("text").strip()

        msg = _msg(
            valid=True,
            magnitude=1.25,
            friction_utilization=0.42,
            contact_mask=0b0101,
            touch_mask=0b1101,
            valid_contact_count=2,
            basis_source=PullEstimate.BASIS_REFERENCE,
            plane_normal=[0.0, 0.0, 1.0],
            basis_x=[1.0, 0.0, 0.0],
            force=[0.1, 0.2, 0.3],
            force_inplane=[0.1, 0.2],
        )

        # Before the first message: blank, and explicitly so.
        DemoControllerGUI._refresh_pull_panel(gui)
        assert badge_text() == BADGE_UNAVAILABLE
        assert set(texts().values()) <= {PLACEHOLDER, f"{PLACEHOLDER} · {PLACEHOLDER}"}

        # A live valid tick.
        DemoControllerGUI._store_pull_fields(gui, msg, SOURCE_GRASP)
        DemoControllerGUI._refresh_pull_panel(gui)
        live = texts()
        assert badge_text() == BADGE_NO_SLIP
        assert live["magnitude"] == "1.25 N"
        assert live["contact_set"] == "#.#. (2)"
        assert live["basis"] == "reference"
        assert live["frame"] == "base_link"
        assert PLACEHOLDER not in live.values()

        # The same tick gone stale.
        gui._pull = PullSnapshot.from_message(msg, SOURCE_GRASP, -1.0e6)
        DemoControllerGUI._refresh_pull_panel(gui)
        stale = texts()
        assert badge_text() == BADGE_STALE
        assert stale["magnitude"] == PLACEHOLDER
        assert stale["friction_util"] == PLACEHOLDER
        assert stale["frame"] == "base_link"  # provenance survives

        # E-STOP over a fresh tick.
        gui.estop_active = True
        DemoControllerGUI._store_pull_fields(gui, msg, SOURCE_GRASP)
        DemoControllerGUI._refresh_pull_panel(gui)
        assert badge_text() == BADGE_UNAVAILABLE
        assert gui._pull_badge_detail.cget("text") == "E-STOP active"
        assert texts()["magnitude"] == PLACEHOLDER

        # Rewire wipes provenance too — the frame belonged to the old source.
        gui.estop_active = False
        DemoControllerGUI._reset_controller_sourced_state(gui)
        DemoControllerGUI._refresh_pull_panel(gui)
        assert badge_text() == BADGE_UNAVAILABLE
        assert texts()["frame"] == PLACEHOLDER
    finally:
        root.destroy()


def test_app_keeps_no_per_field_pull_attributes():
    """AC-1 at the app layer: the six loose `self._pull_<field>` attributes are
    gone, so there is nothing left that a render could read out of step with
    the snapshot. Asserted against the source because the attributes only ever
    existed on a constructed node (which needs a ROS graph and a Tk thread)."""
    import inspect
    import re

    from integrated_bringup.demo_gui import app as app_module

    source = inspect.getsource(app_module)
    banned = re.findall(
        r"self\._pull_(magnitude|friction_utilization|valid|valid_contact_count"
        r"|slip_risk|baseline_applied)\b",
        source,
    )
    assert not banned, f"per-field pull attributes still present: {sorted(set(banned))}"
