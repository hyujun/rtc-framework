"""Unit tests for the demo GUI's grasp-mode readout, switch and Grasp/Release
gating.

The Grasp tab's ``▶ Grasp`` / ``■ Release`` buttons drive the controller's
``~/grasp_command`` srv, which only reaches a GraspController while that
controller is running ``grasp_controller_type: "force_pi"``. In the other two
whitelist modes the srv answers with a rejection and the hand does nothing. The
GUI used to carry a fixed ``(requires grasp_controller_type: "force_pi" in
YAML)`` hint — right about the rule, silent about the run — so an operator on a
``contact_stop`` or ``none`` build pressed a live-looking button and got nothing
but a throttled WARN on ``/rosout``.

The controllers publish the resolved mode as a parameter, and the GUI gates on
it. That parameter is now also *writable* while the hand is quiet, so the tab
carries a mode combobox + ``Set Mode``: the second half of these tests covers
the request path, the reason a refusal carries, and the cache invalidation that
keeps the readout from asserting a mode the GUI only asked for.

The decision itself lives in ``demo_gui.config`` as a pure function so it is
testable without a display or a ROS graph; the ``app.py`` halves are driven as
unbound methods over a duck-typed state object (the pattern from
``test_demo_gui_task_frame``).
"""

from __future__ import annotations

import inspect

import pytest

from integrated_bringup.demo_gui.config import (
    GRASP_MODE_FG_ACTIVE,
    GRASP_MODE_FG_BLOCKED,
    GRASP_MODE_FG_UNKNOWN,
    GRASP_MODE_OWNERS,
    GRASP_MODE_PARAM,
    GRASP_MODE_UNKNOWN,
    GRASP_MODES,
    grasp_command_enabled,
    grasp_mode_fg,
)

JOINT = "demo_joint_controller"
TASK = "demo_task_controller"
WBC = "demo_wbc_controller"

# The whitelist enforced by ParseGraspHandMode (demo_shared_config.cpp). Kept
# literal rather than imported: this is the contract the C++ side publishes,
# and a test that derived it from the same place the GUI reads it would agree
# with the GUI no matter what either one did.
BLOCKED_MODES = ["contact_stop", "none"]


# ── the decision function ────────────────────────────────────────────────────


@pytest.mark.parametrize("ctrl", [JOINT, TASK])
def test_force_pi_enables_the_buttons(ctrl):
    enabled, text = grasp_command_enabled(ctrl, "force_pi")
    assert enabled is True
    assert "force_pi" in text


@pytest.mark.parametrize("ctrl", [JOINT, TASK])
@pytest.mark.parametrize("mode", BLOCKED_MODES)
def test_non_force_pi_disables_the_buttons(ctrl, mode):
    """The whole point: in these modes the srv is inert, so the button must be
    too — and the label has to name the mode that is actually running, not
    repeat the rule."""
    enabled, text = grasp_command_enabled(ctrl, mode)
    assert enabled is False
    assert mode in text


def test_wbc_is_not_gated_by_the_mode():
    """demo_wbc_controller runs its own 6-state FSM and handles GraspCommand
    directly, so grasp_controller_type says nothing about whether its buttons
    work. It does not even declare the parameter — gating it on the sentinel
    would disable a control that works."""
    assert WBC not in GRASP_MODE_OWNERS
    for mode in [GRASP_MODE_UNKNOWN, "force_pi", *BLOCKED_MODES]:
        enabled, _ = grasp_command_enabled(WBC, mode)
        assert enabled is True, f"WBC gated on mode={mode!r}"


@pytest.mark.parametrize("ctrl", [JOINT, TASK])
def test_unknown_mode_fails_open(ctrl):
    """An unreachable parameter service is not evidence that the controller
    would refuse the command. The GUI must not become a second gate that
    blocks what the controller would have honoured — the controller already
    replies with a precise reason when it will not act."""
    enabled, text = grasp_command_enabled(ctrl, GRASP_MODE_UNKNOWN)
    assert enabled is True
    assert "unknown" in text.lower()


def test_mode_colours_separate_the_three_states():
    assert grasp_mode_fg(JOINT, "force_pi") == GRASP_MODE_FG_ACTIVE
    assert grasp_mode_fg(JOINT, "contact_stop") == GRASP_MODE_FG_BLOCKED
    assert grasp_mode_fg(JOINT, "none") == GRASP_MODE_FG_BLOCKED
    # Unknown gets its own colour rather than the green it shares an
    # enabled-ness with — fail-open must not *look* like a confirmed force_pi.
    assert grasp_mode_fg(JOINT, GRASP_MODE_UNKNOWN) == GRASP_MODE_FG_UNKNOWN
    assert grasp_mode_fg(WBC, GRASP_MODE_UNKNOWN) == GRASP_MODE_FG_ACTIVE


# ── app.py wiring ────────────────────────────────────────────────────────────


class _FakeVar:
    def __init__(self):
        self.value = None

    def set(self, v):
        self.value = v

    def get(self):
        return self.value


class _FakeWidget:
    def __init__(self):
        self.kwargs = {}

    def config(self, **kwargs):
        self.kwargs.update(kwargs)


class _FakeCombo(_FakeWidget):
    """Combobox stand-in: a widget that also carries a selection."""

    def __init__(self):
        super().__init__()
        self.value = GRASP_MODE_UNKNOWN

    def set(self, v):
        self.value = v

    def get(self):
        return self.value


def _gui_state(selected: str):
    """Minimal duck-typed stand-in for the Tk widgets _apply_grasp_mode drives."""

    class _State:
        pass

    state = _State()
    state.selected_ctrl = _FakeVar()
    state.selected_ctrl.set(selected)
    state._grasp_mode_var = _FakeVar()
    state._grasp_mode_label = _FakeWidget()
    state._grasp_btn = _FakeWidget()
    state._release_btn = _FakeWidget()
    state._grasp_modes = {}
    # Mode-switch half of the tab.
    state._grasp_mode_combo = _FakeCombo()
    state._grasp_mode_apply_btn = _FakeWidget()
    state._grasp_mode_result_var = _FakeVar()
    state._grasp_mode_result_label = _FakeWidget()
    state._grasp_mode_seeded = None
    return state


@pytest.mark.parametrize("mode", BLOCKED_MODES)
def test_apply_disables_both_buttons(mode):
    from integrated_bringup.demo_gui.app import DemoControllerGUI

    state = _gui_state(JOINT)
    DemoControllerGUI._apply_grasp_mode(state, JOINT, mode)

    assert state._grasp_btn.kwargs["state"] == "disabled"
    assert state._release_btn.kwargs["state"] == "disabled", "Release left live"
    assert mode in state._grasp_mode_var.get()
    assert state._grasp_mode_label.kwargs["fg"] == GRASP_MODE_FG_BLOCKED


def test_apply_enables_both_buttons_on_force_pi():
    from integrated_bringup.demo_gui.app import DemoControllerGUI

    state = _gui_state(TASK)
    DemoControllerGUI._apply_grasp_mode(state, TASK, "force_pi")

    assert state._grasp_btn.kwargs["state"] == "normal"
    assert state._release_btn.kwargs["state"] == "normal"


def test_apply_ignores_a_result_for_a_deselected_controller():
    """The fetch is async. If it lands after the user moved to another
    controller, painting it would show one controller's mode over another's
    buttons — and could enable buttons the current selection must not have."""
    from integrated_bringup.demo_gui.app import DemoControllerGUI

    state = _gui_state(TASK)
    DemoControllerGUI._apply_grasp_mode(state, TASK, "contact_stop")
    assert state._grasp_btn.kwargs["state"] == "disabled"

    # A late force_pi result for the controller we navigated away from.
    DemoControllerGUI._apply_grasp_mode(state, JOINT, "force_pi")
    assert state._grasp_btn.kwargs["state"] == "disabled", "stale result re-enabled the button"
    assert "contact_stop" in state._grasp_mode_var.get()


def _refresh_state(selected: str, services_ready: bool = False):
    state = _gui_state(selected)
    state.applied = []
    state._apply_grasp_mode = lambda c, m: state.applied.append((c, m))
    state.client_requests = []

    class _Client:
        def services_are_ready(self):
            return services_ready

        def get_parameters(self, names):
            state.client_requests.append(names)
            raise AssertionError("should not be reached when services are down")

    state._get_param_client = lambda c: _Client()
    return state


def test_refresh_uses_the_cache_without_a_round_trip():
    """A cache hit must not hit the parameter service — that is what keeps the
    5 s catalog poll and every radio click off it.

    The original rationale here was "the mode is a configure-time constant, so
    one read holds for the CM's lifetime". That is no longer true: the parameter
    is writable at runtime. The assertion survives unchanged because the cache
    is still authoritative between invalidations, and what used to be an
    immutability argument is now an explicit invalidation contract — pinned by
    ``test_a_successful_set_invalidates_the_cache_before_refetching`` and its
    refusal counterpart rather than by this test.
    """
    from integrated_bringup.demo_gui.app import DemoControllerGUI

    state = _refresh_state(JOINT)
    state._grasp_modes[JOINT] = "contact_stop"
    DemoControllerGUI._refresh_grasp_mode(state, JOINT)

    assert state.applied == [(JOINT, "contact_stop")]
    assert state.client_requests == []


def test_refresh_skips_the_query_for_a_non_owner():
    from integrated_bringup.demo_gui.app import DemoControllerGUI

    state = _refresh_state(WBC)
    DemoControllerGUI._refresh_grasp_mode(state, WBC)

    assert state.applied == [(WBC, GRASP_MODE_UNKNOWN)]
    assert state.client_requests == []


def test_refresh_paints_unknown_and_caches_nothing_when_services_are_down():
    """Two properties at once: the label must drop to unknown rather than keep
    showing the previous controller's mode while a fetch is pending, and a
    failed read must not be cached — that is what makes the next catalog poll
    retry after the CM comes up."""
    from integrated_bringup.demo_gui.app import DemoControllerGUI

    state = _refresh_state(JOINT, services_ready=False)
    DemoControllerGUI._refresh_grasp_mode(state, JOINT)

    assert state.applied == [(JOINT, GRASP_MODE_UNKNOWN)]
    assert state._grasp_modes == {}


def test_the_static_yaml_hint_is_gone():
    """The old label claimed the same thing on every run. If it comes back
    alongside the live readout the tab states the rule twice and the run once."""
    from integrated_bringup.demo_gui import app

    source = inspect.getsource(app)
    assert "requires grasp_controller_type" not in source


def test_selection_change_refreshes_the_mode():
    """The buttons dispatch to the *selected* controller (_send_grasp_command
    reads selected_ctrl), so the gate has to follow the same selection."""
    from integrated_bringup.demo_gui.app import DemoControllerGUI

    source = inspect.getsource(DemoControllerGUI._on_ctrl_radio_change)
    assert "_refresh_grasp_mode" in source


# ── the mode switch (combobox + Set Mode) ────────────────────────────────────


def test_the_offered_modes_are_the_whitelist_and_nothing_else():
    """The combobox values are the controller's whitelist. A GUI-only mode would
    reach the controller as an off-whitelist string and be refused — the failure
    would show up at the operator, not here, which is why the set is pinned."""
    assert set(GRASP_MODES) == {"force_pi", *BLOCKED_MODES}


@pytest.mark.parametrize("ctrl", [JOINT, TASK])
def test_apply_seeds_the_combobox_and_enables_the_switch(ctrl):
    from integrated_bringup.demo_gui.app import DemoControllerGUI

    state = _gui_state(ctrl)
    DemoControllerGUI._apply_grasp_mode(state, ctrl, "contact_stop")

    assert state._grasp_mode_combo.get() == "contact_stop"
    # contact_stop disables Grasp/Release, and it is also the mode you switch
    # out of — gating the switch on that would lock the operator in.
    assert state._grasp_btn.kwargs["state"] == "disabled"
    assert state._grasp_mode_combo.kwargs["state"] == "readonly"
    assert state._grasp_mode_apply_btn.kwargs["state"] == "normal"


def test_apply_leaves_an_unapplied_pick_alone_when_the_poll_repeats():
    """The catalog poll lands here every 5 s with an unchanged mode. Re-seeding
    on each of those would delete the mode the operator picked seconds before
    reaching for Set Mode."""
    from integrated_bringup.demo_gui.app import DemoControllerGUI

    state = _gui_state(JOINT)
    DemoControllerGUI._apply_grasp_mode(state, JOINT, "force_pi")
    state._grasp_mode_combo.set("none")  # operator picks, does not apply yet

    DemoControllerGUI._apply_grasp_mode(state, JOINT, "force_pi")  # poll repaint

    assert state._grasp_mode_combo.get() == "none"


def test_apply_reseeds_when_the_mode_actually_changed():
    """A mode change the GUI did not ask for (another operator's `ros2 param
    set`) is the one case where discarding the pending pick is right: the world
    moved, and the box has to show where it moved to."""
    from integrated_bringup.demo_gui.app import DemoControllerGUI

    state = _gui_state(JOINT)
    DemoControllerGUI._apply_grasp_mode(state, JOINT, "force_pi")
    state._grasp_mode_combo.set("none")

    DemoControllerGUI._apply_grasp_mode(state, JOINT, "contact_stop")

    assert state._grasp_mode_combo.get() == "contact_stop"


def test_apply_reseeds_when_the_selection_moves_to_another_controller():
    """The dirty-check is keyed by (controller, mode), not mode alone. Keyed by
    mode alone, a pick made against the joint controller would stay in the box
    after switching to the task controller and Set Mode would send one
    controller's pick to another."""
    from integrated_bringup.demo_gui.app import DemoControllerGUI

    state = _gui_state(JOINT)
    DemoControllerGUI._apply_grasp_mode(state, JOINT, "force_pi")
    state._grasp_mode_combo.set("none")

    state.selected_ctrl.set(TASK)
    DemoControllerGUI._apply_grasp_mode(state, TASK, "force_pi")

    assert state._grasp_mode_combo.get() == "force_pi"


def test_apply_kills_the_switch_for_a_non_owner():
    """WBC does not declare the parameter, so there is nothing to set — unlike
    its Grasp/Release buttons, which work and stay live."""
    from integrated_bringup.demo_gui.app import DemoControllerGUI

    state = _gui_state(WBC)
    DemoControllerGUI._apply_grasp_mode(state, WBC, GRASP_MODE_UNKNOWN)

    assert state._grasp_mode_combo.kwargs["state"] == "disabled"
    assert state._grasp_mode_apply_btn.kwargs["state"] == "disabled"
    assert state._grasp_mode_combo.get() == GRASP_MODE_UNKNOWN
    assert state._grasp_btn.kwargs["state"] == "normal", "WBC grasp button disabled"


@pytest.mark.parametrize("mode", [GRASP_MODE_UNKNOWN, "quantum_grip"])
def test_apply_blanks_the_combobox_for_a_mode_it_cannot_name(mode):
    """Unknown (service down) and off-whitelist (a controller newer than this
    GUI) both leave the box empty rather than proposing a value. The switch
    itself stays live for an owner — an unreadable parameter is not evidence
    that the controller would refuse the write."""
    from integrated_bringup.demo_gui.app import DemoControllerGUI

    state = _gui_state(JOINT)
    state._grasp_mode_combo.set("force_pi")
    DemoControllerGUI._apply_grasp_mode(state, JOINT, mode)

    assert state._grasp_mode_combo.get() == GRASP_MODE_UNKNOWN
    assert state._grasp_mode_apply_btn.kwargs["state"] == "normal"


class _FakeLogger:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(("info", msg))

    def warn(self, msg):
        self.lines.append(("warn", msg))

    def error(self, msg):
        self.lines.append(("error", msg))

    def debug(self, msg):
        self.lines.append(("debug", msg))


class _FakeRoot:
    """Tk root stand-in: runs the marshalled callback inline so a done-callback
    can be driven to completion inside the test."""

    def after(self, _delay, fn, *args):
        fn(*args)


def _send_state(
    selected: str,
    *,
    successful: bool = True,
    reason: str = "",
    services_ready: bool = True,
    pick: str = "force_pi",
):
    """State for the Set Mode path. The fake future fires its done-callback
    synchronously, so one _send_grasp_mode call exercises the whole chain.

    The three methods of that chain are bound to the state for real, so the test
    drives the actual hand-offs (request → done-callback → outcome + refetch)
    instead of a re-implementation of them. Only the collaborators outside the
    chain are faked: the parameter client, the logger, the Tk root and the
    refetch."""
    import types

    from integrated_bringup.demo_gui.app import DemoControllerGUI

    state = _gui_state(selected)
    for name in ("_send_grasp_mode", "_after_grasp_mode_set", "_set_grasp_mode_result"):
        setattr(state, name, types.MethodType(getattr(DemoControllerGUI, name), state))
    state._grasp_mode_combo.set(pick)
    state.root = _FakeRoot()
    state._logger = _FakeLogger()
    state.get_logger = lambda: state._logger
    state.sent_params = []
    state.refreshes = []
    # Record the cache *as seen by the refetch*: invalidating after the refetch
    # would leave the refetch reading the value it was meant to replace.
    state._refresh_grasp_mode = lambda c: state.refreshes.append((c, dict(state._grasp_modes)))

    class _Future:
        def __init__(self, resp):
            self._resp = resp

        def result(self):
            return self._resp

        def add_done_callback(self, cb):
            cb(self)

    class _Result:
        successful = None
        reason = None

    class _Response:
        pass

    class _Client:
        def wait_for_services(self, timeout_sec=0.0):
            return services_ready

        def set_parameters_atomically(self, params):
            state.sent_params.append(params)
            res = _Result()
            res.successful = successful
            res.reason = reason
            resp = _Response()
            resp.result = res
            return _Future(resp)

    state._get_param_client = lambda c: _Client()
    return state


def test_send_refuses_a_non_owner_without_a_round_trip():
    from integrated_bringup.demo_gui.app import DemoControllerGUI

    state = _send_state(WBC)
    DemoControllerGUI._send_grasp_mode(state)

    assert state.sent_params == []
    assert GRASP_MODE_PARAM in state._grasp_mode_result_var.get()
    assert state._grasp_mode_result_label.kwargs["fg"] == GRASP_MODE_FG_BLOCKED


def test_send_refuses_a_blank_selection_without_a_round_trip():
    """Blank means the live mode has not been read yet. Sending a guessed
    default would switch the hand's control law on the operator's behalf."""
    from integrated_bringup.demo_gui.app import DemoControllerGUI

    state = _send_state(JOINT, pick=GRASP_MODE_UNKNOWN)
    DemoControllerGUI._send_grasp_mode(state)

    assert state.sent_params == []
    assert "select a mode" in state._grasp_mode_result_var.get()


def test_send_reports_unreachable_parameter_services():
    from integrated_bringup.demo_gui.app import DemoControllerGUI

    state = _send_state(JOINT, services_ready=False)
    DemoControllerGUI._send_grasp_mode(state)

    assert state.sent_params == []
    assert state.refreshes == [], "a dropped request must not repaint from a refetch"
    assert "unavailable" in state._grasp_mode_result_var.get()


@pytest.mark.parametrize("ctrl", [JOINT, TASK])
def test_send_transmits_the_selected_mode_as_a_string_parameter(ctrl):
    from rclpy.parameter import Parameter

    from integrated_bringup.demo_gui.app import DemoControllerGUI

    state = _send_state(ctrl, pick="contact_stop")
    DemoControllerGUI._send_grasp_mode(state)

    assert len(state.sent_params) == 1
    (param,) = state.sent_params[0]
    assert param.name == GRASP_MODE_PARAM
    assert param.type_ == Parameter.Type.STRING
    assert param.value == "contact_stop"


def test_a_successful_set_invalidates_the_cache_before_refetching():
    """The switch is only real once the controller confirms it, so the readout
    is repainted from a fresh read rather than from the request. Invalidating
    *before* the refetch is the load-bearing half: the refetch short-circuits on
    a cache hit, so the wrong order would leave the tab showing the mode the
    controller just moved away from."""
    from integrated_bringup.demo_gui.app import DemoControllerGUI

    state = _send_state(JOINT, pick="force_pi")
    state._grasp_modes[JOINT] = "contact_stop"

    DemoControllerGUI._send_grasp_mode(state)

    assert JOINT not in state._grasp_modes
    assert state.refreshes == [(JOINT, {})]
    assert "force_pi" in state._grasp_mode_result_var.get()
    assert state._grasp_mode_result_label.kwargs["fg"] == GRASP_MODE_FG_ACTIVE


def test_a_refusal_shows_the_controller_reason_verbatim_and_still_refetches():
    """The reason names what to do next (open the hand vs release the PI grasp)
    and is the controller's wording, not a paraphrase. The refetch matters on
    this path too: the gate decided against the controller's current mode, which
    is not necessarily the one the cache holds."""
    from integrated_bringup.demo_gui.app import DemoControllerGUI

    reason = "hand is holding (contact_stop latch engaged) — release it first"
    state = _send_state(JOINT, successful=False, reason=reason)
    state._grasp_modes[JOINT] = "contact_stop"

    DemoControllerGUI._send_grasp_mode(state)

    assert reason in state._grasp_mode_result_var.get()
    assert state._grasp_mode_result_label.kwargs["fg"] == GRASP_MODE_FG_BLOCKED
    assert state.refreshes == [(JOINT, {})]


def test_the_refusal_reason_survives_a_catalog_poll_repaint():
    """The reason has its own label for this reason: the poll repaints the mode
    readout every 5 s, so a reason parked there would vanish before it was
    read."""
    from integrated_bringup.demo_gui.app import DemoControllerGUI

    reason = "force_pi grasp is in progress — release it first"
    state = _send_state(JOINT, successful=False, reason=reason)
    DemoControllerGUI._send_grasp_mode(state)

    DemoControllerGUI._apply_grasp_mode(state, JOINT, "contact_stop")  # poll

    assert reason in state._grasp_mode_result_var.get()


def test_an_in_flight_request_is_not_painted_as_accepted():
    """A request that has left the GUI is not a switch that happened. The
    pending line therefore cannot wear the accepted colour — the one that would
    be there if the set had succeeded."""
    from integrated_bringup.demo_gui.app import DemoControllerGUI

    state = _send_state(JOINT)
    pending = []
    real = state._set_grasp_mode_result

    # Watch the sequence rather than the end state: the done-callback fires
    # synchronously here, so by the time _send_grasp_mode returns the label
    # already shows the final outcome.
    def _capture(text, ok):
        pending.append((text, ok))
        real(text, ok)

    state._set_grasp_mode_result = _capture
    DemoControllerGUI._send_grasp_mode(state)

    assert pending[0][1] is None, f"in-flight state painted with ok={pending[0][1]!r}"
    assert "setting" in pending[0][0]
