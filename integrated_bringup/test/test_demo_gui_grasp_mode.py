"""Unit tests for the demo GUI's grasp-mode readout and Grasp/Release gating.

The Grasp tab's ``▶ Grasp`` / ``■ Release`` buttons drive the controller's
``~/grasp_command`` srv, which only reaches a GraspController when that
controller was configured with ``grasp_controller_type: "force_pi"``. In the
other two whitelist modes the srv answers with a rejection and the hand does
nothing. The GUI used to carry a fixed ``(requires grasp_controller_type:
"force_pi" in YAML)`` hint — right about the rule, silent about the run — so an
operator on a ``contact_stop`` or ``none`` build pressed a live-looking button
and got nothing but a throttled WARN on ``/rosout``.

The controllers now publish the resolved mode as a read-only parameter, and the
GUI gates on it. The decision itself lives in ``demo_gui.config`` as a pure
function so it is testable without a display or a ROS graph; the ``app.py``
halves are driven as unbound methods over a duck-typed state object (the
pattern from ``test_demo_gui_task_frame``).
"""

from __future__ import annotations

import inspect

import pytest

from integrated_bringup.demo_gui.config import (
    GRASP_MODE_FG_ACTIVE,
    GRASP_MODE_FG_BLOCKED,
    GRASP_MODE_FG_UNKNOWN,
    GRASP_MODE_OWNERS,
    GRASP_MODE_UNKNOWN,
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
    """The mode is a configure-time constant — controller switching only
    activates/deactivates, never re-configures — so one successful read holds
    for the CM's lifetime."""
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
