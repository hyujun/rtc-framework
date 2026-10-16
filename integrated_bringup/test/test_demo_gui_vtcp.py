"""Unit tests for the demo GUI's Virtual TCP panel (#847).

The task, compliance and joint controllers declare ``virtual_tcp_mode``,
``virtual_tcp_offset`` and ``virtual_tcp_orientation`` and accept a set only
while inactive. The panel reads them, edits them, and gates Apply on whether the
selected controller is the active one. What these tests hold:

- the pure decisions in ``demo_gui.vtcp`` (parsing, input validation, the
  re-seed rule, the fail-closed Apply gate),
- that the names, modes and owning controllers agree with the C++ side, read
  from its source rather than copied,
- the ``app.py`` wiring that pytest can see: the three places the Apply gate is
  repainted, a single ``set_parameters_atomically`` per Apply, the controller's
  refusal shown verbatim, and the cache dropped before the re-read.

Tk layout and the buttons on a live screen are checked at runtime (verify
skill), as for every other panel.
"""

from __future__ import annotations

import inspect
import math
import re
import types
from pathlib import Path

import pytest

from integrated_bringup.demo_gui.vtcp import (
    HINT_ACTIVE,
    HINT_ACTIVE_UNKNOWN,
    HINT_INACTIVE,
    HINT_NOT_OWNER,
    VTCP_MODE_PARAM,
    VTCP_MODES,
    VTCP_OFFSET_PARAM,
    VTCP_ORIENTATION_PARAM,
    VTCP_OWNERS,
    VTCP_PARAMS,
    VtcpConfigView,
    apply_enabled,
    format_readout,
    parse_entries,
    parse_inputs,
    seed_due,
)

JOINT = "demo_joint_controller"
TASK = "demo_task_controller"
COMPLIANCE = "demo_compliance_controller"
WBC = "demo_wbc_controller"

_PKG = Path(__file__).resolve().parents[1]
_HELPER_HEADER = _PKG / "include" / "integrated_bringup" / "support" / "virtual_tcp_params.hpp"
_CONTROLLERS = _PKG / "src" / "controllers"

_STRING = 4  # rcl_interfaces/msg/ParameterType.PARAMETER_STRING
_DOUBLE_ARRAY = 8  # rcl_interfaces/msg/ParameterType.PARAMETER_DOUBLE_ARRAY


def _value(type_: int, string_value: str = "", double_array_value=()):
    return types.SimpleNamespace(
        type=type_, string_value=string_value, double_array_value=list(double_array_value)
    )


def _answer(mode="constant", offset=(0.0, 0.0, 0.1), rpy=(0.0, 0.0, 0.0)):
    return [
        _value(_STRING, string_value=mode),
        _value(_DOUBLE_ARRAY, double_array_value=offset),
        _value(_DOUBLE_ARRAY, double_array_value=rpy),
    ]


# ── the C++ side is the owner — pin the copies here to it ────────────────────


def test_parameter_type_codes_match_rcl_interfaces():
    from rcl_interfaces.msg import ParameterType

    assert ParameterType.PARAMETER_STRING == _STRING
    assert ParameterType.PARAMETER_DOUBLE_ARRAY == _DOUBLE_ARRAY


def test_parameter_names_match_the_cpp_helper():
    src = _HELPER_HEADER.read_text()
    declared = dict(re.findall(r'constexpr const char\* (kVirtualTcp\w+Param) = "([^"]+)";', src))
    assert declared == {
        "kVirtualTcpModeParam": VTCP_MODE_PARAM,
        "kVirtualTcpOffsetParam": VTCP_OFFSET_PARAM,
        "kVirtualTcpOrientationParam": VTCP_ORIENTATION_PARAM,
    }
    assert VTCP_PARAMS == (VTCP_MODE_PARAM, VTCP_OFFSET_PARAM, VTCP_ORIENTATION_PARAM)


def test_modes_match_the_cpp_whitelist_in_enum_order():
    src = _HELPER_HEADER.read_text()
    table = src[
        src.index("kVirtualTcpModeNames{{") : src.index("}};", src.index("kVirtualTcpModeNames"))
    ]
    assert tuple(re.findall(r'VirtualTcpMode::k\w+, "(\w+)"', table)) == VTCP_MODES


def test_owners_are_the_controllers_that_declare_the_parameters():
    """VTCP_OWNERS is the only Python definition; the C++ side says who really
    declares the keys. A controller added on either side alone fails here."""
    declaring = {
        f"demo_{p.parent.name}_controller"
        for p in _CONTROLLERS.glob("*/parameters.cpp")
        if "DeclareVirtualTcpParameters(" in p.read_text()
    }
    assert declaring == set(VTCP_OWNERS)
    assert {JOINT, TASK, COMPLIANCE} == set(VTCP_OWNERS)


# ── pure decisions ───────────────────────────────────────────────────────────


def test_parse_entries_reads_the_declared_shape():
    view = parse_entries(_answer("weighted", (0.01, -0.02, 0.15), (0.1, 0.0, -0.1)))
    assert view == VtcpConfigView("weighted", (0.01, -0.02, 0.15), (0.1, 0.0, -0.1))


@pytest.mark.parametrize(
    "answer",
    [
        [],  # no answer
        _answer()[:2],  # one entry short
        [_value(0), *_answer()[1:]],  # PARAMETER_NOT_SET: a controller older than #847
        [_value(_STRING, "fixed"), *_answer()[1:]],  # off-whitelist mode
        [_answer()[0], _value(_STRING, "x"), _answer()[2]],  # wrong type
        _answer(offset=(0.0, 0.1)),  # short array
        _answer(rpy=(0.0, math.nan, 0.0)),  # non-finite
    ],
)
def test_parse_entries_returns_unknown_instead_of_a_guess(answer):
    assert parse_entries(answer) is None


def test_parse_inputs_accepts_three_numbers_each():
    view, why = parse_inputs("constant", ["0", "0", "0.15"], ["0.1", "-0.2", "1e-3"])
    assert why == ""
    assert view == VtcpConfigView("constant", (0.0, 0.0, 0.15), (0.1, -0.2, 0.001))


@pytest.mark.parametrize(
    ("mode", "offset", "rpy", "fragment"),
    [
        ("", ["0", "0", "0"], ["0", "0", "0"], "mode"),  # not read yet
        ("fixed", ["0", "0", "0"], ["0", "0", "0"], "mode"),
        ("constant", ["0", "", "0"], ["0", "0", "0"], "offset"),
        ("constant", ["0", "0", "0"], ["0", "abc", "0"], "rpy"),
        ("constant", ["nan", "0", "0"], ["0", "0", "0"], "finite"),
        ("constant", ["0", "0"], ["0", "0", "0"], "offset"),
    ],
)
def test_parse_inputs_refuses_before_a_round_trip(mode, offset, rpy, fragment):
    view, why = parse_inputs(mode, offset, rpy)
    assert view is None
    assert fragment in why


def test_readout_names_every_value_and_admits_unknown():
    text = format_readout(VtcpConfigView("constant", (0.0, 0.0, 0.1), (1.5708, 0.0, -1.5708)))
    assert "constant" in text and "0.1000" in text and "1.5708" in text and "-1.5708" in text
    assert "unknown" in format_readout(None)


def test_seed_due_only_when_the_values_or_the_controller_changed():
    a = VtcpConfigView("constant", (0.0, 0.0, 0.1), (0.0, 0.0, 0.0))
    b = VtcpConfigView("constant", (0.0, 0.0, 0.2), (0.0, 0.0, 0.0))
    assert seed_due(None, TASK, a)
    assert not seed_due((TASK, a), TASK, a), "a repeat read must keep an unapplied edit"
    assert seed_due((TASK, a), TASK, b)
    assert seed_due((TASK, a), JOINT, a)


def test_apply_gate():
    assert apply_enabled(TASK, JOINT, VTCP_OWNERS) == (True, HINT_INACTIVE)
    assert apply_enabled(TASK, TASK, VTCP_OWNERS) == (False, HINT_ACTIVE)
    assert apply_enabled(WBC, TASK, VTCP_OWNERS) == (False, HINT_NOT_OWNER)


def test_apply_gate_fails_closed_before_the_active_controller_is_known():
    """No active_controller_name message yet: the selected controller may be the
    running one, so Apply is not offered."""
    for ctrl in VTCP_OWNERS:
        assert apply_enabled(ctrl, "", VTCP_OWNERS) == (False, HINT_ACTIVE_UNKNOWN)


# ── app.py wiring ────────────────────────────────────────────────────────────


def _gui():
    from integrated_bringup.demo_gui.app import DemoControllerGUI

    return DemoControllerGUI


def test_the_apply_gate_is_repainted_on_all_three_events():
    """Selection change, catalog poll, active-controller change. The last one is
    the easy one to miss: _sync_selected_to_active_controller returns early when
    the newly active controller is the one already selected (select, then
    Switch), so relying on it would leave Apply open on the running controller."""
    gui = _gui()
    for method in (
        gui._on_ctrl_radio_change,
        gui._refresh_controller_widgets,
        gui._on_active_controller,
    ):
        assert "self._apply_vtcp_state()" in inspect.getsource(method), method.__name__


def test_the_panel_state_is_not_reset_from_the_executor_thread():
    """_reset_controller_sourced_state runs on the rclpy executor thread and is
    for topic-sourced state; the panel's state is parameter-sourced and Tk-owned."""
    assert "vtcp" not in inspect.getsource(_gui()._reset_controller_sourced_state)


class _FakeVar:
    def __init__(self, value=""):
        self.value = value

    def set(self, v):
        self.value = v

    def get(self):
        return self.value


class _FakeWidget:
    def __init__(self):
        self.kwargs = {"state": "normal"}

    def config(self, **kwargs):
        self.kwargs.update(kwargs)

    def cget(self, key):
        return self.kwargs[key]


class _FakeEntry(_FakeWidget):
    def __init__(self, text=""):
        super().__init__()
        self.text = text

    def get(self):
        return self.text

    def delete(self, *_):
        assert self.kwargs["state"] == "normal", "write into a disabled entry is dropped by Tk"
        self.text = ""

    def insert(self, _index, text):
        self.text = text


class _FakeCombo(_FakeEntry):
    def set(self, v):
        assert self.kwargs["state"] != "disabled", "write into a disabled combobox"
        self.text = v


class _FakeRoot:
    def after(self, _delay, fn, *args):
        fn(*args)


class _FakeLogger:
    def __init__(self):
        self.lines = []

    def __getattr__(self, level):
        return lambda msg: self.lines.append((level, msg))


def _state(selected: str, active: str, *, successful=True, reason="", inputs=None):
    """Duck-typed GUI with the real panel methods bound to it. Only the
    collaborators outside the panel are faked: the parameter client, the
    logger and the Tk root."""
    gui = _gui()
    state = types.SimpleNamespace()
    for name in (
        "_apply_vtcp_state",
        "_paint_vtcp",
        "_fill_vtcp_inputs",
        "_vtcp_input_widgets",
        "_request_load_vtcp",
        "_send_vtcp",
        "_after_vtcp_set",
        "_set_vtcp_result",
    ):
        setattr(state, name, types.MethodType(getattr(gui, name), state))
    state.selected_ctrl = _FakeVar(selected)
    state._active_ctrl = active
    state._vtcp_mode_combo = _FakeCombo()
    state._vtcp_offset_entries = [_FakeEntry() for _ in range(3)]
    state._vtcp_rpy_entries = [_FakeEntry() for _ in range(3)]
    state._vtcp_apply_btn = _FakeWidget()
    state._vtcp_load_btn = _FakeWidget()
    state._vtcp_readout_var = _FakeVar()
    state._vtcp_hint_var = _FakeVar()
    state._vtcp_hint_label = _FakeWidget()
    state._vtcp_result_var = _FakeVar()
    state._vtcp_result_label = _FakeWidget()
    state._vtcp_result_ctrl = ""
    state._vtcp_configs = {}
    state._vtcp_seeded = None
    state.root = _FakeRoot()
    state._logger = _FakeLogger()
    state.get_logger = lambda: state._logger
    if inputs is not None:
        mode, offset, rpy = inputs
        state._vtcp_mode_combo.text = mode
        for ent, t in zip(state._vtcp_offset_entries, offset, strict=True):
            ent.text = t
        for ent, t in zip(state._vtcp_rpy_entries, rpy, strict=True):
            ent.text = t

    state.sent = []
    state.refreshes = []

    def _fake_refresh(c, **kwargs):
        state.refreshes.append((c, dict(state._vtcp_configs), kwargs))

    state._refresh_vtcp = _fake_refresh

    class _Future:
        def __init__(self, resp):
            self._resp = resp

        def result(self):
            return self._resp

        def add_done_callback(self, cb):
            cb(self)

    class _Client:
        def wait_for_services(self, timeout_sec=0.0):
            return True

        def set_parameters_atomically(self, params):
            state.sent.append(("atomically", params))
            resp = types.SimpleNamespace(
                result=types.SimpleNamespace(successful=successful, reason=reason)
            )
            return _Future(resp)

        def set_parameters(self, params):
            state.sent.append(("separately", params))
            raise AssertionError("three separate sets are three callbacks and three Stores")

    state._get_param_client = lambda c: _Client()
    return state


_GOOD = ("constant", ["0.0", "0.0", "0.15"], ["0.0", "0.0", "0.1"])


@pytest.mark.parametrize("ctrl", sorted(VTCP_OWNERS))
def test_apply_sends_all_three_in_one_atomic_set(ctrl):
    from rclpy.parameter import Parameter

    other = next(c for c in VTCP_OWNERS if c != ctrl)
    state = _state(ctrl, other, inputs=_GOOD)
    state._send_vtcp()

    assert len(state.sent) == 1
    how, params = state.sent[0]
    assert how == "atomically"
    by_name = {p.name: p for p in params}
    assert list(by_name) == list(VTCP_PARAMS)
    assert by_name[VTCP_MODE_PARAM].type_ == Parameter.Type.STRING
    assert by_name[VTCP_MODE_PARAM].value == "constant"
    assert by_name[VTCP_OFFSET_PARAM].type_ == Parameter.Type.DOUBLE_ARRAY
    assert list(by_name[VTCP_OFFSET_PARAM].value) == [0.0, 0.0, 0.15]
    assert by_name[VTCP_ORIENTATION_PARAM].type_ == Parameter.Type.DOUBLE_ARRAY
    assert list(by_name[VTCP_ORIENTATION_PARAM].value) == [0.0, 0.0, 0.1]


@pytest.mark.parametrize("active", [TASK, ""])
def test_apply_sends_nothing_while_the_gate_is_closed(active):
    state = _state(TASK, active, inputs=_GOOD)
    state._send_vtcp()
    assert state.sent == []


def test_apply_sends_nothing_for_a_typo():
    state = _state(TASK, JOINT, inputs=("constant", ["0", "x", "0"], ["0", "0", "0"]))
    state._send_vtcp()
    assert state.sent == []
    assert "offset" in state._vtcp_result_var.get()


def test_a_refusal_is_shown_verbatim_and_the_gate_repainted():
    """The reason is the controller's own sentence; the GUI adds the prefix and
    nothing else. Any string must survive, so the test does not reuse the C++
    constant — the GUI is not allowed to depend on its wording."""
    reason = "an arbitrary reason the GUI has never seen: 42"
    state = _state(TASK, JOINT, successful=False, reason=reason, inputs=_GOOD)
    state._vtcp_configs[TASK] = VtcpConfigView("disabled", (0.0,) * 3, (0.0,) * 3)
    state._send_vtcp()

    assert state._vtcp_result_var.get() == f"refused: {reason}"
    assert TASK not in state._vtcp_configs
    assert state.refreshes == [(TASK, {}, {"paint_unknown": False})], (
        "the cache must be dropped BEFORE the re-read, or the re-read hits it"
    )
    assert state._vtcp_hint_var.get() == HINT_INACTIVE
    assert state._vtcp_result_var.get() == f"refused: {reason}", "the gate repaint wiped it"


def test_a_success_also_invalidates_and_rereads():
    state = _state(TASK, JOINT, inputs=_GOOD)
    state._vtcp_configs[TASK] = VtcpConfigView("disabled", (0.0,) * 3, (0.0,) * 3)
    state._send_vtcp()

    assert "applied" in state._vtcp_result_var.get()
    assert state.refreshes == [(TASK, {}, {"paint_unknown": False})]


def test_the_gate_closes_every_input_on_the_active_controller():
    state = _state(TASK, TASK)
    state._apply_vtcp_state()
    assert state._vtcp_apply_btn.kwargs["state"] == "disabled"
    assert state._vtcp_mode_combo.kwargs["state"] == "disabled"
    assert all(
        e.kwargs["state"] == "disabled"
        for e in [*state._vtcp_offset_entries, *state._vtcp_rpy_entries]
    )
    assert state._vtcp_load_btn.kwargs["state"] == "normal", "reading is fine while active"
    assert state._vtcp_hint_var.get() == HINT_ACTIVE


def test_a_non_owner_gets_a_dead_panel():
    state = _state(WBC, TASK)
    state._apply_vtcp_state()
    assert state._vtcp_apply_btn.kwargs["state"] == "disabled"
    assert state._vtcp_load_btn.kwargs["state"] == "disabled"
    assert state._vtcp_hint_var.get() == HINT_NOT_OWNER


def test_an_outcome_does_not_follow_the_selection_to_another_controller():
    state = _state(TASK, JOINT, successful=False, reason="r", inputs=_GOOD)
    state._send_vtcp()
    assert state._vtcp_result_var.get() == "refused: r"
    state.selected_ctrl.set(COMPLIANCE)
    state._apply_vtcp_state()
    assert state._vtcp_result_var.get() == ""


def test_a_repeat_read_keeps_an_unapplied_edit_even_on_a_closed_panel():
    view = VtcpConfigView("constant", (0.0, 0.0, 0.1), (0.0, 0.0, 0.0))
    state = _state(TASK, TASK)
    state._apply_vtcp_state()  # active: every input disabled
    state._paint_vtcp(TASK, view)
    assert state._vtcp_offset_entries[2].get() == "0.1000", "seeding a disabled panel failed"
    assert state._vtcp_offset_entries[2].kwargs["state"] == "disabled", "seeding reopened it"

    state._vtcp_offset_entries[2].text = "0.2"  # an edit not applied
    state._paint_vtcp(TASK, view)  # catalog-poll repaint, same values
    assert state._vtcp_offset_entries[2].get() == "0.2"


def test_load_overwrites_an_edit_with_a_fresh_read():
    view = VtcpConfigView("constant", (0.0, 0.0, 0.1), (0.0, 0.0, 0.0))
    state = _state(TASK, JOINT)
    state._paint_vtcp(TASK, view)
    state._vtcp_configs[TASK] = view
    state._vtcp_offset_entries[2].text = "0.2"

    state._request_load_vtcp()
    assert state.refreshes == [(TASK, {}, {"paint_unknown": False})], "Load must bypass the cache"
    state._paint_vtcp(TASK, view)  # the read lands
    assert state._vtcp_offset_entries[2].get() == "0.1000"


def test_a_late_read_for_a_deselected_controller_is_dropped():
    state = _state(JOINT, TASK)
    state._paint_vtcp(TASK, VtcpConfigView("constant", (0.0, 0.0, 0.1), (0.0, 0.0, 0.0)))
    assert state._vtcp_readout_var.get() == ""
    assert state._vtcp_seeded is None
