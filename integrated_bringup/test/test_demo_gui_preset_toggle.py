"""Headless tests for demo_controller_gui preset-hotkey state and settings."""

import json
import threading
from types import SimpleNamespace

import pytest

from integrated_bringup.demo_gui.app import DemoControllerGUI
from integrated_bringup.demo_gui.preset_toggle import (
    PresetToggleState,
    format_keysym,
    is_text_input_widget,
    load_toggle_settings,
    normalize_keysym,
    normalize_toggle_settings,
    resolve_preset_pair,
    save_toggle_settings,
    toggle_mapping_warning,
)


def test_keysym_normalization_and_formatting():
    assert normalize_keysym(" ") == "space"
    assert normalize_keysym("Space") == "space"
    assert normalize_keysym("Y") == "y"
    assert normalize_keysym("F8") == "F8"
    assert normalize_keysym("Shift_L") is None
    assert normalize_keysym("") is None
    assert format_keysym("space") == "Space"
    assert format_keysym("y") == "Y"


def test_text_input_widgets_are_suppressed():
    for widget_class in ("Entry", "TEntry", "Text", "TCombobox", "Spinbox", "TSpinbox"):
        assert is_text_input_widget(widget_class)
    assert not is_text_input_widget("TButton")
    assert not is_text_input_widget("Treeview")


def test_toggle_is_close_first_and_commits_only_on_success():
    state = PresetToggleState()

    assert state.press() == "close"
    assert state.press() is None
    state.rearm()
    assert state.press() == "close"

    state.commit("close")
    state.rearm()
    assert state.press() == "open"
    state.commit("open")
    state.rearm()
    assert state.press() == "close"


def test_preset_pair_keeps_valid_choices_and_falls_back_by_type():
    presets = {
        "open_flat": {"type": "open"},
        "power_grasp": {"type": "close"},
        "pinch_grasp": {"type": "close"},
    }
    assert resolve_preset_pair(presets) == ("power_grasp", "open_flat")
    assert resolve_preset_pair(presets, "pinch_grasp", "open_flat") == (
        "pinch_grasp",
        "open_flat",
    )
    assert resolve_preset_pair(presets, "deleted", "deleted") == (
        "power_grasp",
        "open_flat",
    )


def test_settings_round_trip_and_malformed_fallback(tmp_path):
    path = tmp_path / "demo_gui_settings_p1b.json"
    settings = {
        "preset_toggle": {
            "keysym": "Y",
            "close_preset": "posture_close",
            "open_preset": "posture_open",
        }
    }

    save_toggle_settings(str(path), settings)
    assert load_toggle_settings(str(path)) == {
        "preset_toggle": {
            "keysym": "y",
            "close_preset": "posture_close",
            "open_preset": "posture_open",
        }
    }

    path.write_text("{broken", encoding="utf-8")
    assert load_toggle_settings(str(path))["preset_toggle"]["keysym"] == "space"


def test_invalid_settings_keep_safe_defaults():
    assert normalize_toggle_settings(None)["preset_toggle"]["keysym"] == "space"
    assert (
        normalize_toggle_settings({"preset_toggle": {"keysym": "Control_L"}})["preset_toggle"][
            "keysym"
        ]
        == "space"
    )
    assert (
        normalize_toggle_settings({"preset_toggle": {"close_preset": 3}})["preset_toggle"][
            "close_preset"
        ]
        == ""
    )


def test_atomic_save_leaves_valid_json(tmp_path):
    path = tmp_path / "settings.json"
    save_toggle_settings(str(path), {"preset_toggle": {"keysym": "space"}})
    with path.open() as settings_file:
        assert json.load(settings_file)["preset_toggle"]["keysym"] == "space"
    assert list(tmp_path.glob(".demo_gui_settings_*.json")) == []


class _Widget:
    def __init__(self, widget_class):
        self._widget_class = widget_class

    def winfo_class(self):
        return self._widget_class


class _Root:
    def after_cancel(self, _token):
        pass


class _HotkeyHarness:
    def __init__(self):
        self._toggle_capture_window = None
        self._toggle_rearm_after = None
        self._preset_toggle = PresetToggleState()
        self._toggle_in_flight = False
        self.root = _Root()
        self.requests = []

    def _toggle_event_matches(self, event):
        return normalize_keysym(event.keysym) == "space"

    def _request_toggle_preset(self, role):
        self.requests.append(role)


def test_hotkey_ignores_text_input_and_suppresses_repeat():
    gui = _HotkeyHarness()
    text_event = SimpleNamespace(keysym="space", widget=_Widget("TEntry"))
    button_event = SimpleNamespace(keysym="space", widget=_Widget("TButton"))

    assert DemoControllerGUI._on_toggle_key_press(gui, text_event) is None
    assert gui.requests == []
    assert DemoControllerGUI._on_toggle_key_press(gui, button_event) == "break"
    assert gui.requests == ["close"]
    assert DemoControllerGUI._on_toggle_key_press(gui, button_event) == "break"
    assert gui.requests == ["close"]


def test_hotkey_does_not_dispatch_while_previous_toggle_is_in_flight():
    gui = _HotkeyHarness()
    event = SimpleNamespace(keysym="space", widget=_Widget("TButton"))

    DemoControllerGUI._on_toggle_key_press(gui, event)
    gui._preset_toggle.rearm()
    DemoControllerGUI._on_toggle_key_press(gui, event)
    assert gui.requests == ["close"]


def test_toggle_completion_advances_only_on_success():
    gui = SimpleNamespace(_preset_toggle=PresetToggleState(), _toggle_in_flight=True)
    complete = DemoControllerGUI._toggle_complete(gui, "close")
    complete(False)
    assert gui._preset_toggle.next_role == "close"
    assert not gui._toggle_in_flight
    gui._toggle_in_flight = True
    complete(True)
    assert gui._preset_toggle.next_role == "open"
    assert not gui._toggle_in_flight


def test_estop_blocks_toggle_before_tree_or_publish_access():
    warnings = []
    gui = SimpleNamespace(
        estop_active=True,
        _toggle_in_flight=True,
        _preset_toggle=PresetToggleState(),
        get_logger=lambda: SimpleNamespace(warn=warnings.append),
    )
    gui._toggle_complete = lambda role: DemoControllerGUI._toggle_complete(gui, role)

    DemoControllerGUI._request_toggle_preset(gui, "close")
    assert warnings == ["Preset toggle withheld — E-STOP is active"]
    assert gui._preset_toggle.next_role == "close"
    assert not gui._toggle_in_flight


def test_active_controller_resynchronizes_radio_selection():
    selected = SimpleNamespace(value="demo_task_controller")
    selected.get = lambda: selected.value
    selected.set = lambda value: setattr(selected, "value", value)
    statuses = []
    gui = SimpleNamespace(
        selected_ctrl=selected,
        _switchable_keys=("demo_joint_controller", "demo_task_controller"),
        _on_ctrl_radio_change=lambda: None,
        _ctrl_status=SimpleNamespace(set=statuses.append),
        _catalog=SimpleNamespace(display_label=lambda key: key),
        get_logger=lambda: SimpleNamespace(warn=lambda _message: None),
    )

    DemoControllerGUI._sync_selected_to_active_controller(gui, "demo_joint_controller")
    assert selected.value == "demo_joint_controller"
    assert statuses == ["Active: demo_joint_controller"]


def test_same_preset_for_both_roles_is_flagged():
    assert toggle_mapping_warning("power_grasp", "power_grasp") is not None
    assert toggle_mapping_warning("power_grasp", "open_flat") is None
    # Unconfigured is not "the same preset".
    assert toggle_mapping_warning("", "") is None


class _Publisher:
    def __init__(self, subscribers=1):
        self.subscribers = subscribers

    def get_subscription_count(self):
        return self.subscribers


def _bind_toggle_topics(gui, ctrl_name, subscribers=1):
    """State a completed rewire to ``ctrl_name`` leaves behind."""
    gui._active_ctrl = ctrl_name
    gui._owned_topics_lock = threading.Lock()
    gui._owned_topics_ns = "/" + ctrl_name
    gui.robot_cmd_pub = _Publisher(subscribers)
    gui.hand_cmd_pub = _Publisher(subscribers)
    gui._toggle_publishers_matched = lambda ctrl, needs_robot: (
        DemoControllerGUI._toggle_publishers_matched(gui, ctrl, needs_robot)
    )


def _raising_toggle_gui(errors):
    def boom(**_kwargs):
        raise RuntimeError("parameter service exploded")

    gui = SimpleNamespace(
        estop_active=False,
        _toggle_in_flight=True,
        _preset_toggle=PresetToggleState(),
        _toggle_settings={"preset_toggle": {"close_preset": "power_grasp"}},
        _presets={"power_grasp": {"type": "close"}},
        _preset_tree=SimpleNamespace(selection_set=lambda _n: None, see=lambda _n: None),
        _active_ctrl="",
        _send_preset=boom,
        get_logger=lambda: SimpleNamespace(error=errors.append, warn=errors.append),
    )
    gui._toggle_complete = lambda role: DemoControllerGUI._toggle_complete(gui, role)
    return gui


def test_toggle_releases_hotkey_when_dispatch_raises():
    errors = []
    gui = _raising_toggle_gui(errors)

    with pytest.raises(RuntimeError):
        DemoControllerGUI._request_toggle_preset(gui, "close")
    assert not gui._toggle_in_flight
    # A failed dispatch must not advance close -> open.
    assert gui._preset_toggle.next_role == "close"
    assert any("aborted" in message for message in errors)


def test_toggle_releases_hotkey_when_deferred_stage_raises():
    errors = []
    gui = _raising_toggle_gui(errors)
    _bind_toggle_topics(gui, "demo_joint_controller")

    with pytest.raises(RuntimeError):
        DemoControllerGUI._wait_for_toggle_controller(
            gui, "close", "power_grasp", "demo_joint_controller", float("inf")
        )
    assert not gui._toggle_in_flight


def test_hotkey_owns_space_on_focused_buttons():
    # ttk buttons take focus on click (ttk::clickToFocus), so handing Space to
    # the focused widget would re-press whatever was clicked last — E-STOP or
    # Switch included — and silently disable the hotkey after any click.
    for widget_class in ("TButton", "TCheckbutton", "TRadiobutton", "Treeview"):
        gui = _HotkeyHarness()
        event = SimpleNamespace(keysym="space", widget=_Widget(widget_class))
        assert DemoControllerGUI._on_toggle_key_press(gui, event) == "break", widget_class
        assert gui.requests == ["close"], widget_class


def _sending_toggle_gui(ctrl_name, preset):
    sent = []
    scheduled = []
    gui = SimpleNamespace(
        estop_active=False,
        _toggle_in_flight=True,
        _preset_toggle=PresetToggleState(),
        _toggle_settings={"preset_toggle": {"close_preset": "power_grasp"}},
        _presets={"power_grasp": preset},
        _preset_tree=SimpleNamespace(selection_set=lambda _n: None, see=lambda _n: None),
        selected_ctrl=SimpleNamespace(get=lambda: ctrl_name),
        _send_preset=lambda **kwargs: (
            sent.append(kwargs["preset_name"]),
            kwargs["on_complete"](True),
        ),
        root=SimpleNamespace(after=lambda _ms, callback: scheduled.append(callback)),
        get_logger=lambda: SimpleNamespace(error=lambda _m: None, warn=lambda _m: None),
    )
    _bind_toggle_topics(gui, ctrl_name)
    gui._toggle_complete = lambda role: DemoControllerGUI._toggle_complete(gui, role)
    gui._task_space_ready = lambda _ctrl: True
    gui._wait_for_toggle_controller = lambda *args: DemoControllerGUI._wait_for_toggle_controller(
        gui, *args
    )
    return gui, sent, scheduled


def _run_scheduled(scheduled):
    pending = list(scheduled)
    scheduled.clear()
    for callback in pending:
        callback()


def test_toggle_waits_until_publishers_are_rebound_to_the_controller():
    # _active_ctrl already names the new controller but the executor has not
    # recreated the publishers yet: sending now would hit the old namespace.
    gui, sent, scheduled = _sending_toggle_gui("demo_joint_controller", {"type": "close"})
    gui._owned_topics_ns = "/demo_task_controller"

    DemoControllerGUI._request_toggle_preset(gui, "close")
    assert sent == []
    assert gui._toggle_in_flight

    gui._owned_topics_ns = "/demo_joint_controller"
    _run_scheduled(scheduled)
    assert sent == ["power_grasp"]
    assert gui._preset_toggle.next_role == "open"


def test_toggle_waits_for_a_matched_subscriber():
    # Depth-1 volatile publishers drop a goal published before discovery.
    preset = {"type": "close", "robot_target": [0.0] * 6}
    gui, sent, scheduled = _sending_toggle_gui("demo_joint_controller", preset)
    gui.hand_cmd_pub.subscribers = 0

    DemoControllerGUI._request_toggle_preset(gui, "close")
    assert sent == []

    gui.hand_cmd_pub.subscribers = 1
    gui.robot_cmd_pub.subscribers = 0
    _run_scheduled(scheduled)
    assert sent == [], "a robot target also needs the arm goal subscriber"

    gui.robot_cmd_pub.subscribers = 1
    _run_scheduled(scheduled)
    assert sent == ["power_grasp"]


def test_toggle_times_out_without_advancing():
    gui, sent, scheduled = _sending_toggle_gui("demo_joint_controller", {"type": "close"})
    gui.hand_cmd_pub.subscribers = 0

    DemoControllerGUI._wait_for_toggle_controller(
        gui, "close", "power_grasp", "demo_joint_controller", 0.0
    )
    assert sent == []
    assert scheduled == []
    assert not gui._toggle_in_flight
    assert gui._preset_toggle.next_role == "close"
