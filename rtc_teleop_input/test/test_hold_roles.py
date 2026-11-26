"""Hold buttons beyond reset: switch, clear E-STOP, trigger — and the rules they share.

Safety rules under test (model.ButtonActions):
- one call per continuous hold, none for a short hold;
- the deadman, any E-STOP press, and a second held hold button start every
  hold again;
- the clear hold also needs the E-STOP button released throughout, so a clear
  held while the operator presses E-STOP cannot lower that E-STOP.
"""

from __future__ import annotations

from rtc_teleop_input.layout import load_layout
from rtc_teleop_input.model import (
    ButtonActions,
    HelpCombo,
    TeleopConfig,
    VirtualPad,
    help_entries,
    hold_entries,
    released_frame,
    validate_config,
)

LAYOUT = load_layout()
DT = 0.02

ROLES = {
    "switch_buttons": ("y:ctrl_a", "a:ctrl_b"),
    "clear_estop_button": "start",
    "clear_estop_reason": "operator_reason",
    "trigger_buttons": ("x:/some/service",),
    "key_buttons": (
        "leftshoulder:Space",
        "b:Escape",
        "back:Backspace",
        "y:1",
        "a:2",
        "start:Return",
        "x:R",
    ),
}


class Rig:
    def __init__(self, **overrides) -> None:
        params = {**ROLES, "reset_controller_name": "ctrl", **overrides}
        config = TeleopConfig(output_topic="out", **params)
        self.pad = VirtualPad(LAYOUT, validate_config(config, LAYOUT), config.deflection)
        self.actions = ButtonActions(LAYOUT, config)
        self.now = 0.0
        self.sent: list[str] = []
        self.estops = 0

    def key(self, key, pressed):
        self.pad.key_event(key, pressed, False)

    def tick(self, seconds=DT):
        for _ in range(round(seconds / DT)):
            self.now += DT
            actions = self.actions.step(self.pad.frame(), self.now)
            self.estops += actions.estop_calls
            self.sent += [h.key for h in actions.fired]


def test_entries_and_keys():
    config = TeleopConfig(output_topic="out", reset_controller_name="ctrl", **ROLES)
    assert [h.key for h in hold_entries(config)] == [
        "reset",
        "switch:ctrl_a",
        "switch:ctrl_b",
        "clear",
        "trigger:/some/service",
    ]


def test_each_hold_sends_once_after_hold_s():
    for key, expected in (
        ("1", "switch:ctrl_a"),
        ("2", "switch:ctrl_b"),
        ("Return", "clear"),
        ("R", "trigger:/some/service"),
    ):
        rig = Rig()
        rig.key(key, True)
        rig.tick(0.9)
        assert rig.sent == []
        rig.tick(0.2)
        assert rig.sent == [expected]
        rig.tick(3.0)
        assert rig.sent == [expected]  # no refire while held
        rig.key(key, False)
        rig.tick()
        rig.key(key, True)
        rig.tick(1.1)
        assert rig.sent == [expected, expected]


def test_hold_s_is_its_own_duration():
    rig = Rig(hold_s=0.5, reset_hold_s=2.0)
    rig.key("1", True)
    rig.tick(0.6)
    assert rig.sent == ["switch:ctrl_a"]
    rig.key("1", False)
    rig.key("Backspace", True)
    rig.tick(1.5)
    assert rig.sent == ["switch:ctrl_a"]
    rig.tick(0.6)
    assert rig.sent == ["switch:ctrl_a", "reset"]


def test_the_deadman_restarts_every_hold():
    rig = Rig()
    rig.key("1", True)
    rig.tick(0.6)
    rig.key("Space", True)
    rig.key("Space", False)  # a tap inside one tick
    rig.tick(0.9)
    assert rig.sent == []
    rig.tick(0.2)
    assert rig.sent == ["switch:ctrl_a"]


def test_an_estop_during_a_clear_hold_is_not_cleared():
    rig = Rig()
    rig.key("Return", True)
    rig.tick(0.5)
    rig.key("Escape", True)  # E-STOP pressed and kept down
    rig.tick(3.0)
    assert rig.estops == 1
    assert rig.sent == []
    # released: the clear hold starts over from here
    rig.key("Escape", False)
    rig.tick(0.9)
    assert rig.sent == []
    rig.tick(0.2)
    assert rig.sent == ["clear"]


def test_an_estop_tap_restarts_the_clear_hold():
    rig = Rig()
    rig.key("Return", True)
    rig.tick(0.9)
    rig.key("Escape", True)
    rig.key("Escape", False)  # inside one tick
    rig.tick(0.9)  # 1.8 s since the press, 0.9 s since the tap
    assert rig.estops == 1
    assert rig.sent == []
    rig.tick(0.2)
    assert rig.sent == ["clear"]


def test_the_window_estop_restarts_every_hold():
    rig = Rig()
    rig.key("Return", True)
    rig.tick(0.9)
    rig.actions.request_estop()
    rig.tick(0.9)
    assert rig.estops == 1
    assert rig.sent == []
    rig.tick(0.2)
    assert rig.sent == ["clear"]


def test_an_estop_press_restarts_a_switch_hold():
    rig = Rig()
    rig.key("1", True)
    rig.tick(0.9)
    rig.key("Escape", True)
    rig.key("Escape", False)
    rig.tick(0.9)
    assert rig.sent == []
    rig.tick(0.2)
    assert rig.sent == ["switch:ctrl_a"]


def test_two_switch_buttons_together_send_nothing():
    rig = Rig()
    rig.key("1", True)
    rig.key("2", True)
    rig.tick(3.0)
    assert rig.sent == []
    # one let go: the other starts its hold now
    rig.key("2", False)
    rig.tick(0.9)
    assert rig.sent == []
    rig.tick(0.2)
    assert rig.sent == ["switch:ctrl_a"]


def test_a_second_hold_button_tapped_restarts():
    rig = Rig()
    rig.key("1", True)
    rig.tick(0.9)
    rig.key("Backspace", True)
    rig.key("Backspace", False)  # inside one tick
    rig.tick(0.9)
    assert rig.sent == []
    rig.tick(0.2)
    assert rig.sent == ["switch:ctrl_a"]


def test_a_disabled_reset_still_counts_as_a_held_hold_button():
    rig = Rig(reset_controller_name="")
    rig.key("Backspace", True)
    rig.key("1", True)
    rig.tick(3.0)
    assert rig.sent == []


def test_progress_per_entry():
    rig = Rig()
    assert all(rig.actions.progress(i, rig.now) is None for i in range(5))
    rig.key("Return", True)
    rig.tick(0.5)
    clear = [h.kind for h in rig.actions.holds].index("clear")
    assert 0.4 < rig.actions.progress(clear, rig.now) < 0.5
    assert rig.actions.reset_progress(rig.now) is None


def test_help_combo_toggles_once_per_press():
    combo = HelpCombo(LAYOUT, ("triggerleft", "triggerright"))
    lt, rt = LAYOUT.axis_index("triggerleft"), LAYOUT.axis_index("triggerright")
    base = released_frame(LAYOUT, "pad")

    def frame(left, right):
        axes = list(base.axes)
        axes[lt], axes[rt] = left, right
        return type(base)(
            source="pad", axes=tuple(axes), buttons=base.buttons, presses=base.presses
        )

    assert not combo.step(frame(-1.0, 0.0))
    assert combo.step(frame(-1.0, -0.6))
    for _ in range(10):
        assert not combo.step(frame(-1.0, -1.0))  # held: no second toggle
    assert not combo.step(frame(-1.0, -0.4))  # one let go past -0.5
    assert combo.step(frame(-0.5, -0.5))  # -0.5 counts as pressed
    assert not HelpCombo(LAYOUT, ()).step(frame(-1.0, -1.0))


def test_help_entries_cover_every_role():
    config = TeleopConfig(
        output_topic="out",
        reset_controller_name="ctrl",
        help_combo=("triggerleft", "triggerright"),
        help_labels=("lefty:forward / back", "rightx:turn"),
        **ROLES,
    )
    rows = help_entries(config)
    assert [(r.controls, r.keys) for r in rows] == [
        (("lefty",), "W (+) / S (-)"),
        (("rightx",), "Left (+) / Right (-)"),
        (("leftshoulder",), "Space"),
        (("b",), "Escape"),
        (("back",), "Backspace"),
        (("y",), "1"),
        (("a",), "2"),
        (("start",), "Return"),
        (("x",), "R"),
        (("triggerleft", "triggerright"), "F1"),
    ]
    functions = [r.function for r in rows]
    assert functions[0] == "forward / back"
    assert functions[5] == "switch to ctrl_a — hold 1 s"
    assert "operator_reason only" in functions[7]
    assert functions[8] == "call /some/service — hold 1 s"


def test_help_entries_skip_a_disabled_reset_and_unlabelled_axes():
    rows = help_entries(TeleopConfig(output_topic="out"))
    assert [r.controls for r in rows] == [("leftshoulder",), ("b",), ()]
    assert rows[-1].keys == "F1"
