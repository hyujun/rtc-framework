"""ButtonActions: one E-STOP per press, one reset per continuous hold, deadman released."""

from __future__ import annotations

from rtc_teleop_input.layout import load_layout
from rtc_teleop_input.model import ButtonActions, TeleopConfig, VirtualPad, validate_config

LAYOUT = load_layout()
DT = 0.02  # 50 Hz ticks


class Rig:
    """A virtual pad, the button rules and a fake clock ticking at 50 Hz."""

    def __init__(self, **overrides) -> None:
        overrides.setdefault("reset_controller_name", "ctrl")
        config = TeleopConfig(output_topic="out", **overrides)
        self.pad = VirtualPad(LAYOUT, validate_config(config, LAYOUT), config.deflection)
        self.actions = ButtonActions(LAYOUT, config)
        self.now = 0.0
        self.estops = 0
        self.resets = 0

    def key(self, key, pressed):
        self.pad.key_event(key, pressed, False)

    def tick(self, seconds=DT):
        # round, not floor: seconds / DT is not exact in binary
        for _ in range(round(seconds / DT)):
            self.now += DT
            actions = self.actions.step(self.pad.frame(), self.now)
            self.estops += actions.estop_calls
            self.resets += int(actions.reset_call)


def test_estop_once_per_press_even_when_held():
    rig = Rig()
    rig.key("Escape", True)
    rig.tick(2.0)
    assert rig.estops == 1
    rig.key("Escape", False)
    rig.tick()
    for _ in range(3):
        rig.key("Escape", True)
        rig.tick(0.1)
        rig.key("Escape", False)
        rig.tick(0.1)
    assert rig.estops == 4


def test_estop_tap_inside_one_tick_counts():
    rig = Rig()
    rig.key("Escape", True)
    rig.key("Escape", False)
    rig.tick()
    assert rig.estops == 1


def test_estop_ignores_the_deadman():
    rig = Rig()
    rig.key("Space", True)
    rig.key("Escape", True)
    rig.tick()
    assert rig.estops == 1


def test_window_button_estop():
    rig = Rig()
    rig.actions.request_estop()
    rig.tick()
    rig.tick()
    assert rig.estops == 1


def test_reset_after_a_full_hold_once():
    rig = Rig()
    rig.key("Backspace", True)
    rig.tick(0.9)
    assert rig.resets == 0
    rig.tick(0.2)
    assert rig.resets == 1
    rig.tick(3.0)
    assert rig.resets == 1  # no refire while held
    rig.key("Backspace", False)
    rig.tick()
    rig.key("Backspace", True)
    rig.tick(1.1)
    assert rig.resets == 2


def test_reset_short_hold_does_nothing():
    rig = Rig()
    rig.key("Backspace", True)
    rig.tick(0.9)
    rig.key("Backspace", False)
    rig.tick(1.0)
    assert rig.resets == 0


def test_reset_needs_the_deadman_released_throughout():
    rig = Rig()
    rig.key("Space", True)
    rig.key("Backspace", True)
    rig.tick(2.0)
    assert rig.resets == 0
    # released: the hold starts now
    rig.key("Space", False)
    rig.tick(0.9)
    assert rig.resets == 0
    rig.tick(0.2)
    assert rig.resets == 1


def test_deadman_tap_restarts_the_hold():
    rig = Rig()
    rig.key("Backspace", True)
    rig.tick(0.6)
    rig.key("Space", True)  # a tap inside one tick
    rig.key("Space", False)
    rig.tick(0.9)  # 1.5 s since the press, 0.9 s since the tap
    assert rig.resets == 0
    rig.tick(0.2)
    assert rig.resets == 1


def test_release_and_press_inside_one_tick_restarts_the_hold():
    rig = Rig()
    rig.key("Backspace", True)
    rig.tick(0.6)
    rig.key("Backspace", False)
    rig.key("Backspace", True)
    rig.tick(0.9)
    assert rig.resets == 0
    rig.tick(0.2)
    assert rig.resets == 1


def test_reset_disabled_without_a_controller_name():
    rig = Rig(reset_controller_name="")
    rig.key("Backspace", True)
    rig.tick(3.0)
    assert rig.resets == 0
    assert rig.actions.reset_progress(rig.now) is None


def test_reset_progress():
    rig = Rig()
    assert rig.actions.reset_progress(rig.now) is None
    rig.key("Backspace", True)
    rig.tick(0.5)
    progress = rig.actions.reset_progress(rig.now)
    assert 0.4 < progress < 0.5
