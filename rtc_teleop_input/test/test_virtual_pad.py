"""VirtualPad: keys, mouse, auto-repeat and focus loss -> published pad state."""

from __future__ import annotations

import pytest

from rtc_teleop_input.layout import load_layout
from rtc_teleop_input.model import TeleopConfig, VirtualPad, validate_config

LAYOUT = load_layout()
AXIS = {name: i for i, name in enumerate(LAYOUT.axes)}
BUTTON = {name: i for i, name in enumerate(LAYOUT.buttons)}


def make_pad(**overrides) -> VirtualPad:
    config = TeleopConfig(output_topic="out", **overrides)
    return VirtualPad(LAYOUT, validate_config(config, LAYOUT), config.deflection)


def press(pad, key, auto_repeat=False):
    return pad.key_event(key, True, auto_repeat)


def release(pad, key, auto_repeat=False):
    return pad.key_event(key, False, auto_repeat)


def expect(frame, axes=None, buttons=()):
    """Every axis / button is 0 except the ones named."""
    axes = axes or {}
    assert frame.axes == tuple(axes.get(name, 0.0) for name in LAYOUT.axes)
    assert frame.buttons == tuple(1 if name in buttons else 0 for name in LAYOUT.buttons)


# Default bindings. 0.25 is the default deflection: with a consumer scaling
# the axis by a top speed of 2.0 m/s it asks for 0.5 m/s, the walking speed
# that was evaluated (and 0.3 m/s, 0.25 rad/s on the other two axes).
@pytest.mark.parametrize(
    ("key", "axes", "buttons"),
    [
        ("W", {"lefty": 0.25}, ()),
        ("S", {"lefty": -0.25}, ()),
        ("A", {"leftx": 0.25}, ()),
        ("D", {"leftx": -0.25}, ()),
        ("Left", {"rightx": 0.25}, ()),
        ("Right", {"rightx": -0.25}, ()),
        ("Space", {}, ("leftshoulder",)),
        ("Escape", {}, ("b",)),
        ("Backspace", {}, ("back",)),
    ],
)
def test_each_default_key(key, axes, buttons):
    pad = make_pad()
    assert press(pad, key) is True
    expect(pad.frame(), axes, buttons)
    expect(pad.frame(), axes, buttons)  # held: unchanged
    release(pad, key)
    expect(pad.frame())


def test_unbound_key_is_not_consumed():
    pad = make_pad()
    assert press(pad, "Q") is False
    expect(pad.frame())


def test_deflection_parameter_and_opposite_keys():
    pad = make_pad(deflection=0.4)
    press(pad, "W")
    press(pad, "A")
    expect(pad.frame(), {"lefty": 0.4, "leftx": 0.4})
    press(pad, "S")
    press(pad, "D")
    expect(pad.frame())  # W+S and A+D cancel


def test_rebound_keys():
    pad = make_pad(key_axes=("righty+:I",), key_buttons=("rightshoulder:Shift",))
    press(pad, "I")
    press(pad, "Shift")
    assert press(pad, "W") is False
    expect(pad.frame(), {"righty": 0.25}, ("rightshoulder",))


def test_mouse_stick_is_clamped_per_axis_and_flips_x():
    pad = make_pad()
    pad.drag_stick("left", 0.5, 0.75)  # screen right / up
    expect(pad.frame(), {"leftx": -0.5, "lefty": 0.75})
    pad.drag_stick("left", 2.0, -3.0)
    pad.drag_stick("right", -1.5, 1.25)
    expect(pad.frame(), {"leftx": -1.0, "lefty": -1.0, "rightx": 1.0, "righty": 1.0})
    pad.release_stick("left")
    pad.release_stick("right")
    expect(pad.frame())


def test_key_plus_mouse_is_clamped():
    pad = make_pad()
    press(pad, "W")
    pad.drag_stick("left", 0.0, 0.9)
    expect(pad.frame(), {"lefty": 1.0})


def test_mouse_deadman_only_while_held():
    pad = make_pad()
    pad.mouse_control("leftshoulder", True)
    expect(pad.frame(), buttons=("leftshoulder",))
    expect(pad.frame(), buttons=("leftshoulder",))
    pad.mouse_control("leftshoulder", False)
    expect(pad.frame())


def test_mouse_trigger_reads_minus_one():
    pad = make_pad()
    pad.mouse_control("triggerleft", True)
    expect(pad.frame(), {"triggerleft": -1.0})
    pad.mouse_control("triggerleft", False)
    expect(pad.frame())


def test_unknown_stick_side_is_an_error():
    with pytest.raises(ValueError):
        make_pad().drag_stick("middle", 0.0, 0.0)


# ── auto-repeat ─────────────────────────────────────────────────────────


@pytest.mark.parametrize(("key", "button"), [("Space", "leftshoulder"), ("Escape", "b")])
def test_auto_repeat_pairs_change_nothing(key, button):
    pad = make_pad()
    press(pad, key)
    first = pad.frame()
    assert first.presses[BUTTON[button]] == 1
    for _ in range(100):
        release(pad, key, auto_repeat=True)
        press(pad, key, auto_repeat=True)
        frame = pad.frame()
        assert frame.buttons[BUTTON[button]] == 1
        assert sum(frame.presses) == 0  # no new press: one E-STOP only
    release(pad, key)
    assert pad.frame().buttons[BUTTON[button]] == 0


def test_a_tap_inside_one_tick_is_published_once():
    pad = make_pad()
    press(pad, "Escape")
    release(pad, "Escape")
    frame = pad.frame()
    assert frame.buttons[BUTTON["b"]] == 1
    assert frame.presses[BUTTON["b"]] == 1
    frame = pad.frame()
    assert frame.buttons[BUTTON["b"]] == 0
    assert frame.presses[BUTTON["b"]] == 0


def test_two_sources_on_one_button_are_one_press():
    pad = make_pad()
    press(pad, "Space")
    pad.mouse_control("leftshoulder", True)
    assert pad.frame().presses[BUTTON["leftshoulder"]] == 1
    release(pad, "Space")
    assert pad.frame().buttons[BUTTON["leftshoulder"]] == 1  # mouse still holds it


# ── focus ───────────────────────────────────────────────────────────────


def test_focus_loss_releases_everything():
    pad = make_pad()
    for key in ("W", "D", "Left", "Space", "Backspace"):
        press(pad, key)
    pad.drag_stick("right", 0.3, 0.3)
    pad.mouse_control("a", True)
    pad.mouse_control("triggerright", True)
    pad.focus_lost()
    expect(pad.frame())


def test_keys_held_through_refocus_stay_released():
    pad = make_pad()
    press(pad, "W")
    press(pad, "Space")
    pad.frame()
    pad.focus_lost()
    # focus is back, the keys are still physically down: X11 keeps
    # auto-repeating them, then the real release comes
    for _ in range(10):
        press(pad, "W", auto_repeat=True)
        press(pad, "Space", auto_repeat=True)
        expect(pad.frame())
    release(pad, "W")
    release(pad, "Space")
    expect(pad.frame())
    press(pad, "W")  # pressed again: live again
    expect(pad.frame(), {"lefty": 0.25})


def test_a_press_before_focus_loss_still_counts():
    pad = make_pad()
    press(pad, "Escape")
    pad.focus_lost()
    frame = pad.frame()
    assert frame.buttons == (0,) * len(LAYOUT.buttons)
    assert frame.presses[BUTTON["b"]] == 1
