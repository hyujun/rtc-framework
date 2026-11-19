"""PadRelay: a real pad's Joy relayed bit for bit, released when silent or malformed."""

from __future__ import annotations

from array import array

from rtc_teleop_input.layout import load_layout
from rtc_teleop_input.model import PadRelay

LAYOUT = load_layout()
B = {name: i for i, name in enumerate(LAYOUT.buttons)}
TIMEOUT = 0.25
RELEASED_AXES = (0.0,) * 6
RELEASED_BUTTONS = (0,) * 21


def buttons(*names):
    return [1 if n in names else 0 for n in LAYOUT.buttons]


def float32_bits(values):
    return array("f", values).tobytes()


def test_axes_relayed_bit_for_bit():
    relay = PadRelay(LAYOUT, TIMEOUT)
    # float32 values as joy delivers them (Joy.axes is float32[])
    axes = array("f", [0.1, -0.7, 1e-38, -1.0, 0.33333334, -0.0])
    relay.on_upstream(axes, buttons("a"), now=10.0)
    frame = relay.frame(10.01)
    assert frame.source == "pad"
    assert not frame.lost
    assert float32_bits(frame.axes) == axes.tobytes()
    assert frame.buttons == tuple(buttons("a"))


def test_wrong_size_is_released_and_counted():
    relay = PadRelay(LAYOUT, TIMEOUT)
    relay.on_upstream([0.5] * 6, buttons("leftshoulder"), now=1.0)
    relay.on_upstream([0.5] * 8, buttons("leftshoulder"), now=1.01)
    frame = relay.frame(1.02)
    assert (frame.axes, frame.buttons, frame.lost) == (RELEASED_AXES, RELEASED_BUTTONS, False)
    relay.on_upstream([0.5] * 6, buttons("leftshoulder")[:11], now=1.03)
    assert relay.rejects == 2
    assert relay.frame(1.04).axes == RELEASED_AXES
    relay.on_upstream([0.5] * 6, buttons("leftshoulder"), now=1.05)
    assert relay.frame(1.06).axes == (0.5,) * 6


def test_silent_upstream_is_released_then_relayed_again():
    relay = PadRelay(LAYOUT, TIMEOUT)
    assert relay.frame(0.0).lost  # never heard
    relay.on_upstream([0.5] * 6, buttons("leftshoulder"), now=1.0)
    assert relay.frame(1.0 + TIMEOUT).axes == (0.5,) * 6  # not yet over the timeout
    frame = relay.frame(1.0 + TIMEOUT + 0.001)
    assert frame.lost
    assert (frame.axes, frame.buttons) == (RELEASED_AXES, RELEASED_BUTTONS)
    assert relay.frame(5.0).lost
    relay.on_upstream([0.25] * 6, buttons(), now=6.0)
    frame = relay.frame(6.01)
    assert not frame.lost
    assert frame.axes == (0.25,) * 6


def test_presses_are_edges_not_repeats():
    relay = PadRelay(LAYOUT, TIMEOUT)
    for i in range(40):  # held B, re-published at the autorepeat rate
        relay.on_upstream([0.0] * 6, buttons("b"), now=i * 0.05)
    frame = relay.frame(2.0)
    assert frame.presses[B["b"]] == 1
    assert relay.frame(2.0).presses[B["b"]] == 0


def test_a_tap_between_ticks_is_published_and_counted():
    relay = PadRelay(LAYOUT, TIMEOUT)
    relay.on_upstream([0.0] * 6, buttons(), now=0.0)
    relay.on_upstream([0.0] * 6, buttons("b"), now=0.001)
    relay.on_upstream([0.0] * 6, buttons(), now=0.002)
    frame = relay.frame(0.01)
    assert frame.buttons[B["b"]] == 1
    assert frame.presses[B["b"]] == 1
    assert relay.frame(0.03).buttons[B["b"]] == 0


def test_a_button_held_through_a_loss_is_a_new_press():
    relay = PadRelay(LAYOUT, TIMEOUT)
    relay.on_upstream([0.0] * 6, buttons("back"), now=0.0)
    assert relay.frame(0.01).presses[B["back"]] == 1
    assert relay.frame(1.0).lost
    relay.on_upstream([0.0] * 6, buttons("back"), now=2.0)
    assert relay.frame(2.01).presses[B["back"]] == 1
