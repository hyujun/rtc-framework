"""The standard gamepad layout file: names, order and counts (SDL GameController)."""

from __future__ import annotations

import pytest

from rtc_teleop_input.layout import LayoutError, load_layout, parse_layout

# SDL_CONTROLLER_AXIS_* / SDL_CONTROLLER_BUTTON_* order (SDL 2.30), as joy's
# game_controller_node publishes it. Written out, not derived from the file.
SDL_AXES = ("leftx", "lefty", "rightx", "righty", "triggerleft", "triggerright")
SDL_BUTTONS = (
    "a", "b", "x", "y", "back", "guide", "start", "leftstick", "rightstick",
    "leftshoulder", "rightshoulder", "dpad_up", "dpad_down", "dpad_left", "dpad_right",
    "misc1", "paddle1", "paddle2", "paddle3", "paddle4", "touchpad",
)  # fmt: skip


def test_installed_layout_is_the_sdl_order():
    layout = load_layout()
    assert layout.axes == SDL_AXES
    assert layout.buttons == SDL_BUTTONS
    assert (len(layout.axes), len(layout.buttons)) == (6, 21)


def test_index_lookup_and_unknown_names():
    layout = load_layout()
    assert layout.axis_index("rightx") == 2
    assert layout.button_index("leftshoulder") == 9
    with pytest.raises(LayoutError):
        layout.axis_index("a")  # a button name is not an axis
    with pytest.raises(LayoutError):
        layout.button_index("lefty")


@pytest.mark.parametrize(
    "data",
    [
        None,
        {"axes": ["x"]},
        {"axes": ["x"], "buttons": ["a"], "extra": []},
        {"axes": [], "buttons": ["a"]},
        {"axes": ["x", "x"], "buttons": ["a"]},
        {"axes": ["a"], "buttons": ["a"]},
        {"axes": ["x", 3], "buttons": ["a"]},
    ],
)
def test_malformed_layout_is_refused(data):
    with pytest.raises(LayoutError):
        parse_layout(data)


def test_missing_file_is_refused(tmp_path):
    with pytest.raises(LayoutError):
        load_layout(tmp_path / "none.yaml")
