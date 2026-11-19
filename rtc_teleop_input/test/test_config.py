"""validate_config: what the node refuses to start with."""

from __future__ import annotations

import math

import pytest

from rtc_teleop_input.layout import load_layout
from rtc_teleop_input.model import (
    MAX_RATE_HZ,
    ConfigError,
    StatusBoard,
    StatusEntry,
    TeleopConfig,
    validate_config,
)

LAYOUT = load_layout()


def check(**overrides):
    overrides.setdefault("output_topic", "out")
    return validate_config(TeleopConfig(**overrides), LAYOUT)


def test_defaults():
    config = TeleopConfig()
    assert config.rate_hz == 50.0
    assert config.pad_timeout_s == 0.25
    assert config.reset_hold_s == 1.0
    assert config.deflection == 0.25
    assert (config.estop_button, config.reset_button, config.deadman_button) == (
        "b",
        "back",
        "leftshoulder",
    )
    assert config.estop_service == "/rtc_cm/trigger_estop"
    assert config.reset_service == "/rtc_cm/reset_fault"
    # no topic or controller defaults: the consumer names them
    assert config.output_topic == ""
    assert config.reset_controller_name == ""
    assert config.status_topics == ()
    assert MAX_RATE_HZ == 1000.0
    check()


@pytest.mark.parametrize("rate", [0.0, -1.0, 1000.001, math.nan, math.inf])
def test_rate_out_of_range_is_refused(rate):
    with pytest.raises(ConfigError, match="rate_hz"):
        check(rate_hz=rate)


def test_rate_bounds_accepted():
    check(rate_hz=1000.0)
    check(rate_hz=0.5)


@pytest.mark.parametrize(
    "overrides",
    [
        {"output_topic": ""},
        {"input": "keyboard"},
        {"skin": "ps4"},
        {"deflection": 0.0},
        {"deflection": 1.5},
        {"pad_timeout_s": 0.0},
        {"reset_hold_s": -1.0},
        {"input": "pad", "pad_topic": ""},
        {"estop_service": ""},
        {"estop_button": "lefty"},
        {"deadman_button": "nope"},
        {"reset_button": "b"},  # same as the E-STOP button
        {"status_topics": ("a", "a")},
        {"status_topics": ("",)},
        {"key_axes": ("lefty:W",)},  # no direction
        {"key_axes": ("leftshoulder+:W",)},  # a button where an axis goes
        {"key_axes": ("lefty+:W", "lefty+:I")},  # one direction twice
        {"key_axes": ("lefty+:W", "leftx+:W")},  # one key twice
        {"key_buttons": ("lefty:Space",)},  # an axis where a button goes
        {"key_buttons": ("b:Escape", "a:Escape")},
        {"key_buttons": ("b",)},
    ],
)
def test_bad_config_is_refused(overrides):
    with pytest.raises(ConfigError):
        check(**overrides)


def test_status_board_keeps_the_latest_per_configured_topic():
    board = StatusBoard(("s1", "s2"))
    board.update(StatusEntry("s2", "n2", 1, "warn", (("k", "v"),), 1.0))
    board.update(StatusEntry("s1", "n1", 0, "", (), 1.0))
    board.update(StatusEntry("other", "x", 0, "", (), 1.0))
    board.update(StatusEntry("s1", "n1", 2, "bad", (("a", "1"), ("b", "2")), 2.0))
    rows = board.rows()
    assert [r.topic for r in rows] == ["s1", "s2"]
    assert rows[0].level_name == "ERROR"
    assert rows[0].values == (("a", "1"), ("b", "2"))
    assert rows[1].level_name == "WARN"
