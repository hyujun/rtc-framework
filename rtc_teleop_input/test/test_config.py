"""validate_config: what the node refuses to start with."""

from __future__ import annotations

import math

import pytest

from rtc_teleop_input.layout import load_layout
from rtc_teleop_input.model import (
    MAX_RATE_HZ,
    STATUS_STALE_S,
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


@pytest.mark.parametrize("rate", [0.0, -1.0, 1000.001, math.nan, math.inf, 30.0, 300.0, 600.0])
def test_rate_out_of_range_is_refused(rate):
    with pytest.raises(ConfigError, match="rate_hz"):
        check(rate_hz=rate)


def test_rate_bounds_accepted():
    # whole-millisecond periods only (the timer counts ms)
    for rate in (1000.0, 500.0, 250.0, 100.0, 50.0, 40.0, 0.5):
        check(rate_hz=rate)


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


def test_status_goes_stale_after_two_seconds():
    assert STATUS_STALE_S == 2.0
    entry = StatusEntry("s", "n", 0, "", (), received_s=10.0)
    assert not entry.is_stale(12.0)
    assert entry.is_stale(12.001)


ROLES = {
    "switch_buttons": ("y:ctrl_a", "a:ctrl_b"),
    "clear_estop_button": "start",
    "clear_estop_reason": "reason",
    "trigger_buttons": ("x:/svc",),
}


def test_new_role_defaults_are_off():
    config = TeleopConfig()
    assert config.switch_buttons == () and config.trigger_buttons == ()
    assert config.clear_estop_button == "" and config.clear_estop_reason == ""
    assert config.hold_s == 1.0
    assert config.switch_service == "/rtc_cm/switch_controller"
    assert config.clear_estop_service == "/rtc_cm/clear_estop"
    assert config.active_controller_topic == "/rtc_cm/active_controller_name"
    assert config.estop_status_topic == "/system/estop_status"
    # help: the key is on; the combination and labels are the consumer's
    assert (config.help_key, config.help_combo, config.help_labels) == ("F1", (), ())
    check(**ROLES, help_combo=("triggerleft", "triggerright"), help_labels=("lefty:fwd",))


@pytest.mark.parametrize(
    "overrides",
    [
        {"hold_s": 0.0},
        {"hold_s": math.nan},
        {"switch_buttons": ("y",)},
        {"switch_buttons": ("y:",)},
        {"switch_buttons": (":ctrl",)},
        {"switch_buttons": ("y:a", "y:b")},  # one button twice
        {"switch_buttons": ("b:ctrl",)},  # the E-STOP button
        {"switch_buttons": ("leftshoulder:ctrl",)},  # the deadman
        {"switch_buttons": ("back:ctrl",)},  # the reset button
        {"switch_buttons": ("paddle1:ctrl",)},  # not drawn: cannot be seen or clicked
        {"switch_buttons": ("lefty:ctrl",)},  # an axis
        {"switch_buttons": ("y:ctrl",), "switch_service": ""},
        {"trigger_buttons": ("y:/svc",), "switch_buttons": ("y:ctrl",)},
        {"trigger_buttons": ("touchpad:/svc",)},
        {"clear_estop_button": "start"},  # no reason: it could clear nothing
        {"clear_estop_button": "start", "clear_estop_reason": "r", "clear_estop_service": ""},
        {"clear_estop_button": "x", "clear_estop_reason": "r", "trigger_buttons": ("x:/svc",)},
        {"clear_estop_button": "misc1", "clear_estop_reason": "r"},
        {"estop_button": "misc1"},
        {"help_key": "Space"},  # bound to the deadman
        {"help_key": "Escape"},
        {"help_combo": ("triggerleft", "triggerleft")},
        {"help_combo": ("nope",)},
        {"help_combo": ("b",)},  # a role button
        {"help_combo": ("paddle2",)},
        {"help_labels": ("lefty",)},
        {"help_labels": ("lefty: ",)},
        {"help_labels": ("nope:x",)},
        {"help_labels": ("lefty:a", "lefty:b")},
        {"help_labels": ("misc1:x",)},
    ],
)
def test_bad_roles_are_refused(overrides):
    with pytest.raises(ConfigError):
        check(**overrides)
