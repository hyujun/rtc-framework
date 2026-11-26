"""teleop:= of the humanoid sims: integrated_bringup.teleop_launch and the
shipped config/<profile>/teleop_input.yaml files.

The files must start the teleop node (the teleop launch's params_file check and
the node's own validation), clear only the controller manager's operator
E-STOP reason, and agree with the Joy overlay the controller reads: every
labelled axis is one the controller maps, and the help combination's axes are
ones it does not (pressing it must not move the robot).
"""

from __future__ import annotations

import os
import re

import pytest
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory

from integrated_bringup.teleop_launch import (
    TELEOP_MODES,
    joy_topic,
    resolve_command_input,
    teleop_launch_arguments,
    teleop_mode,
)
from rtc_teleop_input.launch_params import check_params_file
from rtc_teleop_input.layout import load_layout
from rtc_teleop_input.model import TeleopConfig, help_entries, validate_config

SHARE = get_package_share_directory("integrated_bringup")
PROFILES = {"g1_loco": "g1", "b7_loco": "b7"}


def _params(profile: str) -> dict:
    path = os.path.join(SHARE, "config", profile, "teleop_input.yaml")
    with open(path) as stream:
        return yaml.safe_load(stream)["teleop_input"]["ros__parameters"]


def _config(profile: str) -> TeleopConfig:
    values = {k: tuple(v) if isinstance(v, list) else v for k, v in _params(profile).items()}
    return TeleopConfig(output_topic=joy_topic(PROFILES[profile]), **values)


def test_teleop_mode():
    assert TELEOP_MODES == ("none", "virtual", "pad")
    assert teleop_mode("") == "none"
    with pytest.raises(ValueError, match="teleop"):
        teleop_mode("keyboard")


@pytest.mark.parametrize(
    ("command_input", "teleop", "expected"),
    [
        ("auto", "none", "twist"),
        ("", "", "twist"),
        ("auto", "virtual", "joy"),
        ("auto", "pad", "joy"),
        ("joy", "none", "joy"),
        ("twist", "none", "twist"),
        ("joy", "pad", "joy"),
    ],
)
def test_command_input_resolution(command_input, teleop, expected):
    assert resolve_command_input(command_input, teleop) == expected


def test_twist_with_a_teleop_is_refused():
    with pytest.raises(ValueError, match="command_input:=twist"):
        resolve_command_input("twist", "virtual")


def test_launch_arguments():
    assert teleop_launch_arguments("none", SHARE, "g1_loco", "g1") == {}
    assert teleop_launch_arguments("pad", SHARE, "g1_loco", "g1") == {
        "input": "pad",
        "joy_topic": "/locomotion_controller/g1/joy",
        "params_file": os.path.join(SHARE, "config", "g1_loco", "teleop_input.yaml"),
    }


@pytest.mark.parametrize("profile", PROFILES)
def test_profile_starts_the_teleop_node(profile):
    path = os.path.join(SHARE, "config", profile, "teleop_input.yaml")
    check_params_file(path)  # no launch-owned parameter
    config = _config(profile)
    validate_config(config, load_layout())
    # the operation loop: both switches, the clear, the recover trigger
    assert set(config.switch_buttons) == {
        "y:locomotion_controller",
        "a:demo_joint_controller",
    }
    assert config.clear_estop_button
    assert config.trigger_buttons == ("x:/sim_recover/recover",)
    assert config.reset_controller_name == "locomotion_controller"


def _operator_estop_reason() -> str:
    header = os.path.join(
        get_package_prefix("rtc_controller_manager"),
        "include",
        "rtc_controller_manager",
        "rt_controller_node.hpp",
    )
    with open(header) as stream:
        found = re.findall(r'kOperatorEstopReason\s*=\s*"([^"]*)"', stream.read())
    assert len(found) == 1, header
    return found[0]


@pytest.mark.parametrize("profile", PROFILES)
def test_the_pad_clears_only_the_operator_estop(profile):
    assert _params(profile)["clear_estop_reason"] == _operator_estop_reason()


def _joy_map(profile: str) -> dict:
    path = os.path.join(SHARE, "config", profile, "sim_overlays", "joy_command.yaml")
    with open(path) as stream:
        data = yaml.safe_load(stream)
    return data["integrated_rt_controller"]["ros__parameters"]["locomotion_controller"][
        "velocity_command"
    ]["joy"]


def test_g1_loco_help_matches_the_joy_overlay():
    joy = _joy_map("g1_loco")
    assert joy["height_rate"] == "righty"
    mapped = {v for k, v in joy.items() if k != "deadman"}
    config = _config("g1_loco")
    assert joy["deadman"] == config.deadman_button
    # the combination's axes are not commands
    assert config.help_combo and not set(config.help_combo) & mapped
    # every mapped axis is labelled, and nothing else is
    labelled = {e.split(":", 1)[0] for e in config.help_labels}
    assert labelled == mapped
    # and each one has keys in the virtual pad
    rows = {e.controls[0]: e for e in help_entries(config) if len(e.controls) == 1}
    assert all(rows[axis].keys for axis in mapped)


def test_b7_loco_has_no_stick_meaning_yet():
    # its locomotion_controller reads no command: no axis is labelled
    assert "help_labels" not in _params("b7_loco")
    assert not os.path.exists(
        os.path.join(SHARE, "config", "b7_loco", "sim_overlays", "joy_command.yaml")
    )
