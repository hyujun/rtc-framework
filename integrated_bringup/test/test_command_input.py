"""sim_g1_loco's controller parameter layers and ``command_input:=``.

The default (twist) must build the list the launch built before the argument
existed; joy inserts the shipped overlay after the enable_mpc mapping and
before the user's sim_overlay.
"""

from __future__ import annotations

import ast
import os

import pytest
from ament_index_python.packages import get_package_share_directory

from integrated_bringup.command_input import (
    COMMAND_INPUTS,
    command_input_overlay,
    controller_param_layers,
)

BASE, CTRL, SIM, OVERLAY = "base.yaml", "sim.yaml", "mujoco_simulator.yaml", "user.yaml"


def test_twist_is_the_default_and_adds_nothing():
    assert COMMAND_INPUTS == ("twist", "joy")
    assert command_input_overlay("twist", "g1_loco") is None
    assert command_input_overlay("", "g1_loco") is None


def test_default_layers_are_the_previous_list():
    # the composition the launch had before command_input, written out
    assert controller_param_layers(BASE, CTRL, SIM, None, None, None) == [BASE, CTRL, SIM]
    assert controller_param_layers(BASE, CTRL, SIM, True, None, OVERLAY) == [
        BASE,
        CTRL,
        SIM,
        {"locomotion_controller.mpc.enabled": True},
        OVERLAY,
    ]
    assert controller_param_layers(BASE, CTRL, SIM, False, None, None) == [
        BASE,
        CTRL,
        SIM,
        {"locomotion_controller.mpc.enabled": False},
    ]


def test_joy_goes_after_the_mpc_mapping_and_before_the_user_overlay():
    joy = command_input_overlay("joy", "g1_loco")
    expected = os.path.join(
        get_package_share_directory("integrated_bringup"),
        "config",
        "g1_loco",
        "sim_overlays",
        "joy_command.yaml",
    )
    assert joy == expected
    assert os.path.isfile(joy)
    assert controller_param_layers(BASE, CTRL, SIM, True, joy, OVERLAY) == [
        BASE,
        CTRL,
        SIM,
        {"locomotion_controller.mpc.enabled": True},
        joy,
        OVERLAY,
    ]
    assert controller_param_layers(BASE, CTRL, SIM, None, joy, None) == [BASE, CTRL, SIM, joy]


def test_an_unknown_input_is_refused():
    with pytest.raises(ValueError, match="command_input"):
        command_input_overlay("keyboard", "g1_loco")


def _launch_source(name: str) -> str:
    path = os.path.join(get_package_share_directory("integrated_bringup"), "launch", name)
    with open(path) as stream:
        return stream.read()


def test_sim_g1_loco_builds_its_layers_with_these_functions():
    tree = ast.parse(_launch_source("sim_g1_loco.launch.py"))
    called = {
        node.func.id
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    }
    assert {"controller_param_layers", "command_input_overlay"} <= called


def test_b7_loco_has_no_command_input():
    assert "command_input" not in _launch_source("sim_b7_loco.launch.py")
