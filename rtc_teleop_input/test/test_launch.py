"""teleop_input.launch.py: the nodes and parameters it starts."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch_ros.actions import Node

from rtc_teleop_input.launch_params import GAME_CONTROLLER_PARAMETERS, node_specs

LAUNCH_FILE = Path(__file__).resolve().parents[1] / "launch" / "teleop_input.launch.py"


def test_game_controller_parameters_are_written_out():
    # game_controller_node's own defaults, pinned (joy-params.yaml has deadzone 0.5)
    assert GAME_CONTROLLER_PARAMETERS == {
        "deadzone": 0.05,
        "autorepeat_rate": 20.0,
        "sticky_buttons": False,
        "coalesce_interval_ms": 1,
    }


def test_virtual_starts_the_input_node_only():
    specs = node_specs({"input": "virtual", "joy_topic": "/x/joy"})
    assert len(specs) == 1
    assert (specs[0].package, specs[0].executable) == ("rtc_teleop_input", "teleop_input")
    assert specs[0].parameters == {"input": "virtual", "output_topic": "/x/joy"}


def test_pad_adds_game_controller_node():
    specs = node_specs(
        {
            "input": "pad",
            "joy_topic": "/x/joy",
            "skin": "xbox",
            "controller_name": "ctrl",
            "status_topics": "/a/status, /b/status",
            "rate_hz": "100",
            "pad_topic": "/x/pad",
        }
    )
    assert specs[0].parameters == {
        "input": "pad",
        "output_topic": "/x/joy",
        "skin": "xbox",
        "reset_controller_name": "ctrl",
        "status_topics": ["/a/status", "/b/status"],
        "rate_hz": 100.0,
        "pad_topic": "/x/pad",
    }
    pad = specs[1]
    assert (pad.package, pad.executable) == ("joy", "game_controller_node")
    assert pad.parameters == GAME_CONTROLLER_PARAMETERS
    assert pad.remappings == (("joy", "/x/pad"),)


def test_pad_default_upstream_is_joy():
    specs = node_specs({"input": "pad", "joy_topic": "/x/joy"})
    assert specs[1].remappings == (("joy", "joy"),)


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("teleop_input_launch", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(("mode", "count"), [("virtual", 1), ("pad", 2)])
def test_launch_file_builds_the_specs(mode, count):
    module = _load_launch_module()
    description = module.generate_launch_description()
    assert description.entities
    context = LaunchContext()
    context.launch_configurations.update(
        {"input": mode, "joy_topic": "/x/joy", **dict.fromkeys(module._OPTIONAL, "")}
    )
    actions = module._launch_setup(context)
    assert len(actions) == count
    assert all(isinstance(a, Node) for a in actions)


def test_launch_file_does_not_use_joy_params_yaml():
    assert "joy-params" not in LAUNCH_FILE.read_text()
