"""Every bringup launch uses the shared shield wiring (issue #344).

This is the sensor that was missing. The cset-aware shield probe (#151), the
sudo diagnostic (#233), and the adopt-before-activate chain each landed in
``robot_ur5e_p1b`` alone, and the other four launches kept the old behaviour
through all three fixes — because nothing anywhere asserted that a launch file
was wired correctly. ``rtc_tools/test/test_cpu_shield_actions.py`` tests the
helpers; these tests assert the launches actually *call* them.

The checks are deliberately structural rather than behavioural: building a real
launch description would need a running ROS graph, while what drifted was always
which helper a file called. Both the import and the call are asserted, so
deleting the call while keeping the import (or vice versa) fails.
"""

from __future__ import annotations

import ast
import os

import pytest
from ament_index_python.packages import get_package_share_directory

# Every launch that brings up integrated_rt_controller under a possible shield.
# --robot and --sim shield the same CM-spanning range (cpu_shield.sh funnels both
# to get_cm_shield_cpus), so the sim launches carry the same EINVAL exposure.
LAUNCH_FILES = [
    "robot_ur5e_p1a.launch.py",
    "robot_ur5e_p1b.launch.py",
    "sim_ur5e_p1a.launch.py",
    "sim_ur5e_p1b.launch.py",
    "sim_iiwa7_leap.launch.py",
]


def _source(filename: str) -> str:
    path = os.path.join(get_package_share_directory("integrated_bringup"), "launch", filename)
    with open(path) as handle:
        return handle.read()


def _called_functions(source: str) -> set[str]:
    """Names of every function called, including ``a.b()`` as ``b``."""
    tree = ast.parse(source)
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            func = node.func
            if isinstance(func, ast.Attribute):
                names.add(func.attr)
            elif isinstance(func, ast.Name):
                names.add(func.id)
    return names


@pytest.mark.parametrize("filename", LAUNCH_FILES)
def test_uses_shared_shield_probe(filename):
    """No launch may hand-roll the shield-active probe.

    An inline probe is how the isolcpus-only version survived in four files: a
    cset shield writes nothing to /sys/devices/system/cpu/isolated, so that test
    reads an active shield as inactive and re-runs `on` every launch (#151).
    """
    source = _source(filename)
    assert "enable_cpu_shield" in _called_functions(source)
    # The give-away that a local copy came back.
    assert "/sys/devices/system/cpu/isolated" not in source


@pytest.mark.parametrize("filename", LAUNCH_FILES)
def test_shield_probe_receives_the_mpc_profile(filename):
    """The shield must be built for the profile this launch actually runs (#350).

    ``enable_mpc`` reached the controller override and stopped there, so the cset
    shield reserved the MPC cores on every launch no matter what. Omitting the
    keyword is silent — the helper defaults to reserving — which is exactly the
    shape of drift this module exists to catch: the fix would land in one launch
    and the other four would keep isolating cores nobody runs on.
    """
    tree = ast.parse(_source(filename))
    calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "enable_cpu_shield"
    ]
    assert calls, "no enable_cpu_shield call found"
    for call in calls:
        assert "enable_mpc" in {kw.arg for kw in call.keywords}, (
            f"{filename}: enable_cpu_shield must be passed enable_mpc, otherwise the "
            "shield keeps reserving the MPC cores regardless of the launch argument"
        )


@pytest.mark.parametrize("filename", LAUNCH_FILES)
def test_activate_is_gated_on_shield_adopt(filename):
    """ACTIVATE must come from the shared adopt chain, never straight from configure.

    The RT threads are spawned in on_activate and only inherit the shielded
    cpuset if the process was adopted first; activating directly is what left the
    controller unpinned and off SCHED_FIFO under an active shield (#151, #344).
    """
    source = _source(filename)
    assert "shield_adopt_then_activate" in _called_functions(source)
    # The pre-#344 shape: a bare handler that emitted ACTIVATE with no adopt in
    # between. Its absence is what makes the assertion above load-bearing.
    assert "rt_auto_activate" not in source


@pytest.mark.parametrize("filename", LAUNCH_FILES)
def test_imports_the_shared_module(filename):
    """The call must resolve to rtc_tools.launch.cpu_shield, not a local shim."""
    tree = ast.parse(_source(filename))
    imported = any(
        isinstance(node, ast.ImportFrom)
        and node.module == "rtc_tools.launch"
        and any(alias.name == "cpu_shield" for alias in node.names)
        for node in ast.walk(tree)
    )
    assert imported, "expected `from rtc_tools.launch import cpu_shield`"


@pytest.mark.parametrize("filename", LAUNCH_FILES)
def test_no_direct_adopt_call(filename):
    """Launches must not call ``adopt_process_into_shield`` directly.

    Calling it directly is what makes the exit-code contract easy to drop: the
    ordering *and* the fail-closed check live in shield_adopt_then_activate, and
    p1b's own hand-rolled chain ignored the exit code entirely.
    """
    assert "adopt_process_into_shield" not in _called_functions(_source(filename))
