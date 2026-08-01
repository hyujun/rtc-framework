"""The robot launches hand ``use_cpu_affinity`` down to udp_hand_node (issue #345).

The hand process now pins its own threads: the main thread to the ``hand_driver``
slot and ``hand_aux_io`` to the aux slot. Those pins live inside the process, so
the launch-level ``use_cpu_affinity`` switch no longer reaches them by itself —
it has to be forwarded as a node parameter. Forget the forward and everything
still launches, the topics still publish, and ``use_cpu_affinity:=false`` quietly
keeps pinning: the exact silent-drift shape that made #344's wiring tests
necessary in the first place.

Structural (AST) rather than behavioural for the same reason as
``test_launch_shield_wiring``: a real launch description needs a running ROS
graph, and what drifts is which arguments a launch file passes.
"""

from __future__ import annotations

import ast
import os

import pytest
from ament_index_python.packages import get_package_share_directory

# Only the robot launches own the hand driver process. The sim launches bring up
# no udp_hand_node, which the sanity test at the bottom pins.
HAND_LAUNCH_FILES = [
    "robot_ur5e_p1a.launch.py",
    "robot_ur5e_p1b.launch.py",
]

SIM_LAUNCH_FILES = [
    "sim_ur5e_p1a.launch.py",
    "sim_ur5e_p1b.launch.py",
    "sim_iiwa7_leap.launch.py",
]


def _source(filename: str) -> str:
    path = os.path.join(get_package_share_directory("integrated_bringup"), "launch", filename)
    with open(path) as handle:
        return handle.read()


def _hand_node_call(source: str) -> ast.Call | None:
    """The ``LifecycleNode(...)``/``Node(...)`` call whose executable is udp_hand_node."""
    for node in ast.walk(ast.parse(source)):
        if not isinstance(node, ast.Call):
            continue
        for kw in node.keywords:
            if (
                kw.arg == "executable"
                and isinstance(kw.value, ast.Constant)
                and kw.value.value == "udp_hand_node"
            ):
                return node
    return None


def _parameter_keys(call: ast.Call) -> set[str]:
    """String keys of every dict literal inside the call's ``parameters=[...]``."""
    keys: set[str] = set()
    for kw in call.keywords:
        if kw.arg != "parameters":
            continue
        for element in ast.walk(kw.value):
            if isinstance(element, ast.Dict):
                for key in element.keys:
                    if isinstance(key, ast.Constant) and isinstance(key.value, str):
                        keys.add(key.value)
    return keys


@pytest.mark.parametrize("filename", HAND_LAUNCH_FILES)
def test_robot_launch_declares_a_hand_node(filename: str) -> None:
    assert _hand_node_call(_source(filename)) is not None, (
        f"{filename} no longer launches udp_hand_node — the affinity forwarding "
        "tests below would pass vacuously"
    )


@pytest.mark.parametrize("filename", HAND_LAUNCH_FILES)
def test_hand_node_receives_use_cpu_affinity(filename: str) -> None:
    call = _hand_node_call(_source(filename))
    assert call is not None
    assert "use_cpu_affinity" in _parameter_keys(call), (
        f"{filename} does not forward use_cpu_affinity to udp_hand_node; the "
        "process would keep pinning its own threads with the switch off"
    )


def _parameter_expr(call: ast.Call, key: str) -> ast.expr | None:
    """The value expression bound to ``key`` inside the call's ``parameters=[...]``."""
    for kw in call.keywords:
        if kw.arg != "parameters":
            continue
        for element in ast.walk(kw.value):
            if not isinstance(element, ast.Dict):
                continue
            for dict_key, dict_value in zip(element.keys, element.values):
                if isinstance(dict_key, ast.Constant) and dict_key.value == key:
                    return dict_value
    return None


@pytest.mark.parametrize("filename", HAND_LAUNCH_FILES)
def test_use_cpu_affinity_is_forwarded_as_a_bool(filename: str) -> None:
    """``value_type=bool`` is load-bearing, not decoration.

    The launch argument is the string "true"/"false"; an unqualified
    LaunchConfiguration reaches the node as a *string*, which clashes with the
    node's bool declaration and fails the transition. The failure mode is
    lopsided too: the default "true" path is exercised constantly, so a missing
    conversion would only surface the first time someone sets it false.

    Asserted on the AST rather than the source text: the first version of this
    test searched ``ast.get_source_segment`` output, which includes comments, and
    the explanatory comment beside the forward happens to contain the literal
    ``value_type=bool`` — so it passed with the conversion deleted.
    """
    call = _hand_node_call(_source(filename))
    assert call is not None
    value = _parameter_expr(call, "use_cpu_affinity")
    assert isinstance(value, ast.Call), (
        f"{filename}: use_cpu_affinity is not wrapped in ParameterValue(...)"
    )
    assert isinstance(value.func, ast.Name) and value.func.id == "ParameterValue"
    value_types = [
        kw.value.id
        for kw in value.keywords
        if kw.arg == "value_type" and isinstance(kw.value, ast.Name)
    ]
    assert value_types == ["bool"], (
        f"{filename} forwards use_cpu_affinity without value_type=bool "
        f"(found {value_types or 'nothing'}); the node would receive a string"
    )


@pytest.mark.parametrize("filename", SIM_LAUNCH_FILES)
def test_sim_launches_have_no_hand_driver_process(filename: str) -> None:
    assert _hand_node_call(_source(filename)) is None
