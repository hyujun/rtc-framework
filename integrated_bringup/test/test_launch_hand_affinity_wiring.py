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
import re
from pathlib import Path

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


_REPO_ROOT = Path(__file__).resolve().parents[2]
_UDP_HAND_NODE_HPP = (
    _REPO_ROOT / "udp_hand_driver" / "include" / "udp_hand_driver" / "udp_hand_node.hpp"
)


def _aux_thread_name() -> str:
    """The aux-lane thread name as C++ spells it (``kHandAuxIoConfig.name``).

    The name crosses a language boundary: udp_hand_node applies it with
    pthread_setname_np, and the launch files hand the same literal to
    pin_dds_threads_to_slot so the DDS sweep skips that TID. Nothing but this
    test couples the two — hand_aux_io is package-local, so it sits outside the
    RTC_OWNED_THREAD_NAMES ↔ thread_config.hpp mirror that protects every
    framework-owned thread. Rename one side and the compiler stays silent while
    the sweep drags the aux lane back onto the hand_driver core (the failure
    #345 actually shipped into and had to back out).
    """
    text = _UDP_HAND_NODE_HPP.read_text()
    block = re.search(r"kHandAuxIoConfig\s*\{(.*?)\}", text, re.DOTALL)
    assert block, f"kHandAuxIoConfig not found in {_UDP_HAND_NODE_HPP}"
    name = re.search(r'\.name\s*=\s*"([^"]+)"', block.group(1))
    assert name, f"kHandAuxIoConfig has no .name initializer: {block.group(1)!r}"
    return name.group(1)


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
            for dict_key, dict_value in zip(element.keys, element.values, strict=True):
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


# ── The hand pin is a DDS co-pin, not an all-thread sweep (issue #345) ───────


def _hand_pin_call(source: str) -> ast.Call | None:
    """The pinning call whose first argument labels the hand node."""
    for node in ast.walk(ast.parse(source)):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
            continue
        if not node.func.id.startswith("pin_"):
            continue
        if (
            node.args
            and isinstance(node.args[0], ast.Constant)
            and node.args[0].value == "udp_hand_node"
        ):
            return node
    return None


@pytest.mark.parametrize("filename", HAND_LAUNCH_FILES)
def test_hand_pin_uses_the_dds_helper(filename: str) -> None:
    """``pin_process_to_slot`` pinned every TID onto the hand_driver core.

    That is now wrong: the aux lane belongs on a different core, and the process
    pins its own threads. The remaining job is the DDS threads rclcpp::init()
    creates before the node exists.
    """
    call = _hand_pin_call(_source(filename))
    assert call is not None, f"{filename} no longer pins udp_hand_node at all"
    assert isinstance(call.func, ast.Name)
    assert call.func.id == "pin_dds_threads_to_slot", (
        f"{filename} pins the hand process with {call.func.id}; an all-thread "
        "sweep would drag hand_aux_io back onto the hand_driver core"
    )


@pytest.mark.parametrize("filename", HAND_LAUNCH_FILES)
def test_hand_pin_protects_the_aux_thread(filename: str) -> None:
    """The aux lane must be named explicitly, using the name C++ actually sets.

    It is SCHED_OTHER and absent from thread_config.hpp, so the helper's two
    default filters — the RTC-owned name mirror and the FIFO guard — both miss
    it. Without this argument the sweep silently re-collapses the lane split.

    Read from kHandAuxIoConfig rather than spelled as a literal here: that makes
    this the cross-language drift lock (#353). A literal would keep passing
    after the C++ side is renamed, which is precisely the state that breaks the
    pin.
    """
    call = _hand_pin_call(_source(filename))
    assert call is not None
    protected: set[str] = set()
    for kw in call.keywords:
        if kw.arg != "extra_protected_names":
            continue
        for element in ast.walk(kw.value):
            if isinstance(element, ast.Constant) and isinstance(element.value, str):
                protected.add(element.value)
    aux_name = _aux_thread_name()
    assert aux_name in protected, (
        f"{filename} does not protect {aux_name} (kHandAuxIoConfig.name) from the "
        f"DDS sweep (found {protected or 'none'})"
    )


@pytest.mark.parametrize("filename", HAND_LAUNCH_FILES)
def test_no_launch_requests_an_all_thread_sweep(filename: str) -> None:
    """Belt-and-suspenders against the flag returning under any caller."""
    source = _source(filename)
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Call):
            for kw in node.keywords:
                assert kw.arg != "all_threads", f"{filename} still requests an all-thread sweep"
