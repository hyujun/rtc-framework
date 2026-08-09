"""Opening a session is a launch-time action, not a build-time side effect (#402).

What this closes
----------------
Both robot launches used to call ``create_session_dir`` and
``cleanup_old_sessions`` from the body of ``generate_launch_description()``.
Merely *building* a description therefore created a session directory and
deleted every session past the tenth: measured on a tree seeded with 14,
one created and five deleted per call, without launching anything. Anything
that builds a description without running it — ``ros2 launch --show-args``, a
linter, ``test_combo_keys_are_declared_arguments`` next door — silently pruned
the developer's ``logging_data``.

Why the evaluation here walks entities in order
-----------------------------------------------
``test_launch_description_evaluates`` seeds a context with every declared
default up front and then executes the ``OpaqueFunction``s. That is the right
shape for asking "does evaluating raise?", but it makes argument ordering
invisible: an argument is present in the context whether or not its
``DeclareLaunchArgument`` has run yet. The session prologue introduced a real
ordering constraint — it must sit *below* the argument declarations, because it
reads ``max_log_sessions`` — so this file replays the description the way the
launch service does: only command-line overrides are pre-seeded, and each
entity contributes as it is visited. Moving the prologue above the arguments
makes these tests fail, which is the entire point of the arrangement.

The retention argument reaches two actors
-----------------------------------------
The RT controller node prunes the same tree again in ``on_configure`` with its
own ``max_log_sessions`` ROS parameter, which made the effective retention
``min(launch value, node parameter)``. So the assertions below check both ends
of one knob, not just the launch end.
"""

from __future__ import annotations

import importlib.util
import os
from typing import Any

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.utilities import perform_substitutions
from launch_ros.actions import LifecycleNode

from rtc_tools.launch.cm_rt_params import CM_RT_PARAM_FILENAME
from rtc_tools.utils.session_dir import is_session_dir_name, resolve_logging_root

SIM_LAUNCH_FILES = [
    "sim_ur5e_p1a.launch.py",
    "sim_ur5e_p1b.launch.py",
    "sim_iiwa7_leap.launch.py",
]
ROBOT_LAUNCH_FILES = [
    "robot_ur5e_p1a.launch.py",
    "robot_ur5e_p1b.launch.py",
]
LAUNCH_FILES = SIM_LAUNCH_FILES + ROBOT_LAUNCH_FILES

RT_CONTROLLER_EXECUTABLE = "integrated_rt_controller"


@pytest.fixture(autouse=True)
def isolated_logging_root(monkeypatch, tmp_path):
    """Point the session tree at ``tmp_path`` — and prove the redirect took.

    These tests both create and delete session directories. ``resolve_logging_root``
    prefers ``$COLCON_PREFIX_PATH``'s first entry but falls back to walking the
    cwd for an ``install/`` + ``src/`` pair, which under ``colcon test`` finds
    the real workspace; the assertion is what makes the redirect a fact rather
    than an intention.
    """
    prefix = tmp_path / "install"
    prefix.mkdir()
    monkeypatch.setenv("COLCON_PREFIX_PATH", str(prefix))
    # ament_index reads AMENT_PREFIX_PATH, so the real share stays reachable.
    root = resolve_logging_root()
    assert root.startswith(str(tmp_path)), (
        f"logging root escaped the sandbox ({root}); these tests prune sessions"
    )
    os.makedirs(root, exist_ok=True)
    return root


_MODULE_CACHE: dict[str, Any] = {}


def _load(filename: str):
    """Load a launch file from the *installed* share, as ``ros2 launch`` does."""
    if filename not in _MODULE_CACHE:
        path = os.path.join(get_package_share_directory("integrated_bringup"), "launch", filename)
        spec = importlib.util.spec_from_file_location(f"_session_{filename}", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        _MODULE_CACHE[filename] = module
    return _MODULE_CACHE[filename]


def _seed_sessions(root: str, count: int) -> list[str]:
    """Create ``count`` session directories that sort before any real one."""
    names = [f"2601{index:02d}_1200" for index in range(count)]
    for name in names:
        os.makedirs(os.path.join(root, name), exist_ok=True)
    return names


def _sessions_in(root: str) -> list[str]:
    return sorted(name for name in os.listdir(root) if is_session_dir_name(name))


def _walk(entities, depth: int = 0):
    """Yield every entity, including those behind event handlers and timers.

    The RT controller node is not a top-level entity: it is launched from an
    ``OnProcessExit`` handler, so a flat scan of ``description.entities`` would
    miss precisely the node whose ``log_dir`` this file is about.
    """
    for entity in entities:
        yield entity
        if depth >= 8:
            continue
        sub: list = []
        try:
            sub += list(entity.describe_sub_entities())
        except Exception:  # noqa: BLE001 — not every action describes children
            pass
        try:
            for _, conditional in entity.describe_conditional_sub_entities():
                sub += list(conditional)
        except Exception:  # noqa: BLE001
            pass
        yield from _walk(sub, depth + 1)


def _evaluate_in_order(
    description: LaunchDescription, overrides: dict[str, str] | None = None
) -> tuple[LaunchContext, list]:
    """Replay the description the way the launch service does.

    Only overrides are pre-seeded (those are the command line); every declared
    default arrives when its ``DeclareLaunchArgument`` is visited. Actions an
    ``OpaqueFunction`` returns are applied before the next sibling, which is how
    ``$RTC_SESSION_DIR`` reaches later entities in a real launch.
    """
    context = LaunchContext()
    context.launch_configurations.update(overrides or {})
    produced: list = []

    def apply(entity) -> None:
        if isinstance(entity, (DeclareLaunchArgument, SetLaunchConfiguration)) or isinstance(
            entity, SetEnvironmentVariable
        ):
            entity.execute(context)
        elif isinstance(entity, OpaqueFunction):
            for returned in entity.execute(context) or []:
                produced.append(returned)
                apply(returned)

    for entity in description.entities:
        apply(entity)
    return context, produced


def _resolved_rt_controller_parameters(description, context, produced) -> dict:
    """Resolve the RT controller node's parameters exactly as launch_ros will.

    ``_perform_substitutions`` is what the node action runs before spawning:
    it evaluates every substitution and writes the parameter dicts out as real
    YAML files. Reading those back — in order, later files winning, which is
    ROS 2's own per-key overlay rule — gives the values the node process would
    actually be started with.

    Both roots have to be searched: the robot launches build the node while the
    description is built and reach it through an event handler, while the sim
    launches build it inside ``launch_setup`` and return it as an action.
    """
    nodes = [
        entity
        for entity in _walk(list(description.entities) + list(produced))
        if isinstance(entity, LifecycleNode) and entity.node_executable == RT_CONTROLLER_EXECUTABLE
    ]
    assert len(nodes) == 1, (
        f"expected exactly one {RT_CONTROLLER_EXECUTABLE} node, found {len(nodes)}; "
        "the traversal or the launch structure changed"
    )
    node = nodes[0]
    node._perform_substitutions(context)

    expanded = getattr(node, "_Node__expanded_parameter_arguments", None)
    assert expanded, (
        "launch_ros no longer exposes _Node__expanded_parameter_arguments, so "
        "this test can no longer see the resolved parameters and must be rewired "
        "— it must not silently assert nothing"
    )

    merged: dict = {}
    for argument, is_file in expanded:
        if not is_file or not os.path.isfile(argument):
            continue
        with open(argument) as handle:
            merged.update(_flatten_ros_parameters(yaml.safe_load(handle) or {}))
    return merged


def _flatten_ros_parameters(document: dict) -> dict:
    """Collect ``ros__parameters`` blocks from a parameter file, any node key."""
    collected: dict = {}
    for value in document.values():
        if isinstance(value, dict) and "ros__parameters" in value:
            collected.update(value["ros__parameters"] or {})
    return collected


# ── Axis A: building a description must not touch the filesystem ────────────


@pytest.mark.parametrize("filename", LAUNCH_FILES)
def test_building_a_description_creates_and_deletes_nothing(filename, isolated_logging_root):
    """The regression #402 exists for: this failed for both robot launches.

    Fourteen seeded sessions is the number the reporting session actually had
    when #397's evaluation sensor nearly truncated them.
    """
    before = _seed_sessions(isolated_logging_root, 14)

    _load(filename).generate_launch_description()

    assert _sessions_in(isolated_logging_root) == sorted(before), (
        f"{filename} mutated the session tree while merely building its "
        "description; the session must be opened from a launch action"
    )


# ── AC #3: every consumer receives the same session path ────────────────────


@pytest.mark.parametrize("filename", LAUNCH_FILES)
def test_the_session_path_reaches_every_consumer(filename, isolated_logging_root):
    """One session per run, and all four consumers agree on it.

    "Real-robot behaviour is unchanged" is operationally this: ``$RTC_SESSION_DIR``,
    the RT node's ``log_dir``, and (for the robot launches) the generated
    controller_manager RT parameter file all name the same ``YYMMDD_HHMM``
    directory, and exactly one is created.
    """
    description = _load(filename).generate_launch_description()
    context, produced = _evaluate_in_order(description)

    sessions = _sessions_in(isolated_logging_root)
    assert len(sessions) == 1, f"expected one session per run, got {sessions}"
    session_dir = os.path.join(isolated_logging_root, sessions[0])

    assert context.environment["RTC_SESSION_DIR"] == session_dir
    assert context.environment["RTC_RUN_ID"].isdigit()

    parameters = _resolved_rt_controller_parameters(description, context, produced)
    assert parameters["log_dir"] == session_dir, (
        "the RT node would log outside this run's session directory"
    )

    if filename in ROBOT_LAUNCH_FILES:
        # Written by _launch_setup, which reads the session path from the
        # context rather than a build-time kwarg since #402.
        assert os.path.isfile(os.path.join(session_dir, CM_RT_PARAM_FILENAME)), (
            "the controller_manager RT parameter file left this run's session"
        )


@pytest.mark.parametrize("filename", LAUNCH_FILES)
def test_the_trace_output_stays_inside_the_session(filename, isolated_logging_root):
    """The fourth consumer, which only exists when tracing is on."""
    pytest.importorskip(
        "tracetools_launch.action",
        reason="ros2_tracing is not installed; make_trace_action declines and "
        "there is no Trace action whose base_path could be checked",
    )
    description = _load(filename).generate_launch_description()
    context, produced = _evaluate_in_order(description, {"enable_tracing": "true"})

    session_dir = os.path.join(isolated_logging_root, _sessions_in(isolated_logging_root)[0])
    traces = [action for action in produced if type(action).__name__ == "Trace"]
    assert traces, f"{filename} produced no Trace action with enable_tracing:=true"
    for trace in traces:
        # Trace normalises base_path to substitutions; resolve as launch does.
        assert perform_substitutions(context, trace.base_path) == os.path.join(
            session_dir, "tracing"
        )


# ── Axis B: one knob, both actors ───────────────────────────────────────────


@pytest.mark.parametrize("filename", LAUNCH_FILES)
def test_retention_default_comes_from_the_node_yaml(filename, isolated_logging_root):
    """The launch default must equal what the RT node's own YAML declares.

    Not a source-text check: the argument default is resolved, the description
    evaluated, and the node's parameter read back. If the two ever diverge the
    effective retention silently becomes ``min()`` of them again.

    This one cannot stand alone, and measurably so: deleting the node's
    ``max_log_sessions`` override leaves this test green, because on the default
    path the argument and the YAML carry the same 10 and nothing distinguishes
    "the launch passed it on" from "the node read its own YAML". The override
    test below is what actually pins the wiring — keep both.
    """
    description = _load(filename).generate_launch_description()
    context, produced = _evaluate_in_order(description)

    declared = context.launch_configurations["max_log_sessions"]
    parameters = _resolved_rt_controller_parameters(description, context, produced)
    assert parameters["max_log_sessions"] == int(declared)


@pytest.mark.parametrize("filename", LAUNCH_FILES)
def test_an_override_moves_both_the_launch_and_the_node(filename, isolated_logging_root):
    """``max_log_sessions:=3`` must prune to 3 *and* reach the node as 3.

    Before #402 the robot launches hardcoded 10 and ignored the argument
    entirely, while the sim launches honoured it on their own side only — so
    raising it above the YAML value did nothing on either.
    """
    _seed_sessions(isolated_logging_root, 6)
    description = _load(filename).generate_launch_description()
    context, produced = _evaluate_in_order(description, {"max_log_sessions": "3"})

    assert len(_sessions_in(isolated_logging_root)) == 3, "the launch ignored the override"

    parameters = _resolved_rt_controller_parameters(description, context, produced)
    assert parameters["max_log_sessions"] == 3, (
        "the node would prune with its YAML value instead, restoring min()"
    )
