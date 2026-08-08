"""Every bringup launch description actually builds, arguments and all (#397).

The gap this closes
-------------------
``rtc_tools/test/test_launch_imports.py`` *imports* the launch modules, and the
three wiring tests next to this one read their **AST**. Neither runs a line of
``launch_setup``: everything a bringup launch actually computes lives inside an
``OpaqueFunction`` body, which only executes when a launch description is
evaluated. So when ``layout_profile`` ended up used fifteen lines above its
assignment, all three sim launches died at ``ros2 launch`` — *"cannot access
local variable 'layout_profile' where it is not associated with a value"* — with
the whole suite green. The repo had no sensor that asked "does this launch even
start?" (#397, regression from #350).

Why this can be a plain unit test
---------------------------------
``test_launch_shield_wiring`` and ``test_launch_hand_affinity_wiring`` both used
to say a behavioural check "would need a running ROS graph". That is not true: a
bare ``LaunchContext`` seeded with the declared defaults evaluates every
``OpaqueFunction`` in all five launches with no ROS graph, no daemon, and no
node — because the actions are *built*, never executed. Roughly 1 s for the
whole matrix.

Evaluating is not free, though — see ``_isolated_logging_root``.

What is asserted, and what deliberately is not
----------------------------------------------
Asserted: the description builds, every ``OpaqueFunction`` runs without raising,
and the result is not vacuous (an ``OpaqueFunction`` exists and actions come
back). Not asserted: how *many* actions, or which. Both vary legitimately with
the host (``get_sim_core()`` returns -1 on small tiers, dropping a
``TimerAction``) and with the arguments (``use_cpu_affinity:=false`` drops two),
and "which helper is called" is what the AST tests already own. Pinning counts
here would buy nothing and break on every honest change.
"""

from __future__ import annotations

import importlib.util
import os
from typing import Any

import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.utilities import perform_substitutions

from rtc_tools.utils.session_dir import resolve_logging_root

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

# Argument combinations, not just defaults. Every override argument declares an
# empty default and is applied only when non-empty, so a defaults-only run never
# enters those branches: measured with coverage.py, defaults alone reach 88% of
# the five files' statements and this matrix reaches 96%. The remainder is
# exception handlers and the ``mpc_engine`` reject below.
SIM_COMBOS: list[tuple[str, dict[str, str]]] = [
    ("defaults", {}),
    # The CI / no-sudo path: drops the cset shield and the pinning timers.
    ("no_affinity", {"use_cpu_affinity": "false"}),
    # The axis #350 added and #397 broke — MPC_OFF is the only value that is not
    # the fallback, so it is the one a mapping bug hides behind.
    ("mpc_off", {"enable_mpc": "false"}),
    (
        "all_overrides",
        {
            "enable_viewer": "false",
            "model_path": "/nonexistent/scene.xml",
            "sync_timeout_ms": "50.0",
            "max_rtf": "10.0",
            "kp": "10.0",
            "kd": "1.0",
            "use_yaml_servo_gains": "true",
            "initial_controller": "demo_wbc_controller",
            "mpc_engine": "handler",
            "enable_mpc": "true",
        },
    ),
]

ROBOT_COMBOS: list[tuple[str, dict[str, str]]] = [
    ("defaults", {}),
    ("no_affinity", {"use_cpu_affinity": "false"}),
    ("mpc_off", {"enable_mpc": "false"}),
    # mock hardware adds the controller-switch TimerAction inside _launch_setup.
    ("mock_hardware", {"use_mock_hardware": "true", "enable_mpc": "true"}),
]


def _combos(filename: str) -> list[tuple[str, dict[str, str]]]:
    return SIM_COMBOS if filename in SIM_LAUNCH_FILES else ROBOT_COMBOS


def _cases() -> list[tuple[str, str, dict[str, str]]]:
    return [
        (filename, combo_id, overrides)
        for filename in LAUNCH_FILES
        for combo_id, overrides in _combos(filename)
    ]


CASES = _cases()
CASE_IDS = [f"{filename}-{combo_id}" for filename, combo_id, _ in CASES]


@pytest.fixture(autouse=True)
def _isolated_logging_root(monkeypatch, tmp_path):
    """Point the session tree at ``tmp_path`` — and prove the redirect took.

    Building a launch description is **not** side-effect free. The robot
    launches call ``create_session_dir()`` and ``cleanup_old_sessions(root, 10)``
    from ``generate_launch_description()`` itself, so merely evaluating one
    creates a session directory and *deletes* every session past the tenth; the
    sim launches do the same one level down, inside ``launch_setup``. Run
    unisolated under ``colcon test`` and this file quietly truncates the
    developer's ``logging_data``.

    ``resolve_logging_root`` prefers ``$COLCON_PREFIX_PATH``'s first entry, but
    it falls back to walking the cwd for an ``install/`` + ``src/`` pair — which
    under ``colcon test`` finds the real workspace. Setting the variable to a
    writable tmp directory beats both, and the assertion below is what makes
    that a fact rather than an intention: a redirect that silently stopped
    working would otherwise show up as missing user data, never as a red test.
    """
    prefix = tmp_path / "install"
    prefix.mkdir()
    monkeypatch.setenv("COLCON_PREFIX_PATH", str(prefix))
    # ament_index reads AMENT_PREFIX_PATH, so the real share stays reachable.
    root = resolve_logging_root()
    assert root.startswith(str(tmp_path)), (
        f"logging root escaped the sandbox ({root}); evaluating these launches "
        f"would create and prune session directories under the real workspace"
    )
    return tmp_path


_MODULE_CACHE: dict[str, Any] = {}


def _load_launch_module(filename: str):
    """Load a launch file from the *installed* share, as ``ros2 launch`` does.

    Importing is side-effect free — every filesystem touch happens in
    ``generate_launch_description()`` — so the module is cached across cases
    while each case still resolves the logging root afresh.
    """
    if filename not in _MODULE_CACHE:
        path = os.path.join(get_package_share_directory("integrated_bringup"), "launch", filename)
        spec = importlib.util.spec_from_file_location(f"_eval_{filename}", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        _MODULE_CACHE[filename] = module
    return _MODULE_CACHE[filename]


def _declared_arguments(description: LaunchDescription) -> dict[str, DeclareLaunchArgument]:
    return {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }


def _seeded_context(description: LaunchDescription, overrides: dict[str, str]) -> LaunchContext:
    """A context carrying every declared default, then the case's overrides.

    This is what ``ros2 launch`` does before it visits the entities; without it
    the first ``LaunchConfiguration(...).perform(context)`` raises on a lookup
    miss and every case would fail for a reason that has nothing to do with the
    launch file.
    """
    context = LaunchContext()
    for name, argument in _declared_arguments(description).items():
        context.launch_configurations[name] = perform_substitutions(
            context, argument.default_value
        )
    context.launch_configurations.update(overrides)
    return context


def _evaluate(filename: str, overrides: dict[str, str]) -> list:
    """Build the description and run its OpaqueFunctions; return their actions.

    The returned actions are *not* executed — that is where processes would be
    spawned. Everything #397 belongs to happens before that line.
    """
    description = _load_launch_module(filename).generate_launch_description()
    assert isinstance(description, LaunchDescription)
    context = _seeded_context(description, overrides)

    opaque = [e for e in description.entities if isinstance(e, OpaqueFunction)]
    assert opaque, (
        f"{filename} declares no OpaqueFunction — either the launch was "
        "restructured (and this sensor no longer evaluates anything) or it "
        "regressed to a static description"
    )

    actions: list = []
    for function in opaque:
        actions.extend(function.execute(context) or [])
    return actions


@pytest.mark.parametrize(("filename", "combo_id", "overrides"), CASES, ids=CASE_IDS)
def test_launch_description_evaluates(filename, combo_id, overrides):
    """The whole point: evaluating must not raise.

    #397 was ``UnboundLocalError`` on an unconditional line, so it fails every
    combination. A conditional one — a bad float conversion behind
    ``max_rtf:=``, an override branch that references a name the default path
    never builds — only fails its own, which is why the matrix exists.
    """
    actions = _evaluate(filename, overrides)
    assert actions, (
        f"{filename} [{combo_id}] evaluated to no actions at all; the sensor "
        "would pass on an empty launch"
    )


@pytest.mark.parametrize(("filename", "combo_id", "overrides"), CASES, ids=CASE_IDS)
def test_combo_keys_are_declared_arguments(filename, combo_id, overrides):
    """Each override must name an argument the launch actually declares.

    Without this, renaming an argument turns its combo into a no-op: the key
    lands in ``launch_configurations`` where nothing reads it, the evaluation
    still passes, and the branch the combo existed to cover stops being covered
    — silently, and precisely when a rename makes that branch most worth
    checking.
    """
    declared = _declared_arguments(_load_launch_module(filename).generate_launch_description())
    unknown = sorted(set(overrides) - set(declared))
    assert not unknown, (
        f"{filename} [{combo_id}] overrides undeclared argument(s) {unknown}; "
        f"declared: {sorted(declared)}"
    )


@pytest.mark.parametrize("filename", SIM_LAUNCH_FILES)
def test_invalid_mpc_engine_is_rejected(filename):
    """Positive control: prove the OpaqueFunction body really runs.

    Every other assertion here is "nothing raised", which a sensor that quietly
    stopped evaluating would also satisfy. This one demands a *specific*
    exception from a line deep inside ``launch_setup``, so a sensor that has
    gone inert fails here instead of passing everywhere.

    It cannot mask a #397-shaped regression either: the ``layout_profile`` use
    sits *above* this validation, so an unbound name raises ``UnboundLocalError``
    first and this test goes red rather than swallowing it as the expected
    ``RuntimeError``.
    """
    with pytest.raises(RuntimeError, match="Invalid mpc_engine"):
        _evaluate(filename, {"mpc_engine": "definitely-not-an-engine"})
