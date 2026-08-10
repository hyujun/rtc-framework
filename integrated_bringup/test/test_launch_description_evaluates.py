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
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
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

    Still load-bearing after #402, for a narrower reason than before. Building
    a description is now side-effect free in all five launches: the robot ones
    no longer call ``create_session_dir()`` / ``cleanup_old_sessions()`` from
    ``generate_launch_description()`` itself. But every launch still opens its
    session from inside an ``OpaqueFunction``, and *executing those* is exactly
    what this file does — so an unisolated run would still create a session
    directory and delete every session past the configured count. What changed
    is that the exposure now needs `_evaluate`; the sibling
    ``test_combo_keys_are_declared_arguments``, which only builds descriptions,
    no longer has it.

    ``test_launch_session_wiring.py`` owns the assertion that building stays
    clean; this fixture only keeps the evaluation from reaching real data.

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


def _on_exit_entities(handler: OnProcessExit) -> list:
    """Actions an ``OnProcessExit`` would run, without firing the event."""
    return list(handler.describe()[1] or [])


def _on_exit_target(handler: OnProcessExit):
    """The action this handler waits for.

    ``launch`` keeps it in a name-mangled slot and exposes no accessor. Reading
    it is a deliberate coupling to launch's internals: the alternative is
    fabricating a ``ProcessExited`` event just to ask "does this match?", which
    couples harder. If a launch upgrade renames the slot this raises instead of
    silently reporting "nothing waits for the shield" — a false green here is
    exactly the #405 regression this file is meant to catch.
    """
    try:
        return handler._OnActionEventBase__action_matcher
    except AttributeError as exc:  # pragma: no cover - launch API change
        raise AssertionError(
            "launch's OnProcessExit no longer stores its target in "
            "_OnActionEventBase__action_matcher; this sensor must be reworked "
            "rather than deleted (issue #405)"
        ) from exc


def _shield_actions(entities) -> list:
    """The cset-shield ``ExecuteProcess`` among ``entities``.

    Matched on the ``$0`` the helper passes its inline script
    (``rtc_cpu_shield``), which is the one token in that command line that
    exists to name it.
    """

    def _text(token) -> str:
        # ExecuteProcess normalises cmd into List[List[Substitution]], so a
        # plain str never survives to here — the literal lives in a
        # TextSubstitution's .text. str() on the list yields repr addresses and
        # would silently match nothing, which is how this helper first shipped.
        if isinstance(token, (list, tuple)):
            return "".join(_text(part) for part in token)
        return str(getattr(token, "text", token))

    found = []
    for entity in entities:
        if not isinstance(entity, ExecuteProcess):
            continue
        if any("rtc_cpu_shield" in _text(token) for token in entity.cmd or []):
            found.append(entity)
    return found


def _walk(entities, context) -> list:
    """Flatten ``entities``, running OpaqueFunctions and descending into handlers.

    Descending is load-bearing, not tidiness. #405 moved the robot launches'
    ``_launch_setup`` OpaqueFunction inside an ``OnProcessExit``; a scan that
    only looks at ``description.entities`` would stop evaluating it and this
    file would keep passing while covering strictly less than #397 built it to
    cover. The sensor follows the actions wherever they are declared.
    """
    out: list = []
    for entity in entities:
        out.append(entity)
        if isinstance(entity, OpaqueFunction):
            out.extend(_walk(entity.execute(context) or [], context))
        elif isinstance(entity, RegisterEventHandler) and isinstance(
            entity.event_handler, OnProcessExit
        ):
            out.extend(_walk(_on_exit_entities(entity.event_handler), context))
    return out


def _evaluate(filename: str, overrides: dict[str, str]) -> list:
    """Build the description and run its OpaqueFunctions; return their actions.

    The returned actions are *not* executed — that is where processes would be
    spawned. Everything #397 belongs to happens before that line.
    """
    description = _load_launch_module(filename).generate_launch_description()
    assert isinstance(description, LaunchDescription)
    context = _seeded_context(description, overrides)

    opaque = [e for e in _walk(description.entities, context) if isinstance(e, OpaqueFunction)]
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


# ── Shield-first ordering (issue #405) ───────────────────────────────────────
# The launches already listed `enable_cpu_shield` above the drivers, and that
# bought nothing: it is an ExecuteProcess, so declaration order is not
# completion order. `cset` spent seconds moving hundreds of tasks while
# ros2_control_node was already up, and attaching that task to the "system"
# cpuset overwrote the cpu_affinity its 500 Hz loop had applied — measured on
# NUC13 tier 12 as `0-1,8-15` where the layout says `8`.
#
# No value-axis test can see this. The slot tables were right, the resolved
# logical cpu was right (`ros2 param get /controller_manager cpu_affinity` → 8),
# the launch logged the right number. Only the *shape* of the description is
# wrong, so the shape is what this asserts.
@pytest.mark.parametrize(("filename", "combo_id", "overrides"), CASES, ids=CASE_IDS)
def test_pinning_starts_only_after_the_cpu_shield(filename, combo_id, overrides):
    description = _load_launch_module(filename).generate_launch_description()
    context = _seeded_context(description, overrides)

    # Top level only — deliberately NOT _walk: the question is what sits beside
    # the shield, and descending would hide exactly that.
    top: list = []
    for entity in description.entities:
        top.append(entity)
        if isinstance(entity, OpaqueFunction):
            top.extend(entity.execute(context) or [])

    shields = _shield_actions(top)

    if not shields:
        # sim + use_cpu_affinity:=false: the action is never built, so there is
        # no exit event to wait for and the nodes are top-level by design. The
        # robot launches must NOT reach here — they pass gated=False precisely
        # so the chain's target always exists (#405).
        assert filename in SIM_LAUNCH_FILES and overrides.get("use_cpu_affinity") == "false", (
            f"{filename} [{combo_id}] builds no cpu-shield action. A robot launch "
            "that skips it has re-gated the action with IfCondition, which "
            "suppresses the OnProcessExit event and with it every node below."
        )
        return

    assert len(shields) == 1, f"{filename} [{combo_id}] builds {len(shields)} shield actions"
    shield = shields[0]

    waiters = [
        entity
        for entity in top
        if isinstance(entity, RegisterEventHandler)
        and isinstance(entity.event_handler, OnProcessExit)
        and _on_exit_target(entity.event_handler) is shield
    ]
    assert waiters, (
        f"{filename} [{combo_id}] starts the cpu shield but nothing waits for its "
        "exit — the drivers race cset and lose their pin (issue #405)"
    )

    waited = [e for handler in waiters for e in _on_exit_entities(handler.event_handler)]
    assert waited, f"{filename} [{combo_id}] waits for the shield but runs nothing afterwards"

    # The chain's target may not carry an IfCondition. A false condition does not
    # make the action a no-op — it makes it never start, so ProcessExited never
    # fires and every entity in on_exit is dropped. The launch would come up
    # empty on `use_cpu_affinity:=false`, which is why the robot launches pass
    # gated=False and hand the flag to the script as argv instead (#405).
    assert shield.condition is None, (
        f"{filename} [{combo_id}] gates the cpu-shield action with a condition "
        "while other actions wait for its exit — when that condition is false "
        "the exit event never arrives and the launch starts nothing at all. "
        "Use gated=False + pin_enabled=... so the script no-ops with exit 0."
    )

    # Nothing that spawns or pins may sit beside the shield. OpaqueFunctions are
    # named because two of them legitimately stay top-level: the session dir has
    # to exist before anything reads it (#402), and tracing is opt-in and
    # unpinned.
    # ``launch_setup`` is the sim launches' single top-level OpaqueFunction —
    # the container whose *output* the scan above already expanded, not an
    # action that starts anything. The robot launches' UR-driver body is
    # ``_launch_setup`` (leading underscore, a different function) and is NOT
    # exempt: it is exactly what has to move behind the shield. If either is
    # ever renamed this list stops matching and the test fails loudly.
    top_level_ok = {"launch_setup", "_open_session", "_build_trace_action"}
    beside = []
    for entity in top:
        if entity is shield or isinstance(entity, (DeclareLaunchArgument, RegisterEventHandler)):
            continue
        if isinstance(entity, OpaqueFunction):
            name = getattr(entity._OpaqueFunction__function, "__name__", "?")
            if name not in top_level_ok:
                beside.append(name)
            continue
        if type(entity).__name__ in (
            "Node",
            "LifecycleNode",
            "IncludeLaunchDescription",
            "TimerAction",
            "ExecuteProcess",
        ):
            beside.append(type(entity).__name__)
    assert not beside, (
        f"{filename} [{combo_id}] starts {beside} beside the shield instead of "
        "after it. Declaration order is not completion order — see issue #405."
    )
