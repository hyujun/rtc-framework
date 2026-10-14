"""Resolve the ``catch_search_mode:=`` / ``catch_segment_mode:=`` launch arguments.

Shared by the per-profile sim launch files that run catching trials (#821). An
arm of the throw population is the shipped controller configuration with two
keys changed and nothing else — ``catching.planner.search.mode`` and
``catching.planner.segment.mode`` — so the two keys are launch arguments and no
overlay file is written per arm.

**The path these overrides take creates whatever key it is given.** The RT node
hands every ``demo_catching_controller.*`` parameter to
``ApplyControllerParamOverrides``, which pokes it into the YAML the controller
loads: a key one level off is written, never read, and the run is the shipped
pair — the configure logs a ``[param-override]`` warning and goes on. So the
keys are spelled once, here (``test_launch_description_evaluates.py`` holds
them against the shipped controller YAML), a value that is
not a mode is refused before a node starts, and the caller still reads the
controller's mirror back (``planner.search.mode`` / ``planner.segment.mode``)
before it throws — ``catch_population_trials`` does.
"""

from __future__ import annotations

CONTROLLER = "demo_catching_controller"
SEARCH_MODES = ("grid", "nlp")
SEGMENT_MODES = ("closed_form", "mpc", "mpc_docking")
SEARCH_ARGUMENT = "catch_search_mode"
SEGMENT_ARGUMENT = "catch_segment_mode"
SEARCH_KEY = f"{CONTROLLER}.catching.planner.search.mode"
SEGMENT_KEY = f"{CONTROLLER}.catching.planner.segment.mode"


def catch_search_mode_argument_description() -> str:
    return (
        f"Override {CONTROLLER}'s catching.planner.search.mode "
        f"({' | '.join(SEARCH_MODES)}). Empty = the shipped value. Any other value is refused."
    )


def catch_segment_mode_argument_description() -> str:
    return (
        f"Override {CONTROLLER}'s catching.planner.segment.mode "
        f"({' | '.join(SEGMENT_MODES)}). Empty = the shipped value. Any other value is refused."
    )


def _mode(value: str, argument: str, modes: tuple[str, ...]) -> str | None:
    value = value.strip()
    if value == "":
        return None
    if value not in modes:
        raise RuntimeError(
            f"{argument} must be one of {', '.join(modes)} (or empty), got {value!r}"
        )
    return value


def catch_mode_overrides(search: str, segment: str) -> dict:
    """The two launch arguments → the RT node's controller overrides, or {}.

    An empty argument writes nothing: the controller runs the shipped value of
    that key. A value that is not one of the modes RAISES — the override path
    would hand the controller a mode it parks on, or (misspelled the other way,
    in the key) one it never reads.
    """
    overrides: dict = {}
    for value, argument, modes, key in (
        (search, SEARCH_ARGUMENT, SEARCH_MODES, SEARCH_KEY),
        (segment, SEGMENT_ARGUMENT, SEGMENT_MODES, SEGMENT_KEY),
    ):
        mode = _mode(value, argument, modes)
        if mode is not None:
            overrides[key] = mode
    return overrides
