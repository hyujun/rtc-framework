"""The run's inputs, read once and strictly — rclpy-free.

The GUI's loaders are lenient on purpose: a malformed step is dropped, a time
outside its range is clamped, a missing preset file becomes the defaults and
is written back. An unattended run cannot afford any of that — a sequence that
silently lost a step or plays 600 s where 6000 was written is found out hours
later — so this module refuses instead, and writes nothing:

- the preset file and the sequence file must both exist and parse,
- a preset tagged for the other hand refuses the run (the GUI moves such a
  file aside; here the operator decides),
- every step, its time, its dwell and the file's tolerance are checked against
  the ranges the GUI would clamp to, and a value outside one is an error,
- the steps resolve to targets all or nothing (``sequence.resolve_steps``).
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass

from ..demo_gui.sequence import (
    DEFAULT_TOLERANCE_DEG,
    MAX_TIME_S,
    MAX_TOLERANCE_DEG,
    MIN_TIME_S,
    MIN_TOLERANCE_DEG,
    ResolvedStep,
    SequenceStep,
    resolve_steps,
)

PRESET_SIDE_KEY = "side"  # config.PRESET_SIDE_KEY (config.py imports rclpy)


class FileProblem(Exception):
    """An input file that refuses the run; the message names the file."""


@dataclass(frozen=True)
class RunInputs:
    steps: tuple[ResolvedStep, ...]
    tolerance_deg: float
    preset_path: str
    sequence_path: str
    # Presets the run uses that carry no side tag: shown at the confirmation,
    # since nothing proves they were made for this hand.
    untagged: tuple[str, ...]


def preset_path(logging_root: str, token: str) -> str:
    return os.path.join(logging_root, f"hand_presets_{token}.json")


def sequence_path(logging_root: str, token: str) -> str:
    return os.path.join(logging_root, f"demo_gui_sequences_{token}.json")


def _read_json(path: str, what: str) -> object:
    try:
        with open(path, encoding="utf-8") as handle:
            return json.load(handle)
    except FileNotFoundError:
        raise FileProblem(f"{what} {path} does not exist") from None
    except (OSError, UnicodeDecodeError) as exc:
        raise FileProblem(f"{what} {path} cannot be read: {exc}") from None
    except json.JSONDecodeError as exc:
        raise FileProblem(f"{what} {path} is not JSON: {exc}") from None


def _number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value) if math.isfinite(value) else None


def parse_presets(data: object, path: str, side: str) -> dict:
    """The preset roster as stored; refused when it is not a name → object map
    or when a preset is tagged for another hand than ``side``."""
    if not isinstance(data, dict) or not all(isinstance(v, dict) for v in data.values()):
        raise FileProblem(f"preset file {path} is not a map of preset objects")
    other = sorted(
        name for name, p in data.items() if p.get(PRESET_SIDE_KEY, side) != side and side
    )
    if other:
        raise FileProblem(
            f"preset file {path} holds presets tagged for another hand than '{side}': {other}"
        )
    return data


def parse_sequence(data: object, path: str) -> tuple[tuple[SequenceStep, ...], float]:
    """(steps, tolerance [deg]); every value the GUI would clamp is refused."""
    if not isinstance(data, dict) or not isinstance(data.get("steps"), list):
        raise FileProblem(f"sequence file {path} has no 'steps' list")
    steps = []
    for number, raw in enumerate(data["steps"], start=1):
        where = f"sequence file {path} step {number}"
        if not isinstance(raw, dict):
            raise FileProblem(f"{where} is not an object")
        preset = raw.get("preset")
        if not isinstance(preset, str) or not preset:
            raise FileProblem(f"{where} names no preset")
        time_s = _number(raw.get("time_s"))
        if time_s is None or not MIN_TIME_S <= time_s <= MAX_TIME_S:
            raise FileProblem(
                f"{where} '{preset}': time_s {raw.get('time_s')!r} is outside "
                f"[{MIN_TIME_S}, {MAX_TIME_S}] s"
            )
        dwell_s = _number(raw.get("dwell_s", 0.0))
        if dwell_s is None or not 0.0 <= dwell_s <= MAX_TIME_S:
            raise FileProblem(
                f"{where} '{preset}': dwell_s {raw.get('dwell_s')!r} is outside [0, {MAX_TIME_S}] s"
            )
        steps.append(SequenceStep(preset, time_s, dwell_s))
    if not steps:
        raise FileProblem(f"sequence file {path} has no steps")
    raw_tol = data.get("tolerance_deg", DEFAULT_TOLERANCE_DEG)
    tolerance = _number(raw_tol)
    if tolerance is None or not MIN_TOLERANCE_DEG <= tolerance <= MAX_TOLERANCE_DEG:
        raise FileProblem(
            f"sequence file {path}: tolerance_deg {raw_tol!r} is outside "
            f"[{MIN_TOLERANCE_DEG}, {MAX_TOLERANCE_DEG}] deg"
        )
    return tuple(steps), tolerance


def load_run_inputs(logging_root: str, token: str, side: str, hand_dof: int) -> RunInputs:
    """Read both files for profile ``token`` and resolve the run; FileProblem on
    the first thing that refuses it. Nothing is created or written."""
    p_path, s_path = preset_path(logging_root, token), sequence_path(logging_root, token)
    presets = parse_presets(_read_json(p_path, "preset file"), p_path, side)
    steps, tolerance = parse_sequence(_read_json(s_path, "sequence file"), s_path)
    resolved, problem = resolve_steps(steps, presets, hand_dof)
    if problem:
        raise FileProblem(f"sequence file {s_path}: {problem} (presets: {p_path})")
    used = dict.fromkeys(step.preset for step in steps)
    untagged = tuple(name for name in used if PRESET_SIDE_KEY not in presets[name])
    return RunInputs(resolved, tolerance, p_path, s_path, untagged)
