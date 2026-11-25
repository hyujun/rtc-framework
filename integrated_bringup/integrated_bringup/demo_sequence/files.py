"""The run's inputs — the robot's shipped postures, read once and strictly; rclpy-free.

The run plays what git ships for the robot, ``config/<robot>/demo_gui/`` of the
installed package (``hand_presets.json`` + ``sequence.json``), not the GUI's
working copy in the logging root: a field PC that pulled and built plays
exactly what was committed, whatever its own logging_data holds. The shipped
files are checked as a release (``demo_gui.shipped``), and on top of that:

- the GUI's loaders are lenient on purpose (a malformed step is dropped, a
  time outside its range is clamped); an unattended run cannot afford that —
  a sequence that silently lost a step or plays 600 s where 6000 was written
  is found out hours later — so every step, its time, its dwell and the
  file's tolerance are checked against the ranges the GUI would clamp to, and
  a value outside one is an error,
- the steps resolve to targets all or nothing (``sequence.resolve_steps``),
- nothing is created or written.

What the run names them by: the posture code (``shipped.posture_code``), the
files' real paths, and the git commit they came from when git can tell.
"""

from __future__ import annotations

import math
import os
import subprocess
from dataclasses import dataclass
from typing import TYPE_CHECKING

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

if TYPE_CHECKING:  # demo_gui.shipped imports this module
    from ..demo_gui.shipped import Calibration

PRESET_SIDE_KEY = "side"  # config.PRESET_SIDE_KEY (config.py imports rclpy)


class FileProblem(Exception):
    """An input file that refuses the run; the message names the file."""


@dataclass(frozen=True)
class RunInputs:
    steps: tuple[ResolvedStep, ...]
    tolerance_deg: float
    preset_path: str  # real paths: what was read, not the share's link
    sequence_path: str
    posture_code: str = ""  # shipped.posture_code of the two files
    # Last commit of the two files ("<hash> <date>", "+ local changes" when
    # the bytes are not that commit's); "" when git cannot tell.
    provenance: str = ""
    # The calibration the postures were recorded under (always shipped; None
    # only in tests that build RunInputs by hand: not checked).
    calibration: Calibration | None = None


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


BUILD_HINT = "after git pull, build the workspace (build.sh) so the install has the files"
GIT_TIMEOUT_S = 2.0


def load_run_inputs(config_dir: str, robot: str) -> RunInputs:
    """The shipped postures of ``robot`` under ``config_dir`` (the installed
    package's ``config``), resolved; FileProblem on the first thing that
    refuses the run. Nothing is created or written."""
    # demo_gui.shipped imports this module: import it here, not at the top.
    from ..demo_gui import shipped

    directory = shipped.shipped_dir(config_dir, robot)
    p_path = os.path.join(directory, shipped.PRESETS_FILE)
    s_path = os.path.join(directory, shipped.SEQUENCE_FILE)
    cal_path = os.path.join(directory, shipped.CALIBRATION_FILE)
    # The record too: promote never ships without one, and a run that cannot
    # compare the driver against it must not start on hardware.
    for path in (p_path, s_path, cal_path):
        if not os.path.exists(path):
            raise FileProblem(f"{path} does not exist — {BUILD_HINT}")
    prof = shipped.ShippedProfile.for_robot(robot)
    presets, sequence = shipped.load_shipped(config_dir, prof)
    steps, tolerance = parse_sequence(sequence, s_path)
    # load_shipped has resolved the steps already (validate_sequence).
    resolved, _ = resolve_steps(steps, presets, len(prof.hand_joint_names))
    calibration = shipped.read_calibration(cal_path)
    real = (os.path.realpath(p_path), os.path.realpath(s_path))
    return RunInputs(
        resolved,
        tolerance,
        *real,
        posture_code=shipped.posture_code(presets, sequence),
        provenance=provenance(real),
        calibration=calibration,
    )


def _git(cwd: str, *args: str) -> str:
    """stdout of ``git args`` in ``cwd``, stripped; OSError / CalledProcessError
    / TimeoutExpired when git cannot answer."""
    done = subprocess.run(
        ["git", *args],
        cwd=cwd,
        capture_output=True,
        text=True,
        timeout=GIT_TIMEOUT_S,
        check=True,
    )
    return done.stdout.strip()


def provenance(paths: tuple[str, ...]) -> str:
    """ "<hash> <date>" of the last commit that touched ``paths`` (real paths in
    one git checkout), with " + local changes" when a file's bytes are not that
    HEAD's; "" on any failure — an install that copied the files, no git, a
    file git does not track."""
    try:
        cwd = os.path.dirname(paths[0])
        top = _git(cwd, "rev-parse", "--show-toplevel")
        rel = [os.path.relpath(p, top) for p in paths]
        last = _git(top, "log", "-1", "--format=%h %cs", "--", *rel)
        if not last:
            return ""
        changed = False
        for path, name in zip(paths, rel, strict=True):
            try:
                committed = _git(top, "rev-parse", f"HEAD:{name}")
            except subprocess.CalledProcessError:
                changed = True  # not in HEAD: a file promote has just written
                continue
            changed = changed or _git(top, "hash-object", "--", path) != committed
    except (OSError, ValueError, subprocess.SubprocessError):
        return ""
    return last + (" + local changes" if changed else "")
