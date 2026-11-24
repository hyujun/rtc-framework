"""Shipped hand postures of demo_controller_gui — rclpy- and Tk-free.

A robot may ship, under ``config/<robot>/demo_gui/`` of the package, the hand
postures validated on its hardware:

- ``hand_presets.json`` — the preset table, in the GUI's own file format,
- ``sequence.json`` — optional, the Sequence tab's list over those presets,
- ``calibration.yaml`` — the driver calibration (joint direction signs and
  position offsets) the postures were recorded under: a preset is a joint
  position AFTER the driver's offset, so the same numbers point elsewhere
  under another calibration.

The GUI runs on its working copy in the logging root
(``hand_presets_<token>.json`` · ``demo_gui_sequences_<token>.json``) and
seeds it from the shipped copy (``choose``); ``promote_main`` writes a working
copy back into the source tree, so a change becomes a commit. Operator guide:
docs/demo-controller-gui.md §Shipped postures.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

import yaml

from ..demo_sequence.files import FileProblem, parse_presets, parse_sequence
from .preset_speed import MIN_GRASP_TIME
from .sequence import resolve_steps

SHIPPED_DIR = "demo_gui"
PRESETS_FILE = "hand_presets.json"
SEQUENCE_FILE = "sequence.json"
CALIBRATION_FILE = "calibration.yaml"
# Beside a working file: the content hash of the shipped copy it was seeded
# from. Equal to the working file's own hash = the operator has not edited it.
SEED_SUFFIX = ".seed"

PRESET_SIDE_KEY = "side"  # config.PRESET_SIDE_KEY (config.py imports rclpy)
PRESET_TYPES = ("open", "close")
ROBOT_GOAL_TYPES = {"joint": None, "task": 6}  # robot_target length; None = arm DoF
# A position within half the GUI's 0.01 deg storage step of a limit is on it:
# a limit read from radians is not a round number of degrees.
LIMIT_TOLERANCE_DEG = 0.005
CALIBRATION_KEYS = ("joint_direction_signs", "joint_position_offsets_deg")


def shipped_dir(config_dir: str, robot: str) -> str:
    """``<config_dir>/<robot>/demo_gui`` — ``config_dir`` is the package's
    ``config`` (installed share or source tree)."""
    return os.path.join(config_dir, robot, SHIPPED_DIR)


def content_hash(data: object) -> str:
    """sha256 of ``data`` as canonical JSON: key order and indentation do not
    count, so a file the GUI rewrites unchanged keeps its hash."""
    text = json.dumps(data, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value) if math.isfinite(value) else None


def read_json(path: str, what: str) -> object:
    """The parsed file; FileProblem naming ``path`` when it is missing or unreadable."""
    try:
        with open(path, encoding="utf-8") as handle:
            return json.load(handle)
    except FileNotFoundError:
        raise FileProblem(f"{what} {path} does not exist") from None
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise FileProblem(f"{what} {path} cannot be read: {exc}") from None


def _ros_parameters(path: str) -> dict:
    try:
        with open(path, encoding="utf-8") as handle:
            doc = yaml.safe_load(handle)
    except (OSError, yaml.YAMLError) as exc:
        raise FileProblem(f"{path} cannot be read: {exc}") from None
    block = doc.get("/**") if isinstance(doc, dict) else None
    params = block.get("ros__parameters") if isinstance(block, dict) else None
    if not isinstance(params, dict):
        raise FileProblem(f"{path} has no '/**: ros__parameters' block")
    return params


def hand_limits_deg(
    base_yaml: str, group: str, joint_names: Sequence[str]
) -> tuple[tuple[float, float], ...]:
    """(lower, upper) [deg] per joint of ``joint_names``, from the device
    group's ``joint_limits`` in a profile's ``_base.yaml`` (the YAML side of
    the limits; the CM may narrow them further to the URDF's)."""
    devices = _ros_parameters(base_yaml).get("devices")
    device = devices.get(group) if isinstance(devices, dict) else None
    if not isinstance(device, dict):
        raise FileProblem(f"{base_yaml} has no device group '{group}'")
    names = device.get("joint_state_names")
    limits = device.get("joint_limits")
    limits = limits if isinstance(limits, dict) else {}
    lower, upper = limits.get("position_lower"), limits.get("position_upper")
    if not (
        isinstance(names, list)
        and isinstance(lower, list)
        and isinstance(upper, list)
        and len(names) == len(lower) == len(upper)
    ):
        raise FileProblem(f"{base_yaml} '{group}': joint names and position limits do not match")
    if not all(isinstance(n, str) for n in names):
        raise FileProblem(f"{base_yaml} '{group}': a joint name is not a string")
    by_name = dict(zip(names, zip(lower, upper, strict=True), strict=True))
    missing = [n for n in joint_names if n not in by_name]
    if missing:
        raise FileProblem(f"{base_yaml} '{group}' has no limits for {missing}")
    out = []
    for name in joint_names:
        lo, hi = (_number(v) for v in by_name[name])
        if lo is None or hi is None or lo > hi:
            raise FileProblem(f"{base_yaml} '{group}' {name}: limits are not an interval")
        out.append((math.degrees(lo), math.degrees(hi)))
    return tuple(out)


def validate_presets(
    data: object,
    path: str,
    side: str,
    hand_dof: int,
    limits_deg: Sequence[tuple[float, float]] | None,
    arm_dof: int = 0,
) -> dict:
    """``data`` as a preset table the hand can play; FileProblem on the first
    preset that is not. Stricter than the GUI's own loading: a shipped table
    is a release, so on a profile that is one side of a pair every preset must
    carry that side's tag (an untagged one proves nothing about its hand)."""
    presets = parse_presets(data, path, side)
    if not presets:
        raise FileProblem(f"preset file {path} holds no preset")
    for name, preset in presets.items():
        where = f"preset file {path} '{name}'"
        if side and preset.get(PRESET_SIDE_KEY) != side:
            raise FileProblem(f"{where} carries no '{PRESET_SIDE_KEY}: {side}' tag")
        if preset.get("type") not in PRESET_TYPES:
            raise FileProblem(f"{where}: type {preset.get('type')!r} is not one of {PRESET_TYPES}")
        positions = preset.get("positions_deg")
        values = [_number(v) for v in positions] if isinstance(positions, list) else []
        if len(values) != hand_dof or None in values:
            raise FileProblem(f"{where}: positions_deg is not {hand_dof} finite numbers")
        for joint, (value, (lo, hi)) in enumerate(zip(values, limits_deg or (), strict=False)):
            if not lo - LIMIT_TOLERANCE_DEG <= value <= hi + LIMIT_TOLERANCE_DEG:
                raise FileProblem(
                    f"{where}: joint {joint} at {value} deg is outside [{lo:.2f}, {hi:.2f}] deg"
                )
        grasp_time = _number(preset.get("grasp_time"))
        if grasp_time is None or grasp_time < MIN_GRASP_TIME:
            raise FileProblem(
                f"{where}: grasp_time {preset.get('grasp_time')!r} is not >= {MIN_GRASP_TIME} s"
            )
        if "robot_target" in preset:
            _check_robot_fields(preset, where, arm_dof)
    return presets


def _check_robot_fields(preset: Mapping, where: str, arm_dof: int) -> None:
    goal_type = preset.get("robot_goal_type")
    if not isinstance(goal_type, str) or goal_type not in ROBOT_GOAL_TYPES:
        raise FileProblem(f"{where}: robot_goal_type {goal_type!r} is not joint or task")
    length = ROBOT_GOAL_TYPES[goal_type] or arm_dof
    target = preset.get("robot_target")
    values = [_number(v) for v in target] if isinstance(target, list) else []
    if arm_dof == 0 or len(values) != length or None in values:
        raise FileProblem(f"{where}: robot_target is not {length} finite numbers for this robot")
    if not isinstance(preset.get("controller"), str) or not preset["controller"]:
        raise FileProblem(f"{where}: a robot target names no controller")


def validate_sequence(data: object, path: str, presets: Mapping, hand_dof: int) -> None:
    """FileProblem unless every step of ``data`` plays against ``presets``."""
    steps, _ = parse_sequence(data, path)
    _, problem = resolve_steps(steps, presets, hand_dof)
    if problem:
        raise FileProblem(f"sequence file {path}: {problem}")


# ── The load rule ───────────────────────────────────────────────────────────

MISSING = "missing"
CORRUPT = "corrupt"  # not JSON, or not the file's shape
REFUSED = "refused"  # a preset tagged for the other hand
PRESENT = "present"

KEEP = "keep"
SEED = "seed"


@dataclass(frozen=True)
class Choice:
    """What to do with a working file, given its shipped copy.

    ``action`` SEED writes the shipped copy over the working one (after moving
    it aside when ``set_aside``) and its hash to the sidecar; KEEP runs on the
    working copy and writes the sidecar only when ``seed_hash`` is set.
    ``differs``: the kept copy is not the shipped one (the operator edited it).
    """

    action: str
    reason: str
    set_aside: bool = False
    seed_hash: str = ""
    differs: bool = False


def choose(
    state: str,
    working: object,
    seed_hash: str | None,
    shipped: object,
    stale: Sequence[object] = (),
) -> Choice:
    """The load rule of one working file (presets or sequence).

    - missing / corrupt / refused → seed from shipped (a corrupt or refused
      file is moved aside first, never overwritten);
    - equal to one of ``stale`` (a roster written before anything shipped) →
      seed;
    - equal to shipped → keep (and record the seed, so a later shipped change
      reaches it);
    - unedited since its seed (its hash is the sidecar's) while shipped has
      changed → seed the new shipped copy;
    - otherwise the operator edited it → keep, and say it differs.
    """
    shipped_hash = content_hash(shipped)
    if state == MISSING:
        return Choice(SEED, "no working copy", seed_hash=shipped_hash)
    if state in (CORRUPT, REFUSED):
        return Choice(SEED, f"working copy {state}", set_aside=True, seed_hash=shipped_hash)
    if any(working == s for s in stale):
        return Choice(SEED, "working copy is an unedited default roster", seed_hash=shipped_hash)
    working_hash = content_hash(working)
    if working_hash == shipped_hash:
        return Choice(
            KEEP, "same as shipped", seed_hash="" if seed_hash == shipped_hash else shipped_hash
        )
    if seed_hash == working_hash:
        return Choice(SEED, "shipped copy changed since the seed", seed_hash=shipped_hash)
    return Choice(KEEP, "edited — differs from shipped", differs=True)


def read_seed_hash(working_path: str) -> str | None:
    try:
        with open(working_path + SEED_SUFFIX, encoding="utf-8") as handle:
            return handle.read().strip() or None
    except OSError:
        return None


def write_seed_hash(working_path: str, digest: str) -> None:
    tmp = f"{working_path}{SEED_SUFFIX}.tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        handle.write(digest + "\n")
    os.replace(tmp, working_path + SEED_SUFFIX)


def read_working(path: str, side: str = "", sequence: bool = False) -> tuple[str, object]:
    """(state, data) of a working file as the load rule sees it."""
    try:
        with open(path, encoding="utf-8") as handle:
            data = json.load(handle)
    except FileNotFoundError:
        return MISSING, None
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        return CORRUPT, None
    if sequence:
        ok = isinstance(data, dict) and isinstance(data.get("steps"), list)
        return (PRESENT if ok else CORRUPT), data
    if not isinstance(data, dict) or not all(isinstance(v, dict) for v in data.values()):
        return CORRUPT, data
    if side and any(p.get(PRESET_SIDE_KEY, side) != side for p in data.values()):
        return REFUSED, data
    return PRESENT, data


# ── Calibration ─────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class Calibration:
    driver_config: str  # the driver YAML's file name beside _base.yaml
    joint_names: tuple[str, ...]
    joint_direction_signs: tuple[float, ...]
    joint_position_offsets_deg: tuple[float, ...]


def read_calibration(path: str) -> Calibration:
    try:
        with open(path, encoding="utf-8") as handle:
            doc = yaml.safe_load(handle)
    except (OSError, yaml.YAMLError) as exc:
        raise FileProblem(f"calibration {path} cannot be read: {exc}") from None
    if not isinstance(doc, dict) or not isinstance(doc.get("driver_config"), str):
        raise FileProblem(f"calibration {path} names no driver_config")
    names = doc.get("joint_names")
    lists = [doc.get(k) for k in CALIBRATION_KEYS]
    if not isinstance(names, list) or not all(
        isinstance(v, list) and len(v) == len(names) and None not in [_number(x) for x in v]
        for v in lists
    ):
        raise FileProblem(f"calibration {path}: joint_names, signs and offsets do not match")
    return Calibration(
        doc["driver_config"],
        tuple(names),
        *(tuple(float(x) for x in v) for v in lists),
    )


def driver_calibration(driver_yaml: str, side: str) -> Calibration:
    """The signs and offsets a driver YAML runs with — its ``<side>.`` keys
    when set, else the flat ones (the node's own rule); an absent key is the
    node's default (+1, 0)."""
    params = _ros_parameters(driver_yaml)
    per_hand = params.get(side, {}) if side else {}

    def key(name: str) -> object:
        value = per_hand.get(name) if isinstance(per_hand, dict) else None
        return value if value else params.get(name)

    names = key("joint_state_names")
    if not isinstance(names, list) or not names or not all(isinstance(n, str) for n in names):
        raise FileProblem(f"{driver_yaml} has no joint_state_names")

    def numbers(name: str, default: float) -> tuple[float, ...]:
        raw = key(name) or [default] * len(names)
        values = [_number(v) for v in raw] if isinstance(raw, list) else [None]
        if None in values:
            raise FileProblem(f"{driver_yaml}: {name} is not a list of numbers")
        return tuple(values)

    return Calibration(
        os.path.basename(driver_yaml),
        tuple(names),
        numbers("joint_direction_signs", 1.0),
        numbers("joint_position_offsets_deg", 0.0),
    )


def calibration_mismatch(recorded: Calibration, in_use: Calibration) -> str:
    """Why the driver in use is not the calibration the postures were
    recorded under ("" = it is)."""
    if recorded.joint_names != in_use.joint_names:
        return "the driver's joint order differs from the recorded one"
    diffs = []
    for field in CALIBRATION_KEYS:
        for name, a, b in zip(
            recorded.joint_names,
            getattr(recorded, field),
            getattr(in_use, field),
            strict=False,
        ):
            if not math.isclose(a, b, abs_tol=1e-9):
                diffs.append(f"{name} {field} {b:g} (recorded {a:g})")
        if len(getattr(recorded, field)) != len(getattr(in_use, field)):
            diffs.append(f"{field} has another length")
    return "; ".join(diffs)


def check_calibration(config_dir: str, robot: str, side: str) -> str:
    """The calibration warning for ``robot`` ("" = the driver YAML under
    ``config_dir`` is the one the shipped postures were recorded under, or
    nothing ships)."""
    path = os.path.join(shipped_dir(config_dir, robot), CALIBRATION_FILE)
    if not os.path.exists(path):
        return ""
    try:
        recorded = read_calibration(path)
        in_use = driver_calibration(os.path.join(config_dir, robot, recorded.driver_config), side)
    except FileProblem as exc:
        return str(exc)
    mismatch = calibration_mismatch(recorded, in_use)
    return (
        f"driver calibration is not the one the shipped postures were recorded under: {mismatch}"
        if mismatch
        else ""
    )


# ── Shipped copy as the GUI and the CLI read it ─────────────────────────────


@dataclass(frozen=True)
class ShippedProfile:
    """What validating a robot's shipped files needs of its profile."""

    robot: str
    side: str
    hand_group: str
    hand_joint_names: tuple[str, ...]
    arm_dof: int = 0

    @classmethod
    def for_robot(cls, robot: str) -> ShippedProfile:
        """The GUI's ``--robot`` profile of ``robot`` (KeyError if none)."""
        from .discovery import ROBOT_PROFILES

        prof = ROBOT_PROFILES[robot]
        return cls(
            robot,
            prof.side,
            prof.hand_group,
            tuple(prof.shape.hand_motor_names),
            prof.shape.arm_dof,
        )


def load_shipped(config_dir: str, prof: ShippedProfile) -> tuple[dict | None, dict | None]:
    """(presets, sequence) shipped for the robot, each None when it does not
    ship. FileProblem when a shipped file is there but unusable."""
    directory = shipped_dir(config_dir, prof.robot)
    p_path = os.path.join(directory, PRESETS_FILE)
    if not os.path.exists(p_path):
        return None, None
    limits = hand_limits_deg(
        os.path.join(config_dir, prof.robot, "_base.yaml"), prof.hand_group, prof.hand_joint_names
    )
    presets = validate_presets(
        read_json(p_path, "shipped preset file"),
        p_path,
        prof.side,
        len(prof.hand_joint_names),
        limits,
        prof.arm_dof,
    )
    s_path = os.path.join(directory, SEQUENCE_FILE)
    if not os.path.exists(s_path):
        return presets, None
    sequence = read_json(s_path, "shipped sequence file")
    validate_sequence(sequence, s_path, presets, len(prof.hand_joint_names))
    return presets, sequence


def working_notes(config_dir: str, robot: str, preset_path: str, sequence_path: str) -> list[str]:
    """What an unattended run should say about its working copy against the
    shipped one: the run plays the working copy either way (the CLI writes
    nothing), so these are notes, not refusals. Empty when nothing ships."""
    try:
        presets, sequence = load_shipped(config_dir, ShippedProfile.for_robot(robot))
    except (FileProblem, KeyError) as exc:
        return [f"the shipped postures of '{robot}' are unusable: {exc}"]
    if presets is None:
        return []
    notes = []
    where = shipped_dir(config_dir, robot)
    for path, ship in ((preset_path, presets), (sequence_path, sequence)):
        if ship is None:
            continue
        try:
            working = read_json(path, "working file")
        except FileProblem:
            continue  # the run's own loading has already refused it
        if content_hash(working) != content_hash(ship):
            notes.append(f"{path} differs from the shipped copy in {where}")
    calibration = check_calibration(config_dir, robot, ShippedProfile.for_robot(robot).side)
    if calibration:
        notes.append(calibration)
    return notes
