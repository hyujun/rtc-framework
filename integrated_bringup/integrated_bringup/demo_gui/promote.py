"""Promote a GUI working copy to the shipped hand postures — rclpy-free.

    python3 integrated_bringup/tools/demo_postures/promote.py --robot <key> --check
    python3 integrated_bringup/tools/demo_postures/promote.py --robot <key> [--robot <key>]

Reads the working copy from the logging root, checks it as a release
(``shipped.validate_presets`` · ``validate_sequence``) and that the driver in
use runs the calibration the shipped postures are recorded under, then writes
``config/<robot>/demo_gui/`` of the SOURCE tree (never the installed share: a
symlink install points it at the source, and a copy install would lose the
change on the next build). Nothing is written unless every robot passes.
Each robot's posture code (``shipped.posture_code``) is printed:
demo_controller_sequence shows the same code before 'start' on a field PC that
has pulled and built the commit. Operator guide: docs/demo-controller-gui.md
§Shipped postures.

Exit codes: 0 written (or ``--check`` passed) · 1 refused · 2 arguments.
"""

from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass

from ..demo_sequence.files import FileProblem
from .preset_toggle import atomic_write_json
from .shipped import (
    CALIBRATION_FILE,
    PRESETS_FILE,
    SEQUENCE_FILE,
    ShippedProfile,
    calibration_mismatch,
    driver_calibration,
    hand_limits_deg,
    posture_code,
    read_calibration,
    read_json,
    shipped_dir,
    validate_presets,
    validate_sequence,
)

EXIT_OK = 0
EXIT_REFUSED = 1
EXIT_USAGE = 2


@dataclass(frozen=True)
class Plan:
    """One robot's checked promotion: what will be written where."""

    robot: str
    writes: tuple[tuple[str, object], ...]  # (source path, data)
    summary: tuple[str, ...]
    code: str  # posture_code of what ships after the writes


def _diff(name: str, old: object, new: object) -> list[str]:
    if old == new:
        return [f"  {name}: unchanged"]
    if old is None:
        old = {}
    if not isinstance(old, dict) or not isinstance(new, dict):
        return [f"  {name}: replaced"]
    added = sorted(set(new) - set(old))
    removed = sorted(set(old) - set(new))
    changed = sorted(k for k in set(old) & set(new) if old[k] != new[k])
    lines = [f"  {name}:"]
    for label, keys in (("added", added), ("removed", removed), ("changed", changed)):
        if keys:
            lines.append(f"    {label} {len(keys)}: {', '.join(keys)}")
    return lines


def plan_robot(
    prof: ShippedProfile, token: str, src_config: str, share_config: str, logging_root: str
) -> Plan:
    """The checked promotion of ``prof.robot``; FileProblem names the first refusal."""
    target = shipped_dir(src_config, prof.robot)
    cal_path = os.path.join(target, CALIBRATION_FILE)
    if not os.path.exists(cal_path):
        raise FileProblem(
            f"{cal_path} does not exist — write the calibration the postures were "
            "recorded under first"
        )
    recorded = read_calibration(cal_path)
    driver = os.path.join(share_config, prof.robot, recorded.driver_config)
    mismatch = calibration_mismatch(recorded, driver_calibration(driver, prof.side))
    if mismatch:
        raise FileProblem(
            f"the driver in use ({driver}) is not the calibration in {cal_path}: {mismatch}. "
            "Postures recorded under another calibration point elsewhere; if they were "
            "recorded under this one, update calibration.yaml in the same commit"
        )
    dof = len(prof.hand_joint_names)
    limits = hand_limits_deg(
        os.path.join(src_config, prof.robot, "_base.yaml"), prof.hand_group, prof.hand_joint_names
    )
    p_work = os.path.join(logging_root, f"hand_presets_{token}.json")
    presets = validate_presets(
        read_json(p_work, "preset file"), p_work, prof.side, dof, limits, prof.arm_dof
    )
    p_ship = os.path.join(target, PRESETS_FILE)
    old_presets = read_json(p_ship, "shipped preset file") if os.path.exists(p_ship) else None
    writes: list[tuple[str, object]] = [(p_ship, presets)]
    summary = [f"{prof.robot}: {p_work} → {p_ship}", *_diff(PRESETS_FILE, old_presets, presets)]

    s_work = os.path.join(logging_root, f"demo_gui_sequences_{token}.json")
    s_ship = os.path.join(target, SEQUENCE_FILE)
    sequence = None
    if os.path.exists(s_work):
        sequence = read_json(s_work, "sequence file")
        validate_sequence(sequence, s_work, presets, dof)
        old = read_json(s_ship, "shipped sequence file") if os.path.exists(s_ship) else None
        writes.append((s_ship, sequence))
        verdict = "new" if old is None else "unchanged" if old == sequence else "changed"
        summary.append(f"  {SEQUENCE_FILE}: {verdict}")
    elif os.path.exists(s_ship):
        # The shipped sequence stays; it must still play on the new presets.
        sequence = read_json(s_ship, "shipped sequence file")
        validate_sequence(sequence, s_ship, presets, dof)
        summary.append(f"  {SEQUENCE_FILE}: kept (no working sequence)")
    if sequence is None:
        summary.append(f"  no {SEQUENCE_FILE} ships: demo_controller_sequence refuses this robot")
    code = posture_code(presets, sequence)
    summary.append(f"  posture code {code}")
    return Plan(prof.robot, tuple(writes), tuple(summary), code)


def build_parser(robots: tuple[str, ...]) -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="promote.py", description="Promote GUI hand postures to the shipped copy."
    )
    p.add_argument("--robot", action="append", required=True, choices=robots)
    p.add_argument("--check", action="store_true", help="check and show the diff, write nothing")
    p.add_argument("--src-root", required=True, help="the package's source config directory")
    p.add_argument("--share-root", help="the installed config directory (the driver in use)")
    p.add_argument("--logging-root", help="where the GUI's working copy is (absolute)")
    return p


def profile_of(robot: str) -> tuple[ShippedProfile, str]:
    from .discovery import ROBOT_PROFILES

    return ShippedProfile.for_robot(robot), ROBOT_PROFILES[robot].preset_token()


def main(argv: list[str], default_src_root: str) -> int:
    from .discovery import ROBOT_PROFILES

    args = build_parser(tuple(sorted(ROBOT_PROFILES))).parse_args(
        ["--src-root", default_src_root, *argv]
    )
    try:
        share = args.share_root
        if share is None:
            from ament_index_python.packages import (
                PackageNotFoundError,
                get_package_share_directory,
            )

            try:
                share = os.path.join(get_package_share_directory("integrated_bringup"), "config")
            except PackageNotFoundError:
                raise FileProblem(
                    "integrated_bringup is not installed — source the workspace, or pass "
                    "--share-root"
                ) from None
        root = args.logging_root
        if root is None:
            from ..demo_sequence import env

            root, problem = env.logging_root()
            if problem:
                raise FileProblem(f"{problem}, or pass --logging-root")
        elif not os.path.isabs(root) or not os.path.isdir(root):
            raise FileProblem(f"--logging-root {root} is not an absolute directory")
        plans = [
            plan_robot(*profile_of(robot), args.src_root, share, root)
            for robot in dict.fromkeys(args.robot)
        ]
    except FileProblem as exc:
        print(f"refused: {exc}", file=sys.stderr)
        return EXIT_REFUSED
    for plan in plans:
        print("\n".join(plan.summary))
    if args.check:
        print("check passed — nothing written")
        return EXIT_OK
    written: list[str] = []
    for plan in plans:
        for path, data in plan.writes:
            try:
                atomic_write_json(path, data, prefix=".promote_")
            except OSError as exc:
                # Each file is replaced whole, but the set is not: say which
                # ones are new so the tree can be restored with git.
                print(
                    f"refused: writing {path} failed ({exc}); already written: "
                    f"{written or 'nothing'} — restore them with git checkout",
                    file=sys.stderr,
                )
                return EXIT_REFUSED
            written.append(path)
    print("written to the source tree — review with git diff and commit")
    for plan in plans:
        # What the field PCs compare: demo_controller_sequence shows the same
        # code before 'start' once they have pulled and built this commit.
        print(f"{plan.robot}: posture code {plan.code}")
    return EXIT_OK
