#!/usr/bin/env python3
"""Compare MJCF and URDF physics parameters for a robot.

Parses both model files and reports differences in:
  - Link mass
  - Link inertia (diagonal components only; MJCF stores diaginertia)
  - Joint position limits (range)
  - Joint effort / force limits
  - Joint axis vectors
  - Link-to-link offsets (DH-like origin positions)

Robot-agnostic — link/joint name sets and the MJCF default class are auto-
detected from the MJCF root (`<mujoco model="...">` / `<default class="...">`)
unless explicitly overridden via CLI flags.

Usage:
  # Explicit paths (always works):
  ros2 run rtc_tools compare_mjcf_urdf \\
      --mjcf /path/to/robot.xml --urdf /path/to/robot.urdf

  # Auto-detect via ament package + canonical layout
  # (<robot-pkg>/robots/<robot-name>/{mjcf,urdf}/<robot-name>.{xml,urdf}):
  ros2 run rtc_tools compare_mjcf_urdf \\
      --robot-pkg ur5e_description --robot-name ur5e

  # Override the MJCF default class root if it differs from the model name:
  ros2 run rtc_tools compare_mjcf_urdf --mjcf-class my_robot ...

  # Override link name mapping when MJCF body names differ from URDF link
  # names (provide a YAML/JSON file with {<mjcf_body>: <urdf_link>, ...}):
  ros2 run rtc_tools compare_mjcf_urdf --link-map link_map.yaml ...
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


# ── Data structures ────────────────────────────────────────────────────────────


@dataclass
class InertialParams:
    mass: float = 0.0
    origin_xyz: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    # Diagonal inertia (ixx, iyy, izz) — comparable between formats
    diag_inertia: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    # Off-diagonal (ixy, ixz, iyz) — URDF only; MJCF diaginertia assumes zero
    off_diag_inertia: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    # Inertial frame rotation — URDF uses rpy, MJCF uses quat
    origin_rpy: list = field(default_factory=lambda: [0.0, 0.0, 0.0])


@dataclass
class JointParams:
    axis: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    lower: float = 0.0
    upper: float = 0.0
    effort: float = 0.0
    velocity: float = 0.0
    origin_xyz: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    armature: float = 0.0


# ── Auto-detection helpers ────────────────────────────────────────────────────
#
# MJCF body ↔ URDF link mapping and joint name list are detected from the model
# files themselves (or supplied via --link-map / --joints).  No robot-specific
# constants live in this module — see _detect_link_mapping / _detect_joints.


# ── Parsers ────────────────────────────────────────────────────────────────────


def _parse_floats(text: str) -> list:
    return [float(x) for x in text.split()]


def _detect_root_class(root: ET.Element) -> Optional[str]:
    """Return the MJCF top-level default class name, or None if not declared.

    MJCF allows nesting; the root <default> may have no class (acts as a
    catch-all) or a class such as "<robot_name>" used as the inheritance root
    of every other <default class="..."> block. We walk the immediate children
    of root/default looking for the first child <default class="...">.
    """
    top = root.find("default")
    if top is None:
        return None
    # If top-level has class attr, it is the root class
    cls = top.get("class")
    if cls:
        return cls
    # Otherwise walk children for the first inner <default class="...">
    for child in top.findall("default"):
        cls = child.get("class")
        if cls:
            return cls
    return None


def parse_mjcf(
    path: Path,
    link_names: set[str],
    joint_names: set[str],
    root_class_override: Optional[str] = None,
) -> tuple[dict[str, InertialParams], dict[str, JointParams]]:
    """Parse MJCF and extract inertial + joint parameters.

    Args:
        path: MJCF file path.
        link_names: Set of MJCF body names whose <inertial> we extract.
        joint_names: Set of joint names whose attributes we extract.
        root_class_override: Optional MJCF default class to use as the
            inheritance root. If None, autodetected from the file.
    """
    tree = ET.parse(path)
    root = tree.getroot()

    root_class = root_class_override or _detect_root_class(root)

    # ── Resolve default classes ──
    defaults = {}
    for default_elem in root.iter("default"):
        cls = default_elem.get("class")
        if cls is None:
            continue
        attrs = {}
        joint_elem = default_elem.find("joint")
        if joint_elem is not None:
            attrs["joint"] = dict(joint_elem.attrib)
        general_elem = default_elem.find("general")
        if general_elem is not None:
            attrs["general"] = dict(general_elem.attrib)
        defaults[cls] = attrs

    # Build effective joint defaults by resolving inheritance from root_class
    def resolve_joint_defaults(cls_name: str) -> dict:
        """Walk the default tree to merge inherited joint attributes."""
        result = {}
        if root_class and root_class in defaults and "joint" in defaults[root_class]:
            result.update(defaults[root_class]["joint"])
        if cls_name in defaults and "joint" in defaults[cls_name]:
            result.update(defaults[cls_name]["joint"])
        return result

    def resolve_general_defaults(cls_name: str) -> dict:
        result = {}
        if root_class and root_class in defaults and "general" in defaults[root_class]:
            result.update(defaults[root_class]["general"])
        if cls_name in defaults and "general" in defaults[cls_name]:
            result.update(defaults[cls_name]["general"])
        return result

    # ── Parse bodies (links) ──
    links = {}
    for body in root.iter("body"):
        name = body.get("name")
        if name not in link_names:
            continue
        inertial = body.find("inertial")
        if inertial is None:
            continue
        params = InertialParams()
        params.mass = float(inertial.get("mass", "0"))
        params.origin_xyz = _parse_floats(inertial.get("pos", "0 0 0"))
        params.diag_inertia = _parse_floats(inertial.get("diaginertia", "0 0 0"))
        quat = inertial.get("quat")
        if quat:
            params.origin_rpy = _parse_floats(quat)  # Store raw quat for reporting
        links[name] = params

    # ── Parse joints ──
    joints = {}
    for body in root.iter("body"):
        for joint_elem in body.findall("joint"):
            jname = joint_elem.get("name")
            if jname not in joint_names:
                continue
            cls = joint_elem.get("class", root_class or "")
            jdefaults = resolve_joint_defaults(cls)
            gdefaults = resolve_general_defaults(cls)

            jp = JointParams()

            # Axis: joint-level overrides default
            axis_str = joint_elem.get("axis", jdefaults.get("axis", "0 1 0"))
            jp.axis = _parse_floats(axis_str)

            # Range: joint-level overrides default
            range_str = joint_elem.get("range", jdefaults.get("range", "0 0"))
            rng = _parse_floats(range_str)
            jp.lower, jp.upper = rng[0], rng[1]

            # Effort (forcerange)
            fr_str = gdefaults.get("forcerange", "-150 150")
            fr = _parse_floats(fr_str)
            jp.effort = fr[1]  # positive limit

            # Armature
            jp.armature = float(jdefaults.get("armature", "0"))

            # Origin (from parent body's pos attribute)
            jp.origin_xyz = _parse_floats(body.get("pos", "0 0 0"))

            joints[jname] = jp

    return links, joints


def parse_urdf(
    path: Path,
    link_names: set[str],
    joint_names: set[str],
) -> tuple[dict[str, InertialParams], dict[str, JointParams]]:
    """Parse URDF and extract inertial + joint parameters.

    Args:
        path: URDF file path.
        link_names: Set of URDF link names whose <inertial> we extract.
        joint_names: Set of joint names whose attributes we extract.
    """
    tree = ET.parse(path)
    root = tree.getroot()

    # ── Parse links ──
    links = {}
    for link_elem in root.findall("link"):
        name = link_elem.get("name")
        if name not in link_names:
            continue
        inertial = link_elem.find("inertial")
        if inertial is None:
            continue
        params = InertialParams()
        mass_elem = inertial.find("mass")
        if mass_elem is not None:
            params.mass = float(mass_elem.get("value", "0"))

        origin_elem = inertial.find("origin")
        if origin_elem is not None:
            params.origin_xyz = _parse_floats(origin_elem.get("xyz", "0 0 0"))
            params.origin_rpy = _parse_floats(origin_elem.get("rpy", "0 0 0"))

        inertia_elem = inertial.find("inertia")
        if inertia_elem is not None:
            params.diag_inertia = [
                float(inertia_elem.get("ixx", "0")),
                float(inertia_elem.get("iyy", "0")),
                float(inertia_elem.get("izz", "0")),
            ]
            params.off_diag_inertia = [
                float(inertia_elem.get("ixy", "0")),
                float(inertia_elem.get("ixz", "0")),
                float(inertia_elem.get("iyz", "0")),
            ]

        links[name] = params

    # ── Parse joints ──
    joints = {}
    for joint_elem in root.findall("joint"):
        jname = joint_elem.get("name")
        if jname not in joint_names:
            continue
        jp = JointParams()

        axis_elem = joint_elem.find("axis")
        if axis_elem is not None:
            jp.axis = _parse_floats(axis_elem.get("xyz", "0 0 0"))

        limit_elem = joint_elem.find("limit")
        if limit_elem is not None:
            jp.lower = float(limit_elem.get("lower", "0"))
            jp.upper = float(limit_elem.get("upper", "0"))
            jp.effort = float(limit_elem.get("effort", "0"))
            jp.velocity = float(limit_elem.get("velocity", "0"))

        origin_elem = joint_elem.find("origin")
        if origin_elem is not None:
            jp.origin_xyz = _parse_floats(origin_elem.get("xyz", "0 0 0"))

        joints[jname] = jp

    return links, joints


# ── Comparison ─────────────────────────────────────────────────────────────────


def _close(a: float, b: float, tol: float) -> bool:
    return math.isclose(a, b, abs_tol=tol)


def _fmt(val: float) -> str:
    if abs(val) < 1e-10:
        return "0"
    return f"{val:.6g}"


def compare(
    mjcf_path: Path,
    urdf_path: Path,
    link_map: dict[str, str],
    joint_names: list[str],
    tolerance: float = 1e-4,
    mjcf_class: Optional[str] = None,
    robot_label: str = "",
) -> int:
    """Compare MJCF and URDF parameters. Returns number of mismatches.

    Args:
        mjcf_path / urdf_path: Model file paths.
        link_map: Mapping {<mjcf_body_name>: <urdf_link_name>, ...} of which
            link pairs to compare.
        joint_names: Joint names to compare (assumed identical in both files).
        tolerance: Numerical tolerance.
        mjcf_class: Optional MJCF default class override; auto-detected if None.
        robot_label: Optional human label printed in the header.
    """
    mjcf_link_names = set(link_map.keys())
    urdf_link_names = set(link_map.values())
    joint_set = set(joint_names)

    mjcf_links, mjcf_joints = parse_mjcf(
        mjcf_path, mjcf_link_names, joint_set, root_class_override=mjcf_class
    )
    urdf_links, urdf_joints = parse_urdf(urdf_path, urdf_link_names, joint_set)

    mismatches = 0
    warnings = 0

    print("=" * 78)
    label = f"{robot_label} " if robot_label else ""
    print(f"  {label}Model Comparison: MJCF vs URDF")
    print(f"  MJCF: {mjcf_path}")
    print(f"  URDF: {urdf_path}")
    print(f"  Tolerance: {tolerance}")
    print("=" * 78)

    # ── Link inertial comparison ──
    print("\n--- Link Inertial Parameters ---\n")

    for mjcf_name, urdf_name in link_map.items():
        mjcf_ip = mjcf_links.get(mjcf_name)
        urdf_ip = urdf_links.get(urdf_name)

        if mjcf_ip is None:
            print(f"  [WARN] {mjcf_name}: not found in MJCF")
            warnings += 1
            continue
        if urdf_ip is None:
            print(f"  [WARN] {urdf_name}: not found in URDF")
            warnings += 1
            continue

        label = mjcf_name
        if mjcf_name != urdf_name:
            label = f"{mjcf_name} (URDF: {urdf_name})"

        print(f"  [{label}]")

        # Mass
        if not _close(mjcf_ip.mass, urdf_ip.mass, tolerance):
            print(
                f"    MASS MISMATCH:  MJCF={_fmt(mjcf_ip.mass)}  URDF={_fmt(urdf_ip.mass)}"
                f"  (delta={_fmt(abs(mjcf_ip.mass - urdf_ip.mass))})"
            )
            mismatches += 1
        else:
            print(f"    mass: {_fmt(mjcf_ip.mass)}  OK")

        # Diagonal inertia
        inertia_labels = ["Ixx", "Iyy", "Izz"]
        for i, lbl in enumerate(inertia_labels):
            m_val = mjcf_ip.diag_inertia[i]
            u_val = urdf_ip.diag_inertia[i]
            if not _close(m_val, u_val, tolerance):
                print(
                    f"    {lbl} MISMATCH:  MJCF={_fmt(m_val)}  URDF={_fmt(u_val)}"
                    f"  (delta={_fmt(abs(m_val - u_val))})"
                )
                mismatches += 1

        # Check if URDF has significant off-diagonal inertia
        off_diag_mag = math.sqrt(sum(x * x for x in urdf_ip.off_diag_inertia))
        if off_diag_mag > tolerance:
            print(
                f"    [NOTE] URDF has non-zero off-diagonal inertia: "
                f"Ixy={_fmt(urdf_ip.off_diag_inertia[0])}, "
                f"Ixz={_fmt(urdf_ip.off_diag_inertia[1])}, "
                f"Iyz={_fmt(urdf_ip.off_diag_inertia[2])}"
            )
            print(f"           MJCF diaginertia assumes these are zero (principal axes frame).")
            warnings += 1

        # Check if inertial frames differ
        if urdf_ip.origin_rpy != [0.0, 0.0, 0.0]:
            rpy_str = " ".join(_fmt(v) for v in urdf_ip.origin_rpy)
            print(f"    [NOTE] URDF inertial frame rotated: rpy=[{rpy_str}]")
            print(
                f"           Diagonal inertia values are NOT directly comparable "
                f"when frames differ."
            )
            warnings += 1

        print()

    # ── Joint comparison ──
    print("--- Joint Parameters ---\n")

    for jname in joint_names:
        mjcf_jp = mjcf_joints.get(jname)
        urdf_jp = urdf_joints.get(jname)

        if mjcf_jp is None:
            print(f"  [WARN] {jname}: not found in MJCF")
            warnings += 1
            continue
        if urdf_jp is None:
            print(f"  [WARN] {jname}: not found in URDF")
            warnings += 1
            continue

        print(f"  [{jname}]")

        # Position limits
        if not _close(mjcf_jp.lower, urdf_jp.lower, tolerance) or not _close(
            mjcf_jp.upper, urdf_jp.upper, tolerance
        ):
            print(
                f"    RANGE MISMATCH:"
                f"  MJCF=[{_fmt(mjcf_jp.lower)}, {_fmt(mjcf_jp.upper)}]"
                f"  URDF=[{_fmt(urdf_jp.lower)}, {_fmt(urdf_jp.upper)}]"
            )
            mismatches += 1
        else:
            print(f"    range: [{_fmt(mjcf_jp.lower)}, {_fmt(mjcf_jp.upper)}]  OK")

        # Effort limits
        if not _close(mjcf_jp.effort, urdf_jp.effort, tolerance):
            print(
                f"    EFFORT MISMATCH:  MJCF={_fmt(mjcf_jp.effort)}  URDF={_fmt(urdf_jp.effort)}"
            )
            mismatches += 1
        else:
            print(f"    effort: {_fmt(mjcf_jp.effort)}  OK")

        # Axis
        axes_match = all(_close(mjcf_jp.axis[i], urdf_jp.axis[i], tolerance) for i in range(3))
        if not axes_match:
            m_axis = " ".join(_fmt(v) for v in mjcf_jp.axis)
            u_axis = " ".join(_fmt(v) for v in urdf_jp.axis)
            print(f"    AXIS MISMATCH:  MJCF=[{m_axis}]  URDF=[{u_axis}]")
            mismatches += 1
        else:
            axis_str = " ".join(_fmt(v) for v in mjcf_jp.axis)
            print(f"    axis: [{axis_str}]  OK")

        # Origin position
        origins_match = all(
            _close(mjcf_jp.origin_xyz[i], urdf_jp.origin_xyz[i], tolerance) for i in range(3)
        )
        if not origins_match:
            m_orig = " ".join(_fmt(v) for v in mjcf_jp.origin_xyz)
            u_orig = " ".join(_fmt(v) for v in urdf_jp.origin_xyz)
            print(f"    ORIGIN MISMATCH:  MJCF=[{m_orig}]  URDF=[{u_orig}]")
            print(f"           [NOTE] MJCF and URDF use different coordinate conventions.")
            warnings += 1

        # Armature (MJCF-only)
        if mjcf_jp.armature > 0:
            print(f"    armature: {_fmt(mjcf_jp.armature)} (MJCF-only, no URDF equivalent)")

        print()

    # ── Summary ──
    print("=" * 78)
    print(f"  SUMMARY")
    print(f"  Mismatches: {mismatches}")
    print(f"  Warnings:   {warnings}")

    if mismatches == 0 and warnings == 0:
        print(f"  Result: ALL PARAMETERS MATCH within tolerance {tolerance}")
    elif mismatches == 0:
        print(f"  Result: Parameters match but {warnings} warning(s) — review notes above")
    else:
        print(f"  Result: {mismatches} MISMATCH(ES) FOUND — see details above")

    print("=" * 78)
    return mismatches


# ── CLI ────────────────────────────────────────────────────────────────────────


def _resolve_robot_layout(
    pkg: str,
    robot_name: str,
) -> tuple[Optional[Path], Optional[Path]]:
    """Resolve <pkg>/robots/<robot>/{mjcf,urdf}/<robot>.{xml,urdf}.

    Returns (mjcf, urdf) using ament share dir if installed, else falling back
    to the in-source layout (`<repo>/<pkg>/robots/<robot>/...`).
    """
    # Try ament installed package
    try:
        from ament_index_python.packages import get_package_share_directory

        pkg_dir = Path(get_package_share_directory(pkg))
        mjcf = pkg_dir / "robots" / robot_name / "mjcf" / f"{robot_name}.xml"
        urdf = pkg_dir / "robots" / robot_name / "urdf" / f"{robot_name}.urdf"
        if mjcf.exists() and urdf.exists():
            return mjcf, urdf
    except Exception:
        pass

    # Try in-source layout (sibling to rtc_tools)
    repo_root = Path(__file__).resolve().parents[3]
    mjcf = repo_root / pkg / "robots" / robot_name / "mjcf" / f"{robot_name}.xml"
    urdf = repo_root / pkg / "robots" / robot_name / "urdf" / f"{robot_name}.urdf"
    if mjcf.exists() and urdf.exists():
        return mjcf, urdf

    return None, None


def _detect_link_mapping(
    mjcf_path: Path,
    urdf_path: Path,
) -> dict[str, str]:
    """Build {<mjcf_body>: <urdf_link>} for bodies/links present in both files
    that carry an <inertial> child. Names that match exactly are paired; bodies
    with no same-named URDF link are skipped (caller can supply --link-map for
    cases where MJCF and URDF use different naming).
    """
    mjcf_root = ET.parse(mjcf_path).getroot()
    urdf_root = ET.parse(urdf_path).getroot()

    mjcf_inertial_bodies = {
        b.get("name")
        for b in mjcf_root.iter("body")
        if b.get("name") and b.find("inertial") is not None
    }
    urdf_inertial_links = {
        l.get("name")
        for l in urdf_root.findall("link")
        if l.get("name") and l.find("inertial") is not None
    }

    return {name: name for name in sorted(mjcf_inertial_bodies & urdf_inertial_links)}


def _detect_joints(mjcf_path: Path, urdf_path: Path) -> list[str]:
    """Joint names that appear in both files."""
    mjcf_root = ET.parse(mjcf_path).getroot()
    urdf_root = ET.parse(urdf_path).getroot()

    mjcf_joints = {j.get("name") for j in mjcf_root.iter("joint") if j.get("name")}
    urdf_joints = {j.get("name") for j in urdf_root.findall("joint") if j.get("name")}

    return sorted(mjcf_joints & urdf_joints)


def _load_link_map(path: Path) -> dict[str, str]:
    """Load a link-name mapping from YAML or JSON."""
    text = path.read_text()
    try:
        import yaml

        data = yaml.safe_load(text)
    except ImportError:
        import json

        data = json.loads(text)
    if not isinstance(data, dict):
        raise SystemExit(
            f"--link-map file {path} must contain a mapping {{<mjcf_body>: <urdf_link>, ...}}"
        )
    return {str(k): str(v) for k, v in data.items()}


def main():
    parser = argparse.ArgumentParser(
        description="Compare MJCF and URDF physics parameters for a robot.",
    )
    parser.add_argument(
        "--mjcf",
        type=Path,
        default=None,
        help="Path to MJCF file. Required unless --robot-pkg + --robot-name "
        "resolve to a canonical layout.",
    )
    parser.add_argument(
        "--urdf",
        type=Path,
        default=None,
        help="Path to URDF file. Required unless --robot-pkg + --robot-name "
        "resolve to a canonical layout.",
    )
    parser.add_argument(
        "--robot-pkg",
        type=str,
        default=None,
        help="ament package containing the robot description (e.g. "
        "'ur5e_description'). Used with --robot-name to resolve "
        "<pkg>/robots/<robot>/{mjcf,urdf}/<robot>.{xml,urdf}.",
    )
    parser.add_argument(
        "--robot-name",
        type=str,
        default=None,
        help="Robot name used as both directory and file stem under --robot-pkg (e.g. 'ur5e').",
    )
    parser.add_argument(
        "--mjcf-class",
        type=str,
        default=None,
        help="MJCF default class to use as inheritance root. Auto-detected "
        'from <default class="..."> if omitted.',
    )
    parser.add_argument(
        "--link-map",
        type=Path,
        default=None,
        help="YAML/JSON file with {<mjcf_body>: <urdf_link>, ...}. Required "
        "only when MJCF body names differ from URDF link names; "
        "otherwise same-name links are matched automatically.",
    )
    parser.add_argument(
        "--joints",
        type=str,
        nargs="+",
        default=None,
        help="Explicit joint name list to compare. Auto-detected (MJCF ∩ URDF) if omitted.",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=1e-4,
        help="Numerical tolerance for comparison (default: 1e-4).",
    )
    args = parser.parse_args()

    mjcf_path = args.mjcf
    urdf_path = args.urdf

    if (mjcf_path is None or urdf_path is None) and args.robot_pkg and args.robot_name:
        auto_mjcf, auto_urdf = _resolve_robot_layout(args.robot_pkg, args.robot_name)
        if mjcf_path is None:
            mjcf_path = auto_mjcf
        if urdf_path is None:
            urdf_path = auto_urdf

    if mjcf_path is None or not mjcf_path.exists():
        print(f"Error: MJCF file not found: {mjcf_path}", file=sys.stderr)
        print("Specify with --mjcf or --robot-pkg + --robot-name.", file=sys.stderr)
        sys.exit(1)
    if urdf_path is None or not urdf_path.exists():
        print(f"Error: URDF file not found: {urdf_path}", file=sys.stderr)
        print("Specify with --urdf or --robot-pkg + --robot-name.", file=sys.stderr)
        sys.exit(1)

    if args.link_map is not None:
        link_map = _load_link_map(args.link_map)
    else:
        link_map = _detect_link_mapping(mjcf_path, urdf_path)

    if not link_map:
        print(
            "Error: no shared inertial links found between MJCF and URDF "
            "(and no --link-map provided).",
            file=sys.stderr,
        )
        sys.exit(1)

    if args.joints is not None:
        joint_names = args.joints
    else:
        joint_names = _detect_joints(mjcf_path, urdf_path)

    if not joint_names:
        print(
            "Error: no shared joint names between MJCF and URDF (and no --joints provided).",
            file=sys.stderr,
        )
        sys.exit(1)

    robot_label = args.robot_name or ""

    mismatches = compare(
        mjcf_path,
        urdf_path,
        link_map=link_map,
        joint_names=joint_names,
        tolerance=args.tolerance,
        mjcf_class=args.mjcf_class,
        robot_label=robot_label,
    )
    sys.exit(1 if mismatches > 0 else 0)


if __name__ == "__main__":
    main()
