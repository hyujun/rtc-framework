#!/usr/bin/env python3
"""Compare UR5e MJCF (ur5e.xml) and URDF (ur5e.urdf) physics parameters.

Parses both model files and reports differences in:
  - Link mass
  - Link inertia (diagonal components only; MJCF stores diaginertia)
  - Joint position limits (range)
  - Joint effort / force limits
  - Joint axis vectors
  - Link-to-link offsets (DH-like origin positions)

Usage:
  ros2 run ur5e_tools compare_mjcf_urdf
  ros2 run ur5e_tools compare_mjcf_urdf --mjcf /path/to/ur5e.xml --urdf /path/to/ur5e.urdf
  ros2 run ur5e_tools compare_mjcf_urdf --tolerance 0.01
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


# ── MJCF mapping ──────────────────────────────────────────────────────────────
# MJCF body names → corresponding URDF link names
MJCF_TO_URDF_LINK = {
    "base": "base_link_inertia",
    "shoulder_link": "shoulder_link",
    "upper_arm_link": "upper_arm_link",
    "forearm_link": "forearm_link",
    "wrist_1_link": "wrist_1_link",
    "wrist_2_link": "wrist_2_link",
    "wrist_3_link": "wrist_3_link",
}

# Joint names are identical in both formats
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


# ── Parsers ────────────────────────────────────────────────────────────────────

def _parse_floats(text: str) -> list:
    return [float(x) for x in text.split()]


def parse_mjcf(path: Path) -> tuple[dict[str, InertialParams], dict[str, JointParams]]:
    """Parse MJCF ur5e.xml and extract inertial + joint parameters."""
    tree = ET.parse(path)
    root = tree.getroot()

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

    # Build effective joint defaults by resolving inheritance
    def resolve_joint_defaults(cls_name: str) -> dict:
        """Walk the default tree to merge inherited joint attributes."""
        result = {}
        # Start from the root "ur5e" class
        if "ur5e" in defaults and "joint" in defaults["ur5e"]:
            result.update(defaults["ur5e"]["joint"])
        # Apply class-specific overrides
        if cls_name in defaults and "joint" in defaults[cls_name]:
            result.update(defaults[cls_name]["joint"])
        return result

    def resolve_general_defaults(cls_name: str) -> dict:
        result = {}
        if "ur5e" in defaults and "general" in defaults["ur5e"]:
            result.update(defaults["ur5e"]["general"])
        if cls_name in defaults and "general" in defaults[cls_name]:
            result.update(defaults[cls_name]["general"])
        return result

    # ── Parse bodies (links) ──
    links = {}
    for body in root.iter("body"):
        name = body.get("name")
        if name not in MJCF_TO_URDF_LINK:
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
            if jname not in JOINT_NAMES:
                continue
            cls = joint_elem.get("class", "ur5e")
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


def parse_urdf(path: Path) -> tuple[dict[str, InertialParams], dict[str, JointParams]]:
    """Parse URDF ur5e.urdf and extract inertial + joint parameters."""
    tree = ET.parse(path)
    root = tree.getroot()

    # ── Parse links ──
    links = {}
    for link_elem in root.findall("link"):
        name = link_elem.get("name")
        if name not in MJCF_TO_URDF_LINK.values():
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
        if jname not in JOINT_NAMES:
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
    tolerance: float = 1e-4,
) -> int:
    """Compare MJCF and URDF parameters. Returns number of mismatches."""
    mjcf_links, mjcf_joints = parse_mjcf(mjcf_path)
    urdf_links, urdf_joints = parse_urdf(urdf_path)

    mismatches = 0
    warnings = 0

    print("=" * 78)
    print(f"  UR5e Model Comparison: MJCF vs URDF")
    print(f"  MJCF: {mjcf_path}")
    print(f"  URDF: {urdf_path}")
    print(f"  Tolerance: {tolerance}")
    print("=" * 78)

    # ── Link inertial comparison ──
    print("\n--- Link Inertial Parameters ---\n")

    for mjcf_name, urdf_name in MJCF_TO_URDF_LINK.items():
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
            print(f"    MASS MISMATCH:  MJCF={_fmt(mjcf_ip.mass)}  URDF={_fmt(urdf_ip.mass)}"
                  f"  (delta={_fmt(abs(mjcf_ip.mass - urdf_ip.mass))})")
            mismatches += 1
        else:
            print(f"    mass: {_fmt(mjcf_ip.mass)}  OK")

        # Diagonal inertia
        inertia_labels = ["Ixx", "Iyy", "Izz"]
        for i, lbl in enumerate(inertia_labels):
            m_val = mjcf_ip.diag_inertia[i]
            u_val = urdf_ip.diag_inertia[i]
            if not _close(m_val, u_val, tolerance):
                print(f"    {lbl} MISMATCH:  MJCF={_fmt(m_val)}  URDF={_fmt(u_val)}"
                      f"  (delta={_fmt(abs(m_val - u_val))})")
                mismatches += 1

        # Check if URDF has significant off-diagonal inertia
        off_diag_mag = math.sqrt(sum(x * x for x in urdf_ip.off_diag_inertia))
        if off_diag_mag > tolerance:
            print(f"    [NOTE] URDF has non-zero off-diagonal inertia: "
                  f"Ixy={_fmt(urdf_ip.off_diag_inertia[0])}, "
                  f"Ixz={_fmt(urdf_ip.off_diag_inertia[1])}, "
                  f"Iyz={_fmt(urdf_ip.off_diag_inertia[2])}")
            print(f"           MJCF diaginertia assumes these are zero (principal axes frame).")
            warnings += 1

        # Check if inertial frames differ
        if urdf_ip.origin_rpy != [0.0, 0.0, 0.0]:
            rpy_str = " ".join(_fmt(v) for v in urdf_ip.origin_rpy)
            print(f"    [NOTE] URDF inertial frame rotated: rpy=[{rpy_str}]")
            print(f"           Diagonal inertia values are NOT directly comparable "
                  f"when frames differ.")
            warnings += 1

        print()

    # ── Joint comparison ──
    print("--- Joint Parameters ---\n")

    for jname in JOINT_NAMES:
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
        if not _close(mjcf_jp.lower, urdf_jp.lower, tolerance) or \
           not _close(mjcf_jp.upper, urdf_jp.upper, tolerance):
            print(f"    RANGE MISMATCH:"
                  f"  MJCF=[{_fmt(mjcf_jp.lower)}, {_fmt(mjcf_jp.upper)}]"
                  f"  URDF=[{_fmt(urdf_jp.lower)}, {_fmt(urdf_jp.upper)}]")
            mismatches += 1
        else:
            print(f"    range: [{_fmt(mjcf_jp.lower)}, {_fmt(mjcf_jp.upper)}]  OK")

        # Effort limits
        if not _close(mjcf_jp.effort, urdf_jp.effort, tolerance):
            print(f"    EFFORT MISMATCH:"
                  f"  MJCF={_fmt(mjcf_jp.effort)}  URDF={_fmt(urdf_jp.effort)}")
            mismatches += 1
        else:
            print(f"    effort: {_fmt(mjcf_jp.effort)}  OK")

        # Axis
        axes_match = all(
            _close(mjcf_jp.axis[i], urdf_jp.axis[i], tolerance)
            for i in range(3)
        )
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
            _close(mjcf_jp.origin_xyz[i], urdf_jp.origin_xyz[i], tolerance)
            for i in range(3)
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

def _find_default_paths() -> tuple[Optional[Path], Optional[Path]]:
    """Try to locate model files via ament package prefix or relative paths."""
    # Try ament (ROS2 installed package)
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_dir = Path(get_package_share_directory("ur5e_description"))
        mjcf = pkg_dir / "robots" / "ur5e" / "mjcf" / "ur5e.xml"
        urdf = pkg_dir / "robots" / "ur5e" / "urdf" / "ur5e.urdf"
        if mjcf.exists() and urdf.exists():
            return mjcf, urdf
    except (ImportError, Exception):
        pass

    # Try relative to this script (development layout)
    repo_root = Path(__file__).resolve().parents[3]
    mjcf = repo_root / "ur5e_description" / "robots" / "ur5e" / "mjcf" / "ur5e.xml"
    urdf = repo_root / "ur5e_description" / "robots" / "ur5e" / "urdf" / "ur5e.urdf"
    if mjcf.exists() and urdf.exists():
        return mjcf, urdf

    return None, None


def main():
    parser = argparse.ArgumentParser(
        description="Compare UR5e MJCF and URDF physics parameters.",
    )
    parser.add_argument(
        "--mjcf", type=Path, default=None,
        help="Path to ur5e.xml (MJCF). Auto-detected if omitted.",
    )
    parser.add_argument(
        "--urdf", type=Path, default=None,
        help="Path to ur5e.urdf (URDF). Auto-detected if omitted.",
    )
    parser.add_argument(
        "--tolerance", type=float, default=1e-4,
        help="Numerical tolerance for comparison (default: 1e-4).",
    )
    args = parser.parse_args()

    mjcf_path = args.mjcf
    urdf_path = args.urdf

    if mjcf_path is None or urdf_path is None:
        auto_mjcf, auto_urdf = _find_default_paths()
        if mjcf_path is None:
            mjcf_path = auto_mjcf
        if urdf_path is None:
            urdf_path = auto_urdf

    if mjcf_path is None or not mjcf_path.exists():
        print(f"Error: MJCF file not found: {mjcf_path}", file=sys.stderr)
        print("Use --mjcf to specify the path.", file=sys.stderr)
        sys.exit(1)
    if urdf_path is None or not urdf_path.exists():
        print(f"Error: URDF file not found: {urdf_path}", file=sys.stderr)
        print("Use --urdf to specify the path.", file=sys.stderr)
        sys.exit(1)

    mismatches = compare(mjcf_path, urdf_path, args.tolerance)
    sys.exit(1 if mismatches > 0 else 0)


if __name__ == "__main__":
    main()
