#!/usr/bin/env python3
"""Convert URDF or XACRO files to MuJoCo MJCF XML.

Supports:
  - Standard robot directory convention: robots/<name>/{urdf,mjcf,meshes}
  - Automatic joint classification (active, mimic, closed-chain)
  - MuJoCo equality constraints for passive joints
  - Actuator generation for active joints only
  - Scene file generation

Usage:
  ros2 run rtc_tools urdf_to_mjcf --robot-dir robots/ur5e
  ros2 run rtc_tools urdf_to_mjcf --input /path/to/robot.urdf --output /path/to/output.xml
  ros2 run rtc_tools urdf_to_mjcf --robot-dir robots/ur5e --scene --validate
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import tempfile
import xml.etree.ElementTree as ET
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


# ════════════════════════════════════════════════════════════════════════════
# Joint Classification
# ════════════════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class MimicParams:
    """Parameters for a URDF mimic joint relationship."""

    master_joint: str
    multiplier: float = 1.0
    offset: float = 0.0


@dataclass(frozen=True)
class JointInfo:
    """Parsed information for a single URDF joint."""

    name: str
    joint_type: str
    parent_link: str
    child_link: str
    axis: tuple[float, float, float] = (0.0, 0.0, 1.0)
    origin_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    origin_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    effort_limit: float = 0.0
    lower_limit: float = 0.0
    upper_limit: float = 0.0
    velocity_limit: float = 0.0
    mimic: MimicParams | None = None
    is_closed_chain: bool = False


@dataclass
class JointClassification:
    """Classification of all non-fixed URDF joints."""

    active: dict[str, JointInfo] = field(default_factory=dict)
    passive_mimic: dict[str, JointInfo] = field(default_factory=dict)
    passive_closed_chain: dict[str, JointInfo] = field(default_factory=dict)
    fixed: list[str] = field(default_factory=list)

    @property
    def passive_names(self) -> set[str]:
        return set(self.passive_mimic.keys()) | set(self.passive_closed_chain.keys())


def _parse_vec3(
    text: str | None, default: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> tuple[float, float, float]:
    if not text:
        return default
    parts = text.strip().split()
    return (float(parts[0]), float(parts[1]), float(parts[2]))


def classify_joints(urdf_xml: str) -> JointClassification:
    """Parse URDF XML and classify joints into active/passive categories.

    Classification rules:
      - fixed joints are recorded but excluded from active/passive.
      - mimic joints (with <mimic> element) are passive.
      - closed-chain joints (child_link referenced by multiple non-fixed joints)
        are passive — the first joint claiming a child_link is kept as active.
      - all remaining joints are active.
    """
    root = ET.fromstring(urdf_xml)
    result = JointClassification()

    parsed: list[JointInfo] = []
    child_link_joints: dict[str, list[int]] = defaultdict(list)

    for jelem in root.findall("joint"):
        jname = jelem.get("name", "")
        jtype = jelem.get("type", "")
        if not jname:
            continue

        if jtype == "fixed":
            result.fixed.append(jname)
            continue

        parent_elem = jelem.find("parent")
        child_elem = jelem.find("child")
        parent_link = parent_elem.get("link", "") if parent_elem is not None else ""
        child_link = child_elem.get("link", "") if child_elem is not None else ""

        origin_elem = jelem.find("origin")
        origin_xyz = _parse_vec3(
            origin_elem.get("xyz") if origin_elem is not None else None,
        )
        origin_rpy = _parse_vec3(
            origin_elem.get("rpy") if origin_elem is not None else None,
        )

        axis_elem = jelem.find("axis")
        axis = _parse_vec3(
            axis_elem.get("xyz") if axis_elem is not None else None,
            default=(0.0, 0.0, 1.0),
        )

        limit_elem = jelem.find("limit")
        effort = float(limit_elem.get("effort", "0")) if limit_elem is not None else 0.0
        lower = float(limit_elem.get("lower", "0")) if limit_elem is not None else 0.0
        upper = float(limit_elem.get("upper", "0")) if limit_elem is not None else 0.0
        velocity = float(limit_elem.get("velocity", "0")) if limit_elem is not None else 0.0

        mimic = None
        mimic_elem = jelem.find("mimic")
        if mimic_elem is not None:
            mimic = MimicParams(
                master_joint=mimic_elem.get("joint", ""),
                multiplier=float(mimic_elem.get("multiplier", "1.0")),
                offset=float(mimic_elem.get("offset", "0.0")),
            )

        info = JointInfo(
            name=jname,
            joint_type=jtype,
            parent_link=parent_link,
            child_link=child_link,
            axis=axis,
            origin_xyz=origin_xyz,
            origin_rpy=origin_rpy,
            effort_limit=effort,
            lower_limit=lower,
            upper_limit=upper,
            velocity_limit=velocity,
            mimic=mimic,
        )
        idx = len(parsed)
        parsed.append(info)

        if child_link:
            child_link_joints[child_link].append(idx)

    # Detect closed-chain: child_link referenced by multiple non-fixed joints
    closed_chain_indices: set[int] = set()
    for indices in child_link_joints.values():
        if len(indices) <= 1:
            continue
        first_tree = True
        for i in indices:
            if parsed[i].mimic is not None:
                continue
            if first_tree:
                first_tree = False
                continue
            closed_chain_indices.add(i)

    for i, info in enumerate(parsed):
        if info.mimic is not None:
            result.passive_mimic[info.name] = info
        elif i in closed_chain_indices:
            result.passive_closed_chain[info.name] = JointInfo(
                name=info.name,
                joint_type=info.joint_type,
                parent_link=info.parent_link,
                child_link=info.child_link,
                axis=info.axis,
                origin_xyz=info.origin_xyz,
                origin_rpy=info.origin_rpy,
                effort_limit=info.effort_limit,
                lower_limit=info.lower_limit,
                upper_limit=info.upper_limit,
                velocity_limit=info.velocity_limit,
                mimic=info.mimic,
                is_closed_chain=True,
            )
        else:
            result.active[info.name] = info

    return result


# ════════════════════════════════════════════════════════════════════════════
# URDF Pre-processing
# ════════════════════════════════════════════════════════════════════════════

def remove_closed_chain_joints(
    urdf_xml: str, closed_chain_joints: dict[str, JointInfo],
) -> str:
    """Remove closed-chain joints from URDF so MuJoCo can compile it as a tree.

    MuJoCo requires a strict tree topology.  Closed-chain joints (kinematic
    loops) break that requirement.  This function removes loop-closing joints
    from the XML; the caller should later add ``<equality><connect>``
    constraints to re-close the loops in the MJCF.
    """
    if not closed_chain_joints:
        return urdf_xml

    root = ET.fromstring(urdf_xml)
    to_remove = set(closed_chain_joints.keys())

    for jelem in list(root.findall("joint")):
        if jelem.get("name", "") in to_remove:
            root.remove(jelem)

    return ET.tostring(root, encoding="unicode", xml_declaration=True)


# ════════════════════════════════════════════════════════════════════════════
# Xacro / Package URI Helpers
# ════════════════════════════════════════════════════════════════════════════

def process_xacro(xacro_path: Path, xacro_args: Optional[list[str]] = None) -> str:
    """Process a xacro file and return URDF XML string."""
    try:
        import xacro
    except ImportError:
        print(
            "Error: 'xacro' package is required for .xacro files.\n"
            "Install with: pip install xacro  or  "
            "apt install ros-${ROS_DISTRO}-xacro",
            file=sys.stderr,
        )
        sys.exit(1)

    mappings: dict[str, str] = {}
    if xacro_args:
        for arg in xacro_args:
            if ":=" in arg:
                key, val = arg.split(":=", 1)
                mappings[key] = val
            else:
                print(
                    f"Warning: ignoring invalid xacro arg '{arg}' "
                    "(expected key:=value)",
                    file=sys.stderr,
                )

    doc = xacro.process_file(str(xacro_path), mappings=mappings)
    return doc.toprettyxml(indent="  ")


def is_xacro_file(path: Path) -> bool:
    """Check if the file is a xacro file by extension."""
    return ".xacro" in [s.lower() for s in path.suffixes]


def resolve_package_uris(urdf_xml: str) -> str:
    """Replace ``package://`` URIs with absolute paths using ament index."""
    pkg_cache: dict[str, Optional[str]] = {}

    def _resolve_pkg(pkg_name: str) -> Optional[str]:
        if pkg_name in pkg_cache:
            return pkg_cache[pkg_name]
        resolved = None
        try:
            from ament_index_python.packages import get_package_share_directory

            resolved = get_package_share_directory(pkg_name)
        except Exception:
            ws_src = Path(__file__).resolve().parents[3]
            candidate = ws_src / pkg_name
            if candidate.is_dir():
                resolved = str(candidate)
        pkg_cache[pkg_name] = resolved
        return resolved

    def _replace(match: re.Match) -> str:
        pkg_name = match.group(1)
        rel_path = match.group(2)
        pkg_dir = _resolve_pkg(pkg_name)
        if pkg_dir:
            return str(Path(pkg_dir) / rel_path)
        return match.group(0)

    return re.sub(r"package://([^/]+)/([^\s\"'<>]+)", _replace, urdf_xml)


def resolve_mesh_paths(urdf_xml: str, urdf_dir: Path, meshdir: Path) -> str:
    """Resolve relative mesh filenames in URDF to absolute paths.

    MuJoCo resolves mesh paths relative to the XML file location.  Since we
    write the URDF to a temporary directory for compilation, any relative
    mesh references would break.  This function converts them to absolute
    paths so MuJoCo can find the files regardless of working directory.

    Search order for each mesh filename:
      1. Already absolute or ``package://`` → skip
      2. Relative to the original URDF directory
      3. Directly inside *meshdir*
      4. Recursively under *meshdir* (handles subdirectories like
         ``collision/``, ``visual/``)
    """
    root = ET.fromstring(urdf_xml)
    meshdir = meshdir.resolve()
    urdf_dir = urdf_dir.resolve()

    for geom in root.iter("geometry"):
        mesh_elem = geom.find("mesh")
        if mesh_elem is None:
            continue

        filename = mesh_elem.get("filename", "")
        if not filename or filename.startswith("package://"):
            continue

        fpath = Path(filename)
        if fpath.is_absolute() and fpath.exists():
            continue

        # Try resolving the relative path
        resolved = None

        # 1) Relative to original URDF directory
        candidate = urdf_dir / filename
        if candidate.exists():
            resolved = candidate
        else:
            # 2) Directly in meshdir
            candidate = meshdir / fpath.name
            if candidate.exists():
                resolved = candidate
            else:
                # 3) Recursive search under meshdir
                matches = list(meshdir.rglob(fpath.name))
                if matches:
                    resolved = matches[0]

        if resolved is not None:
            mesh_elem.set("filename", str(resolved.resolve()))

    return ET.tostring(root, encoding="unicode", xml_declaration=True)


# ════════════════════════════════════════════════════════════════════════════
# Directory Convention Helpers
# ════════════════════════════════════════════════════════════════════════════

def resolve_robot_dir_paths(
    robot_dir: Path,
    urdf_file: Optional[str] = None,
) -> tuple[Path, Path, Path]:
    """Resolve input URDF, output MJCF, and meshdir from robot directory convention.

    Expected layout::

        robot_dir/
        ├── urdf/    ← URDF files
        ├── mjcf/    ← output MJCF files
        └── meshes/  ← mesh files
            └── assets/  (preferred for MuJoCo OBJ files)

    Returns:
        (urdf_path, output_path, meshdir)
    """
    robot_dir = robot_dir.resolve()
    if not robot_dir.is_dir():
        print(f"Error: robot directory not found: {robot_dir}", file=sys.stderr)
        sys.exit(1)

    robot_name = robot_dir.name
    urdf_dir = robot_dir / "urdf"
    mjcf_dir = robot_dir / "mjcf"
    meshes_dir = robot_dir / "meshes"

    # --- Resolve URDF path ---
    if urdf_file:
        urdf_path = urdf_dir / urdf_file
    else:
        # Auto-discover: prefer <robot_name>.urdf, then first .urdf
        candidates = sorted(urdf_dir.glob("*.urdf")) + sorted(urdf_dir.glob("*.urdf.xacro"))
        preferred = urdf_dir / f"{robot_name}.urdf"
        if preferred.exists():
            urdf_path = preferred
        elif candidates:
            urdf_path = candidates[0]
        else:
            print(f"Error: no URDF files found in {urdf_dir}", file=sys.stderr)
            sys.exit(1)

    if not urdf_path.exists():
        print(f"Error: URDF file not found: {urdf_path}", file=sys.stderr)
        sys.exit(1)

    # --- Resolve output path ---
    stem = urdf_path.stem
    if stem.endswith(".urdf"):
        stem = stem[:-5]
    output_path = mjcf_dir / f"{stem}.xml"

    # --- Resolve meshdir ---
    # Prefer meshes/assets/ (OBJ) → meshes/ (STL/DAE)
    assets_dir = meshes_dir / "assets"
    if assets_dir.is_dir() and any(assets_dir.glob("*.obj")):
        meshdir = assets_dir
    elif meshes_dir.is_dir():
        meshdir = meshes_dir
    else:
        meshdir = urdf_path.parent

    return urdf_path, output_path, meshdir


# ════════════════════════════════════════════════════════════════════════════
# MJCF Post-Processing
# ════════════════════════════════════════════════════════════════════════════

def _fix_compiler(mjcf_root: ET.Element, meshdir_rel: str) -> None:
    """Set compiler attributes for clean MJCF output."""
    compiler = mjcf_root.find("compiler")
    if compiler is None:
        # Insert as the first child element
        compiler = ET.Element("compiler")
        mjcf_root.insert(0, compiler)

    compiler.set("angle", "radian")
    compiler.set("meshdir", meshdir_rel)
    compiler.set("autolimits", "true")


def _add_option(mjcf_root: ET.Element) -> None:
    """Add simulation option element if not present."""
    if mjcf_root.find("option") is None:
        compiler = mjcf_root.find("compiler")
        idx = list(mjcf_root).index(compiler) + 1 if compiler is not None else 0
        option = ET.Element("option")
        option.set("integrator", "implicitfast")
        mjcf_root.insert(idx, option)


def _add_equality_constraints(
    mjcf_root: ET.Element, classification: JointClassification,
) -> int:
    """Add ``<equality>`` constraints for mimic and closed-chain joints.

    Mimic joints   → ``<joint>``   with polycoef encoding the linear relation.
    Closed-chain   → ``<connect>`` anchoring the loop-closing point.

    Returns:
        Number of equality constraints added.
    """
    if not classification.passive_mimic and not classification.passive_closed_chain:
        return 0

    equality = mjcf_root.find("equality")
    if equality is None:
        equality = ET.SubElement(mjcf_root, "equality")

    count = 0

    # Mimic joints → <joint joint1="slave" joint2="master" polycoef="o m 0 0 0"/>
    for name, info in classification.passive_mimic.items():
        if info.mimic is None:
            continue
        attrib = {
            "joint1": name,
            "joint2": info.mimic.master_joint,
            "polycoef": (
                f"{info.mimic.offset} {info.mimic.multiplier} 0 0 0"
            ),
        }
        equality.append(_make_element("joint", attrib))
        count += 1

    # Closed-chain joints → <connect body1="parent" body2="child" anchor="x y z"/>
    for name, info in classification.passive_closed_chain.items():
        x, y, z = info.origin_xyz
        attrib = {
            "body1": info.parent_link,
            "body2": info.child_link,
            "anchor": f"{x} {y} {z}",
        }
        equality.append(_make_element("connect", attrib))
        count += 1

    return count


def _add_actuators(
    mjcf_root: ET.Element, classification: JointClassification,
) -> int:
    """Add ``<actuator>`` entries for active (non-passive) joints only.

    Returns:
        Number of actuators added.
    """
    existing = mjcf_root.find("actuator")
    if existing is not None:
        mjcf_root.remove(existing)

    if not classification.active:
        return 0

    actuator = ET.SubElement(mjcf_root, "actuator")
    count = 0

    for name, info in classification.active.items():
        # Derive a short actuator name: strip common suffixes
        act_name = name
        for suffix in ("_joint", "_Joint"):
            if act_name.endswith(suffix):
                act_name = act_name[: -len(suffix)]
                break

        attrib: dict[str, str] = {
            "name": act_name,
            "joint": name,
        }
        if info.lower_limit != 0.0 or info.upper_limit != 0.0:
            attrib["ctrlrange"] = f"{info.lower_limit:.5g} {info.upper_limit:.5g}"
        if info.effort_limit > 0:
            attrib["forcerange"] = f"{-info.effort_limit:.5g} {info.effort_limit:.5g}"

        actuator.append(_make_element("general", attrib))
        count += 1

    return count


def _clean_mesh_paths(mjcf_root: ET.Element) -> None:
    """Strip directory prefixes from mesh file attributes (keep filename only)."""
    for mesh_elem in mjcf_root.iter("mesh"):
        file_attr = mesh_elem.get("file")
        if file_attr:
            mesh_elem.set("file", Path(file_attr).name)


def _make_element(tag: str, attrib: dict[str, str]) -> ET.Element:
    """Create an Element with attributes in insertion order."""
    elem = ET.Element(tag)
    for k, v in attrib.items():
        elem.set(k, v)
    return elem


def postprocess_mjcf(
    mjcf_path: Path,
    classification: JointClassification,
    meshdir: Path,
) -> tuple[int, int]:
    """Post-process raw MuJoCo MJCF output.

    - Fixes ``<compiler>`` meshdir to a relative path.
    - Adds ``<option integrator="implicitfast"/>``.
    - Inserts ``<equality>`` constraints for passive joints.
    - Generates ``<actuator>`` entries for active joints only.
    - Strips absolute paths from mesh file references.

    Returns:
        (num_equality_constraints, num_actuators)
    """
    tree = ET.parse(str(mjcf_path))
    root = tree.getroot()

    # Relative meshdir from the output XML location
    meshdir_rel = os.path.relpath(meshdir, mjcf_path.parent)

    _fix_compiler(root, meshdir_rel)
    _add_option(root)
    _clean_mesh_paths(root)
    n_eq = _add_equality_constraints(root, classification)
    n_act = _add_actuators(root, classification)

    ET.indent(tree, space="  ")
    tree.write(str(mjcf_path), encoding="unicode", xml_declaration=True)

    return n_eq, n_act


# ════════════════════════════════════════════════════════════════════════════
# Scene File Generation
# ════════════════════════════════════════════════════════════════════════════

SCENE_TEMPLATE = """\
<mujoco model="{model_name} scene">
  <include file="{robot_xml}"/>

  <statistic center="0.3 0 0.4" extent="0.8"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.1 0.1 0.1" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="120" elevation="-20"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge"
      rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>

  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
  </worldbody>
</mujoco>
"""


def generate_scene(output_dir: Path, robot_xml_name: str, model_name: str) -> Path:
    """Generate a scene.xml that includes the robot MJCF via ``<include>``."""
    scene_path = output_dir / "scene.xml"
    content = SCENE_TEMPLATE.format(
        model_name=model_name,
        robot_xml=robot_xml_name,
    )
    scene_path.write_text(content)
    return scene_path


# ════════════════════════════════════════════════════════════════════════════
# Report Helpers
# ════════════════════════════════════════════════════════════════════════════

def print_classification(classification: JointClassification) -> None:
    """Print a human-readable summary of joint classification."""
    print("\nJoint classification:")
    print(f"  Active ({len(classification.active)}): ", end="")
    print(", ".join(classification.active.keys()) if classification.active else "-")

    print(f"  Passive mimic ({len(classification.passive_mimic)}): ", end="")
    if classification.passive_mimic:
        parts = []
        for name, info in classification.passive_mimic.items():
            m = info.mimic
            if m:
                parts.append(f"{name} (master={m.master_joint}, m={m.multiplier}, o={m.offset})")
            else:
                parts.append(name)
        print(", ".join(parts))
    else:
        print("-")

    print(f"  Closed-chain ({len(classification.passive_closed_chain)}): ", end="")
    if classification.passive_closed_chain:
        parts = []
        for name, info in classification.passive_closed_chain.items():
            parts.append(f"{name} ({info.parent_link} ↔ {info.child_link})")
        print(", ".join(parts))
    else:
        print("-")

    print(f"  Fixed ({len(classification.fixed)}): ", end="")
    print(", ".join(classification.fixed) if classification.fixed else "-")


def print_model_stats(model: "mujoco.MjModel", n_eq: int, n_act: int) -> None:
    """Print MuJoCo model statistics."""
    print(f"\nModel statistics:")
    print(f"  Bodies:      {model.nbody}")
    print(f"  Joints:      {model.njnt}")
    print(f"  DOF:         {model.nv}")
    print(f"  Geoms:       {model.ngeom}")
    print(f"  Meshes:      {model.nmesh}")
    print(f"  Actuators:   {n_act}")
    print(f"  Equality:    {n_eq}")


# ════════════════════════════════════════════════════════════════════════════
# Core Conversion
# ════════════════════════════════════════════════════════════════════════════

def convert_urdf_to_mjcf(
    input_path: Path,
    output_path: Optional[Path] = None,
    xacro_args: Optional[list[str]] = None,
    meshdir: Optional[Path] = None,
    generate_scene_file: bool = False,
    run_validation: bool = False,
) -> Path:
    """Convert a URDF/XACRO file to MuJoCo MJCF XML.

    Args:
        input_path: Path to ``.urdf`` or ``.xacro`` file.
        output_path: Output MJCF path. Defaults to ``<input_stem>.xml``
            in the same directory.
        xacro_args: Extra xacro arguments (``key:=value``).
        meshdir: Directory containing mesh files.
        generate_scene_file: If True, also generate ``scene.xml``.
        run_validation: If True, run ``compare_mjcf_urdf`` after conversion.

    Returns:
        Path to the generated MJCF XML file.
    """
    input_path = input_path.resolve()
    if not input_path.exists():
        raise FileNotFoundError(f"Input file not found: {input_path}")

    if meshdir is None:
        meshdir = input_path.parent
    meshdir = meshdir.resolve()

    # Determine output path
    if output_path is None:
        stem = input_path.stem
        if stem.endswith(".urdf"):
            stem = stem[:-5]
        output_path = input_path.parent / f"{stem}_mjcf.xml"

    # ── Step 1: Get URDF XML ───────────────────────────────────────────
    if is_xacro_file(input_path):
        print(f"Processing xacro: {input_path}")
        urdf_xml = process_xacro(input_path, xacro_args)
    else:
        urdf_xml = input_path.read_text()

    urdf_xml = resolve_package_uris(urdf_xml)

    # Resolve relative mesh paths to absolute so MuJoCo can find them
    # from the temporary URDF file location
    urdf_xml = resolve_mesh_paths(urdf_xml, input_path.parent, meshdir)

    # ── Step 2: Classify joints ────────────────────────────────────────
    classification = classify_joints(urdf_xml)
    print_classification(classification)

    # ── Step 3: Pre-process — remove closed-chain joints for tree topology
    if classification.passive_closed_chain:
        print(
            f"\nRemoving {len(classification.passive_closed_chain)} closed-chain "
            f"joint(s) for tree compilation..."
        )
        urdf_xml = remove_closed_chain_joints(
            urdf_xml, classification.passive_closed_chain,
        )

    # ── Step 4: Write temp URDF & compile with MuJoCo ─────────────────
    import mujoco

    tmp_dir = tempfile.mkdtemp(prefix="urdf_to_mjcf_")
    tmp_urdf = Path(tmp_dir) / "robot.urdf"
    tmp_urdf.write_text(urdf_xml)

    print(f"\nLoading URDF into MuJoCo...")
    original_dir = os.getcwd()
    try:
        os.chdir(meshdir)
        model = mujoco.MjModel.from_xml_path(str(tmp_urdf))
    except Exception as e:
        print(f"Error: MuJoCo failed to compile URDF: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        os.chdir(original_dir)

    # ── Step 5: Save raw MJCF ─────────────────────────────────────────
    output_path = output_path.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    mujoco.mj_saveLastXML(str(output_path), model)

    # ── Step 6: Post-process MJCF ─────────────────────────────────────
    n_eq, n_act = postprocess_mjcf(output_path, classification, meshdir)

    print(f"\nMJCF saved: {output_path}")
    print_model_stats(model, n_eq, n_act)

    # ── Step 7: Generate scene.xml ────────────────────────────────────
    if generate_scene_file:
        robot_name = output_path.stem
        scene_path = generate_scene(
            output_path.parent, output_path.name, robot_name,
        )
        print(f"Scene saved: {scene_path}")

    # ── Step 8: Validate ──────────────────────────────────────────────
    if run_validation:
        _run_validation(output_path, input_path)

    return output_path


def _run_validation(mjcf_path: Path, urdf_path: Path) -> None:
    """Run compare_mjcf_urdf validation between generated MJCF and source URDF."""
    try:
        from rtc_tools.validation.compare_mjcf_urdf import compare_mjcf_urdf

        print(f"\n{'─' * 60}")
        print("Running MJCF ↔ URDF validation...")
        print(f"{'─' * 60}")
        compare_mjcf_urdf(str(mjcf_path), str(urdf_path))
    except ImportError:
        print(
            "\nWarning: could not import compare_mjcf_urdf for validation.",
            file=sys.stderr,
        )
    except Exception as e:
        print(f"\nValidation error: {e}", file=sys.stderr)


# ════════════════════════════════════════════════════════════════════════════
# CLI Entry Point
# ════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Convert URDF/XACRO to MuJoCo MJCF XML.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Examples:
  # Directory convention (auto-discover URDF, output to mjcf/, resolve meshes)
  ros2 run rtc_tools urdf_to_mjcf --robot-dir ur5e_description/robots/ur5e

  # Specify URDF within robot directory
  ros2 run rtc_tools urdf_to_mjcf --robot-dir robots/ur5e --urdf-file ur5e_with_hand.urdf.xacro

  # Explicit input/output (legacy mode)
  ros2 run rtc_tools urdf_to_mjcf --input robot.urdf --output robot.xml

  # With scene generation and validation
  ros2 run rtc_tools urdf_to_mjcf --robot-dir robots/ur5e --scene --validate

  # Xacro with arguments
  ros2 run rtc_tools urdf_to_mjcf --input robot.xacro --xacro-args name:=ur5e
""",
    )

    # --- Robot directory mode ---
    dir_group = parser.add_argument_group("directory convention")
    dir_group.add_argument(
        "--robot-dir",
        type=Path,
        default=None,
        help=(
            "Robot directory containing urdf/, mjcf/, meshes/ subdirectories. "
            "Enables automatic path resolution."
        ),
    )
    dir_group.add_argument(
        "--urdf-file",
        type=str,
        default=None,
        help="URDF filename within <robot-dir>/urdf/ (auto-detected if omitted).",
    )

    # --- Explicit path mode ---
    path_group = parser.add_argument_group("explicit paths")
    path_group.add_argument(
        "--input",
        "-i",
        type=Path,
        default=None,
        help="Path to input URDF or XACRO file.",
    )
    path_group.add_argument(
        "--output",
        "-o",
        type=Path,
        default=None,
        help="Path to output MJCF XML file (default: <input_stem>_mjcf.xml).",
    )
    path_group.add_argument(
        "--meshdir",
        "-m",
        type=Path,
        default=None,
        help="Directory containing mesh files (default: auto-detect).",
    )

    # --- Options ---
    parser.add_argument(
        "--xacro-args",
        nargs="*",
        default=None,
        help="Extra xacro arguments in key:=value format.",
    )
    parser.add_argument(
        "--scene",
        action="store_true",
        help="Also generate a scene.xml with floor, lights, and <include>.",
    )
    parser.add_argument(
        "--validate",
        action="store_true",
        help="Run MJCF ↔ URDF parameter comparison after conversion.",
    )

    args = parser.parse_args()

    # Resolve paths
    if args.robot_dir is not None:
        input_path, output_path, meshdir = resolve_robot_dir_paths(
            args.robot_dir, args.urdf_file,
        )
        # Allow explicit overrides
        if args.output is not None:
            output_path = args.output
        if args.meshdir is not None:
            meshdir = args.meshdir.resolve()
    elif args.input is not None:
        input_path = args.input
        output_path = args.output
        meshdir = args.meshdir
    else:
        parser.error("Either --robot-dir or --input is required.")
        return

    convert_urdf_to_mjcf(
        input_path=input_path,
        output_path=output_path,
        xacro_args=args.xacro_args,
        meshdir=meshdir,
        generate_scene_file=args.scene,
        run_validation=args.validate,
    )


if __name__ == "__main__":
    main()
