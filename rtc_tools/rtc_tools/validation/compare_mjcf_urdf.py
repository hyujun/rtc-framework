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
      --robot-pkg robot_descriptions --robot-name <robot_name>

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
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    pass

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


def _detect_root_class(root: ET.Element) -> str | None:
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
    root_class_override: str | None = None,
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

    # ── Actuator → joint forcerange map ──
    #
    # urdf_to_mjcf writes ``<actuator><general joint="J" forcerange="...">``
    # directly (without relying on default-class inheritance), so the
    # authoritative effort value lives there rather than in <default>.
    # Build a lookup once; fall back to the default-class value if a joint
    # has no actuator entry.
    actuator_forcerange: dict[str, float] = {}
    for act_root in root.findall("actuator"):
        for act in act_root:
            jname_attr = act.get("joint")
            if not jname_attr:
                continue
            fr_str = act.get("forcerange")
            if fr_str is None:
                cls = act.get("class", root_class or "")
                fr_str = resolve_general_defaults(cls).get("forcerange")
            if fr_str is None:
                continue
            fr = _parse_floats(fr_str)
            if len(fr) == 2:
                actuator_forcerange[jname_attr] = fr[1]

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

            # Effort (forcerange): prefer actuator-level value, fall back to
            # default-class.  If neither exists, leave at 0 to flag MJCF has
            # no force limit declared rather than fabricate a default.
            if jname in actuator_forcerange:
                jp.effort = actuator_forcerange[jname]
            elif "forcerange" in gdefaults:
                fr = _parse_floats(gdefaults["forcerange"])
                jp.effort = fr[1] if len(fr) == 2 else 0.0
            else:
                jp.effort = 0.0

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


def _urdf_principal_moments(params: "InertialParams") -> list[float]:
    """Return URDF inertia eigenvalues sorted descending.

    Builds the full 3x3 symmetric inertia matrix from URDF ``ixx, iyy, izz,
    ixy, ixz, iyz`` and returns its eigenvalues.  Principal moments are a
    rotation-invariant signature, so they compare cleanly against MJCF's
    ``diaginertia`` regardless of how either side oriented the inertial
    frame.

    Falls back to ``sorted(diag_inertia)`` when NumPy is not available.
    """
    ixx, iyy, izz = params.diag_inertia
    ixy, ixz, iyz = params.off_diag_inertia
    try:
        import numpy as np  # noqa: PLC0415

        I = np.array(
            [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]],
            dtype=float,
        )
        eigs = np.linalg.eigvalsh(I)
        return sorted((float(v) for v in eigs), reverse=True)
    except ImportError:
        return sorted((ixx, iyy, izz), reverse=True)


# ── Physical plausibility (URDF inertia vs collision-shape mass distribution) ─


def _rpy_to_rotmat(rpy: tuple[float, float, float]):
    import numpy as np  # noqa: PLC0415

    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return (
        np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        @ np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        @ np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    )


def _shape_volume_and_local_inertia(shape_elem: ET.Element):
    """Return (volume, I_local_unit_mass) for a URDF collision/visual primitive.

    The returned ``I_local_unit_mass`` is the inertia tensor of the shape with
    unit mass, expressed in the shape's local frame about its centroid.
    Returns ``(0.0, None)`` for shapes we can't size (notably ``<mesh>``).
    """
    import numpy as np  # noqa: PLC0415

    box = shape_elem.find("box")
    if box is not None:
        sx, sy, sz = (float(v) for v in box.get("size", "0 0 0").split())
        vol = sx * sy * sz
        I = np.diag(
            [
                (sy * sy + sz * sz) / 12.0,
                (sx * sx + sz * sz) / 12.0,
                (sx * sx + sy * sy) / 12.0,
            ]
        )
        return vol, I

    cyl = shape_elem.find("cylinder")
    if cyl is not None:
        r = float(cyl.get("radius", "0"))
        h = float(cyl.get("length", "0"))
        vol = math.pi * r * r * h
        Ixy = (3.0 * r * r + h * h) / 12.0
        Iz = r * r / 2.0
        return vol, np.diag([Ixy, Ixy, Iz])

    sph = shape_elem.find("sphere")
    if sph is not None:
        r = float(sph.get("radius", "0"))
        vol = 4.0 / 3.0 * math.pi * r * r * r
        Ixyz = 2.0 / 5.0 * r * r
        return vol, np.diag([Ixyz, Ixyz, Ixyz])

    return 0.0, None


def _urdf_link_collision_inertia(
    urdf_root: ET.Element,
    link_name: str,
    total_mass: float,
) -> tuple[float, int]:
    """Estimate the inertia trace of a URDF link from its collision geometry.

    The link's declared mass is distributed across collision primitives in
    proportion to their volume (uniform-density assumption), each primitive's
    inertia is computed in its local frame, rotated into the link frame, and
    shifted to the link origin via the parallel-axis theorem.  We return only
    the trace ``Ixx + Iyy + Izz`` — a rotation-invariant scalar that lets us
    compare against any URDF inertia frame without worrying about axes.

    Returns:
        ``(estimated_trace, n_primitives_used)``.  ``estimated_trace`` is 0
        when no sizable primitives are present (typically mesh-only links).
    """
    try:
        import numpy as np  # noqa: PLC0415
    except ImportError:
        return 0.0, 0

    link = next(
        (l for l in urdf_root.findall("link") if l.get("name") == link_name),
        None,
    )
    if link is None:
        return 0.0, 0

    primitives = []
    for col in link.findall("collision"):
        geom = col.find("geometry")
        if geom is None:
            continue
        vol, I_local = _shape_volume_and_local_inertia(geom)
        if I_local is None or vol <= 0.0:
            continue
        origin = col.find("origin")
        xyz = (0.0, 0.0, 0.0)
        rpy = (0.0, 0.0, 0.0)
        if origin is not None:
            xyz = tuple(float(v) for v in origin.get("xyz", "0 0 0").split())
            rpy = tuple(float(v) for v in origin.get("rpy", "0 0 0").split())
        primitives.append((vol, I_local, xyz, rpy))

    if not primitives:
        return 0.0, 0

    total_vol = sum(p[0] for p in primitives)
    if total_vol <= 0.0:
        return 0.0, 0

    I_total = np.zeros((3, 3))
    for vol, I_local, xyz, rpy in primitives:
        m_i = total_mass * vol / total_vol
        R = _rpy_to_rotmat(rpy)
        I_at_origin_centroid = R @ (m_i * I_local) @ R.T
        r = np.asarray(xyz, dtype=float)
        I_total += I_at_origin_centroid + m_i * (np.dot(r, r) * np.eye(3) - np.outer(r, r))

    return float(np.trace(I_total)), len(primitives)


def _check_inertia_plausibility(
    urdf_root: ET.Element,
    urdf_link_name: str,
    urdf_ip: "InertialParams",
    *,
    ratio_low: float = 0.5,
    ratio_high: float = 2.0,
) -> str:
    """Return a WARN message string when URDF inertia looks physically off.

    Compares the trace of URDF's declared inertia (about its inertial origin,
    lifted to the link origin via parallel-axis) to the trace estimated from
    the link's collision primitives.  If the ratio falls outside
    ``[ratio_low, ratio_high]`` the URDF inertia is likely a placeholder
    (point-mass at link origin, copy-paste from another robot, etc.).

    Returns "" when:
      * no sizable collision primitives exist (mesh-only links),
      * mass is zero,
      * the ratio is in the acceptable band.

    The check is informational — it cannot tell whether the declared inertia
    or the collision geometry is the source of truth.
    """
    if urdf_ip.mass <= 0.0:
        return ""

    try:
        import numpy as np  # noqa: PLC0415
    except ImportError:
        return ""

    geom_trace, n_prim = _urdf_link_collision_inertia(urdf_root, urdf_link_name, urdf_ip.mass)
    if n_prim == 0 or geom_trace <= 0.0:
        return ""

    # URDF declared trace, lifted from inertial origin to link origin via Steiner.
    ixx, iyy, izz = urdf_ip.diag_inertia
    declared_trace = ixx + iyy + izz
    r = urdf_ip.origin_xyz
    declared_trace_at_link = declared_trace + 2.0 * urdf_ip.mass * (
        r[0] * r[0] + r[1] * r[1] + r[2] * r[2]
    )

    if declared_trace_at_link <= 0.0:
        return ""

    ratio = geom_trace / declared_trace_at_link
    if ratio_low <= ratio <= ratio_high:
        return ""

    direction = "smaller than" if ratio > 1.0 else "larger than"
    return (
        f"    [WARN] URDF inertia looks physically implausible: "
        f"declared trace={_fmt(declared_trace_at_link)} is "
        f"{ratio:.2f}x {direction} the collision-geometry estimate "
        f"({_fmt(geom_trace)}, from {n_prim} primitive(s)).  Likely a "
        f"placeholder — sim dynamics will not match physical link."
    )


# ── Structural comparison (counts, mass conservation, missing links) ──────────


def _urdf_link_mass(urdf_root: ET.Element) -> dict[str, float]:
    """Return ``{link_name: mass}`` for every URDF link (0.0 when no inertial)."""
    result: dict[str, float] = {}
    for link in urdf_root.findall("link"):
        name = link.get("name")
        if not name:
            continue
        inertial = link.find("inertial")
        mass = 0.0
        if inertial is not None:
            m = inertial.find("mass")
            if m is not None:
                mass = float(m.get("value", "0"))
        result[name] = mass
    return result


def _urdf_active_joints(urdf_root: ET.Element) -> list[str]:
    """Return URDF joint names that contribute DoF (non-fixed)."""
    return [
        j.get("name", "")
        for j in urdf_root.findall("joint")
        if j.get("type") in ("revolute", "continuous", "prismatic") and j.get("name")
    ]


def _urdf_geom_counts(urdf_root: ET.Element) -> tuple[int, int]:
    """Return ``(visual_count, collision_count)`` across the URDF."""
    return (sum(1 for _ in urdf_root.iter("visual")), sum(1 for _ in urdf_root.iter("collision")))


def _compare_structure(
    mjcf_path: Path,
    urdf_path: Path,
    tolerance: float,
) -> tuple[int, int]:
    """Compare structural counts and mass conservation between MJCF and URDF.

    Uses ``mujoco.MjModel`` to read the *compiled* MJCF (matching what the
    simulator actually sees), so fixed-link fusion, missing visual meshes,
    or broken mesh paths surface as concrete diffs.  Falls back to plain
    XML counts when ``mujoco`` is not importable.

    Reports:
      * Body count: URDF link count vs MJCF nbody (minus world).
      * DoF / actuator count: URDF non-fixed joints vs MJCF nv / nu.
      * Total mass: sum of URDF link masses vs sum of MJCF body_mass.
      * Geom count: URDF (visual + collision) vs MJCF ngeom.
      * Mesh asset count: distinct URDF mesh files vs MJCF nmesh.
      * Missing links: URDF links absent from MJCF body list — these
        usually indicate MuJoCo's ``fusestatic`` optimization absorbed
        the link, taking its mass with it.

    Returns:
        ``(mismatches, warnings)`` counts contributed by this section.
    """
    urdf_root = ET.parse(urdf_path).getroot()
    urdf_link_mass = _urdf_link_mass(urdf_root)
    urdf_links = set(urdf_link_mass.keys())
    urdf_active_joints = _urdf_active_joints(urdf_root)
    urdf_visual, urdf_collision = _urdf_geom_counts(urdf_root)
    urdf_total_mass = sum(urdf_link_mass.values())
    urdf_mesh_files = {
        m.get("filename", "").rsplit("/", 1)[-1]
        for m in urdf_root.iter("mesh")
        if m.get("filename")
    }

    print("\n--- Structural Comparison (MuJoCo-compiled model vs URDF) ---\n")

    mismatches = 0
    warnings = 0

    try:
        import mujoco  # noqa: PLC0415
    except ImportError:
        print(
            "  [WARN] 'mujoco' module not importable — skipping compiled-model "
            "structural comparison.  Install with `pip install mujoco` for "
            "full validation."
        )
        return 0, 1

    try:
        model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    except Exception as e:
        print(f"  [MISMATCH] MJCF failed to compile under MuJoCo: {e}")
        return 1, 0

    mjcf_bodies = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(model.nbody)
    }
    mjcf_bodies.discard("world")
    mjcf_total_mass = float(sum(model.body_mass))

    # Body / link count
    expected_bodies = len(urdf_links)
    actual_bodies = model.nbody - 1  # exclude world
    if actual_bodies != expected_bodies:
        print(
            f"  [MISMATCH] body count: URDF links={expected_bodies}  "
            f"MJCF bodies (excl. world)={actual_bodies}"
        )
        mismatches += 1
    else:
        print(f"  body count: {actual_bodies}  OK")

    # DoF / actuator
    if model.nv != len(urdf_active_joints):
        print(
            f"  [MISMATCH] DoF: URDF non-fixed joints={len(urdf_active_joints)}  "
            f"MJCF nv={model.nv}"
        )
        mismatches += 1
    else:
        print(f"  DoF: {model.nv}  OK")

    if model.nu != len(urdf_active_joints):
        print(
            f"  [WARN] actuator count: URDF non-fixed joints={len(urdf_active_joints)}  "
            f"MJCF nu={model.nu} (MJCF may intentionally omit actuators)"
        )
        warnings += 1
    else:
        print(f"  actuators: {model.nu}  OK")

    # Total mass
    mass_delta = abs(mjcf_total_mass - urdf_total_mass)
    if mass_delta > max(tolerance, 1e-4 * max(urdf_total_mass, 1e-9)):
        print(
            f"  [MISMATCH] total mass: URDF={_fmt(urdf_total_mass)}  "
            f"MJCF={_fmt(mjcf_total_mass)}  (delta={_fmt(mass_delta)})"
        )
        mismatches += 1
    else:
        print(f"  total mass: URDF={_fmt(urdf_total_mass)}  MJCF={_fmt(mjcf_total_mass)}  OK")

    # Geom count
    urdf_geom_total = urdf_visual + urdf_collision
    if model.ngeom != urdf_geom_total:
        print(
            f"  [WARN] geom count: URDF visual+collision={urdf_geom_total}  "
            f"MJCF ngeom={model.ngeom}"
        )
        warnings += 1
    else:
        print(f"  geoms: {model.ngeom}  OK")

    # Mesh asset count
    if urdf_mesh_files:
        if model.nmesh != len(urdf_mesh_files):
            print(
                f"  [WARN] mesh asset count: URDF distinct files={len(urdf_mesh_files)}  "
                f"MJCF nmesh={model.nmesh}  "
                "(missing visuals usually mean MuJoCo's discardvisual was on)"
            )
            warnings += 1
        else:
            print(f"  meshes: {model.nmesh}  OK")

    # Missing links — the strong signal for fusestatic regression
    missing = sorted(urdf_links - mjcf_bodies)
    if missing:
        print(f"\n  [MISMATCH] URDF links absent from MJCF ({len(missing)}):")
        for name in missing:
            m = urdf_link_mass.get(name, 0.0)
            note = f"  mass={_fmt(m)} kg lost" if m > 0 else "  (massless)"
            print(f"    - {name}{note}")
        print(
            "    Hint: MuJoCo's compiler defaults absorb fixed-joint child "
            'links into their parent (fusestatic="true") and silently drop '
            "the root link when it is empty.  Inject "
            '<mujoco><compiler fusestatic="false"/></mujoco> in the URDF '
            "to preserve 1:1 link → body mapping."
        )
        mismatches += 1

    print()
    return mismatches, warnings


def compare(
    mjcf_path: Path,
    urdf_path: Path,
    link_map: dict[str, str],
    joint_names: list[str],
    tolerance: float = 1e-4,
    mjcf_class: str | None = None,
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

    # ── Structural comparison (counts, mass, missing links) ──
    s_mis, s_warn = _compare_structure(mjcf_path, urdf_path, tolerance)
    mismatches += s_mis
    warnings += s_warn

    # ── Link inertial comparison ──
    print("\n--- Link Inertial Parameters ---\n")

    # Cache parsed URDF root so the plausibility helper doesn't reparse per link.
    urdf_root_cached = ET.parse(urdf_path).getroot()

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

        # Inertia comparison via principal moments
        #
        # URDF stores the full inertia tensor in the (possibly rotated)
        # inertial frame.  MJCF normalises this to the principal-axis frame:
        # ``diaginertia`` holds the eigenvalues sorted descending, and an
        # optional ``quat`` encodes the rotation from body frame to that
        # principal frame.  Comparing raw Ixx/Iyy/Izz fails when either side
        # is rotated; comparing sorted eigenvalues is rotation-invariant.
        u_eigs = _urdf_principal_moments(urdf_ip)
        m_eigs = sorted((float(v) for v in mjcf_ip.diag_inertia), reverse=True)

        eig_pairs = list(zip(m_eigs, u_eigs))
        eig_match = all(_close(m, u, tolerance) for m, u in eig_pairs)

        if eig_match:
            print(
                f"    principal moments: "
                f"[{_fmt(m_eigs[0])}, {_fmt(m_eigs[1])}, {_fmt(m_eigs[2])}]  OK"
            )
        else:
            print(
                f"    INERTIA MISMATCH (principal moments):"
                f"  MJCF=[{_fmt(m_eigs[0])}, {_fmt(m_eigs[1])}, {_fmt(m_eigs[2])}]"
                f"  URDF=[{_fmt(u_eigs[0])}, {_fmt(u_eigs[1])}, {_fmt(u_eigs[2])}]"
            )
            for i, lbl in enumerate(("λ1", "λ2", "λ3")):
                d = abs(m_eigs[i] - u_eigs[i])
                if d > tolerance:
                    print(f"      {lbl} delta={_fmt(d)}")
            mismatches += 1

        # Informational notes (non-error): off-diagonal magnitude / frame rotation
        off_diag_mag = math.sqrt(sum(x * x for x in urdf_ip.off_diag_inertia))
        if off_diag_mag > tolerance:
            print(
                f"    [NOTE] URDF has non-zero off-diagonal inertia "
                f"(|off-diag|={_fmt(off_diag_mag)}); compared via principal moments."
            )
        if urdf_ip.origin_rpy != [0.0, 0.0, 0.0]:
            rpy_str = " ".join(_fmt(v) for v in urdf_ip.origin_rpy)
            print(
                f"    [NOTE] URDF inertial frame rotated rpy=[{rpy_str}]; "
                "compared via principal moments."
            )

        # Physical plausibility: compare URDF-declared inertia trace to the
        # estimate from collision-shape mass distribution.  Both quantities
        # are taken about the link origin and are rotation-invariant, so a
        # large ratio (declared << geometric) usually means the URDF inertia
        # was a placeholder, not a proper rigid-body computation.
        plaus_warn = _check_inertia_plausibility(urdf_root_cached, urdf_name, urdf_ip)
        if plaus_warn:
            print(plaus_warn)
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
            print("           [NOTE] MJCF and URDF use different coordinate conventions.")
            warnings += 1

        # Armature (MJCF-only)
        if mjcf_jp.armature > 0:
            print(f"    armature: {_fmt(mjcf_jp.armature)} (MJCF-only, no URDF equivalent)")

        print()

    # ── Summary ──
    print("=" * 78)
    print("  SUMMARY")
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


def compare_mjcf_urdf(
    mjcf_path: str | Path,
    urdf_path: str | Path,
    tolerance: float = 1e-4,
    robot_label: str = "",
) -> int:
    """Convenience wrapper: auto-detect link map + joint list and run compare.

    Intended for callers that have an MJCF/URDF pair and want a single-call
    validation — e.g. ``urdf_to_mjcf`` invoking validation right after
    conversion.  Falls back gracefully when no shared links/joints are
    detected (returns 0 with a printed warning rather than crashing).

    Returns:
        Number of mismatches reported by :func:`compare`.
    """
    mjcf_path = Path(mjcf_path)
    urdf_path = Path(urdf_path)

    link_map = _detect_link_mapping(mjcf_path, urdf_path)
    joint_names = _detect_joints(mjcf_path, urdf_path)

    if not link_map and not joint_names:
        print(
            "Warning: no shared inertial links or joints found between MJCF "
            "and URDF — nothing to compare.",
            file=sys.stderr,
        )
        return 0

    return compare(
        mjcf_path,
        urdf_path,
        link_map=link_map,
        joint_names=joint_names,
        tolerance=tolerance,
        robot_label=robot_label,
    )


# ── CLI ────────────────────────────────────────────────────────────────────────


def _resolve_robot_layout(
    pkg: str,
    robot_name: str,
) -> tuple[Path | None, Path | None]:
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
        "'robot_descriptions'). Used with --robot-name to resolve "
        "<pkg>/robots/<robot>/{mjcf,urdf}/<robot>.{xml,urdf}.",
    )
    parser.add_argument(
        "--robot-name",
        type=str,
        default=None,
        help="Robot name used as both directory and file stem under --robot-pkg.",
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
