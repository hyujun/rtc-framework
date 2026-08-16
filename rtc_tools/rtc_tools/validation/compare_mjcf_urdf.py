#!/usr/bin/env python3
"""Compare MJCF and URDF physics parameters for a robot.

Parses both model files and reports differences in:
  - Link mass
  - Link inertia (principal moments via eigenvalue decomposition)
  - Joint position limits (range; sign-swapped for anti-parallel axes)
  - Joint effort / force limits
  - Joint axis vectors (world-frame FK, parallelism check)
  - Joint axis lines (world-frame FK; a revolute origin may slide along its
    own axis without changing the physics, so only the perpendicular offset
    counts as a mismatch — see _axis_line_offset)

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

  # Declare the pairs whose MJCF body name differs from the URDF link name
  # (YAML/JSON, {<mjcf_body>: <urdf_link>, ...}).  It is an exception list —
  # every other same-name pair is still compared:
  ros2 run rtc_tools compare_mjcf_urdf --link-map link_map.yaml ...

  # Declare a common base frame when the two files mount the robot
  # differently (see _resolve_alignment). For ur5e, MJCF's world is the UR
  # "Base" (DH) frame while the URDF's world is REP-103 'base_link':
  ros2 run rtc_tools compare_mjcf_urdf --align-frames world base ...
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
    # Inertial frame rotation — URDF rpy
    origin_rpy: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    # Inertial frame rotation — MJCF quat (w, x, y, z); identity = [1,0,0,0]
    origin_quat: list = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])


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


# ── SE(3) transform utilities (pure-Python, no numpy dependency) ─────────────
#
# URDF encodes joint frames as (xyz, rpy) while MJCF uses (pos, quat).
# Comparing raw position/axis vectors is meaningless when the parent frames
# differ in orientation.  These helpers compute Forward Kinematics (FK) to
# world frame so that position and axis can be compared correctly.


def _identity_3x3() -> list[list[float]]:
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def _rpy_to_rot3(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    """ZYX Euler angles (URDF convention) to 3x3 rotation matrix."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _quat_to_rot3(w: float, x: float, y: float, z: float) -> list[list[float]]:
    """Quaternion (w, x, y, z) to 3x3 rotation matrix."""
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-15:
        return _identity_3x3()
    w, x, y, z = w / n, x / n, y / n, z / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


def _mat_mul_33(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mat_transpose_33(m: list[list[float]]) -> list[list[float]]:
    return [[m[j][i] for j in range(3)] for i in range(3)]


def _rotate_tensor_33(r: list[list[float]], t: list[list[float]]) -> list[list[float]]:
    """``R · T · Rᵀ`` — express a second-order tensor in a rotated frame."""
    return _mat_mul_33(_mat_mul_33(r, t), _mat_transpose_33(r))


def _mat_vec_3(r: list[list[float]], v: list[float]) -> list[float]:
    return [sum(r[i][k] * v[k] for k in range(3)) for i in range(3)]


def _vec_add_3(a: list[float], b: list[float]) -> list[float]:
    return [a[i] + b[i] for i in range(3)]


def _compose_transform(
    r_parent: list[list[float]],
    p_parent: list[float],
    r_local: list[list[float]],
    p_local: list[float],
) -> tuple[list[list[float]], list[float]]:
    """Compose parent and local SE(3) transforms.

    Returns ``(R_world, p_world)`` where::

        R_world = R_parent @ R_local
        p_world = R_parent @ p_local + p_parent
    """
    r_world = _mat_mul_33(r_parent, r_local)
    p_rotated = _mat_vec_3(r_parent, p_local)
    p_world = _vec_add_3(p_parent, p_rotated)
    return r_world, p_world


def _vec_close_3(a: list[float], b: list[float], tol: float) -> bool:
    return all(math.isclose(a[i], b[i], abs_tol=tol) for i in range(3))


_AXIS_PARALLEL_COS_TOL = 1e-6


def _axes_parallel(a: list[float], b: list[float]) -> tuple[bool, bool]:
    """Check if two 3-vectors are parallel (same rotation line).

    Returns ``(is_parallel, same_sign)``.  ``is_parallel`` is True when the
    vectors point along the same line (either same or opposite direction).
    ``same_sign`` is True when they also point in the same direction.
    Anti-parallel axes (same_sign=False) represent the same physical rotation
    axis but with opposite positive-rotation convention.

    Uses a fixed cosine tolerance (``_AXIS_PARALLEL_COS_TOL``) independent of
    the position tolerance so that axis parallelism is not coupled to the
    caller's metric-space tolerance.
    """
    dot = sum(a[i] * b[i] for i in range(3))
    mag_a = math.sqrt(sum(v * v for v in a))
    mag_b = math.sqrt(sum(v * v for v in b))
    if mag_a < 1e-15 or mag_b < 1e-15:
        return False, False
    cos_angle = dot / (mag_a * mag_b)
    is_parallel = abs(abs(cos_angle) - 1.0) < _AXIS_PARALLEL_COS_TOL
    same_sign = cos_angle > 0
    return is_parallel, same_sign


def _mjcf_body_rotation(body: ET.Element) -> list[list[float]]:
    """Extract body-frame rotation from any MuJoCo orientation attribute.

    MuJoCo supports ``quat``, ``axisangle``, ``euler``, ``xyaxes``, and
    ``zaxis``.  Returns identity when none are present.
    """
    quat_str = body.get("quat")
    if quat_str:
        q = _parse_floats(quat_str)
        return _quat_to_rot3(q[0], q[1], q[2], q[3])

    axisangle_str = body.get("axisangle")
    if axisangle_str:
        vals = _parse_floats(axisangle_str)
        ax, ay, az, angle = vals[0], vals[1], vals[2], vals[3]
        n = math.sqrt(ax * ax + ay * ay + az * az)
        if n < 1e-15:
            return _identity_3x3()
        ax, ay, az = ax / n, ay / n, az / n
        c, s = math.cos(angle), math.sin(angle)
        t = 1.0 - c
        return [
            [t * ax * ax + c, t * ax * ay - s * az, t * ax * az + s * ay],
            [t * ax * ay + s * az, t * ay * ay + c, t * ay * az - s * ax],
            [t * ax * az - s * ay, t * ay * az + s * ax, t * az * az + c],
        ]

    euler_str = body.get("euler")
    if euler_str:
        vals = _parse_floats(euler_str)
        return _rpy_to_rot3(vals[0], vals[1], vals[2])

    xyaxes_str = body.get("xyaxes")
    if xyaxes_str:
        vals = _parse_floats(xyaxes_str)
        x = vals[0:3]
        y = vals[3:6]
        z = [
            x[1] * y[2] - x[2] * y[1],
            x[2] * y[0] - x[0] * y[2],
            x[0] * y[1] - x[1] * y[0],
        ]
        mag_z = math.sqrt(z[0] * z[0] + z[1] * z[1] + z[2] * z[2])
        if mag_z > 1e-15:
            z = [z[i] / mag_z for i in range(3)]
        return [x, y, z]

    zaxis_str = body.get("zaxis")
    if zaxis_str:
        z = _parse_floats(zaxis_str)
        mag = math.sqrt(sum(v * v for v in z))
        if mag > 1e-15:
            z = [v / mag for v in z]
        if abs(z[2]) < 0.99:
            up = [0.0, 0.0, 1.0]
        else:
            up = [1.0, 0.0, 0.0]
        x = [
            up[1] * z[2] - up[2] * z[1],
            up[2] * z[0] - up[0] * z[2],
            up[0] * z[1] - up[1] * z[0],
        ]
        mag_x = math.sqrt(sum(v * v for v in x))
        if mag_x > 1e-15:
            x = [v / mag_x for v in x]
        y = [
            z[1] * x[2] - z[2] * x[1],
            z[2] * x[0] - z[0] * x[2],
            z[0] * x[1] - z[1] * x[0],
        ]
        return [x, y, z]

    return _identity_3x3()


# ── FK chain computation ─────────────────────────────────────────────────────


def _urdf_joint_tree(
    path: Path,
) -> tuple[dict[str, tuple[str, str, list, list, list]], dict[str, str]]:
    """Parse the URDF joint graph once.

    Returns ``(all_joints, child_to_joint)`` where ``all_joints`` maps a joint
    name to ``(parent_link, child_link, xyz, rpy, axis)`` and
    ``child_to_joint`` maps a link to the joint that parents it.  Shared by the
    link-frame and joint-frame FK helpers so the graph is walked once.
    """
    root = ET.parse(path).getroot()

    all_joints: dict[str, tuple[str, str, list, list, list]] = {}
    child_to_joint: dict[str, str] = {}

    for jelem in root.findall("joint"):
        jname = jelem.get("name", "")
        parent_elem = jelem.find("parent")
        child_elem = jelem.find("child")
        if parent_elem is None or child_elem is None:
            continue
        parent_link = parent_elem.get("link", "")
        child_link = child_elem.get("link", "")

        origin = jelem.find("origin")
        xyz = _parse_floats(origin.get("xyz", "0 0 0")) if origin is not None else [0, 0, 0]
        rpy = _parse_floats(origin.get("rpy", "0 0 0")) if origin is not None else [0, 0, 0]

        axis_elem = jelem.find("axis")
        axis = _parse_floats(axis_elem.get("xyz", "0 0 1")) if axis_elem is not None else [0, 0, 1]

        all_joints[jname] = (parent_link, child_link, xyz, rpy, axis)
        child_to_joint[child_link] = jname

    return all_joints, child_to_joint


def _urdf_link_world_frames(
    path: Path,
) -> dict[str, tuple[list[list[float]], list[float]]]:
    """Compute the world-frame pose ``(R, p)`` of every URDF link at zero config.

    Traces the full kinematic chain (including fixed joints) outward from each
    root link.  Links that are not a joint's child (roots) and links caught in
    a cycle resolve to identity, matching the joint-FK helper's behaviour.
    """
    root = ET.parse(path).getroot()
    all_joints, child_to_joint = _urdf_joint_tree(path)

    link_frames: dict[str, tuple[list[list[float]], list[float]]] = {}
    _in_progress: set[str] = set()

    def get_link_frame(link_name: str) -> tuple[list[list[float]], list[float]]:
        if link_name in link_frames:
            return link_frames[link_name]
        if link_name not in child_to_joint or link_name in _in_progress:
            link_frames[link_name] = (_identity_3x3(), [0.0, 0.0, 0.0])
            return link_frames[link_name]
        _in_progress.add(link_name)
        jname = child_to_joint[link_name]
        parent_link, _, xyz, rpy, _ = all_joints[jname]
        parent_r, parent_p = get_link_frame(parent_link)
        local_r = _rpy_to_rot3(rpy[0], rpy[1], rpy[2])
        child_r, child_p = _compose_transform(parent_r, parent_p, local_r, xyz)
        link_frames[link_name] = (child_r, child_p)
        _in_progress.discard(link_name)
        return child_r, child_p

    for link_elem in root.findall("link"):
        name = link_elem.get("name")
        if name:
            get_link_frame(name)
    for parent_link, child_link, _, _, _ in all_joints.values():
        get_link_frame(parent_link)
        get_link_frame(child_link)

    return link_frames


def _compute_urdf_world_frames(
    path: Path,
    joint_names: set[str],
) -> dict[str, tuple[list[float], list[float]]]:
    """Compute world-frame (position, axis) for each URDF joint via FK.

    Traces the full URDF kinematic chain (including fixed joints) from the
    root link to each target joint, accumulating SE(3) transforms.  Returns
    joint position and axis expressed in the world frame at zero configuration.
    """
    all_joints, _ = _urdf_joint_tree(path)
    link_frames = _urdf_link_world_frames(path)

    result: dict[str, tuple[list[float], list[float]]] = {}
    for jname in joint_names:
        if jname not in all_joints:
            continue
        parent_link, _child_link, xyz, rpy, axis = all_joints[jname]
        parent_r, parent_p = link_frames.get(parent_link, (_identity_3x3(), [0.0, 0.0, 0.0]))
        local_r = _rpy_to_rot3(rpy[0], rpy[1], rpy[2])
        joint_r, joint_p = _compose_transform(parent_r, parent_p, local_r, xyz)
        world_axis = _mat_vec_3(joint_r, axis)
        result[jname] = (joint_p, world_axis)

    return result


def _compute_mjcf_world_frames(
    path: Path,
    joint_names: set[str],
) -> dict[str, tuple[list[float], list[float]]]:
    """Compute world-frame (position, axis) for each MJCF joint via FK.

    Walks the ``<worldbody>`` hierarchy, accumulating body ``pos`` and
    orientation transforms (quat/euler/axisangle/xyaxes/zaxis), and
    extracts joint axis in the body frame.  Default-class axis inheritance
    (including ``childclass``) is resolved so explicit and inherited axes
    are treated uniformly.
    """
    root = ET.parse(path).getroot()

    defaults: dict[str, dict] = {}
    for default_elem in root.iter("default"):
        cls = default_elem.get("class")
        if cls is None:
            continue
        joint_elem = default_elem.find("joint")
        if joint_elem is not None:
            defaults[cls] = dict(joint_elem.attrib)

    root_class = _detect_root_class(root)

    def resolve_axis(cls_name: str, joint_elem: ET.Element) -> list[float]:
        explicit = joint_elem.get("axis")
        if explicit is not None:
            return _parse_floats(explicit)
        merged: dict[str, str] = {}
        if root_class and root_class in defaults:
            merged.update(defaults[root_class])
        if cls_name in defaults:
            merged.update(defaults[cls_name])
        return _parse_floats(merged.get("axis", "0 1 0"))

    result: dict[str, tuple[list[float], list[float]]] = {}

    def traverse(
        elem: ET.Element,
        parent_r: list[list[float]],
        parent_p: list[float],
        active_childclass: str,
    ) -> None:
        for body in elem.findall("body"):
            pos = _parse_floats(body.get("pos", "0 0 0"))
            r_local = _mjcf_body_rotation(body)

            r_world, p_world = _compose_transform(parent_r, parent_p, r_local, pos)

            body_childclass = body.get("childclass", active_childclass)

            for joint_elem in body.findall("joint"):
                jname = joint_elem.get("name")
                if jname not in joint_names:
                    continue
                cls = joint_elem.get("class", body_childclass)
                axis_local = resolve_axis(cls, joint_elem)
                world_axis = _mat_vec_3(r_world, axis_local)

                joint_pos_str = joint_elem.get("pos")
                if joint_pos_str:
                    jp = _parse_floats(joint_pos_str)
                    j_world_pos = _vec_add_3(p_world, _mat_vec_3(r_world, jp))
                else:
                    j_world_pos = list(p_world)

                result[jname] = (j_world_pos, world_axis)

            traverse(body, r_world, p_world, body_childclass)

    worldbody = root.find("worldbody")
    if worldbody is not None:
        traverse(worldbody, _identity_3x3(), [0.0, 0.0, 0.0], root_class or "")

    return result


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


def _mjcf_world_frames_by_kind(
    path: Path,
) -> tuple[
    dict[str, tuple[list[list[float]], list[float]]],
    dict[str, tuple[list[list[float]], list[float]]],
]:
    """Walk ``<worldbody>`` once, returning ``(bodies, sites)`` world poses.

    Sites are collected too because a model's tool/flange frame is often a
    ``<site>`` rather than a body (ur5e's ``attachment_site``), and that frame
    is the only thing a downstream offset like the DH ``d6`` moves — it changes
    neither any joint axis line nor any link COM, so nothing else in this file
    can see it (#392).
    """
    root = ET.parse(path).getroot()
    bodies: dict[str, tuple[list[list[float]], list[float]]] = {
        "world": (_identity_3x3(), [0.0, 0.0, 0.0])
    }
    sites: dict[str, tuple[list[list[float]], list[float]]] = {}

    def traverse(elem: ET.Element, parent_r: list[list[float]], parent_p: list[float]) -> None:
        for body in elem.findall("body"):
            pos = _parse_floats(body.get("pos", "0 0 0"))
            r_local = _mjcf_body_rotation(body)
            r_world, p_world = _compose_transform(parent_r, parent_p, r_local, pos)
            name = body.get("name")
            if name:
                bodies[name] = (r_world, p_world)
            for site in body.findall("site"):
                sname = site.get("name")
                if not sname:
                    continue
                s_pos = _parse_floats(site.get("pos", "0 0 0"))
                s_rot = _mjcf_body_rotation(site)
                sites[sname] = _compose_transform(r_world, p_world, s_rot, s_pos)
            traverse(body, r_world, p_world)

    worldbody = root.find("worldbody")
    if worldbody is not None:
        traverse(worldbody, _identity_3x3(), [0.0, 0.0, 0.0])

    return bodies, sites


def _mjcf_body_world_frames(
    path: Path,
) -> dict[str, tuple[list[list[float]], list[float]]]:
    """Compute the world-frame pose ``(R, p)`` of every MJCF body.

    ``"world"`` is included as identity so it can be named as an alignment
    frame — in MJCF the worldbody *is* a frame, whereas a URDF's equivalent is
    a named root link.
    """
    return _mjcf_world_frames_by_kind(path)[0]


def _mjcf_named_frames(
    path: Path,
) -> dict[str, tuple[list[list[float]], list[float]]]:
    """Bodies and sites in one lookup; a body wins a name clash.

    MuJoCo namespaces bodies and sites separately, so the same name can denote
    both.  Bodies take precedence and the clash is vanishingly rare in practice;
    callers that need to disambiguate should rename in the model.
    """
    bodies, sites = _mjcf_world_frames_by_kind(path)
    return {**sites, **bodies}


def _invert_transform(
    r: list[list[float]], p: list[float]
) -> tuple[list[list[float]], list[float]]:
    """Invert an SE(3) transform: ``(R, p)`` -> ``(Rᵀ, -Rᵀp)``."""
    r_inv = [[r[j][i] for j in range(3)] for i in range(3)]
    p_inv = [-v for v in _mat_vec_3(r_inv, p)]
    return r_inv, p_inv


def _resolve_alignment(
    mjcf_path: Path,
    urdf_path: Path,
    mjcf_frame: str,
    urdf_frame: str,
) -> tuple[list[list[float]], list[float]]:
    """Return ``T_urdfworld←mjcfworld`` making two declared frames coincide.

    The two formats need not agree on where "world" is.  MJCF mounts the
    robot's root body wherever the scene author chose; a URDF's world is its
    root link.  Comparing world-frame FK across that gap reports a whole-robot
    offset that is a *mounting* convention, not a model divergence — on the
    ur5e pair it manifested as an apparent sign flip in x on all six joints
    (#392), because MJCF's world is the UR "Base" (DH) frame while the URDF's
    is REP-103 ``base_link``.

    The caller declares one physically identical frame per side rather than
    letting the tool infer the transform: a best-fit would silently absorb a
    genuine divergence into the fit, which is exactly the failure mode this
    sensor exists to catch.

    Raises:
        SystemExit: when either declared frame name is absent from its file.
    """
    mjcf_frames = _mjcf_body_world_frames(mjcf_path)
    urdf_frames = _urdf_link_world_frames(urdf_path)

    if mjcf_frame not in mjcf_frames:
        raise SystemExit(
            f"--align-frames: MJCF has no body named '{mjcf_frame}'. "
            f"Available: {', '.join(sorted(mjcf_frames))}"
        )
    if urdf_frame not in urdf_frames:
        raise SystemExit(
            f"--align-frames: URDF has no link named '{urdf_frame}'. "
            f"Available: {', '.join(sorted(urdf_frames))}"
        )

    r_m, p_m = mjcf_frames[mjcf_frame]
    r_u, p_u = urdf_frames[urdf_frame]
    r_m_inv, p_m_inv = _invert_transform(r_m, p_m)
    return _compose_transform(r_u, p_u, r_m_inv, p_m_inv)


_REVOLUTE_URDF_TYPES = frozenset({"revolute", "continuous"})


def _detect_joint_types(
    mjcf_path: Path,
    urdf_path: Path,
    joint_names: set[str],
) -> dict[str, str]:
    """Classify each joint ``"revolute"`` only when BOTH files agree it is one.

    The distinction matters for the position comparison: a revolute joint's
    origin may sit anywhere on its own axis, but a prismatic joint's origin
    sets its zero position, so a prismatic origin must be compared as a point.
    Anything not confirmed revolute on both sides falls back to the stricter
    point comparison.
    """
    urdf_root = ET.parse(urdf_path).getroot()
    urdf_types = {
        j.get("name"): j.get("type", "") for j in urdf_root.findall("joint") if j.get("name")
    }

    mjcf_root = ET.parse(mjcf_path).getroot()
    root_class = _detect_root_class(mjcf_root)
    class_types: dict[str, str] = {}
    for default_elem in mjcf_root.iter("default"):
        cls = default_elem.get("class")
        if cls is None:
            continue
        jelem = default_elem.find("joint")
        if jelem is not None and jelem.get("type"):
            class_types[cls] = jelem.get("type", "")

    mjcf_types: dict[str, str] = {}
    for body in mjcf_root.iter("body"):
        childclass = body.get("childclass", root_class or "")
        for jelem in body.findall("joint"):
            name = jelem.get("name")
            if not name:
                continue
            explicit = jelem.get("type")
            if explicit:
                mjcf_types[name] = explicit
            else:
                cls = jelem.get("class", childclass)
                # MuJoCo's <joint> type defaults to "hinge".
                mjcf_types[name] = class_types.get(cls, class_types.get(root_class or "", "hinge"))

    return {
        jname: (
            "revolute"
            if urdf_types.get(jname) in _REVOLUTE_URDF_TYPES
            and mjcf_types.get(jname, "hinge") == "hinge"
            else "other"
        )
        for jname in joint_names
    }


def _axis_line_offset(
    p_mjcf: list[float],
    p_urdf: list[float],
    axis: list[float],
) -> tuple[float, list[float], float]:
    """Split the origin difference into along-axis and perpendicular parts.

    A revolute joint is physically defined by its axis *line*, not by where the
    origin sits on that line — sliding the origin along the axis changes no
    dynamics and no reachable set.  MJCF and URDF routinely place the origin
    differently on the same line (MJCF at the visual-mesh datum, URDF at the DH
    frame), so only the perpendicular part is evidence of a real disagreement.

    On the ur5e pair the along-axis part reaches 138 mm while every
    perpendicular offset stays under 0.8 mm (#392): comparing origins as points
    reported six mismatches where the axis lines in fact agree.

    Returns:
        ``(along, perp_vec, perp_magnitude)``.  A degenerate (zero-length) axis
        yields ``along=0`` and the full difference as the perpendicular part,
        so a broken axis cannot silently pass.
    """
    dv = [p_urdf[i] - p_mjcf[i] for i in range(3)]
    mag = math.sqrt(sum(v * v for v in axis))
    if mag < 1e-15:
        return 0.0, dv, math.sqrt(sum(v * v for v in dv))
    unit = [v / mag for v in axis]
    along = sum(dv[i] * unit[i] for i in range(3))
    perp = [dv[i] - along * unit[i] for i in range(3)]
    return along, perp, math.sqrt(sum(v * v for v in perp))


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
            params.origin_quat = _parse_floats(quat)
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

        inertia = np.array(
            [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]],
            dtype=float,
        )
        eigs = np.linalg.eigvalsh(inertia)
        return sorted((float(v) for v in eigs), reverse=True)
    except ImportError:
        return sorted((ixx, iyy, izz), reverse=True)


# ── Fused links (MJCF folded a fixed-joint child into its parent) ────────────


def _urdf_inertia_tensor(params: "InertialParams") -> list[list[float]]:
    """Full symmetric inertia matrix as stored by URDF, in its inertial frame."""
    ixx, iyy, izz = params.diag_inertia
    ixy, ixz, iyz = params.off_diag_inertia
    return [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]


def _shift_inertia(tensor: list[list[float]], mass: float, d: list[float]) -> list[list[float]]:
    """Parallel-axis theorem: inertia about a point ``d`` away from the COM."""
    d2 = sum(x * x for x in d)
    return [
        [tensor[i][j] + mass * ((d2 if i == j else 0.0) - d[i] * d[j]) for j in range(3)]
        for i in range(3)
    ]


def _urdf_joint_types(path: Path) -> dict[str, str]:
    """``{<joint_name>: <type>}`` for every URDF joint."""
    root = ET.parse(path).getroot()
    return {j.get("name", ""): j.get("type", "") for j in root.findall("joint")}


def _fuse_urdf_inertials(
    parent_link: str,
    child_links: list[str],
    urdf_links: dict[str, "InertialParams"],
    urdf_frames: dict[str, tuple[list[list[float]], list[float]]],
) -> "InertialParams":
    """Combine a URDF parent and the links folded into it, as MJCF sees them.

    Composition runs in the URDF **world** frame at zero configuration and the
    result is rotated back into the parent link frame.  Going through world is
    not a detour: it makes the fixed joints' rotations fall out of the existing
    FK, so an arbitrarily deep chain of folded links needs no extra bookkeeping
    and an ``rpy`` on any of those joints is handled for free.

    The result is expressed with ``origin_rpy = 0`` because the tensor is
    already in the parent link frame — the comparison downstream takes
    principal moments, so no principal-axis normalisation is needed here.
    """
    members = [parent_link, *child_links]

    total_mass = sum(urdf_links[name].mass for name in members)
    com_world = [0.0, 0.0, 0.0]
    coms: dict[str, list[float]] = {}
    for name in members:
        ip = urdf_links[name]
        r, p = urdf_frames[name]
        c = _vec_add_3(p, _mat_vec_3(r, ip.origin_xyz))
        coms[name] = c
        for k in range(3):
            com_world[k] += ip.mass * c[k]
    if total_mass > 0:
        com_world = [v / total_mass for v in com_world]

    inertia_world = [[0.0] * 3 for _ in range(3)]
    for name in members:
        ip = urdf_links[name]
        r, _ = urdf_frames[name]
        # About its own COM: inertial frame → link frame → world frame.
        t = _rotate_tensor_33(_rpy_to_rot3(*ip.origin_rpy), _urdf_inertia_tensor(ip))
        t = _rotate_tensor_33(r, t)
        d = [coms[name][k] - com_world[k] for k in range(3)]
        shifted = _shift_inertia(t, ip.mass, d)
        for i in range(3):
            for j in range(3):
                inertia_world[i][j] += shifted[i][j]

    r_p, p_p = urdf_frames[parent_link]
    r_pt = _mat_transpose_33(r_p)
    com_local = _mat_vec_3(r_pt, [com_world[k] - p_p[k] for k in range(3)])
    inertia_local = _rotate_tensor_33(r_pt, inertia_world)

    return InertialParams(
        mass=total_mass,
        origin_xyz=com_local,
        diag_inertia=[inertia_local[0][0], inertia_local[1][1], inertia_local[2][2]],
        off_diag_inertia=[inertia_local[0][1], inertia_local[0][2], inertia_local[1][2]],
        origin_rpy=[0.0, 0.0, 0.0],
    )


def _validate_fuse(
    urdf_path: Path,
    fuse: dict[str, list[str]],
    link_map: dict[str, str],
) -> list[str]:
    """Reject fuse declarations that would disable checking instead of explaining it.

    A declaration says "MuJoCo folded these fixed children into that body".  If
    it is allowed to name a *movable* child, it becomes a way to make any
    mismatch disappear — the shape of hole #392 closed.  So the path from each
    child up to the parent must consist of fixed joints only.
    """
    all_joints, child_to_joint = _urdf_joint_tree(urdf_path)
    joint_types = _urdf_joint_types(urdf_path)
    urdf_link_names = {
        el.get("name") for el in ET.parse(urdf_path).getroot().findall("link") if el.get("name")
    }
    errors: list[str] = []

    for mjcf_body, children in fuse.items():
        if mjcf_body not in link_map:
            errors.append(f"fuse: '{mjcf_body}' is not a mapped MJCF body")
            continue
        parent_link = link_map[mjcf_body]
        for child in children:
            if child not in urdf_link_names:
                errors.append(
                    f"fuse: '{mjcf_body}' names URDF link '{child}', which does not exist"
                )
                continue
            errors.extend(
                _fixed_path_error(child, parent_link, all_joints, child_to_joint, joint_types)
            )

    return errors


def _fixed_path_error(
    child: str,
    parent_link: str,
    all_joints: dict[str, tuple[str, str, list, list, list]],
    child_to_joint: dict[str, str],
    joint_types: dict[str, str],
) -> list[str]:
    """Empty when ``child`` reaches ``parent_link`` through fixed joints only."""
    node = child
    for _ in range(len(child_to_joint) + 1):
        if node == parent_link:
            return []
        jname = child_to_joint.get(node)
        if jname is None:
            break
        if joint_types.get(jname) != "fixed":
            return [
                f"fuse: '{child}' reaches '{parent_link}' through joint '{jname}' of "
                f"type '{joint_types.get(jname)}' — only fixed joints may be folded"
            ]
        node = all_joints[jname][0]
    return [f"fuse: '{child}' is not a descendant of '{parent_link}'"]


# ── Physical plausibility (URDF inertia vs collision-shape mass distribution) ─


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
        inertia = np.diag(
            [
                (sy * sy + sz * sz) / 12.0,
                (sx * sx + sz * sz) / 12.0,
                (sx * sx + sy * sy) / 12.0,
            ]
        )
        return vol, inertia

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
        (el for el in urdf_root.findall("link") if el.get("name") == link_name),
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
        R = np.asarray(_rpy_to_rot3(rpy[0], rpy[1], rpy[2]), dtype=float)
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
        import numpy as np  # noqa: PLC0415, F401  (availability probe only)
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
    link_map: dict[str, str] | None = None,
    fused_children: set[str] | None = None,
) -> tuple[int, int, int]:
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

    Args:
        link_map: Optional ``{<mjcf_body>: <urdf_link>}`` mapping.  Used only
            to resolve deliberate naming differences in the missing-link
            check; ``None`` compares raw names.
        fused_children: URDF links declared as folded into a parent body.  They
            have no body of their own by design, so they are excused from the
            missing-link check — their mass is verified by the composition in
            :func:`compare` instead, not waived (#412).

    Returns:
        ``(mismatches, warnings, unverified)`` counts contributed by this
        section.  ``unverified`` is non-zero when a check could not be
        performed at all — a distinct outcome from "passed".
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
        # UNVERIFIED, not "passed".  Without the compiled model this section
        # checks nothing at all — body count, total mass and missing links all
        # go unexamined — yet the run would still end in "Mismatches: 0" and
        # exit 0.  That is the false green #392 is about, and it is not
        # hypothetical: colcon invokes pytest as `/usr/bin/python3 -m pytest`
        # (colcon's own shebang), which cannot see the venv where mujoco is
        # installed, so this branch is what the repo's test harness actually
        # takes.  Callers that gate on this run must pass
        # fail_on_unverified=True.
        print(
            "  [UNVERIFIED] 'mujoco' module not importable — the compiled-model "
            "structural comparison did not run.  Body count, total mass and "
            "missing-link checks were NOT performed.  Use an interpreter that "
            "can import mujoco (this repo installs it into .venv via "
            "requirements.lock)."
        )
        return 0, 0, 1

    try:
        model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    except Exception as e:
        print(f"  [MISMATCH] MJCF failed to compile under MuJoCo: {e}")
        return 1, 0, 0

    mjcf_bodies = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(model.nbody)
    }
    mjcf_bodies.discard("world")
    mjcf_total_mass = float(sum(model.body_mass))

    # ── Which URDF links does the compiled MJCF actually carry? ──
    #
    # A link counts as present when link_map points at a compiled body, or when
    # its own name is a compiled body that the map has not already claimed for
    # a different link.  That last clause matters: ur5e's URDF has BOTH a
    # massless frame named `base` and the 4 kg `base_link_inertia`, while the
    # MJCF has one body named `base` that is the 4 kg link.  Matching on raw
    # names alone silently paired the massless frame with the massive body and
    # counted the real link as lost (#392).
    claimed_mjcf = {mjcf_name for mjcf_name in (link_map or {}) if mjcf_name in mjcf_bodies}
    mapped_urdf_links = {
        urdf_name for mjcf_name, urdf_name in (link_map or {}).items() if mjcf_name in mjcf_bodies
    }
    present = mapped_urdf_links | {
        name for name in urdf_links if name in mjcf_bodies and name not in claimed_mjcf
    }
    missing = sorted(urdf_links - present)

    # Massless URDF links are pure coordinate frames (tool0, flange, ft_frame,
    # the world anchor).  MuJoCo not creating a body for one is correct
    # behaviour, not a defect, so they are reported but not counted.  This is
    # NOT a relaxation of the fusestatic signal #385 cared about: a link that
    # carries mass and disappears still counts, because its mass was silently
    # folded into a parent.
    # Links declared as folded into a parent are excused here and verified by
    # the composition in compare() instead — the mass is checked, just against
    # the combined body rather than one of its own.
    fused = fused_children or set()
    missing_fused = sorted(n for n in missing if n in fused)
    missing_massive = [n for n in missing if n not in fused and urdf_link_mass.get(n, 0.0) > 0]
    missing_massless = [n for n in missing if n not in fused and urdf_link_mass.get(n, 0.0) <= 0]

    # Body / link count, on the same basis as the missing-link check above:
    # massless frames MuJoCo legitimately dropped are excused, everything else
    # must have a body.  Excusing them wholesale would break models that DO
    # materialise their massless frames (iiwa7 has 1, leap_hand 5).
    expected_bodies = len(urdf_links) - len(missing_massless) - len(missing_fused)
    actual_bodies = model.nbody - 1  # exclude world
    excused_parts = []
    if missing_massless:
        excused_parts.append(f"{len(missing_massless)} massless frame(s)")
    if missing_fused:
        excused_parts.append(f"{len(missing_fused)} fused link(s)")
    excused = f" ({', '.join(excused_parts)} excused)" if excused_parts else ""
    if actual_bodies != expected_bodies:
        print(
            f"  [MISMATCH] body count: URDF links={expected_bodies}{excused}  "
            f"MJCF bodies (excl. world)={actual_bodies}"
        )
        mismatches += 1
    else:
        print(f"  body count: {actual_bodies}  OK{excused}")

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

    # Missing links — the strong signal for fusestatic regression.
    #
    # `present` (computed above) *translates* through link_map, it does not
    # *filter* — a body that fusestatic really absorbed is absent from the
    # compiled model, so it lands in `missing` even for a mapped link (#385).
    if missing_massive:
        print(f"\n  [MISMATCH] URDF links with mass absent from MJCF ({len(missing_massive)}):")
        for name in missing_massive:
            print(f"    - {name}  mass={_fmt(urdf_link_mass.get(name, 0.0))} kg lost")
        print(
            "    Hint: MuJoCo's compiler defaults absorb fixed-joint child "
            'links into their parent (fusestatic="true") and silently drop '
            "the root link when it is empty.  Inject "
            '<mujoco><compiler fusestatic="false"/></mujoco> in the URDF '
            "to preserve 1:1 link → body mapping."
        )
        mismatches += 1

    if missing_fused:
        # Reported, not counted: these were declared as folded into a parent
        # and their mass is checked there, by composition (#412).
        print(f"\n  [FUSED] URDF links folded into a parent body ({len(missing_fused)}):")
        print(f"    {', '.join(missing_fused)}")

    if missing_massless:
        # Reported, not counted: a zero-mass URDF link is a pure coordinate
        # frame and MuJoCo is right not to give it a body.  Still listed so the
        # set stays visible rather than silently disappearing from the report.
        print(f"\n  [NOTE] massless URDF frames with no MJCF body ({len(missing_massless)}):")
        print(f"    {', '.join(missing_massless)}")
        print(
            "    Not counted as mismatches — these carry no physics, so MuJoCo "
            "omitting them changes nothing the simulator integrates."
        )

    print()
    return mismatches, warnings, 0


def compare(
    mjcf_path: Path,
    urdf_path: Path,
    link_map: dict[str, str],
    joint_names: list[str],
    tolerance: float = 1e-4,
    mjcf_class: str | None = None,
    robot_label: str = "",
    align: tuple[list[list[float]], list[float]] | None = None,
    fail_on_unverified: bool = False,
    tip_frames: tuple[str, str] | None = None,
    fuse: dict[str, list[str]] | None = None,
) -> int:
    """Compare MJCF and URDF parameters. Returns number of mismatches.

    Args:
        mjcf_path / urdf_path: Model file paths.
        link_map: Mapping {<mjcf_body_name>: <urdf_link_name>, ...} declaring
            the pairs whose names *differ*.  It is an exception list, not the
            worklist: :func:`_resolve_link_map` widens it with every remaining
            same-name pair, so a partial map cannot silently narrow the
            comparison (#411).
        joint_names: Joint names to compare (assumed identical in both files).
        tolerance: Numerical tolerance.
        mjcf_class: Optional MJCF default class override; auto-detected if None.
        robot_label: Optional human label printed in the header.
        align: Optional ``(R, p)`` mapping MJCF world coordinates into the URDF
            world frame, from :func:`_resolve_alignment`.  ``None`` assumes the
            two world frames already coincide.
        fail_on_unverified: Count checks that could not run (e.g. the compiled-
            model structural comparison when ``mujoco`` is unimportable) as
            mismatches, so the caller's exit status reflects them.  Any gate
            that treats a zero return as "validated" must set this.
        tip_frames: Optional ``(<mjcf_body_or_site>, <urdf_link>)`` naming the
            tool frame on each side.  Closes the chain-tail blind spot: an
            offset past the last joint (the DH ``d6``) moves neither a joint
            axis line nor a link COM, so nothing else here can observe it.
        fuse: Optional ``{<mjcf_body>: [<urdf_link>, ...]}`` declaring URDF
            links that MuJoCo folded into that body through fixed joints.  The
            named links are composed into the parent before comparison instead
            of being reported as lost mass plus an overweight parent (#412).
    """
    link_map, unpaired_mjcf, unpaired_urdf = _resolve_link_map(mjcf_path, urdf_path, link_map)
    fuse = dict(fuse or {})
    mjcf_link_names = set(link_map.keys())
    # The folded links must be parsed too — they are inputs to the composition
    # even though no MJCF body corresponds to them.
    fused_children = {child for children in fuse.values() for child in children}
    urdf_link_names = set(link_map.values()) | fused_children
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
    if align is not None:
        print("  Base alignment: declared (MJCF world FK lifted into URDF world frame)")
    print("=" * 78)

    # ── Fused-link declarations ──
    #
    # Checked before anything uses them: a declaration that names a movable
    # child would otherwise silently absorb a real mismatch into a composition.
    if fuse:
        fuse_errors = _validate_fuse(urdf_path, fuse, link_map)
        for err in fuse_errors:
            print(f"  [MISMATCH] {err}")
        mismatches += len(fuse_errors)
        # Drop the bad entries so the rest of the run still reports something
        # useful rather than composing from a declaration we just rejected.
        if fuse_errors:
            fuse = {b: c for b, c in fuse.items() if b in link_map}
            fused_children = {child for children in fuse.values() for child in children}

    # ── Structural comparison (counts, mass, missing links) ──
    s_mis, s_warn, unverified = _compare_structure(
        mjcf_path, urdf_path, tolerance, link_map, fused_children
    )
    mismatches += s_mis
    warnings += s_warn

    # ── Link inertial comparison ──
    print("\n--- Link Inertial Parameters ---\n")

    # State the coverage before the per-link output.  Without it, a comparison
    # that reached only half the model is indistinguishable from one that
    # reached all of it — both end in "Mismatches: 0" (#411).
    print(f"  Link pairs compared: {len(link_map)}")
    if fuse:
        folded = ", ".join(
            f"{body} ← {'+'.join(children)}" for body, children in sorted(fuse.items())
        )
        print(f"  [FUSED] composed before comparison ({len(fuse)}): {folded}")
    # A folded link has a counterpart — it is inside its parent, not missing.
    unpaired_urdf = [name for name in unpaired_urdf if name not in fused_children]
    if unpaired_mjcf:
        print(
            f"  [NOTE] no name counterpart, not compared per-link — "
            f"MJCF bodies ({len(unpaired_mjcf)}): {', '.join(unpaired_mjcf)}"
        )
    if unpaired_urdf:
        print(
            f"  [NOTE] no name counterpart, not compared per-link — "
            f"URDF links ({len(unpaired_urdf)}): {', '.join(unpaired_urdf)}"
        )
    print()

    # Cache parsed URDF root so the plausibility helper doesn't reparse per link.
    urdf_root_cached = ET.parse(urdf_path).getroot()

    # Link frames for the COM comparison below.  A COM is only comparable in a
    # frame both files agree on: MJCF puts a body origin where the visual mesh
    # wants it and URDF puts it on the DH point, so the same physical COM reads
    # differently in each local frame (ur5e forearm_link: 0.242 m apart, all of
    # it convention).  Lifting both to world at zero configuration removes that
    # — the same reasoning that made joints compare as axis *lines* (#392).
    mjcf_link_frames = _mjcf_body_world_frames(mjcf_path)
    urdf_link_frames = _urdf_link_world_frames(urdf_path)
    r_com_align, p_com_align = align if align is not None else (_identity_3x3(), [0.0, 0.0, 0.0])

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

        # Compose the folded children in before comparing, so mass / COM /
        # principal moments are all checked against the same rigid body the
        # compiled MJCF carries.  Composing only the mass would let a wrong
        # fused inertia through under a declaration — the opposite of the point.
        children = fuse.get(mjcf_name)
        if children:
            missing_frames = [
                n
                for n in (urdf_name, *children)
                if n not in urdf_link_frames or n not in urdf_links
            ]
            if missing_frames:
                print(f"  [WARN] {label}: cannot compose fused links {missing_frames}")
                warnings += 1
            else:
                urdf_ip = _fuse_urdf_inertials(urdf_name, children, urdf_links, urdf_link_frames)
                label = f"{label} + {' + '.join(children)}"

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

        # Centre of mass, in the URDF world frame at zero configuration.
        #
        # Nothing else here observes it: principal moments are taken about the
        # COM, so translating it leaves them unchanged, and total mass is blind
        # to where the mass sits.  A link whose COM slid while its mass and
        # inertia stayed put used to pass silently (#416).
        mjcf_frame = mjcf_link_frames.get(mjcf_name)
        urdf_frame = urdf_link_frames.get(urdf_name)
        if mjcf_frame is None or urdf_frame is None:
            print("    [WARN] COM not compared: link frame missing on one side")
            warnings += 1
        else:
            r_m, p_m = mjcf_frame
            com_m = _vec_add_3(p_m, _mat_vec_3(r_m, mjcf_ip.origin_xyz))
            com_m = _vec_add_3(p_com_align, _mat_vec_3(r_com_align, com_m))
            r_u, p_u = urdf_frame
            com_u = _vec_add_3(p_u, _mat_vec_3(r_u, urdf_ip.origin_xyz))
            if _vec_close_3(com_m, com_u, tolerance):
                print(f"    COM (world): [{', '.join(_fmt(v) for v in com_u)}]  OK")
            else:
                delta = max(abs(a - b) for a, b in zip(com_m, com_u, strict=False))
                print(
                    f"    COM MISMATCH (world):"
                    f"  MJCF=[{', '.join(_fmt(v) for v in com_m)}]"
                    f"  URDF=[{', '.join(_fmt(v) for v in com_u)}]"
                    f"  (max delta={_fmt(delta)})"
                )
                mismatches += 1

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

        eig_pairs = list(zip(m_eigs, u_eigs, strict=False))
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
        if mjcf_ip.origin_quat != [1.0, 0.0, 0.0, 0.0]:
            quat_str = " ".join(_fmt(v) for v in mjcf_ip.origin_quat)
            print(
                f"    [NOTE] MJCF inertial frame rotated quat=[{quat_str}]; "
                "compared via principal moments."
            )

        # Physical plausibility: compare URDF-declared inertia trace to the
        # estimate from collision-shape mass distribution.  Both quantities
        # are taken about the link origin and are rotation-invariant, so a
        # large ratio (declared << geometric) usually means the URDF inertia
        # was a placeholder, not a proper rigid-body computation.
        if children:
            # Not run, and said so rather than skipped quietly.  The estimate
            # only sees `urdf_name`'s own collision shapes, while `urdf_ip` now
            # also carries the folded children's mass at a distance — the two
            # sides measure different bodies, so every fused link would warn.
            #
            # Reviving it means summing the fused children's collision shapes
            # into the estimate so both sides describe the same body.  Left
            # undone deliberately: the check is an extra, and a wrong estimate
            # here would warn on every fused link, which is worse than the
            # honest [NOTE] below.
            print(
                "    [NOTE] plausibility not checked: the collision-geometry "
                "estimate covers only the parent link, not the fused body"
            )
        else:
            plaus_warn = _check_inertia_plausibility(urdf_root_cached, urdf_name, urdf_ip)
            if plaus_warn:
                print(plaus_warn)
                warnings += 1

        print()

    # ── Joint comparison ──
    #
    # URDF and MJCF may use different local coordinate conventions at each
    # joint (e.g. URDF keeps all axes as Z and rotates frames via RPY;
    # MJCF minimises body rotations and uses Y-axis instead).  Comparing
    # raw local-frame position/axis vectors produces false mismatches.
    #
    # We compute Forward Kinematics (FK) to world frame for both files and
    # compare positions and axes there — a frame-invariant comparison.

    print("--- Joint Parameters ---\n")

    urdf_world = _compute_urdf_world_frames(urdf_path, joint_set)
    mjcf_world = _compute_mjcf_world_frames(mjcf_path, joint_set)
    joint_types = _detect_joint_types(mjcf_path, urdf_path, joint_set)

    # Lift MJCF world FK into the URDF world frame when the caller declared an
    # alignment; identity otherwise.  See _resolve_alignment for why the two
    # files' world frames can legitimately differ.
    r_align, p_align = align if align is not None else (_identity_3x3(), [0.0, 0.0, 0.0])
    mjcf_world = {
        name: (
            _vec_add_3(p_align, _mat_vec_3(r_align, pos)),
            _mat_vec_3(r_align, axis),
        )
        for name, (pos, axis) in mjcf_world.items()
    }

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

        # World-frame axis comparison (FK-based) — run first so anti-parallel
        # detection can inform the range comparison below.
        u_wf = urdf_world.get(jname)
        m_wf = mjcf_world.get(jname)
        axis_anti_parallel = False

        if u_wf is not None and m_wf is not None:
            u_world_pos, u_world_axis = u_wf
            m_world_pos, m_world_axis = m_wf

            is_parallel, same_sign = _axes_parallel(m_world_axis, u_world_axis)
            if not is_parallel:
                m_axis_s = " ".join(_fmt(v) for v in m_world_axis)
                u_axis_s = " ".join(_fmt(v) for v in u_world_axis)
                print(f"    AXIS MISMATCH (world frame):  MJCF=[{m_axis_s}]  URDF=[{u_axis_s}]")
                m_local = " ".join(_fmt(v) for v in mjcf_jp.axis)
                u_local = " ".join(_fmt(v) for v in urdf_jp.axis)
                print(f"      local axes: MJCF=[{m_local}]  URDF=[{u_local}]")
                mismatches += 1
            elif not same_sign:
                axis_anti_parallel = True
                axis_str = " ".join(_fmt(v) for v in m_world_axis)
                print(
                    f"    axis (world): [{axis_str}]  OK (anti-parallel — sign convention differs)"
                )
            else:
                axis_str = " ".join(_fmt(v) for v in m_world_axis)
                print(f"    axis (world): [{axis_str}]  OK")

            # World-frame position comparison (FK-based).
            #
            # For a revolute joint with a confirmed-parallel axis, the physical
            # object is the axis LINE — the origin may sit anywhere on it.  We
            # therefore compare the perpendicular offset between the two lines
            # and report the along-axis part as a note.  Comparing origins as
            # points instead reported six false mismatches on the ur5e pair
            # whose axis lines agree to sub-millimetre (#392).
            #
            # Prismatic joints (and anything whose axes are not parallel, where
            # "the line" is not well defined) keep the stricter point test: a
            # prismatic origin sets its zero position, so sliding it matters.
            if joint_types.get(jname) == "revolute" and is_parallel:
                along, _perp_vec, perp_mag = _axis_line_offset(
                    m_world_pos, u_world_pos, u_world_axis
                )
                if perp_mag > tolerance:
                    m_pos_s = " ".join(_fmt(v) for v in m_world_pos)
                    u_pos_s = " ".join(_fmt(v) for v in u_world_pos)
                    print(
                        f"    AXIS-LINE MISMATCH (world frame):  "
                        f"perpendicular offset={_fmt(perp_mag)}"
                    )
                    print(f"      origins: MJCF=[{m_pos_s}]  URDF=[{u_pos_s}]")
                    m_local_o = " ".join(_fmt(v) for v in mjcf_jp.origin_xyz)
                    u_local_o = " ".join(_fmt(v) for v in urdf_jp.origin_xyz)
                    print(f"      local origins: MJCF=[{m_local_o}]  URDF=[{u_local_o}]")
                    mismatches += 1
                else:
                    pos_str = " ".join(_fmt(v) for v in m_world_pos)
                    print(f"    axis line (world): through [{pos_str}]  OK")
                    if abs(along) > tolerance:
                        print(
                            f"    [NOTE] origins differ by {_fmt(along)} m along the joint "
                            "axis; not a disagreement (a revolute origin may sit anywhere "
                            "on its own axis)."
                        )
            elif _vec_close_3(m_world_pos, u_world_pos, tolerance):
                pos_str = " ".join(_fmt(v) for v in m_world_pos)
                print(f"    position (world): [{pos_str}]  OK")
            else:
                m_pos_s = " ".join(_fmt(v) for v in m_world_pos)
                u_pos_s = " ".join(_fmt(v) for v in u_world_pos)
                print(f"    POSITION MISMATCH (world frame):  MJCF=[{m_pos_s}]  URDF=[{u_pos_s}]")
                m_local_o = " ".join(_fmt(v) for v in mjcf_jp.origin_xyz)
                u_local_o = " ".join(_fmt(v) for v in urdf_jp.origin_xyz)
                print(f"      local origins: MJCF=[{m_local_o}]  URDF=[{u_local_o}]")
                mismatches += 1
        else:
            print("    [WARN] FK unavailable — falling back to local-frame comparison")
            warnings += 1
            axes_match = all(_close(mjcf_jp.axis[i], urdf_jp.axis[i], tolerance) for i in range(3))
            if not axes_match:
                m_axis = " ".join(_fmt(v) for v in mjcf_jp.axis)
                u_axis = " ".join(_fmt(v) for v in urdf_jp.axis)
                print(f"    AXIS MISMATCH (local):  MJCF=[{m_axis}]  URDF=[{u_axis}]")
                mismatches += 1
            origins_match = all(
                _close(mjcf_jp.origin_xyz[i], urdf_jp.origin_xyz[i], tolerance) for i in range(3)
            )
            if not origins_match:
                m_orig = " ".join(_fmt(v) for v in mjcf_jp.origin_xyz)
                u_orig = " ".join(_fmt(v) for v in urdf_jp.origin_xyz)
                print(f"    ORIGIN MISMATCH (local):  MJCF=[{m_orig}]  URDF=[{u_orig}]")
                warnings += 1

        # Position limits — when axes are anti-parallel, positive rotation in
        # one format is negative in the other, so limits must be negated and
        # swapped for a correct comparison.
        m_lower = mjcf_jp.lower
        m_upper = mjcf_jp.upper
        u_lower = urdf_jp.lower
        u_upper = urdf_jp.upper
        if axis_anti_parallel:
            u_lower, u_upper = -urdf_jp.upper, -urdf_jp.lower

        if not _close(m_lower, u_lower, tolerance) or not _close(m_upper, u_upper, tolerance):
            print(
                f"    RANGE MISMATCH:"
                f"  MJCF=[{_fmt(mjcf_jp.lower)}, {_fmt(mjcf_jp.upper)}]"
                f"  URDF=[{_fmt(urdf_jp.lower)}, {_fmt(urdf_jp.upper)}]"
            )
            if axis_anti_parallel:
                print(
                    f"      (compared with sign-swapped URDF limits"
                    f" [{_fmt(u_lower)}, {_fmt(u_upper)}] due to anti-parallel axes)"
                )
            mismatches += 1
        else:
            print(f"    range: [{_fmt(m_lower)}, {_fmt(m_upper)}]  OK")

        # Effort limits (frame-independent)
        if not _close(mjcf_jp.effort, urdf_jp.effort, tolerance):
            print(
                f"    EFFORT MISMATCH:  MJCF={_fmt(mjcf_jp.effort)}  URDF={_fmt(urdf_jp.effort)}"
            )
            mismatches += 1
        else:
            print(f"    effort: {_fmt(mjcf_jp.effort)}  OK")

        # Armature (MJCF-only)
        if mjcf_jp.armature > 0:
            print(f"    armature: {_fmt(mjcf_jp.armature)} (MJCF-only, no URDF equivalent)")

        print()

    # ── Tool / tip frame ──
    #
    # The joint section compares axis LINES, which is correct but blind to the
    # tail of the chain: an offset applied after the last joint slides along
    # that joint's own axis, and it shifts no COM either (measured).  Only the
    # tool frame moves, so it has to be compared directly (#392).
    if tip_frames:
        mjcf_tip, urdf_tip = tip_frames
        print("--- Tool Frame ---\n")
        m_frames = _mjcf_named_frames(mjcf_path)
        u_frames = _urdf_link_world_frames(urdf_path)
        if mjcf_tip not in m_frames:
            print(f"  [WARN] MJCF has no body or site named '{mjcf_tip}' — skipped")
            warnings += 1
        elif urdf_tip not in u_frames:
            print(f"  [WARN] URDF has no link named '{urdf_tip}' — skipped")
            warnings += 1
        else:
            r_m, p_m = m_frames[mjcf_tip]
            r_m = _mat_mul_33(r_align, r_m)
            p_m = _vec_add_3(p_align, _mat_vec_3(r_align, p_m))
            r_u, p_u = u_frames[urdf_tip]

            label = f"{mjcf_tip} (URDF: {urdf_tip})"
            if not _vec_close_3(p_m, p_u, tolerance):
                delta = math.sqrt(sum((p_m[i] - p_u[i]) ** 2 for i in range(3)))
                print(
                    f"  [{label}]\n"
                    f"    TIP POSITION MISMATCH:  MJCF=[{' '.join(_fmt(v) for v in p_m)}]"
                    f"  URDF=[{' '.join(_fmt(v) for v in p_u)}]  (|delta|={_fmt(delta)})"
                )
                mismatches += 1
            else:
                print(f"  [{label}]\n    position: [{' '.join(_fmt(v) for v in p_m)}]  OK")

            # Relative rotation angle: frame-invariant, so a differing tool-frame
            # convention shows up as an angle rather than as axis-by-axis noise.
            r_rel = _mat_mul_33([[r_m[j][i] for j in range(3)] for i in range(3)], r_u)
            cos_a = max(-1.0, min(1.0, (r_rel[0][0] + r_rel[1][1] + r_rel[2][2] - 1.0) / 2.0))
            angle = math.degrees(math.acos(cos_a))
            if angle > math.degrees(tolerance):
                print(f"    TIP ORIENTATION MISMATCH:  {_fmt(angle)} deg between the two frames")
                mismatches += 1
            else:
                print(f"    orientation: {_fmt(angle)} deg  OK")
        print()

    # ── Summary ──
    #
    # `unverified` is deliberately NOT folded into `warnings`: a warning means
    # "we looked and it is probably fine", whereas unverified means "we did not
    # look".  Collapsing them is how a skipped section ends up reported as a
    # pass (#392).
    print("=" * 78)
    print("  SUMMARY")
    print(f"  Mismatches: {mismatches}")
    print(f"  Warnings:   {warnings}")
    if unverified:
        print(f"  UNVERIFIED: {unverified}  (checks that could not run — see above)")

    if unverified and fail_on_unverified:
        mismatches += unverified
        print(
            f"  Result: {unverified} check(s) UNVERIFIED and --fail-on-unverified "
            "is set — treated as failure"
        )
    elif unverified:
        print(
            f"  Result: {mismatches} mismatch(es), but {unverified} check(s) never ran "
            "— this is NOT a clean pass"
        )
    elif mismatches == 0 and warnings == 0:
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
    align_frames: tuple[str, str] | None = None,
    fail_on_unverified: bool = False,
    tip_frames: tuple[str, str] | None = None,
) -> int:
    """Convenience wrapper: auto-detect link map + joint list and run compare.

    Intended for callers that have an MJCF/URDF pair and want a single-call
    validation — e.g. ``urdf_to_mjcf`` invoking validation right after
    conversion.  Falls back gracefully when no shared links/joints are
    detected (returns 0 with a printed warning rather than crashing).

    Args:
        align_frames: Optional ``(<mjcf_body>, <urdf_link>)`` naming one
            physically identical frame per side, for pairs whose world frames
            differ.  See :func:`_resolve_alignment`.

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
        align=(
            _resolve_alignment(mjcf_path, urdf_path, align_frames[0], align_frames[1])
            if align_frames
            else None
        ),
        fail_on_unverified=fail_on_unverified,
        tip_frames=tip_frames,
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


def _mjcf_inertial_bodies(mjcf_path: Path) -> set[str]:
    """MJCF body names carrying an explicit ``<inertial>``."""
    root = ET.parse(mjcf_path).getroot()
    return {
        b.get("name")
        for b in root.iter("body")
        if b.get("name") and b.find("inertial") is not None
    }


def _urdf_inertial_links(urdf_path: Path) -> set[str]:
    """URDF link names carrying an ``<inertial>``."""
    root = ET.parse(urdf_path).getroot()
    return {
        el.get("name")
        for el in root.findall("link")
        if el.get("name") and el.find("inertial") is not None
    }


def _detect_link_mapping(
    mjcf_path: Path,
    urdf_path: Path,
) -> dict[str, str]:
    """Build {<mjcf_body>: <urdf_link>} for bodies/links present in both files
    that carry an <inertial> child. Names that match exactly are paired; bodies
    with no same-named URDF link are skipped (caller can supply --link-map for
    cases where MJCF and URDF use different naming).
    """
    return {
        name: name
        for name in sorted(_mjcf_inertial_bodies(mjcf_path) & _urdf_inertial_links(urdf_path))
    }


def _resolve_link_map(
    mjcf_path: Path,
    urdf_path: Path,
    explicit: dict[str, str] | None,
) -> tuple[dict[str, str], list[str], list[str]]:
    """Widen an explicit link map with the same-name pairs it does not claim.

    An explicit map declares **exceptions** — pairs whose names differ, or a
    name that means different things on the two sides.  It is not the worklist.
    Treating it as one is a false green: on ur5e+hand, a 7-entry arm map left
    four real hand mass mismatches out of the per-link comparison, and nothing
    in the report said the comparison had been narrowed (#411).

    Explicit entries win over same-name pairing on both sides, which is what
    keeps ur5e's ``base: base_link_inertia`` from being undone by the URDF's
    unrelated massless ``base`` frame.

    Returns:
        ``(effective_map, unpaired_mjcf, unpaired_urdf)`` — the last two list
        bodies/links that carry an inertial but found no counterpart by name,
        so the caller can report what is *not* being compared.
    """
    mjcf_names = _mjcf_inertial_bodies(mjcf_path)
    urdf_names = _urdf_inertial_links(urdf_path)
    explicit = dict(explicit or {})

    # Same-name pairing fills in whatever the exception list does not cover.
    # A candidate whose URDF link an explicit entry already targets is dropped,
    # so ur5e's `base: base_link_inertia` cannot be undone by re-pairing
    # `base_link_inertia` with a same-named body.  Collisions on the MJCF side
    # need no guard: candidates are keyed by body name, so the update below
    # overwrites them (a guard there survives mutation — it is inert).
    claimed_urdf = set(explicit.values())
    effective = {
        name: name for name in sorted(mjcf_names & urdf_names) if name not in claimed_urdf
    }
    effective.update(explicit)

    unpaired_mjcf = sorted(mjcf_names - set(effective))
    unpaired_urdf = sorted(urdf_names - set(effective.values()))
    return effective, unpaired_mjcf, unpaired_urdf


def _detect_joints(mjcf_path: Path, urdf_path: Path) -> list[str]:
    """Joint names that appear in both files."""
    mjcf_root = ET.parse(mjcf_path).getroot()
    urdf_root = ET.parse(urdf_path).getroot()

    mjcf_joints = {j.get("name") for j in mjcf_root.iter("joint") if j.get("name")}
    urdf_joints = {j.get("name") for j in urdf_root.findall("joint") if j.get("name")}

    return sorted(mjcf_joints & urdf_joints)


def _load_link_map(path: Path) -> tuple[dict[str, str], dict[str, list[str]]]:
    """Load a link-name mapping, and any fused-link declarations, from YAML/JSON.

    Two accepted shapes::

        base: base_link_inertia                 # flat: name → name

        links:                                  # structured
          base: base_link_inertia
          fuse:
            index_dip_fe_link: [index_tip_link]

    The structured form is recognised by a value that is itself a mapping;
    flat files always map a name to a name, so the two cannot be confused and
    a robot may still own a link literally called ``links`` or ``fuse``.

    Returns:
        ``(links, fuse)`` where ``fuse`` maps an MJCF body to the URDF links
        MuJoCo folded into it.
    """
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

    if any(isinstance(v, dict) for v in data.values()):
        unknown = sorted(set(data) - {"links", "fuse"})
        if unknown:
            raise SystemExit(
                f"--link-map file {path}: unknown top-level key(s) {unknown}; "
                "the structured form accepts only 'links' and 'fuse'"
            )
        links_raw = data.get("links") or {}
        fuse_raw = data.get("fuse") or {}
    else:
        links_raw, fuse_raw = data, {}

    fuse: dict[str, list[str]] = {}
    for body, children in fuse_raw.items():
        if isinstance(children, str):
            children = [children]
        if not isinstance(children, list):
            raise SystemExit(
                f"--link-map file {path}: fuse['{body}'] must be a URDF link name or a list of them"
            )
        fuse[str(body)] = [str(c) for c in children]

    return {str(k): str(v) for k, v in links_raw.items()}, fuse


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
        help="YAML/JSON file with {<mjcf_body>: <urdf_link>, ...}. An "
        "exception list for pairs whose names differ, not a worklist: every "
        "remaining same-name pair is compared as well, so a partial file "
        "cannot narrow the comparison.",
    )
    parser.add_argument(
        "--joints",
        type=str,
        nargs="+",
        default=None,
        help="Explicit joint name list to compare. Auto-detected (MJCF ∩ URDF) if omitted.",
    )
    parser.add_argument(
        "--align-frames",
        type=str,
        nargs=2,
        metavar=("MJCF_FRAME", "URDF_FRAME"),
        default=None,
        help="Declare one physically identical frame per side so joint FK is "
        "compared in a common base frame: an MJCF body name (or 'world') and a "
        "URDF link name. Needed when the two files mount the robot differently "
        "-- e.g. for ur5e, MJCF's world is the UR 'Base' (DH) frame while the "
        "URDF's world is REP-103 'base_link', so `--align-frames world base` "
        "reconciles them. Note the names cross: MJCF body 'base' corresponds to "
        "URDF link 'base_link', NOT to URDF link 'base'. Omitted: the two world "
        "frames are assumed to coincide.",
    )
    parser.add_argument(
        "--tip-frames",
        type=str,
        nargs=2,
        metavar=("MJCF_FRAME", "URDF_FRAME"),
        default=None,
        help="Compare the tool frame: an MJCF body or site name and a URDF link "
        "name. Needed because the joint comparison uses axis LINES, which "
        "cannot see an offset applied past the last joint (the DH d6) -- it "
        "slides along that joint's own axis and moves no COM. For ur5e: "
        "`--tip-frames attachment_site tool0`.",
    )
    parser.add_argument(
        "--fail-on-unverified",
        action="store_true",
        help="Exit non-zero when a check could not run at all (e.g. the "
        "compiled-model structural comparison, which needs an importable "
        "mujoco). Without this a skipped section still reports 'Mismatches: 0' "
        "and exits 0 -- a false green. Required for any automated gate.",
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
        link_map, fuse = _load_link_map(args.link_map)
    else:
        link_map, fuse = _detect_link_mapping(mjcf_path, urdf_path), {}

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

    align = None
    if args.align_frames:
        align = _resolve_alignment(
            mjcf_path, urdf_path, args.align_frames[0], args.align_frames[1]
        )

    mismatches = compare(
        mjcf_path,
        urdf_path,
        link_map=link_map,
        joint_names=joint_names,
        tolerance=args.tolerance,
        mjcf_class=args.mjcf_class,
        robot_label=robot_label,
        align=align,
        fail_on_unverified=args.fail_on_unverified,
        tip_frames=tuple(args.tip_frames) if args.tip_frames else None,
        fuse=fuse,
    )
    sys.exit(1 if mismatches > 0 else 0)


if __name__ == "__main__":
    main()
