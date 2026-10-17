"""The frame a task goal from the Task Target panel is stated in (#779).

A task goal names its frame in ``header.frame_id``, and the task-goal
controllers (WBC, Task, Compliance) take exactly two names: the arm's
``root_link`` (the profile's ``tcp_parent``, the frame the panel's live pose is
read in from TF) and ``scene_world`` — the scene's world, a model frame the CM
adds from its ``urdf.extra_frames.scene_world`` parameters (sim.yaml and the
sim overlays; a real arm has none until it is calibrated).

The world is not in TF, so the panel cannot read a pose in it. It reads the
CM's parameters instead and carries the live base-frame pose into the world
through them — the same placement the controller converts a ``scene_world``
goal back through. Seeding a world goal with base numbers would put it off by
exactly that placement (0.7 m on the shipped pedestal scene), with nothing
refusing it, so:

- ``world (scene)`` is offered only when the CM defines a usable world: present,
  not provisional (the controllers do not use a provisional one), on the arm's
  root link;
- the live pose display and the panel's seed follow the selected frame, and
  changing the frame re-seeds the panel.

Placement convention (``urdf.extra_frames``): ``p_root = R(rpy) · p_world + xyz``
with ``R = Rz(yaw) · Ry(pitch) · Rx(roll)`` — the controllers' RPY too.

The dual-arm controller is not under this contract (its ``world`` is a model
frame of its own model, #779 R6); its goal frames are ``demo_gui.dualarm``'s.

Pure Python — no Tk, no rclpy — so it is unit-testable without a display or a
ROS graph, like ``demo_gui.task_frame``.

The rotation convention is ``rtc_tools.utils.rotations.rpy_matrix`` (one copy
for the tools; integrated_bringup exec-depends on rtc_tools).

Public surface (imported by app.py):
- CM_NODE, SCENE_WORLD, SCENE_WORLD_PREFIX, SCENE_WORLD_PARAMS
- SceneWorld, parse_scene_world, answer_in_request_order
- frame_labels, frame_id_for, BASE_LABEL_PREFIX, WORLD_LABEL
- root_pose_to_world, world_pose_to_root, convert_pose
"""

from __future__ import annotations

import math
import types
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np

from rtc_tools.utils.rotations import rpy_matrix

# The CM node that declares urdf.extra_frames (launch: name="integrated_rt_controller").
CM_NODE = "/integrated_rt_controller"
SCENE_WORLD = "scene_world"
SCENE_WORLD_PREFIX = f"urdf.extra_frames.{SCENE_WORLD}"
# The order parse_scene_world reads its answer in.
SCENE_WORLD_PARAMS = (
    f"{SCENE_WORLD_PREFIX}.parent",
    f"{SCENE_WORLD_PREFIX}.xyz",
    f"{SCENE_WORLD_PREFIX}.rpy",
    f"{SCENE_WORLD_PREFIX}.provisional",
)

# rcl_interfaces/msg/ParameterType values (kept here to stay rclpy-free).
_NOT_SET, _BOOL, _STRING, _INTEGER_ARRAY, _DOUBLE_ARRAY = 0, 1, 4, 7, 8
_NOT_SET_VALUE = types.SimpleNamespace(type=_NOT_SET)

BASE_LABEL_PREFIX = "base"
WORLD_LABEL = "world (scene)"


@dataclass(frozen=True)
class SceneWorld:
    """``scene_world``'s placement on its parent, ``p_parent = R(rpy) p_world + xyz``."""

    parent: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]


def _vec3(value: Any) -> tuple[float, float, float] | None:
    if value.type == _DOUBLE_ARRAY:
        raw = list(value.double_array_value)
    elif value.type == _INTEGER_ARRAY:  # the CM accepts an integer list too
        raw = [float(v) for v in value.integer_array_value]
    else:
        return None
    if len(raw) != 3 or not all(math.isfinite(v) for v in raw):
        return None
    return (float(raw[0]), float(raw[1]), float(raw[2]))


def answer_in_request_order(names: Sequence[str], values: Sequence[Any]) -> list[Any]:
    """``get_parameters(names)``'s values laid out as :data:`SCENE_WORLD_PARAMS`.

    The CM declares only the parameters its files set, and its GetParameters
    service answers a request naming an undeclared one with no values at all.
    So the app lists what is declared under :data:`SCENE_WORLD_PREFIX` first,
    gets those, and hands both here: a name that was not declared is NOT_SET.
    """
    by_name = dict(zip(names, values, strict=False))
    return [by_name.get(n, _NOT_SET_VALUE) for n in SCENE_WORLD_PARAMS]


def parse_scene_world(values: Sequence[Any], root_link: str) -> tuple[SceneWorld | None, str]:
    """The CM's answer for :data:`SCENE_WORLD_PARAMS` -> ``(world, why_not)``.

    ``values`` are ``rcl_interfaces/msg/ParameterValue``-shaped (``type`` plus
    the typed fields), in request order. ``world`` is None when a world goal
    cannot be authored from this panel, and ``why_not`` then says why (for the
    selector's tooltip line); it is empty otherwise.
    """
    if len(values) != len(SCENE_WORLD_PARAMS):
        return None, "the CM's scene_world parameters could not be read"
    parent, xyz, rpy, provisional = values
    if parent.type == _NOT_SET:
        return None, "this bring-up defines no scene_world (a real arm until it is calibrated)"
    # As the CM parses it: a missing flag means provisional.
    if provisional.type != _BOOL or provisional.bool_value:
        return None, "scene_world is provisional here — the controllers refuse world goals"
    if parent.type != _STRING:
        return None, "scene_world.parent is not a string"
    xyz3, rpy3 = _vec3(xyz), _vec3(rpy)
    if xyz3 is None or rpy3 is None:
        return None, "scene_world.xyz / rpy are not three finite numbers"
    if parent.string_value != root_link:
        # The controllers would still convert it (through the model), but the
        # panel's live pose is read in root_link and could not be carried over.
        return None, (
            f"scene_world hangs off '{parent.string_value}', not the arm root '{root_link}'"
        )
    return SceneWorld(parent.string_value, xyz3, rpy3), ""


def frame_labels(root_link: str, world: SceneWorld | None) -> tuple[str, ...]:
    """The selector's entries: the arm root first, then the world when usable."""
    base = f"{BASE_LABEL_PREFIX} ({root_link})"
    return (base, WORLD_LABEL) if world is not None else (base,)


def frame_id_for(label: str, root_link: str) -> str:
    """``header.frame_id`` for a goal stated in the selector entry ``label``."""
    return SCENE_WORLD if label == WORLD_LABEL else root_link


# ── Pose conversion (x, y, z, roll, pitch, yaw), rad ─────────────────────────


def _matrix_rpy(m: np.ndarray) -> list[float]:
    """Inverse of ``rpy_matrix`` (pitch in [-pi/2, pi/2], as pinocchio's matrixToRpy)."""
    pitch = math.atan2(-m[2, 0], math.hypot(m[0, 0], m[1, 0]))
    yaw = math.atan2(m[1, 0], m[0, 0])
    roll = math.atan2(m[2, 1], m[2, 2])
    return [roll, pitch, yaw]


def root_pose_to_world(pose: Sequence[float], world: SceneWorld) -> list[float]:
    """A pose in the arm root -> the same pose in ``scene_world``.

    ``p_world = Rᵀ (p_root - xyz)``, ``R_world = Rᵀ R_root``.
    """
    r = rpy_matrix(world.rpy)
    p = r.T @ (np.asarray(pose[:3], dtype=float) - np.asarray(world.xyz))
    return [*p.tolist(), *_matrix_rpy(r.T @ rpy_matrix(pose[3:6]))]


def world_pose_to_root(pose: Sequence[float], world: SceneWorld) -> list[float]:
    """A pose in ``scene_world`` -> the arm root: what the controller does with it.

    ``p_root = R p_world + xyz``, ``R_root = R R_world``.
    """
    r = rpy_matrix(world.rpy)
    p = r @ np.asarray(pose[:3], dtype=float) + np.asarray(world.xyz)
    return [*p.tolist(), *_matrix_rpy(r @ rpy_matrix(pose[3:6]))]


def convert_pose(
    pose: Sequence[float], source: SceneWorld | None, target: SceneWorld | None
) -> list[float]:
    """``pose`` stated in ``source`` -> stated in ``target`` (None = the arm root)."""
    root = list(pose) if source is None else world_pose_to_root(pose, source)
    return root if target is None else root_pose_to_world(root, target)
