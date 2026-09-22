"""Kinematic catchability map — numeric core plus the batch judge driver (S3.5a).

The judgement itself is NOT reimplemented here: it is the C++ executable
``catch_pose_ik_batch`` (``rtc_controllers``), which calls the very
``CatchPoseIk::Solve`` the runtime planner will call, with the very YAML keys it
will read (plan §11 / S1.9). This module owns everything that is not the
judgement — the grid, the flight, the frames, the ModelConfig translation, the
sharded subprocess driver, the aggregation and the plots — and it reaches the
judge over CSV rather than re-deriving its verdict in python.

The parts that are pure functions of numbers:

1. **Ball flight** — free flight of the ball from a release state, integrated
   with fixed-step RK4 under the SAME force law the simulator runs
   (``rtc_mujoco_sim/src/projectile_ball.cpp`` ``ComputeProjectileBallAeroForce``):
   gravity plus quadratic drag ``-1/2 rho Cd A |v| v``.
   **Assumption: the angular velocity is zero**, so the simulator's Magnus term
   (``C_L(S) w x v``) vanishes identically and is not implemented here. A throw
   launched with spin through ``rtc_msgs/srv/LaunchBall`` will NOT follow the
   trajectory this module predicts.
2. **Throw grid** — the candidate release states, parameterised so that each one
   maps one-to-one onto the ``LaunchBall`` request fields (world-frame position,
   linear velocity, angular velocity).
3. **Frame conversion** — world-frame points and vectors into the arm base
   frame, with the transform supplied by the caller.
4. **Candidate sampling** — the catch instants taken out of one flight, filtered
   by flight time and by a reachability pre-filter the caller supplies.
5. **Aggregation** — per-throw outcome, reason histogram, w₅/w₆ and θ
   distributions, the ``sim.throw_region`` proposal, the seed comparison and the
   ε-perturbation boundary count. Every one of these except the seed comparison
   describes ONE wait-pose seed (the best of the ranking): a robot waits in one
   pose, so a throw "some seed accepts" is not a throw the robot catches. The
   union over seeds is reported only as ``accepted_throws_any_seed``.

**THE JUDGE'S FRAME IS THE PINOCCHIO MODEL WORLD, NOT THE ARM BASE.**
``catch_pose_ik_batch`` documents ``p_c`` / ``v`` as *model world*, i.e. the
universe frame of the model ``PinocchioModelBuilder`` built — which is the URDF
model root, not whichever link the robot config calls the arm base frame. On at
least one shipped arm the two differ by exactly a 180° turn about z: that URDF
carries both a ``<name>`` and a ``<name>_link`` at the SAME origin, the model
root is the latter and the configured arm base frame is the former. So
world→base being the identity does not make world→*model world* the identity.
plan §11 has the measured per-robot table. Feeding
world coordinates straight to the judge is therefore the "ball arrives from
behind the arm" trap, with every number still plausible. So this module never
takes "the" transform as one opaque matrix: the caller gives ``base_T_world``
(measured), :func:`frame_placement_in_model_world` reads
``model_world_T_base`` out of the model, and the two are composed. Nothing
per-robot is embedded either way.

**Why the aerodynamic parameters have no defaults.** The simulator's drag
coefficient lives in a ``constexpr`` preset table in C++
(``rtc_mujoco_sim/src/projectile_ball.cpp``) and is NOT exposed through YAML —
only the ball's radius and mass are (``projectile_ball.radius_m`` /
``.mass_kg``). A hardcoded python mirror of ``Cd`` or of the air density would
silently rot the day the preset changes, and nothing would fail. So
:class:`BallParams` demands all four quantities from the caller *together with a
source label for each*, and :func:`build_provenance` writes those labels into
the report. Radius and mass, which ARE exposed, are read from the YAML by
:func:`ball_shape_from_config` instead of being passed in.

**The caller owns the world-to-base transform.** A URDF that carries both a
``base`` and a ``base_link`` frame may place the two at the same origin rotated
180 degrees about z. Choosing the wrong one flips the downrange axis: the ball
appears to arrive from behind the arm, and every number in the report stays
plausible. Hence :func:`world_to_base` takes the transform as an argument and
this module embeds no per-robot value — pass the transform measured for the
robot at hand (same-q MuJoCo FK against Pinocchio FK). Plan §11 carried the
wrong row for one robot profile until 2026-09-21 (a 180-degree turn counted
twice), which is exactly why this is a parameter and not a constant.
"""

from __future__ import annotations

import argparse
import csv
import datetime as _dt
import hashlib
import io
import itertools
import json
import math
import os
import subprocess
import sys
from collections.abc import Callable, Iterable, Mapping, Sequence
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np
import yaml

# Reused, not forked (design-principles.md P5): the ROS 2 ``ros__parameters``
# merge, the git-head stamp and the xacro expansion already exist in the
# sibling analysis tool.
from rtc_tools.analysis.derive_accel_limits import (
    _git_head,
    load_robot_params,
    resolve_urdf_text,
)

# ── Physical constants that are NOT ball parameters ───────────────────────────

# L0 §6 ``sim.ball.gravity`` default. Named (rather than inlined) for the same
# reason clock_phase.py names its constants: the provenance has to be able to
# say where the number came from.
GRAVITY_W_M_S2: tuple[float, float, float] = (0.0, 0.0, -9.81)
GRAVITY_SOURCE = "docs/dynamic_catching/L0_core.md §6 sim.ball.gravity default"

# Mirrors ``kMinAirspeed`` in rtc_mujoco_sim/src/projectile_ball.cpp:13 — below
# it the simulator returns zero aerodynamic force.
MIN_AIRSPEED_M_S = 1e-6

# L0 §4.1 / IMPLEMENTATION_PLAN.md §"항력 k" representative value [1/m], used by
# the L2/L3/L4 reference tests and by clock_phase.py's a_bound default.
#
# It does NOT agree with the shipped tennis preset: rho 1.204, Cd 0.55,
# r 0.0335 m, m 0.057 kg give k = 0.02048 1/m, i.e. the documented value is
# ~12 % larger. Nor can the two be reconciled by conversion — L0 §7 says so
# explicitly: the documented k is a scalar [1/m] fitted to Magnus-free flight,
# while the simulator runs a dimensionless preset Cd plus a Magnus term. This
# module integrates the *preset* law from caller-supplied parameters and carries
# the documented number only for side-by-side reporting.
DOCS_REPRESENTATIVE_DRAG_K_PER_M = 0.0229
DOCS_REPRESENTATIVE_DRAG_K_SOURCE = (
    "docs/dynamic_catching/L0_core.md §4.1 (representative value, "
    "sim.ball.drag_k itself stays TBD — S3.8 is out of scope)"
)

BALL_SHAPE_KEYS = ("radius_m", "mass_kg")
_AERO_KEYS = ("radius_m", "mass_kg", "drag_coefficient", "air_density_kg_m3")


# ── Ball parameters ───────────────────────────────────────────────────────────


@dataclass(frozen=True)
class BallShape:
    """Radius and mass, i.e. the two ball quantities the YAML does expose."""

    radius_m: float
    mass_kg: float
    sources: Mapping[str, str]


@dataclass(frozen=True)
class BallParams:
    """Everything the free-flight force law needs. Deliberately without defaults.

    ``sources`` must carry one label per quantity (see :data:`_AERO_KEYS`) saying
    where the value came from: a ``file:line`` for the C++ preset constants, a
    ``file:key`` for the YAML ones.
    """

    radius_m: float
    mass_kg: float
    drag_coefficient: float
    air_density_kg_m3: float
    sources: Mapping[str, str]

    def __post_init__(self) -> None:
        for name in ("radius_m", "mass_kg"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"BallParams.{name} must be finite and > 0 (got {value!r})")
        # Cd and rho are only allowed to be zero, not negative: zero is the
        # shipped ``projectile_ball.aerodynamics: false`` case (and the drag-free
        # closed form the tests check against), whereas a negative value would
        # accelerate the ball along its velocity.
        for name in ("drag_coefficient", "air_density_kg_m3"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"BallParams.{name} must be finite and >= 0 (got {value!r})")
        missing = [k for k in _AERO_KEYS if k not in self.sources]
        if missing:
            raise ValueError(
                f"BallParams.sources lacks a provenance label for {missing}. Every "
                "aerodynamic quantity needs one: the drag coefficient and the air "
                "density are constexpr C++ presets with no YAML exposure, so a "
                "report that does not name their file:line cannot be re-derived."
            )

    @property
    def area_m2(self) -> float:
        """Frontal area pi r^2, as the simulator computes it."""
        return math.pi * self.radius_m * self.radius_m

    @property
    def drag_k_per_m(self) -> float:
        """``k = rho Cd A / (2 m)`` [1/m] — DERIVED, FOR REFERENCE ONLY.

        Exposed so a report can be put next to
        :data:`DOCS_REPRESENTATIVE_DRAG_K_PER_M`. It is not an input: nothing in
        this module reads a ``sim.ball.drag_k`` value back, and the documented
        number must not be substituted for the preset ``Cd`` (L0 §7).
        """
        return self.air_density_kg_m3 * self.drag_coefficient * self.area_m2 / (2.0 * self.mass_kg)


def ball_shape_from_config(config_paths: Sequence[Path]) -> BallShape:
    """Read ``projectile_ball.radius_m`` / ``.mass_kg`` from simulator YAML(s).

    ``config_paths`` are ROS 2 ``/**: ros__parameters`` files in override order
    (later wins), e.g. a package default followed by a robot-specific one. The
    returned ``sources`` name the file each value actually came from, so an
    override is visible in the report rather than averaged away.
    """
    paths = [Path(p) for p in config_paths]
    if not paths:
        raise ValueError("ball_shape_from_config needs at least one config path")

    origin: dict[str, Path] = {}
    for path in paths:
        block = load_robot_params([path]).get("projectile_ball") or {}
        for key in BALL_SHAPE_KEYS:
            if key in block:
                origin[key] = path

    merged = load_robot_params(paths).get("projectile_ball") or {}
    missing = [k for k in BALL_SHAPE_KEYS if k not in merged]
    if missing:
        raise SystemExit(
            f"no projectile_ball.{missing} in {[str(p) for p in paths]} — this tool "
            "reads the ball shape from the simulator config rather than hardcoding it"
        )
    return BallShape(
        radius_m=float(merged["radius_m"]),
        mass_kg=float(merged["mass_kg"]),
        sources={k: f"{origin[k]}:projectile_ball.{k}" for k in BALL_SHAPE_KEYS},
    )


def ball_params_from_shape(
    shape: BallShape,
    *,
    drag_coefficient: float,
    drag_coefficient_source: str,
    air_density_kg_m3: float,
    air_density_source: str,
) -> BallParams:
    """Combine the YAML-exposed shape with the caller's preset constants.

    The two keyword pairs are the whole point of this function: the caller has
    to state the value AND where it was read, because neither quantity is
    reachable from YAML (see the module docstring).
    """
    sources = dict(shape.sources)
    sources["drag_coefficient"] = drag_coefficient_source
    sources["air_density_kg_m3"] = air_density_source
    return BallParams(
        radius_m=shape.radius_m,
        mass_kg=shape.mass_kg,
        drag_coefficient=float(drag_coefficient),
        air_density_kg_m3=float(air_density_kg_m3),
        sources=sources,
    )


# ── Ball flight ───────────────────────────────────────────────────────────────


@dataclass(frozen=True, eq=False)
class Trajectory:
    """Sampled free flight. ``position_m`` / ``velocity_m_s`` are ``(n, 3)``."""

    time_s: np.ndarray
    position_m: np.ndarray
    velocity_m_s: np.ndarray
    step_s: float

    @property
    def speed_m_s(self) -> np.ndarray:
        return np.linalg.norm(self.velocity_m_s, axis=1)


def drag_acceleration(velocity_w: np.ndarray, params: BallParams) -> np.ndarray:
    """``-1/2 rho Cd A |v| v / m`` — the simulator's drag term, per unit mass.

    Quadratic in the speed, so ``|v|`` is recomputed here from whatever velocity
    it is handed; the RK4 substages must not share a precomputed speed.
    """
    v = np.asarray(velocity_w, dtype=float)
    speed = float(np.linalg.norm(v))
    if not speed > MIN_AIRSPEED_M_S:
        return np.zeros(3)
    # Written out rather than via ``params.drag_k_per_m`` so the grouping mirrors
    # the C++ force law; the two are numerically the same quantity.
    scale = (
        0.5 * params.air_density_kg_m3 * params.drag_coefficient * params.area_m2 / params.mass_kg
    )
    return -(scale * speed) * v


def ball_acceleration(
    velocity_w: np.ndarray,
    params: BallParams,
    gravity_w: Sequence[float] = GRAVITY_W_M_S2,
) -> np.ndarray:
    """Gravity plus quadratic drag. Spin is assumed zero, so there is no Magnus term."""
    return np.asarray(gravity_w, dtype=float) + drag_acceleration(velocity_w, params)


def integrate_flight(
    position_w: Sequence[float],
    velocity_w: Sequence[float],
    params: BallParams,
    *,
    horizon_s: float,
    step_s: float,
    gravity_w: Sequence[float] = GRAVITY_W_M_S2,
) -> Trajectory:
    """Integrate free flight to ``horizon_s`` with fixed-step RK4.

    The step is a parameter and the horizon is covered by ``ceil`` steps, so the
    last sample is at or just past ``horizon_s``. Nothing is clipped: the ball
    keeps falling past the floor, and trimming the trajectory to the useful
    window is the caller's job (candidate catch points come from it).
    """
    if not (math.isfinite(step_s) and step_s > 0.0):
        raise ValueError(f"step_s must be finite and > 0 (got {step_s!r})")
    if not (math.isfinite(horizon_s) and horizon_s > 0.0):
        raise ValueError(f"horizon_s must be finite and > 0 (got {horizon_s!r})")

    p0 = np.asarray(position_w, dtype=float).reshape(3)
    v0 = np.asarray(velocity_w, dtype=float).reshape(3)
    gravity = np.asarray(gravity_w, dtype=float).reshape(3)
    steps = int(math.ceil(horizon_s / step_s - 1e-12))

    def deriv(state: np.ndarray) -> np.ndarray:
        out = np.empty(6)
        out[:3] = state[3:]
        out[3:] = gravity + drag_acceleration(state[3:], params)
        return out

    states = np.empty((steps + 1, 6))
    states[0, :3] = p0
    states[0, 3:] = v0
    h = float(step_s)
    for i in range(steps):
        y = states[i]
        k1 = deriv(y)
        k2 = deriv(y + 0.5 * h * k1)
        k3 = deriv(y + 0.5 * h * k2)
        k4 = deriv(y + h * k3)
        states[i + 1] = y + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    return Trajectory(
        time_s=h * np.arange(steps + 1),
        position_m=states[:, :3].copy(),
        velocity_m_s=states[:, 3:].copy(),
        step_s=h,
    )


# ── Throw grid ────────────────────────────────────────────────────────────────

# Documented defaults, sized so a caller can shrink any axis for a smoke test.
# The horizontal distance is an axis like the others, not a constant, and
# defaults to the single value the S0.7 sweep used.
DEFAULT_DISTANCES_M: tuple[float, ...] = (4.0,)
DEFAULT_AZIMUTHS_DEG: tuple[float, ...] = (-30.0, 0.0, 30.0)
DEFAULT_RELEASE_HEIGHTS_M: tuple[float, ...] = (1.2, 1.8, 2.4)
DEFAULT_AIM_DEVIATIONS_DEG: tuple[float, ...] = (-10.0, 0.0, 10.0)
DEFAULT_SPEEDS_M_S: tuple[float, ...] = (5.0, 7.0, 9.0)
DEFAULT_ELEVATIONS_DEG: tuple[float, ...] = (10.0, 20.0, 30.0)


@dataclass(frozen=True, eq=False)
class Throw:
    """One candidate throw: its grid coordinates plus the world release state.

    ``position_m`` and ``velocity_m_s`` are what
    :func:`throw_to_launch_request` hands to ``rtc_msgs/srv/LaunchBall``.
    """

    distance_m: float
    azimuth_deg: float
    release_height_m: float
    aim_deviation_deg: float
    speed_m_s: float
    elevation_deg: float
    position_m: np.ndarray
    velocity_m_s: np.ndarray

    @property
    def inward_horizontal(self) -> np.ndarray:
        """Unit horizontal vector from the release point towards the base axis."""
        az = math.radians(self.azimuth_deg)
        return np.array([-math.cos(az), -math.sin(az), 0.0])


def generate_throw_grid(
    *,
    base_xy_m: Sequence[float] = (0.0, 0.0),
    distances_m: Sequence[float] = DEFAULT_DISTANCES_M,
    azimuths_deg: Sequence[float] = DEFAULT_AZIMUTHS_DEG,
    release_heights_m: Sequence[float] = DEFAULT_RELEASE_HEIGHTS_M,
    aim_deviations_deg: Sequence[float] = DEFAULT_AIM_DEVIATIONS_DEG,
    speeds_m_s: Sequence[float] = DEFAULT_SPEEDS_M_S,
    elevations_deg: Sequence[float] = DEFAULT_ELEVATIONS_DEG,
) -> list[Throw]:
    """Full Cartesian product of the six axes, as world-frame release states.

    Geometry: the release point sits at horizontal ``distance`` from the base
    axis at ``azimuth`` about world +z (0 deg = world +x), at world height
    ``release_height``. The nominal aim is the horizontal direction back towards
    the base axis; ``aim_deviation`` rotates it about world +z and ``elevation``
    tilts the velocity up from horizontal, exactly as the simulator's own
    sampler splits speed into ``cos(angle)`` horizontal and ``sin(angle)``
    vertical components.

    ``base_xy_m`` places the base axis in the world. It is a parameter, not a
    constant, and the identity default is not a per-robot value — see the module
    docstring on frames.
    """
    axes = {
        "distances_m": distances_m,
        "azimuths_deg": azimuths_deg,
        "release_heights_m": release_heights_m,
        "aim_deviations_deg": aim_deviations_deg,
        "speeds_m_s": speeds_m_s,
        "elevations_deg": elevations_deg,
    }
    for name, values in axes.items():
        if len(values) == 0:
            raise ValueError(f"{name} must not be empty")
    if any(not (math.isfinite(d) and d > 0.0) for d in distances_m):
        raise ValueError("distances_m must all be finite and > 0")
    if any(not (math.isfinite(s) and s > 0.0) for s in speeds_m_s):
        raise ValueError("speeds_m_s must all be finite and > 0")
    if any(abs(a) >= 90.0 for a in aim_deviations_deg):
        raise ValueError(
            "aim_deviations_deg must be within (-90, 90) — at or beyond 90 the throw no "
            "longer travels towards the base at all"
        )
    if any(abs(e) >= 90.0 for e in elevations_deg):
        raise ValueError("elevations_deg must be within (-90, 90) — 90 is a vertical throw")

    base = np.asarray(base_xy_m, dtype=float).reshape(2)
    up = np.array([0.0, 0.0, 1.0])
    throws: list[Throw] = []
    for distance, azimuth, height, aim, speed, elevation in itertools.product(
        distances_m,
        azimuths_deg,
        release_heights_m,
        aim_deviations_deg,
        speeds_m_s,
        elevations_deg,
    ):
        az = math.radians(azimuth)
        outward = np.array([math.cos(az), math.sin(az), 0.0])
        position = np.array(
            [base[0] + distance * outward[0], base[1] + distance * outward[1], float(height)]
        )
        aim_dir = rotation_z(math.radians(aim)) @ (-outward)
        el = math.radians(elevation)
        velocity = speed * math.cos(el) * aim_dir + speed * math.sin(el) * up
        throws.append(
            Throw(
                distance_m=float(distance),
                azimuth_deg=float(azimuth),
                release_height_m=float(height),
                aim_deviation_deg=float(aim),
                speed_m_s=float(speed),
                elevation_deg=float(elevation),
                position_m=position,
                velocity_m_s=velocity,
            )
        )
    return throws


def throw_to_launch_request(
    throw: Throw, *, angular_velocity_rad_s: Sequence[float] = (0.0, 0.0, 0.0)
) -> dict:
    """The throw as ``rtc_msgs/srv/LaunchBall`` request fields (all world frame).

    The angular velocity defaults to zero because that is the assumption the
    flight model in this module is built on; passing a non-zero spin here makes
    the simulator's Magnus term active and the predicted trajectory invalid.
    """
    w = np.asarray(angular_velocity_rad_s, dtype=float).reshape(3)
    p = throw.position_m
    v = throw.velocity_m_s
    return {
        "position": {"x": float(p[0]), "y": float(p[1]), "z": float(p[2])},
        "velocity": {"x": float(v[0]), "y": float(v[1]), "z": float(v[2])},
        "angular_velocity": {"x": float(w[0]), "y": float(w[1]), "z": float(w[2])},
    }


# ── Frame conversion ──────────────────────────────────────────────────────────


def rotation_x(angle_rad: float) -> np.ndarray:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rotation_y(angle_rad: float) -> np.ndarray:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rotation_z(angle_rad: float) -> np.ndarray:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def make_transform(rotation: np.ndarray, translation: Sequence[float]) -> np.ndarray:
    """Assemble a 4x4 homogeneous transform from a 3x3 rotation and a translation."""
    t = np.eye(4)
    t[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
    t[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return t


def as_transform(transform) -> np.ndarray:
    """Normalise a 4x4, or a ``(rotation, translation)`` pair, into a 4x4.

    The rotation is checked for orthonormality and a positive determinant: a
    transposed or scaled matrix still produces plausible-looking output, so it
    is worth refusing here rather than reading it out of a map later.
    """
    array = None
    if isinstance(transform, tuple | list) and len(transform) == 2:
        array = make_transform(transform[0], transform[1])
    else:
        candidate = np.asarray(transform, dtype=float)
        if candidate.shape == (4, 4):
            array = candidate.copy()
    if array is None:
        raise ValueError("transform must be a 4x4 array or a (rotation 3x3, translation 3) pair")

    r = array[:3, :3]
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-9):
        raise ValueError(f"transform rotation block is not orthonormal:\n{r}")
    if float(np.linalg.det(r)) <= 0.0:
        raise ValueError(f"transform rotation block has a non-positive determinant:\n{r}")
    if not np.allclose(array[3, :], (0.0, 0.0, 0.0, 1.0), atol=1e-12):
        raise ValueError(f"transform last row must be [0 0 0 1], got {array[3, :]}")
    return array


def invert_transform(transform) -> np.ndarray:
    """Inverse of a rigid transform, without a general matrix inverse."""
    t = as_transform(transform)
    r = t[:3, :3]
    out = np.eye(4)
    out[:3, :3] = r.T
    out[:3, 3] = -r.T @ t[:3, 3]
    return out


def transform_point(point: np.ndarray, transform) -> np.ndarray:
    """Apply a rigid transform to a point or to an ``(n, 3)`` stack of points."""
    t = as_transform(transform)
    p = np.asarray(point, dtype=float)
    flat = p.reshape(-1, 3)
    out = flat @ t[:3, :3].T + t[:3, 3]
    return out.reshape(p.shape)


def transform_direction(vector: np.ndarray, transform) -> np.ndarray:
    """Apply only the rotation — for velocities, axes and other free vectors."""
    t = as_transform(transform)
    v = np.asarray(vector, dtype=float)
    flat = v.reshape(-1, 3)
    out = flat @ t[:3, :3].T
    return out.reshape(v.shape)


def world_to_base(
    base_t_world, point_w: np.ndarray, vector_w: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Convert a world-frame point and a world-frame vector into the base frame.

    ``base_t_world`` is the transform that takes world coordinates to base
    coordinates: ``p_base = R p_world + t``. It may be a 4x4 or a
    ``(rotation, translation)`` pair.

    **The caller owns this transform.** It is never inferred and never defaulted
    per robot. Where a URDF offers two candidate base frames a 180-degree error
    about z is silent — downrange flips sign and the map still looks sensible —
    so pass the transform that was measured for the robot in hand.
    """
    return transform_point(point_w, base_t_world), transform_direction(vector_w, base_t_world)


# ── Provenance ────────────────────────────────────────────────────────────────


def build_provenance(
    params: BallParams,
    *,
    configs: Sequence[Path] = (),
    grid: Sequence[Throw] | None = None,
    integration: Mapping[str, float] | None = None,
    extra: Mapping[str, object] | None = None,
) -> dict:
    """Record what the numbers were, where each came from, and what was derived."""
    derived_k = params.drag_k_per_m
    report: dict = {
        "tool": "rtc_tools.analysis.catchability_map",
        "date": _dt.date.today().isoformat(),
        "git_head": _git_head(Path(__file__).parent),
        "robot_config": [str(p) for p in configs],
        "robot_config_sha256": {
            str(p): hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in configs
        },
        "ball": {
            "radius_m": params.radius_m,
            "mass_kg": params.mass_kg,
            "drag_coefficient": params.drag_coefficient,
            "air_density_kg_m3": params.air_density_kg_m3,
            "area_m2": params.area_m2,
            "sources": dict(params.sources),
            "spin": "zero — the simulator's Magnus term is not modelled here",
        },
        "derived_drag_k_per_m": derived_k,
        "derived_drag_k_note": (
            "DERIVED FOR REFERENCE ONLY: k = rho Cd A / (2 m). This model "
            "integrates the simulator's preset force law from the four "
            "quantities above; k is reported only to sit next to the documented "
            "scalar, which is a different quantity (L0 §7: the preset Cd cannot "
            "be converted into it and vice versa)."
        ),
        "docs_representative_drag_k_per_m": DOCS_REPRESENTATIVE_DRAG_K_PER_M,
        "docs_representative_drag_k_source": DOCS_REPRESENTATIVE_DRAG_K_SOURCE,
        "docs_over_derived_drag_k_ratio": (
            DOCS_REPRESENTATIVE_DRAG_K_PER_M / derived_k if derived_k > 0.0 else None
        ),
        "gravity_w_m_s2": list(GRAVITY_W_M_S2),
        "gravity_source": GRAVITY_SOURCE,
    }
    if grid is not None:
        report["grid"] = {
            "throws": len(grid),
            "distances_m": sorted({t.distance_m for t in grid}),
            "azimuths_deg": sorted({t.azimuth_deg for t in grid}),
            "release_heights_m": sorted({t.release_height_m for t in grid}),
            "aim_deviations_deg": sorted({t.aim_deviation_deg for t in grid}),
            "speeds_m_s": sorted({t.speed_m_s for t in grid}),
            "elevations_deg": sorted({t.elevation_deg for t in grid}),
        }
    if integration is not None:
        report["integration"] = {"integrator": "fixed-step RK4", **dict(integration)}
    if extra is not None:
        report.update(dict(extra))
    return report


# ── ModelConfig translation ───────────────────────────────────────────────────

DEFAULT_CATCH_FRAME = "catch_frame"

# The sub-model this tool adds and judges in. Named here because two places have
# to agree on it (the emitted config and the judge's ``--sub-model``), not
# because the name means anything.
CATCH_SUB_MODEL_NAME = "arm_catch"

GENERATED_HEADER = (
    "# Generated by rtc_tools.analysis.catchability_map — rerun the tool, do not\n"
    "# hand-edit the values (dynamic_catching S3.5a, plan §11).\n"
)


@dataclass(frozen=True)
class ModelConfigArtifacts:
    """What :func:`write_model_config` put on disk, and where each part came from."""

    model_config_path: Path
    urdf_path: Path
    arm_sub_model: str
    arm_root_link: str
    arm_tip_link: str
    catch_sub_model: str
    catch_frame: str
    catch_frame_parent: str
    urdf_text: str = field(repr=False, default="")
    provenance: dict = field(repr=False, default_factory=dict)


def write_model_config(
    config_paths: Sequence[Path],
    out_dir: Path,
    *,
    arm_sub_model: str | None = None,
    catch_frame: str = DEFAULT_CATCH_FRAME,
    catch_sub_model: str = CATCH_SUB_MODEL_NAME,
    urdf_override: Path | None = None,
) -> ModelConfigArtifacts:
    """Translate shipped robot config YAML(s) into a ``LoadModelConfig`` YAML.

    ``PinocchioModelBuilder::LoadModelConfig`` does NOT read the shipped
    ``ros__parameters`` schema. Three differences, all silent if missed:

    * the tree is flat at the document root, not under ``/**: ros__parameters``,
    * the URDF is ``urdf_path`` (one path) rather than ``urdf.package`` +
      ``urdf.path``, so the xacro has to be expanded first — this function
      writes the expansion next to the config and points at it,
    * ``sub_models`` is a **sequence** of ``{name, root_link, tip_link}``, while
      the shipped config uses a map ``<name>: {root_link, tip_link}``. Only
      ``extra_frames`` has the same (map) shape in both.

    **Why an extra sub-model is emitted, and why locking the hand is correct.**
    The shipped ``urdf.sub_models.<arm>`` entry stops at the flange (``tool0`` /
    ``ee_link``), which is UPSTREAM of the palm. The catch frame hangs off the
    palm link, so it is simply not in that sub-model and the judge's
    ``--catch-frame`` cannot resolve — the map would be empty for a reason that
    looks like the frame being misdeclared. So a second sub-model
    (``catch_sub_model``, default ``arm_catch``) is declared from the arm's own
    ``root_link`` to the catch frame's **parent link**, read from
    ``urdf.extra_frames.<catch_frame>.parent``. ``buildReducedModel`` then locks
    everything past that link, i.e. the whole hand. That is not an
    approximation: the palm is upstream of the hand's closed-chain loops, so no
    hand joint moves the palm, and the catch frame's FK and its Jacobian **in
    the arm joints** are exact. The locked hand still contributes its mass, but
    this judge is kinematic and never forms M(q).

    ``tree_models`` are deliberately NOT translated — the judge only ever asks
    for a reduced sub-model, and every derived model is cut from the same full
    model, so a tree model would only cost build time. ``closure_yaml_path`` IS
    forwarded when the shipped config declares one, so the full model this tool
    builds is the one production builds.

    Returns the paths plus a provenance dict (config sha256s, URDF label and
    sha256, the sub-model delta, and the locked-hand note above).
    """
    paths = [Path(p) for p in config_paths]
    if not paths:
        raise ValueError("write_model_config needs at least one robot config path")

    params = load_robot_params(paths)
    urdf = params.get("urdf") or {}
    if not urdf:
        raise SystemExit(f"no `urdf` section in {[str(p) for p in paths]}")

    shipped_subs = urdf.get("sub_models") or {}
    if not isinstance(shipped_subs, Mapping) or not shipped_subs:
        raise SystemExit("robot config lacks a `urdf.sub_models` map")
    if arm_sub_model is None:
        arm_sub_model = next(iter(shipped_subs))
    if arm_sub_model not in shipped_subs:
        raise SystemExit(
            f"sub-model '{arm_sub_model}' is not in urdf.sub_models (have {sorted(shipped_subs)})"
        )
    arm = shipped_subs[arm_sub_model]
    arm_root = str(arm["root_link"])
    arm_tip = str(arm["tip_link"])

    frames = urdf.get("extra_frames") or {}
    if catch_frame not in frames:
        raise SystemExit(
            f"robot config lacks urdf.extra_frames.{catch_frame} — the judge needs the "
            "catch frame declared there, not guessed here"
        )
    catch_parent = str(frames[catch_frame]["parent"])

    urdf_text, urdf_label = resolve_urdf_text(params, urdf_override)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    urdf_path = out_dir / "model.urdf"
    urdf_path.write_text(urdf_text)

    doc: dict = {
        "urdf_path": str(urdf_path),
        "root_joint_type": str(urdf.get("root_joint_type", "fixed")),
        "sub_models": [
            {"name": arm_sub_model, "root_link": arm_root, "tip_link": arm_tip},
            {"name": catch_sub_model, "root_link": arm_root, "tip_link": catch_parent},
        ],
        "extra_frames": {
            str(name): {
                "parent": str(entry["parent"]),
                "xyz": [float(v) for v in entry["xyz"]],
                "rpy": [float(v) for v in entry["rpy"]],
                "provisional": bool(entry.get("provisional", True)),
            }
            for name, entry in frames.items()
        },
    }
    closure_path = _resolve_closure_path(urdf)
    if closure_path is not None:
        doc["closure_yaml_path"] = str(closure_path)
    for key in ("passive_joints", "lock_reference_config", "yaml_passive_override"):
        if key in urdf:
            doc[key] = urdf[key]

    model_config_path = out_dir / "model_config.yaml"
    model_config_path.write_text(
        GENERATED_HEADER + yaml.safe_dump(doc, sort_keys=False, default_flow_style=None, width=100)
    )

    provenance = {
        "robot_config": [str(p) for p in paths],
        "robot_config_sha256": {
            str(p): hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in paths
        },
        "urdf": urdf_label,
        "urdf_sha256": hashlib.sha256(urdf_text.encode()).hexdigest(),
        "urdf_expanded_to": str(urdf_path),
        "model_config": str(model_config_path),
        "closure_yaml_path": None if closure_path is None else str(closure_path),
        "shipped_arm_sub_model": {
            "name": arm_sub_model,
            "root_link": arm_root,
            "tip_link": arm_tip,
        },
        "catch_sub_model": {
            "name": catch_sub_model,
            "root_link": arm_root,
            "tip_link": catch_parent,
        },
        "catch_frame": catch_frame,
        "catch_frame_parent": catch_parent,
        "sub_model_note": (
            f"the shipped '{arm_sub_model}' sub-model stops at '{arm_tip}', upstream of the "
            f"palm, so '{catch_frame}' is not in it; '{catch_sub_model}' runs to the catch "
            f"frame's parent '{catch_parent}' instead. buildReducedModel locks everything "
            "past that link (the whole hand), which is exact for this judge: the palm is "
            "upstream of the hand's closed-chain loops, so the catch-frame FK and its "
            "Jacobian in the arm joints do not depend on any hand joint."
        ),
        "tree_models": "not translated — the judge only asks for a reduced sub-model",
    }
    return ModelConfigArtifacts(
        model_config_path=model_config_path,
        urdf_path=urdf_path,
        arm_sub_model=arm_sub_model,
        arm_root_link=arm_root,
        arm_tip_link=arm_tip,
        catch_sub_model=catch_sub_model,
        catch_frame=catch_frame,
        catch_frame_parent=catch_parent,
        urdf_text=urdf_text,
        provenance=provenance,
    )


def _resolve_closure_path(urdf: Mapping) -> Path | None:
    """``urdf.closure_path`` as an absolute path, or None for a plain URDF."""
    rel = urdf.get("closure_path")
    if not rel or urdf.get("extended") is False:
        return None
    candidate = Path(str(rel))
    if candidate.is_absolute():
        return candidate
    package = urdf.get("package")
    if not package:
        raise SystemExit("urdf.closure_path is relative but urdf.package is absent")
    from ament_index_python.packages import get_package_share_directory  # noqa: PLC0415

    return Path(get_package_share_directory(str(package))) / candidate


def frame_placement_in_model_world(
    urdf_text: str, frame_name: str, *, seed: int = 0
) -> np.ndarray:
    """``model_world_T_frame`` — where a rigid frame sits in the Pinocchio universe.

    This is the missing half of the judge's frame contract (see the module
    docstring): the judge wants MODEL WORLD coordinates, and the transform the
    caller can actually measure is world→arm-base. Composing the two needs
    ``model_world_T_base``, which is a property of the URDF and is read here
    rather than assumed.

    The frame must be RIGID with respect to the model root, which is what makes
    the answer a constant. That is checked, not trusted: the placement is
    evaluated at the neutral configuration and again at a random configuration
    inside the model's finite position limits, and a frame that moves is refused
    (it would make ``model_world_T_base`` a function of q, and picking the
    neutral value would be quietly wrong at every other posture).
    """
    import pinocchio as pin  # noqa: PLC0415

    model = pin.buildModelFromXML(urdf_text)
    if not model.existFrame(frame_name):
        raise SystemExit(f"the URDF has no frame '{frame_name}'")
    data = model.createData()
    frame_id = model.getFrameId(frame_name)

    def placement(q: np.ndarray) -> np.ndarray:
        pin.framesForwardKinematics(model, data, q)
        m = data.oMf[frame_id]
        return make_transform(np.asarray(m.rotation), np.asarray(m.translation))

    neutral = pin.neutral(model)
    at_neutral = placement(neutral)
    lower = np.asarray(model.lowerPositionLimit, dtype=float)
    upper = np.asarray(model.upperPositionLimit, dtype=float)
    finite = np.isfinite(lower) & np.isfinite(upper) & (upper > lower)
    q = np.asarray(neutral, dtype=float).copy()
    if np.any(finite):
        rng = np.random.default_rng(seed)
        q[finite] = rng.uniform(lower[finite], upper[finite])
    at_random = placement(q)
    if not np.allclose(at_neutral, at_random, atol=1e-12):
        raise SystemExit(
            f"frame '{frame_name}' is not rigid with respect to the model root — its "
            "placement changes with q, so it cannot serve as the arm base frame"
        )
    return at_neutral


# ── Catch-candidate sampling ──────────────────────────────────────────────────

# D-18: candidates below this flight time are dropped. A parameter with a
# documented default, not a constant — the map is allowed to explore a shorter
# window, it just has to say so.
DEFAULT_MIN_FLIGHT_TIME_S = 1.0
MIN_FLIGHT_TIME_SOURCE = "docs/dynamic_catching/IMPLEMENTATION_PLAN.md §11 (D-18), T_f >= 1.0 s"


@dataclass(frozen=True, eq=False)
class CatchCandidate:
    """One candidate catch instant of one throw, in WORLD coordinates."""

    throw_index: int
    time_s: float
    position_m: np.ndarray
    velocity_m_s: np.ndarray
    speed_m_s: float


def max_distance_filter(
    center_m: Sequence[float], max_distance_m: float
) -> Callable[[np.ndarray, np.ndarray], bool]:
    """A reachability pre-filter: keep positions within ``max_distance_m`` of a point.

    Both the point and the radius are the caller's — this function embeds no
    distance and knows no robot. It exists so the judge is not spent on points
    that are metres outside any arm's reach, and it is deliberately a
    PRE-filter: it never accepts anything, it only declines to ask.
    """
    center = np.asarray(center_m, dtype=float).reshape(3)
    radius = float(max_distance_m)
    if not (math.isfinite(radius) and radius > 0.0):
        raise ValueError(f"max_distance_m must be finite and > 0 (got {max_distance_m!r})")

    def keep(position_m: np.ndarray, velocity_m_s: np.ndarray) -> bool:  # noqa: ARG001
        return bool(
            np.linalg.norm(np.asarray(position_m, dtype=float).reshape(3) - center) <= radius
        )

    return keep


def sample_catch_candidates(
    trajectory: Trajectory,
    *,
    throw_index: int = 0,
    window_s: tuple[float, float],
    stride_s: float,
    min_flight_time_s: float = DEFAULT_MIN_FLIGHT_TIME_S,
    reach_filter: Callable[[np.ndarray, np.ndarray], bool] | None = None,
) -> list[CatchCandidate]:
    """Candidate catch instants of one flight: time, position, velocity, speed.

    The samples are INTEGRATOR SAMPLES, never interpolated: ``stride_s`` is
    rounded to the nearest whole number of trajectory steps (at least one), and
    the stride grid is anchored at the start of the effective window. So the
    states the judge sees are bit-for-bit the states the flight model produced,
    and a disagreement between the map and a re-run of the flight cannot be an
    interpolation artefact.

    The effective window starts at ``max(window_s[0], min_flight_time_s)`` — the
    flight-time floor is a filter on the candidate, so raising it removes
    candidates rather than shifting the grid. ``reach_filter`` is called with
    ``(position_m, velocity_m_s)`` in the trajectory's own frame and returning
    False drops that instant; see :func:`max_distance_filter`.
    """
    lo, hi = (float(window_s[0]), float(window_s[1]))
    if not (math.isfinite(lo) and math.isfinite(hi)):
        raise ValueError(f"window_s must be finite (got {window_s!r})")
    if hi < lo:
        raise ValueError(f"window_s must be ordered (got {window_s!r})")
    if not (math.isfinite(stride_s) and stride_s > 0.0):
        raise ValueError(f"stride_s must be finite and > 0 (got {stride_s!r})")
    if not math.isfinite(min_flight_time_s):
        raise ValueError(f"min_flight_time_s must be finite (got {min_flight_time_s!r})")

    step_count = max(1, int(round(stride_s / trajectory.step_s)))
    times = trajectory.time_s
    start = max(lo, float(min_flight_time_s))
    tol = 1e-9 * max(1.0, abs(hi))
    first = int(np.searchsorted(times, start - tol, side="left"))
    out: list[CatchCandidate] = []
    for i in range(first, times.size, step_count):
        t = float(times[i])
        if t > hi + tol:
            break
        position = trajectory.position_m[i]
        velocity = trajectory.velocity_m_s[i]
        if reach_filter is not None and not reach_filter(position, velocity):
            continue
        out.append(
            CatchCandidate(
                throw_index=int(throw_index),
                time_s=t,
                position_m=position.copy(),
                velocity_m_s=velocity.copy(),
                speed_m_s=float(np.linalg.norm(velocity)),
            )
        )
    return out


# ── The judge: candidate / result CSV and the sharded subprocess driver ───────

JUDGE_PACKAGE = "rtc_controllers"
JUDGE_EXECUTABLE = "catch_pose_ik_batch"

# This machine's limit (task constraint, S3.5a): six concurrent judges, and the
# children are pinned to one OpenMP thread each so six processes mean six cores.
MAX_WORKERS = 6
DEFAULT_WORKERS = 6

CANDIDATE_CSV_HEADER = ("id", "seed_id", "p_c_x", "p_c_y", "p_c_z", "v_x", "v_y", "v_z")

_RESULT_SCALARS: tuple[tuple[str, type], ...] = (
    ("id", int),
    ("seed_id", int),
    ("accepted", int),
    ("reason", int),
    ("reason_name", str),
    ("iterations", int),
    ("pos_error", float),
    ("theta", float),
    ("w5", float),
    ("w6", float),
    ("w5_valid", int),
    ("w6_valid", int),
    ("manip_converged", int),
    ("manip_grad_norm", float),
    ("manip_grad_failures", int),
    ("sigma_min", float),
    ("lambda_sq", float),
    ("qp_status", int),
    ("qp_iterations", int),
    ("qp_failures", int),
    ("nv", int),
)


@dataclass(frozen=True, eq=False)
class JudgeCandidate:
    """One row of the judge's candidate CSV, plus what it came from.

    ``p_c_model_m`` / ``v_model_m_s`` are in MODEL WORLD coordinates, which is
    what the judge documents — see the module docstring on why that is not the
    arm base frame. ``throw_index`` / ``time_s`` / ``speed_m_s`` are carried
    along so the result can be joined back onto the throw without a second
    lookup table.
    """

    id: int
    seed_id: int
    p_c_model_m: np.ndarray
    v_model_m_s: np.ndarray
    throw_index: int = -1
    time_s: float = float("nan")
    speed_m_s: float = float("nan")
    p_c_world_m: np.ndarray | None = None
    v_world_m_s: np.ndarray | None = None


@dataclass(frozen=True, eq=False)
class JudgeResult:
    """One row of the judge's result CSV.

    ``q`` is None for every reason that produced no pose. That is a THIRD state
    next to "a pose" and "a rejection", and it is the one a naive
    ``float(cell)`` — or a splitter that drops trailing empty fields — turns
    into a posture of zeros.
    """

    id: int
    seed_id: int
    accepted: bool
    reason: int
    reason_name: str
    iterations: int
    pos_error: float
    theta: float
    w5: float
    w6: float
    w5_valid: bool
    w6_valid: bool
    manip_converged: bool
    manip_grad_norm: float
    manip_grad_failures: int
    sigma_min: float
    lambda_sq: float
    qp_status: int
    qp_iterations: int
    qp_failures: int
    nv: int
    q: np.ndarray | None


@dataclass(frozen=True)
class JudgeInvocation:
    """Everything about the judge that does not change from shard to shard."""

    judge: Path
    model_config: Path
    seeds: Path
    sub_model: str = CATCH_SUB_MODEL_NAME
    catch_frame: str = DEFAULT_CATCH_FRAME
    params: Path | None = None


def find_judge(override: Path | None = None, executable: str | None = None) -> Path:
    """Locate ``catch_pose_ik_batch`` (or a sibling ``executable``), or fail saying what to build.

    ``override`` wins if given. Otherwise the executable is looked up under the
    ament prefix of ``rtc_controllers``, which is where colcon installs it.
    """
    name = executable or JUDGE_EXECUTABLE
    if override is not None:
        path = Path(override)
        if not path.is_file():
            raise SystemExit(f"--judge '{path}' does not exist")
        return path
    try:
        from ament_index_python.packages import get_package_prefix  # noqa: PLC0415

        prefix = Path(get_package_prefix(JUDGE_PACKAGE))
    except Exception as exc:  # noqa: BLE001 — any lookup failure means the same thing
        raise SystemExit(
            f"cannot find the ament prefix of {JUDGE_PACKAGE} ({exc}) — source the workspace "
            f"env, or pass --judge with the path to {name}"
        ) from exc
    path = prefix / "lib" / JUDGE_PACKAGE / name
    if not path.is_file():
        raise SystemExit(
            f"{path} is missing — build it with "
            f"`colcon build --packages-select {JUDGE_PACKAGE}`, or pass --judge"
        )
    return path


def candidate_csv_bytes(candidates: Sequence[JudgeCandidate]) -> bytes:
    """The judge's candidate CSV, as the exact bytes that go on disk.

    Split out of :func:`write_candidate_csv` because those bytes are also an
    input of :func:`shard_fingerprint`: the resume check has to hash what the
    judge would READ, and hashing a re-rendering of it would be a second
    encoder that can drift from the first.

    The header is written in :data:`CANDIDATE_CSV_HEADER` order, which the judge
    reads the column order OUT of rather than assuming — so this order is a
    convenience, not a contract.
    """
    ids = [c.id for c in candidates]
    if len(set(ids)) != len(ids):
        raise ValueError("candidate ids must be unique — the result join is on id")
    buffer = io.StringIO(newline="")
    writer = csv.writer(buffer)
    writer.writerow(CANDIDATE_CSV_HEADER)
    for c in candidates:
        p = np.asarray(c.p_c_model_m, dtype=float).reshape(3)
        v = np.asarray(c.v_model_m_s, dtype=float).reshape(3)
        if not (np.all(np.isfinite(p)) and np.all(np.isfinite(v))):
            raise ValueError(f"candidate id {c.id} has a non-finite p_c or v")
        writer.writerow([c.id, c.seed_id, *(repr(float(x)) for x in (*p, *v))])
    return buffer.getvalue().encode()


def write_candidate_csv(path: Path, candidates: Sequence[JudgeCandidate]) -> int:
    """Write the judge's candidate CSV and return the row count."""
    Path(path).write_bytes(candidate_csv_bytes(candidates))
    return len(candidates)


def write_seed_csv(path: Path, seeds: Sequence[Sequence[float]]) -> None:
    """Write ``seed_id,q0,...`` with ids ``0..len(seeds)-1``."""
    if not seeds:
        raise ValueError("at least one seed is required — the judge refuses an empty seed file")
    widths = {len(list(s)) for s in seeds}
    if len(widths) != 1:
        raise ValueError(f"every seed must have the same width (got {sorted(widths)})")
    with Path(path).open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["seed_id", *(f"q{i}" for i in range(next(iter(widths))))])
        for seed_id, seed in enumerate(seeds):
            writer.writerow([seed_id, *(repr(float(v)) for v in seed)])


def parse_result_csv(text: str) -> list[JudgeResult]:
    """Parse the judge's result CSV, keeping "no pose" distinct from "q = 0".

    The ``q*`` columns are empty for every reason that left no pose
    (``CatchPoseIkResult::q`` is only valid for ``none`` /
    ``below_manip_min`` / ``rank_deficient``). A row whose ``q*`` cells are all
    empty gets ``q = None``; a row whose first ``nv`` cells are all filled gets
    the vector; anything in between is a garbled row and raises, because a
    half-written pose is not something to average over.
    """
    reader = csv.reader(text.splitlines())
    rows = [r for r in reader if r and any(cell.strip() for cell in r[:1])]
    if not rows:
        raise ValueError("result CSV is empty (a header line is expected)")
    header = [cell.strip() for cell in rows[0]]
    index = {name: i for i, name in enumerate(header)}
    missing = [name for name, _ in _RESULT_SCALARS if name not in index]
    if missing:
        raise ValueError(f"result CSV header lacks {missing} (header was {header})")
    q_columns = sorted(
        (
            (int(name[1:]), i)
            for name, i in index.items()
            if len(name) > 1 and name[0] == "q" and name[1:].isdigit()
        ),
    )
    q_indices = [i for _, i in q_columns]

    out: list[JudgeResult] = []
    for line_no, row in enumerate(rows[1:], start=2):
        if len(row) != len(header):
            raise ValueError(
                f"result CSV line {line_no}: expected {len(header)} columns, got {len(row)}"
            )
        values: dict[str, object] = {}
        for name, kind in _RESULT_SCALARS:
            cell = row[index[name]].strip()
            if kind is str:
                values[name] = cell
            elif cell == "":
                raise ValueError(f"result CSV line {line_no}: column '{name}' is empty")
            else:
                values[name] = kind(float(cell)) if kind is int else float(cell)
        nv = int(values["nv"])
        cells = [row[i].strip() for i in q_indices]
        filled = [i for i, cell in enumerate(cells) if cell != ""]
        if not filled:
            pose: np.ndarray | None = None
        elif filled == list(range(nv)):
            pose = np.asarray([float(cells[i]) for i in range(nv)], dtype=float)
        else:
            raise ValueError(
                f"result CSV line {line_no}: {len(filled)} of the {nv} q columns are filled — "
                "a partially written pose is not a pose"
            )
        out.append(
            JudgeResult(
                id=int(values["id"]),
                seed_id=int(values["seed_id"]),
                accepted=bool(int(values["accepted"])),
                reason=int(values["reason"]),
                reason_name=str(values["reason_name"]),
                iterations=int(values["iterations"]),
                pos_error=float(values["pos_error"]),
                theta=float(values["theta"]),
                w5=float(values["w5"]),
                w6=float(values["w6"]),
                w5_valid=bool(int(values["w5_valid"])),
                w6_valid=bool(int(values["w6_valid"])),
                manip_converged=bool(int(values["manip_converged"])),
                manip_grad_norm=float(values["manip_grad_norm"]),
                manip_grad_failures=int(values["manip_grad_failures"]),
                sigma_min=float(values["sigma_min"]),
                lambda_sq=float(values["lambda_sq"]),
                qp_status=int(values["qp_status"]),
                qp_iterations=int(values["qp_iterations"]),
                qp_failures=int(values["qp_failures"]),
                nv=nv,
                q=pose,
            )
        )
    return out


def shard_is_complete(out_path: Path, expected_ids: Sequence[int]) -> bool:
    """True iff ``out_path`` holds exactly one parseable row per expected id.

    Resumability rests on this, so it is a CHECK and not an assumption: a shard
    whose judge died halfway leaves a short file, and reading that as the answer
    would report those throws as "not catchable". Row count alone would already
    catch the truncation; the id set is compared as well so a stale file from a
    different shard cannot pass.

    **NECESSARY, NOT SUFFICIENT, FOR REUSE.** Candidate ids are always
    ``0..N-1``, so a file that passes this check may still hold the verdicts of
    a different params file, seed, model or grid. Whether an existing shard may
    be REUSED is :func:`shard_is_reusable`, which adds the input fingerprint.
    """
    path = Path(out_path)
    if not path.is_file():
        return False
    try:
        rows = parse_result_csv(path.read_text())
    except (ValueError, OSError):
        return False
    return [r.id for r in rows] == list(expected_ids)


# Bumped whenever the set or the encoding of the fingerprinted inputs changes,
# so a sidecar written under an older rule never matches a newer one.
SHARD_FINGERPRINT_SCHEMA = "rtc_tools.catchability_map/shard-fingerprint/1"

# Keys of the model config whose VALUE is a path to a file the judge loads. The
# config's own bytes only name these files; their content is what decides the
# model, so each is hashed as well.
_MODEL_CONFIG_FILE_KEYS = ("urdf_path", "closure_yaml_path")


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def invocation_fingerprint_parts(
    invocation: JudgeInvocation, *, extra_env: Mapping[str, str] | None = None
) -> dict[str, str]:
    """Everything that decides a verdict and is the SAME for every shard of a run.

    One labelled entry per input, so a mismatch can say WHICH input changed:

    * ``model_config`` — sha256 of the model config file's bytes,
    * ``urdf_path`` / ``closure_yaml_path`` — sha256 of the bytes of each file
      the model config points at (the expanded URDF; the closure YAML when one
      is declared). A relative path is resolved against the model config's
      directory,
    * ``seeds`` — sha256 of the seed file's bytes,
    * ``params`` — sha256 of the params file's bytes, or the literal ``none``
      when the judge runs on its in-code defaults,
    * ``sub_model`` / ``catch_frame`` — the names, verbatim,
    * ``judge`` — the executable's IDENTITY: resolved path, size in bytes and
      ``st_mtime_ns``. **This is identity, not content**: a multi-megabyte
      binary is not re-hashed per run, and a rebuild that changes behaviour
      changes size or mtime in practice. The blind spot is a rebuild that
      preserves both (or a shared library the judge links changing underneath
      it); after such a change, delete the shard directories,
    * ``extra_env`` — the caller's extra child environment, sorted.

    Raises ``OSError`` / ``ValueError`` when an input cannot be read. The caller
    must treat that as "nothing is reusable" — see :func:`run_judge_batch`.
    """
    model_config = Path(invocation.model_config)
    model_bytes = model_config.read_bytes()
    parts: dict[str, str] = {
        "schema": SHARD_FINGERPRINT_SCHEMA,
        "model_config": _sha256(model_bytes),
    }
    doc = yaml.safe_load(model_bytes)
    if not isinstance(doc, Mapping):
        raise ValueError(f"model config {model_config} is not a YAML map")
    for key in _MODEL_CONFIG_FILE_KEYS:
        ref = doc.get(key)
        if ref is None or str(ref) == "":
            if key == "urdf_path":
                raise ValueError(f"model config {model_config} names no urdf_path")
            parts[key] = "none"
            continue
        target = Path(str(ref))
        if not target.is_absolute():
            target = model_config.parent / target
        parts[key] = _sha256(target.read_bytes())
    parts["seeds"] = _sha256(Path(invocation.seeds).read_bytes())
    parts["params"] = (
        "none" if invocation.params is None else _sha256(Path(invocation.params).read_bytes())
    )
    parts["sub_model"] = invocation.sub_model
    parts["catch_frame"] = invocation.catch_frame
    judge = Path(invocation.judge).resolve()
    stat = judge.stat()
    parts["judge"] = f"{judge}|size={stat.st_size}|mtime_ns={stat.st_mtime_ns}"
    parts["extra_env"] = json.dumps(sorted((extra_env or {}).items()))
    return parts


def shard_fingerprint(invocation_parts: Mapping[str, str], candidate_bytes: bytes) -> dict:
    """The sidecar document for one shard: the labelled parts and their digest.

    ``candidate_bytes`` are the shard's candidate CSV exactly as written
    (:func:`candidate_csv_bytes`), which is where a changed grid VALUE shows up:
    the ids of a re-run are the same ``0..N-1`` whatever the coordinates are.
    """
    parts = {**dict(invocation_parts), "candidates": _sha256(candidate_bytes)}
    canonical = json.dumps(parts, sort_keys=True, separators=(",", ":")).encode()
    return {"fingerprint": _sha256(canonical), "parts": parts}


def read_shard_fingerprint(path: Path) -> dict | None:
    """The sidecar at ``path``, or None when it is absent or unreadable."""
    try:
        doc = json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None
    if not isinstance(doc, dict) or not isinstance(doc.get("fingerprint"), str):
        return None
    return doc


def shard_is_reusable(
    out_path: Path,
    expected_ids: Sequence[int],
    fingerprint_path: Path,
    expected: Mapping | None,
) -> bool:
    """True iff the shard is complete AND was produced from these very inputs.

    ``expected`` is this run's :func:`shard_fingerprint` document, or None when
    the inputs could not be fingerprinted. A None, a missing sidecar (an output
    directory written before fingerprints existed), an unreadable one and a
    mismatching one all mean the same thing: NOT reusable, re-judge and
    overwrite. The failure this guards is silent by construction — a stale shard
    has the right ids, the right seed ids and plausible numbers, so nothing
    downstream can contradict it.
    """
    if expected is None:
        return False
    stored = read_shard_fingerprint(fingerprint_path)
    if stored is None or stored["fingerprint"] != expected["fingerprint"]:
        return False
    return shard_is_complete(out_path, expected_ids)


def _changed_fingerprint_parts(stored: Mapping | None, expected: Mapping) -> list[str]:
    old = (stored or {}).get("parts")
    if not isinstance(old, Mapping):
        return ["<no fingerprint sidecar>"]
    new = expected["parts"]
    return sorted(k for k in set(old) | set(new) if old.get(k) != new.get(k))


def _judge_argv(invocation: JudgeInvocation, candidates_in: Path, out: Path) -> list[str]:
    argv = [
        str(invocation.judge),
        "--model-config",
        str(invocation.model_config),
        "--sub-model",
        invocation.sub_model,
        "--catch-frame",
        invocation.catch_frame,
        "--candidates",
        str(candidates_in),
        "--seeds",
        str(invocation.seeds),
        "--out",
        str(out),
    ]
    if invocation.params is not None:
        argv += ["--params", str(invocation.params)]
    return argv


def _child_env(extra: Mapping[str, str] | None = None) -> dict[str, str]:
    """The child environment: one OpenMP thread per judge, plus the caller's extras.

    The shards are the parallelism. Letting each child spawn an OpenMP pool as
    well would oversubscribe the machine and make the wall time depend on how
    many shards happened to overlap (reference: OpenMP owns its own threads).
    """
    env = dict(os.environ)
    env["OMP_NUM_THREADS"] = "1"
    if extra:
        env.update(extra)
    return env


def dump_catch_frame(invocation: JudgeInvocation) -> dict:
    """Run ``--dump-frame`` and return the judge's own view of the model.

    Two things come back that nothing else can supply: ``nv`` of the sub-model
    the judge will actually solve in (which is how many entries a seed needs),
    and the residual between the built catch frame and the declared offset. A
    non-zero exit means the frame does not sit where the config says, and that
    is raised rather than reported as a field.
    """
    argv = [
        str(invocation.judge),
        "--model-config",
        str(invocation.model_config),
        "--sub-model",
        invocation.sub_model,
        "--catch-frame",
        invocation.catch_frame,
        "--dump-frame",
    ]
    proc = subprocess.run(argv, capture_output=True, text=True, env=_child_env(), check=False)
    if proc.returncode != 0:
        raise RuntimeError(
            f"{JUDGE_EXECUTABLE} --dump-frame exited {proc.returncode}\n"
            f"argv: {' '.join(argv)}\nstderr:\n{proc.stderr}"
        )
    info: dict = {"argv": argv, "stdout": proc.stdout}
    for line in proc.stdout.splitlines():
        parts = line.split()
        if parts and parts[0] == "frame" and "nv" in parts:
            info["frame"] = parts[1]
            info["parent"] = parts[3]
            info["nv"] = int(parts[parts.index("nv") + 1])
        elif len(parts) == 2 and parts[0] in (
            "residual_translation_m",
            "residual_rotation_fro",
        ):
            info[parts[0]] = float(parts[1])
    if "nv" not in info:
        raise RuntimeError(f"could not read nv out of --dump-frame output:\n{proc.stdout}")
    return info


def run_judge_batch(
    invocation: JudgeInvocation,
    candidates: Sequence[JudgeCandidate],
    *,
    work_dir: Path,
    shard_size: int = 200,
    workers: int = DEFAULT_WORKERS,
    prefix: str = "shard",
    extra_env: Mapping[str, str] | None = None,
) -> list[JudgeResult]:
    """Judge every candidate, sharded across at most :data:`MAX_WORKERS` processes.

    Resumable: a shard is not re-run when its output is complete AND its
    fingerprint sidecar (``<prefix>_<n>_fingerprint.json``) matches this run's
    inputs (:func:`shard_is_reusable`, :func:`invocation_fingerprint_parts`).
    Completeness alone is not enough — ids are ``0..N-1`` on every run, so a
    re-run into the same directory after changing the params file, a seed, the
    robot config or a grid value would otherwise be answered with the OLD
    verdicts. If this run's inputs cannot be fingerprinted (a file is
    unreadable), nothing is reused and no sidecar is written. The sidecar is
    removed before the judge starts and written only after its output has been
    validated, so a sidecar on disk always vouches for the file next to it.

    Fails closed: a non-zero exit, an unparseable file or a short file raises
    with that shard's stderr attached, because a silently short shard reads as
    "those throws were not catchable" and there is no later signal that would
    contradict it.

    Results come back in the input order, not in completion order.
    """
    if not candidates:
        return []
    if shard_size <= 0:
        raise ValueError(f"shard_size must be > 0 (got {shard_size})")
    workers = max(1, min(int(workers), MAX_WORKERS))
    work_dir = Path(work_dir)
    work_dir.mkdir(parents=True, exist_ok=True)

    chunks = [candidates[i : i + shard_size] for i in range(0, len(candidates), shard_size)]
    try:
        invocation_parts: dict[str, str] | None = invocation_fingerprint_parts(
            invocation, extra_env=extra_env
        )
    except (OSError, ValueError, yaml.YAMLError):
        # The judge will say what is wrong with its inputs; all that matters
        # here is that an unfingerprintable run reuses nothing.
        invocation_parts = None

    def run_one(job: tuple[int, Sequence[JudgeCandidate]]) -> list[JudgeResult]:
        shard, chunk = job
        stem = work_dir / f"{prefix}_{shard:05d}"
        in_path = stem.with_name(stem.name + "_in.csv")
        out_path = stem.with_name(stem.name + "_out.csv")
        err_path = stem.with_name(stem.name + "_err.txt")
        fingerprint_path = stem.with_name(stem.name + "_fingerprint.json")
        expected_ids = [c.id for c in chunk]
        candidate_bytes = candidate_csv_bytes(chunk)
        expected = (
            None
            if invocation_parts is None
            else shard_fingerprint(invocation_parts, candidate_bytes)
        )
        if shard_is_reusable(out_path, expected_ids, fingerprint_path, expected):
            return parse_result_csv(out_path.read_text())
        if expected is not None and out_path.is_file():
            changed = _changed_fingerprint_parts(
                read_shard_fingerprint(fingerprint_path), expected
            )
            print(
                f"{prefix} {shard}: existing output is not reusable, re-judging "
                f"(changed or incomplete: {changed})",
                file=sys.stderr,
            )
        # Both go before the judge starts: a judge that exits 0 without writing
        # would otherwise leave the STALE output to be parsed below, and its ids
        # would match.
        fingerprint_path.unlink(missing_ok=True)
        out_path.unlink(missing_ok=True)
        in_path.write_bytes(candidate_bytes)
        argv = _judge_argv(invocation, in_path, out_path)
        proc = subprocess.run(
            argv, capture_output=True, text=True, env=_child_env(extra_env), check=False
        )
        err_path.write_text(proc.stderr)
        if proc.returncode != 0:
            raise RuntimeError(
                f"{JUDGE_EXECUTABLE} shard {shard} exited {proc.returncode}\n"
                f"argv: {' '.join(argv)}\nstderr:\n{proc.stderr}"
            )
        try:
            rows = parse_result_csv(out_path.read_text())
        except (ValueError, OSError) as exc:
            raise RuntimeError(
                f"{JUDGE_EXECUTABLE} shard {shard} produced an unreadable CSV ({exc})\n"
                f"argv: {' '.join(argv)}\nstderr:\n{proc.stderr}"
            ) from exc
        if [r.id for r in rows] != expected_ids:
            raise RuntimeError(
                f"{JUDGE_EXECUTABLE} shard {shard} returned {len(rows)} rows for "
                f"{len(expected_ids)} candidates (ids do not match) — a short shard would "
                f"read as 'not catchable'\nargv: {' '.join(argv)}\nstderr:\n{proc.stderr}"
            )
        if expected is not None:
            fingerprint_path.write_text(json.dumps(expected, indent=2, sort_keys=True) + "\n")
        return rows

    jobs = list(enumerate(chunks))
    if workers == 1:
        return [row for job in jobs for row in run_one(job)]
    with ThreadPoolExecutor(max_workers=workers) as pool:
        return [row for rows in pool.map(run_one, jobs) for row in rows]


def to_judge_candidates(
    candidates: Sequence[CatchCandidate],
    *,
    model_world_t_world,
    seed_ids: Sequence[int] = (0,),
    id_start: int = 0,
) -> list[JudgeCandidate]:
    """Convert world-frame catch instants into the judge's model-world rows.

    Every candidate is emitted once per seed id, which is what makes the wait-
    pose comparison (:func:`rank_wait_pose_seeds`) a comparison over the SAME
    throw set rather than over two runs that happened to use different grids.
    """
    transform = as_transform(model_world_t_world)
    if not seed_ids:
        raise ValueError("seed_ids must not be empty")
    out: list[JudgeCandidate] = []
    next_id = int(id_start)
    for candidate in candidates:
        p_model, v_model = world_to_base(transform, candidate.position_m, candidate.velocity_m_s)
        for seed_id in seed_ids:
            out.append(
                JudgeCandidate(
                    id=next_id,
                    seed_id=int(seed_id),
                    p_c_model_m=p_model.copy(),
                    v_model_m_s=v_model.copy(),
                    throw_index=candidate.throw_index,
                    time_s=candidate.time_s,
                    speed_m_s=candidate.speed_m_s,
                    p_c_world_m=candidate.position_m.copy(),
                    v_world_m_s=candidate.velocity_m_s.copy(),
                )
            )
            next_id += 1
    return out


def perturb_candidates(
    candidates: Sequence[JudgeCandidate], *, epsilon_m: float, id_offset: int
) -> tuple[list[JudgeCandidate], dict[int, int]]:
    """Six axis-aligned ±ε copies of every candidate, plus ``perturbed id → source id``.

    This is the candidate-GENERATION half of :func:`count_boundary_candidates`:
    the boundary test re-judges these copies rather than guessing from the
    nominal row's numbers, so the answer is the judge's and not a proxy.
    """
    epsilon = float(epsilon_m)
    if not (math.isfinite(epsilon) and epsilon > 0.0):
        raise ValueError(f"epsilon_m must be finite and > 0 (got {epsilon_m!r})")
    out: list[JudgeCandidate] = []
    sources: dict[int, int] = {}
    next_id = int(id_offset)
    for candidate in candidates:
        for axis in range(3):
            for sign in (-1.0, 1.0):
                delta = np.zeros(3)
                delta[axis] = sign * epsilon
                out.append(
                    JudgeCandidate(
                        id=next_id,
                        seed_id=candidate.seed_id,
                        p_c_model_m=np.asarray(candidate.p_c_model_m, dtype=float) + delta,
                        v_model_m_s=np.asarray(candidate.v_model_m_s, dtype=float).copy(),
                        throw_index=candidate.throw_index,
                        time_s=candidate.time_s,
                        speed_m_s=candidate.speed_m_s,
                    )
                )
                sources[next_id] = candidate.id
                next_id += 1
    return out, sources


# ── Aggregation ───────────────────────────────────────────────────────────────


@dataclass(frozen=True, eq=False)
class JudgedCandidate:
    """A candidate together with the judge's verdict on it."""

    candidate: JudgeCandidate
    result: JudgeResult


def join_results(
    candidates: Sequence[JudgeCandidate], results: Sequence[JudgeResult]
) -> list[JudgedCandidate]:
    """Join candidates and results on ``id``, refusing a partial join.

    A result that names an unknown id, or a candidate with no result, is an
    error: both would silently shrink the map.
    """
    by_id = {c.id: c for c in candidates}
    if len(by_id) != len(candidates):
        raise ValueError("candidate ids are not unique")
    seen: set[int] = set()
    out: list[JudgedCandidate] = []
    for result in results:
        candidate = by_id.get(result.id)
        if candidate is None:
            raise ValueError(f"result id {result.id} has no candidate")
        if result.id in seen:
            raise ValueError(f"result id {result.id} appears twice")
        if result.seed_id != candidate.seed_id:
            raise ValueError(
                f"result id {result.id} names seed {result.seed_id}, candidate says "
                f"{candidate.seed_id}"
            )
        seen.add(result.id)
        out.append(JudgedCandidate(candidate=candidate, result=result))
    missing = sorted(set(by_id) - seen)
    if missing:
        raise ValueError(f"{len(missing)} candidates have no result (first: {missing[:5]})")
    return out


def _manip(result: JudgeResult, column: str) -> float:
    if column == "w5":
        return result.w5 if result.w5_valid else float("-inf")
    if column == "w6":
        return result.w6 if result.w6_valid else float("-inf")
    raise ValueError(f"manip column must be 'w5' or 'w6' (got {column!r})")


@dataclass(frozen=True, eq=False)
class ThrowOutcome:
    """Whether a throw is catchable, and the best accepted candidate if it is."""

    throw_index: int
    candidates: int
    accepted_candidates: int
    accepted: bool
    best_id: int | None = None
    best_seed_id: int | None = None
    best_w: float | None = None
    best_w5: float | None = None
    best_w6: float | None = None
    best_time_s: float | None = None
    best_speed_m_s: float | None = None
    best_p_c_world_m: np.ndarray | None = None
    best_p_c_model_m: np.ndarray | None = None
    best_q: np.ndarray | None = None
    best_theta: float | None = None


def judged_for_seed(judged: Sequence[JudgedCandidate], seed_id: int) -> list[JudgedCandidate]:
    """The rows one wait-pose seed produced — the unit every headline figure is over."""
    return [i for i in judged if i.result.seed_id == int(seed_id)]


def summarize_throws(
    judged: Sequence[JudgedCandidate],
    *,
    manip_column: str = "w5",
    seed_id: int | None = None,
    throw_count: int | None = None,
) -> list[ThrowOutcome]:
    """Per throw, FOR ONE WAIT-POSE SEED: was anything accepted, and the max-w pose.

    **One seed, never a union.** A robot waits in ONE pose, so "this throw is
    catchable" only means something for a fixed seed. Grouping rows of several
    seeds by throw would call a throw accepted when ANY seed accepts it, which
    overstates every figure built on it (the headline, the ``throw_region``
    proposal, the azimuth plot). So: ``seed_id`` selects the seed, and rows of
    more than one seed WITHOUT a ``seed_id`` are refused rather than pooled. The
    any-seed count exists only under its own name,
    :func:`count_throws_accepted_by_any_seed`.

    ``throw_count`` is the size of the throw grid. When given, the result has
    exactly one outcome per grid throw, in index order: a throw whose every
    catch instant was dropped before the judge (reach pre-filter, flight-time
    floor) never appears in ``judged``, and it is reported as ``candidates = 0``
    / ``accepted = False`` instead of being absent. Without it only the throws
    that reached the judge are listed, which is NOT the grid.

    ``manip_column`` selects which manipulability ranks the accepted candidates.
    It is a parameter because the GATE's definition is a YAML choice
    (``planner.catchability.definition``) — but w₅ and w₆ are two measurements
    of the SAME pose (plan §11 C-3), so ranking by one and reporting both is
    coherent. A candidate whose chosen measure is invalid ranks last rather than
    being dropped.
    """
    present = sorted({i.result.seed_id for i in judged})
    if seed_id is None:
        if len(present) > 1:
            raise ValueError(
                f"rows of {len(present)} wait-pose seeds {present} and no seed_id: a throw is "
                "catchable from ONE wait pose, and pooling seeds would count a throw any seed "
                "accepts (use count_throws_accepted_by_any_seed for that figure, by name)"
            )
        rows = list(judged)
    else:
        if present and int(seed_id) not in present:
            raise ValueError(f"seed_id {seed_id} is not among the judged seeds {present}")
        rows = judged_for_seed(judged, seed_id)

    groups: dict[int, list[JudgedCandidate]] = {}
    for item in rows:
        groups.setdefault(item.candidate.throw_index, []).append(item)
    if throw_count is None:
        indices = sorted(groups)
    else:
        outside = sorted(i for i in groups if not 0 <= i < int(throw_count))
        if outside:
            raise ValueError(
                f"throw_index {outside[:5]} lies outside the grid of {throw_count} throws"
            )
        indices = list(range(int(throw_count)))
    out: list[ThrowOutcome] = []
    for throw_index in indices:
        items = groups.get(throw_index, [])
        accepted = [i for i in items if i.result.accepted]
        if not accepted:
            out.append(
                ThrowOutcome(
                    throw_index=throw_index,
                    candidates=len(items),
                    accepted_candidates=0,
                    accepted=False,
                )
            )
            continue
        best = max(accepted, key=lambda i: _manip(i.result, manip_column))
        out.append(
            ThrowOutcome(
                throw_index=throw_index,
                candidates=len(items),
                accepted_candidates=len(accepted),
                accepted=True,
                best_id=best.result.id,
                best_seed_id=best.result.seed_id,
                best_w=_manip(best.result, manip_column),
                best_w5=best.result.w5,
                best_w6=best.result.w6,
                best_time_s=best.candidate.time_s,
                best_speed_m_s=best.candidate.speed_m_s,
                best_p_c_world_m=best.candidate.p_c_world_m,
                best_p_c_model_m=best.candidate.p_c_model_m,
                best_q=best.result.q,
                best_theta=best.result.theta,
            )
        )
    return out


ANY_SEED_NOTE = (
    "union over wait-pose seeds: a throw counts when AT LEAST ONE seed accepts it. NOT "
    "coverage — no single wait pose achieves it. accepted_throws is the one-seed figure."
)


def count_throws_accepted_by_any_seed(judged: Sequence[JudgedCandidate]) -> int:
    """Throws that AT LEAST ONE seed accepts — a union over wait poses.

    **NOT COVERAGE.** No single wait pose achieves this number; it is an upper
    bound on what a better-chosen pose could reach, and it is only ever reported
    under a name that says ``any_seed``. The figure a robot can actually deliver
    is the per-seed one (:func:`summarize_throws` with a ``seed_id``).
    """
    return len({i.candidate.throw_index for i in judged if i.result.accepted})


def reason_histogram(judged: Sequence[JudgedCandidate]) -> dict[str, int]:
    """Rejection reasons over the REJECTED candidates, by ``reason_name``.

    Accepted candidates are excluded on purpose: ``none`` in a rejection
    histogram is the one bar that cannot be read.
    """
    counts: dict[str, int] = {}
    for item in judged:
        if item.result.accepted:
            continue
        counts[item.result.reason_name] = counts.get(item.result.reason_name, 0) + 1
    return dict(sorted(counts.items(), key=lambda kv: (-kv[1], kv[0])))


def distribution(values: Iterable[float]) -> dict:
    """n / min / p05 / p25 / median / p75 / p95 / max / mean of finite values."""
    array = np.asarray([float(v) for v in values], dtype=float)
    array = array[np.isfinite(array)]
    if array.size == 0:
        return {"n": 0}
    percentiles = np.percentile(array, [5.0, 25.0, 50.0, 75.0, 95.0])
    return {
        "n": int(array.size),
        "min": float(array.min()),
        "p05": float(percentiles[0]),
        "p25": float(percentiles[1]),
        "median": float(percentiles[2]),
        "p75": float(percentiles[3]),
        "p95": float(percentiles[4]),
        "max": float(array.max()),
        "mean": float(array.mean()),
    }


def manipulability_distributions(judged: Sequence[JudgedCandidate]) -> dict:
    """w₅ and w₆ over ACCEPTED candidates, reported SEPARATELY.

    They are not pooled and no combined statistic is offered: L3 §6 keeps a row
    for each because they are quantities of different dimension ("차원이 달라
    따로 둔다", C-3), so a mean over the union would be a number about nothing.
    Only the rows whose own ``*_valid`` flag is set are counted.
    """
    accepted = [i.result for i in judged if i.result.accepted]
    return {
        "accepted_candidates": len(accepted),
        "w5": distribution(r.w5 for r in accepted if r.w5_valid),
        "w6": distribution(r.w6 for r in accepted if r.w6_valid),
        "w5_invalid": sum(1 for r in accepted if not r.w5_valid),
        "w6_invalid": sum(1 for r in accepted if not r.w6_valid),
        "note": (
            "w5 = sqrt(det(J5 J5^T)) and w6 = sqrt(det(J6 J6^T)) at the same q*; mixed-unit "
            "quantities of different dimension, never pooled (plan §11 C-3)"
        ),
    }


# Mirror of the judge's in-code provisional default, used ONLY when neither the
# params file nor the caller states a value. A mirror can rot, which is why it
# is never applied silently: :func:`resolve_alpha_max` labels it as a mirror,
# with this source, in the provenance.
JUDGE_DEFAULT_ALPHA_MAX_RAD = 0.26
JUDGE_DEFAULT_ALPHA_MAX_SOURCE = (
    "rtc_controllers/include/rtc_controllers/catching/catch_pose_ik.hpp "
    "CatchPoseIkOptions::alpha_max (the judge's in-code provisional default; this tool "
    "MIRRORS it, it does not read it back from the binary)"
)
ALPHA_MAX_KEY = "planner.ik.alpha_max"
TBD_LITERAL = "TBD"

PARAMS_SHAPE_CATCHING_AT_ROOT = "`catching:` at the document root"
PARAMS_SHAPE_CONTROLLER_CONFIG = (
    "the shipped controller-config shape, `<controller_name>: {catching: ...}`"
)
PARAMS_SHAPE_TREE_ITSELF = "the catching tree itself (`planner:` at the document root)"


@dataclass(frozen=True)
class AlphaMaxResolution:
    """The ``alpha_max`` the θ report is computed against, and where it came from."""

    value_rad: float
    origin: str  # "params" | "flag" | "judge_default"
    source: str
    params_shape: str | None = None
    params_value_rad: float | None = None
    flag_value_rad: float | None = None
    warning: str | None = None
    judge_reported_rad: float | None = None

    def as_provenance(self) -> dict:
        return {
            "alpha_max_rad": self.value_rad,
            "origin": self.origin,
            "source": self.source,
            "params_shape": self.params_shape,
            "params_value_rad": self.params_value_rad,
            "flag_value_rad": self.flag_value_rad,
            "judge_default_mirror_rad": JUDGE_DEFAULT_ALPHA_MAX_RAD,
            "judge_reported_rad": self.judge_reported_rad,
            "judge_report": (
                "confirmed by the judge's own --print-options"
                if self.judge_reported_rad is not None
                else "NOT cross-checked — this judge binary does not offer --print-options"
            ),
            "warning": self.warning,
        }


def catching_tree_from_params(doc: object, *, label: str = "params") -> tuple[Mapping, str]:
    """The ``catching`` tree out of a params document, plus which shape it had.

    Three shapes are accepted — the same three, under the same rules, as the
    judge's own params loader, so the two cannot locate different trees in one
    file:

    1. a ``catching:`` map at the document root,
    2. the catching tree itself, recognised by a ``planner:`` map at the root
       (both 1 and 2 at once is refused as ambiguous),
    3. the shipped controller-config shape, ``<controller_name>: {catching: ...}``
       — the document's ONLY top-level entry, whose value carries a ``catching``
       map.

    Anything else raises. It must not fall through to "no alpha_max here, use
    the default": a file whose tree was not FOUND is not a file that leaves the
    value open, and treating it as one reports θ against a bound nobody chose.
    """
    if not isinstance(doc, Mapping):
        raise ValueError(f"{label}: the document root must be a map (got {type(doc).__name__})")

    def has_map(node: object, key: str) -> bool:
        return isinstance(node, Mapping) and isinstance(node.get(key), Mapping)

    if has_map(doc, "catching") and has_map(doc, "planner"):
        raise ValueError(
            f"{label}: both a top-level `catching:` and a top-level `planner:` map — which "
            "one is the catching tree is ambiguous"
        )
    if has_map(doc, "catching"):
        return doc["catching"], PARAMS_SHAPE_CATCHING_AT_ROOT
    if has_map(doc, "planner"):
        return doc, PARAMS_SHAPE_TREE_ITSELF
    if len(doc) == 1:
        (only,) = doc.values()
        if has_map(only, "catching"):
            return only["catching"], PARAMS_SHAPE_CONTROLLER_CONFIG
    raise ValueError(
        f"{label}: no `catching` tree found. Accepted shapes: "
        f"{PARAMS_SHAPE_CATCHING_AT_ROOT}; {PARAMS_SHAPE_TREE_ITSELF}; "
        f"{PARAMS_SHAPE_CONTROLLER_CONFIG} (as the ONLY top-level entry). Top-level keys "
        f"were {sorted(map(str, doc))}"
    )


def alpha_max_from_catching_tree(tree: Mapping, *, label: str = "params") -> float | None:
    """``planner.ik.alpha_max`` [rad], or None when the tree leaves it open.

    "Open" is: the key (or a section above it) absent or null, the literal
    string ``TBD``, or a non-finite number — the same three the judge's parser
    treats as TBD before falling back to its in-code default. Any other
    non-number raises, as it does in the judge.
    """
    node: object = tree
    walked = ""
    for key in ("planner", "ik"):
        if node is None:
            return None
        if not isinstance(node, Mapping):
            raise ValueError(f"{label}: section '{walked}' must be a map")
        node = node.get(key)
        walked = f"{walked}.{key}" if walked else key
    if node is None:
        return None
    if not isinstance(node, Mapping):
        raise ValueError(f"{label}: section '{walked}' must be a map")
    raw = node.get("alpha_max")
    if raw is None or (isinstance(raw, str) and raw.strip() == TBD_LITERAL):
        return None
    if isinstance(raw, bool):
        raise ValueError(f"{label}: {ALPHA_MAX_KEY} must be a number or '{TBD_LITERAL}'")
    try:
        # A str is allowed through float(): YAML 1.1 reads `2.6e-1` without a
        # dot as a string, and the judge's parser reads the same text as a number.
        value = float(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"{label}: {ALPHA_MAX_KEY} must be a number or '{TBD_LITERAL}' (got {raw!r})"
        ) from exc
    if not math.isfinite(value):
        return None
    if value <= 0.0:
        raise ValueError(f"{label}: {ALPHA_MAX_KEY} must be > 0 (got {value!r})")
    return value


def resolve_alpha_max(
    params_path: Path | None, flag_value_rad: float | None = None
) -> AlphaMaxResolution:
    """The ``alpha_max`` the JUDGE applied — one source of truth, named.

    The θ report is the evidence plan §11 names for closing
    ``planner.ik.alpha_max``, so it has to be computed against the bound the
    judge actually accepted candidates under, not against a number this tool
    happens to default to.

    * the params file specifies it → that value. A ``--alpha-max-rad`` that
      AGREES is redundant and accepted; one that DIFFERS is an error — two
      sources of truth, and whichever "won" the report would be wrong about the
      other,
    * only ``--alpha-max-rad`` → that value. The judge was then NOT handed an
      alpha_max and applied its in-code default, so the flag is the caller's
      statement of that default; if it differs from this tool's mirror
      (:data:`JUDGE_DEFAULT_ALPHA_MAX_RAD`) a ``warning`` says what has to be
      true for the report to be right,
    * neither → the mirror, labelled as the judge's in-code default with its
      source.

    A params file whose ``catching`` tree cannot be located raises
    (:func:`catching_tree_from_params`).
    """
    if flag_value_rad is not None and not (
        math.isfinite(float(flag_value_rad)) and float(flag_value_rad) > 0.0
    ):
        raise ValueError(f"--alpha-max-rad must be finite and > 0 (got {flag_value_rad!r})")
    flag = None if flag_value_rad is None else float(flag_value_rad)

    shape: str | None = None
    from_params: float | None = None
    if params_path is not None:
        label = str(params_path)
        tree, shape = catching_tree_from_params(
            yaml.safe_load(Path(params_path).read_text()), label=label
        )
        from_params = alpha_max_from_catching_tree(tree, label=label)

    if from_params is not None:
        if flag is not None and not math.isclose(flag, from_params, rel_tol=1e-12, abs_tol=0.0):
            raise ValueError(
                f"two sources of truth for alpha_max: {params_path} sets {ALPHA_MAX_KEY} = "
                f"{from_params!r} (which is what the judge applied) and --alpha-max-rad says "
                f"{flag!r}. Drop the flag, or make the two agree."
            )
        return AlphaMaxResolution(
            value_rad=from_params,
            origin="params",
            source=f"{params_path}:{ALPHA_MAX_KEY} ({shape}) — the file the judge read",
            params_shape=shape,
            params_value_rad=from_params,
            flag_value_rad=flag,
        )
    open_note = (
        "no --params was given"
        if params_path is None
        else f"{params_path} leaves {ALPHA_MAX_KEY} open (absent or '{TBD_LITERAL}')"
    )
    if flag is not None:
        differs = not math.isclose(flag, JUDGE_DEFAULT_ALPHA_MAX_RAD, rel_tol=1e-12, abs_tol=0.0)
        return AlphaMaxResolution(
            value_rad=flag,
            origin="flag",
            source=(
                f"--alpha-max-rad; {open_note}, so the judge applied its in-code default and "
                "this flag is the caller's statement of it"
            ),
            params_shape=shape,
            flag_value_rad=flag,
            warning=(
                f"--alpha-max-rad {flag!r} differs from this tool's mirror of the judge's "
                f"in-code default ({JUDGE_DEFAULT_ALPHA_MAX_RAD!r}); the judge was not handed "
                "an alpha_max, so the theta report is right only if the judge binary was "
                f"built with {flag!r}. To CHANGE the bound, set {ALPHA_MAX_KEY} in --params."
                if differs
                else None
            ),
        )
    return AlphaMaxResolution(
        value_rad=JUDGE_DEFAULT_ALPHA_MAX_RAD,
        origin="judge_default",
        source=f"{open_note}; {JUDGE_DEFAULT_ALPHA_MAX_SOURCE}",
        params_shape=shape,
    )


def judge_reported_alpha_max(judge: Path, params_path: Path | None) -> float | None:
    """``planner.ik.alpha_max`` as the JUDGE resolves it, or None if it cannot say.

    Asks the binary itself (``--print-options``, with the same ``--params``)
    instead of trusting this module's reading of the file or its mirror of the
    in-code default. The capability is OPTIONAL: a judge that does not offer it,
    exits non-zero, or prints no parseable ``planner.ik.alpha_max`` line gives
    None, and the caller records that the value was not cross-checked.
    """
    argv = [str(judge), "--print-options"]
    if params_path is not None:
        argv += ["--params", str(params_path)]
    try:
        proc = subprocess.run(
            argv, capture_output=True, text=True, env=_child_env(), check=False, timeout=60
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if proc.returncode != 0:
        return None
    for line in proc.stdout.splitlines():
        parts = line.split()
        if len(parts) == 2 and parts[0] == ALPHA_MAX_KEY:
            try:
                value = float(parts[1])
            except ValueError:
                return None
            return value if math.isfinite(value) else None
    return None


def confirm_alpha_max_with_judge(
    resolution: AlphaMaxResolution, judge_reported_rad: float | None
) -> AlphaMaxResolution:
    """Hold the resolved ``alpha_max`` against what the judge says it applied.

    None (the judge cannot say) leaves the resolution as it is. A value that
    agrees confirms it and retires the mirror warning. A value that DIFFERS
    raises: the θ report would be computed against a bound the judge did not
    use, which is the one thing this whole resolution exists to prevent — e.g. a
    lone ``--alpha-max-rad`` that is not the judge's in-code default, or this
    module's mirror of that default having rotted.
    """
    if judge_reported_rad is None:
        return resolution
    reported = float(judge_reported_rad)
    if not math.isclose(reported, resolution.value_rad, rel_tol=1e-9, abs_tol=0.0):
        raise ValueError(
            f"the judge reports {ALPHA_MAX_KEY} = {reported!r} but this tool resolved "
            f"{resolution.value_rad!r} from {resolution.source}. The theta report must use the "
            f"bound the judge applied; to change the bound, set {ALPHA_MAX_KEY} in --params."
        )
    return replace(resolution, judge_reported_rad=reported, warning=None)


def theta_report(
    judged: Sequence[JudgedCandidate], *, alpha_max: float, near_fraction: float = 0.9
) -> dict:
    """θ over accepted candidates, and how much of it sits near ``alpha_max``.

    θ is ‖e_a^C‖ [rad], the approach-axis cone angle at q*, and ``alpha_max`` is
    the acceptance bound ``planner.ik.alpha_max`` (still provisional, and this
    distribution is the evidence meant to close it — plan §11). It MUST be the
    bound the judge applied: take it from :func:`resolve_alpha_max`, never from
    a constant. Against a smaller number than the judge used,
    ``max_over_alpha_max`` exceeds 1 and ``fraction_near_limit`` is inflated.

    "Within a fraction of ``alpha_max``" is ambiguous in a way that would invert
    the conclusion, so BOTH readings are returned under names that cannot be
    confused:

    * ``fraction_below`` — θ ≤ ``near_fraction``·α_max, i.e. comfortably inside
      the cone,
    * ``fraction_near_limit`` — θ ≥ ``near_fraction``·α_max, i.e. pressed against
      it. **This** is the number that says the cone binds.
    """
    alpha = float(alpha_max)
    if not (math.isfinite(alpha) and alpha > 0.0):
        raise ValueError(f"alpha_max must be finite and > 0 (got {alpha_max!r})")
    if not (0.0 < float(near_fraction) <= 1.0):
        raise ValueError(f"near_fraction must be in (0, 1] (got {near_fraction!r})")
    thetas = np.asarray(
        [i.result.theta for i in judged if i.result.accepted and math.isfinite(i.result.theta)],
        dtype=float,
    )
    threshold = float(near_fraction) * alpha
    n = int(thetas.size)
    return {
        "alpha_max_rad": alpha,
        "near_fraction": float(near_fraction),
        "near_threshold_rad": threshold,
        "theta_rad": distribution(thetas),
        "accepted_candidates": n,
        "count_below": int(np.count_nonzero(thetas <= threshold)) if n else 0,
        "count_near_limit": int(np.count_nonzero(thetas >= threshold)) if n else 0,
        "fraction_below": float(np.count_nonzero(thetas <= threshold) / n) if n else None,
        "fraction_near_limit": float(np.count_nonzero(thetas >= threshold) / n) if n else None,
        "max_over_alpha_max": float(thetas.max() / alpha) if n else None,
    }


# What an "accepted fraction" is a fraction OF. Written into every output that
# carries one, because the two candidates differ by exactly the throws the
# pre-filters emptied, and a fraction without its denominator cannot be compared
# with another one.
DENOMINATOR_GRID_THROWS = (
    "grid_throws — every throw of the grid, including those left with no catch instant by "
    "the reach / flight-time pre-filters"
)
DENOMINATOR_THROWS_WITH_CANDIDATES = (
    "throws_with_candidates — only the throws that reached the judge; NOT the grid, so not "
    "comparable with a grid fraction"
)


@dataclass(frozen=True)
class SeedRanking:
    """One wait-pose seed's coverage of the throw set.

    ``throws`` is the DENOMINATOR of ``accepted_fraction`` and ``denominator``
    says which one it is; ``throws_with_candidates`` is how many of them reached
    the judge at all.
    """

    seed_id: int
    throws: int
    accepted_throws: int
    accepted_fraction: float
    mean_log_w5: float
    accepted_candidates: int
    throws_with_candidates: int = 0
    denominator: str = DENOMINATOR_THROWS_WITH_CANDIDATES


@dataclass(frozen=True)
class SeedComparison:
    """The ranking, the winner, the runner-up and the coverage the runner-up loses."""

    ranking: tuple[SeedRanking, ...]
    best: SeedRanking | None
    runner_up: SeedRanking | None
    coverage_gap: float | None
    throws: int
    throws_with_candidates: int = 0
    denominator: str = DENOMINATOR_THROWS_WITH_CANDIDATES


def rank_wait_pose_seeds(
    judged: Sequence[JudgedCandidate], *, throw_count: int | None = None
) -> SeedComparison:
    """Rank wait-pose seeds over the SAME throw set, coverage first.

    ``throw_count`` is the size of the throw grid and, when given, the
    denominator of every ``accepted_fraction`` here — the SAME denominator the
    headline and the ``throw_region`` proposal use, so the figures can be put
    side by side. Pass it whenever a grid exists. Without it the denominator
    falls back to the throws that reached the judge, and the result says so in
    ``denominator`` rather than leaving the reader to guess.

    Order: accepted-throw fraction DESCENDING, ties broken by mean ``log w5``
    over that seed's accepted candidates (descending), then by ``seed_id`` for
    determinism. Coverage comes first because a better-conditioned pose that
    catches fewer throws is a worse wait pose; ``log w5`` rather than ``w5``
    because w₅ spans orders of magnitude near a singularity and the mean of the
    raw value is then set by its largest member.

    The seeds must cover the same throws — otherwise the fractions are over
    different denominators and the comparison is meaningless, so that is refused
    rather than normalised away. ``coverage_gap`` is the winner's accepted
    fraction minus the runner-up's: the sensitivity of the map to the wait pose.
    """
    per_seed: dict[int, dict[int, list[JudgedCandidate]]] = {}
    for item in judged:
        per_seed.setdefault(item.result.seed_id, {}).setdefault(
            item.candidate.throw_index, []
        ).append(item)
    denominator_label = (
        DENOMINATOR_THROWS_WITH_CANDIDATES if throw_count is None else DENOMINATOR_GRID_THROWS
    )
    if not per_seed:
        return SeedComparison(
            ranking=(),
            best=None,
            runner_up=None,
            coverage_gap=None,
            throws=0 if throw_count is None else int(throw_count),
            throws_with_candidates=0,
            denominator=denominator_label,
        )

    throw_sets = {seed_id: frozenset(groups) for seed_id, groups in per_seed.items()}
    reference = next(iter(throw_sets.values()))
    for seed_id, throws in throw_sets.items():
        if throws != reference:
            raise ValueError(
                f"seed {seed_id} covers {len(throws)} throws, another covers {len(reference)} — "
                "seed ranking compares coverage of the SAME throw set"
            )

    denominator = len(reference) if throw_count is None else int(throw_count)
    if throw_count is not None:
        outside = sorted(i for i in reference if not 0 <= i < denominator)
        if outside:
            raise ValueError(
                f"throw_index {outside[:5]} lies outside the grid of {throw_count} throws"
            )

    rankings: list[SeedRanking] = []
    for seed_id, groups in per_seed.items():
        accepted_throws = sum(
            1 for items in groups.values() if any(i.result.accepted for i in items)
        )
        logs = [
            math.log(i.result.w5)
            for items in groups.values()
            for i in items
            if i.result.accepted and i.result.w5_valid and i.result.w5 > 0.0
        ]
        rankings.append(
            SeedRanking(
                seed_id=seed_id,
                throws=denominator,
                accepted_throws=accepted_throws,
                accepted_fraction=accepted_throws / denominator if denominator else 0.0,
                mean_log_w5=float(np.mean(logs)) if logs else float("-inf"),
                accepted_candidates=sum(
                    1 for items in groups.values() for i in items if i.result.accepted
                ),
                throws_with_candidates=len(groups),
                denominator=denominator_label,
            )
        )
    rankings.sort(key=lambda r: (-r.accepted_fraction, -r.mean_log_w5, r.seed_id))
    best = rankings[0]
    runner_up = rankings[1] if len(rankings) > 1 else None
    return SeedComparison(
        ranking=tuple(rankings),
        best=best,
        runner_up=runner_up,
        coverage_gap=(
            None if runner_up is None else best.accepted_fraction - runner_up.accepted_fraction
        ),
        throws=denominator,
        throws_with_candidates=len(reference),
        denominator=denominator_label,
    )


def count_boundary_candidates(
    nominal: Sequence[JudgeResult],
    perturbed: Sequence[JudgeResult],
    sources: Mapping[int, int],
    *,
    epsilon_m: float,
) -> dict:
    """How many accepted candidates sit within ε of the accept/reject boundary.

    **Boundary, defined.** An accepted candidate is ON the boundary at scale ε
    when at least one of its ε-displaced copies is REJECTED by the judge. The
    displacement is a candidate-generation step (:func:`perturb_candidates`,
    six axis-aligned ±ε copies of p_c), so the verdict on the copy comes from
    the same ``CatchPoseIk::Solve`` as the verdict on the original.

    It is deliberately NOT inferred from the nominal row's columns. "w₅ is close
    to the threshold" would be a proxy for one of the several ways a candidate
    can fail (``not_converged`` and ``qp_failed`` have no margin column at all),
    and it would report a margin in the wrong units.

    ``flip_reasons`` says HOW the copies failed, which distinguishes a
    manipulability margin from a reach margin.
    """
    by_id = {r.id: r for r in nominal}
    accepted_ids = {r.id for r in nominal if r.accepted}
    flipped: dict[int, list[str]] = {}
    probed: set[int] = set()
    for row in perturbed:
        source = sources.get(row.id)
        if source is None:
            raise ValueError(f"perturbed result id {row.id} has no source candidate")
        if source not in by_id:
            raise ValueError(f"perturbed result id {row.id} names unknown source {source}")
        probed.add(source)
        if not row.accepted:
            flipped.setdefault(source, []).append(row.reason_name)
    on_boundary = sorted(accepted_ids & set(flipped))
    probed_accepted = accepted_ids & probed
    reasons: dict[str, int] = {}
    for source in on_boundary:
        for name in flipped[source]:
            reasons[name] = reasons.get(name, 0) + 1
    return {
        "epsilon_m": float(epsilon_m),
        "definition": (
            "an accepted candidate is on the boundary when >=1 of its six axis-aligned "
            "+/-epsilon copies of p_c is rejected by the judge"
        ),
        "accepted": len(accepted_ids),
        "probed_accepted": len(probed_accepted),
        "on_boundary": len(on_boundary),
        "on_boundary_ids": on_boundary,
        "fraction_of_probed_accepted": (
            len(on_boundary) / len(probed_accepted) if probed_accepted else None
        ),
        "flip_reasons": dict(sorted(reasons.items(), key=lambda kv: (-kv[1], kv[0]))),
    }


def _range_of(values: Sequence[float]) -> list[float] | None:
    array = np.asarray([float(v) for v in values], dtype=float)
    if array.size == 0:
        return None
    return [float(array.min()), float(array.max())]


def propose_throw_region(
    throws: Sequence[Throw],
    outcomes: Sequence[ThrowOutcome],
    judged: Sequence[JudgedCandidate],
    *,
    base_frame: str,
    wait_pose_seed_id: int | None = None,
    wait_pose_q: Sequence[float] | None = None,
    accepted_throws_any_seed: int | None = None,
) -> dict:
    """The ``sim.throw_region`` proposal: the axis-aligned box covering accepted throws.

    The keys are the ones plan §11 reserves for this block, in radians where the
    plan uses radians, with the grid's own degree values mirrored under
    ``covered_axes_deg``.

    **The region of ONE wait pose.** ``outcomes`` and ``judged`` must both be
    one seed's (:func:`summarize_throws` with a ``seed_id``,
    :func:`judged_for_seed`); ``judged`` carrying several seeds is refused,
    because ``flight_time_s`` is read from it and would silently become a union
    over wait poses. ``wait_pose_seed_id`` / ``wait_pose_q`` name that pose in
    the block. ``accepted_throws_any_seed`` is the union, carried ONLY under that
    name and next to a note saying it is not coverage.

    **A box is a superset.** The accepted set is not axis-aligned, so the box
    contains rejected throws too. That is reported rather than hidden:
    ``box_contains_throws`` counts the grid throws inside the box and
    ``box_accepted_fraction`` is how many of those were accepted. A low value
    means the launcher will be handed conditions this map rejected.

    ``aim_point_base_m`` is NOT proposed — it needs the wait pose's catch-frame
    position, which is the judge's model, not the grid's.
    """
    judged_seeds = sorted({i.result.seed_id for i in judged})
    if len(judged_seeds) > 1:
        raise ValueError(
            f"propose_throw_region was handed rows of {len(judged_seeds)} seeds {judged_seeds}: "
            "the region describes ONE wait pose — pass judged_for_seed(...)"
        )
    accepted_indices = [o.throw_index for o in outcomes if o.accepted]
    accepted = [throws[i] for i in accepted_indices]
    flight_times = [
        i.candidate.time_s
        for i in judged
        if i.result.accepted and math.isfinite(i.candidate.time_s)
    ]
    region: dict = {
        "base_frame": base_frame,
        "provisional": True,
        "throws": len(throws),
        "accepted_throws": len(accepted),
        "accepted_fraction": len(accepted) / len(throws) if throws else None,
        "accepted_fraction_denominator": DENOMINATOR_GRID_THROWS,
    }
    if wait_pose_seed_id is not None:
        region["wait_pose_seed_id"] = int(wait_pose_seed_id)
    if wait_pose_q is not None:
        region["wait_pose_q"] = [float(v) for v in wait_pose_q]
    if accepted_throws_any_seed is not None:
        region["accepted_throws_any_seed"] = int(accepted_throws_any_seed)
        region["accepted_throws_any_seed_note"] = ANY_SEED_NOTE
    if not accepted:
        region["note"] = "no throw was accepted — nothing to propose"
        return {"sim": {"throw_region": region}}

    azimuth = _range_of([t.azimuth_deg for t in accepted])
    heading = _range_of([t.aim_deviation_deg for t in accepted])
    elevation = _range_of([t.elevation_deg for t in accepted])
    speed = _range_of([t.speed_m_s for t in accepted])
    height = _range_of([t.release_height_m for t in accepted])
    distance = _range_of([t.distance_m for t in accepted])

    def inside(t: Throw) -> bool:
        return (
            azimuth[0] <= t.azimuth_deg <= azimuth[1]
            and heading[0] <= t.aim_deviation_deg <= heading[1]
            and elevation[0] <= t.elevation_deg <= elevation[1]
            and speed[0] <= t.speed_m_s <= speed[1]
            and height[0] <= t.release_height_m <= height[1]
            and distance[0] <= t.distance_m <= distance[1]
        )

    in_box = [i for i, t in enumerate(throws) if inside(t)]
    accepted_set = set(accepted_indices)
    region.update(
        {
            "distance_base_m": distance,
            "azimuth_base_rad": [math.radians(v) for v in azimuth],
            "z_world_m": height,
            "heading_offset_rad": [math.radians(v) for v in heading],
            "speed_m_s": speed,
            "elevation_rad": [math.radians(v) for v in elevation],
            "flight_time_s": _range_of(flight_times),
            "covered_axes_deg": {
                "azimuth_base_deg": azimuth,
                "heading_offset_deg": heading,
                "elevation_deg": elevation,
            },
            "box_contains_throws": len(in_box),
            "box_accepted_fraction": (
                len([i for i in in_box if i in accepted_set]) / len(in_box) if in_box else None
            ),
            "note": (
                "axis-aligned cover of the ACCEPTED throws; the accepted set is not itself "
                "axis-aligned, so box_accepted_fraction says how much of the box this map "
                "actually accepted. aim_point_base_m is not proposed here."
            ),
        }
    )
    return {"sim": {"throw_region": region}}


def render_generated_yaml(payload: Mapping, provenance: Mapping | None = None) -> str:
    """``payload`` as YAML behind the generated-file header, with provenance attached."""
    doc = dict(payload)
    if provenance is not None:
        doc["provenance"] = dict(provenance)
    return GENERATED_HEADER + yaml.safe_dump(
        doc, sort_keys=False, default_flow_style=None, width=100, allow_unicode=True
    )


# ── Plots (never called from the pure functions above) ────────────────────────


def _pyplot():
    """matplotlib with a non-interactive backend, selected BEFORE pyplot loads."""
    import matplotlib  # noqa: PLC0415

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt  # noqa: PLC0415

    return plt


def _seed_suffix(seed_id: int | None) -> str:
    return "" if seed_id is None else f" — wait-pose seed {int(seed_id)}"


def plot_azimuth_coverage(
    throws: Sequence[Throw],
    outcomes: Sequence[ThrowOutcome],
    out_path: Path,
    *,
    seed_id: int | None = None,
) -> Path:
    """ "방위별 포구 가능 구간" (plan §11): accepted fraction and count per azimuth.

    ``outcomes`` are ONE seed's (:func:`summarize_throws`); ``seed_id`` goes into
    the title so the figure says which wait pose it is about.
    """
    plt = _pyplot()
    accepted = {o.throw_index for o in outcomes if o.accepted}
    per_azimuth: dict[float, list[int]] = {}
    for index, throw in enumerate(throws):
        total, hit = per_azimuth.setdefault(throw.azimuth_deg, [0, 0])
        per_azimuth[throw.azimuth_deg] = [total + 1, hit + (1 if index in accepted else 0)]
    azimuths = sorted(per_azimuth)
    totals = [per_azimuth[a][0] for a in azimuths]
    hits = [per_azimuth[a][1] for a in azimuths]
    fractions = [h / t if t else 0.0 for h, t in zip(hits, totals, strict=True)]

    fig, (top, bottom) = plt.subplots(2, 1, figsize=(9.0, 6.5), sharex=True)
    top.bar(azimuths, fractions, width=max(1.0, _bar_width(azimuths)), color="#1f77b4")
    top.set_ylabel("accepted fraction")
    top.set_ylim(0.0, 1.05)
    top.grid(True, alpha=0.3)
    top.set_title(
        f"catchable throws by release azimuth (S3.5a kinematic map){_seed_suffix(seed_id)}"
    )
    bottom.bar(
        azimuths, hits, width=max(1.0, _bar_width(azimuths)), color="#2ca02c", label="accepted"
    )
    bottom.plot(azimuths, totals, "k.--", label="throws")
    bottom.set_xlabel("release azimuth [deg]  (world +x = 0)")
    bottom.set_ylabel("throws")
    bottom.grid(True, alpha=0.3)
    bottom.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    return Path(out_path)


def _bar_width(values: Sequence[float]) -> float:
    if len(values) < 2:
        return 5.0
    gaps = np.diff(np.asarray(sorted(values), dtype=float))
    return 0.8 * float(gaps.min())


def plot_manipulability(
    judged: Sequence[JudgedCandidate], out_path: Path, *, seed_id: int | None = None
) -> Path:
    """w₅ and w₆ over accepted candidates, in two panels — never one pooled axis.

    Pass one seed's rows (:func:`judged_for_seed`) and its ``seed_id`` for the
    title: a histogram pooled over wait poses describes no pose.
    """
    plt = _pyplot()
    accepted = [i.result for i in judged if i.result.accepted]
    w5 = [r.w5 for r in accepted if r.w5_valid]
    w6 = [r.w6 for r in accepted if r.w6_valid]
    fig, axes = plt.subplots(1, 2, figsize=(10.0, 4.0))
    for axis, values, label in (
        (axes[0], w5, "w5 = sqrt(det(J5 J5^T))"),
        (axes[1], w6, "w6 = sqrt(det(J6 J6^T))"),
    ):
        if values:
            axis.hist(values, bins=min(40, max(5, len(values) // 3)), color="#1f77b4")
        else:
            axis.text(0.5, 0.5, "no accepted candidate", ha="center", va="center")
        axis.set_xlabel(label)
        axis.set_ylabel("accepted candidates")
        axis.grid(True, alpha=0.3)
    fig.suptitle(
        "manipulability at q* (mixed units, different dimensions — not pooled)"
        + _seed_suffix(seed_id)
    )
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    return Path(out_path)


# ── CSV output ────────────────────────────────────────────────────────────────

_JOINED_HEADER = (
    "id",
    "seed_id",
    "throw_index",
    "t_c_s",
    "speed_m_s",
    "p_world_x",
    "p_world_y",
    "p_world_z",
    "v_world_x",
    "v_world_y",
    "v_world_z",
    "p_model_x",
    "p_model_y",
    "p_model_z",
    "v_model_x",
    "v_model_y",
    "v_model_z",
    "accepted",
    "reason_name",
    "iterations",
    "pos_error",
    "theta",
    "w5",
    "w6",
    "w5_valid",
    "w6_valid",
    "manip_converged",
    "manip_grad_norm",
    "manip_grad_failures",
    "sigma_min",
    "lambda_sq",
    "qp_status",
    "qp_iterations",
    "qp_failures",
    "nv",
)


def write_candidate_result_csv(path: Path, judged: Sequence[JudgedCandidate]) -> Path:
    """The per-candidate map: the throw it came from, the frames, and the verdict.

    The ``q*`` columns stay EMPTY for a poseless row, exactly as the judge wrote
    them — this file is the map's record, and turning "no pose" into zeros here
    would undo the care taken reading it.
    """
    nv = max((i.result.nv for i in judged), default=0)
    with Path(path).open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([*_JOINED_HEADER, *(f"q{i}" for i in range(nv))])
        for item in judged:
            c, r = item.candidate, item.result
            world_p = np.full(3, np.nan) if c.p_c_world_m is None else c.p_c_world_m
            world_v = np.full(3, np.nan) if c.v_world_m_s is None else c.v_world_m_s
            row = [
                r.id,
                r.seed_id,
                c.throw_index,
                repr(float(c.time_s)),
                repr(float(c.speed_m_s)),
                *(repr(float(v)) for v in np.asarray(world_p, dtype=float).reshape(3)),
                *(repr(float(v)) for v in np.asarray(world_v, dtype=float).reshape(3)),
                *(repr(float(v)) for v in np.asarray(c.p_c_model_m, dtype=float).reshape(3)),
                *(repr(float(v)) for v in np.asarray(c.v_model_m_s, dtype=float).reshape(3)),
                int(r.accepted),
                r.reason_name,
                r.iterations,
                repr(r.pos_error),
                repr(r.theta),
                repr(r.w5),
                repr(r.w6),
                int(r.w5_valid),
                int(r.w6_valid),
                int(r.manip_converged),
                repr(r.manip_grad_norm),
                r.manip_grad_failures,
                repr(r.sigma_min),
                repr(r.lambda_sq),
                r.qp_status,
                r.qp_iterations,
                r.qp_failures,
                r.nv,
            ]
            pose = ["" for _ in range(nv)]
            if r.q is not None:
                for i in range(min(nv, r.q.size)):
                    pose[i] = repr(float(r.q[i]))
            writer.writerow([*row, *pose])
    return Path(path)


def write_throw_summary_csv(
    path: Path,
    throws: Sequence[Throw],
    outcomes: Sequence[ThrowOutcome],
    *,
    seed_id: int | None = None,
) -> Path:
    """One row per GRID throw: its axes, whether it is catchable, and the best pose.

    "Per grid throw" is enforced here rather than assumed of the caller: a throw
    with no entry in ``outcomes`` — every catch instant dropped by the reach /
    flight-time pre-filters, so nothing reached the judge — is written with
    ``candidates = 0`` and ``accepted = 0``, not left out. Leaving it out makes
    the file's row count a function of the pre-filter, and "rows accepted / rows"
    a different fraction from the headline.

    ``outcomes`` are ONE wait-pose seed's; ``seed_id`` is written on every row
    as ``wait_pose_seed_id`` so the file says which pose it is about.
    """
    by_index = {o.throw_index: o for o in outcomes}
    if len(by_index) != len(outcomes):
        raise ValueError("outcomes name a throw twice — they must be ONE seed's, one per throw")
    outside = sorted(i for i in by_index if not 0 <= i < len(throws))
    if outside:
        raise ValueError(f"outcome throw_index {outside[:5]} is outside the {len(throws)} throws")
    nv = max((o.best_q.size for o in outcomes if o.best_q is not None), default=0)
    with Path(path).open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "throw_index",
                "wait_pose_seed_id",
                "distance_m",
                "azimuth_deg",
                "release_height_m",
                "aim_deviation_deg",
                "speed_m_s",
                "elevation_deg",
                "candidates",
                "accepted_candidates",
                "accepted",
                "best_id",
                "best_seed_id",
                "best_w",
                "best_w5",
                "best_w6",
                "best_theta",
                "best_t_c_s",
                "best_speed_m_s",
                "best_p_c_world_x",
                "best_p_c_world_y",
                "best_p_c_world_z",
                *(f"best_q{i}" for i in range(nv)),
            ]
        )
        for throw_index, throw in enumerate(throws):
            outcome = by_index.get(throw_index) or ThrowOutcome(
                throw_index=throw_index, candidates=0, accepted_candidates=0, accepted=False
            )
            point = (
                ["", "", ""]
                if outcome.best_p_c_world_m is None
                else [repr(float(v)) for v in np.asarray(outcome.best_p_c_world_m).reshape(3)]
            )
            pose = ["" for _ in range(nv)]
            if outcome.best_q is not None:
                for i in range(min(nv, outcome.best_q.size)):
                    pose[i] = repr(float(outcome.best_q[i]))
            writer.writerow(
                [
                    outcome.throw_index,
                    "" if seed_id is None else int(seed_id),
                    throw.distance_m,
                    throw.azimuth_deg,
                    throw.release_height_m,
                    throw.aim_deviation_deg,
                    throw.speed_m_s,
                    throw.elevation_deg,
                    outcome.candidates,
                    outcome.accepted_candidates,
                    int(outcome.accepted),
                    "" if outcome.best_id is None else outcome.best_id,
                    "" if outcome.best_seed_id is None else outcome.best_seed_id,
                    "" if outcome.best_w is None else repr(outcome.best_w),
                    "" if outcome.best_w5 is None else repr(outcome.best_w5),
                    "" if outcome.best_w6 is None else repr(outcome.best_w6),
                    "" if outcome.best_theta is None else repr(outcome.best_theta),
                    "" if outcome.best_time_s is None else repr(outcome.best_time_s),
                    "" if outcome.best_speed_m_s is None else repr(outcome.best_speed_m_s),
                    *point,
                    *pose,
                ]
            )
    return Path(path)


def write_reason_histogram_csv(path: Path, histogram: Mapping[str, int]) -> Path:
    """The rejection histogram as ``reason_name,count``, most frequent first."""
    total = sum(histogram.values())
    with Path(path).open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["reason_name", "count", "fraction_of_rejected"])
        for name, count in histogram.items():
            writer.writerow([name, count, repr(count / total) if total else ""])
    return Path(path)


# ── CLI ───────────────────────────────────────────────────────────────────────


def _axis(text: str) -> list[float]:
    return [float(v) for v in text.replace(",", " ").split()]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="catchability_map",
        description=(
            "Kinematic catchability map (dynamic_catching S3.5a): throw grid -> drag flight -> "
            "catch candidates -> catch_pose_ik_batch -> per-throw map, sim.throw_region "
            "proposal and plots."
        ),
    )
    ap.add_argument(
        "--robot-config",
        type=Path,
        nargs="+",
        required=True,
        help="shipped robot config YAML(s), later files override (e.g. _base.yaml sim.yaml)",
    )
    ap.add_argument(
        "--ball-config",
        type=Path,
        nargs="+",
        required=True,
        help="config YAML(s) carrying projectile_ball.radius_m / .mass_kg (e.g. "
        "mujoco_simulator.yaml)",
    )
    ap.add_argument("--out-dir", type=Path, required=True, help="output directory")
    ap.add_argument(
        "--arm-sub-model",
        help="urdf.sub_models key of the arm (default: the first one in the config)",
    )
    ap.add_argument("--catch-frame", default=DEFAULT_CATCH_FRAME)
    ap.add_argument(
        "--arm-base-frame",
        required=True,
        help="URDF frame this robot's CLIK config calls base_frame (its value in that "
        "robot's controllers/*.yaml). The judge works in MODEL WORLD, which is the URDF model "
        "root and NOT this frame — the two are composed, so naming it wrong is what flips "
        "downrange",
    )
    ap.add_argument("--urdf", type=Path, help="URDF/xacro override (default: urdf.package/path)")
    ap.add_argument(
        "--params",
        type=Path,
        help="YAML with the `catching:` tree for the judge. planner.ik.alpha_max, when it is "
        "set there, is also the bound the theta report is computed against",
    )
    ap.add_argument("--judge", type=Path, help=f"path to {JUDGE_EXECUTABLE} (default: ament)")

    # ── aerodynamics: no defaults, by module policy ──
    ap.add_argument("--drag-coefficient", type=float, required=True)
    ap.add_argument(
        "--drag-coefficient-source",
        required=True,
        help="file:line the Cd came from (it is a constexpr C++ preset, not YAML)",
    )
    ap.add_argument("--air-density", type=float, required=True, dest="air_density_kg_m3")
    ap.add_argument(
        "--air-density-source", required=True, help="file:line the air density came from"
    )

    # ── the world -> arm base transform: the caller owns it ──
    ap.add_argument(
        "--world-yaw-deg",
        type=float,
        default=0.0,
        help="yaw of base_T_world about z [deg] (p_base = Rz(yaw) p_world + t). The CALLER owns "
        "this: measure it with MuJoCo FK against Pinocchio FK at the same q. Default 0 is what "
        "plan §11 measured for both shipped robots; whatever is used is recorded in the "
        "provenance",
    )
    ap.add_argument(
        "--world-translation-m",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.0),
        metavar=("X", "Y", "Z"),
        help="translation t of base_T_world [m] (see --world-yaw-deg)",
    )

    # ── grid axes ──
    ap.add_argument("--distances-m", type=_axis, default=list(DEFAULT_DISTANCES_M))
    ap.add_argument("--azimuths-deg", type=_axis, default=list(DEFAULT_AZIMUTHS_DEG))
    ap.add_argument("--release-heights-m", type=_axis, default=list(DEFAULT_RELEASE_HEIGHTS_M))
    ap.add_argument("--aim-deviations-deg", type=_axis, default=list(DEFAULT_AIM_DEVIATIONS_DEG))
    ap.add_argument("--speeds-m-s", type=_axis, default=list(DEFAULT_SPEEDS_M_S))
    ap.add_argument("--elevations-deg", type=_axis, default=list(DEFAULT_ELEVATIONS_DEG))

    # ── flight and candidate sampling ──
    ap.add_argument("--horizon-s", type=float, default=2.5)
    ap.add_argument("--step-s", type=float, default=0.002)
    ap.add_argument(
        "--window-s",
        type=float,
        nargs=2,
        default=(1.0, 2.0),
        metavar=("LO", "HI"),
        help="candidate time window [s] after release",
    )
    ap.add_argument("--stride-s", type=float, default=0.05)
    ap.add_argument(
        "--min-flight-time-s",
        type=float,
        default=DEFAULT_MIN_FLIGHT_TIME_S,
        help=f"D-18 flight-time floor [s] ({MIN_FLIGHT_TIME_SOURCE})",
    )
    ap.add_argument(
        "--max-reach-m",
        type=float,
        required=True,
        help="reachability PRE-filter: drop candidates farther than this from the arm base "
        "origin. No default — a wrong value silently prunes the map",
    )
    ap.add_argument("--min-catch-height-m", type=float, default=0.0)

    # ── seeds, sharding, extras ──
    ap.add_argument(
        "--seed",
        type=_axis,
        action="append",
        default=None,
        help="a wait-pose seed q (whitespace/comma separated, nv entries). Repeat for the "
        "seed comparison. Default: one all-zero seed",
    )
    ap.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    ap.add_argument("--shard-size", type=int, default=200)
    ap.add_argument("--manip-column", choices=["w5", "w6"], default="w5")
    ap.add_argument(
        "--alpha-max-rad",
        type=float,
        default=None,
        help=f"{ALPHA_MAX_KEY} for the theta report, ONLY for when --params does not set it "
        "(then the judge ran on its in-code default and this states that default). Differing "
        "from a value --params does set is an error. Default: the params value, else the "
        f"mirror of the judge's in-code default {JUDGE_DEFAULT_ALPHA_MAX_RAD}",
    )
    ap.add_argument("--theta-near-fraction", type=float, default=0.9)
    ap.add_argument(
        "--epsilon-m",
        type=float,
        help="also re-judge +/-epsilon copies of the accepted candidates and count how many "
        "sit on the accept/reject boundary",
    )
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args(argv)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    workers = max(1, min(int(args.workers), MAX_WORKERS))

    # Before anything is judged: a params file whose tree cannot be located, or
    # a flag that contradicts it, should not cost a sweep to find out.
    try:
        alpha_max = resolve_alpha_max(args.params, args.alpha_max_rad)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        raise SystemExit(f"alpha_max: {exc}") from exc

    artifacts = write_model_config(
        args.robot_config,
        out_dir,
        arm_sub_model=args.arm_sub_model,
        catch_frame=args.catch_frame,
        urdf_override=args.urdf,
    )
    judge = find_judge(args.judge)
    # ... and against the judge's own account of what it will apply, when this
    # binary can give one. Still before the sweep.
    try:
        alpha_max = confirm_alpha_max_with_judge(
            alpha_max, judge_reported_alpha_max(judge, args.params)
        )
    except ValueError as exc:
        raise SystemExit(f"alpha_max: {exc}") from exc
    if alpha_max.warning:
        print(f"WARNING: {alpha_max.warning}", file=sys.stderr)
    seeds_path = out_dir / "seeds.csv"
    probe = dump_catch_frame(
        JudgeInvocation(
            judge=judge,
            model_config=artifacts.model_config_path,
            seeds=seeds_path,
            sub_model=artifacts.catch_sub_model,
            catch_frame=artifacts.catch_frame,
        )
    )
    nv = int(probe["nv"])
    seeds = args.seed if args.seed else [[0.0] * nv]
    for i, seed in enumerate(seeds):
        if len(seed) != nv:
            raise SystemExit(f"--seed #{i} has {len(seed)} entries, the sub-model nv is {nv}")
    write_seed_csv(seeds_path, seeds)
    invocation = JudgeInvocation(
        judge=judge,
        model_config=artifacts.model_config_path,
        seeds=seeds_path,
        sub_model=artifacts.catch_sub_model,
        catch_frame=artifacts.catch_frame,
        params=args.params,
    )

    shape = ball_shape_from_config(args.ball_config)
    ball = ball_params_from_shape(
        shape,
        drag_coefficient=args.drag_coefficient,
        drag_coefficient_source=args.drag_coefficient_source,
        air_density_kg_m3=args.air_density_kg_m3,
        air_density_source=args.air_density_source,
    )

    base_t_world = make_transform(
        rotation_z(math.radians(args.world_yaw_deg)), args.world_translation_m
    )
    world_t_base = invert_transform(base_t_world)
    base_origin_w = world_t_base[:3, 3]
    model_world_t_base = frame_placement_in_model_world(artifacts.urdf_text, args.arm_base_frame)
    model_world_t_world = as_transform(model_world_t_base @ base_t_world)

    throws = generate_throw_grid(
        base_xy_m=base_origin_w[:2],
        distances_m=args.distances_m,
        azimuths_deg=args.azimuths_deg,
        release_heights_m=args.release_heights_m,
        aim_deviations_deg=args.aim_deviations_deg,
        speeds_m_s=args.speeds_m_s,
        elevations_deg=args.elevations_deg,
    )
    reach = max_distance_filter(base_origin_w, args.max_reach_m)
    floor = float(args.min_catch_height_m)

    def keep(position_m: np.ndarray, velocity_m_s: np.ndarray) -> bool:
        return bool(position_m[2] >= floor) and reach(position_m, velocity_m_s)

    sampled: list[CatchCandidate] = []
    for index, throw in enumerate(throws):
        trajectory = integrate_flight(
            throw.position_m,
            throw.velocity_m_s,
            ball,
            horizon_s=args.horizon_s,
            step_s=args.step_s,
        )
        sampled += sample_catch_candidates(
            trajectory,
            throw_index=index,
            window_s=tuple(args.window_s),
            stride_s=args.stride_s,
            min_flight_time_s=args.min_flight_time_s,
            reach_filter=keep,
        )
    candidates = to_judge_candidates(
        sampled, model_world_t_world=model_world_t_world, seed_ids=range(len(seeds))
    )
    print(
        f"{len(throws)} throws -> {len(sampled)} catch instants -> {len(candidates)} candidates "
        f"({len(seeds)} seed(s)), {workers} worker(s)",
        file=sys.stderr,
    )
    results = run_judge_batch(
        invocation,
        candidates,
        work_dir=out_dir / "shards",
        shard_size=args.shard_size,
        workers=workers,
    )
    judged = join_results(candidates, results)

    # ONE wait pose. Every headline figure below — accepted throws, the region
    # proposal, the per-throw CSV, the distributions, the plots — describes the
    # best seed of the ranking, because a robot waits in one pose. The union over
    # seeds survives only as `accepted_throws_any_seed`.
    seed_comparison = rank_wait_pose_seeds(judged, throw_count=len(throws))
    headline_seed_id = None if seed_comparison.best is None else seed_comparison.best.seed_id
    headline = [] if headline_seed_id is None else judged_for_seed(judged, headline_seed_id)
    accepted_throws_any_seed = count_throws_accepted_by_any_seed(judged)

    outcomes = summarize_throws(
        headline, manip_column=args.manip_column, seed_id=headline_seed_id, throw_count=len(throws)
    )
    histogram = reason_histogram(headline)
    manip = manipulability_distributions(headline)
    theta = theta_report(
        headline, alpha_max=alpha_max.value_rad, near_fraction=args.theta_near_fraction
    )

    boundary: dict | None = None
    if args.epsilon_m is not None:
        accepted_candidates = [i.candidate for i in headline if i.result.accepted]
        if accepted_candidates:
            perturbed, sources = perturb_candidates(
                accepted_candidates,
                epsilon_m=args.epsilon_m,
                id_offset=max(c.id for c in candidates) + 1,
            )
            perturbed_results = run_judge_batch(
                invocation,
                perturbed,
                work_dir=out_dir / "shards_eps",
                shard_size=args.shard_size,
                workers=workers,
                prefix="eps",
            )
            boundary = count_boundary_candidates(
                [i.result for i in headline],
                perturbed_results,
                sources,
                epsilon_m=args.epsilon_m,
            )
        else:
            boundary = {"epsilon_m": float(args.epsilon_m), "accepted": 0, "on_boundary": 0}

    write_candidate_result_csv(out_dir / "candidates.csv", judged)
    write_throw_summary_csv(
        out_dir / "throw_summary.csv", throws, outcomes, seed_id=headline_seed_id
    )
    write_reason_histogram_csv(out_dir / "reason_histogram.csv", histogram)

    provenance = build_provenance(
        ball,
        configs=list(args.ball_config),
        grid=throws,
        integration={"step_s": args.step_s, "horizon_s": args.horizon_s},
        extra={
            "model": artifacts.provenance,
            "judge": {
                "executable": str(judge),
                "sub_model": artifacts.catch_sub_model,
                "catch_frame": artifacts.catch_frame,
                "executable_identity": {
                    "size_bytes": judge.stat().st_size,
                    "mtime_ns": judge.stat().st_mtime_ns,
                },
                "params": None if args.params is None else str(args.params),
                "params_sha256": (
                    None if args.params is None else _sha256(Path(args.params).read_bytes())
                ),
                "resume": (
                    "a shard is reused only when its fingerprint sidecar matches: candidate "
                    "CSV, model config, the URDF / closure YAML it names, seeds and params by "
                    "sha256; sub-model and catch-frame by name; the judge by path+size+mtime "
                    f"({SHARD_FINGERPRINT_SCHEMA})"
                ),
                "dump_frame": {k: v for k, v in probe.items() if k != "stdout"},
                "omp_num_threads": "1",
                "workers": workers,
                "shard_size": args.shard_size,
            },
            "frames": {
                "base_t_world": base_t_world.tolist(),
                "world_yaw_deg": args.world_yaw_deg,
                "world_translation_m": list(args.world_translation_m),
                "world_transform_owner": (
                    "the caller — measured MuJoCo FK vs Pinocchio FK at the same q (plan §11)"
                ),
                "arm_base_frame": args.arm_base_frame,
                "model_world_t_base": model_world_t_base.tolist(),
                "model_world_t_world": model_world_t_world.tolist(),
                "note": (
                    "the judge takes MODEL WORLD coordinates (the URDF model root), which is "
                    "not the arm base frame; model_world_t_world = model_world_t_base @ "
                    "base_t_world"
                ),
            },
            "candidates": {
                "window_s": list(args.window_s),
                "stride_s": args.stride_s,
                "min_flight_time_s": args.min_flight_time_s,
                "min_flight_time_source": MIN_FLIGHT_TIME_SOURCE,
                "max_reach_m": args.max_reach_m,
                "min_catch_height_m": args.min_catch_height_m,
                "catch_instants": len(sampled),
                "judged": len(candidates),
            },
            "seeds": [list(map(float, s)) for s in seeds],
            "results": {
                "scope": (
                    "ONE wait pose: every figure in this block except the *_any_seed / "
                    "*_all_seeds ones and seed_ranking is over the headline seed only"
                ),
                "headline_seed": {
                    "seed_id": headline_seed_id,
                    "q": None
                    if headline_seed_id is None
                    else [float(v) for v in seeds[headline_seed_id]],
                    "selection": (
                        "rank_wait_pose_seeds best: accepted-throw fraction over the full "
                        "grid, ties by mean log w5, then seed_id"
                    ),
                },
                "accepted_candidates": sum(1 for i in headline if i.result.accepted),
                "judged_candidates": len(headline),
                "accepted_throws": sum(1 for o in outcomes if o.accepted),
                "throws": len(throws),
                "accepted_fraction": (
                    sum(1 for o in outcomes if o.accepted) / len(throws) if throws else None
                ),
                "accepted_fraction_denominator": DENOMINATOR_GRID_THROWS,
                "throws_with_candidates": seed_comparison.throws_with_candidates,
                "accepted_throws_any_seed": accepted_throws_any_seed,
                "accepted_throws_any_seed_note": ANY_SEED_NOTE,
                "accepted_candidates_all_seeds": sum(1 for i in judged if i.result.accepted),
                "judged_candidates_all_seeds": len(judged),
                "manip_column": args.manip_column,
                "reason_histogram": histogram,
                "manipulability": manip,
                "theta": theta,
                "alpha_max": alpha_max.as_provenance(),
                "seed_ranking": [vars(r) for r in seed_comparison.ranking],
                "seed_ranking_denominator": seed_comparison.denominator,
                "seed_coverage_gap": seed_comparison.coverage_gap,
                "boundary": boundary,
            },
        },
    )
    (out_dir / "throw_region.yaml").write_text(
        render_generated_yaml(
            propose_throw_region(
                throws,
                outcomes,
                headline,
                base_frame=args.arm_base_frame,
                wait_pose_seed_id=headline_seed_id,
                wait_pose_q=None
                if headline_seed_id is None
                else [float(v) for v in seeds[headline_seed_id]],
                accepted_throws_any_seed=accepted_throws_any_seed,
            ),
            provenance,
        )
    )
    (out_dir / "provenance.yaml").write_text(render_generated_yaml({"provenance": provenance}))

    if not args.no_plots:
        plot_azimuth_coverage(
            throws, outcomes, out_dir / "azimuth_coverage.png", seed_id=headline_seed_id
        )
        plot_manipulability(headline, out_dir / "manipulability.png", seed_id=headline_seed_id)

    accepted_throws = sum(1 for o in outcomes if o.accepted)
    print(
        f"wait-pose seed {headline_seed_id}: accepted {accepted_throws}/{len(throws)} grid "
        f"throws, {sum(1 for i in headline if i.result.accepted)}/{len(headline)} candidates",
        file=sys.stderr,
    )
    if len(seeds) > 1:
        print(
            f"accepted_throws_any_seed = {accepted_throws_any_seed}/{len(throws)} — union over "
            f"{len(seeds)} seeds, NOT coverage (no single wait pose achieves it)",
            file=sys.stderr,
        )
    confirmed = (
        "confirmed by the judge"
        if alpha_max.judge_reported_rad is not None
        else "not cross-checked"
    )
    print(
        f"alpha_max = {alpha_max.value_rad} rad [{alpha_max.origin}; {confirmed}]",
        file=sys.stderr,
    )
    print(f"reason histogram: {histogram}", file=sys.stderr)
    print(f"wrote {out_dir}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
