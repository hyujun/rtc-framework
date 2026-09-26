"""Throw trials at the catching controller in sim, one full S7 cycle each.

From S7.2 the controller homes itself: armed, it moves the arm to
``planner.wait_pose`` in joint space and waits there with the hand at q_pre
(IDLE → ARMED), and after every trial it returns there (RETREAT → ARMED). The
external alignment this driver used to do (#537 decision ④, "until S7.2" —
switch to ``demo_joint_controller``, send the wait pose, switch back) is gone:
a trial that starts elsewhere is now the controller's bug to show, not the
driver's to hide.

Per trial:

1. A latched FAULT is reset; the controller is armed (``catching.enable``) if
   it is not; wait for ARMED — the controller's own homing — and record how far
   the measured arm is from the wait pose at that moment.
2. Launch the ball at a stated release state (``/sim/launch_ball_at`` — a stated
   launch does not consume the sim's RNG, so the series replays exactly) and
   record ground truth and every mode transition.
3. The trial ends when the controller is back in ARMED after a RETREAT (the
   cycle closed), or in IDLE / FAULT, or after ``record_s``. The attempt's
   verdict is the ``outcome`` published on the RETREAT entry (L7 §4.7).
4. Reset the ball.

This drives an already-running sim. It launches nothing: the sim, the ball
estimator and the switch into the catching controller are the caller's
(``integrated_bringup/README.md`` §Catching sim trials has the commands).

Robot constants come from the shipped profile, not from this file: the arm
device is the first device in the catching controller's ``topics``, its joint
names and state topic are that device's roster. The wait pose, the commit lead
and the lead axis are what the RUNNING controller loaded — its read-only mirror
parameters (``planner.wait_pose``, ``planner.freeze.T_freeze``,
``joint_cmd.lag.{T_arm,lead_enable}``, ``control.dt``) — because a sim overlay
changes them without changing the installed YAML (plan §4.4 S8-A). They are
written to ``run_meta.json`` and to every trial record.

Two throw series (``--dist``):

* ``reference`` (default, unchanged since S6-C): ``--n-ref`` S3.5b reference
  throws then ``--n-pert`` seeded perturbations. A regression set — repeated
  throws are not an iid sample, so not a success-rate input (D-S8-2).
* ``s35b``: ``--n`` iid throws drawn with ``--seed`` from the profile's frozen
  gate-map box (D-S8-2; ur5e_p1b's is the S3.5b 90 % box, iiwa7_leap's the
  S8-D re-run's), built by ``rtc_tools.analysis.catchability_map`` so the
  geometry is the one the gate map judged.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import math
import os
import random
import time
from typing import Any

import yaml

CATCHING = "demo_catching_controller"
JOINT = "demo_joint_controller"

MODE_NAMES = (
    "IDLE",
    "ARMED",
    "TRACKING",
    "APPROACH",
    "COMMITTED",
    "CLOSING",
    "DECEL",
    "HOLD",
    "RETREAT",
    "ABORT_SAFE",
    "FAULT",
)
OUTCOME_NAMES = ("NONE", "CAPTURED", "MISSED", "UNDETERMINED", "ABORTED")

# The S3.5b reference throw for ur5e_p1b (1.0 m out, 0.2 m up, 4.75 m/s at 60°).
REFERENCE_RELEASE_POS = (1.0, 0.0, 0.2)
REFERENCE_RELEASE_VEL = (-2.375, 0.0, 4.11362)

# The controller's read-only mirror of what it loaded (lifecycle.cpp
# DeclareProfileParameters). All must exist: a controller that parked at
# configure declares none of them, and a run against it is not a trial.
MIRROR_PARAMETERS = (
    "planner.wait_pose",
    "planner.freeze.T_freeze",
    "joint_cmd.lag.T_arm",
    "joint_cmd.lag.lead_enable",
    "control.dt",
)


@dataclasses.dataclass(frozen=True)
class ThrowBox:
    """A frozen throw distribution: uniform on each axis of the gate-map grid.

    The axes are :func:`rtc_tools.analysis.catchability_map.generate_throw_grid`'s,
    in the same frames (sim world; the base axis at ``base_xy_m``).
    """

    distance_m: tuple[float, float]
    release_height_m: tuple[float, float]
    aim_deviation_deg: tuple[float, float]
    speed_m_s: tuple[float, float]
    elevation_deg: tuple[float, float]
    azimuth_deg: float = 0.0
    base_xy_m: tuple[float, float] = (0.0, 0.0)


# D-S8-2 (a): the frozen gate-map box of each profile. Per profile, because the
# box is the gate map's verdict for that robot and wait pose.
#   ur5e_p1b   — the S3.5b box that opened ≥ 90 % of its throws (plan §4.4
#                S3.5b result — 163/180 on the torque layer).
#   iiwa7_leap — the S8-D map re-run (D-S8-14/15, plan §4.4 S8-D): of the
#                boxes of ur5e_p1b's width, the one opening the most throws
#                that ALSO rise clear of the robot parked at the wait pose —
#                164/180 at a first plan of 0.215 s (115/180 at 0.24 s),
#                wait pose = the shipped `planner.wait_pose`. The clearance
#                condition is not in the gate map: steeper lobs (86-88°, 180/180
#                on the map) rise straight into the waiting hand (S8-D smoke).
FROZEN_DISTRIBUTIONS: dict[str, dict[str, ThrowBox]] = {
    "s35b": {
        "ur5e_p1b": ThrowBox(
            distance_m=(0.9, 1.0),
            release_height_m=(0.15, 0.25),
            aim_deviation_deg=(-6.0, 6.0),
            speed_m_s=(4.65, 4.85),
            elevation_deg=(62.0, 64.0),
        ),
        "iiwa7_leap": ThrowBox(
            distance_m=(0.95, 1.05),
            release_height_m=(0.10, 0.20),
            aim_deviation_deg=(-6.0, 6.0),
            speed_m_s=(2.85, 3.05),
            elevation_deg=(78.0, 80.0),
        ),
    },
}


@dataclasses.dataclass(frozen=True)
class ArmProfile:
    """What the driver needs to know about the arm, read from a shipped profile."""

    device: str
    joint_names: tuple[str, ...]
    state_topic: str
    wait_pose: tuple[float, ...]

    @property
    def goal_topic(self) -> str:
        return f"/{JOINT}/{self.device}/joint_goal"


def _device_entries(node: Any, device: str):
    """Every ``devices.<device>`` mapping anywhere under ``node``."""
    if isinstance(node, dict):
        devices = node.get("devices")
        if isinstance(devices, dict) and isinstance(devices.get(device), dict):
            yield devices[device]
        for value in node.values():
            yield from _device_entries(value, device)


def _find_key(node: Any, key: str):
    if isinstance(node, dict):
        if key in node:
            return node[key]
        for value in node.values():
            found = _find_key(value, key)
            if found is not None:
                return found
    return None


def load_arm_profile(config_dir: str) -> ArmProfile:
    """Resolve the arm from ``<config_dir>`` (one robot profile's config tree).

    Raises ``ValueError`` when something is missing or inconsistent: a driver
    that guessed would home the arm to the wrong pose and every trial would
    still look like a trial.
    """
    with open(os.path.join(config_dir, "controllers", f"{CATCHING}.yaml")) as f:
        ctrl = yaml.safe_load(f)[CATCHING]
    topics = ctrl.get("topics") or {}
    if not topics:
        raise ValueError(f"{CATCHING}.yaml has no topics — cannot tell which device is the arm")
    device = next(iter(topics))
    wait_pose = ctrl["catching"]["planner"].get("wait_pose")
    if not wait_pose:
        raise ValueError(f"{CATCHING}.yaml has no catching.planner.wait_pose")

    joint_names = None
    state_topic = None
    for name in ("_base.yaml", "sim.yaml"):
        path = os.path.join(config_dir, name)
        if not os.path.isfile(path):
            continue
        with open(path) as f:
            doc = yaml.safe_load(f)
        for entry in _device_entries(doc, device):
            joint_names = joint_names or entry.get("joint_state_names")
            state_topic = state_topic or _find_key(entry, "state_topic")
    if not joint_names:
        raise ValueError(f"no devices.{device}.joint_state_names under {config_dir}")
    if not state_topic:
        raise ValueError(f"no state_topic for device {device} under {config_dir}")
    if len(wait_pose) != len(joint_names):
        raise ValueError(
            f"planner.wait_pose has {len(wait_pose)} values but device {device} "
            f"has {len(joint_names)} joints"
        )
    return ArmProfile(
        device=device,
        joint_names=tuple(joint_names),
        state_topic=state_topic,
        wait_pose=tuple(float(v) for v in wait_pose),
    )


def alignment_error(q, qd, target) -> tuple[float, float]:
    """``(max|q - target|, max|qd|)`` — the two numbers the alignment gate reads."""
    return (
        max(abs(a - b) for a, b in zip(q, target, strict=True)),
        max(abs(v) for v in qd),
    )


def trial_throws(n_ref: int, n_pert: int, seed: int, pos, vel):
    """The throw series: ``n_ref`` reference throws, then ``n_pert`` perturbed ones.

    Perturbation (unchanged since the first S6-C run, so series stay
    comparable): speed × U(0.9, 1.1), plus U(-0.3, 0.3) m/s lateral.
    """
    rng = random.Random(seed)
    throws = [{"kind": "reference", "pos": tuple(pos), "vel": tuple(vel)} for _ in range(n_ref)]
    for _ in range(n_pert):
        scale = rng.uniform(0.9, 1.1)
        lateral = rng.uniform(-0.3, 0.3)
        vx, vy, vz = vel
        throws.append(
            {
                "kind": "varied",
                "pos": tuple(pos),
                "vel": (vx * scale, vy * scale + lateral, vz * scale),
                "mag_scale": scale,
                "lateral_y": lateral,
            }
        )
    return throws


def frozen_throws(dist: str, profile: str, n: int, seed: int) -> list[dict]:
    """``n`` iid throws from the profile's frozen distribution ``dist``.

    Deterministic in ``seed`` (``random.Random``, one draw per axis per throw in
    a fixed order), so a series is replayed from ``(dist, profile, n, seed)``
    alone. Each throw carries its axis values, its spin (zero — the flight model
    the gate map judged with has no Magnus term) and its provenance.
    """
    boxes = FROZEN_DISTRIBUTIONS.get(dist)
    if boxes is None:
        raise ValueError(f"unknown distribution {dist!r} (known: {sorted(FROZEN_DISTRIBUTIONS)})")
    box = boxes.get(profile)
    if box is None:
        raise ValueError(
            f"distribution {dist!r} has no box for profile {profile!r} "
            f"(frozen for: {sorted(boxes)})"
        )
    if n < 0:
        raise ValueError(f"n must be >= 0, got {n}")
    # Imported here: the helpers above stay importable where rtc_tools is not.
    from rtc_tools.analysis.catchability_map import generate_throw_grid, throw_to_launch_request

    rng = random.Random(seed)
    throws = []
    for i in range(n):
        axes = {
            "distance_m": rng.uniform(*box.distance_m),
            "release_height_m": rng.uniform(*box.release_height_m),
            "aim_deviation_deg": rng.uniform(*box.aim_deviation_deg),
            "speed_m_s": rng.uniform(*box.speed_m_s),
            "elevation_deg": rng.uniform(*box.elevation_deg),
        }
        (throw,) = generate_throw_grid(
            base_xy_m=box.base_xy_m,
            distances_m=(axes["distance_m"],),
            azimuths_deg=(box.azimuth_deg,),
            release_heights_m=(axes["release_height_m"],),
            aim_deviations_deg=(axes["aim_deviation_deg"],),
            speeds_m_s=(axes["speed_m_s"],),
            elevations_deg=(axes["elevation_deg"],),
        )
        req = throw_to_launch_request(throw)
        throws.append(
            {
                "kind": dist,
                "pos": tuple(req["position"][k] for k in "xyz"),
                "vel": tuple(req["velocity"][k] for k in "xyz"),
                "omega": tuple(req["angular_velocity"][k] for k in "xyz"),
                "seed": seed,
                "sample_idx": i,
                "azimuth_deg": box.azimuth_deg,
                **axes,
            }
        )
    return throws


# ── Hand-near throws (dynamic_catching S8-F, #537) ────────────────────────────
#
# A throw is specified by its ARRIVAL beside the waiting hand — speed v, flight
# time T, offset (r, ψ) in the palm plane, incidence α off the approach axis —
# and `catchability_map.aim_at_hand` integrates the release backwards from it.
# The hand's catch point and approach axis come from FK of the wait pose the
# controller LOADED (the mirror), so an overlay that moves the wait pose moves
# the throws with it. Designs are frozen here for the same reason the s35b box
# is: a series is replayed from (dist, n, seed) alone.


@dataclasses.dataclass(frozen=True)
class HandGrid:
    """A speed sweep at one (T, r, ψ, α): ``repeats`` throws per speed, speeds interleaved.

    Interleaved (all speeds once, then again, …) so a unit cut short still
    covers every speed. ``incidence_deg`` (below the horizontal) overrides
    ``incidence_offset_deg`` when set — a lob is steep in the world, whatever
    the palm's elevation.
    """

    speeds_m_s: tuple[float, ...]
    repeats: int
    flight_time_s: float
    offset_m: float = 0.0
    offset_angle_deg: float = 0.0
    incidence_offset_deg: float = 0.0
    incidence_deg: float | None = None

    @property
    def n(self) -> int:
        return len(self.speeds_m_s) * self.repeats


@dataclasses.dataclass(frozen=True)
class HandBox:
    """Latin-hypercube box over (v, T, r, ψ, α); draws the floor refuses are redrawn."""

    speed_m_s: tuple[float, float]
    flight_time_s: tuple[float, float]
    offset_m: tuple[float, float]
    offset_angle_deg: tuple[float, float]
    incidence_offset_deg: tuple[float, float]


# #537 S8-F-1 (2026-09-26): the cliff at the catch point, face-on, at the
# shortest flight the commit window allows (first plan 0.20 + T_freeze 0.37 +
# margin); the lob arm for the slow balls a face-on throw cannot deliver above
# the work table; the LHS box the v50(r) map is fitted on. Speeds below 3.5 m/s
# face-on put the release under the table (the re-derivation's §2b).
HAND_DESIGNS: dict[str, HandGrid | HandBox] = {
    "hand_cliff": HandGrid(
        speeds_m_s=(3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 7.0), repeats=8, flight_time_s=0.65
    ),
    "hand_lob": HandGrid(
        speeds_m_s=(2.5, 3.0, 3.5, 4.0), repeats=14, flight_time_s=0.65, incidence_deg=85.0
    ),
    "hand_lhs": HandBox(
        speed_m_s=(3.5, 7.0),
        flight_time_s=(0.65, 0.8),
        offset_m=(0.0, 0.2),
        offset_angle_deg=(0.0, 360.0),
        incidence_offset_deg=(-10.0, 15.0),
    ),
}

HAND_BOX_AXES = (
    "speed_m_s",
    "flight_time_s",
    "offset_m",
    "offset_angle_deg",
    "incidence_offset_deg",
)


@dataclasses.dataclass(frozen=True)
class HandGeometry:
    """Where the hand is and what the ball is, in the sim world, plus where each came from."""

    p_c_m: tuple[float, float, float]
    approach_axis: tuple[float, float, float]
    floor_z_m: float
    params: object  # rtc_tools.analysis.catchability_map.BallParams
    sources: dict

    @property
    def axis_elevation_deg(self) -> float:
        x, y, z = self.approach_axis
        return math.degrees(math.atan2(z, math.hypot(x, y)))

    def as_record(self) -> dict:
        p = self.params
        return {
            "p_c_m": list(self.p_c_m),
            "approach_axis": list(self.approach_axis),
            "axis_elevation_deg": self.axis_elevation_deg,
            "floor_z_m": self.floor_z_m,
            "ball": {
                "radius_m": p.radius_m,
                "mass_kg": p.mass_kg,
                "drag_coefficient": p.drag_coefficient,
                "air_density_kg_m3": p.air_density_kg_m3,
                "sources": dict(p.sources),
            },
            "sources": dict(self.sources),
        }


def hand_geometry(
    config_dir: str,
    profile: ArmProfile,
    *,
    floor_z_m: float,
    drag_coefficient: float,
    drag_coefficient_source: str,
    air_density_kg_m3: float,
    air_density_source: str,
    urdf: str | None = None,
) -> HandGeometry:
    """FK of ``profile.wait_pose`` → the catch point and approach axis in the sim world.

    Composed the way the catching controller composes its frames (the
    catching_trials analyser's ``CatchFrameFk``: URDF ``model_world_T_base`` ×
    ``catching.io.base_T_world``), so the throws land where the controller
    thinks the hand is. Ball shape comes from the profile's simulator YAML; the
    drag preset is C++-only and must be handed in with its file:line.
    """
    from pathlib import Path

    from rtc_tools.analysis.catchability_map import ball_params_from_shape, ball_shape_from_config
    from rtc_tools.analysis.catching_trials import CatchFrameFk, load_profile
    from rtc_tools.analysis.derive_accel_limits import resolve_urdf_text

    catching = load_profile(Path(config_dir))
    urdf_text, urdf_source = resolve_urdf_text(catching.robot_params, Path(urdf) if urdf else None)
    fk = CatchFrameFk(urdf_text, profile.joint_names, catching)
    p_c, rotation = fk.pose_world(list(profile.wait_pose))
    shape = ball_shape_from_config([Path(config_dir) / "mujoco_simulator.yaml"])
    params = ball_params_from_shape(
        shape,
        drag_coefficient=drag_coefficient,
        drag_coefficient_source=drag_coefficient_source,
        air_density_kg_m3=air_density_kg_m3,
        air_density_source=air_density_source,
    )
    return HandGeometry(
        p_c_m=tuple(float(v) for v in p_c),
        approach_axis=tuple(float(v) for v in rotation[:, 2]),
        floor_z_m=float(floor_z_m),
        params=params,
        sources={
            "urdf": urdf_source,
            "catch_frame": catching.catch_frame_name,
            "arm_base_frame": catching.arm_base_frame,
            "wait_pose": "controller mirror planner.wait_pose",
        },
    )


def _hand_record(dist: str, seed: int, idx: int, throw, draws: int) -> dict:
    return {
        "kind": dist,
        "pos": tuple(float(v) for v in throw.position_m),
        "vel": tuple(float(v) for v in throw.velocity_m_s),
        "omega": (0.0, 0.0, 0.0),
        "seed": seed,
        "sample_idx": idx,
        "draws": draws,
        "speed_m_s": throw.speed_m_s,
        "flight_time_s": throw.flight_time_s,
        "offset_m": throw.offset_m,
        "offset_angle_deg": throw.offset_angle_deg,
        "incidence_offset_deg": throw.incidence_offset_deg,
        "target_m": tuple(float(v) for v in throw.target_m),
        "incidence_deg": throw.incidence_deg,
        "release_height_offset_m": throw.release_height_offset_m,
        "horizontal_distance_m": throw.horizontal_distance_m,
        "release_speed_m_s": throw.release_speed_m_s,
        "release_elevation_deg": throw.release_elevation_deg,
        "apex_z_m": throw.apex_z_m,
    }


def hand_near_throws(dist: str, n: int, seed: int, geometry: HandGeometry) -> list[dict]:
    """The hand-near series ``dist`` (:data:`HAND_DESIGNS`) aimed at ``geometry``.

    A grid ignores ``n`` (its size is the design's) and needs no random draw; a
    box draws ``n`` accepted Latin-hypercube samples in ``seed`` order,
    redrawing (a fresh hypercube of ``n``) whenever the floor refuses one, so
    the accepted set is a function of ``(dist, n, seed, geometry)`` alone.
    """
    design = HAND_DESIGNS.get(dist)
    if design is None:
        raise ValueError(f"unknown hand-near design {dist!r} (known: {sorted(HAND_DESIGNS)})")
    from rtc_tools.analysis.catchability_map import aim_at_hand

    def aim(**factors):
        return aim_at_hand(
            geometry.p_c_m,
            geometry.approach_axis,
            params=geometry.params,
            floor_z_m=geometry.floor_z_m,
            **factors,
        )

    throws: list[dict] = []
    if isinstance(design, HandGrid):
        alpha = design.incidence_offset_deg
        if design.incidence_deg is not None:
            alpha = design.incidence_deg - geometry.axis_elevation_deg
        for rep in range(design.repeats):
            for speed in design.speeds_m_s:
                throw = aim(
                    speed_m_s=speed,
                    flight_time_s=design.flight_time_s,
                    offset_m=design.offset_m,
                    offset_angle_deg=design.offset_angle_deg,
                    incidence_offset_deg=alpha,
                )
                throws.append(_hand_record(dist, seed, len(throws), throw, draws=1))
                throws[-1]["repeat"] = rep
        return throws

    if n < 0:
        raise ValueError(f"n must be >= 0, got {n}")
    rng = random.Random(seed)
    draws = 0
    while len(throws) < n:
        # One Latin hypercube of n points: each axis split into n strata, one
        # point per stratum, strata paired by independent permutations.
        columns = {}
        for axis in HAND_BOX_AXES:
            lo, hi = getattr(design, axis)
            strata = list(range(max(n, 1)))
            rng.shuffle(strata)
            columns[axis] = [lo + (hi - lo) * (s + rng.random()) / max(n, 1) for s in strata]
        for i in range(n):
            if len(throws) >= n:
                break
            draws += 1
            factors = {axis: columns[axis][i] for axis in HAND_BOX_AXES}
            try:
                throw = aim(**factors)
            except ValueError:
                continue  # the floor (or the vertical) refused it: redraw
            throws.append(_hand_record(dist, seed, len(throws), throw, draws=draws))
    return throws


def build_throws(args, profile: str, geometry: HandGeometry | None = None) -> list[dict]:
    """The series ``--dist`` selects (module docstring)."""
    if args.dist == "reference":
        return trial_throws(args.n_ref, args.n_pert, args.seed, args.release_pos, args.release_vel)
    if args.dist in HAND_DESIGNS:
        if geometry is None:
            raise ValueError(
                f"--dist {args.dist} needs the hand geometry (FK of the loaded wait pose)"
            )
        return hand_near_throws(args.dist, args.n, args.seed, geometry)
    return frozen_throws(args.dist, profile, args.n, args.seed)


def apply_mirror(profile: ArmProfile, mirror: dict) -> ArmProfile:
    """``profile`` with the wait pose the running controller loaded.

    Raises ``ValueError`` when a mirror value is missing or its wait pose does
    not fit the arm: aligning to anything else would refuse every trial of an
    overlay run, or accept a start pose the controller does not wait at.
    """
    missing = [name for name in MIRROR_PARAMETERS if mirror.get(name) is None]
    if missing:
        raise ValueError(
            f"{CATCHING} does not expose {missing} — not configured, or parked at configure "
            "(its log names the value)"
        )
    wait_pose = tuple(float(v) for v in mirror["planner.wait_pose"])
    if len(wait_pose) != len(profile.joint_names):
        raise ValueError(
            f"the controller's planner.wait_pose has {len(wait_pose)} values but device "
            f"{profile.device} has {len(profile.joint_names)} joints"
        )
    return dataclasses.replace(profile, wait_pose=wait_pose)


def mirror_from_replies(names, ask) -> dict:
    """``{name: value | None}`` from a GetParameters caller ``ask(names)``.

    ``ask`` returns one ``(type, value)`` per name. But rclcpp's parameter
    service answers with an EMPTY list when ANY requested name is undeclared
    (the node does not allow undeclared parameters, and the whole lookup
    throws) — exactly the parked-controller case this runner must name. So an
    answer of the wrong length is re-asked one name at a time, and a name that
    still has no value is ``None``: ``apply_mirror`` then says which.
    """
    names = list(names)
    replies = ask(names)
    if len(replies) != len(names):
        replies = []
        for name in names:
            one = ask([name])
            replies.append(one[0] if len(one) == 1 else (0, None))
    return {
        name: (None if kind == 0 else value)
        for name, (kind, value) in zip(names, replies, strict=True)
    }


def _parameter_value(value):
    """The Python value of an ``rcl_interfaces/ParameterValue`` the mirror uses."""
    from rcl_interfaces.msg import ParameterType

    if value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return list(value.double_array_value)
    if value.type == ParameterType.PARAMETER_DOUBLE:
        return value.double_value
    if value.type == ParameterType.PARAMETER_BOOL:
        return value.bool_value
    return None


def _cycle_closed(modes: list[str]) -> bool:
    """Whether a trial's mode sequence re-armed after its RETREAT (one S7 cycle)."""
    if "RETREAT" not in modes:
        return False
    return "ARMED" in modes[modes.index("RETREAT") + 1 :]


def _make_driver(profile: ArmProfile, args):
    # ROS imports stay here so the helpers above are importable without a
    # sourced ROS environment.
    import rclpy
    from nav_msgs.msg import Odometry
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters, SetParameters
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger

    from rtc_msgs.msg import CatchingState
    from rtc_msgs.srv import LaunchBall, ResetFault

    class TrialDriver(Node):
        def __init__(self) -> None:
            super().__init__("catching_sim_trials")
            # The sim publishes state best-effort: a default (reliable)
            # subscriber is incompatible and silently receives nothing.
            best_effort = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST
            )
            self.launch_cli = self.create_client(LaunchBall, "/sim/launch_ball_at")
            self.reset_ball_cli = self.create_client(Trigger, "/sim/reset_ball")
            self.reset_fault_cli = self.create_client(ResetFault, "/rtc_cm/reset_fault")
            self.param_cli = self.create_client(
                SetParameters, f"/{CATCHING}/{CATCHING}/set_parameters"
            )
            self.get_param_cli = self.create_client(
                GetParameters, f"/{CATCHING}/{CATCHING}/get_parameters"
            )
            self.create_subscription(
                CatchingState, f"/{CATCHING}/catching_state", self._on_state, 10
            )
            self.create_subscription(
                Odometry, "/sim/ball/ground_truth", self._on_truth, best_effort
            )
            self.create_subscription(JointState, profile.state_topic, self._on_joints, best_effort)
            # Replaced by main() with the controller's mirrored wait pose.
            self.profile = profile
            self.ball_params = None  # set for a hand-near series (the aim check's flight law)
            self.mode = None
            self.armed = None
            self.outcome = None
            self.retreat_outcome = None
            self.mode_log = []
            self.truth_rows = []
            self.offsets = []
            self.tick_range = [None, None]
            self.recording = False
            self.q = None
            self.qd = None

        # ── subscriptions ────────────────────────────────────────────────
        def _on_state(self, msg) -> None:
            now = time.time()
            if msg.mode != self.mode:
                name = MODE_NAMES[msg.mode] if msg.mode < len(MODE_NAMES) else str(msg.mode)
                self.mode_log.append((now, name, msg.reason))
                self.mode = msg.mode
                if name == "RETREAT" and self.recording:
                    # The verdict the controller published on the RETREAT entry
                    # (HOLD judges on that edge; an abort sets it there).
                    self.retreat_outcome = msg.outcome
            self.outcome = msg.outcome
            self.armed = msg.armed
            if self.recording:
                if self.tick_range[0] is None:
                    self.tick_range[0] = msg.tick
                self.tick_range[1] = msg.tick
                self.offsets.append(now - msg.t_relative_s)

        def _on_truth(self, msg) -> None:
            if self.recording:
                p = msg.pose.pose.position
                v = msg.twist.twist.linear
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                self.truth_rows.append((time.time(), stamp, p.x, p.y, p.z, v.x, v.y, v.z))

        def _on_joints(self, msg) -> None:
            index = {n: i for i, n in enumerate(msg.name)}
            if all(n in index for n in profile.joint_names):
                self.q = [msg.position[index[n]] for n in profile.joint_names]
                self.qd = (
                    [msg.velocity[index[n]] for n in profile.joint_names]
                    if msg.velocity
                    else [0.0] * len(profile.joint_names)
                )

        # ── helpers ──────────────────────────────────────────────────────
        def mode_name(self) -> str:
            return MODE_NAMES[self.mode] if self.mode is not None else "UNKNOWN"

        def call(self, client, request, timeout_s: float = 5.0):
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
            return future.result()

        def spin_for(self, seconds: float) -> None:
            end = time.time() + seconds
            while time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.02)

        def wait_for_mode(self, name: str, timeout_s: float) -> bool:
            end = time.time() + timeout_s
            while time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.02)
                if self.mode_name() == name:
                    return True
            return False

        def set_enable(self, value: bool) -> bool:
            req = SetParameters.Request()
            req.parameters = [
                Parameter(
                    name="catching.enable",
                    value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value),
                )
            ]
            res = self.call(self.param_cli, req)
            return res is not None and all(r.successful for r in res.results)

        def read_mirror(self) -> dict:
            """The controller's read-only mirror (``MIRROR_PARAMETERS``); None = absent."""

            def ask(names):
                req = GetParameters.Request()
                req.names = list(names)
                res = self.call(self.get_param_cli, req)
                if res is None:
                    raise RuntimeError(f"/{CATCHING}/{CATCHING}/get_parameters did not answer")
                return [(v.type, _parameter_value(v)) for v in res.values]

            return mirror_from_replies(MIRROR_PARAMETERS, ask)

        def wait_for_services(self, timeout_s: float = 10.0) -> None:
            for client, name in (
                (self.launch_cli, "/sim/launch_ball_at"),
                (self.reset_ball_cli, "/sim/reset_ball"),
                (self.param_cli, f"/{CATCHING}/{CATCHING}/set_parameters"),
                (self.get_param_cli, f"/{CATCHING}/{CATCHING}/get_parameters"),
            ):
                if not client.wait_for_service(timeout_sec=timeout_s):
                    raise RuntimeError(f"service {name} unavailable")

        # ── the procedure ────────────────────────────────────────────────
        def home(self) -> dict:
            """Step 1 of the module docstring. Raises when it fails."""
            t0 = time.time()
            self.spin_for(0.1)
            if self.mode_name() == "FAULT":
                req = ResetFault.Request()
                req.controller_name = CATCHING
                res = self.call(self.reset_fault_cli, req)
                self.get_logger().warn(f"reset_fault before the trial -> {res}")
                self.spin_for(0.2)
            start_q = list(self.q) if self.q else None
            if self.mode_name() != "ARMED":
                # False → True, not just True: the controller lowers its latch
                # itself (E-STOP, fault, a supervisor disarm) and the parameter
                # can still read true, so a bare `true` would re-set the value it
                # already has. The edge is the request (A-S5-3).
                if not self.set_enable(False):
                    raise RuntimeError("disarm before re-arm failed")
                self.spin_for(0.05)
                if not self.set_enable(True):
                    raise RuntimeError("arm failed")
            # The controller homes itself (S7.2); ARMED means it says it is at
            # the wait pose with the hand at q_pre.
            if not self.wait_for_mode("ARMED", args.home_timeout):
                err = alignment_error(self.q, self.qd, self.profile.wait_pose) if self.q else None
                raise RuntimeError(f"not ARMED (mode {self.mode_name()}), error {err}")
            self.spin_for(0.2)
            eq, eqd = alignment_error(self.q, self.qd, self.profile.wait_pose)
            if eq > args.tol_q or eqd > args.tol_qd:
                # Recorded AND refused: the controller's own arrival test is
                # `supervisor.ready.pose_tol`, and an ARMED arm outside this gate
                # is a finding about the homing, not a start pose to accept.
                raise RuntimeError(f"ARMED away from the wait pose: {eq:.4f} rad, {eqd:.4f} rad/s")
            return {
                "armed_at_throw": self.armed,
                "home_start_q": start_q,
                "home_total_s": round(time.time() - t0, 2),
                "q_at_throw": [round(v, 5) for v in self.q],
                "err_q_at_throw": round(eq, 5),
                "err_qd_at_throw": round(eqd, 5),
            }

        def throw(self, idx: int, throw: dict) -> dict:
            """Step 5. Returns the trial record written to trial_results.json."""
            self.mode_log, self.truth_rows, self.offsets = [], [], []
            self.retreat_outcome = None
            self.tick_range = [None, None]
            self.recording = True
            req = LaunchBall.Request()
            req.position.x, req.position.y, req.position.z = throw["pos"]
            req.velocity.x, req.velocity.y, req.velocity.z = throw["vel"]
            omega = tuple(throw.get("omega", (0.0, 0.0, 0.0)))
            req.angular_velocity.x, req.angular_velocity.y, req.angular_velocity.z = omega
            res = self.call(self.launch_cli, req)
            launch_wall_time = time.time()
            record = {"idx": idx, **throw, "accepted": bool(res and res.accepted)}
            record["omega"] = omega
            record["seed"] = args.seed
            record["message"] = res.message if res else "no response"
            if not record["accepted"]:
                self.recording = False
                self.get_logger().error(f"trial {idx}: launch refused: {record['message']}")
                return record
            # Until the cycle closes (RETREAT → ARMED), the controller gives up
            # (IDLE, FAULT), or the window runs out.
            end = time.time() + args.record_s
            while time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.02)
                names = [m[1] for m in self.mode_log]
                if "RETREAT" in names and names[-1] in ("ARMED", "IDLE", "FAULT"):
                    break
                if names and names[-1] == "FAULT":
                    break
            self.spin_for(0.1)
            self.recording = False

            truth_csv = os.path.join(args.out_dir, f"truth_trial_{idx:02d}.csv")
            with open(truth_csv, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["wall_recv_s", "stamp_s", "x", "y", "z", "vx", "vy", "vz"])
                writer.writerows(self.truth_rows)
            offsets = sorted(self.offsets)
            self.call(self.reset_ball_cli, Trigger.Request())
            if (
                throw.get("target_m") is not None
                and self.ball_params is not None
                and self.truth_rows
            ):
                # The hand-near aim, checked against the flight the sim actually
                # started (the truth ends at the hand, often before the aimed
                # point): free-flight law from the first truth sample.
                from rtc_tools.analysis.catchability_map import aim_check_from_truth

                rows = self.truth_rows
                record.update(
                    aim_check_from_truth(
                        [r[1] for r in rows],
                        [[r[2], r[3], r[4]] for r in rows],
                        [[r[5], r[6], r[7]] for r in rows],
                        throw["target_m"],
                        self.ball_params,
                    )
                )
            record.update(
                {
                    "mode_log": self.mode_log,
                    "n_truth_rows": len(self.truth_rows),
                    # Relative to the trials dir (the file sits next to
                    # trial_results.json): an absolute path breaks when the dir
                    # is renamed and silently reads ANOTHER run's file when its
                    # old path is reused (S8-E smoke). The analyser still
                    # resolves the absolute paths of older records.
                    "truth_csv": os.path.basename(truth_csv),
                    "final_outcome": (
                        OUTCOME_NAMES[self.outcome] if self.outcome is not None else None
                    ),
                    # The attempt's verdict (L7 §4.7): published on RETREAT entry.
                    "outcome": (
                        OUTCOME_NAMES[self.retreat_outcome]
                        if self.retreat_outcome is not None
                        else None
                    ),
                    # RETREAT followed by ARMED — the controller re-armed. What
                    # comes after (a new ball seen, TRACKING) does not reopen it.
                    "cycle_closed": _cycle_closed([m[1] for m in self.mode_log]),
                    "final_mode": self.mode_name(),
                    "launch_wall_time": launch_wall_time,
                    "tick_min": self.tick_range[0],
                    "tick_max": self.tick_range[1],
                    # median of (wall − t_relative_s): joins this record to the
                    # controller CSV's time axis
                    "wall_t_relative_offset": offsets[len(offsets) // 2] if offsets else None,
                }
            )
            self.get_logger().info(
                f"trial {idx}: modes={[m[1] for m in self.mode_log]} "
                f"outcome={record['outcome']} final_mode={record['final_mode']}"
            )
            return record

    return rclpy, TrialDriver


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("out_dir", help="where truth_trial_NN.csv and trial_results.json go")
    parser.add_argument("--profile", default="ur5e_p1b", help="robot profile (config/<profile>)")
    parser.add_argument(
        "--config-dir", help="override the profile's config directory (default: installed share)"
    )
    parser.add_argument(
        "--dist",
        choices=("reference", *sorted(FROZEN_DISTRIBUTIONS), *sorted(HAND_DESIGNS)),
        default="reference",
        help=(
            "throw series: the reference regression set, iid draws from a frozen box, or a "
            "hand-near design aimed at the loaded wait pose (S8-F; a hand_* grid ignores --n)"
        ),
    )
    parser.add_argument(
        "--n", type=int, default=25, help="throws drawn with --dist <box|hand_lhs>"
    )
    # Hand-near (S8-F) inputs. The drag preset is a C++ constexpr the runner
    # cannot read, so it is passed with its file:line like catchability_map's
    # CLI does; the beanbag preset is 0.5 (projectile_ball.cpp:39).
    parser.add_argument("--arm", help="free label for run_meta.json (which overlay this unit ran)")
    parser.add_argument(
        "--limit",
        type=int,
        help="run only the first N throws of the series (a smoke of a grid); the series itself is unchanged",
    )
    parser.add_argument(
        "--floor-z",
        type=float,
        default=0.05,
        help="hand-near: lowest z the ball surface may reach [m] (ur5e_p1b work-table top)",
    )
    parser.add_argument(
        "--drag-coefficient", type=float, default=0.55, help="hand-near: sim ball Cd"
    )
    parser.add_argument(
        "--drag-coefficient-source",
        default="rtc_mujoco_sim/src/projectile_ball.cpp:30",
        help="hand-near: file:line the Cd came from",
    )
    parser.add_argument(
        "--air-density", type=float, default=1.204, help="hand-near: sim air density [kg/m^3]"
    )
    parser.add_argument(
        "--air-density-source",
        default="rtc_mujoco_sim/include/rtc_mujoco_sim/projectile_ball.hpp:98",
        help="hand-near: file:line the air density came from",
    )
    parser.add_argument(
        "--urdf", help="hand-near: expanded URDF (default: the profile's urdf.package/path)"
    )
    parser.add_argument("--n-ref", type=int, default=15, help="reference throws")
    parser.add_argument("--n-pert", type=int, default=10, help="perturbed throws after them")
    parser.add_argument("--seed", type=int, default=42, help="perturbation / sampling RNG seed")
    parser.add_argument("--release-pos", type=float, nargs=3, default=REFERENCE_RELEASE_POS)
    parser.add_argument("--release-vel", type=float, nargs=3, default=REFERENCE_RELEASE_VEL)
    parser.add_argument(
        "--tol-q", type=float, default=0.03, help="alignment check at ARMED, rad (pose_tol 0.02)"
    )
    parser.add_argument(
        "--tol-qd", type=float, default=0.05, help="alignment check at ARMED, rad/s"
    )
    parser.add_argument("--home-timeout", type=float, default=30.0, help="s to reach ARMED")
    parser.add_argument(
        "--record-s", type=float, default=12.0, help="longest window per throw (cycle end first)"
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    config_dir = args.config_dir
    if config_dir is None:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory("integrated_bringup")
        config_dir = os.path.join(share, "config", args.profile)
    file_profile = load_arm_profile(config_dir)
    os.makedirs(args.out_dir, exist_ok=True)
    # A hand-near series is aimed at the wait pose the controller LOADED, so it
    # is built after the mirror is read (below); the others need no controller.
    hand = args.dist in HAND_DESIGNS
    throws = [] if hand else build_throws(args, args.profile)
    if args.limit is not None and not hand:
        throws = throws[: max(args.limit, 0)]

    rclpy, TrialDriver = _make_driver(file_profile, args)
    rclpy.init()
    node = TrialDriver()
    results = []
    try:
        node.wait_for_services()
        mirror = node.read_mirror()
        profile = apply_mirror(file_profile, mirror)
        # The driver's closures read the profile it was built with; the homing
        # gate must use the pose the controller loaded, not the file's.
        node.profile = profile
        if profile.wait_pose != file_profile.wait_pose:
            node.get_logger().warn(
                f"the controller's wait pose differs from {config_dir} (an overlay?) — "
                f"aligning to the controller's {profile.wait_pose}"
            )
        geometry = None
        if hand:
            geometry = hand_geometry(
                config_dir,
                profile,
                floor_z_m=args.floor_z,
                drag_coefficient=args.drag_coefficient,
                drag_coefficient_source=args.drag_coefficient_source,
                air_density_kg_m3=args.air_density,
                air_density_source=args.air_density_source,
                urdf=args.urdf,
            )
            throws = build_throws(args, args.profile, geometry)
            node.ball_params = geometry.params
            if args.limit is not None:
                throws = throws[: max(args.limit, 0)]
            node.get_logger().info(
                f"hand-near series {args.dist}: {len(throws)} throws at p_c {geometry.p_c_m} "
                f"axis {geometry.approach_axis} (elevation {geometry.axis_elevation_deg:.1f}°)"
            )
        with open(os.path.join(args.out_dir, "run_meta.json"), "w") as f:
            json.dump(
                {
                    "args": dict(vars(args)),
                    "config_dir": config_dir,
                    "controller_mirror": mirror,
                    "n_throws": len(throws),
                    "arm": args.arm,
                    "hand_geometry": geometry.as_record() if geometry else None,
                },
                f,
                indent=2,
                default=str,
            )
        for idx, throw in enumerate(throws):
            alignment = node.home()
            node.get_logger().info(f"trial {idx}: aligned {alignment}")
            record = node.throw(idx, throw)
            record.update(alignment)
            record["controller_mirror"] = mirror
            results.append(record)
            node.spin_for(0.3)
        verdicts: dict[str, int] = {}
        for r in results:
            key = str(r.get("outcome"))
            verdicts[key] = verdicts.get(key, 0) + 1
        node.get_logger().info(
            f"{len(results)} trials, cycles closed "
            f"{sum(1 for r in results if r.get('cycle_closed'))}, verdicts {verdicts}"
        )
    finally:
        with open(os.path.join(args.out_dir, "trial_results.json"), "w") as f:
            json.dump(results, f, indent=2, default=str)
        node.destroy_node()
        rclpy.shutdown()
    return 0
