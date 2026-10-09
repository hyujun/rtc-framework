#!/usr/bin/env python3
"""Search-acceptance map: which throws the planner's catch search accepts (dynamic_catching L3 §4.1).

``catchability_map`` asks whether a catch posture exists and ``catch_gate_map``
whether the gates behind it open, one candidate at a time. This asks the
planner's SEARCH itself: for each throw, the wakes a planner would see are
handed to ``catch_search_batch`` (rtc_controllers), which calls the runtime
``CatchSearch::Plan`` — the grid search or the nlp search — and says what each
wake decided. The verdict is that binary's and is not re-derived here. What
this module owns is everything around it:

1. **The throws.** A grid over the launch axes the gate map uses (distance,
   azimuth, release height, aim deviation, speed, elevation), optionally a
   Latin-hypercube sample of the same box, or a throw list read back. Every
   throw has an integer ``throw_id``; a grid throw's id is its index in the
   product order ``catchability_map.generate_throw_grid`` uses. The throws are
   written as a ``catching_throw_list/1`` file (``catching_throw_list``, the
   format both sides read and write through), which the sim trial driver
   throws in file order: the offline map and the sim judge the same launches.
2. **The wakes.** The prediction a wake is handed is the flight model itself:
   the shipped drag law, integrated by ``catchability_map.integrate_flight``.
   A wake stands every vision period from the detection delay after release,
   for as long as the ball is in flight (above the floor, inside the flight
   horizon). Its snapshot is the flight's position, velocity and acceleration
   at the estimator's prediction instants — ``k × dt`` after the wake for
   ``k = 1 … horizon / dt``. The wake instant is the instant of the state the
   prediction starts from: no transport latency is modelled.
3. **The binding.** A controller resolves, at configure time, values that are
   not plain keys of the ``catching:`` tree (device ratings with their margins,
   the hand's close lead on the axis the plan's catch instant is on, the
   control period). :func:`search_binding` derives them from the composed tree
   and the robot configuration, one formula per value, each citing the
   controller function it mirrors, and records where every value came from.
   Nothing compiled into the controller is copied: a key whose absence would
   make the controller run a built-in number is refused by name.
4. **The reduction.** One row per throw: accepted or not (accepted = some wake
   returned a plan), the first accepting wake with its catch instant and lead,
   and for a refused throw its representative reason — the most frequent wake
   reason, ties to the later one, by the very function that reduces a sim
   throw's planner events (``catching_trials.most_frequent_reason`` over
   ``catching_trials._wake_reject_reason``). The wake × reason counts are kept
   unreduced next to it. Only the wakes up to a throw's first plan are judged:
   the arm stands on the wait pose and follows no plan.
5. **The summaries.** Acceptance rate per axis bin, reason distributions, the
   throws on which the two searches disagree, a per-throw join against a
   ``catch_gate_map`` output (gate verdict × search verdict per reach layer),
   and a sim table — search verdict × published or not × ``truth_success`` —
   from ``catching_trials`` output joined by ``throw_id``. Descriptive counts
   only; nothing here is a pass or a fail.

The map's axis values are defined without the search: a throw's flight time
and terminal speed are those of the flight model at its closest approach to
the catch point of the wait pose (FK of ``planner.wait_pose``), so a refused
throw has them too.

**Which search runs** is an argument, not the tree's ``planner.search.mode``:
both searches can be run on the same throws and the same tree. The tree's own
mode decides nothing here; what matters is that the selected search's map
(``planner.search.grid`` / ``planner.search.nlp``) is IN the composed tree. A
tree without it is refused by :func:`search_binding` before the binary runs:
the binary itself would configure the search on its built-in defaults. The
tree's ``planner.segment.mode`` does matter — it decides the close lead, the
speed margin of the nlp search's limits and ``follows_segments``.

**Frames.** Throws, the throw list, the flight and the map's axis values are
in the SIM WORLD. The wake CSV handed to the binary, and the plan it returns
(``p_c``, ``v_c``), are in the MODEL WORLD (the URDF root). The two are
composed the way the controller composes them: ``model_world_T_base`` read from
the URDF at ``catching.io.arm_base_frame`` times ``catching.io.base_T_world``
(``catching_trials.CatchFrameFk``). Instants are integer nanoseconds on one
axis on which every throw is released at :data:`RELEASE_NS`.

**Not reproduced.** What a running clock produces (a search cut by its budget,
a solve past its deadline), what the covariance changes (it is zero here), the
search after a plan is followed, and the conditions a controller parks on
before it ever searches. Those layers are read from the sim.

Every robot value is an argument or read from the profile.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import heapq
import json
import math
import random
import subprocess
import sys
import time
from collections import Counter
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from rtc_tools.analysis.catch_gate_map import LAYERS, REASON_NONE, composed_catching
from rtc_tools.analysis.catch_speed_budget import DEFAULT_CATCH_FRAME, extra_frame_from_params
from rtc_tools.analysis.catchability_map import (
    GRAVITY_W_M_S2,
    BallParams,
    _child_env,
    ball_acceleration,
    ball_params_from_shape,
    ball_shape_from_config,
    closest_approach,
    distribution,
    find_judge,
    generate_throw_grid,
    integrate_flight,
    invert_transform,
    throw_to_launch_request,
    transform_direction,
    transform_point,
    write_model_config,
)

# The throw-list format is catching_throw_list's alone: the sim driver reads and
# writes through the same module, so a list this map writes is one it throws.
from rtc_tools.analysis.catching_throw_list import (
    THROW_LIST_FRAME,
    THROW_LIST_SCHEMA,
    load_throw_list as read_throw_list,
    parse_throw_list,  # noqa: F401 (the map's parser is the format's)
    write_throw_list,
)
from rtc_tools.analysis.catching_trials import (
    GATE_MAP_AXES,
    PLAN_VERDICT_NO_PLAN,
    PLAN_VERDICT_NO_SEARCH,
    PLAN_VERDICT_PUBLISHED,
    PLAN_VERDICT_WITHHELD,
    ROBOT_CONFIG_FILES,
    CatchFrameFk,
    _wake_reject_reason,
    load_gate_map_summary,
    load_profile,
    load_throw_grid_axes,
    most_frequent_reason,
    vision_frame_from_io,
)
from rtc_tools.analysis.derive_accel_limits import load_robot_params

SEARCH_EXECUTABLE = "catch_search_batch"
SEARCH_KINDS = ("grid", "nlp")
# The binary's exit code for "the search refused its configuration": an answer
# about the profile, with no map to write from it.
SEARCH_REFUSED_EXIT = 3

THROW_AXES = GATE_MAP_AXES
# generate_throw_grid's keyword for each axis.
_GRID_KEYWORD = {
    "distance_m": "distances_m",
    "azimuth_deg": "azimuths_deg",
    "release_height_m": "release_heights_m",
    "aim_deviation_deg": "aim_deviations_deg",
    "speed_m_s": "speeds_m_s",
    "elevation_deg": "elevations_deg",
}
FLIGHT_AXES = ("flight_time_s", "terminal_speed_m_s", "closest_distance_m")

# Every throw is released at this instant of the batch's time axis [ns]. Positive,
# so no instant of a throw is zero or negative.
RELEASE_NS = 1_000_000_000

# A throw that was given no wake (the ball left flight before the first one):
# refused, by a label that is not a search reason.
REASON_NO_WAKE = "no_wake"

SEGMENT_MODES = ("closed_form", "mpc", "mpc_docking")
TBD_LITERAL = "TBD"

SIM_ACCEPTED_PUBLISHED = "accepted_published"
SIM_ACCEPTED_NOT_PUBLISHED = "accepted_not_published"
SIM_REFUSED = "refused"
SIM_CLASSES = (SIM_ACCEPTED_PUBLISHED, SIM_ACCEPTED_NOT_PUBLISHED, SIM_REFUSED)

_TRUE_CELLS = ("1", "True", "true")
_FALSE_CELLS = ("0", "False", "false")


# ── Throws ────────────────────────────────────────────────────────────────────


def _throw_record(throw, throw_id: int, kind: str) -> dict:
    """One throw as a throw-list entry: launch state, id, and its six axis values."""
    request = throw_to_launch_request(throw)
    return {
        "throw_id": int(throw_id),
        "kind": kind,
        "pos": tuple(request["position"][k] for k in "xyz"),
        "vel": tuple(request["velocity"][k] for k in "xyz"),
        "omega": tuple(request["angular_velocity"][k] for k in "xyz"),
        **{axis: float(getattr(throw, axis)) for axis in THROW_AXES},
    }


def grid_throws(
    axes: Mapping[str, Sequence[float]],
    *,
    origin_xy_m: Sequence[float],
    first_id: int = 0,
    kind: str = "grid",
) -> list[dict]:
    """The full grid over ``axes`` (one list per :data:`THROW_AXES` key), SIM WORLD.

    Product order and geometry are ``generate_throw_grid``'s, so with
    ``first_id = 0`` a throw's ``throw_id`` is the ``throw_index`` a
    ``catchability_map`` over the same axes and origin gives it.
    """
    missing = [axis for axis in THROW_AXES if axis not in axes]
    if missing:
        raise ValueError(f"the throw grid needs every axis; missing {missing}")
    grid = generate_throw_grid(
        base_xy_m=origin_xy_m, **{_GRID_KEYWORD[axis]: list(axes[axis]) for axis in THROW_AXES}
    )
    return [_throw_record(throw, first_id + i, kind) for i, throw in enumerate(grid)]


def lhs_throws(
    ranges: Mapping[str, tuple[float, float]],
    n: int,
    seed: int,
    *,
    origin_xy_m: Sequence[float],
    first_id: int = 0,
    kind: str = "lhs",
) -> list[dict]:
    """A Latin-hypercube sample of ``n`` throws from the box ``ranges`` spans, SIM WORLD.

    Each axis is cut into ``n`` equal strata and every stratum is drawn from
    exactly once, the strata paired across axes by independent shuffles.
    Deterministic in ``seed`` (``random.Random``): a sample is replayed from
    ``(ranges, n, seed)`` alone. An axis whose range is one value stays there.
    """
    if n < 1:
        raise ValueError(f"the sample needs n >= 1, got {n}")
    missing = [axis for axis in THROW_AXES if axis not in ranges]
    if missing:
        raise ValueError(f"the sample box needs every axis; missing {missing}")
    rng = random.Random(seed)
    columns: dict[str, list[float]] = {}
    for axis in THROW_AXES:
        lo, hi = (float(v) for v in ranges[axis])
        if hi < lo:
            raise ValueError(f"{axis}: the range ({lo}, {hi}) is not ordered")
        strata = list(range(n))
        rng.shuffle(strata)
        columns[axis] = [lo + (hi - lo) * (s + rng.random()) / n for s in strata]
    throws = []
    for i in range(n):
        (throw,) = generate_throw_grid(
            base_xy_m=origin_xy_m,
            **{_GRID_KEYWORD[axis]: (columns[axis][i],) for axis in THROW_AXES},
        )
        throws.append(_throw_record(throw, first_id + i, kind))
    return throws


def axis_ranges(axes: Mapping[str, Sequence[float]]) -> dict[str, tuple[float, float]]:
    """The box a grid spans: (min, max) of each axis's values."""
    return {axis: (float(min(values)), float(max(values))) for axis, values in axes.items()}


# ── The flight and the wakes ──────────────────────────────────────────────────


class _Flight:
    """One flight carried forward through ascending instants after release, leg by leg.

    The leg from the instant last reached to the next one is ``integrate_flight``
    in equal RK4 steps no longer than ``max_step_s``. A state depends only on the
    instants before it, so a caller that picks its instants as it goes reads, at
    each one, the state :func:`flight_states` returns for the same instants. A
    :meth:`branch` reads a state without moving the flight: the instants it is
    carried through, and so every later state, stay what they were.
    """

    def __init__(
        self,
        position_w: Sequence[float],
        velocity_w: Sequence[float],
        ball: BallParams,
        *,
        max_step_s: float,
        gravity_w: Sequence[float] = GRAVITY_W_M_S2,
    ) -> None:
        if not (math.isfinite(max_step_s) and max_step_s > 0.0):
            raise ValueError(f"max_step_s must be finite and > 0 (got {max_step_s!r})")
        self.position_w = np.asarray(position_w, dtype=float).reshape(3)
        self.velocity_w = np.asarray(velocity_w, dtype=float).reshape(3)
        self.reached_ns = 0
        self._ball = ball
        self._max_step_s = max_step_s
        self._gravity_w = gravity_w

    def _leg(self, instant_ns: int):
        gap_ns = int(instant_ns) - self.reached_ns
        if gap_ns < 0:
            raise ValueError(
                f"{int(instant_ns)} ns is before {self.reached_ns} ns, already reached"
            )
        if gap_ns == 0:
            return None
        gap_s = gap_ns * 1e-9
        steps = max(1, math.ceil(gap_s / self._max_step_s - 1e-12))
        leg = integrate_flight(
            self.position_w,
            self.velocity_w,
            self._ball,
            horizon_s=gap_s,
            step_s=gap_s / steps,
            gravity_w=self._gravity_w,
        )
        if leg.time_s.size != steps + 1:
            raise RuntimeError(
                f"the flight leg to {int(instant_ns)} ns took {leg.time_s.size - 1} steps, "
                f"not {steps}: its end is not the requested instant"
            )
        return leg

    def advance(self, instant_ns: int):
        """Carry the flight to ``instant_ns``; the leg's ``Trajectory`` (None if already there)."""
        leg = self._leg(instant_ns)
        if leg is not None:
            self.position_w, self.velocity_w = leg.position_m[-1], leg.velocity_m_s[-1]
            self.reached_ns = int(instant_ns)
        return leg

    def branch(self, instant_ns: int) -> tuple[np.ndarray, np.ndarray]:
        """Position and velocity at ``instant_ns``, the flight left where it is."""
        leg = self._leg(instant_ns)
        if leg is None:
            return self.position_w, self.velocity_w
        return leg.position_m[-1], leg.velocity_m_s[-1]


def flight_states(
    position_w: Sequence[float],
    velocity_w: Sequence[float],
    ball: BallParams,
    instants_ns: Sequence[int],
    *,
    max_step_s: float,
    gravity_w: Sequence[float] = GRAVITY_W_M_S2,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Position, velocity and acceleration of one flight at ``instants_ns`` after release.

    ``instants_ns`` are ascending, distinct and not negative. The flight is
    integrated from one instant to the next with ``integrate_flight`` in equal
    RK4 steps no longer than ``max_step_s``, so every returned state is an
    integrator state — none is interpolated — and all of them lie on ONE
    trajectory. The acceleration is the force law's at the returned velocity
    (``ball_acceleration``: gravity plus the drag term).
    """
    instants = np.asarray(instants_ns, dtype=np.int64)
    if instants.ndim != 1 or instants.size == 0:
        raise ValueError("instants_ns must be a non-empty 1-D sequence")
    if instants[0] < 0 or np.any(np.diff(instants) <= 0):
        raise ValueError("instants_ns must be ascending, distinct and >= 0")
    flight = _Flight(position_w, velocity_w, ball, max_step_s=max_step_s, gravity_w=gravity_w)
    positions = np.empty((instants.size, 3))
    velocities = np.empty((instants.size, 3))
    for i, instant in enumerate(instants):
        flight.advance(int(instant))
        positions[i], velocities[i] = flight.position_w, flight.velocity_w
    accelerations = np.array([ball_acceleration(row, ball, gravity_w) for row in velocities])
    return positions, velocities, accelerations


def _to_ns(seconds: float, name: str) -> int:
    if not math.isfinite(seconds):
        raise ValueError(f"{name} must be finite (got {seconds!r})")
    return int(round(float(seconds) * 1e9))


@dataclass(frozen=True)
class WakeTiming:
    """When the wakes of a throw stand and what each one's snapshot covers. All ns."""

    detection_delay_ns: int
    vision_period_ns: int
    prediction_dt_ns: int
    prediction_points: int
    max_flight_ns: int
    floor_world_z_m: float
    max_step_s: float

    @classmethod
    def from_seconds(
        cls,
        *,
        detection_delay_s: float,
        vision_period_s: float,
        prediction_dt_s: float,
        prediction_horizon_s: float,
        max_flight_s: float,
        floor_world_z_m: float,
        max_step_s: float,
    ) -> WakeTiming:
        """Round every time to whole ns. The horizon is an exact multiple of the spacing."""
        delay = _to_ns(detection_delay_s, "detection_delay_s")
        period = _to_ns(vision_period_s, "vision_period_s")
        dt = _to_ns(prediction_dt_s, "prediction_dt_s")
        horizon = _to_ns(prediction_horizon_s, "prediction_horizon_s")
        flight = _to_ns(max_flight_s, "max_flight_s")
        if delay < 0:
            raise ValueError("detection_delay_s must be >= 0")
        if period <= 0 or dt <= 0 or flight <= 0:
            raise ValueError("vision_period_s, prediction_dt_s and max_flight_s must be > 0")
        if horizon < dt or horizon % dt != 0:
            raise ValueError(
                f"prediction_horizon_s ({prediction_horizon_s}) must be a whole multiple of "
                f"prediction_dt_s ({prediction_dt_s}), at least one"
            )
        if not math.isfinite(floor_world_z_m):
            raise ValueError("floor_world_z_m must be finite")
        if not (math.isfinite(max_step_s) and max_step_s > 0.0):
            raise ValueError("max_step_s must be finite and > 0")
        return cls(delay, period, dt, horizon // dt, flight, float(floor_world_z_m), max_step_s)

    def wake_offsets_ns(self) -> list[int]:
        """Every wake instant after release inside the flight horizon, ascending."""
        count = (self.max_flight_ns - self.detection_delay_ns) // self.vision_period_ns + 1
        return [self.detection_delay_ns + i * self.vision_period_ns for i in range(max(count, 0))]


@dataclass(frozen=True, eq=False)
class Wake:
    """One wake of one throw: its instant and the prediction it is handed (MODEL WORLD)."""

    throw_id: int
    wake: int
    now_ns: int
    t_ns: np.ndarray
    position_m: np.ndarray
    velocity_m_s: np.ndarray
    acceleration_m_s2: np.ndarray


@dataclass(frozen=True, eq=False)
class ThrowFlight:
    """One throw's flight, integrated once (SIM WORLD, seconds and ns after release).

    The flight is carried through ``sample_instants_ns`` — the prediction
    samples of every wake in flight, ascending — and on to ``horizon_s`` when
    one is asked for; ``sample_position_w`` / ``sample_velocity_w`` are its
    states there. ``wake_offsets_ns`` are the wakes found in flight.
    ``path_time_s`` / ``path_position_w`` / ``path_velocity_w`` are every
    integrator state the flight went through, ascending: the closest approach
    is read off them.
    """

    sample_instants_ns: np.ndarray
    sample_position_w: np.ndarray
    sample_velocity_w: np.ndarray
    wake_offsets_ns: tuple[int, ...]
    path_time_s: np.ndarray
    path_position_w: np.ndarray
    path_velocity_w: np.ndarray


def _prediction_steps_ns(timing: WakeTiming) -> list[int]:
    return [k * timing.prediction_dt_ns for k in range(1, timing.prediction_points + 1)]


def integrate_throw(
    throw: Mapping,
    ball: BallParams,
    *,
    max_step_s: float,
    timing: WakeTiming | None = None,
    horizon_s: float | None = None,
) -> ThrowFlight:
    """Carry ``throw``'s flight ONCE through what its wakes and its closest approach need.

    The flight goes through the prediction samples of the wakes in flight, so
    every sample is, bit for bit, :func:`flight_states` over those samples. A
    wake is judged (above ``floor_world_z_m`` or not) on a branch from the
    flight's last sample before it, which leaves the samples untouched: the
    first wake below the floor ends the wakes, and its state is
    :func:`flight_states` over the earlier samples and the wake. With
    ``horizon_s`` the flight then goes on to it in ``integrate_flight`` steps
    of ``max_step_s`` — from release, with no wake, that is
    ``integrate_flight(horizon_s, max_step_s)`` itself.
    """
    flight = _Flight(throw["pos"], throw["vel"], ball, max_step_s=max_step_s)
    offsets = timing.wake_offsets_ns() if timing is not None else []
    steps = _prediction_steps_ns(timing) if timing is not None else []
    queue: list[int] = []
    queued: set[int] = set()
    next_wake, wakes_open = 0, True
    in_flight: list[int] = []
    samples: list[int] = []
    sample_p: list[np.ndarray] = []
    sample_v: list[np.ndarray] = []
    path_t: list[np.ndarray] = [np.zeros(1)]
    path_p: list[np.ndarray] = [flight.position_w.reshape(1, 3)]
    path_v: list[np.ndarray] = [flight.velocity_w.reshape(1, 3)]

    def keep_path(start_ns: int, leg) -> None:
        if leg is not None:
            path_t.append(start_ns * 1e-9 + leg.time_s[1:])
            path_p.append(leg.position_m[1:])
            path_v.append(leg.velocity_m_s[1:])

    while True:
        wake_t = offsets[next_wake] if wakes_open and next_wake < len(offsets) else None
        if wake_t is not None and (not queue or wake_t < queue[0]):
            p, _ = flight.branch(wake_t)
            next_wake += 1
            if p[2] < timing.floor_world_z_m:
                wakes_open = False
                continue
            in_flight.append(wake_t)
            for step in steps:
                if wake_t + step not in queued:
                    queued.add(wake_t + step)
                    heapq.heappush(queue, wake_t + step)
            continue
        if not queue:
            break
        t = heapq.heappop(queue)
        start = flight.reached_ns
        keep_path(start, flight.advance(t))
        samples.append(t)
        sample_p.append(flight.position_w)
        sample_v.append(flight.velocity_w)
    if horizon_s is not None:
        rest_s = horizon_s - flight.reached_ns * 1e-9 if flight.reached_ns else horizon_s
        if rest_s > 0.0:
            start = flight.reached_ns
            keep_path(
                start,
                integrate_flight(
                    flight.position_w,
                    flight.velocity_w,
                    ball,
                    horizon_s=rest_s,
                    step_s=max_step_s,
                ),
            )
    return ThrowFlight(
        np.asarray(samples, dtype=np.int64),
        np.asarray(sample_p, dtype=float).reshape(-1, 3),
        np.asarray(sample_v, dtype=float).reshape(-1, 3),
        tuple(in_flight),
        np.concatenate(path_t),
        np.concatenate(path_p),
        np.concatenate(path_v),
    )


def _wakes_of(
    throw: Mapping,
    flight: ThrowFlight,
    ball: BallParams,
    timing: WakeTiming,
    model_t_world: np.ndarray,
) -> list[Wake]:
    if not flight.wake_offsets_ns:
        return []
    steps = _prediction_steps_ns(timing)
    row_of = {int(t): i for i, t in enumerate(flight.sample_instants_ns)}
    a_w = np.array(
        [ball_acceleration(row, ball, GRAVITY_W_M_S2) for row in flight.sample_velocity_w]
    )
    p_m = transform_point(flight.sample_position_w, model_t_world)
    v_m = transform_direction(flight.sample_velocity_w, model_t_world)
    a_m = transform_direction(a_w, model_t_world)
    wakes = []
    for index, offset in enumerate(flight.wake_offsets_ns):
        rows = [row_of[offset + step] for step in steps]
        wakes.append(
            Wake(
                throw_id=int(throw["throw_id"]),
                wake=index,
                now_ns=RELEASE_NS + offset,
                t_ns=np.array([RELEASE_NS + offset + step for step in steps], dtype=np.int64),
                position_m=p_m[rows],
                velocity_m_s=v_m[rows],
                acceleration_m_s2=a_m[rows],
            )
        )
    return wakes


def _axes_of(flight: ThrowFlight, target_w: Sequence[float], horizon_s: float) -> dict[str, float]:
    # Every integrator state up to the first one at or past the horizon.
    end = int(np.searchsorted(flight.path_time_s, horizon_s, side="left")) + 1
    distance, at, speed = closest_approach(
        flight.path_time_s[:end],
        flight.path_position_w[:end],
        flight.path_velocity_w[:end],
        target_w,
    )
    return {"flight_time_s": at, "terminal_speed_m_s": speed, "closest_distance_m": distance}


def throw_wakes(
    throw: Mapping, ball: BallParams, timing: WakeTiming, model_t_world: np.ndarray
) -> list[Wake]:
    """The wakes of ``throw`` (launch state in the SIM WORLD), samples in the MODEL WORLD.

    Wake ``i`` stands ``detection_delay + i × vision_period`` after release. The
    first wake at which the ball is below ``floor_world_z_m`` (sim world height)
    or past the flight horizon ends the throw: it and every later one are not
    produced. A wake's samples are the flight's states ``k × prediction_dt``
    after it, ``k = 1 … prediction_points`` — all of them, also those below
    the floor: the prediction is the flight model, which has no ground.
    """
    flight = integrate_throw(throw, ball, max_step_s=timing.max_step_s, timing=timing)
    return _wakes_of(throw, flight, ball, timing, model_t_world)


def flight_axes(
    throw: Mapping,
    ball: BallParams,
    target_w: Sequence[float],
    *,
    horizon_s: float,
    step_s: float,
) -> dict[str, float]:
    """A throw's map axes that need the flight: time, speed and distance at its closest
    approach to ``target_w`` (SIM WORLD) — the catch point of the wait pose — over the
    flight integrated to ``horizon_s`` in steps of ``step_s``. They are the flight
    model's alone, so a throw the search refused has them as well.
    """
    flight = integrate_throw(throw, ball, max_step_s=step_s, horizon_s=horizon_s)
    return _axes_of(flight, target_w, horizon_s)


def throw_flight(
    throw: Mapping,
    ball: BallParams,
    timing: WakeTiming,
    model_t_world: np.ndarray,
    target_w: Sequence[float],
    *,
    horizon_s: float,
) -> tuple[list[Wake], dict[str, float]]:
    """:func:`throw_wakes` and :func:`flight_axes` of one throw from ONE integration.

    The wakes are :func:`throw_wakes`' bit for bit (:func:`integrate_throw`).
    The closest approach is read off every integrator state of that same
    flight, ``timing.max_step_s`` apart at most, up to ``horizon_s``.
    """
    flight = integrate_throw(
        throw, ball, max_step_s=timing.max_step_s, timing=timing, horizon_s=horizon_s
    )
    return _wakes_of(throw, flight, ball, timing, model_t_world), _axes_of(
        flight, target_w, horizon_s
    )


WAKE_CSV_HEADER = (
    "throw_id,wake,now_ns,t_ns,p_x,p_y,p_z,v_x,v_y,v_z,a_x,a_y,a_z"  # catch_search_batch --wakes
)


def wake_csv_lines(wakes: Sequence[Wake]) -> list[str]:
    """The binary's wake CSV, one line per predicted sample. ``repr`` round-trips a double."""
    lines = [WAKE_CSV_HEADER]
    for wake in wakes:
        head = f"{wake.throw_id},{wake.wake},{wake.now_ns}"
        for t, p, v, a in zip(
            wake.t_ns, wake.position_m, wake.velocity_m_s, wake.acceleration_m_s2, strict=True
        ):
            cells = ",".join(repr(float(x)) for x in (*p, *v, *a))
            lines.append(f"{head},{int(t)},{cells}")
    return lines


# ── The binding ───────────────────────────────────────────────────────────────


class BindingError(ValueError):
    """A value the search needs cannot be bound from the configuration. Names the key."""


_ABSENT = object()


def _lookup(tree: Mapping, path: str):
    node = tree
    for key in path.split("."):
        if not isinstance(node, Mapping) or key not in node:
            return _ABSENT
        node = node[key]
    return node


def _scalar(value, path: str) -> float:
    """A scalar of the tree as the controller's parser reads it: a number, also one the
    YAML loader left as text. NaN for the ``TBD`` literal and for a non-finite number."""
    if isinstance(value, str):
        if value == TBD_LITERAL:
            return math.nan
        try:
            number = float(value)
        except ValueError:
            number = None
    elif isinstance(value, bool) or not isinstance(value, int | float):
        number = None
    else:
        number = float(value)
    if number is None:
        raise BindingError(f"catching.{path} = {value!r} is neither a number nor '{TBD_LITERAL}'")
    return number if math.isfinite(number) else math.nan


def _decision(tree: Mapping, path: str) -> float:
    """A key the controller leaves open when it is absent: absent, ``TBD`` → NaN."""
    value = _lookup(tree, path)
    return math.nan if value is _ABSENT else _scalar(value, path)


def _written(tree: Mapping, path: str, why: str):
    """A key the controller has a built-in value for: this tool copies none, so it must
    be written in the composed tree."""
    value = _lookup(tree, path)
    if value is _ABSENT:
        raise BindingError(
            f"catching.{path} is not set — {why}; the controller would run a value compiled "
            "into it, which this tool does not copy"
        )
    return value


def _resolved(tree: Mapping, path: str, why: str) -> float:
    """As :func:`_written`, and resolved: a ``TBD`` there also runs a built-in value."""
    number = _scalar(_written(tree, path, why), path)
    if math.isnan(number):
        raise BindingError(
            f"catching.{path} is '{TBD_LITERAL}' — {why}; the controller would run a value "
            "compiled into it, which this tool does not copy"
        )
    return number


@dataclass(frozen=True)
class DeviceLimits:
    """One device's joint limits as a controller receives them, in DEVICE order.

    An array is ``None`` when the device's ``joint_limits`` block exists and
    does not give it: a controller then runs a fallback constant of its own.
    """

    device: str
    joint_names: tuple[str, ...]
    position_lower: np.ndarray | None
    position_upper: np.ndarray | None
    max_velocity: np.ndarray | None
    max_torque: np.ndarray | None


_LIMIT_ARRAYS = ("position_lower", "position_upper", "max_velocity", "max_torque")


def merged_device_limits(
    robot_params: Mapping, device: str, urdf_limits: Mapping[str, Mapping[str, float]]
) -> DeviceLimits:
    """``devices.<device>.joint_limits`` merged with the URDF's, tighter bound per joint.

    Mirrors the controller manager's device configuration
    (``RtControllerNode``, "Merge joint limits with URDF"): an array the block
    gives is intersected with the URDF limit of the same joint; a device with
    no array at all takes all four from the URDF; an array missing next to
    others stays ``None``. ``urdf_limits`` maps a joint name to its
    ``position_lower`` / ``position_upper`` / ``max_velocity`` / ``max_torque``.
    """
    try:
        node = robot_params["devices"][device]
        names = tuple(str(n) for n in node["joint_state_names"])
    except (KeyError, TypeError) as exc:
        raise BindingError(f"robot config lacks devices.{device}.joint_state_names") from exc
    unknown = [name for name in names if name not in urdf_limits]
    if unknown:
        raise BindingError(f"devices.{device}: joints {unknown} are not in the URDF")
    block = node.get("joint_limits") or {}
    given: dict[str, np.ndarray] = {}
    for key in _LIMIT_ARRAYS:
        values = block.get(key)
        if values is None or len(values) == 0:
            continue
        array = np.asarray(values, dtype=float)
        if array.shape != (len(names),):
            raise BindingError(
                f"devices.{device}.joint_limits.{key} has {array.size} entries, the device "
                f"{len(names)} joints"
            )
        given[key] = array
    from_urdf = {
        key: np.array([urdf_limits[name][key] for name in names]) for key in _LIMIT_ARRAYS
    }
    if not given:
        return DeviceLimits(device, names, **from_urdf)
    merged: dict[str, np.ndarray | None] = {}
    for key in _LIMIT_ARRAYS:
        if key not in given:
            merged[key] = None
        elif key == "position_lower":
            merged[key] = np.maximum(given[key], from_urdf[key])
        else:
            merged[key] = np.minimum(given[key], from_urdf[key])
    return DeviceLimits(device, names, **merged)


@dataclass(frozen=True)
class RobotFacts:
    """What :func:`search_binding` reads besides the ``catching:`` tree."""

    control_rate_hz: float
    arm: DeviceLimits
    # The hand device's limits as the controller manager hands them over, or
    # None when they were not read (only the nlp binding reads them).
    hand: DeviceLimits | None
    # The catch sub-model's joints in the model's velocity order, and the URDF
    # position limits in that order.
    model_joint_names: tuple[str, ...]
    urdf_position_lower: np.ndarray
    urdf_position_upper: np.ndarray


def robot_facts(
    robot_params: Mapping,
    arm_device: str,
    hand_device: str | None,
    urdf_text: str,
    sub_model: str,
) -> RobotFacts:
    """Read :class:`RobotFacts` from the merged robot parameters and the URDF.

    The catch sub-model is ``urdf.sub_models.<sub_model>``: the chain from its
    root link to its tip link with every other joint locked, so its joints keep
    the full model's relative order. It must hold exactly the arm device's
    joints, as a controller requires (``ResolveCatchSubModel``).

    The arm's limits enter the binding, so an arm joint whose limits the
    controller manager reads from another joint's entry is refused. The hand's
    enter only as "is its position box complete" (``BuildClikBoxes``), so they
    are read exactly as the manager reads them, whatever the joint's kind.
    ``hand_device`` None reads nothing of the hand (``hand`` is None): the grid
    binding needs none of it.
    """
    import pinocchio as pin  # noqa: PLC0415

    rate = robot_params.get("control_rate")
    if rate is None:
        raise BindingError(
            "robot config lacks control_rate — the control period is 1 / control_rate, and "
            "the node's built-in rate is not copied here"
        )
    model = pin.buildModelFromXML(urdf_text)

    def joint_limits(name: str) -> dict[str, float]:
        joint_id = model.getJointId(name)
        joint = model.joints[joint_id]
        # The controller manager indexes the model's limit vectors by a joint's
        # place in the joint list; that is the joint's own index only while every
        # joint before it has one degree of freedom.
        if joint.nq != 1 or joint.nv != 1 or joint.idx_q != joint_id - 1:
            raise BindingError(
                f"joint '{name}': the limit merge reads entry {joint_id - 1} of the URDF "
                f"limit vectors, but the joint's own entry is {joint.idx_q} (nq {joint.nq})"
            )
        return {
            "position_lower": float(model.lowerPositionLimit[joint.idx_q]),
            "position_upper": float(model.upperPositionLimit[joint.idx_q]),
            "max_velocity": float(model.velocityLimit[joint.idx_v]),
            "max_torque": float(model.effortLimit[joint.idx_v]),
        }

    def manager_limits(name: str) -> dict[str, float]:
        # The controller manager's read ("Merge joint limits with URDF"): entry
        # joint_id − 1 of each limit vector. For a multi-DoF or continuous joint,
        # or any joint after one, that is another coordinate's entry — what the
        # manager hands the controller all the same.
        index = model.getJointId(name) - 1
        return {
            "position_lower": float(model.lowerPositionLimit[index]),
            "position_upper": float(model.upperPositionLimit[index]),
            "max_velocity": float(model.velocityLimit[index]),
            "max_torque": float(model.effortLimit[index]),
        }

    def device(name: str, read) -> DeviceLimits:
        try:
            joints = [str(j) for j in robot_params["devices"][name]["joint_state_names"]]
        except (KeyError, TypeError) as exc:
            raise BindingError(f"robot config lacks devices.{name}.joint_state_names") from exc
        missing = [j for j in joints if not model.existJointName(j)]
        if missing:
            raise BindingError(f"devices.{name}: joints {missing} are not in the URDF")
        return merged_device_limits(robot_params, name, {j: read(j) for j in joints})

    arm = device(arm_device, joint_limits)
    try:
        entry = robot_params["urdf"]["sub_models"][sub_model]
        root, tip = str(entry["root_link"]), str(entry["tip_link"])
    except (KeyError, TypeError) as exc:
        raise BindingError(
            f"robot config lacks urdf.sub_models.{sub_model} (catching.planner.sub_model)"
        ) from exc
    for link in (root, tip):
        if not model.existFrame(link):
            raise BindingError(f"urdf.sub_models.{sub_model}: the URDF has no link '{link}'")
    stop = model.frames[model.getFrameId(root)].parentJoint
    joint_id = model.frames[model.getFrameId(tip)].parentJoint
    chain = []
    while joint_id not in (0, stop):
        chain.append(joint_id)
        joint_id = model.parents[joint_id]
    chain_names = {model.names[j] for j in chain}
    if chain_names != set(arm.joint_names):
        raise BindingError(
            f"urdf.sub_models.{sub_model} holds joints {sorted(chain_names)}, the arm device "
            f"'{arm_device}' {sorted(arm.joint_names)}: the catch sub-model must contain the "
            "arm's joints only"
        )
    ordered = sorted(chain, key=lambda j: model.joints[j].idx_v)
    return RobotFacts(
        control_rate_hz=float(rate),
        arm=arm,
        hand=None if hand_device is None else device(hand_device, manager_limits),
        model_joint_names=tuple(model.names[j] for j in ordered),
        urdf_position_lower=np.array(
            [float(model.lowerPositionLimit[model.joints[j].idx_q]) for j in ordered]
        ),
        urdf_position_upper=np.array(
            [float(model.upperPositionLimit[model.joints[j].idx_q]) for j in ordered]
        ),
    )


@dataclass(frozen=True)
class SearchBinding:
    """A binding file's content and where each of its values came from.

    ``document`` is what ``ParseSearchBatchBinding`` reads. ``sources`` maps a
    dotted key of it to ``source`` (the key path or the formula), ``mirrors``
    (the controller function the formula copies) and, where the controller
    declares one, ``mirror_param`` — its read-only parameter that carries the
    same value at run time.
    """

    kind: str
    document: dict
    sources: dict

    def yaml_text(self) -> str:
        return yaml.safe_dump(self.document, sort_keys=False, default_flow_style=None, width=100)


def _floats(values) -> list[float]:
    return [float(v) for v in values]


def _device_array(limits: DeviceLimits, key: str) -> np.ndarray:
    array = getattr(limits, key)
    if array is None:
        raise BindingError(
            f"devices.{limits.device}.joint_limits.{key} is not set while other limits of the "
            "device are — the controller would run a fallback constant compiled into it, which "
            "this tool does not copy"
        )
    return array


def _box_complete(limits: DeviceLimits) -> bool:
    """Whether a device's position limits let the CLIK position box stand
    (``BuildClikBoxes``): every joint finite and ordered. A device whose arrays
    are absent runs the controller's fallback constants, which are."""
    lower, upper = limits.position_lower, limits.position_upper
    if lower is None or upper is None:
        return True
    return bool(
        np.all(np.isfinite(lower)) and np.all(np.isfinite(upper)) and np.all(lower <= upper)
    )


def search_binding(catching: Mapping, robot: RobotFacts, kind: str) -> SearchBinding:
    """What a controller configured with ``catching`` on ``robot`` hands the ``kind`` search.

    ``catching`` is the COMPOSED tree (includes and overlays merged); ``kind``
    stands in for its ``planner.search.mode``. Joint vectors are in MODEL
    order. Raises :class:`BindingError` naming the key on a missing input.
    """
    if kind not in SEARCH_KINDS:
        raise BindingError(f"the search must be one of {SEARCH_KINDS}, got {kind!r}")
    sources: dict[str, dict] = {}

    def note(key: str, source: str, mirrors: str, mirror_param: str | None = None) -> None:
        sources[key] = {"source": source, "mirrors": mirrors}
        if mirror_param is not None:
            sources[key]["mirror_param"] = mirror_param

    arm = robot.arm
    nv = len(robot.model_joint_names)
    if sorted(robot.model_joint_names) != sorted(arm.joint_names):
        raise BindingError(
            f"the catch sub-model's joints {list(robot.model_joint_names)} are not the arm "
            f"device's {list(arm.joint_names)}"
        )
    # ResolveCatchSubModel: model joint (velocity order) → index in the arm
    # device's joint_state_names.
    device_of_model = [arm.joint_names.index(name) for name in robot.model_joint_names]
    note(
        "device_of_model",
        f"index in devices.{arm.device}.joint_state_names of each catch sub-model joint, "
        "model velocity order",
        "DemoCatchingController::ResolveCatchSubModel",
    )

    segment_mode = _written(catching, "planner.segment.mode", "what the arm follows")
    if segment_mode not in SEGMENT_MODES:
        raise BindingError(
            f"catching.planner.segment.mode = {segment_mode!r} must be one of {SEGMENT_MODES}"
        )
    docking = segment_mode == "mpc_docking"

    # RTControllerInterface::GetDefaultDt: 1 / control_rate.
    if not (math.isfinite(robot.control_rate_hz) and robot.control_rate_hz > 0.0):
        raise BindingError(f"control_rate = {robot.control_rate_hz!r} must be > 0")
    control_dt = 1.0 / robot.control_rate_hz
    dt_source = ("1 / control_rate (robot config)", "RTControllerInterface::GetDefaultDt")

    # SetupTrajInput: the lead is T_arm in whole ns, and zero unless the lead
    # axis is on and T_arm is resolved.
    lead_enable = _written(catching, "joint_cmd.lag.lead_enable", "whether the lead axis is on")
    if not isinstance(lead_enable, bool):
        raise BindingError(f"catching.joint_cmd.lag.lead_enable = {lead_enable!r} must be a bool")
    t_arm = _scalar(
        _written(catching, "joint_cmd.lag.T_arm", "the arm lag the lead axis runs on"),
        "joint_cmd.lag.T_arm",
    )
    t_arm_s = float(int(t_arm * 1e9)) * 1e-9 if lead_enable and not math.isnan(t_arm) else 0.0
    t_arm_source = (
        "joint_cmd.lag.T_arm in whole ns when joint_cmd.lag.lead_enable is true and T_arm is "
        "resolved, else 0",
        "DemoCatchingController::SetupTrajInput (t_arm_ns_)",
        "joint_cmd.lag.T_arm, joint_cmd.lag.lead_enable",
    )

    # HandProfile::CloseLead: T_close_lead when the profile writes it, else T_close_e2e.
    t_close_e2e = _decision(catching, "robot.hand.T_close_e2e")
    lead_given = _lookup(catching, "robot.hand.T_close_lead") is not _ABSENT
    close_lead = _decision(catching, "robot.hand.T_close_lead") if lead_given else t_close_e2e
    lead_key = "robot.hand.T_close_lead" if lead_given else "robot.hand.T_close_e2e"
    s_ent = _decision(catching, "robot.hand.docking.s_ent")

    def entrance_close_lead(core: str) -> tuple[float, str]:
        """EntranceCloseLead: the lead from a docking core's catch node, the ball's crossing
        of the entrance plane — shorter by the flight over s_ent at the reference closing
        speed c = −nu_ref.z of that core."""
        path = f"{core}.catch.nu_ref"
        nu_ref = _written(catching, path, "the reference closing speed of that core")
        if not isinstance(nu_ref, list | tuple) or len(nu_ref) != 3:
            raise BindingError(f"catching.{path} = {nu_ref!r} must be three numbers")
        c = -_scalar(nu_ref[2], path)
        text = f"{lead_key} − robot.hand.docking.s_ent / (−{path}[2])"
        if math.isnan(close_lead) or math.isnan(s_ent) or math.isnan(c) or not c > 0.0:
            return math.nan, text
        return close_lead - s_ent / c, text

    # ResolvedCloseLead: the lead from the catch instant of the planner whose
    # SEGMENTS the arm follows — the entrance crossing under mpc_docking.
    if docking:
        resolved_lead, lead_text = entrance_close_lead("planner.segment.mpc_docking.core")
        lead_text += " (planner.segment.mode mpc_docking)"
    else:
        resolved_lead = close_lead
        lead_text = f"{lead_key} (planner.segment.mode {segment_mode})"
    resolved_lead_source = (
        lead_text,
        "DemoCatchingController::ResolvedCloseLead",
        "hand.T_close_lead_from_t_c",
    )
    ball_mass = _decision(catching, "core.ball.mass")

    # ApplyArmAccelBox / ResolvePlannerArm: the acceleration box is on only when
    # robot.arm.qdd_max gives one positive number per arm joint.
    box = _lookup(catching, "robot.arm.qdd_max")
    if box is _ABSENT:
        qdd_model: list[float] = []
        qdd_text = "robot.arm.qdd_max is absent: no acceleration box"
    else:
        try:
            qdd = np.asarray(box, dtype=float)
        except (TypeError, ValueError) as exc:
            raise BindingError(f"catching.robot.arm.qdd_max = {box!r} is not a list") from exc
        if qdd.shape != (nv,) or not np.all(np.isfinite(qdd)) or not np.all(qdd > 0.0):
            raise BindingError(
                f"catching.robot.arm.qdd_max = {box!r} must be {nv} positive numbers (arm "
                "joint order)"
            )
        qdd_model = _floats(qdd[device_of_model])
        qdd_text = "robot.arm.qdd_max, device order → model order"
    rating = _device_array(arm, "max_velocity")
    rating_text = (
        f"min(devices.{arm.device}.joint_limits.max_velocity, URDF velocity limit), device "
        "order → model order"
    )

    document: dict = {"search": kind, "device_of_model": device_of_model}
    if kind == "grid":
        need = "the grid search's key"
        if not isinstance(_lookup(catching, "planner.search.grid"), Mapping):
            raise BindingError(
                "catching.planner.search.grid is not in this tree (the selected search's map "
                "— is its file included?)"
            )
        val = {
            # ResolvedSearchGridEtaV.
            "eta_v": _resolved(catching, "planner.search.grid.gamma.eta_v", need),
            "v_max": _decision(catching, "planner.search.grid.reference.v_max"),
            "a_dec": _decision(catching, "planner.search.grid.stop.a_dec"),
            "ref_omega": _scalar(
                _written(catching, "planner.search.grid.reference.omega", need),
                "planner.search.grid.reference.omega",
            ),
            "ref_zeta": _scalar(
                _written(catching, "planner.search.grid.reference.zeta", need),
                "planner.search.grid.reference.zeta",
            ),
            "ref_a_max": _decision(catching, "planner.search.grid.reference.a_max"),
        }
        setup = "DemoCatchingController::SetupGridCatchSearch"
        document["grid"] = {
            "qdot_max": _floats(rating[device_of_model]),
            "qddot_max": qdd_model,
            "eta_v": val["eta_v"],
            "v_max": val["v_max"],
            "a_dec": val["a_dec"],
            "t_arm_s": t_arm_s,
            "t_close_lead": resolved_lead,
            # T_close,tot: the measured closure time plus half a control period.
            "t_close_total": t_close_e2e + 0.5 * control_dt,
            "ball_mass": ball_mass,
            "ref_omega": val["ref_omega"],
            "ref_zeta": val["ref_zeta"],
            "ref_a_max": val["ref_a_max"],
            "control_dt": control_dt,
            # FollowsSegments: every segment mode but closed_form.
            "follows_segments": segment_mode != "closed_form",
        }
        planner_arm = "DemoCatchingController::ResolvePlannerArm"
        note("grid.qdot_max", rating_text, planner_arm)
        note("grid.qddot_max", qdd_text, planner_arm, "robot.arm.qdd_max")
        note(
            "grid.eta_v",
            "planner.search.grid.gamma.eta_v",
            "DemoCatchingController::ResolvedSearchGridEtaV",
            "planner.search.grid.gamma.eta_v",
        )
        note(
            "grid.v_max",
            "planner.search.grid.reference.v_max (TBD → NaN)",
            setup,
            "planner.search.grid.reference.v_max",
        )
        note("grid.a_dec", "planner.search.grid.stop.a_dec (TBD → NaN)", setup)
        note("grid.t_arm_s", *t_arm_source)
        note("grid.t_close_lead", *resolved_lead_source)
        note(
            "grid.t_close_total",
            "robot.hand.T_close_e2e + control_dt / 2 (TBD → NaN)",
            setup,
            "hand.T_close_e2e, control.dt",
        )
        note("grid.ball_mass", "core.ball.mass (TBD → NaN)", setup)
        note(
            "grid.ref_omega",
            "planner.search.grid.reference.omega (TBD → NaN)",
            setup,
            "planner.search.grid.reference.omega",
        )
        note("grid.ref_zeta", "planner.search.grid.reference.zeta (TBD → NaN)", setup)
        note(
            "grid.ref_a_max",
            "planner.search.grid.reference.a_max (TBD → NaN)",
            setup,
            "planner.search.grid.reference.a_max",
        )
        note("grid.control_dt", *dt_source, "control.dt")
        note(
            "grid.follows_segments",
            "planner.segment.mode is not closed_form",
            "rtc::catching::FollowsSegments",
            "planner.segment.mode",
        )
        return SearchBinding(kind, document, sources)

    # ── nlp ──
    # DockingConfigInvalid: the selected search's map and its core design have
    # to be in the profile, or every value of it is a built-in default.
    if not isinstance(_lookup(catching, "planner.search.nlp.core"), Mapping):
        raise BindingError(
            "catching.planner.search.nlp.core is not in this tree (the selected search's map "
            "— is its file included?)"
        )
    # ResolvedSegmentEtaV: the speed margin of the planner whose segments the
    # arm follows.
    eta_key = "planner.segment.mpc_docking.eta_v" if docking else "planner.segment.mpc.eta_v"
    eta_v = _resolved(catching, eta_key, "the speed margin of the cores' velocity box")
    # BuildDockingLimits: the CLIK's torque margin under its dynamic form, the
    # ratings themselves otherwise.
    form = _written(catching, "joint_cmd.accel_constraint", "the CLIK's acceleration form")
    if form not in ("kinematic", "dynamic"):
        raise BindingError(
            f"catching.joint_cmd.accel_constraint = {form!r} selects no form (kinematic or "
            "dynamic): the controller does not run the arm on it"
        )
    if form == "dynamic":
        eta_tau = _scalar(
            _written(catching, "joint_cmd.eta_tau", "the torque margin of the dynamic form"),
            "joint_cmd.eta_tau",
        )
        if math.isnan(eta_tau):
            eta_tau = 1.0
        eta_tau_text = "joint_cmd.eta_tau (accel_constraint dynamic; TBD → 1)"
    else:
        eta_tau = 1.0
        eta_tau_text = "1 (accel_constraint kinematic)"
    # SetupArmCommand / BuildClikBoxes: the device box pulled limit_margin
    # inwards from both sides, never past the joint's midpoint; all or nothing
    # over the arm and the hand.
    margin = _resolved(catching, "robot.arm.limit_margin", "the CLIK position box's margin")
    lower = _device_array(arm, "position_lower")
    upper = _device_array(arm, "position_upper")
    if robot.hand is None:
        raise BindingError(
            "the nlp binding needs the hand device's position limits (the CLIK box is all or "
            "nothing over the arm and the hand) — read them with robot_facts(hand_device=...)"
        )
    clik_box = _box_complete(arm) and _box_complete(robot.hand)
    mid = 0.5 * (lower + upper)
    margined_lower = np.minimum(lower + margin, mid)
    margined_upper = np.maximum(upper - margin, mid)
    q_min = robot.urdf_position_lower.copy()
    q_max = robot.urdf_position_upper.copy()
    if clik_box:
        q_min = np.maximum(q_min, margined_lower[device_of_model])
        q_max = np.minimum(q_max, margined_upper[device_of_model])
    torque = _device_array(arm, "max_torque")[device_of_model]
    hand_lead, hand_lead_text = entrance_close_lead("planner.search.nlp.core")
    document["nlp"] = {
        "t_arm_s": t_arm_s,
        "control_dt": control_dt,
        "t_close_lead": resolved_lead,
        "hand_t_close_e2e": t_close_e2e,
        "hand_t_close_lead": hand_lead,
        "ball_mass": ball_mass,
        "limits": {
            "q_min": _floats(q_min),
            "q_max": _floats(q_max),
            "qd_max": _floats(eta_v * rating[device_of_model]),
            "qdd_max": qdd_model,
            "tau_max": _floats(torque),
            "tau_lo": _floats(-eta_tau * torque),
            "tau_hi": _floats(eta_tau * torque),
        },
    }
    setup = "DemoCatchingController::SetupNlpCatchSearch"
    limits = "DemoCatchingController::BuildDockingLimits"
    box_text = (
        "URDF position limit ∩ the device limit pulled robot.arm.limit_margin inwards (not "
        "past the joint's midpoint)"
        if clik_box
        else "URDF position limit (the device position box is incomplete: no CLIK box)"
    )
    note("nlp.t_arm_s", *t_arm_source)
    note("nlp.control_dt", *dt_source, "control.dt")
    note("nlp.t_close_lead", *resolved_lead_source)
    note("nlp.hand_t_close_e2e", "robot.hand.T_close_e2e (TBD → NaN)", setup, "hand.T_close_e2e")
    note(
        "nlp.hand_t_close_lead",
        hand_lead_text,
        "DemoCatchingController::EntranceCloseLead (the search's own core)",
    )
    note("nlp.ball_mass", "core.ball.mass (TBD → NaN)", setup)
    note("nlp.limits.q_min", box_text, f"{limits}, DemoCatchingController::BuildClikBoxes")
    note("nlp.limits.q_max", box_text, f"{limits}, DemoCatchingController::BuildClikBoxes")
    note(
        "nlp.limits.qd_max",
        f"{eta_key} × {rating_text}",
        f"{limits}, DemoCatchingController::ResolvedSegmentEtaV",
        eta_key,
    )
    note("nlp.limits.qdd_max", qdd_text, limits, "robot.arm.qdd_max")
    torque_text = (
        f"min(devices.{arm.device}.joint_limits.max_torque, URDF effort limit), device order "
        "→ model order"
    )
    note("nlp.limits.tau_max", torque_text, limits)
    note("nlp.limits.tau_lo", f"−({eta_tau_text}) × tau_max", limits)
    note("nlp.limits.tau_hi", f"+({eta_tau_text}) × tau_max", limits)
    return SearchBinding(kind, document, sources)


# ── The batch ─────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class SearchRun:
    """What one run of the binary gave: its rows, or why the search refused its
    configuration (``refusal``, the binary's own text) and then no rows."""

    kind: str
    rows: list[dict] | None
    refusal: str | None
    wall_s: float
    argv: list[str]


def _cell(column: str, text: str):
    """One result cell, typed: the reason name stays text, an empty cell is NaN."""
    if column == "nlp_reason":
        return text
    if text == "":
        return math.nan
    try:
        return int(text)
    except ValueError:
        return float(text)


def parse_search_rows(text: str) -> list[dict]:
    """The binary's result CSV as typed rows (ints, floats, NaN for an empty plan cell)."""
    reader = csv.DictReader(text.splitlines())
    return [{column: _cell(column, cell) for column, cell in row.items()} for row in reader]


def run_search_batch(
    judge: Path,
    kind: str,
    *,
    model_config: Path,
    sub_model: str,
    catch_frame: str,
    params: Path,
    binding: Path,
    wakes: Path,
    out: Path,
) -> SearchRun:
    """Run ``catch_search_batch`` once. A search that refuses its configuration is a
    result (``SearchRun.refusal``); any other failure raises with the binary's stderr."""
    argv = [
        str(judge),
        "--model-config",
        str(model_config),
        "--sub-model",
        sub_model,
        "--catch-frame",
        catch_frame,
        "--params",
        str(params),
        "--binding",
        str(binding),
        "--wakes",
        str(wakes),
        "--out",
        str(out),
    ]
    start = time.monotonic()
    done = subprocess.run(argv, capture_output=True, text=True, env=_child_env(), check=False)
    wall = time.monotonic() - start
    # The model builder logs to stderr too: the binary's own lines carry its name.
    own = [line for line in done.stderr.splitlines() if line.startswith(f"{SEARCH_EXECUTABLE}:")]
    if done.returncode == SEARCH_REFUSED_EXIT:
        return SearchRun(kind, None, own[-1] if own else done.stderr.strip(), wall, argv)
    if done.returncode != 0:
        tail = "\n".join(own) if own else "\n".join(done.stderr.splitlines()[-20:])
        raise RuntimeError(
            f"{Path(judge).name} exited {done.returncode}\nargv: {' '.join(argv)}\nstderr:\n{tail}"
        )
    return SearchRun(kind, parse_search_rows(Path(out).read_text()), None, wall, argv)


# ── Reduction: wakes → one verdict per throw ──────────────────────────────────


def wake_reason(row: Mapping) -> str:
    """Why one batch wake returned no plan, in the planner event log's reading.

    A batch wake is a wake whose search result the planner cycle would publish
    as it is, so the row is read as a ``planner_events`` row of outcome
    ``published`` by the function that reads those.
    """
    return _wake_reject_reason({**row, "outcome": "published"})


def throw_verdicts(
    throw_ids: Sequence[int], rows: Sequence[Mapping], wakes_given: Mapping[int, int]
) -> list[dict]:
    """One verdict per throw of ``throw_ids``, in that order, from the batch's wake rows.

    - ``accepted``: some wake returned a plan. ``first_accept_wake`` is that
      wake, ``plan_after_release_s`` / ``t_c_s`` its instant and its plan's
      catch instant after release, ``lead_s`` the plan's lead.
    - ``reject_reason``: for a refused throw, the most frequent
      :func:`wake_reason` of its wakes (ties to the later one); ``no_wake`` for
      a throw that was given none; "" for an accepted throw.
    - ``wake_reasons``: reason → number of wakes that returned no plan, never
      reduced (for an accepted throw: the wakes before its plan).
    - ``wakes_given`` / ``wakes_run``: the wakes the throw had, and those the
      search ran. Rows after a throw's first plan are not read.
    """
    by_throw: dict[int, list[Mapping]] = {}
    for row in rows:
        by_throw.setdefault(int(row["throw_id"]), []).append(row)
    out = []
    for throw_id in throw_ids:
        own = sorted(by_throw.get(int(throw_id), ()), key=lambda r: int(r["wake"]))
        reasons: list[str] = []
        plan = None
        for row in own:
            if int(row["search_valid"]) == 1:
                plan = row
                break
            reasons.append(wake_reason(row))
        verdict = {
            "throw_id": int(throw_id),
            "accepted": plan is not None,
            "wakes_given": int(wakes_given.get(int(throw_id), 0)),
            "wakes_run": len(reasons) + (plan is not None),
            "first_accept_wake": None,
            "plan_after_release_s": math.nan,
            "t_c_s": math.nan,
            "lead_s": math.nan,
            "reject_reason": "",
            "wake_reasons": dict(Counter(reasons)),
        }
        if plan is not None:
            verdict["first_accept_wake"] = int(plan["wake"])
            verdict["plan_after_release_s"] = (int(plan["now_ns"]) - RELEASE_NS) * 1e-9
            verdict["t_c_s"] = (int(plan["t_c_ns"]) - RELEASE_NS) * 1e-9
            verdict["lead_s"] = float(plan["lead_s"])
        else:
            verdict["reject_reason"] = most_frequent_reason(reasons) or REASON_NO_WAKE
        out.append(verdict)
    return out


def accepted_plans(rows: Sequence[Mapping]) -> dict[int, Mapping]:
    """Each accepted throw's first plan row, by ``throw_id``."""
    plans: dict[int, Mapping] = {}
    for row in sorted(rows, key=lambda r: (int(r["throw_id"]), int(r["wake"]))):
        if int(row["search_valid"]) == 1:
            plans.setdefault(int(row["throw_id"]), row)
    return plans


# ── Summaries ─────────────────────────────────────────────────────────────────


def _finite(value) -> bool:
    return isinstance(value, int | float) and not isinstance(value, bool) and math.isfinite(value)


def acceptance_by_axis(
    rows: Sequence[Mapping], axis: str, edges: Sequence[float] | None = None
) -> list[dict]:
    """Accepted throws over throws along ``axis``.

    Without ``edges`` the throws are grouped by the axis's distinct values
    (``value``); with them into the bins ``[edges[i], edges[i + 1])``, the last
    one closed (``lo`` / ``hi``). A throw without a finite value of the axis is
    in no group; an empty bin has ``rate`` None.
    """
    have = [r for r in rows if _finite(r.get(axis))]
    if edges is None:
        groups: dict[float, list[Mapping]] = {}
        for row in have:
            groups.setdefault(float(row[axis]), []).append(row)
        return [
            {
                "axis": axis,
                "value": value,
                "n": len(group),
                "accepted": sum(1 for r in group if r["accepted"]),
                "rate": sum(1 for r in group if r["accepted"]) / len(group),
            }
            for value, group in sorted(groups.items())
        ]
    edges = [float(e) for e in edges]
    if len(edges) < 2 or any(b <= a for a, b in zip(edges, edges[1:], strict=False)):
        raise ValueError(f"{axis}: bin edges must be at least two ascending values")
    out = []
    last = len(edges) - 2
    for i, (lo, hi) in enumerate(zip(edges, edges[1:], strict=False)):
        group = [
            r for r in have if lo <= float(r[axis]) < hi or (i == last and float(r[axis]) == hi)
        ]
        accepted = sum(1 for r in group if r["accepted"])
        out.append(
            {
                "axis": axis,
                "lo": lo,
                "hi": hi,
                "n": len(group),
                "accepted": accepted,
                "rate": accepted / len(group) if group else None,
            }
        )
    return out


def axis_edges(rows: Sequence[Mapping], axis: str, bins: int) -> list[float] | None:
    """``bins`` equal-width bin edges over the finite values of ``axis``; None when the
    axis has fewer than two distinct values (nothing to bin)."""
    values = sorted({float(r[axis]) for r in rows if _finite(r.get(axis))})
    if len(values) < 2 or bins < 1:
        return None
    return [float(x) for x in np.linspace(values[0], values[-1], bins + 1)]


def _ranked(counts: Mapping[str, int]) -> dict[str, int]:
    return dict(sorted(counts.items(), key=lambda kv: (-kv[1], kv[0])))


def reason_distribution(verdicts: Sequence[Mapping]) -> dict:
    """Refusals by reason, at both levels: ``throws`` counts each refused throw once by
    its representative reason; ``wakes`` counts every wake that returned no plan, of
    every throw; ``wakes_of_refused_throws`` those of the refused throws only."""
    throws: Counter = Counter()
    wakes: Counter = Counter()
    wakes_refused: Counter = Counter()
    for verdict in verdicts:
        wakes.update(verdict["wake_reasons"])
        if not verdict["accepted"]:
            throws[verdict["reject_reason"]] += 1
            wakes_refused.update(verdict["wake_reasons"])
    return {
        "throws": _ranked(throws),
        "wakes": _ranked(wakes),
        "wakes_of_refused_throws": _ranked(wakes_refused),
    }


def search_disagreement(
    first: Sequence[Mapping], second: Sequence[Mapping], names: tuple[str, str] = SEARCH_KINDS
) -> dict:
    """The throws two searches judge differently.

    ``both_accepted`` / ``both_refused`` are counts; ``only_<name>`` lists the
    ``throw_id``s that search alone accepted, each with the other's reason;
    ``missing_in_<name>`` the ids only the other map holds.
    """
    a = {int(v["throw_id"]): v for v in first}
    b = {int(v["throw_id"]): v for v in second}
    common = sorted(set(a) & set(b))
    only_a = [t for t in common if a[t]["accepted"] and not b[t]["accepted"]]
    only_b = [t for t in common if b[t]["accepted"] and not a[t]["accepted"]]
    return {
        "throws": len(common),
        "both_accepted": sum(1 for t in common if a[t]["accepted"] and b[t]["accepted"]),
        "both_refused": sum(1 for t in common if not a[t]["accepted"] and not b[t]["accepted"]),
        f"only_{names[0]}": [
            {"throw_id": t, f"{names[1]}_reason": b[t]["reject_reason"]} for t in only_a
        ],
        f"only_{names[1]}": [
            {"throw_id": t, f"{names[0]}_reason": a[t]["reject_reason"]} for t in only_b
        ],
        f"missing_in_{names[0]}": sorted(set(b) - set(a)),
        f"missing_in_{names[1]}": sorted(set(a) - set(b)),
    }


GATE_OPEN = "open"
GATE_CLOSED = "closed"
GATE_NO_CANDIDATE = "no_candidate"
# Two throws are the same throw when every axis agrees to this many decimals.
AXIS_KEY_DECIMALS = 9


def _axis_key(row: Mapping) -> tuple | None:
    try:
        return tuple(round(float(row[axis]), AXIS_KEY_DECIMALS) for axis in THROW_AXES)
    except (KeyError, TypeError, ValueError):
        return None


def gate_throw_verdicts(
    throw_axes: Mapping[int, Mapping[str, float]], gate_rows: Sequence[Mapping], seed_id: int
) -> list[dict]:
    """One gate verdict per GRID throw of a ``catch_gate_map`` output, per reach layer.

    ``open``: at least one of the throw's candidates passes every gate of the
    layer. ``closed``: it has candidates and none passes — ``gate_reason`` is
    then the most frequent candidate reason (ties to the later candidate).
    ``no_candidate``: the kinematic map handed the gate map none for it.
    """
    reasons: dict[int, dict[str, list[str]]] = {}
    for row in gate_rows:
        if int(row["seed_id"]) != seed_id:
            continue
        per_layer = reasons.setdefault(int(row["throw_index"]), {layer: [] for layer in LAYERS})
        for layer in LAYERS:
            per_layer[layer].append(str(row[f"reason_{layer}"]))
    out = []
    for index in sorted(throw_axes):
        entry = {"throw_index": index, **{a: float(throw_axes[index][a]) for a in THROW_AXES}}
        for layer in LAYERS:
            own = reasons.get(index, {}).get(layer, [])
            if not own:
                entry[f"gate_{layer}"], entry[f"gate_reason_{layer}"] = GATE_NO_CANDIDATE, ""
            elif REASON_NONE in own:
                entry[f"gate_{layer}"], entry[f"gate_reason_{layer}"] = GATE_OPEN, ""
            else:
                entry[f"gate_{layer}"] = GATE_CLOSED
                entry[f"gate_reason_{layer}"] = most_frequent_reason(own)
        out.append(entry)
    return out


def load_gate_throws(gate_map_dir: Path) -> tuple[list[dict], dict]:
    """:func:`gate_throw_verdicts` of a ``catch_gate_map`` output directory, and the
    part of its summary a join is read against (the kinematic map it gated, the wait
    pose it gated from)."""
    gate_map_dir = Path(gate_map_dir)
    map_dir, seed_id = load_gate_map_summary(gate_map_dir)
    summary = yaml.safe_load((gate_map_dir / "gate_map_summary.yaml").read_text()) or {}
    with (gate_map_dir / "gate_map.csv").open(newline="") as handle:
        gate_rows = list(csv.DictReader(handle))
    throws = gate_throw_verdicts(
        load_throw_grid_axes(map_dir / "throw_summary.csv"), gate_rows, seed_id
    )
    return throws, {
        "gate_map_dir": str(gate_map_dir),
        "map_dir": str(map_dir),
        "seed_id": seed_id,
        "wait_pose": summary.get("wait_pose"),
    }


def gate_map_join(
    rows: Sequence[Mapping], gate_throws: Sequence[Mapping]
) -> tuple[list[dict], dict]:
    """Each search-map throw next to the gate-map throw with the SAME six axis values.

    Returns the joined rows (one per matched throw) and, per reach layer, the
    table gate verdict × search verdict with the reasons of both mismatching
    cells: the search's reason where the gates are open and it refused, the
    gates' reason where they are closed and it accepted. Throws without a
    partner are counted on both sides, not guessed at.
    """
    by_key = {_axis_key(g): g for g in gate_throws}
    joined = []
    unmatched = 0
    used = set()
    for row in rows:
        key = _axis_key(row)
        gate = by_key.get(key) if key is not None else None
        if gate is None:
            unmatched += 1
            continue
        used.add(key)
        joined.append(
            {
                "throw_id": int(row["throw_id"]),
                "gate_throw_index": gate["throw_index"],
                **{
                    f"{field}_{layer}": gate[f"{field}_{layer}"]
                    for layer in LAYERS
                    for field in ("gate", "gate_reason")
                },
                "accepted": bool(row["accepted"]),
                "reject_reason": row["reject_reason"],
            }
        )
    summary: dict = {
        "matched": len(joined),
        "search_throws_not_in_gate_map": unmatched,
        "gate_throws_not_in_search_map": len(by_key) - len(used),
        "layers": {},
    }
    for layer in LAYERS:
        table = {
            verdict: {"accepted": 0, "refused": 0}
            for verdict in (GATE_OPEN, GATE_CLOSED, GATE_NO_CANDIDATE)
        }
        open_refused: Counter = Counter()
        closed_accepted: Counter = Counter()
        for row in joined:
            gate = row[f"gate_{layer}"]
            table[gate]["accepted" if row["accepted"] else "refused"] += 1
            if gate == GATE_OPEN and not row["accepted"]:
                open_refused[row["reject_reason"]] += 1
            if gate != GATE_OPEN and row["accepted"]:
                closed_accepted[row[f"gate_reason_{layer}"] or gate] += 1
        summary["layers"][layer] = {
            "table": table,
            "gate_open_search_refused_by_search_reason": _ranked(open_refused),
            "gate_not_open_search_accepted_by_gate_reason": _ranked(closed_accepted),
        }
    return joined, summary


def sim_search_class(plan_verdict: str) -> str | None:
    """A sim throw's ``plan_verdict`` as a search verdict.

    The search accepted the throw when some wake's search found a plan:
    ``published`` (a plan reached the RT) and ``withheld`` (found, none
    stored). ``no_plan`` and ``no_search`` are refusals. None for anything
    else — a throw the trial analysis gave no verdict.
    """
    if plan_verdict == PLAN_VERDICT_PUBLISHED:
        return SIM_ACCEPTED_PUBLISHED
    if plan_verdict == PLAN_VERDICT_WITHHELD:
        return SIM_ACCEPTED_NOT_PUBLISHED
    if plan_verdict in (PLAN_VERDICT_NO_PLAN, PLAN_VERDICT_NO_SEARCH):
        return SIM_REFUSED
    return None


def _truth_cell(value) -> bool | None:
    if isinstance(value, bool | np.bool_):
        return bool(value)
    if isinstance(value, str):
        if value in _TRUE_CELLS:
            return True
        if value in _FALSE_CELLS:
            return False
    return None


def _throw_id_cell(value) -> int | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return int(number) if math.isfinite(number) and number == int(number) else None


def sim_table(
    sim_rows: Sequence[Mapping], verdicts: Sequence[Mapping] | None = None
) -> tuple[list[dict], dict]:
    """The sim's search verdict × outcome, and its agreement with an offline map.

    ``sim_rows`` are ``catching_trials`` per-trial rows (in memory or read back
    from its CSV). A trial with an ``invalid_reason`` is a rig failure and is
    left out, as is one without a ``plan_verdict``; both are counted. The table
    has the three rows accepted · published / accepted · not published /
    refused, each split by ``truth_success``. With ``verdicts`` (an offline
    map's), every remaining trial that carries a ``throw_id`` is also compared
    by it: the sim's "accepted" against the offline "accepted".
    """
    offline = {int(v["throw_id"]): v for v in verdicts} if verdicts is not None else None
    table = {c: {"truth_success": 0, "truth_fail": 0, "truth_unknown": 0} for c in SIM_CLASSES}
    counts = {"trials": len(sim_rows), "invalid": 0, "unjudged": 0, "without_throw_id": 0}
    not_published: Counter = Counter()
    refused: Counter = Counter()
    no_search = 0
    agree = {"both_accepted": 0, "both_refused": 0, "offline_only": [], "sim_only": []}
    without_offline: list[int] = []
    thrown: set[int] = set()
    joined = []
    for row in sim_rows:
        if str(row.get("invalid_reason") or "") not in ("", "nan"):
            counts["invalid"] += 1
            continue
        verdict = str(row.get("plan_verdict") or "")
        cls = sim_search_class(verdict)
        if cls is None:
            counts["unjudged"] += 1
            continue
        truth = _truth_cell(row.get("truth_success"))
        cell = "truth_unknown" if truth is None else "truth_success" if truth else "truth_fail"
        table[cls][cell] += 1
        reason = str(row.get("plan_reject") or "")
        if cls == SIM_ACCEPTED_NOT_PUBLISHED:
            not_published[reason] += 1
        elif cls == SIM_REFUSED:
            refused[reason or verdict] += 1
            no_search += verdict == PLAN_VERDICT_NO_SEARCH
        throw_id = _throw_id_cell(row.get("throw_id"))
        entry = {
            "throw_id": throw_id,
            "idx": row.get("idx"),
            "plan_verdict": verdict,
            "sim_class": cls,
            "sim_reason": reason,
            "truth_success": truth,
        }
        if throw_id is None:
            counts["without_throw_id"] += 1
        elif offline is not None:
            thrown.add(throw_id)
            mine = offline.get(throw_id)
            if mine is None:
                without_offline.append(throw_id)
            else:
                sim_accepted = cls != SIM_REFUSED
                entry["offline_accepted"] = bool(mine["accepted"])
                entry["offline_reason"] = mine["reject_reason"]
                if sim_accepted and mine["accepted"]:
                    agree["both_accepted"] += 1
                elif not sim_accepted and not mine["accepted"]:
                    agree["both_refused"] += 1
                elif mine["accepted"]:
                    agree["offline_only"].append(throw_id)
                else:
                    agree["sim_only"].append(throw_id)
        joined.append(entry)
    summary: dict = {
        **counts,
        "table": table,
        "refused_without_a_search": int(no_search),
        "not_published_by_reason": _ranked(not_published),
        "refused_by_reason": _ranked(refused),
    }
    if offline is not None:
        compared = (
            agree["both_accepted"]
            + agree["both_refused"]
            + len(agree["offline_only"])
            + len(agree["sim_only"])
        )
        summary["offline"] = {
            **agree,
            "compared": compared,
            "agreement": (agree["both_accepted"] + agree["both_refused"]) / compared
            if compared
            else None,
            "sim_throws_without_an_offline_verdict": sorted(without_offline),
            "offline_throws_not_thrown": len(set(offline) - thrown),
        }
    return joined, summary


# ── I/O ───────────────────────────────────────────────────────────────────────


def _jsonable(value):
    """``value`` with NaN / inf as null and numpy scalars as python ones."""
    if isinstance(value, Mapping):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, list | tuple):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return _jsonable(value.tolist())
    if isinstance(value, np.bool_):
        return bool(value)
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, float | np.floating):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, Path):
        return str(value)
    return value


def _write_rows(path: Path, rows: Sequence[Mapping]) -> None:
    keys: list[str] = []
    for row in rows:
        keys.extend(k for k in row if k not in keys)
    with Path(path).open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    k: json.dumps(v, sort_keys=True) if isinstance(v, Mapping) else v
                    for k, v in row.items()
                }
            )


def _read_rows(paths: Sequence[Path]) -> list[dict]:
    rows: list[dict] = []
    for path in paths:
        with Path(path).open(newline="") as handle:
            rows.extend(csv.DictReader(handle))
    return rows


def _axis(text: str) -> list[float]:
    return [float(v) for v in text.replace(",", " ").split()]


def map_summary(verdicts: Sequence[Mapping], rows: Sequence[Mapping], bins: int) -> dict:
    """One search's descriptive summary over its per-throw rows (``rows``: the verdicts
    with each throw's axis values)."""
    accepted = [r for r in rows if r["accepted"]]
    by_axis = {}
    for axis in THROW_AXES:
        by_axis[axis] = acceptance_by_axis(rows, axis)
    for axis in FLIGHT_AXES:
        edges = axis_edges(rows, axis, bins)
        by_axis[axis] = acceptance_by_axis(rows, axis, edges) if edges else []
    return {
        "throws": len(verdicts),
        "accepted": len(accepted),
        "refused": len(verdicts) - len(accepted),
        "acceptance_rate": len(accepted) / len(verdicts) if verdicts else None,
        "throws_without_a_wake": sum(1 for v in verdicts if v["wakes_given"] == 0),
        "reasons": reason_distribution(verdicts),
        "first_accept_wake": distribution(r["first_accept_wake"] for r in accepted),
        "lead_s": distribution(r["lead_s"] for r in accepted),
        "t_c_s": distribution(r["t_c_s"] for r in accepted),
        "acceptance_by_axis": by_axis,
    }


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument(
        "--config-dir",
        type=Path,
        required=True,
        help="the robot profile directory (controllers/, the robot config files)",
    )
    ap.add_argument("--controller", help="the catching controller's config key (default: the one)")
    ap.add_argument(
        "--robot-config",
        type=Path,
        nargs="+",
        help="robot config YAML(s) in the order the launch lays them on the node (default: "
        f"those of {', '.join(ROBOT_CONFIG_FILES)} the profile has)",
    )
    ap.add_argument(
        "--overlay",
        type=Path,
        action="append",
        default=[],
        help="a ROS-params YAML laid on after the robot config files, in the order given "
        "(repeatable): a sim overlay, an evaluation overlay. Pass the files the run launched "
        "with, or the map describes another configuration",
    )
    ap.add_argument("--search", nargs="+", choices=SEARCH_KINDS, required=True)
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--judge", type=Path, help=f"path to {SEARCH_EXECUTABLE} (default: ament)")

    # ── the ball: the same arguments as the kinematic map, none defaulted ──
    ap.add_argument("--ball-config", type=Path, nargs="+", required=True)
    ap.add_argument("--drag-coefficient", type=float, required=True)
    ap.add_argument("--drag-coefficient-source", required=True)
    ap.add_argument("--air-density", type=float, required=True, dest="air_density_kg_m3")
    ap.add_argument("--air-density-source", required=True)

    # ── the throws ──
    ap.add_argument(
        "--throws-file",
        type=Path,
        help=f"judge the throws of a {THROW_LIST_SCHEMA} file instead of designing a grid",
    )
    for axis in THROW_AXES:
        ap.add_argument(f"--{_GRID_KEYWORD[axis].replace('_', '-')}", type=_axis, dest=axis)
    ap.add_argument(
        "--grid-origin",
        choices=["arm_base", "wait_catch_point"],
        default="arm_base",
        help="the vertical axis distance, azimuth and aim deviation are measured about: the "
        "arm base (the kinematic map's grid) or the catch point of the wait pose",
    )
    ap.add_argument("--sample", type=int, default=0, help="also a Latin-hypercube sample of N")
    ap.add_argument("--sample-seed", type=int, help="its seed (required with --sample)")

    # ── the wakes ──
    ap.add_argument("--detection-delay-s", type=float, required=True, help="release → first wake")
    ap.add_argument("--vision-period-s", type=float, required=True, help="wake → next wake")
    ap.add_argument(
        "--prediction-horizon-s",
        type=float,
        required=True,
        help="how far a prediction reaches (the estimator profile's prediction.horizon_s)",
    )
    ap.add_argument(
        "--prediction-dt-s",
        type=float,
        help="prediction spacing (default: the tree's prediction.dt_expected)",
    )
    ap.add_argument("--horizon-s", type=float, default=2.5, help="flight horizon after release")
    ap.add_argument("--step-s", type=float, default=0.002, help="largest RK4 step")
    ap.add_argument(
        "--min-catch-height-m",
        type=float,
        default=0.0,
        help="sim-world height below which the ball is no longer in flight",
    )

    # ── joins and bins ──
    ap.add_argument("--gate-map", type=Path, help="a catch_gate_map output dir to join against")
    ap.add_argument(
        "--sim-trials",
        type=Path,
        nargs="+",
        help="catching_trials.csv file(s) of a sim run of the throw list",
    )
    ap.add_argument("--axis-bins", type=int, default=8, help="bins of the flight axes")
    args = ap.parse_args(argv)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    config_dir = Path(args.config_dir)
    profile = load_profile(config_dir, args.controller)
    if profile.hand_device is None:
        raise SystemExit(f"{profile.controller}: `topics` names no hand group after the arm")
    robot_files = (
        [Path(p) for p in args.robot_config]
        if args.robot_config
        else [config_dir / name for name in ROBOT_CONFIG_FILES if (config_dir / name).is_file()]
    )
    layers = [*robot_files, *[Path(p) for p in args.overlay]]
    controller_yaml = config_dir / "controllers" / f"{profile.controller}.yaml"
    catching, _ = composed_catching(controller_yaml, profile.controller, layers)
    robot_params = load_robot_params(layers)

    def tree_value(path: str, why: str):
        value = _lookup(catching, path)
        if value is _ABSENT:
            raise SystemExit(f"catching.{path} is not set — {why}")
        return value

    sub_model = str(tree_value("planner.sub_model", "the catch sub-model the search plans in"))
    wait_pose = _floats(tree_value("planner.wait_pose", "the arm's rest posture"))
    catch_frame = str(catching.get("catch_frame", DEFAULT_CATCH_FRAME))
    sub_models = (robot_params.get("urdf") or {}).get("sub_models") or {}
    artifacts = write_model_config(
        layers,
        out_dir,
        arm_sub_model=profile.arm_device if profile.arm_device in sub_models else None,
        catch_frame=catch_frame,
    )
    if artifacts.catch_sub_model != sub_model:
        raise SystemExit(
            f"catching.planner.sub_model is '{sub_model}' but the model config's catch "
            f"sub-model is '{artifacts.catch_sub_model}': the map and the runtime would plan in "
            "different models"
        )
    try:
        # Only the nlp binding reads the hand (whether its position box is complete).
        facts = robot_facts(
            robot_params,
            profile.arm_device,
            profile.hand_device if "nlp" in args.search else None,
            artifacts.urdf_text,
            sub_model,
        )
    except BindingError as exc:
        raise SystemExit(f"binding: {exc}") from exc
    if len(wait_pose) != len(facts.arm.joint_names):
        raise SystemExit("catching.planner.wait_pose must give one angle per arm joint")

    # The frames and the wait pose's catch point, composed from the COMPOSED
    # tree: an overlay that moves the vision frame moves them with it.
    base_frame, base_t_world = vision_frame_from_io(catching.get("io") or {}, profile.controller)
    profile = dataclasses.replace(
        profile,
        arm_base_frame=base_frame,
        base_t_world=base_t_world,
        robot_params=robot_params,
        catch_frame=extra_frame_from_params(robot_params, catch_frame),
        catch_frame_name=catch_frame,
    )
    fk = CatchFrameFk(artifacts.urdf_text, facts.arm.joint_names, profile)
    catch_point_w = transform_point(fk(np.array(wait_pose)), fk.world_t_model)

    ball = ball_params_from_shape(
        ball_shape_from_config(args.ball_config),
        drag_coefficient=args.drag_coefficient,
        drag_coefficient_source=args.drag_coefficient_source,
        air_density_kg_m3=args.air_density_kg_m3,
        air_density_source=args.air_density_source,
    )

    # ── throws ──
    axes = {axis: getattr(args, axis) for axis in THROW_AXES}
    if args.throws_file is not None:
        if any(v is not None for v in axes.values()) or args.sample:
            raise SystemExit("--throws-file replaces the grid axes and --sample")
        throws, list_meta = read_throw_list(args.throws_file)
        design = {"throws_file": str(Path(args.throws_file).resolve()), "meta": list_meta}
        sample: list[dict] = []
    else:
        missing = [f"--{_GRID_KEYWORD[a].replace('_', '-')}" for a, v in axes.items() if not v]
        if missing:
            raise SystemExit(f"the throw grid needs every axis: missing {' '.join(missing)}")
        origin = (
            invert_transform(base_t_world)[:2, 3]
            if args.grid_origin == "arm_base"
            else catch_point_w[:2]
        )
        throws = grid_throws(axes, origin_xy_m=origin)
        sample = []
        if args.sample:
            if args.sample_seed is None:
                raise SystemExit("--sample needs --sample-seed")
            sample = lhs_throws(
                axis_ranges(axes),
                args.sample,
                args.sample_seed,
                origin_xy_m=origin,
                first_id=len(throws),
            )
        design = {
            "axes": axes,
            "grid_origin": args.grid_origin,
            "origin_xy_m": _floats(origin),
            "grid_throws": len(throws),
            "sample": {"n": len(sample), "seed": args.sample_seed} if sample else None,
        }
        throws = [*throws, *sample]
    list_meta_out = {"tool": "rtc_tools.analysis.catch_search_map", "design": _jsonable(design)}
    write_throw_list(out_dir / "throw_list.json", throws, list_meta_out)
    if sample:
        write_throw_list(out_dir / "throw_list_sample.json", sample, list_meta_out)

    # ── wakes ──
    dt_pred = args.prediction_dt_s
    if dt_pred is None:
        dt_pred = _decision(catching, "prediction.dt_expected")
        if math.isnan(dt_pred):
            raise SystemExit(
                "catching.prediction.dt_expected is not resolved — pass --prediction-dt-s"
            )
    timing = WakeTiming.from_seconds(
        detection_delay_s=args.detection_delay_s,
        vision_period_s=args.vision_period_s,
        prediction_dt_s=dt_pred,
        prediction_horizon_s=args.prediction_horizon_s,
        max_flight_s=args.horizon_s,
        floor_world_z_m=args.min_catch_height_m,
        max_step_s=args.step_s,
    )
    started = time.monotonic()
    wakes: list[Wake] = []
    flight_rows = {}
    for throw in throws:
        # One integration per throw gives both its wakes and its map axes;
        # the integrator's step is --step-s (timing.max_step_s).
        throw_wake_list, flight_rows[throw["throw_id"]] = throw_flight(
            throw, ball, timing, fk.model_t_world, catch_point_w, horizon_s=args.horizon_s
        )
        wakes += throw_wake_list
    wakes_path = out_dir / "wakes.csv"
    wakes_path.write_text("\n".join(wake_csv_lines(wakes)) + "\n")
    wakes_wall = time.monotonic() - started
    wakes_given = Counter(w.throw_id for w in wakes)

    params_path = out_dir / "catching_tree.yaml"
    params_path.write_text(yaml.safe_dump({"catching": catching}, sort_keys=False))
    judge = find_judge(args.judge, SEARCH_EXECUTABLE)

    summary: dict = {
        "tool": "rtc_tools.analysis.catch_search_map",
        "config_dir": str(config_dir.resolve()),
        "controller": profile.controller,
        "layers": [str(p) for p in layers],
        "tree_search_mode": _lookup(catching, "planner.search.mode"),
        "segment_mode": _lookup(catching, "planner.segment.mode"),
        "sub_model": sub_model,
        "catch_frame": catch_frame,
        "wait_pose": wait_pose,
        "wait_catch_point_world_m": _floats(catch_point_w),
        "frames": {
            "throws": THROW_LIST_FRAME,
            "wakes": "model_world",
            "arm_base_frame": base_frame,
            "model_t_world": fk.model_t_world.tolist(),
        },
        "ball": {
            "radius_m": ball.radius_m,
            "mass_kg": ball.mass_kg,
            "drag_coefficient": ball.drag_coefficient,
            "air_density_kg_m3": ball.air_density_kg_m3,
            "sources": dict(ball.sources),
        },
        "design": design,
        "wake_timing": {**dataclasses.asdict(timing), "release_ns": RELEASE_NS},
        "throws": len(throws),
        "wakes": len(wakes),
        "wakes_wall_s": wakes_wall,
        "searches": {},
    }

    verdicts_by_kind: dict[str, list[dict]] = {}
    rows_by_kind: dict[str, list[dict]] = {}
    for kind in args.search:
        try:
            binding = search_binding(catching, facts, kind)
        except BindingError as exc:
            raise SystemExit(f"binding ({kind}): {exc}") from exc
        binding_path = out_dir / f"binding_{kind}.yaml"
        binding_path.write_text(binding.yaml_text())
        (out_dir / f"binding_{kind}_sources.yaml").write_text(
            yaml.safe_dump(binding.sources, sort_keys=False, allow_unicode=True, width=100)
        )
        run = run_search_batch(
            judge,
            kind,
            model_config=artifacts.model_config_path,
            sub_model=sub_model,
            catch_frame=catch_frame,
            params=params_path,
            binding=binding_path,
            wakes=wakes_path,
            out=out_dir / f"search_{kind}.csv",
        )
        entry: dict = {"binding": binding.document, "wall_s": run.wall_s, "argv": run.argv}
        summary["searches"][kind] = entry
        if run.rows is None:
            entry["refused_configuration"] = run.refusal
            print(f"{kind}: {run.refusal}", file=sys.stderr)
            continue
        verdicts = throw_verdicts([t["throw_id"] for t in throws], run.rows, wakes_given)
        rows = [
            {
                **{k: v for k, v in throw.items() if k not in ("pos", "vel", "omega")},
                **flight_rows[throw["throw_id"]],
                **verdict,
            }
            for throw, verdict in zip(throws, verdicts, strict=True)
        ]
        _write_rows(out_dir / f"verdicts_{kind}.csv", rows)
        verdicts_by_kind[kind], rows_by_kind[kind] = verdicts, rows
        entry.update(map_summary(verdicts, rows, args.axis_bins))
        # FK of each accepted plan's posture against the plan's catch point:
        # the binary's joint order and frame, read back through this tool's.
        plans = accepted_plans(run.rows)
        nv = len(facts.arm.joint_names)
        entry["fk_q_star_to_p_c_m"] = distribution(
            float(
                np.linalg.norm(
                    fk(np.array([plan[f"q_star{i}"] for i in range(nv)]))
                    - np.array([plan[f"p_c_{a}"] for a in "xyz"])
                )
            )
            for plan in plans.values()
        )

    if all(kind in verdicts_by_kind for kind in SEARCH_KINDS):
        summary["grid_vs_nlp"] = search_disagreement(
            verdicts_by_kind["grid"], verdicts_by_kind["nlp"]
        )
    if args.gate_map is not None:
        gate_throws, gate_meta = load_gate_throws(args.gate_map)
        pose = gate_meta.get("wait_pose")
        if pose is not None and len(pose) == len(wait_pose):
            gate_meta["wait_pose_max_abs_diff_rad"] = float(
                np.max(np.abs(np.array(pose, dtype=float) - np.array(wait_pose)))
            )
        summary["gate_map"] = {**gate_meta, "searches": {}}
        for kind, rows in rows_by_kind.items():
            joined, join = gate_map_join(rows, gate_throws)
            _write_rows(out_dir / f"gate_join_{kind}.csv", joined)
            summary["gate_map"]["searches"][kind] = join
    if args.sim_trials:
        sim_rows = _read_rows(args.sim_trials)
        summary["sim"] = {"trials_csv": [str(p) for p in args.sim_trials], "searches": {}}
        for kind in rows_by_kind or [None]:
            joined, table = sim_table(sim_rows, verdicts_by_kind.get(kind))
            _write_rows(out_dir / f"sim_join_{kind or 'none'}.csv", joined)
            summary["sim"]["searches"][kind or "none"] = table

    summary_path = out_dir / "search_map_summary.json"
    summary_path.write_text(json.dumps(_jsonable(summary), indent=2, allow_nan=False) + "\n")
    for kind, entry in summary["searches"].items():
        if "refused_configuration" in entry:
            print(f"{kind}: refused its configuration ({entry['wall_s']:.2f} s)")
        else:
            print(
                f"{kind}: {entry['accepted']} / {entry['throws']} throws accepted, "
                f"{entry['wall_s']:.2f} s — reasons {entry['reasons']['throws']}"
            )
    print(f"summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
