#!/usr/bin/env python3
"""Pass-point throw design: launches that cross a point beside the waiting hand (L3 §4.1).

``catch_search_map`` judges throws; this designs them and says, without any
search, which of them no search can accept. A throw is given by where it is
released, where it passes and how steeply it leaves:

- ``distance_m`` / ``azimuth_deg`` — the release point's horizontal distance
  from the arm base axis and its azimuth about world +z (0 = world +x);
- ``release_height_m`` — its world height;
- ``pass_dx_m`` / ``pass_dy_m`` — the pass point's world x / y offset from the
  catch point of the wait pose; the pass point's height is that catch point's
  plus ``--pass-plane-offset-m``;
- ``elevation_deg`` — the launch elevation.

The launch SPEED is not an axis: the release point, the pass point and the
elevation fix the flight, so the speed is solved (:func:`solve_launch_speed`)
on the drag law the simulator runs. Two points and an elevation leave one
trajectory, so whether the ball is still rising at the pass point is a property
of the design point, not a choice — such a point has no descending crossing and
is labelled :data:`SCREEN_ASCENDING`.

A throw's ``throw_id`` is its index in the product of the six axes, in the
order of :data:`DESIGN_AXES` (the first axis outermost). A subset of the grid
and a refinement between its points keep those ids.

**What is recorded per throw** next to the six axes: the launch state, the
solved speed and the aim deviation (so the six axes of the kinematic map's
grid exist as well and a ``catch_gate_map`` join finds the throw), the apex and
its margin over the pass plane, the closest approach to the catch point (time,
speed, distance, descent angle, angle to the hand's approach axis), the last
instant the ball is inside the reach sphere, the lead that leaves the first
wake, and whether the pass point is close to the arm's links.

**Screens** — labels, never a filter of the file. A screened throw is one no
search can return a plan for, by a condition necessary for both searches:

- :data:`SCREEN_ASCENDING` — no speed puts a descending ball through the pass
  point;
- :data:`SCREEN_SHOOT_FAIL` — the speed solve did not converge, or the speed
  is above ``--speed-max-m-s``;
- :data:`SCREEN_REACH` — the flight never enters the reach sphere;
- :data:`SCREEN_TIME` — the last instant inside the sphere is less than the
  smallest lead a search asks for after the first wake.

The sphere and the lead floor come from the binary and the composed tree with
their sources (:func:`screen_constants`); nothing of the searches is copied.
The screens are checked against the searches themselves by running a map with
``catch_search_map --judge-screened``.

Throws, flights and every position here are in the SIM WORLD.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from collections import Counter
from collections.abc import Mapping, Sequence
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from rtc_tools.analysis import catch_search_map as csm
from rtc_tools.analysis.catchability_map import (
    CLOSEST_APPROACH_REFINE_SEGMENTS,
    GRAVITY_W_M_S2,
    BallParams,
    _hermite,
    _hermite_rate,
    ball_acceleration,
    invert_transform,
    transform_point,
)
from rtc_tools.analysis.catching_throw_list import write_throw_list

DESIGN_AXES = (
    "distance_m",
    "azimuth_deg",
    "release_height_m",
    "pass_dx_m",
    "pass_dy_m",
    "elevation_deg",
)
# The CLI argument of each axis.
_AXIS_ARGUMENT = {
    "distance_m": "distances-m",
    "azimuth_deg": "azimuths-deg",
    "release_height_m": "release-heights-m",
    "pass_dx_m": "pass-dx-m",
    "pass_dy_m": "pass-dy-m",
    "elevation_deg": "elevations-deg",
}
# Two axis values are the same value when they agree to this many decimals.
AXIS_DECIMALS = 9

SCREEN_KEY = csm.SCREEN_KEY
SCREEN_ASCENDING = "screen:shoot_ascending"
SCREEN_SHOOT_FAIL = "screen:shoot_fail"
SCREEN_REACH = "screen:reach"
SCREEN_TIME = "screen:time"
# Screens that leave no launch state: such a throw is in no throw list.
UNLAUNCHABLE_SCREENS = (SCREEN_ASCENDING, SCREEN_SHOOT_FAIL)

THROW_KIND = "pass_point"

# The axes a map summary groups a design's throws by: the six by their values,
# the derived ones in bins.
SUMMARY_DISCRETE_AXES = DESIGN_AXES
SUMMARY_BINNED_AXES = (
    "speed_m_s",
    "aim_deviation_deg",
    "apex_margin_m",
    "flight_time_s",
    "terminal_speed_m_s",
    "closest_distance_m",
    "arrival_descent_deg",
    "approach_angle_deg",
    "reach_last_s",
    "lead_available_s",
)

# One row of design.csv, in this order.
DESIGN_COLUMNS = (
    "throw_id",
    *DESIGN_AXES,
    "pos_x",
    "pos_y",
    "pos_z",
    "vel_x",
    "vel_y",
    "vel_z",
    "speed_m_s",
    "aim_deviation_deg",
    "aim_azimuth_deg",
    "pass_x",
    "pass_y",
    "pass_z",
    "pass_time_s",
    "pass_error_m",
    "pass_vz_m_s",
    "apex_height_m",
    "apex_margin_m",
    "apex_flag",
    "flight_time_s",
    "terminal_speed_m_s",
    "closest_distance_m",
    "arrival_descent_deg",
    "approach_angle_deg",
    "reach_min_distance_m",
    "reach_last_s",
    "lead_available_s",
    "body_distance_m",
    "body_flag",
    SCREEN_KEY,
)
_INT_COLUMNS = ("throw_id",)
_BOOL_COLUMNS = ("apex_flag", "body_flag")
_TEXT_COLUMNS = (SCREEN_KEY,)


# ── The grid ──────────────────────────────────────────────────────────────────


def axis_values(text: str) -> list[float]:
    """An axis from its argument: ``lo:hi:step`` (both ends included) or a list of values."""
    if ":" in text:
        parts = text.split(":")
        if len(parts) != 3:
            raise ValueError(f"'{text}': a range is lo:hi:step")
        lo, hi, step = (float(p) for p in parts)
        if not step > 0.0 or hi < lo:
            raise ValueError(f"'{text}': the step must be > 0 and hi >= lo")
        count = round((hi - lo) / step)
        if abs(lo + count * step - hi) > 10.0**-AXIS_DECIMALS:
            raise ValueError(f"'{text}': hi is not a whole number of steps from lo")
        return [round(lo + i * step, AXIS_DECIMALS) for i in range(count + 1)]
    return [float(v) for v in text.replace(",", " ").split()]


@dataclass(frozen=True)
class DesignGrid:
    """The product of the six design axes. A throw's id is its product index."""

    axes: Mapping[str, tuple[float, ...]]

    def __post_init__(self) -> None:
        missing = [axis for axis in DESIGN_AXES if axis not in self.axes]
        if missing:
            raise ValueError(f"the design grid needs every axis; missing {missing}")
        clean = {}
        for axis in DESIGN_AXES:
            values = tuple(round(float(v), AXIS_DECIMALS) for v in self.axes[axis])
            if not values or any(b <= a for a, b in zip(values, values[1:], strict=False)):
                raise ValueError(f"{axis}: the values must be ascending and distinct, not empty")
            clean[axis] = values
        object.__setattr__(self, "axes", clean)

    @property
    def shape(self) -> tuple[int, ...]:
        return tuple(len(self.axes[axis]) for axis in DESIGN_AXES)

    @property
    def size(self) -> int:
        return int(np.prod(self.shape))

    def position(self, axis: str, value: float) -> int:
        """Where ``value`` stands on ``axis``; ``ValueError`` if it is not one of its values."""
        key = round(float(value), AXIS_DECIMALS)
        try:
            return self.axes[axis].index(key)
        except ValueError:
            raise ValueError(f"{axis}: {value} is not a value of the design grid") from None

    def ids(self, subset: Mapping[str, Sequence[float]] | None = None) -> np.ndarray:
        """The ids of the product of ``subset`` (an axis it leaves out keeps all its values),
        ascending. Every value given must be a value of the grid."""
        subset = subset or {}
        unknown = [axis for axis in subset if axis not in DESIGN_AXES]
        if unknown:
            raise ValueError(f"not a design axis: {unknown}")
        picks = [
            np.array(
                sorted({self.position(axis, v) for v in subset[axis]})
                if axis in subset
                else range(len(self.axes[axis])),
                dtype=np.int64,
            )
            for axis in DESIGN_AXES
        ]
        mesh = np.meshgrid(*picks, indexing="ij")
        return np.sort(np.ravel_multi_index([m.ravel() for m in mesh], self.shape))

    def positions(self, ids: Sequence[int]) -> np.ndarray:
        """``(n, 6)`` axis positions of ``ids``."""
        ids = np.asarray(ids, dtype=np.int64)
        if ids.size and (ids.min() < 0 or ids.max() >= self.size):
            raise ValueError(f"a throw id is outside the grid's {self.size}")
        return np.stack(np.unravel_index(ids, self.shape), axis=1)

    def values(self, ids: Sequence[int]) -> dict[str, np.ndarray]:
        """The six axis values of ``ids``, one array per axis."""
        at = self.positions(ids)
        return {
            axis: np.asarray(self.axes[axis], dtype=float)[at[:, i]]
            for i, axis in enumerate(DESIGN_AXES)
        }


def refinement_ids(
    grid: DesignGrid,
    coarse: Mapping[str, Sequence[float]],
    accepted: Mapping[int, bool],
    axes: Sequence[str] | None = None,
) -> tuple[np.ndarray, dict[str, int]]:
    """The grid points BETWEEN coarse neighbours a search judged differently.

    Two points of the ``coarse`` subset are neighbours when they differ on one
    axis only, by one coarse step. For every such pair whose ``accepted`` (by
    throw id) differs, the grid points strictly between the two on that axis
    are returned — one step of refinement, never recursed. A pair with a point
    ``accepted`` does not hold is left out. ``axes`` limits the axes refined.
    Returns the ids, ascending, and the number of differing pairs per axis.
    """
    refine = tuple(DESIGN_AXES if axes is None else axes)
    unknown = [axis for axis in refine if axis not in DESIGN_AXES]
    if unknown:
        raise ValueError(f"not a design axis: {unknown}")
    picks = [
        sorted({grid.position(axis, v) for v in coarse[axis]})
        if axis in coarse
        else list(range(len(grid.axes[axis])))
        for axis in DESIGN_AXES
    ]
    mesh = np.meshgrid(*[np.array(p, dtype=np.int64) for p in picks], indexing="ij")
    at = np.stack([m.ravel() for m in mesh], axis=1)
    ids = np.ravel_multi_index(at.T, grid.shape)
    verdict = np.array([accepted.get(int(i), -1) for i in ids], dtype=np.int64)
    verdict = verdict.reshape([len(p) for p in picks])
    pairs = dict.fromkeys(DESIGN_AXES, 0)
    out: list[np.ndarray] = []
    for k, axis in enumerate(DESIGN_AXES):
        lo = np.take(verdict, range(len(picks[k]) - 1), axis=k)
        hi = np.take(verdict, range(1, len(picks[k])), axis=k)
        differ = (lo >= 0) & (hi >= 0) & (lo != hi)
        pairs[axis] = int(differ.sum())
        if axis not in refine or not differ.any():
            continue
        where = np.argwhere(differ)
        for row in where:
            fine = [picks[j][row[j]] for j in range(len(DESIGN_AXES))]
            first, last = picks[k][row[k]], picks[k][row[k] + 1]
            for between in range(first + 1, last):
                fine[k] = between
                out.append(np.ravel_multi_index(fine, grid.shape))
    found = np.unique(np.array(out, dtype=np.int64)) if out else np.empty(0, dtype=np.int64)
    return found, pairs


# ── Flights, many at once ─────────────────────────────────────────────────────


def _rk4_step(
    position: np.ndarray, velocity: np.ndarray, h: float, ball: BallParams, gravity: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    # integrate_flight's step on (n, 3) arrays; the force law is the one function.
    k1p, k1v = velocity, ball_acceleration(velocity, ball, gravity)
    k2p = velocity + 0.5 * h * k1v
    k2v = ball_acceleration(k2p, ball, gravity)
    k3p = velocity + 0.5 * h * k2v
    k3v = ball_acceleration(k3p, ball, gravity)
    k4p = velocity + h * k3v
    k4v = ball_acceleration(k4p, ball, gravity)
    return (
        position + (h / 6.0) * (k1p + 2.0 * k2p + 2.0 * k3p + k4p),
        velocity + (h / 6.0) * (k1v + 2.0 * k2v + 2.0 * k3v + k4v),
    )


@dataclass(frozen=True, eq=False)
class FlightBatch:
    """``n`` flights sampled together: ``position_m`` / ``velocity_m_s`` are ``(steps + 1, n, 3)``."""

    time_s: np.ndarray
    position_m: np.ndarray
    velocity_m_s: np.ndarray
    step_s: float


def integrate_flights(
    position_w: np.ndarray,
    velocity_w: np.ndarray,
    ball: BallParams,
    *,
    horizon_s: float,
    step_s: float,
    gravity_w: Sequence[float] = GRAVITY_W_M_S2,
) -> FlightBatch:
    """``catchability_map.integrate_flight`` for ``n`` launches at once.

    The same force law, the same fixed-step RK4 and the same number of steps
    (``ceil`` of the horizon over the step), on ``(n, 3)`` arrays: flight ``i``
    is ``integrate_flight`` of launch ``i``.
    """
    if not (math.isfinite(step_s) and step_s > 0.0):
        raise ValueError(f"step_s must be finite and > 0 (got {step_s!r})")
    if not (math.isfinite(horizon_s) and horizon_s > 0.0):
        raise ValueError(f"horizon_s must be finite and > 0 (got {horizon_s!r})")
    p = np.array(position_w, dtype=float).reshape(-1, 3)
    v = np.array(velocity_w, dtype=float).reshape(-1, 3)
    if p.shape != v.shape:
        raise ValueError("positions and velocities must be (n, 3) both")
    gravity = np.asarray(gravity_w, dtype=float).reshape(3)
    steps = int(math.ceil(horizon_s / step_s - 1e-12))
    h = float(step_s)
    positions = np.empty((steps + 1, p.shape[0], 3))
    velocities = np.empty((steps + 1, p.shape[0], 3))
    positions[0], velocities[0] = p, v
    for i in range(steps):
        positions[i + 1], velocities[i + 1] = _rk4_step(
            positions[i], velocities[i], h, ball, gravity
        )
    return FlightBatch(h * np.arange(steps + 1), positions, velocities, h)


def _segment(batch: FlightBatch, index: np.ndarray):
    """Each flight's Hermite segment starting at its own sample ``index`` (clipped)."""
    last = batch.time_s.size - 2
    i = np.clip(index, 0, max(last, 0))
    rows = np.arange(i.size)
    h = batch.step_s
    return (
        batch.position_m[i, rows],
        batch.velocity_m_s[i, rows] * h,
        batch.position_m[i + 1, rows],
        batch.velocity_m_s[i + 1, rows] * h,
    )


def closest_approaches(
    batch: FlightBatch, target_w: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(distance, time, velocity) of every flight's closest pass to ``target_w``.

    ``catchability_map.closest_approach`` on each flight of the batch: the
    closest sample, then the Hermite segments within
    ``CLOSEST_APPROACH_REFINE_SEGMENTS`` of it refined by the same
    golden-section search. ``target_w`` is one point or one per flight.
    """
    n = batch.position_m.shape[1]
    target = np.broadcast_to(np.asarray(target_w, dtype=float), (n, 3))
    sampled = np.linalg.norm(batch.position_m - target, axis=2)
    k = np.argmin(sampled, axis=0)
    rows = np.arange(n)
    best_d = sampled[k, rows].copy()
    best_t = batch.time_s[k].copy()
    best_v = batch.velocity_m_s[k, rows].copy()
    last = batch.time_s.size - 1
    if last == 0:
        return best_d, best_t, best_v
    phi = (math.sqrt(5.0) - 1.0) / 2.0
    h = batch.step_s
    reach = CLOSEST_APPROACH_REFINE_SEGMENTS

    def distance(seg, s):
        return np.linalg.norm(_hermite(seg, s[:, None]) - target, axis=1)

    for offset in range(-reach, reach):
        i = k + offset
        ok = (i >= 0) & (i + 1 <= last)
        if not ok.any():
            continue
        seg = _segment(batch, i)
        a, b = np.zeros(n), np.ones(n)
        c, d = b - phi * (b - a), a + phi * (b - a)
        for _ in range(40):  # phi**40 ≈ 4e-9 of the segment
            left = distance(seg, c) < distance(seg, d)
            b = np.where(left, d, b)
            a = np.where(left, a, c)
            c, d = b - phi * (b - a), a + phi * (b - a)
        for cand in (np.zeros(n), 0.5 * (a + b), np.ones(n)):
            dist = distance(seg, cand)
            better = ok & (dist < best_d)
            if better.any():
                best_d[better] = dist[better]
                best_t[better] = batch.time_s[np.clip(i, 0, last)][better] + cand[better] * h
                best_v[better] = (_hermite_rate(seg, cand[:, None]) / h)[better]
    return best_d, best_t, best_v


def apexes(batch: FlightBatch) -> tuple[np.ndarray, np.ndarray]:
    """(height, time) of every flight's highest point within the batch's horizon.

    A flight released level or downwards has it at release; one still rising at
    the end of the horizon has it there. Otherwise it is where the vertical
    velocity crosses zero inside a step (linear in the step), the height read
    off the step's Hermite segment.
    """
    vz = batch.velocity_m_s[:, :, 2]
    n = vz.shape[1]
    rows = np.arange(n)
    falling = ~(vz > 0.0)
    first = np.argmax(falling, axis=0)
    ever = falling.any(axis=0)
    height = batch.position_m[0, :, 2].copy()
    at = np.zeros(n)
    rising_out = ~ever
    height[rising_out] = batch.position_m[-1, rising_out, 2]
    at[rising_out] = batch.time_s[-1]
    inside = ever & (first > 0)
    if inside.any():
        i = first - 1
        seg = _segment(batch, i)
        up = vz[np.clip(i, 0, None), rows]
        down = vz[np.clip(first, 0, None), rows]
        with np.errstate(divide="ignore", invalid="ignore"):
            s = np.clip(np.where(up > down, up / (up - down), 0.0), 0.0, 1.0)
        z = _hermite(tuple(part[:, 2] for part in seg), s)
        height[inside] = z[inside]
        at[inside] = (batch.time_s[np.clip(i, 0, None)] + s * batch.step_s)[inside]
    return height, at


def last_inside(
    batch: FlightBatch, centre_w: np.ndarray, radius_m: float
) -> tuple[np.ndarray, np.ndarray]:
    """The last instant every flight is inside the sphere, and whether it still is at the
    end of the horizon.

    NaN for a flight with no sample inside. The exit is found inside its step
    by bisection on the step's Hermite segment. The flight model has no ground:
    a ball below the floor is inside the sphere all the same, as the prediction
    a search is handed is.
    """
    centre = np.asarray(centre_w, dtype=float).reshape(3)
    inside = np.linalg.norm(batch.position_m - centre, axis=2) <= radius_m
    n = inside.shape[1]
    last = inside.shape[0] - 1
    ever = inside.any(axis=0)
    k = last - np.argmax(inside[::-1], axis=0)
    at = np.full(n, math.nan)
    still = ever & (k == last)
    at[still] = batch.time_s[-1]
    leaves = ever & (k < last)
    if leaves.any():
        seg = _segment(batch, k)
        a, b = np.zeros(n), np.ones(n)
        for _ in range(50):
            mid = 0.5 * (a + b)
            within = np.linalg.norm(_hermite(seg, mid[:, None]) - centre, axis=1) <= radius_m
            a = np.where(within, mid, a)
            b = np.where(within, b, mid)
        at[leaves] = (batch.time_s[np.clip(k, 0, last)] + a * batch.step_s)[leaves]
    return at, still


def polyline_distance(points: np.ndarray, vertices: np.ndarray) -> np.ndarray:
    """Distance of every point to the polyline through ``vertices`` (in order)."""
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    v = np.asarray(vertices, dtype=float).reshape(-1, 3)
    if v.shape[0] == 0:
        return np.full(p.shape[0], math.inf)
    best = np.linalg.norm(p - v[0], axis=1)
    for start, end in zip(v[:-1], v[1:], strict=False):
        edge = end - start
        length2 = float(edge @ edge)
        s = np.clip(((p - start) @ edge) / length2, 0.0, 1.0) if length2 > 0.0 else 0.0
        best = np.minimum(best, np.linalg.norm(p - (start + np.outer(s, edge)), axis=1))
    return best


# ── The launch speed ──────────────────────────────────────────────────────────


def vacuum_launch_speed(
    distance_m: np.ndarray, rise_m: np.ndarray, elevation_rad: np.ndarray, gravity_m_s2: float
) -> np.ndarray:
    """The drag-free speed that crosses a point ``distance_m`` away and ``rise_m`` above the
    release at elevation θ: ``v0² = g d² / (2 cos²θ (d tanθ − Δz))``. NaN where the
    point is on or above the launch line (no speed reaches it)."""
    d = np.asarray(distance_m, dtype=float)
    over = d * np.tan(elevation_rad) - np.asarray(rise_m, dtype=float)
    with np.errstate(divide="ignore", invalid="ignore"):
        squared = gravity_m_s2 * d * d / (2.0 * np.cos(elevation_rad) ** 2 * over)
    return np.where((over > 0.0) & (d > 0.0), np.sqrt(np.abs(squared)), math.nan)


def vacuum_descending(
    distance_m: np.ndarray, rise_m: np.ndarray, elevation_rad: np.ndarray
) -> np.ndarray:
    """Whether the drag-free flight through the point is past its apex there:
    ``Δz < d tanθ / 2``."""
    return np.asarray(rise_m, dtype=float) < 0.5 * np.asarray(distance_m, dtype=float) * np.tan(
        elevation_rad
    )


def _crossing(
    launch: np.ndarray,
    heading: np.ndarray,
    distance: np.ndarray,
    elevation: np.ndarray,
    speed: np.ndarray,
    ball: BallParams,
    *,
    step_s: float,
    horizon_s: float,
    gravity: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Height, vertical velocity and time at which each flight has travelled ``distance``
    horizontally along ``heading``. NaN where it has not within ``horizon_s``."""
    n = launch.shape[0]
    cos_e, sin_e = np.cos(elevation), np.sin(elevation)
    p = launch.copy()
    v = np.empty((n, 3))
    v[:, :2] = (speed * cos_e)[:, None] * heading
    v[:, 2] = speed * sin_e
    z_at = np.full(n, math.nan)
    vz_at = np.full(n, math.nan)
    t_at = np.full(n, math.nan)
    pending = np.ones(n, dtype=bool)
    travelled = np.zeros(n)
    steps = int(math.ceil(horizon_s / step_s - 1e-12))
    h = float(step_s)
    for i in range(steps):
        p1, v1 = _rk4_step(p, v, h, ball, gravity)
        ahead = np.einsum("ij,ij->i", p1[:, :2] - launch[:, :2], heading)
        hit = pending & (ahead >= distance)
        if hit.any():
            x0, x1 = travelled[hit], ahead[hit]
            m0 = np.einsum("ij,ij->i", v[hit, :2], heading[hit]) * h
            m1 = np.einsum("ij,ij->i", v1[hit, :2], heading[hit]) * h
            want = distance[hit]
            with np.errstate(divide="ignore", invalid="ignore"):
                s = np.clip(np.where(x1 > x0, (want - x0) / (x1 - x0), 1.0), 0.0, 1.0)
            for _ in range(8):  # Newton on the monotone cubic
                rate = _hermite_rate((x0, m0, x1, m1), s)
                with np.errstate(divide="ignore", invalid="ignore"):
                    step = np.where(rate > 0.0, (_hermite((x0, m0, x1, m1), s) - want) / rate, 0.0)
                s = np.clip(s - step, 0.0, 1.0)
            z_seg = (p[hit, 2], v[hit, 2] * h, p1[hit, 2], v1[hit, 2] * h)
            z_at[hit] = _hermite(z_seg, s)
            vz_at[hit] = _hermite_rate(z_seg, s) / h
            t_at[hit] = (i + s) * h
            pending &= ~hit
            if not pending.any():
                break
        p, v, travelled = p1, v1, ahead
    return z_at, vz_at, t_at


@dataclass(frozen=True, eq=False)
class SpeedSolution:
    """What :func:`solve_launch_speed` found, one entry per throw.

    ``solved`` — a speed up to the search cap puts the flight through the pass
    point; ``speed_m_s`` and the ``pass_*`` values are NaN otherwise.
    ``pass_error_m`` is the flight's height error at the pass point's distance.
    """

    speed_m_s: np.ndarray
    solved: np.ndarray
    heading: np.ndarray
    pass_time_s: np.ndarray
    pass_vz_m_s: np.ndarray
    pass_error_m: np.ndarray
    vacuum_speed_m_s: np.ndarray
    evaluations: int


def solve_launch_speed(
    launch_w: np.ndarray,
    pass_w: np.ndarray,
    elevation_rad: np.ndarray,
    ball: BallParams,
    *,
    step_s: float,
    horizon_s: float,
    speed_cap_m_s: float,
    tolerance_m: float = 1e-9,
    max_iterations: int = 80,
    gravity_w: Sequence[float] = GRAVITY_W_M_S2,
) -> SpeedSolution:
    """The launch speed that takes each flight through its pass point, on the drag law.

    The launch leaves ``launch_w`` towards ``pass_w`` at ``elevation_rad``; its
    height where it has covered the horizontal distance to the pass point rises
    with the speed, so the speed is the root of one increasing function. The
    drag-free closed form (:func:`vacuum_launch_speed`) is a lower bound and
    the start; the bracket is widened upwards to ``speed_cap_m_s`` and closed by
    regula falsi (Illinois) until the height error is within ``tolerance_m``.
    A throw whose pass point no speed up to the cap reaches inside ``horizon_s``
    is not solved. Gravity must act along −z.
    """
    launch = np.asarray(launch_w, dtype=float).reshape(-1, 3)
    target = np.asarray(pass_w, dtype=float).reshape(-1, 3)
    elevation = np.asarray(elevation_rad, dtype=float).reshape(-1)
    gravity = np.asarray(gravity_w, dtype=float).reshape(3)
    if gravity[0] != 0.0 or gravity[1] != 0.0 or not gravity[2] < 0.0:
        raise ValueError("the speed solve needs gravity along -z")
    n = launch.shape[0]
    delta = target - launch
    distance = np.hypot(delta[:, 0], delta[:, 1])
    with np.errstate(divide="ignore", invalid="ignore"):
        heading = np.where(distance[:, None] > 0.0, delta[:, :2] / distance[:, None], 0.0)
    vacuum = vacuum_launch_speed(distance, delta[:, 2], elevation, -gravity[2])

    speed = np.full(n, math.nan)
    error = np.full(n, math.nan)
    pass_vz = np.full(n, math.nan)
    pass_t = np.full(n, math.nan)
    evaluations = 0

    def height_error(index: np.ndarray, at: np.ndarray):
        nonlocal evaluations
        evaluations += 1
        z, vz, t = _crossing(
            launch[index],
            heading[index],
            distance[index],
            elevation[index],
            at,
            ball,
            step_s=step_s,
            horizon_s=horizon_s,
            gravity=gravity,
        )
        # A flight that has not come that far within the horizon is too slow.
        return np.where(np.isfinite(z), z - target[index, 2], -math.inf), vz, t

    def keep(index: np.ndarray, at: np.ndarray, f: np.ndarray, vz: np.ndarray, t: np.ndarray):
        better = np.isfinite(f) & ~(np.abs(f) >= np.abs(np.nan_to_num(error[index], nan=math.inf)))
        hit = index[better]
        speed[hit], error[hit], pass_vz[hit], pass_t[hit] = (
            at[better],
            f[better],
            vz[better],
            t[better],
        )

    active = np.flatnonzero(np.isfinite(vacuum) & (vacuum <= speed_cap_m_s))
    lo = vacuum[active]
    f_lo, vz, t = height_error(active, lo)
    keep(active, lo, f_lo, vz, t)
    # Too high at the start (rounding, or no drag): step down until it is not.
    for _ in range(40):
        high = f_lo > tolerance_m
        if not high.any():
            break
        lo = np.where(high, lo * 0.98, lo)
        f_new, vz, t = height_error(active[high], lo[high])
        keep(active[high], lo[high], f_new, vz, t)
        f_lo[high] = f_new
    hi = np.minimum(lo * 1.25, speed_cap_m_s)
    f_hi = np.full(active.size, math.nan)
    done = np.abs(f_lo) <= tolerance_m
    need = ~done
    for _ in range(40):
        if not need.any():
            break
        f_new, vz, t = height_error(active[need], hi[need])
        keep(active[need], hi[need], f_new, vz, t)
        f_hi[need] = f_new
        below = need & (f_hi < 0.0) & (hi < speed_cap_m_s)
        capped = need & (f_hi < 0.0) & ~(hi < speed_cap_m_s)
        done |= capped  # no root up to the cap
        lo = np.where(below, hi, lo)
        f_lo = np.where(below, f_hi, f_lo)
        hi = np.where(below, np.minimum(hi * 1.25, speed_cap_m_s), hi)
        need = below
    bracketed = ~done & np.isfinite(f_hi) & (f_hi >= 0.0)
    done |= bracketed & (np.abs(f_hi) <= tolerance_m)
    side = np.zeros(active.size, dtype=np.int8)
    for _ in range(max_iterations):
        work = bracketed & ~done
        if not work.any():
            break
        a, b, fa, fb = lo[work], hi[work], f_lo[work], f_hi[work]
        with np.errstate(divide="ignore", invalid="ignore"):
            x = np.where(np.isfinite(fa), (a * fb - b * fa) / (fb - fa), 0.5 * (a + b))
        x = np.where((x > a) & (x < b), x, 0.5 * (a + b))
        fx, vz, t = height_error(active[work], x)
        keep(active[work], x, fx, vz, t)
        low = fx < 0.0
        was = side[work]
        fa_next = np.where(low, fx, np.where(was == 1, 0.5 * fa, fa))
        fb_next = np.where(low, np.where(was == -1, 0.5 * fb, fb), fx)
        lo[work] = np.where(low, x, a)
        hi[work] = np.where(low, b, x)
        f_lo[work], f_hi[work] = fa_next, fb_next
        side[work] = np.where(low, -1, 1)
        closed = (np.abs(fx) <= tolerance_m) | ((hi[work] - lo[work]) <= 4e-16 * hi[work])
        done[np.flatnonzero(work)[closed]] = True
    solved = np.abs(error) <= tolerance_m
    for values in (speed, pass_vz, pass_t, error):
        values[~solved] = math.nan
    return SpeedSolution(speed, solved, heading, pass_t, pass_vz, error, vacuum, evaluations)


# ── One design ────────────────────────────────────────────────────────────────


@dataclass(frozen=True, eq=False)
class ScreenConstants:
    """What the screens and the recorded axes are measured against (SIM WORLD)."""

    catch_point_w: np.ndarray
    approach_axis_w: np.ndarray
    reach_centre_w: np.ndarray
    reach_radius_m: float  # the sphere a screen tests: bound + tolerance + entrance offset
    detection_delay_s: float
    lead_floor_s: float  # NaN: no time screen
    body_vertices_w: np.ndarray
    body_flag_radius_m: float
    apex_margin_flag_m: float


def design_throws(
    throw_ids: np.ndarray,
    values: Mapping[str, np.ndarray],
    *,
    origin_xy_m: Sequence[float],
    pass_plane_z_m: float,
    constants: ScreenConstants,
    ball: BallParams,
    speed_max_m_s: float,
    speed_cap_m_s: float,
    horizon_s: float,
    step_s: float,
) -> dict[str, np.ndarray]:
    """Design the throws at ``values`` (one array per :data:`DESIGN_AXES`): the launch,
    the flight's recorded axes and the screen label. One array per
    :data:`DESIGN_COLUMNS`. ``speed_cap_m_s`` bounds the speed SEARCH and is at
    least ``speed_max_m_s``, above which a solved throw is not launched."""
    ids = np.asarray(throw_ids, dtype=np.int64)
    n = ids.size
    az = np.radians(np.asarray(values["azimuth_deg"], dtype=float))
    elevation = np.radians(np.asarray(values["elevation_deg"], dtype=float))
    outward = np.stack([np.cos(az), np.sin(az)], axis=1)
    origin = np.asarray(origin_xy_m, dtype=float).reshape(2)
    launch = np.empty((n, 3))
    launch[:, :2] = origin + np.asarray(values["distance_m"], dtype=float)[:, None] * outward
    launch[:, 2] = values["release_height_m"]
    target = np.empty((n, 3))
    target[:, 0] = constants.catch_point_w[0] + np.asarray(values["pass_dx_m"], dtype=float)
    target[:, 1] = constants.catch_point_w[1] + np.asarray(values["pass_dy_m"], dtype=float)
    target[:, 2] = pass_plane_z_m

    found = solve_launch_speed(
        launch,
        target,
        elevation,
        ball,
        step_s=step_s,
        horizon_s=horizon_s,
        speed_cap_m_s=max(speed_cap_m_s, speed_max_m_s),
    )
    distance = np.hypot(*(target - launch)[:, :2].T)
    rise = target[:, 2] - launch[:, 2]
    descending = found.solved & (found.pass_vz_m_s < 0.0)
    ascending = (found.solved & ~descending) | (
        ~found.solved & ~vacuum_descending(distance, rise, elevation)
    )
    launched = descending & (found.speed_m_s <= speed_max_m_s)
    screen = np.full(n, "", dtype=object)
    screen[~launched] = SCREEN_SHOOT_FAIL
    screen[ascending] = SCREEN_ASCENDING

    out: dict[str, np.ndarray] = {"throw_id": ids}
    for axis in DESIGN_AXES:
        out[axis] = np.asarray(values[axis], dtype=float)
    velocity = np.full((n, 3), math.nan)
    velocity[launched, :2] = (found.speed_m_s[launched] * np.cos(elevation[launched]))[
        :, None
    ] * found.heading[launched]
    velocity[launched, 2] = found.speed_m_s[launched] * np.sin(elevation[launched])
    for i, name in enumerate("xyz"):
        out[f"pos_{name}"] = launch[:, i]
        out[f"vel_{name}"] = velocity[:, i]
        out[f"pass_{name}"] = target[:, i]
    # The kinematic map's aim deviation: the heading's angle from the direction
    # back to the base axis, about world +z.
    inward = -outward
    aim = np.degrees(
        np.arctan2(
            inward[:, 0] * found.heading[:, 1] - inward[:, 1] * found.heading[:, 0],
            np.einsum("ij,ij->i", inward, found.heading),
        )
    )
    out["speed_m_s"] = np.where(launched, found.speed_m_s, math.nan)
    out["aim_deviation_deg"] = aim
    out["aim_azimuth_deg"] = np.degrees(np.arctan2(found.heading[:, 1], found.heading[:, 0]))
    out["pass_time_s"] = np.where(launched, found.pass_time_s, math.nan)
    out["pass_error_m"] = np.where(launched, found.pass_error_m, math.nan)
    out["pass_vz_m_s"] = found.pass_vz_m_s
    flight_columns = (
        "apex_height_m",
        "apex_margin_m",
        "flight_time_s",
        "terminal_speed_m_s",
        "closest_distance_m",
        "arrival_descent_deg",
        "approach_angle_deg",
        "reach_min_distance_m",
        "reach_last_s",
        "lead_available_s",
    )
    for name in flight_columns:
        out[name] = np.full(n, math.nan)
    out["apex_flag"] = np.zeros(n, dtype=bool)

    if launched.any():
        batch = integrate_flights(
            launch[launched], velocity[launched], ball, horizon_s=horizon_s, step_s=step_s
        )
        apex, _ = apexes(batch)
        closest_d, closest_t, closest_v = closest_approaches(batch, constants.catch_point_w)
        reach_d, reach_t, _ = closest_approaches(batch, constants.reach_centre_w)
        last, _ = last_inside(batch, constants.reach_centre_w, constants.reach_radius_m)
        # A flight that only grazes the sphere between two samples is inside
        # it at its closest pass.
        graze = np.isnan(last) & (reach_d <= constants.reach_radius_m)
        last[graze] = reach_t[graze]
        arrival_speed = np.linalg.norm(closest_v, axis=1)
        with np.errstate(divide="ignore", invalid="ignore"):
            unit = closest_v / arrival_speed[:, None]
        against = np.clip(-(unit @ constants.approach_axis_w), -1.0, 1.0)
        margin = apex - pass_plane_z_m
        fill = {
            "apex_height_m": apex,
            "apex_margin_m": margin,
            "flight_time_s": closest_t,
            "terminal_speed_m_s": arrival_speed,
            "closest_distance_m": closest_d,
            "arrival_descent_deg": np.degrees(
                np.arctan2(-closest_v[:, 2], np.hypot(closest_v[:, 0], closest_v[:, 1]))
            ),
            "approach_angle_deg": np.degrees(np.arccos(against)),
            "reach_min_distance_m": reach_d,
            "reach_last_s": last,
            "lead_available_s": last - constants.detection_delay_s,
        }
        for name, column in fill.items():
            out[name][launched] = column
        out["apex_flag"][launched] = margin < constants.apex_margin_flag_m
        where = np.flatnonzero(launched)
        outside = np.isnan(last)
        screen[where[outside]] = SCREEN_REACH
        if math.isfinite(constants.lead_floor_s):
            short = ~outside & (last - constants.detection_delay_s < constants.lead_floor_s)
            screen[where[short]] = SCREEN_TIME
    body = polyline_distance(target, constants.body_vertices_w)
    out["body_distance_m"] = body
    out["body_flag"] = body < constants.body_flag_radius_m
    out[SCREEN_KEY] = screen
    return out


def _design_chunk(job: tuple) -> dict[str, np.ndarray]:
    ids, values, kwargs = job
    return design_throws(ids, values, **kwargs)


def design_table(
    throw_ids: np.ndarray,
    values: Mapping[str, np.ndarray],
    *,
    chunk: int,
    jobs: int,
    **kwargs,
) -> dict[str, np.ndarray]:
    """:func:`design_throws` over ``chunk`` throws at a time, on ``jobs`` processes. The
    result does not depend on either: a throw's row is a function of its own values."""
    ids = np.asarray(throw_ids, dtype=np.int64)
    if chunk < 1 or jobs < 1:
        raise ValueError("chunk and jobs must be >= 1")
    pieces = [
        (
            ids[i : i + chunk],
            {a: np.asarray(values[a])[i : i + chunk] for a in DESIGN_AXES},
            kwargs,
        )
        for i in range(0, ids.size, chunk)
    ]
    if jobs == 1 or len(pieces) <= 1:
        parts = [_design_chunk(piece) for piece in pieces]
    else:
        with ProcessPoolExecutor(max_workers=jobs, mp_context=csm.WORKER_CONTEXT) as pool:
            parts = list(pool.map(_design_chunk, pieces))
    if not parts:
        return {name: np.empty(0) for name in DESIGN_COLUMNS}
    return {name: np.concatenate([part[name] for part in parts]) for name in DESIGN_COLUMNS}


# ── Files ─────────────────────────────────────────────────────────────────────


def _cell(name: str, value) -> str:
    if name in _TEXT_COLUMNS:
        return str(value)
    if name in _INT_COLUMNS:
        return str(int(value))
    if name in _BOOL_COLUMNS:
        return "1" if value else "0"
    return "" if value != value else repr(float(value))


def write_design_csv(path: Path, table: Mapping[str, np.ndarray]) -> None:
    """One row per throw, :data:`DESIGN_COLUMNS` in order; a double at round-trip
    precision, NaN as an empty cell."""
    with Path(path).open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(DESIGN_COLUMNS)
        columns = [table[name] for name in DESIGN_COLUMNS]
        for row in zip(*columns, strict=True):
            writer.writerow(
                [_cell(name, value) for name, value in zip(DESIGN_COLUMNS, row, strict=True)]
            )


def read_design_csv(path: Path) -> list[dict]:
    """The rows of a design CSV, typed: NaN for an empty number cell."""
    rows = []
    with Path(path).open(newline="") as handle:
        for raw in csv.DictReader(handle):
            row: dict = {}
            for name, text in raw.items():
                if name in _TEXT_COLUMNS:
                    row[name] = text
                elif name in _INT_COLUMNS:
                    row[name] = int(text)
                elif name in _BOOL_COLUMNS:
                    row[name] = text == "1"
                else:
                    row[name] = float(text) if text != "" else math.nan
            rows.append(row)
    return rows


_LIST_DROPPED = ("pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z")


def _list_entry(row: Mapping) -> dict:
    return {
        k: (None if isinstance(v, float) and not math.isfinite(v) else v)
        for k, v in row.items()
        if k not in _LIST_DROPPED
    }


def throw_list_of(rows: Sequence[Mapping]) -> tuple[list[dict], list[dict]]:
    """Design rows as throw-list entries, and those that have no launch.

    A launchable row becomes an entry carrying every design column (a
    non-finite number as null) next to ``pos`` / ``vel``; its ``screen`` label
    goes with it. A row screened before any launch exists
    (:data:`UNLAUNCHABLE_SCREENS`) cannot be in a throw list: it is returned
    apart, for the list's ``meta``, so a map still counts it.
    """
    throws, unlaunchable = [], []
    for row in rows:
        entry = _list_entry(row)
        if row[SCREEN_KEY] in UNLAUNCHABLE_SCREENS:
            unlaunchable.append(entry)
            continue
        throws.append(
            {
                "throw_id": int(row["throw_id"]),
                "kind": THROW_KIND,
                "pos": tuple(float(row[f"pos_{a}"]) for a in "xyz"),
                "vel": tuple(float(row[f"vel_{a}"]) for a in "xyz"),
                "omega": (0.0, 0.0, 0.0),
                **{k: v for k, v in entry.items() if k != "throw_id"},
            }
        )
    return throws, unlaunchable


def screen_summary(rows: Sequence[Mapping]) -> dict:
    """Counts of the screen labels: overall, per value of every design axis, and the
    cells (distance × release height × elevation) that hold an ascending throw."""
    labels = Counter(row[SCREEN_KEY] or "none" for row in rows)
    by_axis: dict[str, dict] = {}
    for axis in DESIGN_AXES:
        groups: dict[float, Counter] = {}
        for row in rows:
            groups.setdefault(row[axis], Counter())[row[SCREEN_KEY] or "none"] += 1
        by_axis[axis] = {
            repr(value): dict(sorted(c.items())) for value, c in sorted(groups.items())
        }
    cells: dict[tuple, list[int]] = {}
    for row in rows:
        key = (row["distance_m"], row["release_height_m"], row["elevation_deg"])
        cell = cells.setdefault(key, [0, 0])
        cell[0] += 1
        cell[1] += row[SCREEN_KEY] == SCREEN_ASCENDING
    return {
        "throws": len(rows),
        "labels": dict(sorted(labels.items())),
        "apex_flag": sum(1 for row in rows if row["apex_flag"]),
        "body_flag": sum(1 for row in rows if row["body_flag"]),
        "by_axis": by_axis,
        "ascending_cells": [
            {
                "distance_m": key[0],
                "release_height_m": key[1],
                "elevation_deg": key[2],
                "ascending": count[1],
                "throws": count[0],
            }
            for key, count in sorted(cells.items())
            if count[1]
        ],
    }


# ── CLI ───────────────────────────────────────────────────────────────────────


def _load_grid(design_dir: Path) -> tuple[DesignGrid, dict]:
    meta = json.loads((Path(design_dir) / "design_meta.json").read_text())
    return DesignGrid({axis: tuple(meta["grid"]["axes"][axis]) for axis in DESIGN_AXES}), meta


def _subset(texts: Sequence[str] | None) -> dict[str, list[float]]:
    out: dict[str, list[float]] = {}
    for text in texts or []:
        axis, _, values = text.partition("=")
        if axis not in DESIGN_AXES or not values:
            raise SystemExit(f"--subset wants <axis>=<values> with an axis of {DESIGN_AXES}")
        out[axis] = axis_values(values)
    return out


def _cmd_design(args: argparse.Namespace) -> int:
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    setup = csm.load_setup(args, out_dir, ())
    catching = setup.catching
    grid = DesignGrid({axis: tuple(getattr(args, axis)) for axis in DESIGN_AXES})
    judge = csm.find_judge(args.judge, csm.SEARCH_EXECUTABLE)
    params_path = out_dir / "catching_tree.yaml"
    csm.write_catching_tree(params_path, catching)
    bound = csm.reach_bound(
        judge,
        model_config=setup.artifacts.model_config_path,
        sub_model=setup.sub_model,
        catch_frame=setup.catch_frame,
        params=params_path,
    )
    reach = csm.screen_reach(catching, bound)
    floors = csm.lead_floors(catching)
    lead_floor = csm.common_lead_floor(floors)
    world_t_model = setup.fk.world_t_model
    _, rotation_w = setup.fk.pose_world(np.array(setup.wait_pose))
    body_w = transform_point(setup.fk.joint_origins(np.array(setup.wait_pose)), world_t_model)
    constants = ScreenConstants(
        catch_point_w=setup.catch_point_w,
        approach_axis_w=rotation_w[:, 2],
        reach_centre_w=transform_point(np.array(bound["centre"], dtype=float), world_t_model),
        reach_radius_m=reach["screen_radius_m"],
        detection_delay_s=args.detection_delay_s,
        lead_floor_s=lead_floor,
        body_vertices_w=body_w,
        body_flag_radius_m=args.body_flag_radius_m,
        apex_margin_flag_m=args.apex_margin_flag_m,
    )
    origin = invert_transform(setup.base_t_world)[:2, 3]
    pass_plane = float(setup.catch_point_w[2]) + args.pass_plane_offset_m
    ids = grid.ids()
    started = time.monotonic()
    table = design_table(
        ids,
        grid.values(ids),
        chunk=args.chunk,
        jobs=args.jobs,
        origin_xy_m=origin,
        pass_plane_z_m=pass_plane,
        constants=constants,
        ball=setup.ball,
        speed_max_m_s=args.speed_max_m_s,
        speed_cap_m_s=args.speed_search_cap_m_s,
        horizon_s=args.horizon_s,
        step_s=args.step_s,
    )
    wall = time.monotonic() - started
    write_design_csv(out_dir / "design.csv", table)
    rows = read_design_csv(out_dir / "design.csv")
    summary = screen_summary(rows)
    meta = {
        "tool": "rtc_tools.analysis.catch_throw_design",
        "grid": {"axes": {a: list(grid.axes[a]) for a in DESIGN_AXES}, "throws": grid.size},
        "frame": "sim_world",
        "origin_xy_m": [float(v) for v in origin],
        "wait_pose": setup.wait_pose,
        "wait_catch_point_world_m": [float(v) for v in setup.catch_point_w],
        "approach_axis_world": [float(v) for v in constants.approach_axis_w],
        "pass_plane": {
            "height_world_m": pass_plane,
            "offset_m": args.pass_plane_offset_m,
            "source": "FK of planner.wait_pose (catch frame origin) + --pass-plane-offset-m",
        },
        "speed": {"max_m_s": args.speed_max_m_s, "search_cap_m_s": args.speed_search_cap_m_s},
        "flight": {"horizon_s": args.horizon_s, "step_s": args.step_s},
        "ball": {
            "radius_m": setup.ball.radius_m,
            "mass_kg": setup.ball.mass_kg,
            "drag_coefficient": setup.ball.drag_coefficient,
            "air_density_kg_m3": setup.ball.air_density_kg_m3,
            "sources": dict(setup.ball.sources),
        },
        "reach": {
            **reach,
            "bound": dict(bound),
            "centre_world_m": [float(v) for v in constants.reach_centre_w],
        },
        "lead": {
            "detection_delay_s": args.detection_delay_s,
            "per_search": floors,
            "floor_s": lead_floor,
            "rule": "screen:time when reach_last_s − detection_delay_s < floor_s, the smallest "
            "lead floor of the searches in the tree — one threshold for both, so both "
            "searches are handed the same throws",
        },
        "body": {
            "flag_radius_m": args.body_flag_radius_m,
            "vertices_world_m": body_w.tolist(),
            "rule": "the pass point's distance to the polyline through the arm joints' origins "
            "at the wait pose; a flag, not a screen",
        },
        "apex_margin_flag_m": args.apex_margin_flag_m,
        "axes": {"discrete": list(SUMMARY_DISCRETE_AXES), "binned": list(SUMMARY_BINNED_AXES)},
        "provenance": csm.provenance(
            setup, judge=judge, params_path=params_path, bound=bound, estimator=None
        ),
        "wall_s": wall,
        "screens": summary,
    }
    (out_dir / "design_meta.json").write_text(
        json.dumps(csm._jsonable(meta), indent=2, allow_nan=False) + "\n"
    )
    print(f"{grid.size} throws designed in {wall:.1f} s — {summary['labels']}")
    print(f"design: {out_dir / 'design.csv'}")
    return 0


def _read_verdicts(paths: Sequence[Path]) -> dict[int, bool]:
    accepted: dict[int, bool] = {}
    for path in paths:
        with Path(path).open(newline="") as handle:
            for row in csv.DictReader(handle):
                accepted[int(row["throw_id"])] = row["accepted"] in ("True", "1", "true")
    return accepted


def _cmd_select(args: argparse.Namespace) -> int:
    design_dir = Path(args.design_dir)
    grid, meta = _load_grid(design_dir)
    subset = _subset(args.subset)
    selection: dict = {"design_dir": str(design_dir.resolve()), "subset": subset}
    if args.refine_from:
        wanted: set[int] = set()
        selection["refine"] = {}
        for path in args.refine_from:
            found, pairs = refinement_ids(
                grid, subset, _read_verdicts([path]), axes=args.refine_axes
            )
            selection["refine"][str(path)] = {"points": int(found.size), "pairs_by_axis": pairs}
            wanted.update(int(i) for i in found)
        selection["refine_axes"] = args.refine_axes
        ids = np.array(sorted(wanted), dtype=np.int64)
    elif args.ids_file:
        ids = np.array(sorted({int(v) for v in json.loads(Path(args.ids_file).read_text())}))
        selection["ids_file"] = str(Path(args.ids_file).resolve())
    else:
        ids = grid.ids(subset)
    selection["throws"] = int(ids.size)
    if args.max_throws is not None and ids.size > args.max_throws:
        print(json.dumps(selection, indent=2))
        raise SystemExit(f"{ids.size} throws selected, over --max-throws {args.max_throws}")
    keep = {int(i) for i in ids}
    rows = [row for row in read_design_csv(design_dir / "design.csv") if row["throw_id"] in keep]
    if len(rows) != len(keep):
        raise SystemExit("the selection names throws the design does not hold")
    throws, unlaunchable = throw_list_of(rows)
    labels = Counter(row[SCREEN_KEY] or "none" for row in rows)
    selection["labels"] = dict(sorted(labels.items()))
    list_meta = {
        "tool": "rtc_tools.analysis.catch_throw_design",
        "selection": selection,
        "axes": meta["axes"],
        "unlaunchable": unlaunchable,
        "design": {k: meta[k] for k in ("grid", "pass_plane", "speed", "reach", "lead", "ball")},
        "design_provenance": meta.get("provenance"),
    }
    if not throws:
        raise SystemExit(f"the selection holds no launchable throw ({selection['labels']})")
    write_throw_list(Path(args.out), throws, csm._jsonable(list_meta))
    print(json.dumps(selection, indent=2))
    print(f"throw list: {args.out} ({len(throws)} launches, {len(unlaunchable)} without one)")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="command", required=True)

    design = sub.add_parser("design", help="design the whole grid and label the screens")
    csm.add_setup_arguments(design)
    design.add_argument("--out-dir", type=Path, required=True)
    design.add_argument(
        "--judge", type=Path, help=f"path to {csm.SEARCH_EXECUTABLE} (default: ament)"
    )
    for axis in DESIGN_AXES:
        design.add_argument(
            f"--{_AXIS_ARGUMENT[axis]}",
            type=axis_values,
            dest=axis,
            required=True,
            help="values, or lo:hi:step",
        )
    design.add_argument(
        "--pass-plane-offset-m",
        type=float,
        default=0.0,
        help="the pass point's height above the catch point of the wait pose",
    )
    design.add_argument(
        "--speed-max-m-s", type=float, required=True, help="a faster launch is not thrown"
    )
    design.add_argument(
        "--speed-search-cap-m-s",
        type=float,
        default=48.0,
        help="how far the speed solve looks (above --speed-max-m-s, to tell a throw that "
        "is only too fast from one that has no descending crossing)",
    )
    design.add_argument("--detection-delay-s", type=float, required=True, help="release → wake 0")
    design.add_argument("--horizon-s", type=float, default=2.5, help="flight horizon")
    design.add_argument("--step-s", type=float, default=0.002, help="RK4 step")
    design.add_argument("--apex-margin-flag-m", type=float, default=0.05)
    design.add_argument("--body-flag-radius-m", type=float, default=0.15)
    design.add_argument("--chunk", type=int, default=2048, help="throws integrated together")
    design.add_argument("--jobs", type=int, default=1, help="processes")
    design.set_defaults(run=_cmd_design)

    select = sub.add_parser("select", help="write a throw list of part of a design")
    select.add_argument("--design-dir", type=Path, required=True)
    select.add_argument("--out", type=Path, required=True, help="the throw list to write")
    select.add_argument(
        "--subset",
        action="append",
        help="<axis>=<values>: the values of that axis to keep (repeatable; an axis not "
        "named keeps all of its values)",
    )
    select.add_argument("--ids-file", type=Path, help="a JSON list of throw ids to keep")
    select.add_argument(
        "--refine-from",
        type=Path,
        nargs="+",
        help="verdicts CSV(s) of maps over --subset: select the grid points between "
        "neighbours of the subset a map judged differently (the union over the files)",
    )
    select.add_argument("--refine-axes", nargs="+", choices=DESIGN_AXES, help="refine these only")
    select.add_argument("--max-throws", type=int, help="refuse a larger selection")
    select.set_defaults(run=_cmd_select)

    args = ap.parse_args(argv)
    return args.run(args)


if __name__ == "__main__":
    sys.exit(main())
