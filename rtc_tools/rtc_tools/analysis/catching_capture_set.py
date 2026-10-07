"""Capture-set identification: from fly-in verdicts to the planner's numbers.

dynamic_catching E1-F15 (#741). The docking-style catch planner (reference
``docs/dynamic_catching/ref/ball_catching_inverse_dynamics_mpc.md`` §6, §8.4,
§17.6 – §17.8) is told where the ball may cross the hand's entrance plane, how
fast, and when the hand may be closed around it:

* the entrance plane offset ``s_ent`` and the no-contact condition above it,
* the lateral set ``C_perp`` — a convex polygon of at most eight faces with
  unit normals, in ball-CENTRE coordinates,
* the velocity set ``V_cap`` — closing speed ``c`` in ``[c_min, c_cap_max]`` and
  lateral speed at most ``v_perp_max``,
* the closure window ``[delta_lo, delta_hi]`` after the crossing.

Those are properties of a hand, a posture pair and a ball, measured by flying
the ball into the hand in simulation. This module is the arithmetic between
the measurement and the numbers — pure functions over arrays, no simulator, no
ROS, and nothing that names a robot, a hand or a joint. The driver that knows
those lives with the robot configs.

Coordinates (the planner's, ``mpc_docking_relative_state.hpp``). The capture
frame has its origin where the caught ball rests; ``+e3`` is the approach axis
and the ball travels toward ``-e3``. ``s`` is the ball centre's coordinate along
``e3``, ``rho`` its two lateral coordinates, ``c > 0`` the closing speed.

TWO TIME AXES for the closure instant, and why both exist. Whether a ball is
held depends on when the hand closes relative to the ball's arrival at the
ORIGIN — the hand does not know where an entrance plane was drawn. That is
``delta_o``: (instant the closure is complete) − (instant the ball centre
would reach ``s = 0`` in free flight). The planner's window is on the other
axis, relative to the crossing of ``s = s_ent``:

    delta = delta_o + s_ent / c

The planner takes ONE window for every closing speed it allows, so a box
``[c_lo, c_hi] x [delta_o_lo, delta_o_hi]`` of held conditions gives

    delta_lo = delta_o_lo + s_ent / c_lo        delta_hi = delta_o_hi + s_ent / c_hi

(for ``s_ent >= 0``) — narrower than the box by ``s_ent (1/c_lo − 1/c_hi)``. A
window that comes out empty is a result, not an error: :func:`entrance_window`
returns it as it is.

CELLS, not points. A verdict belongs to a cell of a uniform lattice — the
trials of a cell are spread over it — so a held cell stands for its whole
extent and every interval this module reports ends on cell EDGES.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass

import numpy as np

# Area below which a clipped polygon counts as empty [m^2]: two convex sets
# that only share an edge or a corner do not intersect.
_EMPTY_AREA = 1e-12
# Slack when a width is compared against a requested minimum: the widths are
# multiples of a lattice step and must not lose a cell to rounding.
_WIDTH_SLACK = 1e-9


# ══ Lattices ══════════════════════════════════════════════════════════════════


def lattice_step(centres: Sequence[float]) -> float:
    """The spacing of a uniform ascending lattice (ValueError otherwise)."""
    values = np.asarray(centres, dtype=float)
    if values.ndim != 1 or values.size < 2:
        raise ValueError("a lattice needs at least two centres")
    steps = np.diff(values)
    step = float(steps[0])
    if not step > 0.0 or not np.allclose(steps, step, rtol=0.0, atol=1e-9 * max(1.0, abs(step))):
        raise ValueError("lattice centres must be uniform and ascending")
    return step


def held_cells(counts: np.ndarray, trials: np.ndarray | int) -> np.ndarray:
    """A cell is held when EVERY one of its trials was held (and it had trials).

    ``counts`` is the number of held trials per cell, ``trials`` the number run
    (a scalar, or per cell — a cell nobody ran is not held).
    """
    counts = np.asarray(counts)
    trials = np.broadcast_to(np.asarray(trials), counts.shape)
    if np.any(counts > trials) or np.any(counts < 0):
        raise ValueError("held counts must lie in [0, trials]")
    return (trials > 0) & (counts == trials)


# ══ (c, delta_o) map → boxes ══════════════════════════════════════════════════


@dataclass(frozen=True)
class Box:
    """A rectangle of held cells of the (closing speed, closure instant) map.

    The four bounds are cell EDGES. ``i0..i1`` and ``j0..j1`` are the inclusive
    cell indices along ``c`` and ``delta_o``.
    """

    c_lo: float  # [m/s]
    c_hi: float
    delta_o_lo: float  # [s] closure complete − ball at the origin
    delta_o_hi: float
    i0: int
    i1: int
    j0: int
    j1: int

    @property
    def width(self) -> float:
        return self.delta_o_hi - self.delta_o_lo

    @property
    def c_span(self) -> float:
        return self.c_hi - self.c_lo

    @property
    def area(self) -> float:
        return self.width * self.c_span


def _runs(row: np.ndarray) -> list[tuple[int, int]]:
    """Inclusive index ranges of the maximal runs of True."""
    padded = np.concatenate(([False], row, [False])).astype(np.int8)
    edges = np.diff(padded)
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1) - 1, strict=True))


def held_boxes(c: Sequence[float], delta_o: Sequence[float], held: np.ndarray) -> list[Box]:
    """Every rectangle of held cells that cannot be widened along ``delta_o``.

    ``held[i, j]`` is the verdict of the cell at ``(c[i], delta_o[j])``. For each
    range of ``c`` cells the rows held across the whole range form runs; each
    run is one box. (A box that could be widened along ``delta_o`` is never the
    answer to any of the questions below, so those are not listed.)
    """
    held = np.asarray(held, dtype=bool)
    c = np.asarray(c, dtype=float)
    delta_o = np.asarray(delta_o, dtype=float)
    if held.shape != (c.size, delta_o.size):
        raise ValueError(f"held is {held.shape}, the lattice is {(c.size, delta_o.size)}")
    half_c = 0.5 * lattice_step(c)
    half_d = 0.5 * lattice_step(delta_o)
    boxes = []
    for i0 in range(c.size):
        common = np.ones(delta_o.size, dtype=bool)
        for i1 in range(i0, c.size):
            common &= held[i1]
            if not common.any():
                break
            for j0, j1 in _runs(common):
                boxes.append(
                    Box(
                        c_lo=float(c[i0] - half_c),
                        c_hi=float(c[i1] + half_c),
                        delta_o_lo=float(delta_o[j0] - half_d),
                        delta_o_hi=float(delta_o[j1] + half_d),
                        i0=i0,
                        i1=i1,
                        j0=int(j0),
                        j1=int(j1),
                    )
                )
    return boxes


def box_by_min_width(boxes: Sequence[Box], min_width: float) -> Box | None:
    """Of the boxes at least ``min_width`` wide in ``delta_o``: the one that
    reaches the highest closing speed; among those the largest area; then the
    wider one, the earlier one, the lower ``c_lo`` (so the choice is unique).

    The width is what the closure window has to absorb of the ball's timing
    uncertainty, so a box is asked for BY its width and then pushed as far up
    in closing speed as the map allows. ``None`` when no box is that wide.
    """
    wide = [b for b in boxes if b.width >= min_width - _WIDTH_SLACK]
    if not wide:
        return None
    return min(wide, key=lambda b: (-b.c_hi, -b.area, -b.width, b.delta_o_lo, b.c_lo))


def box_candidates(
    c: Sequence[float],
    delta_o: Sequence[float],
    held: np.ndarray,
    min_widths: Sequence[float] = (0.02, 0.04, 0.08),
) -> dict[float, Box | None]:
    """:func:`box_by_min_width` for each requested width, keyed by the width."""
    boxes = held_boxes(c, delta_o, held)
    return {float(w): box_by_min_width(boxes, float(w)) for w in min_widths}


# ══ The closure window on the planner's axis ══════════════════════════════════


def entrance_window(
    c_lo: float, c_hi: float, delta_o_lo: float, delta_o_hi: float, s_ent: float
) -> tuple[float, float]:
    """``(delta_lo, delta_hi)``: the closure window after the crossing of
    ``s = s_ent`` that lies inside the box for EVERY closing speed in
    ``[c_lo, c_hi]`` (module docstring). ``delta_hi <= delta_lo`` is an empty
    window and is returned as it is.
    """
    if not (0.0 < c_lo <= c_hi):
        raise ValueError(f"closing speeds must satisfy 0 < c_lo <= c_hi, got {c_lo}, {c_hi}")
    shift_lo, shift_hi = s_ent / c_lo, s_ent / c_hi
    return delta_o_lo + max(shift_lo, shift_hi), delta_o_hi + min(shift_lo, shift_hi)


@dataclass(frozen=True)
class EntranceRow:
    """One sub-interval of a box's closing speeds on the planner's axis."""

    c_lo: float
    c_hi: float
    s_ent: float
    delta_lo: float
    delta_hi: float

    @property
    def width(self) -> float:
        return self.delta_hi - self.delta_lo


def entrance_rows(
    box: Box, c_edges: Sequence[float], s_ent_of: Callable[[float], float]
) -> list[EntranceRow]:
    """The box's window on the planner's axis for each way of raising ``c_lo``.

    A low ``c_lo`` costs twice: the tilt a given lateral speed can give the
    approach grows as ``1/c_lo`` (which raises ``s_ent``), and the window loses
    ``s_ent (1/c_lo − 1/c_hi)``. So ``c_hi`` is kept and ``c_lo`` is stepped up
    through ``c_edges`` (the lattice's cell edges inside the box); ``s_ent_of``
    gives the entrance offset that goes with a lower edge. Every row is a
    subset of the box, i.e. of conditions that were measured held.
    """
    rows = []
    for edge in sorted(float(e) for e in c_edges):
        if edge < box.c_lo - _WIDTH_SLACK or edge >= box.c_hi - _WIDTH_SLACK:
            continue
        s_ent = float(s_ent_of(edge))
        lo, hi = entrance_window(edge, box.c_hi, box.delta_o_lo, box.delta_o_hi, s_ent)
        rows.append(EntranceRow(edge, box.c_hi, s_ent, lo, hi))
    return rows


def widest_entrance_row(rows: Sequence[EntranceRow]) -> EntranceRow | None:
    """The row with the widest window; of equals, the one keeping more speeds."""
    if not rows:
        return None
    return min(rows, key=lambda r: (-r.width, r.c_lo))


# ══ Lateral set ═══════════════════════════════════════════════════════════════


def _square_distance(px: float, py: float, qx: np.ndarray, qy: np.ndarray, half: float):
    """Distance from the point to each axis-aligned square of half-width ``half``."""
    dx = np.maximum(np.abs(qx - px) - half, 0.0)
    dy = np.maximum(np.abs(qy - py) - half, 0.0)
    return np.hypot(dx, dy)


def _not_held_centres(
    xs: np.ndarray, ys: np.ndarray, held: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Centres of the cells that are not held, with one ring of cells added
    around the lattice: what was not measured is not held."""
    step = lattice_step(xs)
    if not math.isclose(lattice_step(ys), step, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError("the lateral lattice must have the same step along both axes")
    ext_x = np.concatenate(([xs[0] - step], xs, [xs[-1] + step]))
    ext_y = np.concatenate(([ys[0] - step], ys, [ys[-1] + step]))
    blocked = np.ones((ext_x.size, ext_y.size), dtype=bool)
    blocked[1:-1, 1:-1] = ~held
    ix, iy = np.nonzero(blocked)
    return ext_x[ix], ext_y[iy]


def inscribed_circle(
    xs: Sequence[float], ys: Sequence[float], held: np.ndarray
) -> tuple[tuple[float, float], float] | None:
    """The largest circle centred on a held lattice point that meets no cell
    which is not held. Of equal circles the one nearest the origin (the catch
    point), then the lower x, then the lower y. ``None`` when nothing is held.

    ``held[i, j]`` is the cell at ``(xs[i], ys[j])``; cells are squares of the
    lattice step. The radius is exact for that picture: the distance from the
    centre to the nearest not-held SQUARE, not to its centre.
    """
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    held = np.asarray(held, dtype=bool)
    if held.shape != (xs.size, ys.size):
        raise ValueError(f"held is {held.shape}, the lattice is {(xs.size, ys.size)}")
    half = 0.5 * lattice_step(xs)
    qx, qy = _not_held_centres(xs, ys, held)
    best = None
    for i, j in zip(*np.nonzero(held), strict=True):
        px, py = float(xs[i]), float(ys[j])
        radius = float(np.min(_square_distance(px, py, qx, qy, half)))
        key = (-radius, math.hypot(px, py), px, py)
        if best is None or key < best[0]:
            best = (key, (px, py), radius)
    return None if best is None else (best[1], best[2])


def _clip(polygon: np.ndarray, normal: np.ndarray, offset: float) -> np.ndarray:
    """Sutherland–Hodgman: the part of a convex polygon with ``normal·x <= offset``."""
    if polygon.shape[0] == 0:
        return polygon
    values = polygon @ normal - offset
    out = []
    count = polygon.shape[0]
    for k in range(count):
        a, b = polygon[k], polygon[(k + 1) % count]
        va, vb = values[k], values[(k + 1) % count]
        if va <= 0.0:
            out.append(a)
        if (va < 0.0 < vb) or (vb < 0.0 < va):
            out.append(a + (b - a) * (va / (va - vb)))
    return np.asarray(out, dtype=float).reshape(-1, 2)


def _area(polygon: np.ndarray) -> float:
    if polygon.shape[0] < 3:
        return 0.0
    x, y = polygon[:, 0], polygon[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


@dataclass(frozen=True)
class Polygon:
    """A convex set ``{rho : normals[i]·rho <= offsets[i]}`` with unit normals."""

    normals: np.ndarray  # (n, 2), unit rows
    offsets: np.ndarray  # (n,) [m]

    def contains(self, points: np.ndarray, slack: float = 0.0) -> np.ndarray:
        points = np.atleast_2d(np.asarray(points, dtype=float))
        return np.all(points @ self.normals.T <= self.offsets + slack, axis=1)

    def vertices(self, bound: float = 1e3) -> np.ndarray:
        """The polygon's corners, counter-clockwise (empty when the set is)."""
        poly = np.array([[-bound, -bound], [bound, -bound], [bound, bound], [-bound, bound]])
        for normal, offset in zip(self.normals, self.offsets, strict=True):
            poly = _clip(poly, normal, float(offset))
        return poly

    @property
    def area(self) -> float:
        return _area(self.vertices())

    def inradius_about(self, point: Sequence[float] = (0.0, 0.0)) -> float:
        """Distance from ``point`` to the nearest face; negative when outside."""
        return float(np.min(self.offsets - self.normals @ np.asarray(point, dtype=float)))

    def meets_square(self, cx: float, cy: float, half: float) -> bool:
        square = np.array(
            [
                [cx - half, cy - half],
                [cx + half, cy - half],
                [cx + half, cy + half],
                [cx - half, cy + half],
            ]
        )
        for normal, offset in zip(self.normals, self.offsets, strict=True):
            square = _clip(square, normal, float(offset))
            if square.shape[0] < 3:
                return False
        return _area(square) > _EMPTY_AREA


def _meets_any(polygon: Polygon, qx: np.ndarray, qy: np.ndarray, half: float) -> bool:
    # A square lying wholly beyond one face cannot meet the polygon: that
    # settles almost all of them without clipping.
    corners = np.array([[-half, -half], [half, -half], [half, half], [-half, half]])
    reach = np.min(corners @ polygon.normals.T, axis=0)  # (n,) the nearest corner per face
    centre = np.stack([qx, qy], axis=1) @ polygon.normals.T  # (m, n)
    near = np.all(centre + reach < polygon.offsets, axis=1)
    return any(
        polygon.meets_square(float(x), float(y), half)
        for x, y in zip(qx[near], qy[near], strict=True)
    )


def capture_polygon(
    xs: Sequence[float],
    ys: Sequence[float],
    held: np.ndarray,
    n_faces: int = 8,
    grow: float = 1e-3,
) -> Polygon | None:
    """A convex polygon of ``n_faces`` faces inside the held cells.

    The faces have fixed unit normals at ``2*pi*k / n_faces``. It starts as the
    regular polygon inscribed in :func:`inscribed_circle` and its faces are
    then pushed outward ``grow`` at a time, in turn, each for as long as the
    polygon still meets no cell that is not held. The result is inside the
    held region by construction; it is A large polygon, not the largest one
    (the order the faces move in decides between equals, and is fixed). A face
    that ends up not bounding the set is left touching it.

    ``None`` when nothing is held.
    """
    if n_faces < 3:
        raise ValueError("a polygon needs at least three faces")
    if not grow > 0.0:
        raise ValueError("grow must be positive")
    circle = inscribed_circle(xs, ys, held)
    if circle is None:
        return None
    (px, py), radius = circle
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    half = 0.5 * lattice_step(xs)
    qx, qy = _not_held_centres(xs, ys, np.asarray(held, dtype=bool))
    angles = 2.0 * math.pi * np.arange(n_faces) / n_faces
    normals = np.stack([np.cos(angles), np.sin(angles)], axis=1)
    offsets = normals @ np.array([px, py]) + radius * math.cos(math.pi / n_faces)
    active = np.ones(n_faces, dtype=bool)
    while active.any():
        for k in np.flatnonzero(active):
            trial = offsets.copy()
            trial[k] += grow
            candidate = Polygon(normals, trial)
            # A face the others have cut off no longer bounds the set: pushing
            # it changes nothing and would never be stopped.
            support = float(np.max(candidate.vertices() @ normals[k]))
            if support <= offsets[k] + 1e-12 or _meets_any(candidate, qx, qy, half):
                active[k] = False
            else:
                offsets = trial
    # Every offset on its face's support: a face that does not bound the set
    # is reported touching it, not floating outside.
    corners = Polygon(normals, offsets).vertices()
    return Polygon(normals, np.minimum(offsets, np.max(corners @ normals.T, axis=0)))


# ══ Lateral speed ═════════════════════════════════════════════════════════════


def largest_held_radius(magnitudes: Sequence[float], held: np.ndarray) -> float:
    """The upper edge of the last magnitude cell up to which EVERYTHING is held.

    ``held[k, ...]`` is the verdict of magnitude cell ``k`` for every direction
    and condition (any trailing shape). 0.0 when the first cell is not held —
    the disc below its lower edge is the zero-lateral-speed case, measured
    elsewhere.
    """
    magnitudes = np.asarray(magnitudes, dtype=float)
    held = np.asarray(held, dtype=bool)
    if held.shape[0] != magnitudes.size:
        raise ValueError("held's first axis must run over the magnitudes")
    half = 0.5 * lattice_step(magnitudes)
    every = held.reshape(magnitudes.size, -1).all(axis=1)
    count = int(np.argmin(every)) if not every.all() else magnitudes.size
    return 0.0 if count == 0 else float(magnitudes[count - 1] + half)


def rings_within_drop(held: Sequence[int], flown: Sequence[int], drop: float) -> np.ndarray:
    """Which rings hold as often as the reference ring, to within ``drop``.

    ``held[k]`` of the ``flown[k]`` fly-ins of ring ``k`` held; ring 0 is the
    reference (the same conditions without what the rings vary). Ring ``k``
    passes when ``held[k] / flown[k] >= held[0] / flown[0] − drop``. A ring
    that was not flown does not pass, and without a reference none does.

    The verdict of one fly-in is not certain even where the capture holds, so
    "every fly-in of a ring held" fails on the rate the reference itself has;
    this compares a ring with that rate instead.
    """
    held = np.asarray(held, dtype=float)
    flown = np.asarray(flown, dtype=float)
    if held.shape != flown.shape or held.ndim != 1:
        raise ValueError("held and flown must be 1-D and of one length")
    if not 0.0 <= drop < 1.0:
        raise ValueError("drop must be in [0, 1)")
    if np.any(held > flown) or np.any(held < 0.0):
        raise ValueError("held must be between 0 and flown")
    passed = np.zeros(held.size, dtype=bool)
    if held.size == 0 or flown[0] == 0.0:
        return passed
    done = flown > 0.0
    rate = np.divide(held, flown, out=np.zeros_like(held), where=done)
    passed[done] = rate[done] >= rate[0] - drop - 1e-12
    return passed


# ══ Static contact field → entrance plane, corridor ═══════════════════════════


@dataclass(frozen=True)
class Occupancy:
    """Where the ball centre touches the open (preshape) hand.

    ``contact[i, j, k]`` is True when the ball, its centre at
    ``(xs[i], ys[j], ss[k])`` in the capture frame, is in contact. All three
    lattices are uniform and ascending; ``xs`` and ``ys`` share a step.
    """

    xs: np.ndarray
    ys: np.ndarray
    ss: np.ndarray
    contact: np.ndarray

    def __post_init__(self) -> None:
        contact = np.asarray(self.contact, dtype=bool)
        shape = (len(self.xs), len(self.ys), len(self.ss))
        if contact.shape != shape:
            raise ValueError(f"contact is {contact.shape}, the lattices say {shape}")
        if not math.isclose(lattice_step(self.xs), lattice_step(self.ys), abs_tol=1e-9):
            raise ValueError("the lateral lattice must have the same step along both axes")
        lattice_step(self.ss)
        # The lateral lattice at half its step: an odd index is the gap between
        # two columns and holds the OR of the columns around it, so one lookup
        # reads "any column around this position".
        fine = np.zeros((2 * shape[0] - 1, 2 * shape[1] - 1, shape[2]), dtype=bool)
        fine[::2, ::2] = contact
        fine[1::2, ::2] = contact[:-1] | contact[1:]
        fine[::2, 1::2] = contact[:, :-1] | contact[:, 1:]
        fine[1::2, 1::2] = (
            contact[:-1, :-1] | contact[1:, :-1] | contact[:-1, 1:] | contact[1:, 1:]
        )
        object.__setattr__(self, "_fine", fine)

    def lateral_index(
        self, x: np.ndarray, y: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """``(ix, iy, outside)`` — half-step indices of lateral positions.

        A position on a lattice column reads that column; one between columns
        reads every column around it (conservative). ``outside`` marks
        positions beyond the lateral lattice, where nothing is known; their
        indices are clipped and must not be read as contact.
        """
        out = []
        outside = np.zeros(np.broadcast(x, y).shape, dtype=bool)
        for value, lattice in ((x, self.xs), (y, self.ys)):
            lattice = np.asarray(lattice, dtype=float)
            f = (np.asarray(value, dtype=float) - lattice[0]) / lattice_step(lattice)
            nearest = np.rint(f)
            on_column = np.abs(f - nearest) < 1e-6
            index = np.where(on_column, 2.0 * nearest, 2.0 * np.floor(f) + 1.0)
            outside = outside | (f < -1e-6) | (f > lattice.size - 1 + 1e-6)
            out.append(np.clip(index, 0, 2 * lattice.size - 2).astype(int))
        return out[0], out[1], outside

    def touches(
        self, x: np.ndarray, y: np.ndarray, level: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """``(contact, outside)`` at lateral positions on the given levels."""
        ix, iy, outside = self.lateral_index(x, y)
        return self._fine[ix, iy, level] & ~outside, outside


def trim_above_contact(occupancy: Occupancy) -> Occupancy:
    """The field cut one level above its highest contact.

    Above the hand there is nothing to touch, and an approach is traced only
    as far up as the field goes — so levels of free space above the hand only
    give a tilted line room to drift out of the lateral scan and be counted
    as touching. The cut says where the hand ends; it holds as long as the
    lateral scan covers the hand, which the top level being free of contact
    does not prove and the caller must ensure. A field that still touches on
    its top level is refused: the scan did not reach above the hand.
    """
    contact = np.asarray(occupancy.contact, dtype=bool)
    touching = np.flatnonzero(contact.any(axis=(0, 1)))
    if touching.size and touching[-1] == contact.shape[2] - 1:
        raise ValueError("the field touches on its top level: the scan ends inside the hand")
    # A field with no contact at all keeps two levels: a lattice needs a step.
    keep = max(int(touching[-1]) + 2 if touching.size else 0, 2)
    return Occupancy(
        occupancy.xs, occupancy.ys, np.asarray(occupancy.ss)[:keep], contact[:, :, :keep]
    )


def slope_fan(tan_max: float, tan_step: float = 0.1, directions: int = 8) -> np.ndarray:
    """Approach tilts ``d rho / d s`` covering ``tan_max``: the axis itself and
    rings every ``tan_step`` up to the first ring at or beyond ``tan_max`` —
    ``directions`` tilts each. Rounding the last ring UP keeps the fan on a
    fixed set of rings (so one table serves every ``tan_max``) and can only
    raise the entrance plane.
    """
    if tan_max < 0.0 or not tan_step > 0.0 or directions < 1:
        raise ValueError("slope_fan needs tan_max >= 0, tan_step > 0, directions >= 1")
    slopes = [(0.0, 0.0)]
    for ring in range(1, int(math.ceil(tan_max / tan_step - 1e-9)) + 1):
        for d in range(directions):
            angle = 2.0 * math.pi * d / directions
            slopes.append((ring * tan_step * math.cos(angle), ring * tan_step * math.sin(angle)))
    return np.asarray(slopes, dtype=float)


@dataclass(frozen=True)
class EntranceHeight:
    """The entrance plane a contact field supports for a set of approaches."""

    s_ent: float  # [m]; NaN when `holds` is False
    holds: bool  # the no-contact condition holds above s_ent inside the scan
    # Tilts that stop the plane from going one level lower (empty when the
    # scan's lowest level was reached).
    limit_slopes: tuple[tuple[float, float], ...]
    # True when none of those is stopped by a CONTACT — only by leaving the
    # lateral scan: the plane could be lower with a wider scan.
    scan_limited: bool


@dataclass(frozen=True)
class ApproachTable:
    """For each tilt and each level: is an approach anchored there blocked?

    An approach is a straight line of the ball centre — through a point of the
    lateral set on the plane of level ``n``, at ``point + slope * (s − ss[n])``
    above it. ``hit[k, n]`` is True when, for tilt ``slopes[k]``, the line
    through at least one of the points touches the hand on a level at or above
    ``n``; ``outside[k, n]`` when at least one leaves the lateral scan there.
    """

    ss: np.ndarray
    slopes: np.ndarray  # (K, 2)
    hit: np.ndarray  # (K, N)
    outside: np.ndarray  # (K, N)

    def entrance(self, tan_max: float, tan_step: float = 0.1) -> EntranceHeight:
        """The lowest ``s_ent`` above which no approach of tilt up to ``tan_max``
        touches the open hand.

        The reference's no-contact condition (§6.1): every approach — through
        every point of the lateral set, at every tilt the velocity set allows
        — is free of contact for ``s >= s_ent``. The plane is lowered from the
        top of the scan for as long as that holds, and the level it stops on is
        ``s_ent``. A line that leaves the lateral scan counts as touching.

        ``holds`` is False when the condition already fails on the top level:
        the scan then says nothing about a plane. The tilts are those of
        :func:`slope_fan`; a ``tan_max`` beyond the table's rings is refused.
        """
        norms = np.hypot(self.slopes[:, 0], self.slopes[:, 1])
        reach = math.ceil(tan_max / tan_step - 1e-9) * tan_step
        if reach > float(norms.max()) + 1e-9:
            raise ValueError(f"tilt {tan_max} is beyond the table's {float(norms.max())}")
        use = norms <= reach + 1e-9
        hit, outside = self.hit[use], self.outside[use]
        blocked = (hit | outside).any(axis=0)  # (N,)
        free_from_top = np.cumprod(~blocked[::-1])[::-1].astype(bool)
        if not free_from_top[-1]:
            stops = hit[:, -1] | outside[:, -1]
            return EntranceHeight(
                math.nan,
                False,
                tuple(map(tuple, self.slopes[use][stops])),
                bool(not hit[:, -1].any()),
            )
        level = int(np.argmax(free_from_top))
        if level == 0:
            return EntranceHeight(float(self.ss[0]), True, (), False)
        stops = hit[:, level - 1] | outside[:, level - 1]
        return EntranceHeight(
            float(self.ss[level]),
            True,
            tuple(map(tuple, self.slopes[use][stops])),
            bool(not hit[:, level - 1].any()),
        )


def approach_table(occupancy: Occupancy, points: np.ndarray, slopes: np.ndarray) -> ApproachTable:
    """:class:`ApproachTable` of a contact field for a lateral point set."""
    points = np.atleast_2d(np.asarray(points, dtype=float))
    slopes = np.atleast_2d(np.asarray(slopes, dtype=float))
    if points.shape[0] == 0:
        raise ValueError("an approach table needs at least one lateral point")
    ss = np.asarray(occupancy.ss, dtype=float)
    count = ss.size
    rise = ss - ss[0]  # the rise above the anchor depends on the level DIFFERENCE only
    fine = occupancy._fine
    hit = np.zeros((slopes.shape[0], count), dtype=bool)
    outside = np.zeros_like(hit)
    for k, slope in enumerate(slopes):
        ix, iy, out = occupancy.lateral_index(
            points[:, 0:1] + slope[0] * rise[None, :], points[:, 1:2] + slope[1] * rise[None, :]
        )  # (P, N): column j is the lateral position j levels above the anchor
        for n in range(count):
            span = count - n
            levels = np.arange(n, count)
            leaves = out[:, :span]
            outside[k, n] = bool(leaves.any())
            hit[k, n] = bool((fine[ix[:, :span], iy[:, :span], levels[None, :]] & ~leaves).any())
    return ApproachTable(ss, slopes, hit, outside)


def free_radius(occupancy: Occupancy, level: int) -> tuple[float, bool]:
    """``(radius, scan_limited)``: how far from the axis ``rho = 0`` the ball
    centre can be on that level without touching — the distance to the nearest
    touching cell, or to the edge of the lateral scan when that is nearer."""
    xs = np.asarray(occupancy.xs, dtype=float)
    ys = np.asarray(occupancy.ys, dtype=float)
    half = 0.5 * lattice_step(xs)
    edge = float(min(-xs[0], xs[-1], -ys[0], ys[-1]) + half)
    ix, iy = np.nonzero(np.asarray(occupancy.contact, dtype=bool)[:, :, level])
    if ix.size == 0:
        return max(edge, 0.0), True
    nearest = float(np.min(_square_distance(0.0, 0.0, xs[ix], ys[iy], half)))
    return (nearest, False) if nearest <= edge else (max(edge, 0.0), True)


@dataclass(frozen=True)
class Corridor:
    r_ent: float  # [m] the cone's radius on the entrance plane
    tan_theta: float  # >= 0, the cone's half-angle tangent
    scan_limited: bool  # a radius was set by the edge of the scan, not a contact
    length: float  # [m] the gap range the cone was fitted over


def corridor_fit(occupancy: Occupancy, s_ent: float, length: float) -> Corridor:
    """A cone ``|rho| <= r_ent + (s − s_ent) tan_theta`` about the axis that
    stays free of contact over ``s_ent <= s <= s_ent + length``.

    Of the cones inside the free radius of every level (with ``tan_theta >= 0``)
    it is the one of the largest mean radius over the range — the free radius
    is a staircase on a lattice, so a cone pinned to the radius of the entrance
    level would never open. Where the free region narrows going out, that cone
    is the cylinder of the smallest free radius.
    """
    ss = np.asarray(occupancy.ss, dtype=float)
    step = lattice_step(ss)
    start = int(np.searchsorted(ss, s_ent - 0.5 * step))
    stop = int(np.searchsorted(ss, s_ent + length + 0.5 * step))
    if start >= ss.size or stop <= start:
        raise ValueError("the corridor range lies outside the scanned levels")
    radii, limited = zip(*(free_radius(occupancy, k) for k in range(start, stop)), strict=True)
    radii = np.asarray(radii)
    gaps = ss[start:stop] - ss[start]
    span = float(gaps[-1])

    def radius_at(tan_theta: float) -> float:
        return float(np.min(radii - gaps * tan_theta))

    # r_ent(tan) is concave piecewise linear, so the mean radius
    # r_ent + tan * span / 2 peaks where two levels' bounds cross (or at 0).
    rise = radii[None, :] - radii[:, None]
    run = gaps[None, :] - gaps[:, None]
    crossings = rise[run > 0.0] / run[run > 0.0]
    best = (radius_at(0.0), 0.0, 0.0)  # (mean radius, -tan, tan): of equals the smaller tilt
    for tan_theta in np.unique(crossings[crossings > 0.0]):
        r_ent = radius_at(float(tan_theta))
        if r_ent < 0.0:
            continue
        best = max(
            best, (r_ent + 0.5 * span * float(tan_theta), -float(tan_theta), float(tan_theta))
        )
    tan_theta = best[2]
    return Corridor(max(radius_at(tan_theta), 0.0), tan_theta, bool(any(limited)), span)


# ══ Verification sample and its bound ═════════════════════════════════════════


def sample_capture_set(
    rng: np.random.Generator,
    polygon: Polygon,
    box: Box,
    v_perp_max: float,
    count: int,
) -> np.ndarray:
    """``count`` conditions drawn uniformly from the identified set.

    Columns: ``rho_x, rho_y`` (in the polygon), ``c`` and ``delta_o`` (in the
    box), ``nu_x, nu_y`` (in the disc of radius ``v_perp_max``).
    """
    corners = polygon.vertices()
    if _area(corners) <= _EMPTY_AREA:
        raise ValueError("the lateral polygon is empty")
    lo, hi = corners.min(axis=0), corners.max(axis=0)
    out = np.empty((count, 6))
    filled = 0
    while filled < count:
        rho = rng.uniform(lo, hi, size=(count, 2))
        rho = rho[polygon.contains(rho)][: count - filled]
        out[filled : filled + rho.shape[0], 0:2] = rho
        filled += rho.shape[0]
    out[:, 2] = rng.uniform(box.c_lo, box.c_hi, size=count)
    out[:, 3] = rng.uniform(box.delta_o_lo, box.delta_o_hi, size=count)
    radius = v_perp_max * np.sqrt(rng.uniform(0.0, 1.0, size=count))
    angle = rng.uniform(0.0, 2.0 * math.pi, size=count)
    out[:, 4] = radius * np.cos(angle)
    out[:, 5] = radius * np.sin(angle)
    return out


def _log_binomial_tail(successes: int, n: int, p: float) -> float:
    """``log P(X >= successes)`` for ``X ~ Binomial(n, p)``, ``0 < p < 1``."""
    terms = [
        math.lgamma(n + 1)
        - math.lgamma(i + 1)
        - math.lgamma(n - i + 1)
        + i * math.log(p)
        + (n - i) * math.log1p(-p)
        for i in range(successes, n + 1)
    ]
    peak = max(terms)
    return peak + math.log(sum(math.exp(t - peak) for t in terms))


def clopper_pearson_lower(successes: int, n: int, confidence: float = 0.95) -> float:
    """One-sided exact (Clopper–Pearson) lower bound on a success rate.

    The rate ``p`` at which seeing ``successes`` or more of ``n`` has
    probability ``1 − confidence``. With no failure it is
    ``(1 − confidence) ** (1 / n)`` — 300 held of 300 gives 0.990 at 95 %.
    """
    if not (0 <= successes <= n) or n <= 0:
        raise ValueError("need 0 <= successes <= n and n > 0")
    if not 0.0 < confidence < 1.0:
        raise ValueError("confidence must lie in (0, 1)")
    alpha = 1.0 - confidence
    if successes == 0:
        return 0.0
    if successes == n:
        return alpha ** (1.0 / n)
    lo, hi = 0.0, 1.0
    target = math.log(alpha)
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if _log_binomial_tail(successes, n, mid) < target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)
