"""From the rig's raw verdicts to the numbers the planner is given, and a report.

dynamic_catching E1-F15 (#741). run_ident.py leaves one JSON store per stage
under ``$DATA/<profile>/``; this module reads them and applies the arithmetic
of ``rtc_tools.analysis.catching_capture_set``. It simulates nothing, so the
whole identification can be re-derived from the raw data at any time — and
run_ident.py calls the same functions to decide what the next stage flies.

THE PROTOCOL (fixed before any flight; the lattices are the constants below).

1. ``map-coarse`` — one fly-in per point of a coarse (c, delta_o) lattice at
   the catch point, to find where anything is held at all.
2. ``map-fine`` — the cells of a fine lattice around every coarse point that
   held, FOUR fly-ins per cell spread over the cell (c, delta_o uniform in it,
   the lateral point within 2.5 mm). A cell is held when all four are.
3. Boxes — for each requested closure-window width, the box of held cells at
   least that wide that reaches the highest closing speed.
4. ``lateral`` — per box, at its four corner cells and its centre cell: which
   lateral cells hold (the ball arriving straight). One fly-in of the centre
   condition is flown at every cell of the lateral lattice first; the cells
   where it held, and the cells within two cells of one, are the CANDIDATES.
   Each candidate is then flown EIGHT times under each of the five conditions
   — forty fly-ins, none skipped. The reference cell is the one the centre
   condition holds most often at (among equals, the one whose forty hold most
   often), and a candidate is in the lateral set when its forty hold as often
   as the reference cell's forty to within 5 points of rate. (A fly-in's
   verdict is not certain even inside the set — the same reason the speed
   rings below are compared by rate — so "all of a cell's fly-ins hold"
   shrinks the set with the number of fly-ins, not with the hand. A cell that
   holds only beyond two cells from every cell the first pass found is not
   flown and is not in the set.)
   ``vperp`` — the same five conditions, the ball aimed at the CENTRE OF THAT
   LATERAL SET (its inscribed circle) with a lateral speed: ring 0 has none,
   the rings after it 0.05 m/s apart. A ring passes when it holds as often as
   ring 0 to within 5 points of rate, and the lateral speed limit is the last
   ring up to which every ring passes. (A fly-in's verdict is not certain even
   inside the set, so "every fly-in of a ring holds" cannot be met by a ring
   of 160; and the catch point itself need not be in the lateral set.)
5. ``static`` — where the ball touches the OPEN hand: the entrance plane and
   the corridor are read from that field for the box's lateral set and tilts.
6. ``verify`` — 300 conditions drawn from the identified set, flown once each.
   Failures are reported; nothing is shrunk and re-measured.
7. ``accel`` — the same 300 conditions with the ball accelerating in the
   catch frame (gravity less the hand's own acceleration, which the straight
   flights leave out): toward the hand along the approach axis, and the way
   gravity pulls it past a hand standing still at the wait pose. It
   identifies nothing: it says how much of the set survives when the relative
   motion is not uniform.

Run as ``report.py <profile>`` (with ``DATA`` set) to print the report.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from rtc_tools.analysis import catching_capture_set as cs

# ── The protocol's lattices ───────────────────────────────────────────────────
SEED = 741
TRIALS = 4  # fly-ins per cell (map-fine) and per condition x direction (vperp)
RHO_JITTER = 0.0025  # [m] lateral spread of a cell's fly-ins (map, vperp)
COARSE_C = (0.25, 0.25, 20)  # first, step, count [m/s]: 0.25 … 5.0
COARSE_D = (-0.30, 0.01, 61)  # [s]: −0.30 … +0.30
FINE_DC = 0.1  # [m/s] cells [0.1 a, 0.1 (a + 1)]
FINE_DD = 0.004  # [s]   cells [0.004 b, 0.004 (b + 1)]
FINE_A_MIN = 2  # no cell below c = 0.2 m/s
MIN_WIDTHS = (0.020, 0.040, 0.080)  # [s] closure-window widths a box is asked for
LATERAL = (-0.06, 0.005, 25)  # [m]: −60 … +60 mm, both axes
LATERAL_TRIALS = 8  # fly-ins per candidate cell and condition
LATERAL_RINGS = 2  # cells around a cell the first pass held that are candidates too
LATERAL_DROP = 0.05  # a cell is in the set within this of the reference cell's hold rate
VPERP = (0.05, 0.05, 20)  # [m/s]: rings 0.05 … 1.0
VPERP_DIRECTIONS = 8
VPERP_DROP = 0.05  # a ring passes within this of the zero-speed ring's hold rate
VPERP_PATIENCE = 2  # rings in a row that do not pass, after which none is flown
FIELD_XY = (-0.15, 0.005, 61)  # [m]: −150 … +150 mm, both axes
FIELD_S = (-0.010, 0.001, 261)  # [m]: −10 … +250 mm
TAN_STEP = 0.1
TAN_MAX = 1.0  # tilts of the approach table
CORRIDOR_LENGTH = 0.15  # [m] the gap range the corridor is fitted over
VERIFY_N = 300
# (label, frame, acceleration [m/s²]). "catch": in the catch frame. "world":
# in the model's world, turned into the catch frame of the hand at the wait
# pose — gravity as a hand that stands still sees it.
ACCEL_CASES = (
    ("접근축 5", "catch", (0.0, 0.0, -5.0)),
    ("접근축 9.81", "catch", (0.0, 0.0, -9.81)),
    ("대기 자세의 중력", "world", (0.0, 0.0, -9.81)),
)
CONFIDENCE = 0.95


def lattice(spec: tuple[float, float, int]) -> np.ndarray:
    first, step, count = spec
    return np.round(first + step * np.arange(count), 9)


def fine_c(a: int) -> float:
    return round(FINE_DC * (a + 0.5), 9)


def fine_d(b: int) -> float:
    return round(FINE_DD * (b + 0.5), 9)


def data_dir(profile: str) -> Path:
    root = os.environ.get("DATA")
    if not root:
        raise SystemExit("DATA is not set: name the directory the raw data lives in")
    return Path(root) / profile


class Store:
    """A JSON object ``id -> result`` on disk, rewritten whole and atomically."""

    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self.items: dict = json.loads(self.path.read_text()) if self.path.is_file() else {}

    def save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.items))
        tmp.replace(self.path)


# ── Maps ──────────────────────────────────────────────────────────────────────


def coarse_held(store: Store) -> np.ndarray:
    """``held[i, j]`` of the coarse lattice (False where nothing was flown)."""
    held = np.zeros((COARSE_C[2], COARSE_D[2]), dtype=bool)
    for key, result in store.items.items():
        i, j = (int(part[1:]) for part in key.split("_"))
        held[i, j] = bool(result["held"])
    return held


def fine_cells(held: np.ndarray) -> list[tuple[int, int]]:
    """The fine cells whose centre lies within one coarse cell of a coarse
    point that held."""
    cc, dd = lattice(COARSE_C), lattice(COARSE_D)
    reach_c, reach_d = 1.5 * COARSE_C[1], 1.5 * COARSE_D[1]
    cells = set()
    for i, j in zip(*np.nonzero(held), strict=True):
        a0 = max(FINE_A_MIN, math.ceil((cc[i] - reach_c) / FINE_DC - 0.5 - 1e-9))
        a1 = math.floor((cc[i] + reach_c) / FINE_DC - 0.5 + 1e-9)
        b0 = math.ceil((dd[j] - reach_d) / FINE_DD - 0.5 - 1e-9)
        b1 = math.floor((dd[j] + reach_d) / FINE_DD - 0.5 + 1e-9)
        cells.update((a, b) for a in range(a0, a1 + 1) for b in range(b0, b1 + 1))
    return sorted(cells)


class FineMap:
    """The fine (c, delta_o) map as arrays over the lattice it was flown on."""

    def __init__(self, store: Store) -> None:
        cells: dict[tuple[int, int], list] = {}
        for key, result in store.items.items():
            a, b, _ = (int(part[1:]) for part in key.split("_"))
            cells.setdefault((a, b), []).append(result)
        if not cells:
            raise SystemExit(f"{store.path}: no fine map — run map-fine first")
        self.a0 = min(a for a, _ in cells)
        self.b0 = min(b for _, b in cells)
        na = max(a for a, _ in cells) - self.a0 + 1
        nb = max(b for _, b in cells) - self.b0 + 1
        # At least two cells along each axis: a lattice needs a step.
        na, nb = max(na, 2), max(nb, 2)
        self.c = np.array([fine_c(self.a0 + i) for i in range(na)])
        self.d = np.array([fine_d(self.b0 + j) for j in range(nb)])
        self.counts = np.zeros((na, nb), dtype=int)
        self.trials = np.zeros((na, nb), dtype=int)
        self.s_first = np.full((na, nb), np.nan)  # highest first-contact height of a cell
        self.stray = 0
        for (a, b), results in cells.items():
            i, j = a - self.a0, b - self.b0
            self.trials[i, j] = len(results)
            self.counts[i, j] = sum(bool(r["held"]) for r in results)
            heights = [r["s_first"] for r in results if r["s_first"] is not None]
            if heights:
                self.s_first[i, j] = max(heights)
            self.stray += sum(r["stray"] is not None for r in results)
        # A cell with fewer than TRIALS fly-ins (an interrupted run) is not held.
        self.held = cs.held_cells(np.where(self.trials == TRIALS, self.counts, 0), TRIALS)

    def boxes(self) -> dict[float, cs.Box | None]:
        return cs.box_candidates(self.c, self.d, self.held, MIN_WIDTHS)

    def cell(self, box: cs.Box, which: str) -> tuple[int, int]:
        """A condition cell of a box as fine indices (a, b): its centre cell or
        one of its corner cells (``lo_lo`` = lowest speed, earliest closure)."""
        i = {"lo": box.i0, "hi": box.i1, "mid": (box.i0 + box.i1) // 2}
        j = {"lo": box.j0, "hi": box.j1, "mid": (box.j0 + box.j1) // 2}
        ci, dj = ("mid", "mid") if which == "centre" else which.split("_")
        return self.a0 + i[ci], self.b0 + j[dj]


CONDITIONS = ("centre", "lo_lo", "lo_hi", "hi_lo", "hi_hi")


def box_tag(width: float) -> str:
    return f"w{int(round(width * 1000)):03d}"


def distinct_boxes(boxes: dict[float, cs.Box | None]) -> dict[str, cs.Box]:
    """The boxes to measure, keyed by the tag of the first width that asks for
    each (two widths often ask for the same box)."""
    out: dict[str, cs.Box] = {}
    for width in sorted(boxes):
        box = boxes[width]
        if box is not None and box not in out.values():
            out[box_tag(width)] = box
    return out


@dataclass(frozen=True)
class LateralVerdict:
    """The lateral stage's store, judged. Arrays are over the lateral lattice."""

    held: np.ndarray  # bool: in the lateral set
    candidates: np.ndarray  # bool: flown under every condition
    counts: np.ndarray  # fly-ins that held, all conditions together
    flown: np.ndarray  # fly-ins flown, all conditions together
    reference: tuple[int, int] | None  # the cell the others are compared with
    reference_counts: tuple[int, ...]  # its fly-ins that held, per condition
    missing: int  # candidate fly-ins the store does not have yet

    @property
    def complete(self) -> bool:
        return self.missing == 0

    @property
    def reference_rate(self) -> float:
        full = LATERAL_TRIALS * len(CONDITIONS)
        return sum(self.reference_counts) / full if self.reference is not None else math.nan


def lateral_counts(store: Store) -> tuple[np.ndarray, np.ndarray]:
    """``(held, flown)`` per condition and lateral cell."""
    n = LATERAL[2]
    held = np.zeros((len(CONDITIONS), n, n), dtype=int)
    flown = np.zeros_like(held)
    for key, result in store.items.items():
        q, i, j, _ = (int(part[1:]) for part in key.split("_"))
        flown[q, i, j] += 1
        held[q, i, j] += bool(result["held"])
    return held, flown


def lateral_candidates(store: Store, rings: int = LATERAL_RINGS) -> np.ndarray:
    """The cells the stage flies under every condition: where the first pass
    (fly-in 0 of the centre condition, flown at every cell) held, and the
    cells within ``rings`` cells of one, diagonals included."""
    n = LATERAL[2]
    first = np.zeros((n, n), dtype=bool)
    for i in range(n):
        for j in range(n):
            result = store.items.get(f"q0_x{i}_y{j}_k0")
            first[i, j] = result is not None and bool(result["held"])
    out = np.zeros_like(first)
    for i, j in zip(*np.nonzero(first), strict=True):
        out[max(0, i - rings) : i + rings + 1, max(0, j - rings) : j + rings + 1] = True
    return out


def lateral_verdict(store: Store) -> LateralVerdict:
    """The lateral set by hold rate (step 4 of the protocol).

    A candidate cell is in the set when its fly-ins — all conditions together
    — hold as often as the reference cell's, to within ``LATERAL_DROP``. The
    reference cell is the candidate the centre condition holds most often at;
    among equals, the one whose fly-ins hold most often altogether (which of
    several such cells it is does not change the rate compared with). A cell
    none of whose fly-ins held is never in the set. Until every candidate has
    all its fly-ins nothing is: an interrupted stage has no verdict.
    """
    held, flown = lateral_counts(store)
    candidates = lateral_candidates(store)
    counts, total = held.sum(axis=0), flown.sum(axis=0)
    full = LATERAL_TRIALS * len(CONDITIONS)
    missing = int((LATERAL_TRIALS - flown[:, candidates]).clip(min=0).sum())
    n = LATERAL[2]
    missing += sum(f"q0_x{i}_y{j}_k0" not in store.items for i in range(n) for j in range(n))
    empty = LateralVerdict(np.zeros_like(candidates), candidates, counts, total, None, (), missing)
    if missing or not candidates.any():
        return empty
    cells = [(int(i), int(j)) for i, j in zip(*np.nonzero(candidates), strict=True)]
    ri, rj = max(cells, key=lambda c: (held[0][c], counts[c], -c[0], -c[1]))
    if counts[ri, rj] == 0:
        return empty
    in_set = candidates & (counts > 0)
    in_set &= counts >= counts[ri, rj] - LATERAL_DROP * full - 1e-9
    return LateralVerdict(
        in_set, candidates, counts, total, (ri, rj), tuple(int(v) for v in held[:, ri, rj]), 0
    )


def vperp_counts(store: Store) -> tuple[np.ndarray, np.ndarray]:
    """``(held, flown)`` per ring of the lateral-speed stage. Ring 0 has no
    lateral speed; ring r >= 1 is the (r − 1)th magnitude of ``VPERP``."""
    held = np.zeros(VPERP[2] + 1, dtype=int)
    flown = np.zeros_like(held)
    for key, result in store.items.items():
        ring = key.split("_")[1]
        if not ring.startswith("r"):
            raise SystemExit(
                f"{store.path}: flown before the rings were compared by rate (key {key}) — "
                "move it away and fly vperp again"
            )
        flown[int(ring[1:])] += 1
        held[int(ring[1:])] += bool(result["held"])
    return held, flown


def vperp_aim(ident: dict) -> tuple[float, float] | None:
    """The lateral point the speed rings are aimed at: the centre of the box's
    lateral set, or None when that set is empty."""
    if ident.get("polygon") is None:
        return None
    centre = ident["circle"][0]
    return float(centre[0]), float(centre[1])


def load_field(path: Path) -> tuple[cs.Occupancy, np.ndarray]:
    """The open hand's contact field, cut above the hand, and the stray mask."""
    raw = np.load(path)
    full = cs.Occupancy(raw["xs"], raw["ys"], raw["ss"], raw["hand"])
    return cs.trim_above_contact(full), raw["stray"]


def lateral_points(polygon: cs.Polygon) -> np.ndarray:
    """The lateral set as points: the lateral lattice's points inside it, and
    its corners (the set is convex, but the hand around it is not)."""
    xs = lattice(LATERAL)
    grid = np.array([(x, y) for x in xs for y in xs])
    return np.vstack([grid[polygon.contains(grid, slack=1e-9)], polygon.vertices()])


# ── One box, all the way ──────────────────────────────────────────────────────


def identify(directory: Path, tag: str, box: cs.Box, fine: FineMap) -> dict:
    """Everything that follows from one box and the stages flown for it. Keys
    that depend on a stage not yet run are absent."""
    out: dict = {"tag": tag, "box": box}
    lateral_path = directory / f"lateral_{tag}.json"
    if not lateral_path.is_file():
        return out
    xs = lattice(LATERAL)
    verdict = lateral_verdict(Store(lateral_path))
    out["lateral"] = verdict
    if not verdict.complete:
        return out  # an interrupted stage, or a store flown under another rule
    held = verdict.held
    out["lateral_held"] = held
    polygon = cs.capture_polygon(xs, xs, held)
    out["polygon"] = polygon
    circle = cs.inscribed_circle(xs, xs, held)
    out["circle"] = circle
    if polygon is None:
        return out

    vperp_path = directory / f"vperp_{tag}.json"
    if not vperp_path.is_file():
        return out
    ring_held, ring_flown = vperp_counts(Store(vperp_path))
    passed = cs.rings_within_drop(ring_held, ring_flown, VPERP_DROP)
    out["vperp_held"], out["vperp_flown"], out["vperp_pass"] = ring_held, ring_flown, passed
    v_perp_max = cs.largest_held_radius(lattice(VPERP), passed[1:])
    out["v_perp_max"] = v_perp_max

    field_path = directory / "static.npz"
    if not field_path.is_file():
        return out
    occupancy, _ = load_field(field_path)
    table = cs.approach_table(occupancy, lateral_points(polygon), cs.slope_fan(TAN_MAX, TAN_STEP))

    heights: dict[float, cs.EntranceHeight | None] = {}

    def entrance(c_lo: float) -> cs.EntranceHeight | None:
        if c_lo not in heights:
            tan = v_perp_max / c_lo
            heights[c_lo] = table.entrance(tan, TAN_STEP) if tan <= TAN_MAX + 1e-9 else None
        return heights[c_lo]

    whole = entrance(box.c_lo)
    out["entrance"] = whole  # None: the tilt is beyond the table
    out["tan_max"] = v_perp_max / box.c_lo
    edges = [float(fine.c[i] - 0.5 * FINE_DC) for i in range(box.i0, box.i1 + 1)]
    usable = [e for e in edges if (h := entrance(e)) is not None and h.holds]
    out["rows"] = cs.entrance_rows(box, usable, lambda c_lo: entrance(c_lo).s_ent)
    out["widest"] = cs.widest_entrance_row(out["rows"])
    if whole is not None and whole.holds:
        length = min(CORRIDOR_LENGTH, float(occupancy.ss[-1]) - whole.s_ent)
        if length > 2.0 * FIELD_S[1]:
            out["corridor"] = cs.corridor_fit(occupancy, whole.s_ent, length)

    verify_path = directory / f"verify_{tag}.json"
    straight: dict = {}
    if verify_path.is_file():
        straight = Store(verify_path).items
        results = list(straight.values())
        held_n = sum(bool(r["held"]) for r in results)
        out["verify"] = {
            "n": len(results),
            "held": held_n,
            "lower": cs.clopper_pearson_lower(held_n, len(results), CONFIDENCE),
            "why": {w: sum(r["why"] == w for r in results) for w in {r["why"] for r in results}},
            "stray": sum(r["stray"] is not None for r in results),
            "s_pass": results[0]["s_pass"],
        }
    accel_path = directory / f"accel_{tag}.json"
    if "verify" in out and accel_path.is_file():
        flown = Store(accel_path).items
        out["accel"] = []
        for g, (label, _, _) in enumerate(ACCEL_CASES):
            prefix = f"g{g}_"
            rows = {k[len(prefix) :]: r for k, r in flown.items() if k.startswith(prefix)}
            if not rows:
                continue
            held_n = sum(bool(r["held"]) for r in rows.values())
            pairs = [(bool(straight[k]["held"]), bool(r["held"])) for k, r in rows.items()]
            out["accel"].append(
                {
                    "label": label,
                    "accel": next(iter(rows.values())).get("accel"),  # catch frame
                    "n": len(rows),
                    "held": held_n,
                    "lower": cs.clopper_pearson_lower(held_n, len(rows), CONFIDENCE),
                    "lost": sum(before and not after for before, after in pairs),
                    "gained": sum(after and not before for before, after in pairs),
                    "why": {
                        w: sum(r["why"] == w for r in rows.values())
                        for w in sorted({r["why"] for r in rows.values()})
                    },
                }
            )
    return out


# ── Rendering ─────────────────────────────────────────────────────────────────


def _ms(value: float) -> str:
    return f"{value * 1e3:+.0f}"


def _mm(value: float) -> str:
    return f"{value * 1e3:.1f}"


def render_map(fine: FineMap, max_rows: int = 400) -> str:
    """The count map as text: one row per speed (fastest first), one character
    per closure-instant cell — the number of fly-ins that held, '·' not flown."""
    lines = [f"delta_o from {_ms(fine.d[0] - 0.5 * FINE_DD)} ms, {FINE_DD * 1e3:.0f} ms a cell"]
    for i in range(len(fine.c) - 1, -1, -1):
        if not fine.trials[i].any():
            continue
        row = "".join(
            "·" if fine.trials[i, j] == 0 else str(fine.counts[i, j]) for j in range(len(fine.d))
        )
        lines.append(f"{fine.c[i]:5.2f} {row}")
    return "\n".join(lines[: max_rows + 1])


def render_lateral(verdict: LateralVerdict) -> str:
    """y down the page (high first), x across: '#' in the lateral set, '@' the
    reference cell; a candidate that is not in the set shows the tenth its
    hold rate falls in (9: 90 % or more), a cell that was not a candidate '·'."""
    xs = lattice(LATERAL)
    full = LATERAL_TRIALS * len(CONDITIONS)

    def mark(i: int, j: int) -> str:
        if (i, j) == verdict.reference:
            return "@"
        if verdict.held[i, j]:
            return "#"
        if not verdict.candidates[i, j]:
            return "·"
        return str(min(9, int(verdict.counts[i, j]) * 10 // full))

    lines = [f"x from {_mm(xs[0])} to {_mm(xs[-1])} mm, {LATERAL[1] * 1e3:.0f} mm a cell"]
    for j in range(len(xs) - 1, -1, -1):
        row = "".join(mark(i, j) for i in range(len(xs)))
        if row.strip("·"):
            lines.append(f"{xs[j] * 1e3:+6.1f} {row}")
    return "\n".join(lines)


def render(profile: str, directory: Path) -> str:
    out = [f"### `{profile}`", ""]
    check_path = directory / "selfcheck.json"
    if check_path.is_file():
        check = json.loads(check_path.read_text())
        out += [
            "**자체 점검**",
            "",
            f"- 같은 조건 두 번의 결과: {'일치' if check['repeatable'] else '**불일치**'}",
            f"- 단단한 면에서의 반발 계수: {check['restitution']:.3f} "
            f"(목표 {check['applied']['ball']['restitution_target']})",
            "- 빈 손 폐쇄 시간 (η 도달): "
            + (
                f"{check['empty_close_s'] * 1e3:.1f} ms"
                if check["empty_close_s"] is not None
                else "**η 에 닿지 않음** (빈 손이 제한 시간 안에 닫히지 않는다)"
            )
            + f" (`T_close_e2e` {check['t_close_e2e'] * 1e3:.1f} ms)",
            f"- 정착: {check['settle_s']:.1f} s, `q_pre` 와의 최대 차 {check['hand_error']:.4f} rad",
            f"- 축 위의 첫 접촉: {check['axis_contact']}",
            f"- 접촉 검사 (kinematics + collision) 와 `mj_forward` 의 일치: "
            f"{check['collision_agrees']} / {check['collision_checked']}",
            "",
        ]
    fine_path = directory / "map_fine.json"
    if not fine_path.is_file():
        return "\n".join(out + ["(map-fine 을 아직 돌리지 않았다)"])
    fine = FineMap(Store(fine_path))
    flown = int(fine.trials.sum())
    out += [
        f"**(c, δ^O) 지도** — 셀 {int((fine.trials > 0).sum())} 개, 시행 {flown} 회, "
        f"유지된 셀 {int(fine.held.sum())} 개, 손보다 먼저 다른 것에 닿은 시행 {fine.stray} 회",
        "",
        "```text",
        render_map(fine),
        "```",
        "",
    ]
    boxes = fine.boxes()
    out += [
        "**상자 후보**",
        "",
        "| 폭 ≥ | c [m/s] | δ^O [ms] | 폭 [ms] | 셀 |",
        "|---|---|---|---|---|",
    ]
    for width in sorted(boxes):
        box = boxes[width]
        if box is None:
            out.append(f"| {width * 1e3:.0f} ms | 없음 | | | |")
            continue
        out.append(
            f"| {width * 1e3:.0f} ms | {box.c_lo:.1f} – {box.c_hi:.1f} | "
            f"{_ms(box.delta_o_lo)} … {_ms(box.delta_o_hi)} | {box.width * 1e3:.0f} | "
            f"{(box.i1 - box.i0 + 1) * (box.j1 - box.j0 + 1)} |"
        )
    out.append("")
    for tag, box in distinct_boxes(boxes).items():
        out += render_box(identify(directory, tag, box, fine), fine, closure_axis(directory))
    return "\n".join(out)


def closure_axis(directory: Path) -> float | None:
    """The closure time the stores' ``delta_o`` is measured against [s]: the
    profile's ``T_close_e2e`` when the rig was checked (selfcheck). A fly-in
    commands the closure that long, less ``delta_o``, before the ball reaches
    the origin plane — so the axis has this constant in it, and stores flown
    against two values of it do not belong together. None without a selfcheck."""
    path = directory / "selfcheck.json"
    if not path.is_file():
        return None
    return float(json.loads(path.read_text())["t_close_e2e"])


def render_box(ident: dict, fine: FineMap, t_axis: float | None = None) -> list[str]:
    box: cs.Box = ident["box"]
    out = [
        f"#### 상자 `{ident['tag']}` — c {box.c_lo:.1f} – {box.c_hi:.1f} m/s, "
        f"δ^O {_ms(box.delta_o_lo)} … {_ms(box.delta_o_hi)} ms",
        "",
    ]
    if "lateral" not in ident:
        return out + ["(lateral 을 아직 돌리지 않았다)", ""]
    verdict: LateralVerdict = ident["lateral"]
    full = LATERAL_TRIALS * len(CONDITIONS)
    if not verdict.complete:
        return out + [
            f"(lateral 이 유지율 규칙으로 끝까지 날려지지 않았다 — 후보 "
            f"{int(verdict.candidates.sum())} 셀에 시행 {verdict.missing} 회가 없다)",
            "",
        ]
    polygon = ident["polygon"]
    held = verdict.held
    xs = lattice(LATERAL)
    out.append(
        f"- lateral: 후보 {int(verdict.candidates.sum())} 셀 (가운데 조건의 첫 시행이 유지된 셀과 "
        f"그 둘레 {LATERAL_RINGS} 셀), 셀마다 {len(CONDITIONS)} 조건 × {LATERAL_TRIALS} 회"
    )
    if verdict.reference is not None:
        ri, rj = verdict.reference
        out.append(
            f"- 기준 셀 ({_mm(xs[ri])}, {_mm(xs[rj])}) mm: 유지율 "
            f"{sum(verdict.reference_counts)}/{full} = {verdict.reference_rate:.3f} (조건별 "
            + ", ".join(
                f"{name} {n}/{LATERAL_TRIALS}"
                for name, n in zip(CONDITIONS, verdict.reference_counts, strict=True)
            )
            + f"). 유지율이 이보다 {LATERAL_DROP * 100:.0f} %p 넘게 낮지 않은 셀이 집합이다"
        )
    out += [
        f"- 집합: {int(held.sum())} 셀 ({held.sum() * LATERAL[1] ** 2 * 1e4:.1f} cm²)",
        "",
        "```text",
        render_lateral(verdict),
        "```",
        "",
    ]
    if polygon is None:
        return out + [
            "**유지되는 lateral 셀이 없다 — $\\mathcal C_\\perp$ 가 빈다.**",
            "",
        ]
    (cx, cy), radius = ident["circle"]
    out += [
        f"- 내접 원: 중심 ({_mm(cx)}, {_mm(cy)}) mm, 반지름 {_mm(radius)} mm. "
        f"원점에서 가장 가까운 면까지 {_mm(polygon.inradius_about())} mm, 넓이 "
        f"{polygon.area * 1e4:.1f} cm²",
        "",
        "| 면 | $\\tilde a$ | $\\tilde b$ [mm] |",
        "|---|---|---|",
    ]
    for k, (normal, offset) in enumerate(zip(polygon.normals, polygon.offsets, strict=True)):
        out.append(f"| {k} | ({normal[0]:+.4f}, {normal[1]:+.4f}) | {_mm(offset)} |")
    out.append("")
    if "v_perp_max" not in ident:
        return out + ["(vperp 를 아직 돌리지 않았다)", ""]
    aim = vperp_aim(ident)
    out += [
        f"- $v_{{\\perp,\\max}}$ = {ident['v_perp_max']:.3f} m/s "
        f"(접근의 기울기 $\\tan$ ≤ {ident.get('tan_max', math.nan):.2f}). lateral 집합의 중심 "
        f"({_mm(aim[0])}, {_mm(aim[1])}) mm 를 겨눈 옆 속도 고리 — 옆 속도 0 의 유지율보다 "
        f"{VPERP_DROP * 100:.0f} %p 넘게 낮지 않으면 통과:",
        "",
        "| 옆 속도 [m/s] | 시행 | 유지 | 유지율 | 95 % 하한 | 통과 |",
        "|---|---|---|---|---|---|",
    ]
    speeds = [0.0, *lattice(VPERP)]
    for r, flown in enumerate(ident["vperp_flown"]):
        if flown == 0:
            continue
        held_n = int(ident["vperp_held"][r])
        out.append(
            f"| {speeds[r]:.2f} | {flown} | {held_n} | {held_n / flown:.3f} | "
            f"{cs.clopper_pearson_lower(held_n, int(flown), CONFIDENCE):.3f} | "
            f"{'기준' if r == 0 else ('예' if ident['vperp_pass'][r] else '아니오')} |"
        )
    out.append("")
    if "entrance" not in ident:
        return out + ["", "(static 을 아직 돌리지 않았다)", ""]
    whole = ident["entrance"]
    if whole is None:
        out.append(
            f"- 상자 전체의 기울기가 표의 범위 ({TAN_MAX}) 를 넘는다 — 아래 부분 구간만 계산한다"
        )
    elif not whole.holds:
        out.append("- **무접촉 일관성: 성립하지 않음** — 스캔의 맨 위에서 이미 닿는다")
    else:
        limited = " (lateral 스캔을 벗어나는 직선이 막는다)" if whole.scan_limited else ""
        lo, hi = cs.entrance_window(
            box.c_lo, box.c_hi, box.delta_o_lo, box.delta_o_hi, whole.s_ent
        )
        verdict = f"{_ms(lo)} … {_ms(hi)} ms (폭 {(hi - lo) * 1e3:.0f})" if hi > lo else "**빈다**"
        out += [
            f"- **무접촉 일관성: 성립**, $s_{{ent}}$ = {_mm(whole.s_ent)} mm{limited}",
            f"- 상자 전체의 $\\delta$ 창: {verdict}",
        ]
        if t_axis is not None and hi > lo:
            # The same window without the axis constant: how long before the
            # ball crosses s_ent the closure has to be commanded.
            out.append(
                f"- 폐쇄 명령에서 $s_{{ent}}$ 통과까지: {(t_axis - hi) * 1e3:.1f} … "
                f"{(t_axis - lo) * 1e3:.1f} ms, 가운데 {(t_axis - 0.5 * (lo + hi)) * 1e3:.2f} ms "
                f"($\\delta$ 는 명령 + {t_axis * 1e3:.1f} ms 를 폐쇄 완료로 본 축이다)"
            )
    if "corridor" in ident:
        corridor = ident["corridor"]
        limited = " (스캔 끝에 닿음)" if corridor.scan_limited else ""
        out.append(
            f"- corridor: `r_ent` {_mm(corridor.r_ent)} mm, `tan_theta` {corridor.tan_theta:.3f} "
            f"(gap 0 – {_mm(corridor.length)} mm){limited}"
        )
    out += [
        "",
        "| $c_{lo}$ | $c_{hi}$ | $s_{ent}$ [mm] | $\\delta_{lo}$ [ms] | $\\delta_{hi}$ [ms] | 폭 [ms] |",
        "|---|---|---|---|---|---|",
    ]
    for row in ident["rows"]:
        mark = " ←" if row == ident["widest"] else ""
        width = f"{row.width * 1e3:.0f}" if row.width > 0 else "빈다"
        out.append(
            f"| {row.c_lo:.1f} | {row.c_hi:.1f} | {_mm(row.s_ent)} | {_ms(row.delta_lo)} | "
            f"{_ms(row.delta_hi)} | {width}{mark} |"
        )
    out.append("")
    inside = fine.s_first[box.i0 : box.i1 + 1, box.j0 : box.j1 + 1]
    out.append(
        "- 닫히는 손의 첫 접촉 높이 (상자 안, 속도별 최대) [mm]: "
        + ", ".join(
            f"{fine.c[box.i0 + i]:.2f}: {_mm(np.nanmax(inside[i]))}"
            for i in range(inside.shape[0])
        )
    )
    if "verify" in ident:
        check = ident["verify"]
        out.append(
            f"- **검증**: {check['n']} 조건 가운데 유지 {check['held']} "
            f"({check['why']}), 유지율의 {CONFIDENCE * 100:.0f} % 하한 {check['lower']:.4f}. "
            f"$\\rho$ 를 정의한 높이 {_mm(check['s_pass'])} mm, 손보다 먼저 다른 것에 닿은 시행 "
            f"{check['stray']}"
        )
    if ident.get("accel"):
        out += [
            "",
            "같은 조건을 공이 catch frame 에서 가속하는 채로 (직선 비행과 같은 폐쇄 시각):",
            "",
            "| 상대 가속도 | catch frame 성분 [m/s²] | 조건 | 유지 | 하한 | 직선에서만 유지 "
            "| 여기서만 유지 | 사유 |",
            "|---|---|---|---|---|---|---|---|",
        ]
        for row in ident["accel"]:
            vector = row["accel"]
            shown = "—" if vector is None else "(" + ", ".join(f"{v:+.2f}" for v in vector) + ")"
            out.append(
                f"| {row['label']} | {shown} | {row['n']} | {row['held']} | {row['lower']:.4f} | "
                f"{row['lost']} | {row['gained']} | {row['why']} |"
            )
    return out + [""]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("profile", help="a robot profile under the bring-up's config/")
    args = parser.parse_args(argv)
    directory = data_dir(args.profile)
    text = render(args.profile, directory)
    (directory / "report.md").write_text(text + "\n")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
