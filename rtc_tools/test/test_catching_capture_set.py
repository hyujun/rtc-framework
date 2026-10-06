"""rtc_tools.analysis.catching_capture_set: every answer is planted in the test.

No simulator and no recorded run: the inputs are shapes whose result can be
read off (a rectangle of held cells, a wall beside a palm, a cone), and where
the rule is too long to read off, a slow reference written the obvious way.
"""

import math

import numpy as np
import pytest

from rtc_tools.analysis import catching_capture_set as cs

MM = 1e-3


# ── Lattices ──────────────────────────────────────────────────────────────────


def test_lattice_step_wants_a_uniform_ascending_lattice():
    assert cs.lattice_step([0.5, 0.6, 0.7]) == pytest.approx(0.1)
    for bad in ([0.5], [0.5, 0.6, 0.8], [0.7, 0.6, 0.5], [0.5, 0.5]):
        with pytest.raises(ValueError):
            cs.lattice_step(bad)


def test_a_cell_is_held_only_when_every_trial_was():
    counts = np.array([[4, 3], [0, 4]])
    assert cs.held_cells(counts, 4).tolist() == [[True, False], [False, True]]
    # A cell nobody ran is not held, whatever its count says.
    trials = np.array([[4, 4], [0, 0]])
    assert cs.held_cells(np.array([[4, 4], [0, 0]]), trials).tolist() == [
        [True, True],
        [False, False],
    ]
    with pytest.raises(ValueError):
        cs.held_cells(np.array([5]), 4)


# ── Boxes of the (c, delta_o) map ─────────────────────────────────────────────

C = [0.5, 0.6, 0.7, 0.8]  # step 0.1 m/s
D = [-0.008, -0.004, 0.0, 0.004, 0.008, 0.012]  # step 4 ms


def _map(rows):
    return np.array([[ch == "H" for ch in row] for row in rows])


def test_box_bounds_are_cell_edges():
    held = _map(["......", ".HHH..", ".HHH..", "......"])
    boxes = cs.held_boxes(C, D, held)
    whole = [b for b in boxes if (b.i0, b.i1, b.j0, b.j1) == (1, 2, 1, 3)]
    assert len(whole) == 1
    box = whole[0]
    assert (box.c_lo, box.c_hi) == pytest.approx((0.55, 0.75))
    assert (box.delta_o_lo, box.delta_o_hi) == pytest.approx((-0.006, 0.006))
    assert box.width == pytest.approx(0.012)
    assert box.area == pytest.approx(0.2 * 0.012)


def test_one_cell_that_is_not_held_splits_the_box():
    held = _map(["HHHHHH", "HHH.HH", "HHHHHH", "HHHHHH"])
    spans = {(b.i0, b.i1, b.j0, b.j1) for b in cs.held_boxes(C, D, held)}
    # Across the split row only the two sides remain...
    assert (0, 3, 0, 2) in spans and (0, 3, 4, 5) in spans
    assert not any(i0 <= 1 <= i1 and j0 <= 3 <= j1 for i0, i1, j0, j1 in spans)
    # ...and the rows that do not contain it keep their full width.
    assert (2, 3, 0, 5) in spans and (0, 0, 0, 5) in spans


def test_the_box_is_asked_for_by_width_then_pushed_up_in_speed():
    #        j: 0 1 2 3 4 5        c
    held = _map(
        [
            "HHHHHH",  # 0.5 : six cells = 24 ms
            "HHHHHH",  # 0.6
            ".HHH..",  # 0.7 : three cells = 12 ms
            "..H...",  # 0.8 : one cell = 4 ms
        ]
    )
    boxes = cs.held_boxes(C, D, held)
    # 4 ms: the single cell at the top speed. Its column goes down to 0.5.
    b = cs.box_by_min_width(boxes, 0.004)
    assert (b.i0, b.i1, b.j0, b.j1) == (0, 3, 2, 2)
    # 12 ms: c_hi falls to the 0.7 row; the largest such box spans 0.5 – 0.7.
    b = cs.box_by_min_width(boxes, 0.012)
    assert (b.i0, b.i1, b.j0, b.j1) == (0, 2, 1, 3)
    assert b.c_hi == pytest.approx(0.75)
    # 20 ms: only the two full rows are that wide.
    b = cs.box_by_min_width(boxes, 0.020)
    assert (b.i0, b.i1, b.j0, b.j1) == (0, 1, 0, 5)
    # 40 ms: nothing.
    assert cs.box_by_min_width(boxes, 0.040) is None
    assert cs.box_candidates(C, D, held, (0.012, 0.040)) == {
        0.012: cs.box_by_min_width(boxes, 0.012),
        0.040: None,
    }


def test_equal_boxes_are_told_apart_the_same_way_every_time():
    # Two disjoint boxes of the same top speed and the same area.
    held = _map(["......", "......", "HH..HH", "HH..HH"])
    b = cs.box_by_min_width(cs.held_boxes(C, D, held), 0.008)
    assert (b.j0, b.j1) == (0, 1)  # the earlier one


def test_a_width_exactly_on_the_request_counts():
    held = _map(["HHHHH.", "......", "......", "......"])  # 5 x 4 ms = 20 ms in float steps
    assert cs.box_by_min_width(cs.held_boxes(C, D, held), 0.020) is not None


def test_held_boxes_checks_the_shape():
    with pytest.raises(ValueError):
        cs.held_boxes(C, D, np.zeros((3, 6), dtype=bool))


# ── The window on the planner's axis ──────────────────────────────────────────


def test_entrance_window_is_the_box_shifted_by_the_flight_over_s_ent():
    lo, hi = cs.entrance_window(0.5, 1.5, -0.02, 0.10, 0.062)
    assert lo == pytest.approx(-0.02 + 0.062 / 0.5)
    assert hi == pytest.approx(0.10 + 0.062 / 1.5)
    assert hi - lo == pytest.approx(0.12 - 0.062 * (1 / 0.5 - 1 / 1.5))


def test_every_delta_of_the_window_is_inside_the_box_for_every_speed():
    box = (0.4, 2.0, -0.03, 0.05)
    for s_ent in (-0.01, 0.0, 0.02):
        lo, hi = cs.entrance_window(*box, s_ent)
        assert hi > lo
        for c in np.linspace(box[0], box[1], 9):
            for delta in (lo, hi):
                delta_o = delta - s_ent / c
                assert box[2] - 1e-12 <= delta_o <= box[3] + 1e-12
        # And it is the WHOLE of it: just beyond either end some speed leaves the box.
        if s_ent != 0.0:
            c_at = (box[0], box[1]) if s_ent > 0 else (box[1], box[0])
            assert lo - 1e-6 - s_ent / c_at[0] < box[2]
            assert hi + 1e-6 - s_ent / c_at[1] > box[3]
    assert cs.entrance_window(*box, 0.0) == pytest.approx((box[2], box[3]))


def test_an_empty_window_is_returned_not_hidden():
    lo, hi = cs.entrance_window(0.25, 3.0, 0.0, 0.02, 0.08)
    assert hi < lo  # 20 ms box, the plane costs 0.08 (4 − 1/3) = 293 ms
    with pytest.raises(ValueError):
        cs.entrance_window(0.0, 1.0, 0.0, 0.02, 0.05)
    with pytest.raises(ValueError):
        cs.entrance_window(1.5, 1.0, 0.0, 0.02, 0.05)


def test_entrance_rows_raise_c_lo_and_keep_c_hi():
    box = cs.Box(0.45, 0.85, -0.01, 0.03, 0, 3, 0, 9)
    edges = [0.45, 0.55, 0.65, 0.75, 0.85, 0.95]
    rows = cs.entrance_rows(box, edges, lambda c_lo: 0.05)
    assert [r.c_lo for r in rows] == pytest.approx([0.45, 0.55, 0.65, 0.75])
    assert all(r.c_hi == box.c_hi for r in rows)
    widths = [r.width for r in rows]
    assert widths == sorted(widths)  # constant s_ent: the window only widens
    assert rows[0].width == pytest.approx(0.04 - 0.05 * (1 / 0.45 - 1 / 0.85))
    assert cs.widest_entrance_row(rows) == rows[-1]
    assert cs.widest_entrance_row([]) is None


def test_entrance_rows_ask_for_s_ent_at_each_lower_edge():
    box = cs.Box(0.45, 0.85, -0.01, 0.03, 0, 3, 0, 9)
    asked = []

    def s_ent_of(c_lo):
        asked.append(c_lo)
        return 0.1 if c_lo < 0.6 else 0.0  # a low c_lo tilts the approach and lifts the plane

    rows = cs.entrance_rows(box, [0.45, 0.55, 0.65, 0.75], s_ent_of)
    assert asked == pytest.approx([0.45, 0.55, 0.65, 0.75])
    best = cs.widest_entrance_row(rows)
    # Of the two full-width rows (s_ent 0) the one keeping more speeds.
    assert best.c_lo == pytest.approx(0.65) and best.width == pytest.approx(0.04)


# ── Lateral set ───────────────────────────────────────────────────────────────

STEP = 5 * MM
XS = np.arange(-4, 5) * STEP  # −20 … +20 mm
YS = np.arange(-4, 5) * STEP


def _held(predicate):
    return np.array([[bool(predicate(x, y)) for y in YS] for x in XS])


def _rect(x0, x1, y0, y1):
    return _held(lambda x, y: x0 - 1e-9 <= x <= x1 + 1e-9 and y0 - 1e-9 <= y <= y1 + 1e-9)


def test_inscribed_circle_of_a_rectangle_of_cells():
    held = _rect(-10 * MM, 10 * MM, -5 * MM, 5 * MM)  # cells cover ±12.5 x ±7.5 mm
    centre, radius = cs.inscribed_circle(XS, YS, held)
    assert radius == pytest.approx(7.5 * MM)
    # (−5, 0), (0, 0) and (5, 0) all give 7.5 mm: the one at the catch point.
    assert centre == pytest.approx((0.0, 0.0))


def test_inscribed_circle_ties_go_to_the_point_nearest_the_origin():
    held = _rect(5 * MM, 15 * MM, 0.0, 10 * MM)
    centre, radius = cs.inscribed_circle(XS, YS, held)
    assert radius == pytest.approx(7.5 * MM)
    assert centre == pytest.approx((10 * MM, 5 * MM))  # the only cell 7.5 mm from every edge
    held = _rect(5 * MM, 20 * MM, 0.0, 10 * MM)  # (10, 5) and (15, 5) tie
    assert cs.inscribed_circle(XS, YS, held)[0] == pytest.approx((10 * MM, 5 * MM))


def test_what_was_not_measured_is_not_held():
    xs = ys = np.arange(-1, 2) * STEP
    centre, radius = cs.inscribed_circle(xs, ys, np.ones((3, 3), dtype=bool))
    assert centre == pytest.approx((0.0, 0.0))
    assert radius == pytest.approx(1.5 * STEP)  # the ring of cells around the lattice
    assert cs.inscribed_circle(xs, ys, np.zeros((3, 3), dtype=bool)) is None
    assert cs.capture_polygon(xs, ys, np.zeros((3, 3), dtype=bool)) is None


def _inside_held(points, held):
    ix = np.rint((points[:, 0] - XS[0]) / STEP).astype(int)
    iy = np.rint((points[:, 1] - YS[0]) / STEP).astype(int)
    ok = (ix >= 0) & (ix < XS.size) & (iy >= 0) & (iy < YS.size)
    out = np.zeros(points.shape[0], dtype=bool)
    out[ok] = held[ix[ok], iy[ok]]
    return out


def _dense(polygon, n=120):
    corners = polygon.vertices()
    lo, hi = corners.min(axis=0), corners.max(axis=0)
    gx, gy = np.meshgrid(np.linspace(lo[0], hi[0], n), np.linspace(lo[1], hi[1], n))
    points = np.stack([gx.ravel(), gy.ravel()], axis=1)
    # Strictly inside: a point ON a face may sit on a cell boundary.
    return points[polygon.contains(points, slack=-1e-6)]


def test_capture_polygon_fills_a_rectangle_to_within_one_step_of_growth():
    held = _rect(-10 * MM, 10 * MM, -5 * MM, 5 * MM)
    grow = 1 * MM
    poly = cs.capture_polygon(XS, YS, held, n_faces=8, grow=grow)
    assert poly.normals.shape == (8, 2)
    assert np.linalg.norm(poly.normals, axis=1) == pytest.approx(np.ones(8))
    # Faces 0, 2, 4, 6 are ±x, ±y: each within one growth step of the edge.
    for face, edge in ((0, 12.5 * MM), (4, 12.5 * MM), (2, 7.5 * MM), (6, 7.5 * MM)):
        assert edge - grow < poly.offsets[face] <= edge + 1e-9
    assert _inside_held(_dense(poly), held).all()
    assert poly.inradius_about((0.0, 0.0)) > 7.5 * MM * math.cos(math.pi / 8) - 1e-9


def test_capture_polygon_starts_inside_its_circle():
    # One cell missing 26 degrees off the x axis sets the circle's radius. The
    # octagon AROUND that circle has a corner inside the missing cell; the one
    # inside it does not.
    held = _rect(-10 * MM, 10 * MM, -10 * MM, 10 * MM)
    held[np.argmin(abs(XS - 10 * MM)), np.argmin(abs(YS - 5 * MM))] = False
    centre, radius = cs.inscribed_circle(XS, YS, held)
    assert centre == pytest.approx((0.0, 0.0))
    assert radius == pytest.approx(math.hypot(7.5 * MM, 2.5 * MM))
    poly = cs.capture_polygon(XS, YS, held, grow=0.1 * MM)
    assert _inside_held(_dense(poly, n=400), held).all()


def test_capture_polygon_stays_out_of_the_notch_of_an_l_shape():
    held = _held(lambda x, y: (x <= 5 * MM + 1e-9 or y <= 0.0 + 1e-9) and abs(x) < 16 * MM)
    held &= _rect(-15 * MM, 15 * MM, -15 * MM, 15 * MM)
    poly = cs.capture_polygon(XS, YS, held)
    dense = _dense(poly)
    assert dense.shape[0] > 1000
    assert _inside_held(dense, held).all()
    assert not poly.contains(np.array([[12 * MM, 12 * MM]]))[0]  # the notch


def test_capture_polygon_ends_on_a_strip_one_cell_wide():
    # The diagonal faces are cut off by the strip's long sides: pushing one of
    # them changes nothing, and the growth must still end.
    held = _rect(-10 * MM, 10 * MM, 0.0, 0.0)
    poly = cs.capture_polygon(XS, YS, held)
    assert _inside_held(_dense(poly), held).all()
    corners = poly.vertices()
    # Every face touches the set it bounds.
    assert np.max(corners @ poly.normals.T, axis=0) == pytest.approx(poly.offsets, abs=1e-9)
    assert poly.offsets[2] <= 2.5 * MM + 1e-9 and poly.offsets[6] <= 2.5 * MM + 1e-9


def test_polygon_geometry():
    square = cs.Polygon(
        np.array([[1.0, 0.0], [-1.0, 0.0], [0.0, 1.0], [0.0, -1.0]]),
        np.array([0.02, 0.01, 0.03, 0.0]),
    )
    assert square.area == pytest.approx(0.03 * 0.03)
    assert square.contains(np.array([[0.0, 0.01], [0.021, 0.01], [0.0, -0.001]])).tolist() == [
        True,
        False,
        False,
    ]
    assert square.inradius_about((0.005, 0.015)) == pytest.approx(0.015)
    assert square.inradius_about((0.03, 0.015)) == pytest.approx(-0.01)
    assert square.meets_square(0.0, 0.0, 0.005)
    assert not square.meets_square(0.025, 0.01, 0.005)  # shares an edge only
    assert not square.meets_square(0.05, 0.05, 0.005)
    with pytest.raises(ValueError):
        cs.capture_polygon(XS, YS, _rect(0, 0, 0, 0), n_faces=2)


# ── Lateral speed ─────────────────────────────────────────────────────────────


def test_largest_held_radius_stops_at_the_first_cell_with_a_failure():
    mags = [0.05, 0.10, 0.15, 0.20, 0.25]
    held = np.ones((5, 8, 3), dtype=bool)  # magnitude x direction x condition
    assert cs.largest_held_radius(mags, held) == pytest.approx(0.275)
    held[3, 6, 1] = False  # one direction of one condition at 0.20
    assert cs.largest_held_radius(mags, held) == pytest.approx(0.175)
    held[4] = True  # a cell held BEYOND the failure does not count
    assert cs.largest_held_radius(mags, held) == pytest.approx(0.175)
    held[0, 0, 0] = False
    assert cs.largest_held_radius(mags, held) == 0.0
    with pytest.raises(ValueError):
        cs.largest_held_radius(mags, np.ones((4, 8), dtype=bool))


# ── Static contact field ──────────────────────────────────────────────────────

OX = np.arange(-8, 9) * STEP  # −40 … +40 mm
OY = np.arange(-8, 9) * STEP
OS = np.arange(-5, 121) * MM  # −5 … +120 mm


def _field(predicate):
    contact = np.zeros((OX.size, OY.size, OS.size), dtype=bool)
    for i, x in enumerate(OX):
        for j, y in enumerate(OY):
            for k, s in enumerate(OS):
                contact[i, j, k] = bool(predicate(x, y, s))
    return cs.Occupancy(OX, OY, OS, contact)


# A palm (everything at or below s = 0) and a wall 50 mm high at x >= 20 mm.
PALM_AND_WALL = _field(lambda x, y, s: s <= 1e-9 or (x >= 20 * MM - 1e-9 and s <= 50 * MM + 1e-9))


def _reference_entrance(occupancy, points, slopes):
    """The rule written the slow way: lower the plane while every line is free."""
    xs, ys, ss = occupancy.xs, occupancy.ys, occupancy.ss
    contact = occupancy.contact

    def columns(value, lattice):
        f = (value - lattice[0]) / STEP
        if f < -1e-6 or f > lattice.size - 1 + 1e-6:
            return None
        if abs(f - round(f)) < 1e-6:
            return [int(round(f))]
        return [int(math.floor(f)), int(math.floor(f)) + 1]

    def free(n):
        for px, py in points:
            for kx, ky in slopes:
                for m in range(n, ss.size):
                    rise = ss[m] - ss[n]
                    cx, cy = columns(px + kx * rise, xs), columns(py + ky * rise, ys)
                    if cx is None or cy is None:
                        return False
                    if any(contact[i, j, m] for i in cx for j in cy):
                        return False
        return True

    level = None
    for n in range(ss.size - 1, -1, -1):
        if not free(n):
            break
        level = n
    return None if level is None else float(ss[level])


def test_the_plane_sits_one_level_above_the_highest_contact_on_a_straight_approach():
    axis = cs.slope_fan(0.0)
    assert axis.tolist() == [[0.0, 0.0]]
    over_palm = cs.approach_table(PALM_AND_WALL, np.array([[0.0, 0.0]]), axis).entrance(0.0)
    assert over_palm.holds and over_palm.s_ent == pytest.approx(1 * MM)
    assert not over_palm.scan_limited
    over_wall = cs.approach_table(
        PALM_AND_WALL, np.array([[0.0, 0.0], [20 * MM, 0.0]]), axis
    ).entrance(0.0)
    assert over_wall.s_ent == pytest.approx(51 * MM)


def test_a_tilted_approach_lifts_the_plane():
    points = np.array([[0.0, 0.0]])
    fan = cs.slope_fan(0.5, tan_step=0.1)
    assert fan.shape == (1 + 5 * 8, 2)
    table = cs.approach_table(PALM_AND_WALL, points, fan)
    heights = [table.entrance(t).s_ent for t in (0.0, 0.1, 0.3, 0.5)]
    assert heights == sorted(heights)  # a wider fan can only raise it
    assert heights[0] == pytest.approx(1 * MM)
    # Tilt 0.5 toward +x: the line is over a column that includes the wall once
    # it is more than 15 mm out, 31 mm above its anchor. The wall is 50 mm
    # high, so the anchors that CONTACT blocks are those up to 19 mm.
    toward_wall = int(np.argmin(np.abs(fan - np.array([0.5, 0.0])).sum(axis=1)))
    assert OS[table.hit[toward_wall]].max() == pytest.approx(19 * MM)
    # ...but at that tilt the line also leaves this 40 mm scan 81 mm up, which
    # counts as touching: anchors up to 39 mm are stopped by that, and said so.
    widest = table.entrance(0.5)
    assert widest.s_ent == pytest.approx(40 * MM) and widest.scan_limited
    assert not table.entrance(0.3).scan_limited
    for tan_max in (0.0, 0.2, 0.5):
        slopes = cs.slope_fan(tan_max, tan_step=0.1)
        assert table.entrance(tan_max).s_ent == pytest.approx(
            _reference_entrance(PALM_AND_WALL, points, slopes)
        )


def test_the_table_agrees_with_the_slow_rule_on_an_uneven_field():
    rng = np.random.default_rng(7)
    bumps = rng.uniform(0.0, 60 * MM, size=(OX.size, OY.size))
    field = _field(lambda x, y, s: s <= bumps[int(round(x / STEP)) + 8, int(round(y / STEP)) + 8])
    points = np.array([[0.0, 0.0], [5 * MM, -5 * MM], [-10 * MM, 0.0], [2.5 * MM, 7.5 * MM]])
    fan = cs.slope_fan(0.2, tan_step=0.1)
    table = cs.approach_table(field, points, fan)
    for tan_max in (0.0, 0.1, 0.2):
        got = table.entrance(tan_max)
        want = _reference_entrance(field, points, cs.slope_fan(tan_max, tan_step=0.1))
        assert got.holds and got.s_ent == pytest.approx(want)


def test_the_plane_is_lowered_from_the_top_and_stops_at_the_first_blocked_level():
    # A post 10 mm off the axis, 30 – 40 mm up, and nothing else. At tilt 0.5 a
    # line is over the post's column between 11 and 29 mm above its anchor, so
    # anchors from 1 to 29 mm are blocked — and the ones BELOW them are free
    # again. Those do not count: the plane stops at 30 mm.
    post = _field(
        lambda x, y, s: abs(x - 10 * MM) < 1e-9 and 30 * MM - 1e-9 <= s <= 40 * MM + 1e-9
    )
    post = cs.Occupancy(OX, OY, OS[:60], post.contact[:, :, :60])  # too low to leave the scan
    points = np.array([[0.0, 0.0]])
    slopes = np.array([[0.5, 0.0]])
    table = cs.approach_table(post, points, slopes)
    blocked = OS[:60][table.hit[0]]
    assert (blocked.min(), blocked.max()) == pytest.approx((1 * MM, 29 * MM))
    assert not table.hit[0, 0] and not table.outside.any()
    got = table.entrance(0.5, tan_step=0.5)
    assert got.s_ent == pytest.approx(30 * MM) and got.limit_slopes == ((0.5, 0.0),)
    assert got.s_ent == pytest.approx(_reference_entrance(post, points, slopes))


def test_tilts_are_rounded_up_to_the_next_ring():
    assert cs.slope_fan(0.25, tan_step=0.1).shape == (1 + 3 * 8, 2)
    assert cs.slope_fan(0.30, tan_step=0.1).shape == (1 + 3 * 8, 2)
    table = cs.approach_table(PALM_AND_WALL, np.array([[0.0, 0.0]]), cs.slope_fan(0.3))
    assert table.entrance(0.25).s_ent == table.entrance(0.3).s_ent
    # ...and the ring it rounds to is the one that decides.
    wide = cs.approach_table(PALM_AND_WALL, np.array([[0.0, 0.0]]), cs.slope_fan(0.5))
    assert wide.entrance(0.45).s_ent == wide.entrance(0.5).s_ent
    assert wide.entrance(0.4).s_ent < wide.entrance(0.45).s_ent
    with pytest.raises(ValueError):
        table.entrance(0.31)
    with pytest.raises(ValueError):
        cs.slope_fan(-0.1)


def test_a_field_touching_at_the_top_supports_no_plane():
    solid = _field(lambda x, y, s: True)
    got = cs.approach_table(solid, np.array([[0.0, 0.0]]), cs.slope_fan(0.0)).entrance(0.0)
    assert not got.holds and math.isnan(got.s_ent)
    assert got.limit_slopes == ((0.0, 0.0),) and not got.scan_limited


def test_leaving_the_lateral_scan_counts_as_touching_and_is_flagged():
    empty = _field(lambda x, y, s: s <= -5 * MM + 1e-9)  # contact on the lowest level only
    # From x = 30 mm at tilt 1.0 toward +x the line leaves the scan (40 mm) 11 mm up.
    fan = cs.slope_fan(1.0, tan_step=0.5)
    got = cs.approach_table(empty, np.array([[30 * MM, 0.0]]), fan).entrance(1.0)
    assert got.holds and got.scan_limited
    # Anchors within 10 mm of the top never rise far enough to leave.
    assert got.s_ent == pytest.approx(OS[-1] - 10 * MM)
    straight = cs.approach_table(empty, np.array([[30 * MM, 0.0]]), fan).entrance(0.0)
    assert straight.s_ent == pytest.approx(-4 * MM) and not straight.scan_limited


def test_levels_above_the_hand_are_cut_off():
    cut = cs.trim_above_contact(PALM_AND_WALL)
    assert cut.ss[-1] == pytest.approx(51 * MM)  # one level above the wall
    assert not cut.contact[:, :, -1].any()
    assert np.array_equal(cut.contact, PALM_AND_WALL.contact[:, :, : cut.ss.size])
    # The same plane, and the tilted line no longer has room to leave the scan.
    points = np.array([[0.0, 0.0]])
    fan = cs.slope_fan(0.5, tan_step=0.1)
    got = cs.approach_table(cut, points, fan).entrance(0.5)
    assert got.s_ent == pytest.approx(20 * MM) and not got.scan_limited
    assert cs.trim_above_contact(_field(lambda x, y, s: False)).ss.size == 2
    with pytest.raises(ValueError):
        cs.trim_above_contact(_field(lambda x, y, s: True))  # contact on the top level


def test_occupancy_checks_its_shape():
    with pytest.raises(ValueError):
        cs.Occupancy(OX, OY, OS, np.zeros((3, 3, 3), dtype=bool))
    with pytest.raises(ValueError):
        cs.approach_table(PALM_AND_WALL, np.zeros((0, 2)), cs.slope_fan(0.0))


# ── Corridor ──────────────────────────────────────────────────────────────────


def _lattice_free(occupancy, corridor, s_ent):
    """No touching lattice point lies inside the fitted cone."""
    for k, s in enumerate(occupancy.ss):
        if s < s_ent - 1e-9 or s > s_ent + corridor.length + 1e-9:
            continue
        reach = corridor.r_ent + (s - s_ent) * corridor.tan_theta
        ix, iy = np.nonzero(occupancy.contact[:, :, k])
        if np.any(np.hypot(occupancy.xs[ix], occupancy.ys[iy]) < reach - 1e-9):
            return False
    return True


def test_corridor_of_a_cone_shaped_opening():
    # Free where |rho| <= 15 mm + 0.25 s (and above the palm).
    cone = _field(lambda x, y, s: s <= 1e-9 or math.hypot(x, y) > 15 * MM + 0.25 * s)
    s_ent = 10 * MM
    fit = cs.corridor_fit(cone, s_ent, length=60 * MM)
    assert fit.length == pytest.approx(60 * MM)
    assert _lattice_free(cone, fit, s_ent)
    # Inside the cone it was built from, and not far inside: the lattice costs
    # at most a cell of radius, and the tilt comes back.
    for s in (s_ent, s_ent + 30 * MM, s_ent + 60 * MM):
        true_radius = 15 * MM + 0.25 * s
        fitted = fit.r_ent + (s - s_ent) * fit.tan_theta
        assert true_radius - 1.5 * STEP < fitted <= true_radius + 1e-9
    assert 0.15 < fit.tan_theta <= 0.25 + 1e-9
    assert not fit.scan_limited


def test_a_corridor_that_narrows_outward_becomes_a_cylinder():
    # An overhang: free radius 30 mm near the plane, 10 mm from 40 mm up.
    overhang = _field(
        lambda x, y, s: s <= 1e-9 or math.hypot(x, y) > (30 * MM if s < 40 * MM else 10 * MM)
    )
    fit = cs.corridor_fit(overhang, 5 * MM, length=80 * MM)
    assert fit.tan_theta == 0.0
    assert fit.r_ent == pytest.approx(
        cs.free_radius(overhang, int(np.argmin(abs(OS - 60 * MM))))[0]
    )
    assert fit.r_ent < 10 * MM + 1e-9
    assert _lattice_free(overhang, fit, 5 * MM)


def test_free_radius_says_when_the_scan_ends_first():
    open_above = _field(lambda x, y, s: s <= 1e-9)
    radius, limited = cs.free_radius(open_above, int(np.argmin(abs(OS - 20 * MM))))
    assert limited and radius == pytest.approx(42.5 * MM)  # the lattice edge, a half cell out
    radius, limited = cs.free_radius(open_above, 0)
    assert not limited and radius == 0.0  # the palm itself
    assert cs.corridor_fit(open_above, 10 * MM, 20 * MM).scan_limited
    with pytest.raises(ValueError):
        cs.corridor_fit(open_above, 0.5, 0.02)


# ── Verification sample ───────────────────────────────────────────────────────


def test_the_sample_lies_in_the_set_it_is_drawn_from():
    poly = cs.capture_polygon(XS, YS, _rect(-10 * MM, 10 * MM, -5 * MM, 5 * MM))
    box = cs.Box(0.45, 1.25, -0.012, 0.028, 0, 7, 0, 9)
    sample = cs.sample_capture_set(np.random.default_rng(741), poly, box, 0.2, 300)
    assert sample.shape == (300, 6)
    assert poly.contains(sample[:, 0:2]).all()
    assert ((sample[:, 2] >= box.c_lo) & (sample[:, 2] <= box.c_hi)).all()
    assert ((sample[:, 3] >= box.delta_o_lo) & (sample[:, 3] <= box.delta_o_hi)).all()
    assert (np.hypot(sample[:, 4], sample[:, 5]) <= 0.2 + 1e-12).all()
    # It uses the set, not a corner of it.
    assert np.ptp(sample[:, 0]) > 15 * MM and np.ptp(sample[:, 2]) > 0.7
    assert np.hypot(sample[:, 4], sample[:, 5]).max() > 0.18
    again = cs.sample_capture_set(np.random.default_rng(741), poly, box, 0.2, 300)
    assert np.array_equal(sample, again)


def test_an_empty_polygon_is_refused():
    empty = cs.Polygon(np.array([[1.0, 0.0], [-1.0, 0.0]]), np.array([-0.01, -0.01]))
    box = cs.Box(0.5, 1.0, 0.0, 0.02, 0, 0, 0, 0)
    with pytest.raises(ValueError):
        cs.sample_capture_set(np.random.default_rng(1), empty, box, 0.1, 10)


# ── Clopper–Pearson ───────────────────────────────────────────────────────────


def _tail(successes, n, p):
    return sum(math.comb(n, i) * p**i * (1 - p) ** (n - i) for i in range(successes, n + 1))


def test_no_failure_gives_the_closed_form():
    assert cs.clopper_pearson_lower(300, 300) == pytest.approx(0.05 ** (1 / 300))
    assert cs.clopper_pearson_lower(300, 300) > 0.99
    assert cs.clopper_pearson_lower(59, 59) > 0.95 > cs.clopper_pearson_lower(58, 58)
    assert cs.clopper_pearson_lower(0, 10) == 0.0


@pytest.mark.parametrize(("successes", "n"), [(299, 300), (290, 300), (7, 10), (1, 40)])
def test_the_bound_is_the_rate_at_which_the_result_has_probability_alpha(successes, n):
    for confidence in (0.95, 0.99):
        bound = cs.clopper_pearson_lower(successes, n, confidence)
        assert _tail(successes, n, bound) == pytest.approx(1 - confidence, rel=1e-6)


def test_the_bound_falls_with_every_failure():
    bounds = [cs.clopper_pearson_lower(k, 300) for k in (300, 299, 298, 290)]
    assert bounds == sorted(bounds, reverse=True)
    for bad in ((-1, 10), (11, 10), (0, 0)):
        with pytest.raises(ValueError):
            cs.clopper_pearson_lower(*bad)
    with pytest.raises(ValueError):
        cs.clopper_pearson_lower(5, 10, confidence=1.0)
