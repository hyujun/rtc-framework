"""catch_throw_design (dynamic_catching L3 §4.1).

Pinned here: the grid's ids and the refinement between its points, the batch
integrator and its reductions against the one-flight functions of
``catchability_map``, the speed solve against the drag-free closed form and
against the flight it designs, each screen label on a throw built to get it,
and the files (the design CSV, the throw list a selection writes).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from rtc_tools.analysis import (
    catch_throw_design as d,
    catchability_map as cm,
    catching_throw_list as ctl,
)

SOURCES = dict.fromkeys(("radius_m", "mass_kg", "drag_coefficient", "air_density_kg_m3"), "test")
BALL = cm.BallParams(0.0335, 0.057, 0.55, 1.204, SOURCES)
NO_DRAG = cm.BallParams(0.0335, 0.057, 0.0, 0.0, SOURCES)
G = -cm.GRAVITY_W_M_S2[2]

GRID = d.DesignGrid(
    {
        "distance_m": (2.0, 2.5, 3.0),
        "azimuth_deg": (-20.0, 0.0, 20.0),
        "release_height_m": (1.0, 1.2, 1.4),
        "pass_dx_m": (-0.2, 0.0, 0.2),
        "pass_dy_m": (0.0, 0.2),
        "elevation_deg": (45.0, 60.0),
    }
)
CATCH_POINT = np.array([0.05, -0.1, 1.5])


def _constants(**over) -> d.ScreenConstants:
    base = {
        "catch_point_w": CATCH_POINT,
        "approach_axis_w": np.array([1.0, 0.0, 0.0]),
        "reach_centre_w": np.array([0.0, 0.0, 0.9]),
        "reach_radius_m": 1.1,
        "detection_delay_s": 0.15,
        "lead_floor_s": 0.3,
        "body_vertices_w": np.array([[0.0, 0.0, 0.9], [0.0, 0.0, 1.3]]),
        "body_flag_radius_m": 0.15,
        "apex_margin_flag_m": 0.05,
    }
    return d.ScreenConstants(**{**base, **over})


def _design(ids, values, ball=BALL, **over):
    kwargs = {
        "origin_xy_m": (0.0, 0.0),
        "pass_plane_z_m": float(CATCH_POINT[2]),
        "constants": _constants(),
        "ball": ball,
        "speed_max_m_s": 12.0,
        "speed_cap_m_s": 48.0,
        "horizon_s": 2.5,
        "step_s": 0.002,
    }
    return d.design_throws(np.asarray(ids), values, **{**kwargs, **over})


def _one(**values) -> dict[str, np.ndarray]:
    base = {
        "distance_m": 3.0,
        "azimuth_deg": 10.0,
        "release_height_m": 1.2,
        "pass_dx_m": 0.1,
        "pass_dy_m": -0.1,
        "elevation_deg": 45.0,
    }
    return {axis: np.array([float({**base, **values}[axis])]) for axis in d.DESIGN_AXES}


# ── the grid ──────────────────────────────────────────────────────────────────


def test_an_axis_is_a_list_or_an_inclusive_range():
    assert d.axis_values("1.5:4.0:0.25") == [1.5 + 0.25 * i for i in range(11)]
    assert d.axis_values("-0.4:0.4:0.1")[4] == 0.0
    assert d.axis_values("45 60") == [45.0, 60.0]
    with pytest.raises(ValueError, match="whole number of steps"):
        d.axis_values("0:1:0.3")
    with pytest.raises(ValueError, match="lo:hi:step"):
        d.axis_values("0:1")


def test_a_throw_id_is_the_product_index_first_axis_outermost():
    assert GRID.shape == (3, 3, 3, 3, 2, 2) and GRID.size == 324
    ids = GRID.ids()
    assert ids.tolist() == list(range(324))
    values = GRID.values(ids)
    # the last axis turns fastest, the first slowest
    assert values["elevation_deg"][:4].tolist() == [45.0, 60.0, 45.0, 60.0]
    assert values["distance_m"][0] == 2.0 and values["distance_m"][-1] == 3.0
    assert values["distance_m"][108] == 2.5  # one step of the first axis = 324 / 3
    at = GRID.positions([108 + 36 + 12 + 4 + 2 + 1])
    assert at.tolist() == [[1, 1, 1, 1, 1, 1]]


def test_a_subset_keeps_the_grids_ids_and_refuses_a_value_off_the_grid():
    subset = {"distance_m": [2.0, 3.0], "release_height_m": [1.0, 1.4], "pass_dx_m": [0.0]}
    ids = GRID.ids(subset)
    assert ids.size == 2 * 3 * 2 * 1 * 2 * 2
    values = GRID.values(ids)
    assert set(values["distance_m"]) == {2.0, 3.0} and set(values["pass_dx_m"]) == {0.0}
    assert set(values["azimuth_deg"]) == {-20.0, 0.0, 20.0}
    with pytest.raises(ValueError, match="not a value of the design grid"):
        GRID.ids({"distance_m": [2.25]})
    with pytest.raises(ValueError, match="not a design axis"):
        GRID.ids({"speed_m_s": [5.0]})
    with pytest.raises(ValueError, match="ascending and distinct"):
        d.DesignGrid({**GRID.axes, "distance_m": (2.0, 2.0)})


def test_refinement_is_the_points_between_neighbours_judged_differently():
    coarse = {"distance_m": [2.0, 3.0], "release_height_m": [1.0, 1.4], "pass_dx_m": [-0.2, 0.2]}
    ids = GRID.ids(coarse)
    values = GRID.values(ids)
    # accepted exactly at distance 2.0: every pair across the distance axis differs
    accepted = {int(i): bool(r == 2.0) for i, r in zip(ids, values["distance_m"], strict=True)}
    found, pairs = d.refinement_ids(GRID, coarse, accepted)
    assert pairs["distance_m"] == ids.size // 2
    assert all(pairs[a] == 0 for a in d.DESIGN_AXES if a != "distance_m")
    between = GRID.values(found)
    assert set(between["distance_m"]) == {2.5}
    # the other axes stay on the coarse points of the pair
    assert set(between["release_height_m"]) == {1.0, 1.4}
    assert set(between["pass_dx_m"]) == {-0.2, 0.2}
    assert found.size == ids.size // 2
    # limited to an axis that has no differing pair: nothing
    none, _ = d.refinement_ids(GRID, coarse, accepted, axes=["pass_dx_m"])
    assert none.size == 0
    # a pair with an unjudged point is left out
    del accepted[int(ids[0])]
    fewer, pairs = d.refinement_ids(GRID, coarse, accepted)
    assert pairs["distance_m"] == ids.size // 2 - 1 and fewer.size == found.size - 1


def test_an_axis_with_two_adjacent_values_has_nothing_between():
    coarse = {"elevation_deg": [45.0, 60.0]}
    ids = GRID.ids(coarse)
    values = GRID.values(ids)
    accepted = {int(i): bool(e == 45.0) for i, e in zip(ids, values["elevation_deg"], strict=True)}
    found, pairs = d.refinement_ids(GRID, coarse, accepted)
    assert pairs["elevation_deg"] == ids.size // 2 and found.size == 0


# ── flights, many at once ─────────────────────────────────────────────────────


LAUNCHES = np.array(
    [
        [[3.0, 0.5, 1.2], [-4.0, -0.6, 4.5]],
        [[-2.0, 1.0, 1.7], [3.5, -1.5, 6.0]],
        [[1.0, -3.0, 1.0], [-1.0, 5.0, 2.0]],
        [[0.5, 0.5, 2.0], [0.2, 0.1, -1.0]],  # released downwards
    ]
)


def _batch(ball=BALL, horizon_s=1.5, step_s=0.002) -> d.FlightBatch:
    return d.integrate_flights(
        LAUNCHES[:, 0], LAUNCHES[:, 1], ball, horizon_s=horizon_s, step_s=step_s
    )


def test_the_batch_integrator_is_integrate_flight_on_every_launch():
    batch = _batch()
    for i, (p0, v0) in enumerate(LAUNCHES):
        one = cm.integrate_flight(p0, v0, BALL, horizon_s=1.5, step_s=0.002)
        assert batch.time_s.shape == one.time_s.shape
        assert np.array_equal(batch.time_s, one.time_s)
        assert np.abs(batch.position_m[:, i] - one.position_m).max() < 1e-9
        assert np.abs(batch.velocity_m_s[:, i] - one.velocity_m_s).max() < 1e-9


def test_closest_approaches_are_closest_approach_of_every_flight():
    batch = _batch()
    target = np.array([0.1, -0.1, 1.5])
    distance, at, velocity = d.closest_approaches(batch, target)
    for i in range(len(LAUNCHES)):
        want = cm.closest_approach(
            batch.time_s, batch.position_m[:, i], batch.velocity_m_s[:, i], target
        )
        assert distance[i] == pytest.approx(want[0], abs=1e-9)
        assert at[i] == pytest.approx(want[1], abs=1e-9)
        assert np.linalg.norm(velocity[i]) == pytest.approx(want[2], abs=1e-9)
    # one target per flight
    targets = np.array([[0.1, -0.1, 1.5], [0.0, 0.3, 1.4], [0.4, 0.0, 1.6], [0.5, 0.5, 1.0]])
    per_flight, _, _ = d.closest_approaches(batch, targets)
    for i, own in enumerate(targets):
        want, _, _ = cm.closest_approach(
            batch.time_s, batch.position_m[:, i], batch.velocity_m_s[:, i], own
        )
        assert per_flight[i] == pytest.approx(want, abs=1e-9)


def test_the_apex_is_the_drag_free_closed_form_and_release_for_a_downward_launch():
    batch = _batch(NO_DRAG)
    height, at = d.apexes(batch)
    for i, (p0, v0) in enumerate(LAUNCHES[:3]):
        assert height[i] == pytest.approx(p0[2] + v0[2] ** 2 / (2 * G), abs=1e-9)
        assert at[i] == pytest.approx(v0[2] / G, abs=1e-9)
    assert height[3] == LAUNCHES[3, 0, 2] and at[3] == 0.0
    # still rising at the end of a short horizon: the last sample
    short = d.integrate_flights(
        LAUNCHES[:1, 0], LAUNCHES[:1, 1], NO_DRAG, horizon_s=0.1, step_s=0.002
    )
    height, at = d.apexes(short)
    assert height[0] == short.position_m[-1, 0, 2] and at[0] == short.time_s[-1]


def test_the_last_instant_inside_is_where_the_flight_leaves_the_sphere():
    batch = _batch()
    centre, radius = np.array([0.0, 0.0, 0.9]), 1.1
    last, still = d.last_inside(batch, centre, radius)
    inside = np.linalg.norm(batch.position_m - centre, axis=2) <= radius
    for i, (p0, v0) in enumerate(LAUNCHES):
        if not inside[:, i].any():
            assert math.isnan(last[i])
            continue
        assert not still[i]
        k = int(np.flatnonzero(inside[:, i])[-1])
        assert batch.time_s[k] <= last[i] <= batch.time_s[k + 1]
        # the flight is on the sphere there: the same flight carried to that instant
        steps = max(1, math.ceil(last[i] / 0.0005))
        fine = cm.integrate_flight(p0, v0, BALL, horizon_s=last[i], step_s=last[i] / steps)
        assert np.linalg.norm(fine.position_m[-1] - centre) == pytest.approx(radius, abs=1e-8)
    assert np.isfinite(last).sum() >= 2  # the launches do exercise it
    # a sphere the ball never leaves within the horizon: the horizon, flagged
    last, still = d.last_inside(batch, centre, 100.0)
    assert still.all() and (last == batch.time_s[-1]).all()


def test_the_distance_to_a_polyline_is_to_its_nearest_segment():
    line = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 1.0]])
    points = np.array([[0.3, 0.0, 0.5], [0.5, 0.4, 1.0], [0.0, 0.0, -0.2], [2.0, 0.0, 1.0]])
    assert d.polyline_distance(points, line) == pytest.approx([0.3, 0.4, 0.2, 1.0])
    assert d.polyline_distance(points, line[:1]) == pytest.approx(np.linalg.norm(points, axis=1))


# ── the launch speed ──────────────────────────────────────────────────────────


def test_without_drag_the_speed_is_the_closed_form():
    launch = np.array([[3.0, 0.0, 1.2], [2.0, 1.0, 1.0], [-2.5, 0.5, 1.6]])
    target = np.array([[0.1, 0.0, 1.5], [0.0, 0.0, 1.5], [0.0, -0.2, 1.5]])
    elevation = np.radians([45.0, 60.0, 45.0])
    found = d.solve_launch_speed(
        launch, target, elevation, NO_DRAG, step_s=0.002, horizon_s=2.5, speed_cap_m_s=48.0
    )
    distance = np.hypot(*(target - launch)[:, :2].T)
    rise = target[:, 2] - launch[:, 2]
    closed = np.sqrt(
        G * distance**2 / (2 * np.cos(elevation) ** 2 * (distance * np.tan(elevation) - rise))
    )
    assert found.solved.all()
    assert found.speed_m_s == pytest.approx(closed, abs=1e-9)
    assert found.vacuum_speed_m_s == pytest.approx(closed, abs=1e-12)
    assert found.pass_time_s == pytest.approx(distance / (closed * np.cos(elevation)), abs=1e-9)


def test_with_drag_the_designed_flight_goes_through_the_pass_point():
    table = _design(GRID.ids(), GRID.values(GRID.ids()))
    launched = table["screen"] != d.SCREEN_ASCENDING
    launched &= table["screen"] != d.SCREEN_SHOOT_FAIL
    assert launched.sum() > 200
    assert np.abs(table["pass_error_m"][launched]).max() <= 1e-9
    for i in np.flatnonzero(launched)[::37]:
        p0 = [table[f"pos_{a}"][i] for a in "xyz"]
        v0 = [table[f"vel_{a}"][i] for a in "xyz"]
        flight = cm.integrate_flight(p0, v0, BALL, horizon_s=2.5, step_s=0.002)
        target = [table[f"pass_{a}"][i] for a in "xyz"]
        miss, at, _ = cm.closest_approach(
            flight.time_s, flight.position_m, flight.velocity_m_s, target
        )
        assert miss < 1e-8
        assert at == pytest.approx(table["pass_time_s"][i], abs=1e-6)
        # drag asks for more speed than the vacuum does
        assert table["speed_m_s"][i] > d.vacuum_launch_speed(
            np.hypot(target[0] - p0[0], target[1] - p0[1]),
            target[2] - p0[2],
            np.radians(table["elevation_deg"][i]),
            G,
        )
        assert table["pass_vz_m_s"][i] < 0.0


def test_a_design_point_is_the_kinematic_grids_throw_of_its_six_axes():
    table = _design([7], _one(), origin_xy_m=(0.3, -0.2))
    assert table["screen"][0] == ""
    (throw,) = cm.generate_throw_grid(
        base_xy_m=(0.3, -0.2),
        distances_m=[table["distance_m"][0]],
        azimuths_deg=[table["azimuth_deg"][0]],
        release_heights_m=[table["release_height_m"][0]],
        aim_deviations_deg=[table["aim_deviation_deg"][0]],
        speeds_m_s=[table["speed_m_s"][0]],
        elevations_deg=[table["elevation_deg"][0]],
    )
    assert throw.position_m == pytest.approx([table[f"pos_{a}"][0] for a in "xyz"], abs=1e-12)
    assert throw.velocity_m_s == pytest.approx([table[f"vel_{a}"][0] for a in "xyz"], abs=1e-12)
    assert (
        table["pass_x"][0] == CATCH_POINT[0] + 0.1 and table["pass_y"][0] == CATCH_POINT[1] - 0.1
    )


# ── the screens ───────────────────────────────────────────────────────────────


def test_a_point_the_ball_would_still_be_rising_at_is_ascending():
    # d ≈ 1 m at 45°: the flight is past its apex only below Δz = d / 2
    high = _design(
        [0],
        _one(
            distance_m=1.0, azimuth_deg=0.0, pass_dx_m=-0.05, pass_dy_m=0.1, release_height_m=0.9
        ),
        ball=NO_DRAG,
    )
    assert high["screen"][0] == d.SCREEN_ASCENDING and math.isnan(high["vel_x"][0])
    low = _design(
        [0],
        _one(
            distance_m=1.0, azimuth_deg=0.0, pass_dx_m=-0.05, pass_dy_m=0.1, release_height_m=1.2
        ),
        ball=NO_DRAG,
    )
    assert low["screen"][0] == "" and low["pass_vz_m_s"][0] < 0.0
    # above the launch line no speed reaches the point at all
    over = _design(
        [0],
        _one(
            distance_m=1.0, azimuth_deg=0.0, pass_dx_m=-0.05, pass_dy_m=0.1, release_height_m=0.3
        ),
        ball=NO_DRAG,
    )
    assert over["screen"][0] == d.SCREEN_ASCENDING
    assert not d.vacuum_descending(np.array([1.0]), np.array([0.5]), np.radians([45.0]))[0]
    assert d.vacuum_descending(np.array([1.0]), np.array([0.49]), np.radians([45.0]))[0]
    assert math.isnan(
        d.vacuum_launch_speed(np.array([1.0]), np.array([1.0]), np.radians([45.0]), G)[0]
    )


def test_a_descending_throw_faster_than_the_limit_is_a_shoot_fail():
    values = _one()
    fine = _design([0], values)
    assert fine["screen"][0] == ""
    slow = _design([0], values, speed_max_m_s=float(fine["speed_m_s"][0]) - 0.01)
    assert slow["screen"][0] == d.SCREEN_SHOOT_FAIL and math.isnan(slow["speed_m_s"][0])
    assert math.isnan(slow["flight_time_s"][0])


def test_a_flight_that_never_enters_the_sphere_is_screened_by_reach():
    far = _design([0], _one(), constants=_constants(reach_centre_w=np.array([0.0, 0.0, -5.0])))
    assert far["screen"][0] == d.SCREEN_REACH
    assert math.isnan(far["reach_last_s"][0]) and far["reach_min_distance_m"][0] > 1.1
    assert math.isfinite(far["flight_time_s"][0])  # it has a launch and its axes


def test_too_short_a_lead_after_the_first_wake_is_screened_by_time():
    table = _design([0], _one())
    assert table["screen"][0] == ""
    available = float(table["lead_available_s"][0])
    assert available == pytest.approx(float(table["reach_last_s"][0]) - 0.15)
    just = _design([0], _one(), constants=_constants(lead_floor_s=available - 1e-6))
    assert just["screen"][0] == ""
    short = _design([0], _one(), constants=_constants(lead_floor_s=available + 1e-6))
    assert short["screen"][0] == d.SCREEN_TIME
    # no search sets a floor: no time screen
    none = _design([0], _one(), constants=_constants(lead_floor_s=math.nan))
    assert none["screen"][0] == ""


def test_the_recorded_axes_and_the_flags():
    table = _design([0], _one(), ball=NO_DRAG)
    speed, elevation = float(table["speed_m_s"][0]), math.radians(45.0)
    apex = 1.2 + (speed * math.sin(elevation)) ** 2 / (2 * G)
    assert table["apex_height_m"][0] == pytest.approx(apex, abs=1e-9)
    assert table["apex_margin_m"][0] == pytest.approx(apex - CATCH_POINT[2], abs=1e-9)
    assert not table["apex_flag"][0]
    flagged = _design([0], _one(), ball=NO_DRAG, constants=_constants(apex_margin_flag_m=10.0))
    assert flagged["apex_flag"][0] and flagged["screen"][0] == ""
    # the approach angle is measured against the hand's outward axis
    velocity = np.array([table[f"vel_{a}"][0] for a in "xyz"])
    assert 0.0 < table["approach_angle_deg"][0] < 90.0
    assert table["arrival_descent_deg"][0] > 0.0
    assert velocity[0] < 0.0  # flying towards −x, against the +x axis
    # the body flag is the pass point's distance to the polyline, and only a flag
    near = _design(
        [0],
        _one(pass_dx_m=-0.05, pass_dy_m=0.1),
        constants=_constants(body_vertices_w=np.array([[0.0, 0.0, 0.9], [0.0, 0.0, 1.6]])),
    )
    assert near["body_distance_m"][0] == pytest.approx(0.0, abs=1e-12)
    assert near["body_flag"][0] and near["screen"][0] == ""


def test_a_throws_row_does_not_depend_on_the_chunk_or_the_processes():
    ids = GRID.ids()
    values = GRID.values(ids)
    kwargs = {
        "origin_xy_m": (0.0, 0.0),
        "pass_plane_z_m": float(CATCH_POINT[2]),
        "constants": _constants(lead_floor_s=0.9),
        "ball": BALL,
        "speed_max_m_s": 5.0,
        "speed_cap_m_s": 48.0,
        "horizon_s": 2.5,
        "step_s": 0.002,
    }
    whole = d.design_table(ids, values, chunk=1000, jobs=1, **kwargs)
    cut = d.design_table(ids, values, chunk=50, jobs=2, **kwargs)
    assert set(whole["screen"]) >= {"", d.SCREEN_TIME, d.SCREEN_SHOOT_FAIL}
    for name in d.DESIGN_COLUMNS:
        if whole[name].dtype.kind == "f":
            assert np.allclose(whole[name], cut[name], rtol=0.0, atol=1e-12, equal_nan=True), name
        else:
            assert np.array_equal(whole[name], cut[name]), name


# ── files ─────────────────────────────────────────────────────────────────────


def test_the_design_csv_reads_back_what_was_written(tmp_path):
    ids = GRID.ids({"distance_m": [2.0], "pass_dy_m": [0.0]})
    table = _design(ids, GRID.values(ids), speed_max_m_s=5.0)
    d.write_design_csv(tmp_path / "design.csv", table)
    rows = d.read_design_csv(tmp_path / "design.csv")
    assert [r["throw_id"] for r in rows] == ids.tolist()
    for i, row in enumerate(rows):
        assert list(row) == list(d.DESIGN_COLUMNS)
        for name in d.DESIGN_COLUMNS:
            want = table[name][i]
            if isinstance(want, float | np.floating) and math.isnan(want):
                assert math.isnan(row[name])
            else:
                assert row[name] == want, name
    summary = d.screen_summary(rows)
    assert summary["throws"] == ids.size and sum(summary["labels"].values()) == ids.size
    assert set(summary["by_axis"]) == set(d.DESIGN_AXES)
    assert summary["labels"].get(d.SCREEN_SHOOT_FAIL, 0) > 0


def test_a_throw_list_holds_the_launches_and_names_the_rest_in_its_meta(tmp_path):
    ids = GRID.ids({"pass_dy_m": [0.0]})
    table = _design(
        ids, GRID.values(ids), speed_max_m_s=6.0, constants=_constants(lead_floor_s=0.8)
    )
    d.write_design_csv(tmp_path / "design.csv", table)
    rows = d.read_design_csv(tmp_path / "design.csv")
    throws, unlaunchable = d.throw_list_of(rows)
    labels = {row["throw_id"]: row["screen"] for row in rows}
    assert {u["throw_id"] for u in unlaunchable} == {
        t for t, label in labels.items() if label in d.UNLAUNCHABLE_SCREENS
    }
    assert unlaunchable and all(u["speed_m_s"] is None for u in unlaunchable)
    assert len(throws) + len(unlaunchable) == len(rows)
    # a screen that leaves a launch stays in the list, with its label
    assert d.SCREEN_TIME in {t["screen"] for t in throws}
    ctl.write_throw_list(tmp_path / "list.json", throws, {"unlaunchable": unlaunchable})
    back, meta = ctl.load_throw_list(tmp_path / "list.json")
    assert [t["throw_id"] for t in back] == [t["throw_id"] for t in throws]
    assert back[0]["pos"] == throws[0]["pos"] and back[0]["vel"] == throws[0]["vel"]
    assert back[0]["kind"] == d.THROW_KIND
    assert all(axis in back[0] for axis in d.DESIGN_AXES)
    assert len(meta["unlaunchable"]) == len(unlaunchable)
