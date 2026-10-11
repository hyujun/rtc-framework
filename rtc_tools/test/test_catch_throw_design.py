"""catch_throw_design (dynamic_catching L3 §4.1).

Pinned here: the grid's ids and the refinement between its points, the batch
integrator and its reductions against the one-flight functions of
``catchability_map``, the speed solve against the drag-free closed form and
against the flight it designs, each screen label on a throw built to get it,
and the files (the design CSV, the throw list a selection writes), the
elevation range, the pass-time limit and what ``select`` does with it, and the
plan set and the stratified sample that follow a map.
"""

from __future__ import annotations

import csv
import hashlib
import json
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


# ── the elevation range ───────────────────────────────────────────────────────


def test_the_elevation_axis_steps_down_from_the_maximum_and_is_stored_ascending():
    assert d.elevation_axis(70.0, 5.0, 5.0) == [5.0 + 5.0 * i for i in range(14)]
    # the maximum is always a value, the minimum only when the lattice reaches it
    assert d.elevation_axis(70.0, 12.0, 5.0) == [15.0 + 5.0 * i for i in range(12)]
    assert d.elevation_axis(45.0, 45.0, 5.0) == [45.0]
    # a step that is no binary fraction still lands on the rounded lattice
    fine = d.elevation_axis(1.0, 0.0, 0.1)
    assert fine[0] == 0.0 and fine[-1] == 1.0 and len(fine) == 11
    assert d.DesignGrid({**GRID.axes, "elevation_deg": tuple(fine)}).size > 0


@pytest.mark.parametrize(
    ("args", "match"),
    [((70.0, 5.0, 0.0), "step must be > 0"), ((70.0, 5.0, -5.0), "step must be > 0"),
     ((10.0, 20.0, 5.0), "above the maximum"), ((math.nan, 5.0, 5.0), "finite")],
)  # fmt: skip
def test_a_bad_elevation_range_is_refused(args, match):
    with pytest.raises(ValueError, match=match):
        d.elevation_axis(*args)


def test_the_old_elevation_list_argument_is_gone_and_the_range_is_required():
    assert "elevation_deg" not in d._AXIS_ARGUMENT
    with pytest.raises(SystemExit):
        d.main(["design", "--elevations-deg", "45 60"])


# ── the pass-time limit ───────────────────────────────────────────────────────


def test_a_launched_throw_with_a_pass_time_below_the_threshold_is_limited():
    ids = GRID.ids()
    values = GRID.values(ids)
    free = _design(ids, values)
    assert set(free[d.LIMIT_KEY]) == {""}  # absent threshold: no limit
    times = free["pass_time_s"]
    finite = np.sort(times[np.isfinite(times)])
    threshold = float(finite[finite.size // 2])  # a pass time some throw has
    table = _design(ids, values, min_pass_time_s=threshold)
    limited = table[d.LIMIT_KEY] == d.LIMIT_PASS_TIME
    with np.errstate(invalid="ignore"):
        want = times < threshold  # NaN (not launched) compares false
    assert np.array_equal(limited, want) and 0 < limited.sum() < ids.size
    assert set(table[d.LIMIT_KEY]) == {"", d.LIMIT_PASS_TIME}
    # strictly less: the throw whose time IS the threshold is not limited
    at = np.flatnonzero(times == threshold)
    assert at.size and not limited[at].any()
    # an unlaunched throw has no pass time and is never limited
    assert not limited[np.isnan(times)].any()
    # the limit does not touch the screen
    assert np.array_equal(table["screen"], free["screen"])


def test_a_throw_can_carry_a_screen_and_the_limit_and_each_is_counted_apart():
    ids = GRID.ids()
    values = GRID.values(ids)
    table = _design(
        ids, values, min_pass_time_s=0.9, constants=_constants(lead_floor_s=0.9), speed_max_m_s=6.0
    )
    both = (table["screen"] != "") & (table[d.LIMIT_KEY] != "")
    assert both.any()
    assert ((table["screen"] != "") & (table[d.LIMIT_KEY] == "")).any()
    assert ((table["screen"] == "") & (table[d.LIMIT_KEY] != "")).any()
    assert d.DESIGN_COLUMNS[-2:] == (d.SCREEN_KEY, d.LIMIT_KEY)


def test_limit_counts_give_the_denominators_per_axis_value():
    rows = [
        {**dict.fromkeys(d.DESIGN_AXES, 1.0), "distance_m": 2.0 + (i % 2), d.LIMIT_KEY: lim}
        for i, lim in enumerate([d.LIMIT_PASS_TIME, "", d.LIMIT_PASS_TIME, d.LIMIT_PASS_TIME, ""])
    ]
    counts = d.limit_counts(rows)
    assert counts["throws"] == 5 and counts["limited"] == 3
    assert counts["by_axis"]["distance_m"] == {
        "2.0": {"throws": 3, "limited": 2},
        "3.0": {"throws": 2, "limited": 1},
    }
    assert counts["by_axis"]["elevation_deg"] == {"1.0": {"throws": 5, "limited": 3}}
    # a row of an old CSV has no limit key
    assert not d.is_limited({"throw_id": 0}) and d.limit_counts([{**rows[0]}])["limited"] == 1


# ── select ────────────────────────────────────────────────────────────────────


def _design_dir(tmp_path, *, min_pass_time_s=None, old_csv=False):
    """A design directory of GRID (324 throws) as ``design`` writes it."""
    out = tmp_path / ("old" if old_csv else "design")
    out.mkdir()
    ids = GRID.ids()
    table = _design(ids, GRID.values(ids), min_pass_time_s=min_pass_time_s)
    d.write_design_csv(out / "design.csv", table)
    if old_csv:  # a design.csv from before the column existed
        with (out / "design.csv").open(newline="") as handle:
            rows = list(csv.reader(handle))
        keep = rows[0].index(d.LIMIT_KEY)
        with (out / "design.csv").open("w", newline="") as handle:
            csv.writer(handle).writerows([r[:keep] + r[keep + 1 :] for r in rows])
    rows = d.read_design_csv(out / "design.csv")
    meta = {
        "grid": {"axes": {a: list(GRID.axes[a]) for a in d.DESIGN_AXES}, "throws": GRID.size},
        "axes": {"discrete": list(d.DESIGN_AXES), "binned": []},
        "limits": {"min_pass_time_s": min_pass_time_s, **d.limit_counts(rows)},
    }
    (out / "design_meta.json").write_text(json.dumps(meta))
    return out, table


def _select(design_dir, out, *extra):
    argv = ["select", "--design-dir", str(design_dir), "--out", str(out), *extra]
    return d.main(argv)


def _limited_design(tmp_path, quantile):
    """A design directory of GRID whose limit takes about ``quantile`` of the launched throws."""
    ids = GRID.ids()
    probe = _design(ids, GRID.values(ids))
    threshold = float(np.nanquantile(probe["pass_time_s"], quantile))
    out = tmp_path / "limited"
    out.mkdir()
    d.write_design_csv(
        out / "design.csv", _design(ids, GRID.values(ids), min_pass_time_s=threshold)
    )
    meta = {
        "grid": {"axes": {a: list(GRID.axes[a]) for a in d.DESIGN_AXES}},
        "axes": {},
        "limits": {"min_pass_time_s": threshold},
    }
    (out / "design_meta.json").write_text(json.dumps(meta))
    return out, d.read_design_csv(out / "design.csv"), threshold


def test_select_drops_the_limited_throws_on_every_path_and_records_what_it_dropped(tmp_path):
    dd, rows, threshold = _limited_design(tmp_path, 0.3)
    subset = ["--subset", "distance_m=2.0 3.0", "--subset", "elevation_deg=45"]
    chosen = [r for r in rows if r["distance_m"] in (2.0, 3.0) and r["elevation_deg"] == 45.0]
    limited = {r["throw_id"] for r in chosen if r[d.LIMIT_KEY]}
    kept = {r["throw_id"] for r in chosen if not r[d.LIMIT_KEY]}
    assert limited and kept

    # --subset
    assert _select(dd, tmp_path / "a.json", *subset) == 0
    throws, meta = ctl.load_throw_list(tmp_path / "a.json")
    launched = {
        t for t in kept if next(r for r in chosen if r["throw_id"] == t)["screen"]
        not in d.UNLAUNCHABLE_SCREENS
    }  # fmt: skip
    assert {t["throw_id"] for t in throws} == launched
    info = meta["limit"]
    assert info["min_pass_time_s"] == threshold and info["removed"] == len(limited)
    assert info["selected_before_removal"] == len(chosen)
    by_distance = info["by_axis"]["distance_m"]
    assert by_distance["2.0"]["limited"] + by_distance["3.0"]["limited"] == len(limited)
    assert by_distance["2.0"]["throws"] + by_distance["3.0"]["throws"] == len(chosen)
    assert meta["selection"]["throws_before_limit"] == len(chosen)
    assert meta["selection"]["throws"] == len(kept)

    # --ids-file
    ids_file = tmp_path / "ids.json"
    ids_file.write_text(json.dumps(sorted(r["throw_id"] for r in chosen)))
    assert _select(dd, tmp_path / "b.json", "--ids-file", str(ids_file)) == 0
    throws_b, meta_b = ctl.load_throw_list(tmp_path / "b.json")
    assert {t["throw_id"] for t in throws_b} == launched
    assert meta_b["limit"]["removed"] == len(limited)

    # --max-throws counts what is left after the removal
    assert _select(dd, tmp_path / "c.json", *subset, "--max-throws", str(len(kept))) == 0
    assert len(chosen) > len(kept)
    with pytest.raises(SystemExit, match="over --max-throws"):
        _select(dd, tmp_path / "e.json", *subset, "--max-throws", str(len(kept) - 1))


def test_select_reads_a_design_csv_that_has_no_limit_column(tmp_path):
    dd, _ = _design_dir(tmp_path, old_csv=True)
    assert d.LIMIT_KEY not in d.read_design_csv(dd / "design.csv")[0]
    assert _select(dd, tmp_path / "l.json", "--subset", "distance_m=2.0") == 0
    throws, meta = ctl.load_throw_list(tmp_path / "l.json")
    assert throws and meta["limit"]["removed"] == 0
    assert meta["limit"]["min_pass_time_s"] is None


def test_refine_records_the_pairs_it_skipped_and_the_points_the_limit_removed(tmp_path):
    dd, rows, _ = _limited_design(tmp_path, 0.4)
    subset = ["--subset", "distance_m=2.0 3.0"]
    assert _select(dd, tmp_path / "coarse.json", *subset) == 0
    listed, meta = ctl.load_throw_list(tmp_path / "coarse.json")
    # The map's verdicts: accepted exactly at distance 2.0. It also writes a row for each
    # throw the list's meta names as unlaunchable (refused); a throw the limit removed
    # is in no list and has no row.
    verdict = {t["throw_id"]: t["distance_m"] == 2.0 for t in listed}
    verdict.update((u["throw_id"], False) for u in meta["unlaunchable"])
    verdicts = tmp_path / "verdicts_grid.csv"
    with verdicts.open("w", newline="") as handle:
        w = csv.writer(handle)
        w.writerow(["throw_id", "accepted"])
        w.writerows(verdict.items())
    out = tmp_path / "fine.json"
    assert _select(dd, out, *subset, "--refine-from", str(verdicts)) == 0
    fine, fine_meta = ctl.load_throw_list(out)
    info = fine_meta["selection"]["refine"][str(verdicts)]

    # the pairs along the distance axis: the other five axes fix a pair
    others = [a for a in d.DESIGN_AXES if a != "distance_m"]
    low = {tuple(r[a] for a in others): r for r in rows if r["distance_m"] == 2.0}
    high = {tuple(r[a] for a in others): r for r in rows if r["distance_m"] == 3.0}
    skipped = differ = 0
    between = []
    for key, lo in low.items():
        hi = high[key]
        if lo["throw_id"] not in verdict or hi["throw_id"] not in verdict:
            skipped += 1
        elif verdict[lo["throw_id"]] != verdict[hi["throw_id"]]:
            differ += 1
            between += [r for r in rows if r["distance_m"] == 2.5 and
                        tuple(r[a] for a in others) == key]  # fmt: skip
    assert skipped > 0 and differ > 0
    assert info["pairs_skipped_no_verdict"] == sum(
        info["pairs_skipped_no_verdict_by_axis"].values()
    )
    assert info["pairs_skipped_no_verdict_by_axis"]["distance_m"] == skipped
    assert info["pairs_by_axis"]["distance_m"] == differ
    assert info["points"] == len(between)
    removed = [r for r in between if r[d.LIMIT_KEY]]
    assert info["points_removed_by_limit"] == len(removed)
    assert fine_meta["limit"]["removed"] == len(removed)
    assert not {t["throw_id"] for t in fine} & {r["throw_id"] for r in rows if r[d.LIMIT_KEY]}


# ── the plan set ──────────────────────────────────────────────────────────────


def _list_of_all(tmp_path, name="list.json", **meta):
    """A throw list of the launchable throws of GRID, as select writes one."""
    ids = GRID.ids()
    table = _design(ids, GRID.values(ids))
    d.write_design_csv(tmp_path / "t.csv", table)
    throws, unlaunchable = d.throw_list_of(d.read_design_csv(tmp_path / "t.csv"))
    ctl.write_throw_list(
        tmp_path / name,
        throws,
        {"axes": {"discrete": ["distance_m"]}, "unlaunchable": unlaunchable, "limit": {"removed": 0},
         "design": {"grid": {"throws": GRID.size}}, "selection": {"design_dir": "/somewhere"}, **meta},
    )  # fmt: skip
    return tmp_path / name, throws, unlaunchable


def _verdict_file(path, throws, accepted_ids, extra_rows=()):
    with path.open("w", newline="") as handle:
        w = csv.writer(handle)
        w.writerow(["throw_id", "accepted", "reason"])
        for t in throws:
            w.writerow([t["throw_id"], t["throw_id"] in accepted_ids, ""])
        for i in extra_rows:
            w.writerow([i, False, "screen"])


def test_the_plan_set_is_the_accepted_throws_in_ascending_id_with_the_sources_hashes(tmp_path):
    listed, throws, unlaunchable = _list_of_all(tmp_path)
    accepted = {t["throw_id"] for t in throws[::3]}
    verdicts = tmp_path / "verdicts_grid.csv"
    _verdict_file(
        verdicts, throws[::-1], accepted, extra_rows=[u["throw_id"] for u in unlaunchable]
    )
    out = tmp_path / "plan.json"
    argv = [
        "plan-set",
        "--throws-file",
        str(listed),
        "--verdicts",
        str(verdicts),
        "--out",
        str(out),
    ]
    assert d.main(argv) == 0
    plan, meta = ctl.load_throw_list(out)
    ids = [t["throw_id"] for t in plan]
    assert ids == sorted(accepted)
    assert meta["source_list"]["sha256"] == hashlib.sha256(listed.read_bytes()).hexdigest()
    assert meta["verdicts"] == [
        {"name": verdicts.name, "sha256": hashlib.sha256(verdicts.read_bytes()).hexdigest()}
    ]
    assert meta["counts"] == {"judged": len(throws), "accepted": len(accepted)}
    assert meta["axes"] == {"discrete": ["distance_m"]} and "design" in meta and "limit" in meta
    assert "unlaunchable" not in meta and "selection" not in meta
    assert str(tmp_path) not in out.read_text()
    # the entries are the source's, untouched
    src = {t["throw_id"]: t for t in ctl.load_throw_list(listed)[0]}
    assert all(t == src[t["throw_id"]] for t in plan)
    # the same inputs give the same bytes
    again = tmp_path / "plan2.json"
    d.main(
        [
            "plan-set",
            "--throws-file",
            str(listed),
            "--verdicts",
            str(verdicts),
            "--out",
            str(again),
        ]
    )
    assert again.read_bytes() == out.read_bytes()


def test_the_plan_set_refuses_verdicts_that_do_not_cover_the_list_or_an_unknown_accepted_id(
    tmp_path,
):
    listed, throws, _ = _list_of_all(tmp_path)
    short = tmp_path / "short.csv"
    _verdict_file(short, throws[:-5], {throws[0]["throw_id"]})
    base = ["plan-set", "--throws-file", str(listed), "--out", str(tmp_path / "p.json")]
    with pytest.raises(SystemExit, match=rf"cover {len(throws) - 5} of the list's {len(throws)}"):
        d.main([*base, "--verdicts", str(short)])
    stray = tmp_path / "stray.csv"
    _verdict_file(stray, throws, set())
    with stray.open("a") as handle:
        handle.write("999999,True,\n")
    with pytest.raises(SystemExit, match="not in the list"):
        d.main([*base, "--verdicts", str(stray)])
    none = tmp_path / "none.csv"
    _verdict_file(none, throws, set())
    with pytest.raises(SystemExit, match="no throw"):
        d.main([*base, "--verdicts", str(none)])


# ── the sample ────────────────────────────────────────────────────────────────


def test_allocation_is_proportional_by_largest_remainder_with_ties_in_key_order():
    assert d.allocate({"a": 5, "b": 3, "c": 2}, 4) == {"a": 2, "b": 1, "c": 1}
    assert d.allocate({("a",): 1, ("b",): 1}, 1) == {("a",): 1, ("b",): 0}
    assert d.allocate({"a": 5, "b": 3}, 8) == {"a": 5, "b": 3}
    sizes = {i: 7 + 3 * i for i in range(9)}
    for total in (0, 1, 17, 50, sum(sizes.values())):
        share = d.allocate(sizes, total)
        assert sum(share.values()) == total
        for key, n in share.items():
            exact = total * sizes[key] / sum(sizes.values())
            assert math.floor(exact) <= n <= math.ceil(exact) and n <= sizes[key]


def _plan_set(tmp_path):
    listed, throws, _ = _list_of_all(tmp_path)
    plan = tmp_path / "plan.json"
    ctl.write_throw_list(plan, throws, {"axes": {"discrete": ["distance_m"]}})
    return plan, throws


def _sample(plan, out, *extra, seed=7, chunk=40):
    argv = ["sample", "--plan-set", str(plan), "--out-dir", str(out), "--strata", "distance_m",
            "elevation_deg", "--seed", str(seed), "--chunk", str(chunk), *extra]  # fmt: skip
    return d.main(argv)


def test_a_sample_takes_the_same_fraction_of_every_stratum_and_splits_into_chunks(tmp_path):
    plan, throws = _plan_set(tmp_path)
    assert _sample(plan, tmp_path / "s", "--max-throws", "100") == 0
    sample = ctl.read_throw_list_file(tmp_path / "s" / "sample.json")
    assert len(sample.throws) == 100
    strata = ("distance_m", "elevation_deg")
    sizes: dict = {}
    for t in throws:
        sizes[tuple(t[a] for a in strata)] = sizes.get(tuple(t[a] for a in strata), 0) + 1
    want = d.allocate(sizes, 100)
    got: dict = {}
    for t in sample.throws:
        got[tuple(t[a] for a in strata)] = got.get(tuple(t[a] for a in strata), 0) + 1
    assert got == {k: n for k, n in want.items() if n}
    meta = sample.meta
    assert {(tuple(e["values"].values()), e["total"], e["sampled"]) for e in meta["strata"]} == {
        (k, sizes[k], want[k]) for k in sizes
    }
    assert meta["plan_set_sha256"] == hashlib.sha256(plan.read_bytes()).hexdigest()
    assert meta["strata_axes"] == list(strata) and meta["seed"] == 7
    assert (
        meta["max_throws"] == 100
        and meta["chunk"] == 40
        and meta["plan_set_throws"] == len(throws)
    )
    # a sample is a subset of the plan set, entries untouched, throw order not by id
    by_id = {t["throw_id"]: t for t in throws}
    assert all(by_id[t["throw_id"]] == t for t in sample.throws)
    sample_ids = [t["throw_id"] for t in sample.throws]
    assert sample_ids != sorted(sample_ids) and len(set(sample_ids)) == 100
    # the chunks are consecutive slices of the sample, each a valid list
    files = sorted((tmp_path / "s").glob("chunk_*.json"))
    assert [f.name for f in files] == ["chunk_000.json", "chunk_001.json", "chunk_002.json"]
    joined = []
    for i, f in enumerate(files):
        chunk = ctl.read_throw_list_file(f)
        assert len(chunk.throws) == (40 if i < 2 else 20)
        assert chunk.meta["chunk_index"] == i and chunk.meta["chunk_count"] == 3
        assert chunk.meta["sample_sha256"] == sample.sha256
        assert chunk.meta["plan_set_sha256"] == meta["plan_set_sha256"]
        assert chunk.meta["strata"] == meta["strata"] and chunk.meta["seed"] == 7
        joined += [t["throw_id"] for t in chunk.throws]
    assert joined == sample_ids


def test_a_sample_is_a_function_of_the_inputs_and_the_seed_alone(tmp_path):
    plan, throws = _plan_set(tmp_path)
    _sample(plan, tmp_path / "a", "--max-throws", "90")
    _sample(plan, tmp_path / "b", "--max-throws", "90")
    names = sorted(p.name for p in (tmp_path / "a").iterdir())
    assert names == sorted(p.name for p in (tmp_path / "b").iterdir())
    for name in names:
        assert (tmp_path / "a" / name).read_bytes() == (tmp_path / "b" / name).read_bytes()
        assert str(tmp_path) not in (tmp_path / "a" / name).read_text()
    # the order of the plan set's entries does not matter
    shuffled = tmp_path / "shuffled.json"
    ctl.write_throw_list(shuffled, throws[::-1], {"axes": {"discrete": ["distance_m"]}})
    got, _ = d.sample_throws(
        ctl.load_throw_list(shuffled)[0], ("distance_m", "elevation_deg"), 7, 90
    )
    base, _ = d.sample_throws(throws, ("distance_m", "elevation_deg"), 7, 90)
    assert [t["throw_id"] for t in got] == [t["throw_id"] for t in base]
    # another seed: another sample
    _sample(plan, tmp_path / "c", "--max-throws", "90", seed=8)
    one = ctl.load_throw_list(tmp_path / "a" / "sample.json")[0]
    two = ctl.load_throw_list(tmp_path / "c" / "sample.json")[0]
    assert {t["throw_id"] for t in one} != {t["throw_id"] for t in two}
    # the picks are pinned to the hash rule, not to a library's stream
    first = min(
        (t for t in throws if (t["distance_m"], t["elevation_deg"]) == (2.0, 45.0)),
        key=lambda t: hashlib.sha256(f"7:pick:{t['throw_id']}".encode()).hexdigest(),
    )
    all_ids, _ = d.sample_throws(throws, ("distance_m", "elevation_deg"), 7, 1)
    assert len(all_ids) == 1
    pick_one, counts = d.sample_throws(
        [t for t in throws if (t["distance_m"], t["elevation_deg"]) == (2.0, 45.0)],
        ("distance_m", "elevation_deg"),
        7,
        1,
    )
    assert pick_one[0]["throw_id"] == first["throw_id"] and counts[0]["sampled"] == 1


def test_a_sample_without_a_max_or_with_one_over_the_size_takes_everything(tmp_path):
    plan, throws = _plan_set(tmp_path)
    _sample(plan, tmp_path / "all", chunk=1000)
    _sample(plan, tmp_path / "big", "--max-throws", str(len(throws) + 10), chunk=1000)
    for name in ("all", "big"):
        sample = ctl.load_throw_list(tmp_path / name / "sample.json")[0]
        assert sorted(t["throw_id"] for t in sample) == sorted(t["throw_id"] for t in throws)
        assert [p.name for p in (tmp_path / name).glob("chunk_*")] == ["chunk_000.json"]


def test_a_sample_refuses_a_non_axis_stratum_and_a_used_directory(tmp_path):
    plan, _ = _plan_set(tmp_path)
    argv = ["sample", "--plan-set", str(plan), "--out-dir", str(tmp_path / "x"), "--seed", "1",
            "--chunk", "10", "--strata", "speed_m_s"]  # fmt: skip
    with pytest.raises(SystemExit, match="design axes"):
        d.main(argv)
    _sample(plan, tmp_path / "once")
    with pytest.raises(SystemExit, match="already holds"):
        _sample(plan, tmp_path / "once")
    with pytest.raises(SystemExit):
        d.main(["sample", "--plan-set", str(plan), "--out-dir", str(tmp_path / "y"),
                "--strata", "distance_m", "--chunk", "10"])  # fmt: skip  # no --seed
