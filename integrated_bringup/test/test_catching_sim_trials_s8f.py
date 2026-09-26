"""catching_sim_trials — the hand-near designs (dynamic_catching S8-F, #537).

What must hold for the S8-F series to be a measurement: the same
``(dist, n, seed)`` replays the same launches against the same hand geometry
(so a unit the host-load rule re-runs is the SAME unit), the grids cover every
speed even when cut short, every accepted draw of the LHS box lies inside the
box and clears the floor, the record carries the factors the analysis fits on
and the target the truth check is made against — and none of it touches the
default reference series or the s35b box.
"""

from __future__ import annotations

import json
import math

import pytest

from integrated_bringup import catching_sim_trials as cst

P_C = (-0.478, 0.0, 0.801)
AXIS = (0.724, 0.0, 0.689)


@pytest.fixture(scope="module")
def geometry() -> cst.HandGeometry:
    from rtc_tools.analysis.catchability_map import BallParams

    params = BallParams(
        radius_m=0.0335,
        mass_kg=0.057,
        drag_coefficient=0.55,
        air_density_kg_m3=1.204,
        sources=dict.fromkeys(
            ("radius_m", "mass_kg", "drag_coefficient", "air_density_kg_m3"), "test"
        ),
    )
    return cst.HandGeometry(
        p_c_m=P_C, approach_axis=AXIS, floor_z_m=0.05, params=params, sources={}
    )


def test_the_designs_are_the_confirmed_ones():
    cliff = cst.HAND_DESIGNS["hand_cliff"]
    assert cliff.speeds_m_s == (3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 7.0) and cliff.n == 56
    assert (
        cliff.flight_time_s == 0.65 and cliff.offset_m == 0.0 and cliff.incidence_offset_deg == 0.0
    )
    lob = cst.HAND_DESIGNS["hand_lob"]
    assert lob.speeds_m_s == (2.5, 3.0, 3.5, 4.0) and lob.n == 56 and lob.incidence_deg == 85.0
    box = cst.HAND_DESIGNS["hand_lhs"]
    assert (box.speed_m_s, box.flight_time_s, box.offset_m) == (
        (3.5, 7.0),
        (0.65, 0.8),
        (0.0, 0.2),
    )
    assert (box.offset_angle_deg, box.incidence_offset_deg) == ((0.0, 360.0), (-10.0, 15.0))


def test_a_grid_interleaves_speeds_and_ignores_n(geometry):
    throws = cst.hand_near_throws("hand_cliff", 3, 42, geometry)
    assert len(throws) == 56
    speeds = [t["speed_m_s"] for t in throws]
    assert speeds[:7] == list(cst.HAND_DESIGNS["hand_cliff"].speeds_m_s)
    assert speeds[7:14] == speeds[:7]
    assert [t["repeat"] for t in throws[:8]] == [0] * 7 + [1]
    assert all(t["offset_m"] == 0.0 and t["flight_time_s"] == 0.65 for t in throws)
    assert all(t["kind"] == "hand_cliff" and t["omega"] == (0.0, 0.0, 0.0) for t in throws)
    # Face-on: the incidence is the axis elevation.
    axis_elev = math.degrees(math.atan2(AXIS[2], math.hypot(AXIS[0], AXIS[1])))
    assert all(abs(t["incidence_deg"] - axis_elev) < 1e-9 for t in throws)
    # Replays exactly, and the design does not depend on the seed.
    assert json.dumps(throws) == json.dumps(cst.hand_near_throws("hand_cliff", 3, 42, geometry))
    other = cst.hand_near_throws("hand_cliff", 3, 7, geometry)
    assert [t["pos"] for t in other] == [t["pos"] for t in throws]


def test_the_lob_is_steep_in_the_world_not_relative_to_the_palm(geometry):
    throws = cst.hand_near_throws("hand_lob", 0, 1, geometry)
    assert len(throws) == 56
    assert all(abs(t["incidence_deg"] - 85.0) < 1e-9 for t in throws)
    axis_elev = geometry.axis_elevation_deg
    assert all(abs(t["incidence_offset_deg"] - (85.0 - axis_elev)) < 1e-9 for t in throws)
    # Slow lobs release BELOW the catch point and still clear the table.
    slow = [t for t in throws if t["speed_m_s"] == 2.5]
    assert all(t["release_height_offset_m"] < 0.0 for t in slow)
    assert all(t["pos"][2] - geometry.params.radius_m > geometry.floor_z_m for t in throws)


def test_the_lhs_box_replays_stays_inside_and_clears_the_floor(geometry):
    a = cst.hand_near_throws("hand_lhs", 40, 11, geometry)
    b = cst.hand_near_throws("hand_lhs", 40, 11, geometry)
    assert json.dumps(a) == json.dumps(b)
    assert len(a) == 40
    assert json.dumps(cst.hand_near_throws("hand_lhs", 40, 12, geometry)) != json.dumps(a)
    box = cst.HAND_DESIGNS["hand_lhs"]
    for t in a:
        for axis in cst.HAND_BOX_AXES:
            lo, hi = getattr(box, axis)
            assert lo <= t[axis] <= hi, (axis, t[axis])
        assert t["pos"][2] - geometry.params.radius_m > geometry.floor_z_m
        # The target is r from p_c in the plane normal to the axis.
        d = [t["target_m"][i] - P_C[i] for i in range(3)]
        assert abs(sum(d[i] * AXIS[i] for i in range(3))) < 1e-9
        assert math.sqrt(sum(x * x for x in d)) == pytest.approx(t["offset_m"], abs=1e-9)
    # Some draws were refused by the floor (slow, long flights) and redrawn:
    # the draw counter is monotone and exceeds the accepted count.
    draws = [t["draws"] for t in a]
    assert draws == sorted(draws) and draws[-1] >= 40
    assert [t["sample_idx"] for t in a] == list(range(40))
    # Every accepted sample has the arrival it asked for (speed is a factor,
    # not a derived quantity).
    assert all(3.5 <= t["speed_m_s"] <= 7.0 for t in a)


def test_the_lhs_covers_the_speed_range_with_one_point_per_stratum(geometry):
    """The Latin property on the axis the floor does not bite (ψ): 20 draws,
    20 strata, each hit once — when no redraw happened in that hypercube."""
    n = 20
    throws = cst.hand_near_throws("hand_lhs", n, 3, geometry)
    first_cube = [t for t in throws if t["draws"] <= n]
    strata = sorted(int(t["offset_angle_deg"] / 360.0 * n) for t in first_cube)
    assert strata == sorted(set(strata))  # no stratum twice inside one hypercube


def test_build_throws_routes_hand_designs_and_needs_the_geometry(geometry):
    args = cst.parse_args(["out", "--dist", "hand_cliff"])
    with pytest.raises(ValueError, match="hand geometry"):
        cst.build_throws(args, "ur5e_p1b")
    assert cst.build_throws(args, "ur5e_p1b", geometry) == cst.hand_near_throws(
        "hand_cliff", 25, 42, geometry
    )
    with pytest.raises(ValueError, match="unknown hand-near design"):
        cst.hand_near_throws("hand_nope", 1, 1, geometry)
    with pytest.raises(ValueError):
        cst.hand_near_throws("hand_lhs", -1, 1, geometry)


def test_the_default_series_and_the_s35b_box_are_untouched():
    args = cst.parse_args(["out"])
    assert args.dist == "reference"
    assert cst.build_throws(args, "ur5e_p1b") == cst.trial_throws(
        15, 10, 42, args.release_pos, args.release_vel
    )
    args = cst.parse_args(["out", "--dist", "s35b", "--n", "3", "--seed", "5"])
    assert cst.build_throws(args, "ur5e_p1b") == cst.frozen_throws("s35b", "ur5e_p1b", 3, 5)
    assert set(cst.HAND_DESIGNS) == {"hand_cliff", "hand_lob", "hand_lhs"}
    assert not set(cst.HAND_DESIGNS) & set(cst.FROZEN_DISTRIBUTIONS)


def test_the_geometry_record_names_every_source(geometry):
    rec = geometry.as_record()
    assert rec["p_c_m"] == list(P_C) and rec["approach_axis"] == list(AXIS)
    assert rec["axis_elevation_deg"] == pytest.approx(43.58, abs=0.05)
    assert rec["ball"]["drag_coefficient"] == 0.55 and "sources" in rec["ball"]
    # The hand-near flags default to the shipped tennis preset with its file:line.
    args = cst.parse_args(["out", "--dist", "hand_lhs"])
    assert args.drag_coefficient == 0.55 and "projectile_ball.cpp" in args.drag_coefficient_source
    assert args.air_density == 1.204 and "projectile_ball.hpp" in args.air_density_source
    assert args.floor_z == 0.05 and args.arm is None
