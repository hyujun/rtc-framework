"""catchability_map.aim_at_hand / closest_approach (dynamic_catching S8-F).

Oracles independent of the code under test:

* the release the backward integration returns, flown FORWARD with
  ``integrate_flight`` (the module's own forward integrator, itself pinned
  against the parabola elsewhere), must arrive at the target with the requested
  speed and direction — and a forward shooting solve (scipy, on the release
  velocity) must land on the same release velocity;
* drag-free arrival against the closed-form parabola written out here;
* the vacuum release differs from the drag release by the amount the S8-F
  re-derivation measured (≈ 0.1 m at 5 m/s over 0.8 s), so a caller that
  skipped the drag law would miss the hand by that much;
* the floor refusal fires exactly when the ball surface would cross it;
* closest_approach against a parabola sampled coarsely, where the minimum is
  strictly between samples.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from rtc_tools.analysis import catchability_map as cm

CD_SOURCE = "rtc_mujoco_sim/src/projectile_ball.cpp:30"
RHO_SOURCE = "rtc_mujoco_sim/include/rtc_mujoco_sim/projectile_ball.hpp:98"
G = 9.81
# The shipped ur5e_p1b wait pose's catch frame (MuJoCo FK, 2026-09-26): palm
# normal 43.6° above the horizontal, facing +x.
P_C = (-0.478, 0.0, 0.801)
AXIS = (0.724, 0.0, 0.689)


def params(cd: float = 0.55, rho: float = 1.204) -> cm.BallParams:
    return cm.BallParams(
        radius_m=0.0335,
        mass_kg=0.057,
        drag_coefficient=cd,
        air_density_kg_m3=rho,
        sources={
            "radius_m": "test",
            "mass_kg": "test",
            "drag_coefficient": CD_SOURCE,
            "air_density_kg_m3": RHO_SOURCE,
        },
    )


def aim(**kw) -> cm.HandNearThrow:
    base = {
        "speed_m_s": 5.0,
        "flight_time_s": 0.8,
        "offset_m": 0.0,
        "offset_angle_deg": 0.0,
        "incidence_offset_deg": 0.0,
        "params": params(),
    }
    base.update(kw)
    return cm.aim_at_hand(P_C, AXIS, **base)


@pytest.mark.parametrize(
    "speed, flight, alpha, offset, psi",
    [
        (5.0, 0.8, 0.0, 0.0, 0.0),
        (4.0, 0.65, 10.0, 0.2, 90.0),
        (7.0, 0.65, -10.0, 0.15, 210.0),
        (3.5, 0.7, 15.0, 0.05, 45.0),
    ],
)
def test_the_release_flown_forward_arrives_as_specified(speed, flight, alpha, offset, psi):
    t = aim(
        speed_m_s=speed,
        flight_time_s=flight,
        incidence_offset_deg=alpha,
        offset_m=offset,
        offset_angle_deg=psi,
    )
    traj = cm.integrate_flight(
        t.position_m, t.velocity_m_s, params(), horizon_s=flight, step_s=1e-3
    )
    assert np.linalg.norm(traj.position_m[-1] - t.target_m) < 1e-6
    v_end = traj.velocity_m_s[-1]
    assert abs(np.linalg.norm(v_end) - speed) < 1e-6
    # Direction: below the horizontal by the axis elevation + alpha, against the axis.
    axis_elev = math.degrees(math.atan2(AXIS[2], math.hypot(AXIS[0], AXIS[1])))
    incidence = math.degrees(math.atan2(-v_end[2], math.hypot(v_end[0], v_end[1])))
    assert incidence == pytest.approx(axis_elev + alpha, abs=1e-6)
    assert t.incidence_deg == pytest.approx(incidence, abs=1e-6)
    assert v_end[0] < 0.0  # flies toward -x, into a palm facing +x
    # The target sits in the plane normal to the axis at the requested radius.
    d = t.target_m - np.asarray(P_C)
    assert abs(float(d @ np.asarray(AXIS)) / np.linalg.norm(AXIS)) < 1e-9
    assert np.linalg.norm(d) == pytest.approx(offset, abs=1e-9)


def test_forward_shooting_finds_the_same_release_velocity():
    from scipy.optimize import fsolve

    t = aim(speed_m_s=6.0, flight_time_s=0.7, incidence_offset_deg=5.0)

    def miss(v0):
        traj = cm.integrate_flight(
            t.position_m, v0, params(), horizon_s=t.flight_time_s, step_s=1e-3
        )
        return traj.position_m[-1] - t.target_m

    # Start from the vacuum solution, well away from the drag answer.
    vac = aim(
        speed_m_s=6.0, flight_time_s=0.7, incidence_offset_deg=5.0, params=params(cd=0.0, rho=0.0)
    )
    v0, _, ier, _ = fsolve(miss, vac.velocity_m_s, full_output=True, xtol=1e-12)
    assert ier == 1
    assert np.linalg.norm(v0 - t.velocity_m_s) < 1e-5


def test_drag_free_release_matches_the_parabola():
    t = aim(speed_m_s=4.5, flight_time_s=0.7, params=params(cd=0.0, rho=0.0))
    T = t.flight_time_s
    v_t = t.arrival_velocity_m_s
    # p(0) = p(T) - v(T) T + a T²/2 with a = (0, 0, -g) ; v(0) = v(T) - a T
    expect_p = t.target_m - v_t * T + 0.5 * np.array([0.0, 0.0, -G]) * T * T
    expect_v = v_t - np.array([0.0, 0.0, -G]) * T
    assert np.allclose(t.position_m, expect_p, atol=1e-9)
    assert np.allclose(t.velocity_m_s, expect_v, atol=1e-9)
    assert t.release_height_offset_m == pytest.approx(expect_p[2] - t.target_m[2], abs=1e-9)
    assert t.horizontal_distance_m == pytest.approx(
        np.linalg.norm((expect_p - t.target_m)[:2]), abs=1e-9
    )


def test_skipping_the_drag_law_misses_by_a_tenth_of_a_metre():
    """The S8-F re-derivation (#537, 2026-09-26): the vacuum release for a
    5 m/s arrival over 0.8 s is ~0.1 m from the drag release. Anything aimed
    without drag would pass the hand by that much — far outside r_cap."""
    drag = aim()
    vacuum = aim(params=params(cd=0.0, rho=0.0))
    gap = np.linalg.norm(drag.position_m - vacuum.position_m)
    assert 0.08 < gap < 0.16, gap
    # And flying the vacuum release under drag really misses.
    traj = cm.integrate_flight(
        vacuum.position_m, vacuum.velocity_m_s, params(), horizon_s=0.8, step_s=1e-3
    )
    assert np.linalg.norm(traj.position_m[-1] - drag.target_m) > 0.08


def test_derived_release_values_are_consistent():
    t = aim(speed_m_s=5.0, flight_time_s=0.8)
    assert t.release_speed_m_s == pytest.approx(np.linalg.norm(t.velocity_m_s))
    assert t.release_elevation_deg == pytest.approx(
        math.degrees(math.atan2(t.velocity_m_s[2], np.linalg.norm(t.velocity_m_s[:2])))
    )
    assert t.apex_z_m >= max(t.position_m[2], t.target_m[2])
    assert t.lowest_z_m <= min(t.position_m[2], t.target_m[2])
    # Re-derivation table (§2b, drag on): v 5 · T 0.8 · face-on → d 3.00, Δz −0.36.
    assert t.horizontal_distance_m == pytest.approx(3.000, abs=0.005)
    assert t.release_height_offset_m == pytest.approx(-0.357, abs=0.005)


def test_the_floor_refuses_a_release_under_the_table_and_only_then():
    # v 3 face-on over 0.8 s releases 1.5 m below the catch point (§2b): refused.
    with pytest.raises(ValueError, match="floor"):
        aim(speed_m_s=3.0, flight_time_s=0.8, floor_z_m=0.05)
    # The same throw with no floor is computed (and says where it would start).
    t = aim(speed_m_s=3.0, flight_time_s=0.8)
    assert t.position_m[2] < 0.05
    # A throw whose lowest ball-surface point is exactly the floor is refused;
    # a hair above is accepted.
    t = aim(speed_m_s=4.0, flight_time_s=0.65, floor_z_m=None)
    surface = t.lowest_z_m - params().radius_m
    with pytest.raises(ValueError):
        aim(speed_m_s=4.0, flight_time_s=0.65, floor_z_m=surface)
    aim(speed_m_s=4.0, flight_time_s=0.65, floor_z_m=surface - 1e-6)


def test_the_arrival_must_come_down_into_the_palm():
    axis_elev = math.degrees(math.atan2(AXIS[2], math.hypot(AXIS[0], AXIS[1])))
    with pytest.raises(ValueError, match="incidence"):
        aim(incidence_offset_deg=-axis_elev)  # horizontal
    with pytest.raises(ValueError, match="incidence"):
        aim(incidence_offset_deg=90.0 - axis_elev)  # vertical
    aim(incidence_offset_deg=89.0 - axis_elev)  # nearly vertical lob is fine


def test_bad_inputs_are_refused():
    with pytest.raises(ValueError):
        aim(speed_m_s=0.0)
    with pytest.raises(ValueError):
        aim(flight_time_s=-1.0)
    with pytest.raises(ValueError):
        aim(offset_m=-0.1)
    with pytest.raises(ValueError, match="vertical"):
        cm.aim_at_hand(
            P_C,
            (0.0, 0.0, 1.0),
            speed_m_s=5,
            flight_time_s=0.8,
            offset_m=0,
            offset_angle_deg=0,
            incidence_offset_deg=0,
            params=params(),
        )


def test_offset_angle_zero_is_lateral_and_ninety_is_up():
    lateral = aim(offset_m=0.2, offset_angle_deg=0.0)
    up = aim(offset_m=0.2, offset_angle_deg=90.0)
    d_lat = lateral.target_m - np.asarray(P_C)
    d_up = up.target_m - np.asarray(P_C)
    assert abs(d_lat[2]) < 1e-9 and abs(abs(d_lat[1]) - 0.2) < 1e-9  # sideways along y
    assert d_up[2] > 0.1 and abs(d_up[1]) < 1e-9  # up, in the vertical plane of the axis


def test_launch_request_carries_the_release_and_zero_spin():
    t = aim()
    req = cm.hand_near_throw_to_launch_request(t)
    assert [req["position"][k] for k in "xyz"] == pytest.approx(list(t.position_m))
    assert [req["velocity"][k] for k in "xyz"] == pytest.approx(list(t.velocity_m_s))
    assert req["angular_velocity"] == {"x": 0.0, "y": 0.0, "z": 0.0}


def test_closest_approach_finds_a_minimum_between_coarse_samples():
    # A parabola sampled at 20 Hz; the target sits on the exact path at t* = 0.437,
    # which is not a sample time. The Hermite reconstruction must still get ~0.
    times = np.arange(0.0, 1.01, 0.05)
    p0 = np.array([2.0, 0.3, 0.1])
    v0 = np.array([-3.0, -0.5, 4.5])
    g = np.array([0.0, 0.0, -G])
    pos = p0 + np.outer(times, v0) + 0.5 * np.outer(times**2, g)
    vel = v0 + np.outer(times, g)
    t_star = 0.437
    target = p0 + v0 * t_star + 0.5 * g * t_star**2
    dist, at, speed = cm.closest_approach(times, pos, vel, target)
    assert dist < 2e-4, dist  # cubic Hermite on a parabola with 50 ms steps
    assert at == pytest.approx(t_star, abs=2e-3)
    assert speed == pytest.approx(np.linalg.norm(v0 + g * t_star), rel=1e-3)
    # A target displaced perpendicular to the flight reports that miss (the
    # path's curvature over 5 cm, ~d²/2R with R = v²/g ≈ 2 m, is < 1 mm).
    v_star = v0 + g * t_star
    perp = np.cross(v_star, [0.0, 0.0, 1.0])
    perp /= np.linalg.norm(perp)
    dist_off, _, _ = cm.closest_approach(times, pos, vel, target + 0.05 * perp)
    assert dist_off == pytest.approx(0.05, abs=1e-3)
    # Unsorted samples are sorted first; one sample is handled.
    d2 = cm.closest_approach(times[::-1], pos[::-1], vel[::-1], target)[0]
    assert d2 == pytest.approx(dist, abs=1e-9)
    assert cm.closest_approach(times[:1], pos[:1], vel[:1], target)[0] == pytest.approx(
        np.linalg.norm(pos[0] - target)
    )
    with pytest.raises(ValueError):
        cm.closest_approach([], np.zeros((0, 3)), np.zeros((0, 3)), target)


def test_free_flight_prefix_stops_at_the_first_contact():
    times = np.arange(0.0, 0.5, 0.01)
    g = np.array([0.0, 0.0, -G])
    vel = np.array([-3.0, 0.0, 2.0]) + np.outer(times, g)
    assert cm.free_flight_prefix(times, vel) == times.size  # gravity alone never trips it
    vel[30:] += np.array([2.5, 0.0, 1.0])  # the hand: a 2.7 m/s jump in one step
    assert cm.free_flight_prefix(times, vel) == 30
    # A single sample is all free flight.
    assert cm.free_flight_prefix(times[:1], vel[:1]) == 1


def test_aim_check_from_truth_reads_the_launch_not_the_contact():
    """The truth ends at the hand, before the aimed point; the check must still
    say how close the flight WOULD have passed, and how well the samples up to
    the contact follow the flight law."""
    t = aim(speed_m_s=5.0, flight_time_s=0.8)
    flight = cm.integrate_flight(
        t.position_m, t.velocity_m_s, params(), horizon_s=0.8, step_s=0.01
    )
    # Truth: the sim's first sample is 6 ms after the request; contact at 0.55 s
    # deflects the ball, and the samples after it go elsewhere.
    fine = cm.integrate_flight(t.position_m, t.velocity_m_s, params(), horizon_s=0.9, step_s=1e-3)
    idx = np.arange(6, 900, 10)
    times, pos, vel = fine.time_s[idx], fine.position_m[idx].copy(), fine.velocity_m_s[idx].copy()
    cut = np.searchsorted(times, 0.55)
    vel[cut:] = vel[cut:] * -0.4 + np.array([0.0, 0.0, 1.0])
    pos[cut:] = pos[cut - 1] + np.cumsum(vel[cut:], axis=0) * 0.01
    res = cm.aim_check_from_truth(times, pos, vel, t.target_m, params())
    assert res["aim_error_m"] < 1e-4, res
    assert res["aim_pass_after_first_truth_s"] == pytest.approx(0.8 - 0.006, abs=2e-3)
    assert res["aim_pass_speed_m_s"] == pytest.approx(5.0, abs=1e-3)
    assert res["free_flight_samples"] == cut
    assert res["first_contact_after_first_truth_s"] == pytest.approx(times[cut] - times[0])
    assert res["model_rms_m"] < 1e-3
    # A launch 5 cm off the aimed release misses by about as much.
    res_off = cm.aim_check_from_truth(
        times, pos + np.array([0.0, 0.05, 0.0]), vel, t.target_m, params()
    )
    assert res_off["aim_error_m"] == pytest.approx(0.05, abs=3e-3)
    assert math.isnan(
        cm.aim_check_from_truth(times[:cut], pos[:cut], vel[:cut], t.target_m, params())[
            "first_contact_after_first_truth_s"
        ]
    )
    assert not np.any(
        flight.position_m[-1] != flight.position_m[-1]
    )  # (sanity: the fixture flight is finite)
