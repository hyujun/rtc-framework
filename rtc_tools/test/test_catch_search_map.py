"""catch_search_map (dynamic_catching L3 §4.1).

The search's verdicts belong to ``catch_search_batch`` (C++, tested in
rtc_controllers). Pinned here: the throws and their list file, the wakes the
flight model gives (instants, counts, frame, the drag law's acceleration, the
end of flight), every formula of the binding against hand-written trees, the
reductions and joins on hand-made rows, and one run of the installed binary on
a shipped profile (structural facts only).
"""

from __future__ import annotations

import csv
import json
import math
import re
from pathlib import Path

import numpy as np
import pytest
import yaml

from rtc_tools.analysis import (
    catch_search_map as m,
    catchability_map as cm,
    catching_throw_list as ctl,
)
from rtc_tools.analysis.catching_trials import (
    _wake_reject_reason,
    most_frequent_reason,
)

_REPO = Path(__file__).resolve().parents[2]
SOURCES = dict.fromkeys(("radius_m", "mass_kg", "drag_coefficient", "air_density_kg_m3"), "test")
BALL = cm.BallParams(0.0335, 0.057, 0.55, 1.204, SOURCES)
NO_DRAG = cm.BallParams(0.0335, 0.057, 0.0, 0.0, SOURCES)
AXES = {
    "distance_m": [3.0, 4.0],
    "azimuth_deg": [-20.0, 0.0, 30.0],
    "release_height_m": [1.5],
    "aim_deviation_deg": [0.0, 5.0],
    "speed_m_s": [6.0],
    "elevation_deg": [20.0, 35.0],
}


# ── throws ────────────────────────────────────────────────────────────────────


def test_grid_ids_are_the_kinematic_maps_throw_index():
    throws = m.grid_throws(AXES, origin_xy_m=(0.3, -0.2))
    grid = cm.generate_throw_grid(
        base_xy_m=(0.3, -0.2), **{m._GRID_KEYWORD[a]: v for a, v in AXES.items()}
    )
    assert [t["throw_id"] for t in throws] == list(range(len(grid)))
    for record, throw in zip(throws, grid, strict=True):
        assert record["pos"] == tuple(throw.position_m)
        assert record["vel"] == tuple(throw.velocity_m_s)
        assert record["omega"] == (0.0, 0.0, 0.0)
        assert all(record[a] == getattr(throw, a) for a in m.THROW_AXES)
    shifted = m.grid_throws(AXES, origin_xy_m=(0.3, -0.2), first_id=100, kind="x")
    assert [t["throw_id"] for t in shifted] == list(range(100, 100 + len(grid)))
    assert {t["kind"] for t in shifted} == {"x"}


def test_grid_needs_every_axis():
    with pytest.raises(ValueError, match="elevation_deg"):
        m.grid_throws({k: v for k, v in AXES.items() if k != "elevation_deg"}, origin_xy_m=(0, 0))


def test_lhs_hits_every_stratum_of_every_axis_once_and_replays_from_its_seed():
    ranges = m.axis_ranges(AXES)
    n = 7
    throws = m.lhs_throws(ranges, n, seed=5, origin_xy_m=(0.0, 0.0), first_id=40)
    assert [t["throw_id"] for t in throws] == list(range(40, 47))
    for axis, (lo, hi) in ranges.items():
        values = [t[axis] for t in throws]
        if hi == lo:
            assert values == [lo] * n
            continue
        strata = sorted(int((v - lo) / (hi - lo) * n) for v in values)
        assert strata == list(range(n))
    again = m.lhs_throws(ranges, n, seed=5, origin_xy_m=(0.0, 0.0), first_id=40)
    assert again == throws
    other = m.lhs_throws(ranges, n, seed=6, origin_xy_m=(0.0, 0.0), first_id=40)
    assert other != throws
    # the launch state is the one generate_throw_grid gives for the drawn axes
    (one,) = cm.generate_throw_grid(
        base_xy_m=(0.0, 0.0), **{m._GRID_KEYWORD[a]: (throws[3][a],) for a in m.THROW_AXES}
    )
    assert throws[3]["vel"] == tuple(one.velocity_m_s)


def test_throw_list_round_trips_bit_for_bit(tmp_path):
    throws = m.grid_throws(AXES, origin_xy_m=(0.1, 0.2))
    path = tmp_path / "list.json"
    m.write_throw_list(path, throws, {"note": "x"})
    back, meta = m.read_throw_list(path)
    assert meta == {"note": "x"}
    assert back == throws
    doc = json.loads(path.read_text())
    assert doc["schema"] == "catching_throw_list/1"
    assert doc["frame"] == "sim_world"


@pytest.mark.parametrize(
    ("mutate", "match"),
    [
        (lambda d: d.update(schema="x"), "schema"),
        (lambda d: d.update(frame="model_world"), "frame"),
        (lambda d: d["throws"][1].update(throw_id=0), "duplicated"),
        (lambda d: d["throws"][0].update(throw_id=1.0), "throw_id"),
        (lambda d: d["throws"][0].update(pos=[0, 0]), "pos"),
        (lambda d: d["throws"][0].pop("vel"), "vel"),
        (lambda d: d.update(throws=[]), "non-empty"),
    ],
)
def test_a_bad_throw_list_is_refused_naming_what(mutate, match):
    doc = {
        "schema": "catching_throw_list/1",
        "frame": "sim_world",
        "throws": [
            {"throw_id": 0, "pos": [0, 0, 1], "vel": [1, 2, 3]},
            {"throw_id": 1, "pos": [0, 0, 1], "vel": [1, 2, 3]},
        ],
    }
    mutate(doc)
    with pytest.raises(ValueError, match=match):
        m.parse_throw_list(doc, "src")


def test_the_list_is_read_and_written_by_the_one_format_module():
    """The map has no throw-list code of its own: the sim driver throws through the same
    module (pinned on its side), so a list the map writes is one the driver accepts."""
    assert m.parse_throw_list is ctl.parse_throw_list
    assert m.read_throw_list is ctl.load_throw_list
    assert m.write_throw_list is ctl.write_throw_list
    assert (m.THROW_LIST_SCHEMA, m.THROW_LIST_FRAME) == (
        ctl.THROW_LIST_SCHEMA,
        ctl.THROW_LIST_FRAME,
    )


def test_a_list_the_map_writes_passes_the_drivers_rules(tmp_path):
    # Grid and sample throws carry their axis values as extra keys: none of them
    # is a key the sim trial record writes itself.
    throws = m.grid_throws(AXES, origin_xy_m=(0.1, 0.2))
    throws += m.lhs_throws(m.axis_ranges(AXES), 3, 1, origin_xy_m=(0.1, 0.2), first_id=len(throws))
    assert not ctl.THROW_RECORD_KEYS.intersection(*throws)
    path = tmp_path / "list.json"
    m.write_throw_list(path, throws, {"tool": "x"})
    assert ctl.read_throw_list_file(path).throws == throws


# ── flight and wakes ──────────────────────────────────────────────────────────


def test_flight_states_are_the_integrators_states_and_the_drag_laws_acceleration():
    p0, v0 = (3.0, 0.5, 1.6), (-5.0, -0.4, 3.0)
    step = 0.002
    reference = cm.integrate_flight(p0, v0, BALL, horizon_s=1.0, step_s=step)
    picks = [0, 1, 7, 50, 333, 500]
    p, v, a = m.flight_states(
        p0, v0, BALL, [int(round(i * step * 1e9)) for i in picks], max_step_s=step
    )
    assert p == pytest.approx(reference.position_m[picks], abs=1e-12)
    assert v == pytest.approx(reference.velocity_m_s[picks], abs=1e-12)
    # the force law written out independently: g − ρ Cd A |v| v / (2 m)
    area = math.pi * 0.0335**2
    for vel, acc in zip(v, a, strict=True):
        drag = -0.5 * 1.204 * 0.55 * area * np.linalg.norm(vel) * vel / 0.057
        assert acc == pytest.approx(np.array([0.0, 0.0, -9.81]) + drag, abs=1e-12)


def test_flight_states_land_on_instants_that_are_no_multiple_of_the_step():
    p0, v0 = (0.0, 0.0, 2.0), (3.0, 0.0, 4.0)
    instants = [0, 33_333_333, 66_666_667, 1_000_000_001]
    p, v, _ = m.flight_states(p0, v0, NO_DRAG, instants, max_step_s=0.004)
    for t_ns, pos, vel in zip(instants, p, v, strict=True):
        t = t_ns * 1e-9
        assert pos == pytest.approx([3.0 * t, 0.0, 2.0 + 4.0 * t - 0.5 * 9.81 * t * t], abs=1e-12)
        assert vel == pytest.approx([3.0, 0.0, 4.0 - 9.81 * t], abs=1e-12)


@pytest.mark.parametrize("instants", [[], [5, 5], [7, 3], [-1, 2]])
def test_flight_states_refuse_instants_out_of_order(instants):
    with pytest.raises(ValueError):
        m.flight_states((0, 0, 1), (1, 0, 1), BALL, instants, max_step_s=0.002)


def _timing(**over) -> m.WakeTiming:
    kw = {
        "detection_delay_s": 0.15,
        "vision_period_s": 0.04,
        "prediction_dt_s": 0.05,
        "prediction_horizon_s": 1.0,
        "max_flight_s": 2.5,
        "floor_world_z_m": 0.0,
        "max_step_s": 0.002,
    }
    kw.update(over)
    return m.WakeTiming.from_seconds(**kw)


def test_wake_timing_takes_whole_ns_and_a_horizon_that_is_a_multiple_of_the_spacing():
    t = _timing()
    assert (t.detection_delay_ns, t.vision_period_ns, t.prediction_dt_ns) == (
        150_000_000,
        40_000_000,
        50_000_000,
    )
    assert t.prediction_points == 20
    offsets = t.wake_offsets_ns()
    assert offsets[0] == 150_000_000 and offsets[-1] <= 2_500_000_000
    assert offsets[-1] + 40_000_000 > 2_500_000_000
    with pytest.raises(ValueError, match="multiple"):
        _timing(prediction_horizon_s=1.02)
    with pytest.raises(ValueError):
        _timing(vision_period_s=0.0)


YAW90 = cm.make_transform(cm.rotation_z(math.pi / 2), (0.2, -0.1, -0.7))


def _lob(throw_id=3) -> dict:
    return {"throw_id": throw_id, "pos": (2.5, 0.3, 1.5), "vel": (-4.0, 0.0, 3.5)}


def test_wakes_stand_every_period_from_the_delay_and_carry_the_estimator_grid():
    timing = _timing()
    wakes = m.throw_wakes(_lob(), BALL, timing, YAW90)
    assert wakes
    for i, wake in enumerate(wakes):
        assert wake.throw_id == 3 and wake.wake == i
        assert wake.now_ns == m.RELEASE_NS + 150_000_000 + i * 40_000_000
        steps = np.arange(1, 21, dtype=np.int64) * 50_000_000
        assert np.array_equal(wake.t_ns, wake.now_ns + steps)
        assert wake.position_m.shape == (20, 3)
    instants = np.concatenate([w.now_ns + np.zeros(1, np.int64) for w in wakes])
    assert np.all(np.diff(instants) > 0)


def test_wake_samples_are_the_flight_moved_into_the_model_world():
    timing = _timing()
    wake = m.throw_wakes(_lob(), BALL, timing, YAW90)[2]
    offsets = [int(t) - m.RELEASE_NS for t in wake.t_ns]
    p_w, v_w, a_w = m.flight_states(_lob()["pos"], _lob()["vel"], BALL, offsets, max_step_s=0.002)
    rotation = cm.rotation_z(math.pi / 2)
    assert wake.position_m == pytest.approx(p_w @ rotation.T + [0.2, -0.1, -0.7], abs=1e-9)
    assert wake.velocity_m_s == pytest.approx(v_w @ rotation.T, abs=1e-9)
    assert wake.acceleration_m_s2 == pytest.approx(a_w @ rotation.T, abs=1e-9)
    # gravity stays vertical under a yaw: the drag law's z is the model world's z
    assert wake.acceleration_m_s2[:, 2] == pytest.approx(a_w[:, 2])


def test_the_last_wake_is_the_last_one_with_the_ball_above_the_floor():
    timing = _timing(floor_world_z_m=0.5)
    wakes = m.throw_wakes(_lob(), BALL, timing, YAW90)
    last = wakes[-1].now_ns - m.RELEASE_NS
    after = last + timing.vision_period_ns
    (z_last, z_after), _, _ = m.flight_states(
        _lob()["pos"], _lob()["vel"], BALL, [last, after], max_step_s=0.002
    )
    assert z_last[2] >= 0.5 > z_after[2]
    # a shorter flight horizon ends the wakes earlier, never later
    assert len(m.throw_wakes(_lob(), BALL, _timing(max_flight_s=0.3), YAW90)) == 4
    # a throw already below the floor at its first wake has none
    low = {"throw_id": 9, "pos": (0.0, 0.0, 0.1), "vel": (1.0, 0.0, -1.0)}
    assert m.throw_wakes(low, BALL, timing, YAW90) == []


def test_the_wake_csv_is_one_row_per_sample_at_round_trip_precision():
    wakes = m.throw_wakes(_lob(), BALL, _timing(max_flight_s=0.3), YAW90)
    lines = m.wake_csv_lines(wakes)
    assert lines[0] == m.WAKE_CSV_HEADER
    rows = list(csv.DictReader(lines))
    assert len(rows) == 20 * len(wakes)
    first = rows[0]
    assert int(first["now_ns"]) == wakes[0].now_ns and int(first["t_ns"]) == wakes[0].t_ns[0]
    assert float(first["p_x"]) == wakes[0].position_m[0, 0]
    assert float(rows[-1]["a_z"]) == wakes[-1].acceleration_m_s2[-1, 2]


def test_flight_axes_are_the_closest_approach_of_the_flight():
    throw = {"throw_id": 0, "pos": (0.0, 0.0, 2.0), "vel": (4.0, 0.0, 0.0)}
    # without drag the ball is at x = 4 t: it passes (2, 0, 2 − g/2 · 0.25) at t = 0.5
    target = (2.0, 0.0, 2.0 - 0.5 * 9.81 * 0.25)
    axes = m.flight_axes(throw, NO_DRAG, target, horizon_s=1.5, step_s=0.002)
    assert axes["closest_distance_m"] == pytest.approx(0.0, abs=1e-6)
    assert axes["flight_time_s"] == pytest.approx(0.5, abs=1e-6)
    assert axes["terminal_speed_m_s"] == pytest.approx(math.hypot(4.0, 9.81 * 0.5), abs=1e-6)


def test_one_integration_gives_the_wakes_and_the_axes_bit_for_bit():
    timing = _timing(floor_world_z_m=0.5, vision_period_s=0.033333333)
    target = (0.5, 0.3, 1.2)
    pos, vel = _lob()["pos"], _lob()["vel"]
    wakes, axes = m.throw_flight(_lob(), BALL, timing, np.eye(4), target, horizon_s=2.5)
    # the samples are flight_states over the samples of the wakes in flight —
    # exactly what the wakes alone integrate (identity transform: no rounding)
    samples = sorted({int(t) - m.RELEASE_NS for w in wakes for t in w.t_ns})
    p, v, a = m.flight_states(pos, vel, BALL, samples, max_step_s=0.002)
    row_of = {t: i for i, t in enumerate(samples)}
    alone = m.throw_wakes(_lob(), BALL, timing, np.eye(4))
    assert [w.now_ns for w in alone] == [w.now_ns for w in wakes]
    for wake, same in zip(wakes, alone, strict=True):
        rows = [row_of[int(t) - m.RELEASE_NS] for t in wake.t_ns]
        assert np.array_equal(wake.position_m, p[rows])
        assert np.array_equal(wake.velocity_m_s, v[rows])
        assert np.array_equal(wake.acceleration_m_s2, a[rows])
        assert np.array_equal(wake.position_m, same.position_m)
    # each wake is judged on flight_states over the samples before it and the wake
    offsets = timing.wake_offsets_ns()
    for i, offset in enumerate(offsets[: len(wakes) + 1]):
        p_at, _, _ = m.flight_states(
            pos, vel, BALL, [t for t in samples if t < offset] + [offset], max_step_s=0.002
        )
        assert (p_at[-1, 2] >= 0.5) == (i < len(wakes))
    # the axes are the closest approach over every integrator state of that flight
    flight = m.integrate_throw(_lob(), BALL, max_step_s=0.002, timing=timing, horizon_s=2.5)
    assert np.array_equal(flight.sample_instants_ns, samples)
    assert np.array_equal(flight.sample_position_w, p)
    end = int(np.searchsorted(flight.path_time_s, 2.5)) + 1
    assert np.all(np.diff(flight.path_time_s) > 0) and flight.path_time_s[end - 1] >= 2.5 - 1e-9
    distance, at, speed = cm.closest_approach(
        flight.path_time_s[:end],
        flight.path_position_w[:end],
        flight.path_velocity_w[:end],
        target,
    )
    assert axes == {
        "flight_time_s": at,
        "terminal_speed_m_s": speed,
        "closest_distance_m": distance,
    }
    # alone, the axes are the closest approach over integrate_flight's own steps
    reference = cm.integrate_flight(pos, vel, BALL, horizon_s=2.5, step_s=0.002)
    separate = m.flight_axes(_lob(), BALL, target, horizon_s=2.5, step_s=0.002)
    distance, at, speed = cm.closest_approach(
        reference.time_s, reference.position_m, reference.velocity_m_s, target
    )
    assert separate == {
        "flight_time_s": at,
        "terminal_speed_m_s": speed,
        "closest_distance_m": distance,
    }
    # the two samplings agree to where the refinement lands on the flat minimum
    assert separate["closest_distance_m"] == pytest.approx(axes["closest_distance_m"], abs=1e-9)
    assert separate == pytest.approx(axes, abs=1e-7)


# ── binding ───────────────────────────────────────────────────────────────────

ARM = ("a0", "a1", "a2")
HAND = ("h0", "h1")
# The model's velocity order differs from the device order: a swapped map shows.
MODEL = ("a2", "a0", "a1")


def _limits(names, lower, upper, velocity, torque, device="arm") -> m.DeviceLimits:
    as_array = lambda v: None if v is None else np.asarray(v, dtype=float)  # noqa: E731
    return m.DeviceLimits(
        device, tuple(names), as_array(lower), as_array(upper), as_array(velocity),
        as_array(torque),
    )  # fmt: skip


def _facts(**over) -> m.RobotFacts:
    kw = {
        "control_rate_hz": 500.0,
        "arm": _limits(ARM, [-3.0, -2.0, -1.0], [3.0, 2.0, -0.95], [2.0, 3.0, 4.0], [100, 50, 20]),
        "hand": _limits(HAND, [0.0, 0.0], [1.0, 1.0], [5.0, 5.0], [1.0, 1.0], device="hand"),
        "model_joint_names": MODEL,
        "urdf_position_lower": np.array([-1.2, -2.9, -2.5]),  # model order
        "urdf_position_upper": np.array([1.2, 2.9, 2.5]),
    }
    kw.update(over)
    return m.RobotFacts(**kw)


def _tree() -> dict:
    nlp_core = {"catch": {"nu_ref": [0.0, 0.0, -2.0]}}
    docking_core = {"catch": {"nu_ref": [0.0, 0.0, -1.25]}}
    return {
        "core": {"ball": {"mass": 0.057}},
        "robot": {
            "arm": {"limit_margin": 0.05, "qdd_max": [10.0, 20.0, 30.0]},
            "hand": {"T_close_e2e": 0.3, "T_close_lead": 0.25, "docking": {"s_ent": 0.01}},
        },
        "joint_cmd": {
            "accel_constraint": "dynamic",
            "eta_tau": 0.8,
            "lag": {"T_arm": 0.05, "lead_enable": True},
        },
        "planner": {
            "search": {
                "mode": "grid",
                "grid": {
                    "gamma": {"eta_v": 0.85},
                    "reference": {"v_max": 3.5, "omega": 10.0, "zeta": 1.0, "a_max": 30.0},
                    "stop": {"a_dec": 10.0},
                },
                "nlp": {"core": nlp_core},
            },
            "segment": {
                "mode": "mpc",
                "mpc": {"eta_v": 0.7},
                "mpc_docking": {"eta_v": 0.6, "core": docking_core},
            },
        },
    }


def _set(tree: dict, path: str, value) -> dict:
    node = tree
    keys = path.split(".")
    for key in keys[:-1]:
        node = node.setdefault(key, {})
    if value is m._ABSENT:
        node.pop(keys[-1], None)
    else:
        node[keys[-1]] = value
    return tree


def test_grid_binding_formulas():
    b = m.search_binding(_tree(), _facts(), "grid")
    doc = b.document
    assert doc["search"] == "grid"
    assert doc["device_of_model"] == [2, 0, 1]
    g = doc["grid"]
    assert g["qdot_max"] == [4.0, 2.0, 3.0]  # device ratings, model order
    assert g["qddot_max"] == [30.0, 10.0, 20.0]
    assert g["eta_v"] == 0.85
    assert (g["v_max"], g["a_dec"], g["ref_omega"], g["ref_zeta"], g["ref_a_max"]) == (
        3.5,
        10.0,
        10.0,
        1.0,
        30.0,
    )
    assert g["t_arm_s"] == 0.05
    assert g["t_close_lead"] == 0.25  # segment mode mpc: the profile's lead as written
    assert g["t_close_total"] == 0.3 + 0.5 * 0.002
    assert g["control_dt"] == 1.0 / 500.0
    assert g["ball_mass"] == 0.057
    assert g["follows_segments"] is True
    # every value says where it came from and which controller function it mirrors
    for key in [*g, "device_of_model"]:
        entry = b.sources[key if key == "device_of_model" else f"grid.{key}"]
        assert entry["source"] and "::" in entry["mirrors"]
    assert b.sources["grid.t_close_lead"]["mirror_param"] == "hand.T_close_lead_from_t_c"


def test_tbd_and_absent_open_keys_bind_nan_and_the_file_says_nan():
    tree = _tree()
    _set(tree, "planner.search.grid.reference.v_max", "TBD")
    _set(tree, "planner.search.grid.reference.omega", "TBD")
    _set(tree, "planner.search.grid.stop.a_dec", m._ABSENT)
    _set(tree, "robot.hand.T_close_e2e", "TBD")
    _set(tree, "core.ball.mass", m._ABSENT)
    b = m.search_binding(tree, _facts(), "grid")
    g = b.document["grid"]
    for key in ("v_max", "ref_omega", "a_dec", "t_close_total", "ball_mass"):
        assert math.isnan(g[key]), key
    assert g["t_close_lead"] == 0.25  # the written lead does not depend on T_close_e2e
    text = b.yaml_text()
    assert "v_max: .nan" in text
    assert yaml.safe_load(text)["grid"]["eta_v"] == 0.85


def test_the_lead_and_the_arm_delay_follow_their_rules():
    tree = _set(_tree(), "robot.hand.T_close_lead", m._ABSENT)
    assert m.search_binding(tree, _facts(), "grid").document["grid"]["t_close_lead"] == 0.3
    tree = _set(_tree(), "robot.hand.T_close_lead", "TBD")
    assert math.isnan(m.search_binding(tree, _facts(), "grid").document["grid"]["t_close_lead"])
    tree = _set(_tree(), "joint_cmd.lag.lead_enable", False)
    assert m.search_binding(tree, _facts(), "grid").document["grid"]["t_arm_s"] == 0.0
    tree = _set(_tree(), "joint_cmd.lag.T_arm", "TBD")
    assert m.search_binding(tree, _facts(), "grid").document["grid"]["t_arm_s"] == 0.0
    # whole nanoseconds, truncated as the controller's integer cast does
    tree = _set(_tree(), "joint_cmd.lag.T_arm", 0.0123456789012)
    assert m.search_binding(tree, _facts(), "grid").document["grid"]["t_arm_s"] == (
        12345678 * 1e-9
    )


def test_closed_form_does_not_follow_segments_and_no_box_binds_an_empty_one():
    tree = _set(_tree(), "planner.segment.mode", "closed_form")
    _set(tree, "robot.arm.qdd_max", m._ABSENT)
    g = m.search_binding(tree, _facts(), "grid").document["grid"]
    assert g["follows_segments"] is False
    assert g["qddot_max"] == []


def test_mpc_docking_binds_the_entrance_lead_of_the_docking_core():
    tree = _set(_tree(), "planner.segment.mode", "mpc_docking")
    b = m.search_binding(tree, _facts(), "grid")
    # T_close_lead − s_ent / c with c = −nu_ref.z of the DOCKING planner's core
    assert b.document["grid"]["t_close_lead"] == pytest.approx(0.25 - 0.01 / 1.25)
    assert "mpc_docking.core.catch.nu_ref" in b.sources["grid.t_close_lead"]["source"]
    nlp = m.search_binding(tree, _facts(), "nlp").document["nlp"]
    assert nlp["t_close_lead"] == pytest.approx(0.25 - 0.01 / 1.25)
    # the search's own core gives the hand its lead: the nlp core's closing speed
    assert nlp["hand_t_close_lead"] == pytest.approx(0.25 - 0.01 / 2.0)
    # a missing s_ent leaves the entrance lead unknown, not zero
    _set(tree, "robot.hand.docking.s_ent", "TBD")
    assert math.isnan(m.search_binding(tree, _facts(), "grid").document["grid"]["t_close_lead"])


def test_nlp_limits_are_the_urdf_and_the_margined_device_box():
    b = m.search_binding(_tree(), _facts(), "nlp")
    n = b.document["nlp"]
    lim = n["limits"]
    # model order (a2, a0, a1). a2's device range [-1, -0.95] is narrower than twice
    # the margin: both ends sit on its midpoint −0.975. a0's URDF range is inside its
    # margined device range and binds; a1's margined device range is inside the URDF's.
    assert lim["q_min"] == pytest.approx([-0.975, -2.9, -1.95])
    assert lim["q_max"] == pytest.approx([-0.975, 2.9, 1.95])
    # ResolvedSegmentEtaV under segment mode mpc: planner.segment.mpc.eta_v
    assert lim["qd_max"] == pytest.approx([0.7 * 4.0, 0.7 * 2.0, 0.7 * 3.0])
    assert lim["qdd_max"] == [30.0, 10.0, 20.0]
    assert lim["tau_max"] == [20.0, 100.0, 50.0]
    assert lim["tau_lo"] == pytest.approx([-16.0, -80.0, -40.0])
    assert lim["tau_hi"] == pytest.approx([16.0, 80.0, 40.0])
    assert n["hand_t_close_e2e"] == 0.3
    assert n["t_close_lead"] == 0.25
    assert n["hand_t_close_lead"] == pytest.approx(0.25 - 0.01 / 2.0)
    assert n["t_arm_s"] == 0.05 and n["control_dt"] == 0.002 and n["ball_mass"] == 0.057
    for key in ("q_min", "qd_max", "tau_lo"):
        assert "BuildDockingLimits" in b.sources[f"nlp.limits.{key}"]["mirrors"]


def test_nlp_speed_margin_follows_the_segment_mode():
    tree = _set(_tree(), "planner.segment.mode", "mpc_docking")
    lim = m.search_binding(tree, _facts(), "nlp").document["nlp"]["limits"]
    assert lim["qd_max"] == pytest.approx([0.6 * 4.0, 0.6 * 2.0, 0.6 * 3.0])


def test_an_incomplete_hand_box_drops_the_clik_box_and_leaves_the_urdf_limits():
    hand = _limits(HAND, [0.0, np.inf], [1.0, 1.0], [5.0, 5.0], [1.0, 1.0], device="hand")
    lim = m.search_binding(_tree(), _facts(hand=hand), "nlp").document["nlp"]["limits"]
    assert lim["q_min"] == [-1.2, -2.9, -2.5]
    assert lim["q_max"] == [1.2, 2.9, 2.5]
    # a hand whose arrays are absent runs the controller's fallback, which is a box
    absent = _limits(HAND, None, None, [5.0, 5.0], [1.0, 1.0], device="hand")
    lim = m.search_binding(_tree(), _facts(hand=absent), "nlp").document["nlp"]["limits"]
    assert lim["q_min"][2] == pytest.approx(-1.95)


def test_the_torque_margin_follows_the_clik_form():
    tree = _set(_tree(), "joint_cmd.eta_tau", "TBD")
    assert m.search_binding(tree, _facts(), "nlp").document["nlp"]["limits"]["tau_hi"][0] == 20.0
    tree = _set(_tree(), "joint_cmd.accel_constraint", "kinematic")
    _set(tree, "joint_cmd.eta_tau", m._ABSENT)
    assert m.search_binding(tree, _facts(), "nlp").document["nlp"]["limits"]["tau_hi"][0] == 20.0


@pytest.mark.parametrize(
    ("kind", "path", "value", "named"),
    [
        ("grid", "planner.search.grid.gamma.eta_v", m._ABSENT, "planner.search.grid.gamma.eta_v"),
        ("grid", "planner.search.grid.gamma.eta_v", "TBD", "planner.search.grid.gamma.eta_v"),
        ("grid", "planner.search.grid.reference.zeta", m._ABSENT, "reference.zeta"),
        ("grid", "planner.search.grid", m._ABSENT, "planner.search.grid"),
        ("grid", "planner.segment.mode", m._ABSENT, "planner.segment.mode"),
        ("grid", "planner.segment.mode", "docking", "planner.segment.mode"),
        ("grid", "joint_cmd.lag.lead_enable", m._ABSENT, "joint_cmd.lag.lead_enable"),
        ("grid", "joint_cmd.lag.T_arm", m._ABSENT, "joint_cmd.lag.T_arm"),
        ("grid", "robot.arm.qdd_max", [1.0, 2.0], "robot.arm.qdd_max"),
        ("grid", "robot.hand.T_close_e2e", "fast", "robot.hand.T_close_e2e"),
        ("nlp", "planner.search.nlp", m._ABSENT, "planner.search.nlp.core"),
        ("nlp", "planner.search.nlp.core.catch.nu_ref", m._ABSENT, "nlp.core.catch.nu_ref"),
        ("nlp", "planner.segment.mpc.eta_v", m._ABSENT, "planner.segment.mpc.eta_v"),
        ("nlp", "robot.arm.limit_margin", "TBD", "robot.arm.limit_margin"),
        ("nlp", "joint_cmd.eta_tau", m._ABSENT, "joint_cmd.eta_tau"),
        ("nlp", "joint_cmd.accel_constraint", "TBD", "joint_cmd.accel_constraint"),
    ],
)
def test_a_missing_input_is_refused_naming_the_key(kind, path, value, named):
    tree = _set(_tree(), path, value)
    with pytest.raises(m.BindingError, match=re.escape(named)):
        m.search_binding(tree, _facts(), kind)


def test_a_device_array_the_controller_would_replace_by_a_constant_is_refused():
    arm = _limits(ARM, [-3.0, -2.0, -1.0], [3.0, 2.0, -0.95], None, [100, 50, 20])
    with pytest.raises(m.BindingError, match=r"devices\.arm\.joint_limits\.max_velocity"):
        m.search_binding(_tree(), _facts(arm=arm), "grid")


def test_merged_device_limits_take_the_tighter_bound_of_yaml_and_urdf():
    urdf = {
        "a": {"position_lower": -2.0, "position_upper": 2.0, "max_velocity": 3.0, "max_torque": 9},
        "b": {"position_lower": -1.0, "position_upper": 1.0, "max_velocity": 1.0, "max_torque": 4},
    }
    params = {
        "devices": {
            "d": {
                "joint_state_names": ["a", "b"],
                "joint_limits": {
                    "position_lower": [-3.0, -0.5],
                    "position_upper": [1.5, 3.0],
                    "max_velocity": [2.0, 2.0],
                },
            }
        }
    }
    got = m.merged_device_limits(params, "d", urdf)
    assert got.position_lower.tolist() == [-2.0, -0.5]
    assert got.position_upper.tolist() == [1.5, 1.0]
    assert got.max_velocity.tolist() == [2.0, 1.0]
    assert got.max_torque is None  # the block exists and does not give it
    del params["devices"]["d"]["joint_limits"]
    got = m.merged_device_limits(params, "d", urdf)
    assert got.max_torque.tolist() == [9.0, 4.0]  # no block at all: every array from the URDF
    params["devices"]["d"]["joint_limits"] = {"max_velocity": [1.0]}
    with pytest.raises(m.BindingError, match="max_velocity"):
        m.merged_device_limits(params, "d", urdf)


def _joint(name, parent, child, kind="revolute", lower=-1.0, upper=1.0):
    limit = f'lower="{lower}" upper="{upper}" ' if kind != "continuous" else ""
    return (
        f'<joint name="{name}" type="{kind}"><parent link="{parent}"/><child link="{child}"/>'
        f'<axis xyz="0 0 1"/><limit {limit}velocity="3.0" effort="40.0"/></joint>'
    )


def _robot(hand_first_kind="continuous", arm_kind="revolute"):
    """Arm a0..a2 (base → l3), then a hand h0, h1 after it: a URDF and its robot params."""
    links = "".join(f'<link name="{n}"/>' for n in ("base", "l1", "l2", "l3", "l4", "l5"))
    joints = (
        _joint("a0", "base", "l1", arm_kind)
        + _joint("a1", "l1", "l2", lower=-2.0, upper=2.0)
        + _joint("a2", "l2", "l3", lower=-1.5, upper=1.5)
        + _joint("h0", "l3", "l4", hand_first_kind)
        + _joint("h1", "l4", "l5", lower=0.0, upper=1.6)
    )
    urdf = f'<robot name="r">{links}{joints}</robot>'
    params = {
        "control_rate": 500.0,
        "devices": {
            "arm": {"joint_state_names": list(ARM)},
            "hand": {"joint_state_names": list(HAND)},
        },
        "urdf": {"sub_models": {"catch": {"root_link": "base", "tip_link": "l3"}}},
    }
    return urdf, params


def test_a_hand_with_a_continuous_joint_binds_both_searches():
    pin = pytest.importorskip("pinocchio")
    urdf, params = _robot()
    # the grid binding reads nothing of the hand: it is not looked up
    facts = m.robot_facts(params, "arm", None, urdf, "catch")
    assert facts.hand is None
    assert m.search_binding(_tree(), facts, "grid").document["grid"]["qdot_max"] == [3.0] * 3
    with pytest.raises(m.BindingError, match="hand"):
        m.search_binding(_tree(), facts, "nlp")
    # the nlp binding reads the hand as the controller manager does: entry
    # joint_id − 1 of the limit vectors, here the continuous joint's cos / sin bounds
    facts = m.robot_facts(params, "arm", "hand", urdf, "catch")
    model = pin.buildModelFromXML(urdf)
    assert facts.hand.position_lower.tolist() == model.lowerPositionLimit[3:5].tolist()
    assert facts.hand.position_upper.tolist() == model.upperPositionLimit[3:5].tolist()
    lim = m.search_binding(_tree(), facts, "nlp").document["nlp"]["limits"]
    # finite and ordered, so the CLIK box stands: the margined arm box binds (a0's
    # device range ±1 is inside its URDF range; the margin is 0.05)
    assert lim["q_min"][0] == pytest.approx(-0.95) and lim["q_max"][0] == pytest.approx(0.95)
    # an ARM joint whose limits the manager reads from another entry is still refused
    urdf, params = _robot(hand_first_kind="revolute", arm_kind="continuous")
    with pytest.raises(m.BindingError, match="a0"):
        m.robot_facts(params, "arm", None, urdf, "catch")


# ── reduction ─────────────────────────────────────────────────────────────────


def _row(throw_id, wake, valid=0, plan_reason=0, nlp_reason="off", t_c_ns=math.nan, lead=0.5):
    return {
        "throw_id": throw_id,
        "wake": wake,
        "now_ns": m.RELEASE_NS + 100_000_000 + wake * 40_000_000,
        "search_valid": valid,
        "plan_reason": plan_reason,
        "nlp_reason": nlp_reason,
        "t_c_ns": t_c_ns,
        "lead_s": lead if valid else math.nan,
    }


def test_most_frequent_reason_breaks_ties_to_the_later_one():
    assert most_frequent_reason(["a", "b", "a", "b"]) == "b"
    assert most_frequent_reason(["b", "a", "a", "b", "c"]) == "b"
    assert most_frequent_reason(["a", "a", "b"]) == "a"
    assert most_frequent_reason([]) == ""


def test_a_batch_wake_reads_like_a_published_planner_event():
    row = _row(0, 0, plan_reason=4)
    assert m.wake_reason(row) == "search:reach_time"
    assert m.wake_reason(row) == _wake_reject_reason({**row, "outcome": "published"})
    assert m.wake_reason(_row(0, 0, plan_reason=0, nlp_reason="no_candidate")) == (
        "search:no_candidate"
    )


def test_throw_verdicts():
    plan_tc = m.RELEASE_NS + 900_000_000
    rows = [
        # 0: two refusals, then a plan; the row after it is never read
        _row(0, 0, plan_reason=4),
        _row(0, 1, plan_reason=11),
        _row(0, 2, valid=1, t_c_ns=plan_tc, lead=0.72),
        _row(0, 3, plan_reason=6),
        # 1: refused — reach_time twice, gamma_window twice: the later one wins the tie
        _row(1, 0, plan_reason=4),
        _row(1, 1, plan_reason=6),
        _row(1, 2, plan_reason=4),
        _row(1, 3, plan_reason=6),
        # 3: the nlp search's reason
        _row(3, 0, nlp_reason="no_candidate"),
    ]
    got = m.throw_verdicts([0, 1, 2, 3], rows, {0: 5, 1: 4, 3: 1})
    by = {v["throw_id"]: v for v in got}
    assert [v["throw_id"] for v in got] == [0, 1, 2, 3]
    a = by[0]
    assert a["accepted"] and a["first_accept_wake"] == 2 and a["reject_reason"] == ""
    assert a["plan_after_release_s"] == pytest.approx(0.18)
    assert a["t_c_s"] == pytest.approx(0.9) and a["lead_s"] == 0.72
    assert a["wakes_given"] == 5 and a["wakes_run"] == 3
    assert a["wake_reasons"] == {"search:reach_time": 1, "search:horizon_short": 1}
    r = by[1]
    assert not r["accepted"] and r["reject_reason"] == "search:gamma_window"
    assert r["wake_reasons"] == {"search:reach_time": 2, "search:gamma_window": 2}
    assert math.isnan(r["t_c_s"]) and r["first_accept_wake"] is None
    assert by[2]["reject_reason"] == m.REASON_NO_WAKE and by[2]["wakes_given"] == 0
    assert by[3]["reject_reason"] == "search:no_candidate"


def _verdict(throw_id, accepted, reason="", **axes):
    return {
        "throw_id": throw_id,
        "accepted": accepted,
        "reject_reason": "" if accepted else reason,
        "wake_reasons": {} if accepted else {reason: 2},
        "wakes_given": 2,
        **axes,
    }


def test_acceptance_by_axis_groups_by_value_or_bins_and_keeps_empty_bins():
    rows = [
        _verdict(0, True, speed_m_s=5.0, flight_time_s=0.8),
        _verdict(1, False, "x", speed_m_s=5.0, flight_time_s=1.0),
        _verdict(2, True, speed_m_s=7.0, flight_time_s=1.4),
        _verdict(3, False, "x", speed_m_s=7.0, flight_time_s=math.nan),
    ]
    by_value = m.acceptance_by_axis(rows, "speed_m_s")
    assert [(b["value"], b["n"], b["accepted"], b["rate"]) for b in by_value] == [
        (5.0, 2, 1, 0.5),
        (7.0, 2, 1, 0.5),
    ]
    bins = m.acceptance_by_axis(rows, "flight_time_s", [0.8, 0.9, 1.1, 1.3, 1.4])
    assert [(b["n"], b["accepted"], b["rate"]) for b in bins] == [
        (1, 1, 1.0),
        (1, 0, 0.0),
        (0, 0, None),  # empty bin
        (1, 1, 1.0),  # the last edge belongs to the last bin
    ]
    assert m.axis_edges(rows, "flight_time_s", 2) == pytest.approx([0.8, 1.1, 1.4])
    assert m.axis_edges(rows[:1], "flight_time_s", 2) is None


def test_reason_distribution_counts_throws_once_and_wakes_unreduced():
    verdicts = [
        _verdict(0, False, "search:a"),
        _verdict(1, False, "search:b"),
        _verdict(2, False, "search:a"),
        {**_verdict(3, True), "wake_reasons": {"search:b": 3}},
    ]
    got = m.reason_distribution(verdicts)
    assert got["throws"] == {"search:a": 2, "search:b": 1}
    assert got["wakes"] == {"search:a": 4, "search:b": 5}
    assert got["wakes_of_refused_throws"] == {"search:a": 4, "search:b": 2}


def test_search_disagreement_lists_each_side_and_what_the_other_said():
    grid = [_verdict(0, True), _verdict(1, False, "g1"), _verdict(2, True), _verdict(4, True)]
    nlp = [_verdict(0, True), _verdict(1, True), _verdict(2, False, "n2"), _verdict(3, False, "n")]
    got = m.search_disagreement(grid, nlp)
    assert got["throws"] == 3 and got["both_accepted"] == 1 and got["both_refused"] == 0
    assert got["only_grid"] == [{"throw_id": 2, "nlp_reason": "n2"}]
    assert got["only_nlp"] == [{"throw_id": 1, "grid_reason": "g1"}]
    assert got["missing_in_grid"] == [3] and got["missing_in_nlp"] == [4]


AXIS_A = {"distance_m": 3.0, "azimuth_deg": 0.0, "release_height_m": 1.5}
AXIS_REST = {"aim_deviation_deg": 0.0, "speed_m_s": 6.0, "elevation_deg": 30.0}


def _axes(distance):
    return {**AXIS_A, "distance_m": distance, **AXIS_REST}


def test_gate_verdicts_and_the_layered_join():
    throw_axes = {0: _axes(3.0), 1: _axes(3.5), 2: _axes(4.0), 3: _axes(4.5)}
    gate_rows = [
        {"seed_id": 0, "throw_index": 0, "reason_box": "reach_time", "reason_torque": "none"},
        {"seed_id": 0, "throw_index": 0, "reason_box": "none", "reason_torque": "none"},
        {"seed_id": 0, "throw_index": 1, "reason_box": "gamma_window_empty", "reason_torque": "x"},
        {"seed_id": 0, "throw_index": 1, "reason_box": "reach_time", "reason_torque": "x"},
        {"seed_id": 1, "throw_index": 2, "reason_box": "none", "reason_torque": "none"},
    ]
    gates = m.gate_throw_verdicts(throw_axes, gate_rows, seed_id=0)
    by = {g["throw_index"]: g for g in gates}
    assert by[0]["gate_box"] == "open" and by[0]["gate_torque"] == "open"
    assert by[1]["gate_box"] == "closed" and by[1]["gate_reason_box"] == "reach_time"
    assert by[2]["gate_box"] == "no_candidate"  # its only candidate is another wait pose's
    rows = [
        {**_verdict(10, False, "search:reach_time"), **_axes(3.0)},
        {**_verdict(11, True), **_axes(3.5)},
        {**_verdict(12, True), **_axes(4.0)},
        {**_verdict(13, True), **_axes(5.0)},  # not in the gate map's grid
    ]
    joined, summary = m.gate_map_join(rows, gates)
    assert [j["throw_id"] for j in joined] == [10, 11, 12]
    assert summary["matched"] == 3
    assert summary["search_throws_not_in_gate_map"] == 1
    assert summary["gate_throws_not_in_search_map"] == 1
    box = summary["layers"]["box"]
    assert box["table"]["open"] == {"accepted": 0, "refused": 1}
    assert box["table"]["closed"] == {"accepted": 1, "refused": 0}
    assert box["table"]["no_candidate"] == {"accepted": 1, "refused": 0}
    assert box["gate_open_search_refused_by_search_reason"] == {"search:reach_time": 1}
    assert box["gate_not_open_search_accepted_by_gate_reason"] == {
        "no_candidate": 1,
        "reach_time": 1,
    }


def test_the_sim_table_maps_plan_verdicts_onto_the_search_verdict():
    assert m.sim_search_class("published") == m.SIM_ACCEPTED_PUBLISHED
    assert m.sim_search_class("withheld") == m.SIM_ACCEPTED_NOT_PUBLISHED
    assert m.sim_search_class("no_plan") == m.SIM_REFUSED
    assert m.sim_search_class("no_search") == m.SIM_REFUSED
    assert m.sim_search_class("") is None
    sim = [
        {"idx": 0, "throw_id": "0", "plan_verdict": "published", "truth_success": "True"},
        {"idx": 1, "throw_id": "1", "plan_verdict": "published", "truth_success": "False"},
        {"idx": 2, "throw_id": "2", "plan_verdict": "withheld", "plan_reject": "segment:x"},
        {"idx": 3, "throw_id": "3", "plan_verdict": "no_plan", "plan_reject": "search:r"},
        {"idx": 4, "throw_id": "4", "plan_verdict": "no_search", "truth_success": "False"},
        {"idx": 5, "throw_id": "5", "plan_verdict": "published", "invalid_reason": "lane_drop"},
        {"idx": 6, "throw_id": "6", "plan_verdict": ""},  # no verdict (no clock lane)
        {"idx": 7, "throw_id": "", "plan_verdict": "no_plan", "plan_reject": "search:r"},
        {"idx": 8, "throw_id": "99", "plan_verdict": "published", "truth_success": "True"},
    ]
    offline = [
        _verdict(0, True),
        _verdict(1, False, "o"),
        _verdict(2, True),
        _verdict(3, True),
        _verdict(4, False, "o"),
        _verdict(50, True),
    ]
    joined, got = m.sim_table(sim, offline)
    assert (got["trials"], got["invalid"], got["unjudged"], got["without_throw_id"]) == (
        9,
        1,
        1,
        1,
    )
    table = got["table"]
    assert table[m.SIM_ACCEPTED_PUBLISHED] == {
        "truth_success": 2,
        "truth_fail": 1,
        "truth_unknown": 0,
    }
    assert table[m.SIM_ACCEPTED_NOT_PUBLISHED]["truth_unknown"] == 1
    assert table[m.SIM_REFUSED] == {"truth_success": 0, "truth_fail": 1, "truth_unknown": 2}
    assert got["refused_without_a_search"] == 1
    assert got["not_published_by_reason"] == {"segment:x": 1}
    assert got["refused_by_reason"] == {"search:r": 2, "no_search": 1}
    off = got["offline"]
    assert off["both_accepted"] == 2 and off["both_refused"] == 1
    assert off["offline_only"] == [3] and off["sim_only"] == [1]
    assert off["compared"] == 5 and off["agreement"] == pytest.approx(3 / 5)
    assert off["sim_throws_without_an_offline_verdict"] == [99]
    assert off["offline_throws_not_thrown"] == 1  # throw 50
    assert len(joined) == 7
    _, alone = m.sim_table(sim)
    assert "offline" not in alone and alone["table"] == table


# ── through the installed binary, on a shipped profile ────────────────────────


def _installed_search() -> Path | None:
    try:
        return cm.find_judge(None, m.SEARCH_EXECUTABLE)
    except SystemExit:
        return None


SHIPPED_AXES = {
    "distance_m": [3.5],
    "azimuth_deg": [0.0],
    "release_height_m": [1.6, 2.0],
    "aim_deviation_deg": [0.0],
    "speed_m_s": [6.5],
    "elevation_deg": [20.0, 30.0],
}


def _gate_map_dir(root: Path, axes: dict, open_ids: set[int]) -> Path:
    """A catch_gate_map output over the grid of ``axes``: its kinematic map's
    throw_summary.csv, a gate_map.csv with one candidate per throw (open for
    ``open_ids``) and the summary that points at the map."""
    throws = m.grid_throws(axes, origin_xy_m=(0.0, 0.0))
    map_dir = root / "map"
    map_dir.mkdir()
    with (map_dir / "throw_summary.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["throw_index", *m.THROW_AXES])
        for t in throws:
            writer.writerow([t["throw_id"], *[t[a] for a in m.THROW_AXES]])
    gate = root / "gate"
    gate.mkdir()
    with (gate / "gate_map.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["seed_id", "throw_index", "reason_box", "reason_torque"])
        for t in throws:
            reason = "none" if t["throw_id"] in open_ids else "reach_time"
            writer.writerow([0, t["throw_id"], reason, reason])
    (gate / "gate_map_summary.yaml").write_text(
        yaml.safe_dump({"map_dir": str(map_dir), "seed_id": 0, "wait_pose": [0.0] * 6})
    )
    return gate


def test_a_gate_map_output_reads_back_per_throw_and_layer(tmp_path):
    gate = _gate_map_dir(tmp_path, AXES, open_ids={0, 5})
    throws, meta = m.load_gate_throws(gate)
    assert [t["throw_index"] for t in throws] == list(range(24))
    assert {t["throw_index"] for t in throws if t["gate_box"] == "open"} == {0, 5}
    assert {t["gate_reason_torque"] for t in throws if t["gate_torque"] == "closed"} == {
        "reach_time"
    }
    assert meta["seed_id"] == 0 and meta["wait_pose"] == [0.0] * 6


@pytest.fixture(scope="module")
def shipped_run(tmp_path_factory):
    pytest.importorskip("pinocchio")
    config = _REPO / "integrated_bringup/config/ur5e_p1b"
    if not config.is_dir():
        pytest.skip("integrated_bringup config is not beside this checkout")
    if _installed_search() is None:
        pytest.skip(f"{m.SEARCH_EXECUTABLE} is not installed (build rtc_controllers)")
    out = tmp_path_factory.mktemp("shipped")
    gate_dir = _gate_map_dir(tmp_path_factory.mktemp("gate"), SHIPPED_AXES, open_ids={1, 3})
    sim_csv = out.parent / "catching_trials.csv"
    sim_csv.write_text(
        "idx,throw_id,plan_verdict,plan_reject,truth_success,invalid_reason\n"
        "0,0,published,,True,\n1,1,no_plan,search:reach_time,False,\n2,7,withheld,x,,\n"
    )
    argv = [
        "--config-dir", str(config), "--search", "grid", "--out-dir", str(out),
        "--gate-map", str(gate_dir), "--sim-trials", str(sim_csv),
        "--ball-config", str(config / "mujoco_simulator.yaml"),
        "--drag-coefficient", "0.55", "--drag-coefficient-source", "test",
        "--air-density", "1.204", "--air-density-source", "test",
        "--distances-m", "3.5", "--azimuths-deg", "0", "--release-heights-m", "1.6 2.0",
        "--aim-deviations-deg", "0", "--speeds-m-s", "6.5", "--elevations-deg", "20 30",
        "--grid-origin", "wait_catch_point",
        "--detection-delay-s", "0.15", "--vision-period-s", "0.05",
        "--prediction-horizon-s", "1.0", "--horizon-s", "2.0",
    ]  # fmt: skip
    assert m.main(argv) == 0
    return out


def test_the_installed_search_judges_every_throw_up_to_its_first_plan(shipped_run):
    out = shipped_run
    throws, _ = m.read_throw_list(out / "throw_list.json")
    assert len(throws) == 4
    with (out / "wakes.csv").open() as handle:
        given = {int(r["throw_id"]) for r in csv.DictReader(handle)}
    rows = m.parse_search_rows((out / "search_grid.csv").read_text())
    assert rows
    by_throw: dict[int, list[dict]] = {}
    for row in rows:
        by_throw.setdefault(row["throw_id"], []).append(row)
    # every throw was given wakes, and the search ran at least one of each
    assert given == {t["throw_id"] for t in throws}
    assert set(by_throw) == given
    for throw_id, own in by_throw.items():
        assert [r["wake"] for r in own] == list(range(len(own))), throw_id
        valid = [r["search_valid"] for r in own]
        # rows stop at the first plan: only the last row of a throw may hold one
        assert all(v == 0 for v in valid[:-1])
        if valid[-1] == 1:
            assert math.isfinite(own[-1]["lead_s"])
    summary = json.loads((out / "search_map_summary.json").read_text())
    grid = summary["searches"]["grid"]
    assert grid["throws"] == 4 and grid["accepted"] + grid["refused"] == 4
    assert yaml.safe_load((out / "binding_grid.yaml").read_text())["search"] == "grid"


def test_the_cli_joins_a_gate_map_and_a_sim_run_by_throw(shipped_run):
    summary = json.loads((shipped_run / "search_map_summary.json").read_text())
    join = summary["gate_map"]["searches"]["grid"]
    assert join["matched"] == 4 and join["search_throws_not_in_gate_map"] == 0
    box = join["layers"]["box"]["table"]
    assert box["open"]["accepted"] + box["open"]["refused"] == 2
    sim = summary["sim"]["searches"]["grid"]
    assert sim["table"]["accepted_published"]["truth_success"] == 1
    assert sim["table"]["refused"]["truth_fail"] == 1
    assert sim["offline"]["compared"] == 2
    assert sim["offline"]["sim_throws_without_an_offline_verdict"] == [7]
    assert (shipped_run / "gate_join_grid.csv").is_file()
    assert (shipped_run / "sim_join_grid.csv").is_file()


def test_the_shipped_tree_without_the_selected_search_map_is_refused():
    pytest.importorskip("pinocchio")
    config = _REPO / "integrated_bringup/config/ur5e_p1b"
    if not config.is_dir():
        pytest.skip("integrated_bringup config is not beside this checkout")
    from rtc_tools.analysis.catch_gate_map import composed_catching  # noqa: PLC0415

    tree, _ = composed_catching(
        config / "controllers/demo_catching_controller.yaml",
        "demo_catching_controller",
        [config / "_base.yaml", config / "sim.yaml"],
    )
    facts = _facts(
        arm=_limits(("a0", "a1", "a2", "a3", "a4", "a5"), [-3] * 6, [3] * 6, [3] * 6, [50] * 6),
        model_joint_names=("a0", "a1", "a2", "a3", "a4", "a5"),
        urdf_position_lower=np.full(6, -3.0),
        urdf_position_upper=np.full(6, 3.0),
    )
    assert m.search_binding(tree, facts, "nlp").document["search"] == "nlp"
    del tree["planner"]["search"]["nlp"]
    with pytest.raises(m.BindingError, match=r"planner\.search\.nlp\.core is not in this tree"):
        m.search_binding(tree, facts, "nlp")
    del tree["planner"]["search"]["grid"]
    with pytest.raises(m.BindingError, match=r"planner\.search\.grid"):
        m.search_binding(tree, facts, "grid")


def _fake_binary(tmp_path: Path, body: str) -> Path:
    path = tmp_path / "fake_search"
    path.write_text("#!/bin/sh\n" + body)
    path.chmod(0o755)
    return path


def _run_fake(tmp_path: Path, binary: Path) -> m.SearchRun:
    files = {k: tmp_path / f"{k}.txt" for k in ("model", "params", "binding", "wakes")}
    return m.run_search_batch(
        binary,
        "nlp",
        model_config=files["model"],
        sub_model="arm_catch",
        catch_frame="catch_frame",
        params=files["params"],
        binding=files["binding"],
        wakes=files["wakes"],
        out=tmp_path / "out.csv",
    )


def test_a_refused_configuration_is_an_answer_carrying_the_binarys_own_line(tmp_path):
    refusal = "catch_search_batch: the search refused its configuration — planner.search.nlp: x"
    binary = _fake_binary(
        tmp_path,
        f"echo '[WARN] [urdf.analyzer]: model chatter' >&2\necho '{refusal}' >&2\nexit 3\n",
    )
    run = _run_fake(tmp_path, binary)
    assert run.rows is None and run.refusal == refusal and run.kind == "nlp"


def test_any_other_failure_raises_with_the_binarys_lines(tmp_path):
    binary = _fake_binary(tmp_path, "echo 'catch_search_batch: binding: bad' >&2\nexit 1\n")
    with pytest.raises(RuntimeError, match="binding: bad"):
        _run_fake(tmp_path, binary)


def test_a_run_reads_the_rows_back_typed(tmp_path):
    header = "throw_id,wake,now_ns,search_valid,plan_reason,nlp_reason,t_c_ns,lead_s"
    binary = _fake_binary(
        tmp_path,
        f'shift 13\nprintf "{header}\\n4,0,1150000000,0,4,off,,\\n4,1,1200000000,1,0,none,'
        f'1900000000,0.7\\n" > "$1"\n',
    )
    run = _run_fake(tmp_path, binary)
    assert run.refusal is None
    assert run.rows[0]["nlp_reason"] == "off" and math.isnan(run.rows[0]["t_c_ns"])
    assert run.rows[1]["t_c_ns"] == 1_900_000_000 and run.rows[1]["lead_s"] == 0.7
