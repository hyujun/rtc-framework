"""catching_decel — synthetic positive controls (MPC · dual-arm plan E0-F02, #625).

Every number the tool reports is planted first: a joint that stops along a
smooth, closed-form deceleration (``q̈ = −A sin²(πt/T)``, so peak acceleration
``A``, peak jerk ``Aπ/T`` and stop distance ``A T²/4`` per unit of lever), a
lever-arm FK, planted limits and a planted torque fraction. The tool must read
each one back, and must move when the planted value moves.
"""

from __future__ import annotations

import csv
import json
import math
import shutil
import types
from pathlib import Path

import numpy as np
import pytest
import yaml

from rtc_tools.analysis import catching_decel as cd, catching_trials as ct

DT = 0.002
A_PEAK = 12.0  # rad/s², planted peak deceleration of joint j_a
T_STOP = 0.2  # s, planted stop time
W0 = 0.5 * A_PEAK * T_STOP  # entry speed that stops exactly at T_STOP
LEVER = 0.5  # m/rad, the planted FK: p = (LEVER · q_a, 0, 0)
Q_STOP = A_PEAK * T_STOP**2 / 4.0  # rad travelled while stopping
N_APPROACH = 100
N_HOLD = 250
N_RETREAT = 50
JOINTS = ("j_a", "j_b")
ARM = "armdev"
CONTROLLER = "demo_catching_controller"
A_DEC = 10.0
FIXTURE = Path(__file__).parent / "data" / "catching_pilot_260924_1218"


TILT = 0.4  # rad/rad, the planted FK tilts the approach axis with j_a


def _rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def lever_fk(q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """j_a moves the task (position and approach axis), j_b only rolls about the axis."""
    return np.array([LEVER * q[0], 0.0, 0.0]), _rot_x(TILT * q[0]) @ _rot_z(q[1])


def stop_profile(a_peak: float = A_PEAK, t_stop: float = T_STOP) -> tuple[np.ndarray, int, int]:
    """q of (j_a, j_b): cruise at the entry speed, stop along sin², rest. → (q, k0, k_stop)."""
    w0 = 0.5 * a_peak * t_stop
    n_stop = round(t_stop / DT)
    t = np.arange(N_APPROACH + n_stop + N_HOLD + N_RETREAT) * DT
    tau = np.clip(t - N_APPROACH * DT, 0.0, t_stop)
    # ∫∫ −A sin²(πs/T): position gained since entry, closed form.
    gained = w0 * tau - a_peak * (
        tau**2 / 4.0
        + (t_stop / (2.0 * math.pi)) ** 2 * (np.cos(2.0 * math.pi * tau / t_stop) - 1.0) / 2.0
    )
    q_a = np.where(t < N_APPROACH * DT, w0 * (t - N_APPROACH * DT), gained)
    return np.column_stack([q_a, np.zeros_like(q_a)]), N_APPROACH, N_APPROACH + n_stop


def metrics(q: np.ndarray, k0: int, k_limit: int, **kw) -> dict:
    args = {
        "position_lower": [-3.0, -3.0],
        "position_upper": [3.0, 3.0],
        "max_velocity": [2.0, 2.0],
        "entry_speed": LEVER * W0,
        "a_dec": A_DEC,
    }
    args.update(kw)
    return cd.window_metrics(q, q, DT, k0, k0 + 50, k_limit, lever_fk, **args)


# ── Window ───────────────────────────────────────────────────────────────────
def test_decel_entries_end_at_the_retreat_or_the_next_stretch():
    m = np.array(
        [ct.MODE_CLOSING] * 3
        + [ct.MODE_DECEL] * 4
        + [ct.MODE_HOLD] * 5
        + [ct.MODE_RETREAT] * 2
        + [ct.MODE_ARMED] * 3
        + [ct.MODE_DECEL] * 2
        + [ct.MODE_ABORT_SAFE] * 4
        + [ct.MODE_DECEL] * 1
        + [ct.MODE_HOLD] * 2
    )
    assert cd.decel_entries(m) == [(3, 7, 12), (17, 19, 23), (23, 24, 26)]


def test_rest_tick_is_the_first_tick_of_the_first_full_run():
    speed = np.array([1.0, 0.5, 0.01, 0.5, 0.01, 0.01, 0.01, 0.01, 0.5])
    assert cd.rest_tick(speed, 0, len(speed), 0.02, 3) == (4, True)
    # The same run cut short by the limit is not a rest.
    assert cd.rest_tick(speed, 0, 6, 0.02, 3) == (6, False)


def test_window_runs_to_the_hands_rest_not_to_the_end_of_the_mode():
    q, k0, k_stop = stop_profile()
    m = metrics(q, k0, len(q) - N_RETREAT)
    assert m["rest_reached"] and m["joint_rest_reached"]
    assert m["mode_decel_s"] == pytest.approx(50 * DT)
    # Rest is declared where the planted hand speed falls under the threshold —
    # before the planted stop, which sin² approaches with zero slope.
    tau = (np.arange(k0, k_stop + 1) - k0) * DT
    speed = LEVER * (
        W0 - A_PEAK * (tau / 2 - T_STOP * np.sin(2 * math.pi * tau / T_STOP) / (4 * math.pi))
    )
    expected = k0 + int(np.flatnonzero(speed < cd.REST_SPEED_M_S)[0])
    assert abs(m["k1"] - expected) <= 1
    assert expected < k_stop
    assert m["stop_time_s"] == pytest.approx((m["k1"] - k0) * DT)


def test_a_null_space_motion_does_not_hide_the_task_poses_stop():
    q, k0, k_stop = stop_profile()
    still = metrics(q, k0, len(q) - N_RETREAT)
    q = q.copy()
    q[:, 1] = 0.5 * np.arange(len(q)) * DT  # j_b rolls the hand about its axis at 0.5 rad/s
    m = metrics(q, k0, len(q) - N_RETREAT)
    assert m["rest_reached"] and not m["joint_rest_reached"]
    assert m["k1"] == still["k1"]
    assert m["axis_rate_peak_rad_s"] == pytest.approx(still["axis_rate_peak_rad_s"], rel=1e-6)
    # The roll is seen, and reported as what it is.
    assert m["angular_speed_at_rest_rad_s"] == pytest.approx(0.5, rel=0.02)


def test_an_approach_axis_that_keeps_turning_is_not_at_rest():
    def tilting(q):
        return np.array([LEVER * q[0], 0.0, 0.0]), _rot_x(q[1])

    q, k0, _ = stop_profile()
    q = q.copy()
    q[:, 1] = 0.5 * np.arange(len(q)) * DT  # the position stops, the axis tilts at 0.5 rad/s
    k_limit = len(q) - N_RETREAT
    m = cd.window_metrics(
        q,
        q,
        DT,
        k0,
        k0 + 50,
        k_limit,
        tilting,
        position_lower=[-3.0, -3.0],
        position_upper=[3.0, 3.0],
        max_velocity=[2.0, 2.0],
    )
    assert not m["rest_reached"] and m["k1"] == k_limit
    assert m["axis_rate_peak_rad_s"] == pytest.approx(0.5, rel=0.02)


def test_the_axis_is_what_ends_the_window_when_it_stops_last():
    # A steep tilt gain: the approach axis is still above its threshold when
    # the position already rests.
    def steep(q):
        return np.array([LEVER * q[0], 0.0, 0.0]), _rot_x(20.0 * LEVER * q[0])

    q, k0, k_stop = stop_profile()
    args = {
        "position_lower": [-3.0, -3.0],
        "position_upper": [3.0, 3.0],
        "max_velocity": [2.0, 2.0],
    }
    by_position = metrics(q, k0, len(q) - N_RETREAT)
    m = cd.window_metrics(q, q, DT, k0, k0 + 50, len(q) - N_RETREAT, steep, **args)
    assert m["rest_reached"]
    assert by_position["k1"] < m["k1"] <= k_stop


def test_angular_speed_reads_a_planted_rotation_rate():
    rotations = np.array([_rot_z(0.3 * k * DT) @ _rot_x(0.1) for k in range(50)])
    assert cd.angular_speed(rotations, DT) == pytest.approx(np.full(50, 0.3), rel=1e-6)


def test_an_arm_that_never_rests_ends_at_the_limit_and_says_so():
    q, k0, _ = stop_profile()
    q = q.copy()
    q[:, 0] += 0.2 * np.arange(len(q)) * DT  # the hand keeps drifting at 0.1 m/s
    k_limit = len(q) - N_RETREAT
    m = metrics(q, k0, k_limit)
    assert not m["rest_reached"]
    assert m["k1"] == k_limit


# ── Planted values come back ─────────────────────────────────────────────────
def test_planted_peaks_distance_and_margins_are_recovered():
    q, k0, _ = stop_profile()
    m = metrics(
        q,
        k0,
        len(q) - N_RETREAT,
        position_lower=[-1.0, -3.0],
        position_upper=[Q_STOP + 0.25, 3.0],
    )
    assert m["qdd_cmd_peak"] == pytest.approx(A_PEAK, rel=0.02)
    assert m["qdd_meas_peak"] == pytest.approx(A_PEAK, rel=0.02)
    assert m["jerk_cmd_peak"] == pytest.approx(A_PEAK * math.pi / T_STOP, rel=0.03)
    assert m["stop_distance_mm"] == pytest.approx(1e3 * LEVER * Q_STOP, rel=0.01)
    assert m["stop_path_mm"] == pytest.approx(m["stop_distance_mm"], rel=1e-6)
    assert m["stop_distance_closed_form_mm"] == pytest.approx(
        1e3 * (LEVER * W0) ** 2 / (2 * A_DEC)
    )
    assert m["hand_speed_entry_m_s"] == pytest.approx(LEVER * W0, rel=0.02)
    assert m["velocity_ratio"] == pytest.approx(W0 / 2.0, rel=0.02)
    assert m["position_margin_rad"] == pytest.approx(0.25, abs=2e-3)
    assert not (m["violation_position"] or m["violation_velocity"] or m["violation_torque"])


def _with_bump(q: np.ndarray, tick: int, height: float) -> np.ndarray:
    """``q`` with a one-tick acceleration bump of ``height`` rad/s² on joint j_a at ``tick``."""
    bump = np.zeros(len(q))
    bump[tick] = height
    out = q.copy()
    out[:, 0] += np.cumsum(np.cumsum(bump)) * DT**2
    return out


def test_a_one_tick_bump_is_averaged_over_the_smoothing_window():
    q, k0, k_stop = stop_profile()
    height = -50.0
    k_mid = (k0 + k_stop) // 2  # where the planted deceleration peaks
    m = metrics(_with_bump(q, k_mid, height), k0, len(q) - N_RETREAT)
    # A one-tick bump of H reads H / smooth_ticks; unsmoothed central
    # differences would read H / 2.
    assert m["qdd_cmd_peak"] == pytest.approx(A_PEAK + abs(height) / cd.SMOOTH_TICKS, rel=0.05)


def test_what_happens_before_the_entry_is_not_the_stops_peak():
    q, k0, _ = stop_profile()
    m = metrics(_with_bump(q, k0 - 12, -400.0), k0, len(q) - N_RETREAT)
    assert m["qdd_cmd_peak"] == pytest.approx(A_PEAK, rel=0.02)


def test_the_position_margin_reads_the_nearer_limit():
    q, k0, _ = stop_profile()
    lower = metrics(q, k0, len(q) - N_RETREAT, position_lower=[-0.1, -3.0])
    assert lower["position_margin_rad"] == pytest.approx(0.1, abs=2e-3)  # at entry, q_a = 0
    other = metrics(q, k0, len(q) - N_RETREAT, position_upper=[3.0, 0.07])
    assert other["position_margin_rad"] == pytest.approx(0.07, abs=1e-9)  # j_b rests at 0


@pytest.mark.parametrize("scale", [0.5, 2.0])
def test_the_metrics_move_with_the_planted_deceleration(scale):
    base = metrics(*_window(stop_profile()))
    moved = metrics(*_window(stop_profile(a_peak=scale * A_PEAK)))
    for key in ("qdd_cmd_peak", "jerk_cmd_peak", "jerk_meas_peak", "stop_distance_mm"):
        assert moved[key] == pytest.approx(scale * base[key], rel=0.03), key


def _window(profile):
    q, k0, _ = profile
    return q, k0, len(q) - N_RETREAT


def test_a_step_in_acceleration_makes_the_unsmoothed_jerk_a_function_of_the_tick():
    # v1's closed form: constant deceleration switched on at entry and off at
    # the stop. The smoothed jerk is bounded by a / (smooth · dt); the raw one
    # is not the same number, which is why it is reported as a reference only.
    n_stop = round(T_STOP / DT)
    t = np.arange(N_APPROACH + n_stop + N_HOLD) * DT
    tau = np.clip(t - N_APPROACH * DT, 0.0, T_STOP)
    a = 10.0
    w0 = a * T_STOP
    q_a = np.where(t < N_APPROACH * DT, w0 * (t - N_APPROACH * DT), w0 * tau - 0.5 * a * tau**2)
    q = np.column_stack([q_a, np.zeros_like(q_a)])
    m = metrics(q, N_APPROACH, len(q))
    assert m["qdd_cmd_peak"] == pytest.approx(a, rel=0.02)
    assert m["jerk_cmd_raw_peak"] > 1.5 * m["jerk_cmd_peak"]
    assert m["jerk_cmd_peak"] <= a / (cd.SMOOTH_TICKS * DT) * 1.05


def test_violations_are_counted_from_the_planted_limits():
    q, k0, _ = stop_profile()
    ratio = np.full_like(q, 0.4)
    ratio[k0 + 10, 1] = 1.2
    m = metrics(
        q,
        k0,
        len(q) - N_RETREAT,
        position_upper=[0.5 * Q_STOP, 3.0],
        max_velocity=[0.5 * W0, 2.0],
        torque_ratio=ratio,
    )
    assert m["violation_position"] and m["position_margin_rad"] < 0.0
    assert m["violation_velocity"] and m["velocity_ratio"] == pytest.approx(2.0, rel=0.02)
    assert m["violation_torque"] and m["torque_ratio"] == pytest.approx(1.2)


def test_a_torque_row_outside_the_window_is_not_the_windows_peak():
    q, k0, _ = stop_profile()
    ratio = np.full_like(q, 0.4)
    ratio[k0 - 5, 0] = 3.0  # before entry
    ratio[len(q) - 10, 0] = 3.0  # in RETREAT
    m = metrics(q, k0, len(q) - N_RETREAT, torque_ratio=ratio)
    assert m["torque_ratio"] == pytest.approx(0.4)


# ── Pairs and sample size ────────────────────────────────────────────────────
def test_pair_table_counts_the_planted_cells():
    a = {("s35b", 601, i): i < 60 for i in range(100)}  # 60 successes
    b = {("s35b", 601, i): 10 <= i < 65 for i in range(100)}  # 55, 10 lost and 5 gained
    b[("s35b", 602, 0)] = True  # a throw only b has
    t = cd.pair_table(a, b)
    assert (t["n_pairs"], t["unpaired_a"], t["unpaired_b"]) == (100, 0, 1)
    assert (t["both"], t["only_a"], t["only_b"], t["neither"]) == (50, 10, 5, 35)
    assert t["discordance"] == pytest.approx(0.15)
    assert t["discordance_ci95"] == pytest.approx(list(ct.wilson_interval(15, 100)))
    assert t["mcnemar_p"] == pytest.approx(ct.mcnemar_exact(10, 5))


def test_identical_outcomes_have_no_discordance():
    a = {("s35b", 1, i): i % 2 == 0 for i in range(20)}
    t = cd.pair_table(a, dict(a))
    assert t["discordance"] == 0.0 and t["mcnemar_p"] == 1.0


@pytest.mark.parametrize(
    ("psi", "margin", "n"),
    [(0.18, 0.10, 142), (0.18, 0.05, 566), (0.49, 0.10, 385), (0.49, 0.05, 1539)],
)
def test_sample_size_matches_the_hand_calculation(psi, margin, n):
    # (z_0.975 + z_0.8)² = (1.959964 + 0.841621)² = 7.848879
    assert cd.paired_noninferiority_n(psi, margin) == n
    assert n == math.ceil(7.848879 * psi / margin**2)


def test_sample_size_shrinks_when_b_is_truly_better_and_refuses_the_impossible():
    assert cd.paired_noninferiority_n(0.2, 0.1, diff=0.05) < cd.paired_noninferiority_n(0.2, 0.1)
    with pytest.raises(ValueError, match="no finite sample size"):
        cd.paired_noninferiority_n(0.2, 0.1, diff=-0.1)


# ── One unit, end to end ─────────────────────────────────────────────────────
def _write_yaml(path: Path, doc: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(doc, sort_keys=False))


def _write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def make_config(share: Path, *, sim_a_dec: float | None = None) -> Path:
    cfg = share / "config" / "robot"
    _write_yaml(
        cfg / "_base.yaml",
        {
            "/**": {
                "ros__parameters": {
                    "urdf": {
                        "extra_frames": {
                            "catch_frame": {"parent": "palm", "xyz": [0, 0, 0], "rpy": [0, 0, 0]}
                        }
                    },
                    "devices": {
                        ARM: {
                            "joint_limits": {
                                "max_torque": [50.0, 20.0],
                                "max_velocity": [2.0, 2.0],
                                "position_lower": [-3.0, -3.0],
                                "position_upper": [3.0, 3.0],
                            }
                        }
                    },
                }
            }
        },
    )
    if sim_a_dec is not None:
        _write_yaml(
            cfg / "sim.yaml",
            {
                "rt": {
                    "ros__parameters": {
                        CONTROLLER: {"catching": {"supervisor": {"decel": {"a_dec": sim_a_dec}}}}
                    }
                }
            },
        )
    _write_yaml(
        cfg / "controllers" / f"{CONTROLLER}.yaml",
        {
            CONTROLLER: {
                "catching": {
                    "io": {
                        "arm_base_frame": "base",
                        "base_T_world": {"yaw_deg": 0.0, "translation": [0, 0, 0]},
                    },
                    "supervisor": {"decel": {"a_dec": A_DEC}},
                },
                "topics": {ARM: {"subscribe": []}},
                "logs": [
                    {"msg_type": "rtc_msgs/DeviceStateLog", "instance": f"{ARM}_state"},
                    {
                        "msg_type": "integrated_bringup/CatchingDiagLog",
                        "instance": "catching_diag",
                    },
                ],
            }
        },
    )
    return cfg


def make_unit(
    root: Path, name: str, outcomes: list[bool], *, seed: int = 601, second_stretch: bool = False
) -> Path:
    """A unit of ``len(outcomes)`` trials; trial 1 aborts before DECEL when there are ≥ 3.

    ``second_stretch`` puts a short second DECEL stretch into every trial's HOLD.
    """
    unit = root / name
    ctl = unit / "session" / "controllers" / CONTROLLER
    q, k0, _ = stop_profile()
    n = len(q)
    mode = np.full(n, ct.MODE_APPROACH)
    mode[k0 : k0 + 50] = ct.MODE_DECEL
    mode[k0 + 50 : n - N_RETREAT] = ct.MODE_HOLD
    mode[n - N_RETREAT :] = ct.MODE_RETREAT
    if second_stretch:
        mode[k0 + 150 : k0 + 160] = ct.MODE_DECEL
    rows, effort, launches = [], [], []
    for i in range(len(outcomes)):
        offset = i * n
        aborts = len(outcomes) >= 3 and i == 1
        launches.append((offset + 5) * DT)
        for k in range(n):
            t = round((offset + k) * DT, 6)
            m = ct.MODE_ABORT_SAFE if aborts and k >= k0 else mode[k]
            rows.append(
                {
                    "t_relative_s": t,
                    "mode": int(m),
                    "ref_xd_x": LEVER * W0,
                    "ref_xd_y": 0.0,
                    "ref_xd_z": 0.0,
                    "q_cmd_j_a": q[k, 0],
                    "q_cmd_j_b": 0.0,
                    "q_meas_j_a": q[k, 0],
                    "q_meas_j_b": 0.0,
                }
            )
            # 0.5 of the rating inside the stop, 0.1 elsewhere.
            inside = k0 <= k < k0 + round(T_STOP / DT)
            effort.append(
                {"t_relative_s": t, "effort_j_a": 25.0 if inside else 5.0, "effort_j_b": 2.0}
            )
    _write_csv(ctl / "catching_diag.csv", rows)
    _write_csv(ctl / f"{ARM}_state.csv", effort)
    _write_csv(
        unit / "ct" / "catching_trials.csv",
        [
            {
                "idx": i,
                "kind": "s35b",
                "t_launch": launches[i],
                "invalid_reason": "",
                "truth_success": str(ok),
                "supervisor": "CAPTURED" if ok else "MISSED",
            }
            for i, ok in enumerate(outcomes)
        ],
    )
    (unit / "trials").mkdir(parents=True)
    (unit / "trials" / "trial_results.json").write_text(
        json.dumps(
            [
                {"idx": i, "kind": "s35b", "seed": seed, "sample_idx": i, "accepted": False}
                for i in range(len(outcomes))
            ]
        )
    )
    (unit / "trials" / "run_meta.json").write_text(
        json.dumps({"arm": "synthetic", "controller_mirror": {"control.dt": DT}})
    )
    return unit


@pytest.fixture
def lever(monkeypatch):
    """The planted FK in place of the URDF one (the pilot test below runs the real one)."""
    from rtc_tools.analysis import derive_accel_limits

    monkeypatch.setattr(derive_accel_limits, "resolve_urdf_text", lambda *_: ("<robot/>", "x"))
    monkeypatch.setattr(ct, "CatchFrameFk", lambda *_: types.SimpleNamespace(pose_world=lever_fk))


def test_unit_reads_its_files_and_skips_the_trial_that_never_decelerated(tmp_path, lever):
    cfg = make_config(tmp_path / "share")
    unit = make_unit(tmp_path, "u", [True, False, False])
    out = cd.analyse_unit(*cd.parse_unit_arg(str(unit)), cfg)
    s = out["summary"]
    assert [r["idx"] for r in out["trials"]] == [0, 2]
    assert (s["n_trials"], s["n_valid"], s["truth_success"]) == (3, 3, 1)
    assert s["decel"]["n"] == 2 and s["decel_success"]["n"] == 1
    assert s["a_dec"] == A_DEC and s["sources"]["a_dec"] == "profile"
    d = s["decel"]
    assert d["qdd_meas_peak"]["p50"] == pytest.approx(A_PEAK, rel=0.02)
    assert d["stop_distance_mm"]["max"] == pytest.approx(1e3 * LEVER * Q_STOP, rel=0.01)
    assert d["torque_ratio"]["max"] == pytest.approx(0.5)
    assert d["violations"] == {"position": 0, "velocity": 0, "torque": 0}


def test_a_trial_has_one_stop_even_when_it_enters_decel_twice(tmp_path, lever):
    cfg = make_config(tmp_path / "share")
    unit = make_unit(tmp_path, "u", [True, False], second_stretch=True)
    out = cd.analyse_unit(*cd.parse_unit_arg(str(unit)), cfg)
    assert [r["idx"] for r in out["trials"]] == [0, 1]
    assert out["summary"]["unclaimed_decel_stretches"] == 2
    n = len(stop_profile()[0])
    assert [r["k0"] for r in out["trials"]] == [N_APPROACH, n + N_APPROACH]


def test_a_dec_is_read_through_the_layers_the_launch_composes(tmp_path, lever):
    cfg = make_config(tmp_path / "share", sim_a_dec=4.0)
    unit = make_unit(tmp_path, "u", [True])
    s = cd.analyse_unit(*cd.parse_unit_arg(str(unit)), cfg)["summary"]
    assert s["a_dec"] == 4.0 and s["sources"]["a_dec"] == "sim.yaml"
    assert s["decel"]["stop_distance_closed_form_mm"]["p50"] == pytest.approx(
        1e3 * (LEVER * W0) ** 2 / 8.0
    )


def test_cli_pairs_two_sets_and_writes_the_sample_size(tmp_path, lever, capsys):
    cfg = make_config(tmp_path / "share")
    a = make_unit(tmp_path, "a", [True, True, False, False])
    b = make_unit(tmp_path, "b", [True, False, True, True])
    rc = cd.main(
        ["--a", str(a), "--b", str(b), "--config-dir", str(cfg), "--out", str(tmp_path / "out")]
    )
    assert rc == 0
    doc = json.loads((tmp_path / "out" / "decel_summary.json").read_text())
    pairs = doc["pairs"]
    # Trial 1 aborts in both units but stays a valid, failed throw: 4 pairs.
    assert (pairs["n_pairs"], pairs["only_a"], pairs["only_b"]) == (4, 1, 2)
    assert pairs["discordance"] == pytest.approx(0.75)
    assert [r["margin"] for r in pairs["sample_size"]] == [0.05, 0.10]
    assert doc["pooled"]["n_valid"] == 8 and doc["pooled"]["truth_success"] == 5
    with (tmp_path / "out" / "decel_trials.csv").open() as f:
        assert len(list(csv.DictReader(f))) == 6
    assert "ψ 0.750" in capsys.readouterr().out


def test_the_same_throw_twice_in_one_set_is_refused(tmp_path, lever):
    cfg = make_config(tmp_path / "share")
    units = [
        cd.analyse_unit(*cd.parse_unit_arg(str(make_unit(tmp_path, n, [True]))), cfg)
        for n in ("u1", "u2")
    ]
    with pytest.raises(SystemExit, match="appears twice"):
        cd.outcome_map(units)


def _pilot_diag(profile) -> Path:
    path = FIXTURE / "session" / "controllers" / profile.controller / "catching_diag.csv"
    return ct._exists(path) or path


def test_the_pilot_session_runs_through_the_real_fk(tmp_path):
    pytest.importorskip("pinocchio")
    ct_dir = tmp_path / "ct"
    assert (
        ct.main(
            [
                str(FIXTURE / "session"),
                str(FIXTURE / "trials"),
                "--config-dir",
                str(FIXTURE / "config"),
                "--urdf",
                str(FIXTURE / "robot.urdf"),
                "--out",
                str(ct_dir),
                "--n-boot",
                "20",
            ]
        )
        == 0
    )
    # The fixture's trimmed profile carries no ratings; lay wide ones over a copy.
    cfg = tmp_path / "config"
    shutil.copytree(FIXTURE / "config", cfg)
    profile = ct.load_profile(cfg, session=FIXTURE / "session")
    n = len(ct.arm_joints_from_diag(ct._csv_header(_pilot_diag(profile))))
    _write_yaml(
        cfg / "sim.yaml",
        {
            "/**": {
                "ros__parameters": {
                    "devices": {
                        profile.arm_device: {
                            "joint_limits": {
                                "max_torque": [150.0] * n,
                                "max_velocity": [3.0] * n,
                                "position_lower": [-6.3] * n,
                                "position_upper": [6.3] * n,
                            }
                        }
                    }
                }
            }
        },
    )
    out = cd.analyse_unit(
        FIXTURE, FIXTURE / "session", cfg, urdf=FIXTURE / "robot.urdf", ct_dir=ct_dir
    )
    rows = out["trials"]
    assert len(rows) == out["summary"]["decel"]["n"] > 0
    assert len({r["idx"] for r in rows}) == len(rows)
    for r in rows:
        assert math.isnan(r["stop_distance_closed_form_mm"])  # the fixture diag has no ref_xd_*
        assert 0.0 < r["stop_distance_mm"] < 1000.0
        assert 0.0 < r["qdd_meas_peak"] < 200.0
        assert r["k1"] > r["k0"]


def test_arch1_module_has_no_robot_constants():
    text = Path(cd.__file__).read_text().lower()
    for word in ("ur5e", "iiwa", "leap", "p1b", "panda"):
        assert word not in text, word
