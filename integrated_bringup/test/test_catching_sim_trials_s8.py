"""S8-A additions to the sim-trial driver: the frozen throw distribution and the mirror.

The frozen series is the success-rate input (D-S8-2), so what is pinned here is
what makes a rate reproducible and meaningful: the same ``(dist, profile, n,
seed)`` replays the same launches bit for bit, every throw is inside the box
the gate map opened, and the release state is the geometry that box was judged
on — checked against the box's own axes (speed, elevation, distance, height)
rather than against the helper that built it, which would only compare the
helper with itself.

The mirror is the other half: a runner that aligned to the installed YAML would
refuse every trial of an overlay run.
"""

import json
import math
import os

import pytest

from integrated_bringup.catching_sim_trials import (
    FROZEN_DISTRIBUTIONS,
    MIRROR_PARAMETERS,
    apply_mirror,
    build_throws,
    frozen_throws,
    load_arm_profile,
    mirror_from_replies,
    parse_args,
    trial_throws,
)

CONFIG = os.path.join(os.path.dirname(__file__), "..", "config")
BOX = FROZEN_DISTRIBUTIONS["s35b"]["ur5e_p1b"]


def test_the_same_seed_replays_the_same_launches_and_another_seed_does_not():
    a = frozen_throws("s35b", "ur5e_p1b", 20, 7)
    b = frozen_throws("s35b", "ur5e_p1b", 20, 7)
    assert json.dumps(a) == json.dumps(b)
    assert [t["pos"] for t in frozen_throws("s35b", "ur5e_p1b", 20, 8)] != [t["pos"] for t in a]
    # A prefix of a longer series is the shorter series: n does not reseed.
    assert frozen_throws("s35b", "ur5e_p1b", 5, 7) == a[:5]
    assert all(t["seed"] == 7 for t in a)
    assert [t["sample_idx"] for t in a] == list(range(20))


def test_every_throw_is_inside_the_box_and_has_the_geometry_the_map_judged():
    throws = frozen_throws("s35b", "ur5e_p1b", 200, 3)
    for t in throws:
        for axis in (
            "distance_m",
            "release_height_m",
            "aim_deviation_deg",
            "speed_m_s",
            "elevation_deg",
        ):
            lo, hi = getattr(BOX, axis)
            assert lo <= t[axis] <= hi, axis
        px, py, pz = t["pos"]
        vx, vy, vz = t["vel"]
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        assert speed == pytest.approx(t["speed_m_s"], abs=1e-9)
        assert math.degrees(math.asin(vz / speed)) == pytest.approx(t["elevation_deg"], abs=1e-9)
        assert math.hypot(px - BOX.base_xy_m[0], py - BOX.base_xy_m[1]) == pytest.approx(
            t["distance_m"], abs=1e-9
        )
        assert pz == pytest.approx(t["release_height_m"], abs=1e-12)
        # Thrown back towards the base axis: the horizontal velocity, rotated
        # by minus the aim deviation, points from the release point at the axis.
        heading = math.degrees(math.atan2(vy, vx) - math.atan2(-py, -px))
        heading = (heading + 180.0) % 360.0 - 180.0
        assert heading == pytest.approx(t["aim_deviation_deg"], abs=1e-9)
        assert t["omega"] == (0.0, 0.0, 0.0)
    # The draws cover the box rather than sitting in one corner of it.
    speeds = [t["speed_m_s"] for t in throws]
    assert max(speeds) - min(speeds) > 0.8 * (BOX.speed_m_s[1] - BOX.speed_m_s[0])


def test_the_reference_throw_is_the_centre_of_the_box_it_was_frozen_around():
    # Positive control for the box itself: the S3.5b reference throw (1.0 m,
    # 0.2 m, 4.75 m/s, 60°) is what the box was mapped around. Its distance,
    # height and speed sit inside; its elevation (60°) is below the box
    # (62–64°) — the map opened steeper throws than the reference.
    ref = trial_throws(1, 0, 0, (1.0, 0.0, 0.2), (-2.375, 0.0, 4.11362))[0]
    vx, _, vz = ref["vel"]
    assert BOX.distance_m[0] <= math.hypot(*ref["pos"][:2]) <= BOX.distance_m[1]
    assert BOX.release_height_m[0] <= ref["pos"][2] <= BOX.release_height_m[1]
    assert BOX.speed_m_s[0] <= math.hypot(vx, vz) <= BOX.speed_m_s[1]


def test_an_unknown_distribution_or_a_profile_without_a_box_is_refused():
    with pytest.raises(ValueError, match="unknown distribution"):
        frozen_throws("uniform", "ur5e_p1b", 3, 0)
    # ur5e_p1a runs no catching trials, so it has no box. (This was iiwa7_leap
    # until S8-D froze that robot's box, #537.)
    with pytest.raises(ValueError, match="no box for profile 'ur5e_p1a'"):
        frozen_throws("s35b", "ur5e_p1a", 3, 0)


def test_the_leap_box_replays_and_stays_inside_its_own_axes():
    """S8-D froze iiwa7_leap's own box (D-S8-14/15): same contract as ur5e_p1b's."""
    box = FROZEN_DISTRIBUTIONS["s35b"]["iiwa7_leap"]
    assert box != BOX, "the two robots' boxes are separate map verdicts"
    a = frozen_throws("s35b", "iiwa7_leap", 50, 505)
    assert json.dumps(a) == json.dumps(frozen_throws("s35b", "iiwa7_leap", 50, 505))
    assert a != frozen_throws("s35b", "ur5e_p1b", 50, 505)
    for t in a:
        for axis in (
            "distance_m",
            "release_height_m",
            "aim_deviation_deg",
            "speed_m_s",
            "elevation_deg",
        ):
            lo, hi = getattr(box, axis)
            assert lo <= t[axis] <= hi, axis
        vx, vy, vz = t["vel"]
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        assert speed == pytest.approx(t["speed_m_s"], abs=1e-9)
        assert math.degrees(math.asin(vz / speed)) == pytest.approx(t["elevation_deg"], abs=1e-9)
        assert t["pos"][2] == pytest.approx(t["release_height_m"], abs=1e-12)


def test_the_default_arguments_still_build_the_reference_series():
    args = parse_args(["out"])
    assert args.dist == "reference"
    assert build_throws(args, "ur5e_p1b") == trial_throws(
        15, 10, 42, args.release_pos, args.release_vel
    )
    s35b = parse_args(["out", "--dist", "s35b", "--n", "4", "--seed", "11"])
    assert build_throws(s35b, "ur5e_p1b") == frozen_throws("s35b", "ur5e_p1b", 4, 11)


def _mirror(wait_pose):
    return {
        "planner.wait_pose": wait_pose,
        "planner.freeze.T_freeze": 0.52,
        "joint_cmd.lag.T_arm": 0.2,
        "joint_cmd.lag.lead_enable": True,
        "control.dt": 0.002,
        "reference.omega": 10.0,
        "reference.a_max": 21.0,
        "reference.v_max": 3.5,
        "planner.gamma.eta_v": 0.9,
        "planner.time.margin": 0.03,
        "robot.arm.qdd_max": [2.03] * 6,
    }


def test_the_mirror_replaces_the_file_wait_pose():
    arm = load_arm_profile(os.path.join(CONFIG, "ur5e_p1b"))
    moved = [v + 0.1 for v in arm.wait_pose]
    aligned = apply_mirror(arm, _mirror(moved))
    assert aligned.wait_pose == tuple(moved)
    assert aligned.joint_names == arm.joint_names
    assert set(_mirror(moved)) == set(MIRROR_PARAMETERS)


def test_a_missing_mirror_value_or_a_wrong_length_pose_is_refused():
    arm = load_arm_profile(os.path.join(CONFIG, "ur5e_p1b"))
    parked = _mirror(list(arm.wait_pose))
    parked["planner.freeze.T_freeze"] = None  # not declared: parked at configure
    with pytest.raises(ValueError, match="planner.freeze.T_freeze"):
        apply_mirror(arm, parked)
    with pytest.raises(ValueError, match="has 7 values"):
        apply_mirror(arm, _mirror([0.0] * 7))


def test_a_parked_controller_is_named_even_though_the_batch_reply_is_empty():
    # rclcpp's get_parameters answers [] when ANY name is undeclared. The
    # runner must still say which one, so it re-asks name by name.
    declared = {"control.dt": (3, 0.002), "planner.wait_pose": (8, [0.0] * 6)}
    calls = []

    def ask(names):
        calls.append(list(names))
        if any(n not in declared for n in names):
            return []
        return [declared[n] for n in names]

    mirror = mirror_from_replies(MIRROR_PARAMETERS, ask)
    assert calls[0] == list(MIRROR_PARAMETERS) and len(calls) == 1 + len(MIRROR_PARAMETERS)
    assert mirror["control.dt"] == 0.002 and mirror["planner.wait_pose"] == [0.0] * 6
    assert mirror["planner.freeze.T_freeze"] is None
    arm = load_arm_profile(os.path.join(CONFIG, "ur5e_p1b"))
    with pytest.raises(ValueError, match="parked at configure"):
        apply_mirror(arm, mirror)


def test_a_full_reply_is_read_in_one_call_and_not_set_reads_as_absent():
    calls = []

    def ask(names):
        calls.append(names)
        return [(0, None) if n == "joint_cmd.lag.T_arm" else (3, 1.0) for n in names]

    mirror = mirror_from_replies(MIRROR_PARAMETERS, ask)
    assert len(calls) == 1
    assert mirror["joint_cmd.lag.T_arm"] is None
    assert mirror["control.dt"] == 1.0
