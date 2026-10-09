"""S8-A additions to the sim-trial driver: the throws a run names, and the mirror.

The frozen series of before #798 (``s35b`` · ``reference``) are gone: a run
names its throws (``--throws-file`` or a ``--dist`` hand design) or is refused
before the sim is touched — the driver has no throws of its own.

The mirror is the other half: a runner that aligned to the installed YAML would
refuse every trial of an overlay run.
"""

import os

import pytest

from integrated_bringup.catching_sim_trials import (
    MIRROR_PARAMETERS,
    _parameter_value,
    apply_mirror,
    build_throws,
    load_arm_profile,
    mirror_from_replies,
    mirror_names,
    parse_args,
)

CONFIG = os.path.join(os.path.dirname(__file__), "..", "config")


def test_a_run_that_names_no_throws_is_refused(capsys):
    """#798: no series is built in — --throws-file or a --dist hand design, else an
    argument error before the sim is touched (``parse_args`` exits 2)."""
    with pytest.raises(SystemExit) as exc:
        parse_args(["out"])
    assert exc.value.code == 2
    assert "--throws-file" in capsys.readouterr().err
    for gone in ("s35b", "reference"):
        with pytest.raises(SystemExit):
            parse_args(["out", "--dist", gone])
    args = parse_args(["out", "--dist", "hand_lhs"])
    args.dist = None  # a caller that bypassed parse_args
    with pytest.raises(ValueError, match="no throws named"):
        build_throws(args, "ur5e_p1b")


def _mirror(wait_pose):
    # grid x closed_form: the one selection that declares every name.
    return {
        "planner.search.mode": "grid",
        "planner.segment.mode": "closed_form",
        "planner.wait_pose": wait_pose,
        "planner.freeze.T_freeze": 0.52,
        "joint_cmd.lag.T_arm": 0.2,
        "joint_cmd.lag.lead_enable": True,
        "control.dt": 0.002,
        "reference.omega": 10.0,
        "reference.a_max": 21.0,
        "reference.v_max": 3.5,
        "planner.search.grid.gamma.eta_v": 0.9,
        "planner.search.grid.time.margin": 0.03,
        "robot.arm.qdd_max": [2.03] * 6,
        "prediction.dt_expected": 0.05,
        "io.n_min": 12,
        "planner.search.grid.slice.dt": 0.05,
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


@pytest.mark.parametrize(
    ("search", "segment", "absent"),
    [
        ("grid", "mpc", ("reference.omega", "reference.a_max", "reference.v_max")),
        ("grid", "mpc_docking", ("reference.omega", "reference.a_max", "reference.v_max")),
        (
            "nlp",
            "mpc_docking",
            (
                "reference.omega",
                "reference.a_max",
                "reference.v_max",
                "planner.search.grid.gamma.eta_v",
                "planner.search.grid.time.margin",
                "planner.search.grid.slice.dt",
            ),
        ),
    ],
)
def test_a_function_that_does_not_run_owes_no_mirror(search, segment, absent):
    # E1-F16: the controller mirrors only the functions it runs. Their names
    # being absent is the selection, not a parked configure.
    arm = load_arm_profile(os.path.join(CONFIG, "ur5e_p1b"))
    mirror = _mirror(list(arm.wait_pose))
    mirror["planner.search.mode"] = search
    mirror["planner.segment.mode"] = segment
    for name in absent:
        del mirror[name]
    assert set(mirror) == set(mirror_names(search, segment))
    assert apply_mirror(arm, mirror).wait_pose == arm.wait_pose
    # ... and what the selection DOES run is still owed.
    owed = [n for n in mirror_names(search, segment) if n.startswith("planner.search.grid.")]
    for name in owed:
        short = dict(mirror)
        short[name] = None
        with pytest.raises(ValueError, match=name):
            apply_mirror(arm, short)


def test_a_selector_that_was_not_read_owes_every_name_and_is_named_itself():
    arm = load_arm_profile(os.path.join(CONFIG, "ur5e_p1b"))
    mirror = _mirror(list(arm.wait_pose))
    mirror["planner.search.mode"] = None
    assert mirror_names(None, "mpc") == MIRROR_PARAMETERS
    assert mirror_names("grid", "spline") == MIRROR_PARAMETERS
    with pytest.raises(ValueError, match="planner.search.mode"):
        apply_mirror(arm, mirror)


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


def test_every_mirrored_parameter_type_decodes_to_a_value():
    """A mirror type the decoder does not know reads as ``None`` — the same answer
    as an undeclared parameter — so the runner would refuse a running controller
    as parked. ``io.n_min`` is the first integer in the mirror (E0-F04)."""
    rcl = pytest.importorskip("rcl_interfaces.msg")
    t = rcl.ParameterType
    cases = [
        (rcl.ParameterValue(type=t.PARAMETER_DOUBLE, double_value=0.025), 0.025),
        (rcl.ParameterValue(type=t.PARAMETER_INTEGER, integer_value=22), 22),
        (rcl.ParameterValue(type=t.PARAMETER_BOOL, bool_value=True), True),
        (
            rcl.ParameterValue(type=t.PARAMETER_DOUBLE_ARRAY, double_array_value=[1.0, 2.0]),
            [1.0, 2.0],
        ),
    ]
    for value, expected in cases:
        assert _parameter_value(value) == expected
    assert _parameter_value(rcl.ParameterValue(type=t.PARAMETER_NOT_SET)) is None
