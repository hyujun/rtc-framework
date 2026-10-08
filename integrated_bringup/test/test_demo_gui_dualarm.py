"""Unit tests for demo_controller_gui's multi-frame CLIK support.

Pure-Python (no rclpy Node / Tk): ``demo_gui.dualarm`` — the YAML-derived spec,
the gain rows, the task-goal conversions, the measured-pose gate and the status
readout built from the tail of the controller's per-tick CSV log — plus the two
places it plugs into: ``config.register_scalar_gain_schema`` and the fault
fields ``catalog.build_entries`` carries.

Three of the tables under test are hand-written copies of something the C++
controller owns (parameter names, log columns, enum labels). Those are pinned to
the producer's source at the end of this file: checked only against themselves
they stay green when the controller changes.
"""

import math
import os
import re
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory

from integrated_bringup.demo_gui import config as gui_config, dualarm
from integrated_bringup.demo_gui.catalog import build_entries
from integrated_bringup.demo_gui.discovery import ROBOT_PROFILES
from integrated_bringup.demo_gui.dualarm import (
    DUALARM_CONFIG_KEY,
    LEVEL_BAD,
    LEVEL_IDLE,
    LEVEL_OK,
    LEVEL_WARN,
    CmView,
    DiagSample,
    DiagTail,
    MeasuredPose,
    MeasuredPoses,
    build_status,
    copy_allowed,
    diag_csv_path,
    dualarm_config_path,
    euler_note,
    format_goal_entries,
    frame_choices,
    frame_note,
    gain_group_layout,
    gain_rows,
    goal_frame_id,
    goal_topic,
    load_dualarm_spec,
    measured_child_frame,
    parse_dualarm_config,
    parse_goal_entries,
    resolve_session_dir,
    status_keys,
    task_status_key,
)
from integrated_bringup.demo_gui.pull import PLACEHOLDER
from rtc_msgs.msg import ControllerState

G1 = "g1_p1b"


def _share() -> str:
    return get_package_share_directory("integrated_bringup")


@pytest.fixture(scope="module")
def shipped():
    spec = load_dualarm_spec(dualarm_config_path(_share(), G1))
    assert spec is not None, "config/g1_p1b ships no demo_dualarm_controller.yaml"
    return spec


def _doc(**overrides):
    """A minimal well-formed controller YAML, as parsed."""
    root = {
        "clik": {
            "target_frames": ["world", "base"],
            "tasks": [
                {
                    "name": "tool",
                    "frame": "tool_link",
                    "base_frame": "base",
                    "gain_linear": 4.0,
                    "gain_angular": 2.0,
                }
            ],
            "posture_groups": [{"name": "arm", "joints": ["j1", "j2"], "gain": 1.5}],
        },
        "trajectory": {
            "linear_speed": 0.1,
            "angular_speed": 0.5,
            "linear_speed_max": 0.5,
            "angular_speed_max": 1.0,
            "hand_speed": 3.0,
            "hand_speed_max": 6.0,
        },
        "logs": [
            {"msg_type": "rtc_msgs/DeviceStateLog", "instance": "arm_state"},
            {"msg_type": "integrated_bringup/DualArmDiagLog", "instance": "diag"},
        ],
    }
    root.update(overrides)
    return {DUALARM_CONFIG_KEY: root}


# ── The spec is read from the controller's own YAML ──────────────────────────


def test_shipped_config_yields_the_tasks_frames_and_groups(shipped):
    assert [(t.name, t.frame, t.base_frame) for t in shipped.tasks] == [
        ("right_hand", "catch_frame", "pelvis"),
        ("left_hand", "left_rubber_hand", "torso_link"),
    ]
    assert shipped.target_frames == ("world", "pelvis", "torso_link")
    assert [g.name for g in shipped.posture_groups] == ["waist", "left_arm", "right_arm"]
    assert shipped.diag_instance == "dualarm_diag"


def test_shipped_posture_groups_cover_the_profile_joints_in_order(shipped):
    """The GUI's Joint Target sends every joint of the first device group as the
    posture goal. The posture groups partition that same roster — if the two
    drift, a posture goal names joints no group holds."""
    flat = tuple(j for g in shipped.posture_groups for j in g.joints)
    assert flat == ROBOT_PROFILES[G1].shape.arm_joint_names


def test_only_the_profiles_that_ship_the_yaml_get_a_spec():
    share = _share()
    have = {k for k in ROBOT_PROFILES if load_dualarm_spec(dualarm_config_path(share, k))}
    offered = {
        k for k, p in ROBOT_PROFILES.items() if DUALARM_CONFIG_KEY in p.switchable_controllers(())
    }
    assert have == offered == {G1}


def test_a_profile_without_the_yaml_has_no_spec(tmp_path):
    assert load_dualarm_spec(str(tmp_path / "nope.yaml")) is None


def test_an_unusable_yaml_is_an_error_not_a_half_built_panel(tmp_path):
    path = tmp_path / "demo_dualarm_controller.yaml"
    path.write_text("demo_dualarm_controller: {clik: {tasks: []}}\n")
    with pytest.raises(ValueError, match="not usable"):
        load_dualarm_spec(str(path))
    path.write_text("demo_dualarm_controller: [unclosed\n")
    with pytest.raises(ValueError):
        load_dualarm_spec(str(path))


@pytest.mark.parametrize("missing", ["target_frames", "tasks", "posture_groups"])
def test_a_missing_clik_key_is_refused(missing):
    doc = _doc()
    del doc[DUALARM_CONFIG_KEY]["clik"][missing]
    with pytest.raises(ValueError, match=missing):
        parse_dualarm_config(doc)


def test_no_task_and_a_repeated_task_name_are_refused():
    doc = _doc()
    doc[DUALARM_CONFIG_KEY]["clik"]["tasks"] = []
    with pytest.raises(ValueError, match="no task"):
        parse_dualarm_config(doc)
    doc = _doc()
    task = doc[DUALARM_CONFIG_KEY]["clik"]["tasks"][0]
    doc[DUALARM_CONFIG_KEY]["clik"]["tasks"] = [task, dict(task)]
    with pytest.raises(ValueError, match="repeats"):
        parse_dualarm_config(doc)


def test_a_run_without_the_diag_log_has_no_instance():
    doc = _doc(logs=[{"msg_type": "rtc_msgs/DeviceStateLog", "instance": "arm_state"}])
    assert parse_dualarm_config(doc).diag_instance == ""
    doc = _doc()
    del doc[DUALARM_CONFIG_KEY]["logs"]
    assert parse_dualarm_config(doc).diag_instance == ""
    assert parse_dualarm_config(_doc()).diag_instance == "diag"


# ── Gain rows ────────────────────────────────────────────────────────────────


def test_gain_rows_name_the_parameters_after_the_configured_names(shipped):
    rows = gain_rows(shipped)
    assert [r.param for r in rows] == [
        "tasks.right_hand.gain_linear",
        "tasks.right_hand.gain_angular",
        "tasks.left_hand.gain_linear",
        "tasks.left_hand.gain_angular",
        "posture.waist.gain",
        "posture.left_arm.gain",
        "posture.right_arm.gain",
        "trajectory.linear_speed",
        "trajectory.linear_speed_max",
        "trajectory.angular_speed",
        "trajectory.angular_speed_max",
        "trajectory.hand_speed",
        "trajectory.hand_speed_max",
    ]
    # Exactly the caps are read-only: Apply Gains skips them, and a writable
    # row marked read-only would silently never be applied.
    assert [r.param for r in rows if r.read_only] == [
        "trajectory.linear_speed_max",
        "trajectory.angular_speed_max",
        "trajectory.hand_speed_max",
    ]
    assert [r.default for r in rows[:4]] == [10.0, 5.0, 10.0, 5.0]


def test_gain_rows_follow_the_config_not_a_fixed_roster():
    doc = _doc()
    clik = doc[DUALARM_CONFIG_KEY]["clik"]
    clik["tasks"].append({**clik["tasks"][0], "name": "elbow", "gain_linear": 7.0})
    clik["posture_groups"] = []
    spec = parse_dualarm_config(doc)
    rows = gain_rows(spec)
    assert [r.param for r in rows[:4]] == [
        "tasks.tool.gain_linear",
        "tasks.tool.gain_angular",
        "tasks.elbow.gain_linear",
        "tasks.elbow.gain_angular",
    ]
    assert not [r for r in rows if r.param.startswith("posture.")]
    labels = [r.label for r in rows]
    assert len(set(labels)) == len(labels)
    # Every group a row is placed in has a box in the layout.
    laid_out = {g for row in gain_group_layout(spec) for g in row}
    assert {r.group for r in rows} == laid_out


@pytest.fixture
def clean_gain_tables():
    """register_scalar_gain_schema writes process-wide tables — put them back."""
    tables = (
        gui_config.GAIN_DEFS,
        gui_config.GAIN_PARAM_DISPATCH,
        gui_config.GAIN_ROW_NAMES,
        gui_config.GAIN_GROUP_LAYOUT,
    )
    before = [dict(t) for t in tables]
    yield
    for table, saved in zip(tables, before, strict=True):
        table.clear()
        table.update(saved)


def test_registered_schema_is_what_the_gains_panel_reads(shipped, clean_gain_tables):
    rows = gain_rows(shipped)
    gui_config.register_scalar_gain_schema(DUALARM_CONFIG_KEY, rows, gain_group_layout(shipped))

    defs = gui_config.GAIN_DEFS[DUALARM_CONFIG_KEY]
    dispatch = gui_config.GAIN_PARAM_DISPATCH[DUALARM_CONFIG_KEY]
    # One scalar widget per row, in row order: Load Gain fills the widgets by
    # zipping the dispatch with the parameter reply, so the two orders must agree.
    assert [d[0] for d in defs] == list(dispatch) == [r.label for r in rows]
    assert all(size == 1 and not is_bool for _, size, _, is_bool, _ in defs)
    assert [dispatch[r.label][0] for r in rows] == [r.param for r in rows]
    for row in rows:
        built = dispatch[row.label][1]([row.default])
        if row.read_only:
            assert built is None
        else:
            assert built[1] == row.default
    # It is a joint-space controller to the arm panel: a posture goal, no task half.
    assert gui_config.target_panel_states(DUALARM_CONFIG_KEY) == (True, False)


def test_a_built_schema_never_replaces_a_written_one(shipped, clean_gain_tables):
    rows = gain_rows(shipped)
    with pytest.raises(ValueError, match="already has a gain schema"):
        gui_config.register_scalar_gain_schema("demo_joint_controller", rows, [])
    gui_config.register_scalar_gain_schema(DUALARM_CONFIG_KEY, rows, [])
    with pytest.raises(ValueError, match="already has a gain schema"):
        gui_config.register_scalar_gain_schema(DUALARM_CONFIG_KEY, rows, [])


def test_static_tables_do_not_name_the_controller():
    """Its rows carry one robot's task names. Written into the static tables
    they would be offered as a radio on every profile's offline list."""
    for table in (gui_config.GAIN_DEFS, gui_config.GAIN_PARAM_DISPATCH):
        assert DUALARM_CONFIG_KEY not in table


# ── Task goals ───────────────────────────────────────────────────────────────


def test_goal_topic_is_per_task():
    assert goal_topic("right_hand") == "/demo_dualarm_controller/right_hand/task_goal"


def test_frame_choices_lead_with_the_task_base_and_do_not_repeat_it(shipped):
    right, left = shipped.tasks
    assert frame_choices(shipped, right) == ("pelvis", "world", "torso_link")
    assert frame_choices(shipped, left) == ("torso_link", "world", "pelvis")


def test_the_base_frame_goes_out_as_the_empty_frame_id(shipped):
    right, left = shipped.tasks
    assert goal_frame_id(right, "pelvis") == ""
    assert goal_frame_id(right, "world") == "world"
    assert goal_frame_id(left, "torso_link") == ""
    assert goal_frame_id(left, "pelvis") == "pelvis"


def test_the_base_frame_is_offered_even_when_it_is_not_a_goal_frame():
    doc = _doc()
    doc[DUALARM_CONFIG_KEY]["clik"]["target_frames"] = ["world"]
    spec = parse_dualarm_config(doc)
    task = spec.tasks[0]
    assert frame_choices(spec, task) == ("base", "world")
    # ...and reaches the controller by the one spelling it always accepts.
    assert goal_frame_id(task, "base") == ""


def test_goal_entries_are_metres_and_degrees():
    target = parse_goal_entries(["0.3", "-0.1", "0.25", "90", "-45", "180"])
    assert target[:3] == [0.3, -0.1, 0.25]
    assert target[3:] == pytest.approx([math.pi / 2, -math.pi / 4, math.pi])


@pytest.mark.parametrize(
    "texts",
    [
        ["", "", "", "", "", ""],  # never seeded
        ["0.1", "0.2", "0.3", "0", "0"],  # five values
        ["0.1", "0.2", "x", "0", "0", "0"],
        ["0.1", "0.2", "nan", "0", "0", "0"],
        ["0.1", "0.2", "0.3", "inf", "0", "0"],
    ],
)
def test_a_goal_that_is_not_six_finite_numbers_is_refused(texts):
    with pytest.raises(ValueError):
        parse_goal_entries(texts)


def test_formatting_a_pose_and_parsing_it_back_is_the_same_pose():
    xyz = (0.31234, -0.2, 0.05)
    rpy = (0.4, -1.567, 2.9)
    back = parse_goal_entries(format_goal_entries(xyz, rpy))
    assert back[:3] == pytest.approx(xyz, abs=1e-4)
    assert back[3:] == pytest.approx(rpy, abs=math.radians(1e-4))


def test_an_unfilled_goal_says_so_in_words():
    with pytest.raises(ValueError, match="six numbers"):
        parse_goal_entries(["", "", "", "", "", ""])


def test_a_goal_outside_the_base_frame_is_said_to_be_converted_once(shipped):
    right, left = shipped.tasks
    assert frame_note(right, "pelvis") == ""
    assert frame_note(left, "torso_link") == ""
    note = frame_note(left, "pelvis")
    assert "'pelvis'" in note and "'torso_link'" in note and "once" in note


def test_the_euler_note_appears_only_next_to_the_singularity():
    assert euler_note(0.0) == ""
    assert euler_note(math.radians(80.0)) == ""
    assert "-89.8" in euler_note(math.radians(-89.8))
    assert euler_note(math.radians(88.0)) != ""


# ── Measured poses ───────────────────────────────────────────────────────────


def _pose(parent="pelvis", stamp=10.0):
    return MeasuredPose(parent=parent, xyz=(0.1, 0.2, 0.3), rpy=(0.0, 0.1, 0.2), stamp_s=stamp)


def test_measured_frame_is_the_actual_suffixed_task_frame(shipped):
    assert [measured_child_frame(t) for t in shipped.tasks] == [
        "catch_frame_actual",
        "left_rubber_hand_actual",
    ]


def test_a_pose_stops_being_current_once_the_broadcast_stops():
    poses = MeasuredPoses()
    poses.observe("tool_actual", _pose(stamp=10.0))
    assert poses.get("tool_actual", 10.5) is not None
    assert poses.get("tool_actual", 10.0 + MeasuredPoses.MAX_AGE_S + 0.01) is None
    assert poses.get("other_actual", 10.0) is None
    poses.observe("tool_actual", _pose(stamp=20.0))
    poses.clear()
    assert poses.get("tool_actual", 20.0) is None


def test_copy_is_allowed_only_into_the_frame_the_pose_is_expressed_in():
    pose = _pose(parent="pelvis")
    assert copy_allowed("pelvis", pose)
    # The GUI has no transform between these: a copy would be a guess that reads
    # as "stay here".
    assert not copy_allowed("torso_link", pose)
    assert not copy_allowed("world", pose)
    assert not copy_allowed("pelvis", None)


# ── Finding the log ──────────────────────────────────────────────────────────


def test_session_resolution_order(tmp_path):
    root = tmp_path / "logging_data"
    for name in ("261008_0900", "261008_1015", "stats", "ur_plot"):
        (root / name).mkdir(parents=True)
    (root / "261009_0000").write_text("a file, not a session")
    (root / "hand_presets_p1b.json").write_text("{}")

    assert resolve_session_dir("/explicit", "/env", str(root)) == ("/explicit", "--session")
    assert resolve_session_dir(None, "/env", str(root)) == ("/env", "$RTC_SESSION_DIR")
    path, how = resolve_session_dir(None, None, str(root))
    assert (path, how) == (str(root / "261008_1015"), "newest session")


def test_no_session_is_said_not_guessed(tmp_path):
    path, how = resolve_session_dir(None, None, str(tmp_path / "missing"))
    assert path is None and "no session" in how
    (tmp_path / "empty").mkdir()
    assert resolve_session_dir(None, None, str(tmp_path / "empty"))[0] is None


def test_diag_csv_path_is_under_the_controllers_own_directory():
    assert diag_csv_path("/s", "dualarm_diag") == (
        "/s/controllers/demo_dualarm_controller/dualarm_diag.csv"
    )


# ── Reading the tail of a file that is still being written ───────────────────

_HEADER = "t_relative_s,tick,hold,clik_ran"


def _write(path, text):
    with open(path, "w") as f:
        f.write(text)


def test_tail_returns_the_last_complete_row(tmp_path):
    path = str(tmp_path / "diag.csv")
    _write(path, f"{_HEADER}\n0.0,1,0,1\n0.002,2,0,1\n")
    sample = DiagTail().read(path)
    assert sample.values == {"t_relative_s": "0.002", "tick": "2", "hold": "0", "clik_ran": "1"}
    assert sample.mtime_s == pytest.approx(os.stat(path).st_mtime)


def test_tail_skips_the_row_still_being_written(tmp_path):
    path = str(tmp_path / "diag.csv")
    # The writer is mid-row: the last line has no newline. With four of four
    # cells already out it would pass a cell count, and its last cell is cut.
    _write(path, f"{_HEADER}\n0.0,1,0,1\n0.002,2,0,1\n0.004,3,0,")
    assert DiagTail().read(path).values["tick"] == "2"
    _write(path, f"{_HEADER}\n0.0,1,0,1\n0.002,2,0,1\n0.004,3,2,0")
    assert DiagTail().read(path).values["tick"] == "2"


def test_tail_has_nothing_before_the_first_row(tmp_path):
    tail = DiagTail()
    path = str(tmp_path / "diag.csv")
    assert tail.read(path) is None  # no file
    _write(path, "")
    assert tail.read(path) is None
    _write(path, _HEADER)  # header still being written
    assert tail.read(path) is None
    _write(path, f"{_HEADER}\n")
    assert tail.read(path) is None
    _write(path, f"{_HEADER}\n0.0,1,0,1\n")
    assert tail.read(path).values["tick"] == "1"


def test_tail_reads_only_the_end_of_a_long_file(tmp_path):
    path = str(tmp_path / "diag.csv")
    rows = "".join(f"{i * 0.002:.3f},{i},0,1\n" for i in range(1, 5001))
    _write(path, f"{_HEADER}\n{rows}")
    tail = DiagTail(tail_bytes=64)
    assert tail.read(path).values["tick"] == "5000"
    # A window that starts inside a row: the cut row is at the front of the
    # chunk and must not be taken for a row, however many cells it has.
    for size in range(20, 40):
        sample = DiagTail(tail_bytes=size).read(path)
        assert sample is None or sample.values["tick"] == "5000", size


def test_tail_follows_a_new_file_with_a_different_header(tmp_path):
    tail = DiagTail()
    first = str(tmp_path / "a.csv")
    second = str(tmp_path / "b.csv")
    _write(first, f"{_HEADER}\n0.0,1,0,1\n")
    _write(second, "tick,hold\n9,4\n")
    assert tail.read(first).values["tick"] == "1"
    assert tail.read(second).values == {"tick": "9", "hold": "4"}
    assert tail.read(first).values["clik_ran"] == "1"


# ── Status rows ──────────────────────────────────────────────────────────────

TASKS = ("right_hand", "left_hand")


def _row(**overrides):
    values = {
        "tick": "4200",
        "hold": "0",
        "clik_ran": "1",
        "fault_latched": "0",
        "fault_cause": "0",
        "reseeded": "0",
        "reached_solve": "1",
        "converged": "1",
        "rejected_input": "0",
        "non_finite": "0",
        "command_mismatch": "0",
        "accel_rows_violated": "0",
        "brake_box_empty": "0",
        "status": "0",
        "iterations": "3",
        "solve_time_us": "85",
        "accel_rows": "17",
        "accel_rows_binding": "0",
        "fb_saturated": "0",
        "rot_near_pi": "0",
        "brake_active": "0",
        "brake_static_infeasible": "0",
        "qp_fail_streak": "0",
        "track_err": "0.0012",
        "group_goal_rejects": "0",
    }
    for task in TASKS:
        values.update(
            {
                f"{task}_valid": "1",
                f"{task}_traj_active": "0",
                f"{task}_goal_sequence": "2",
                f"{task}_err_lin": "0.0005",
                f"{task}_err_ang": "0.001",
                f"{task}_goals_accepted": "2",
                f"{task}_reject_goal_type": "0",
                f"{task}_reject_non_finite": "0",
                f"{task}_reject_unknown_frame": "0",
                f"{task}_drop_stale": "0",
                f"{task}_drop_unusable": "0",
                f"{task}_drop_held": "0",
            }
        )
    values.update({k: str(v) for k, v in overrides.items()})
    return values


def _status(values=None, *, age_s=0.1, cm=None, source="/s/diag.csv"):
    sample = None if values is None else DiagSample(values, mtime_s=1000.0 - age_s)
    return build_status(TASKS, sample, 1000.0, cm, source)


def test_status_has_every_key_whatever_the_input():
    keys = set(status_keys(TASKS))
    assert keys == set(_status()) == set(_status(_row())) == set(_status(_row(), age_s=60.0))
    assert task_status_key("left_hand", "error") in keys


def test_no_log_blanks_the_rows_and_says_why():
    fields = _status(source="no session under /x/logging_data")
    assert fields["log"].text == "no session under /x/logging_data"
    for key in status_keys(TASKS):
        if key not in ("cm", "log"):
            assert fields[key].text == PLACEHOLDER, key
            assert fields[key].level == LEVEL_IDLE


def test_a_log_that_stopped_growing_is_withheld_not_frozen():
    """The last row of a controller that is no longer running is not its state:
    shown, it reads as a live solve."""
    fields = _status(_row(), age_s=dualarm.STALE_AFTER_S + 0.5)
    assert "withheld" in fields["log"].text and fields["log"].level == LEVEL_IDLE
    for key in status_keys(TASKS):
        if key not in ("cm", "log"):
            assert fields[key].text == PLACEHOLDER, key


def test_a_solved_tick_reads_ok():
    fields = _status(_row())
    assert fields["log"].level == LEVEL_OK
    assert fields["tick"].text.startswith("solved") and "4200" in fields["tick"].text
    assert fields["tick"].level == LEVEL_OK
    assert "converged" in fields["solver"].text and "85 µs" in fields["solver"].text
    assert "3 iter" in fields["solver"].text
    assert fields["solver"].level == LEVEL_OK
    assert fields["fault"].text.startswith("none") and fields["fault"].level == LEVEL_OK
    assert fields["limits"].level == LEVEL_OK
    err = fields[task_status_key("right_hand", "error")]
    assert err.text.startswith("0.50 mm") and "0.057°" in err.text
    assert fields[task_status_key("right_hand", "goal")].text == "#2  ·  settled"
    counters = fields[task_status_key("left_hand", "counters")]
    assert counters.text == "accepted 2  ·  rejected 0  ·  dropped 0"
    assert counters.level == LEVEL_OK


@pytest.mark.parametrize(
    ("hold", "name", "level"),
    [
        (1, "E-STOP", LEVEL_BAD),
        (2, "fault latch", LEVEL_BAD),
        (3, "body unreadable", LEVEL_WARN),
        (4, "not seeded", LEVEL_WARN),
        (5, "no model", LEVEL_WARN),
        (9, "code 9", LEVEL_WARN),
    ],
)
def test_a_held_tick_names_why_and_shows_no_solve(hold, name, level):
    fields = _status(_row(hold=hold, clik_ran=0, reached_solve=0))
    assert fields["tick"].text.startswith(f"HOLD — {name}")
    assert fields["tick"].level == level
    # The solve fields of a tick that did not solve are zeros in the file.
    assert fields["solver"].text == PLACEHOLDER
    assert fields["limits"].text == PLACEHOLDER


def test_a_refused_input_is_not_shown_as_a_zero_microsecond_solve():
    fields = _status(_row(reached_solve=0, rejected_input=1, solve_time_us=0, iterations=0))
    assert "refused before the solve" in fields["solver"].text
    assert "rejected_input" in fields["solver"].text
    assert "µs" not in fields["solver"].text
    assert fields["solver"].level == LEVEL_BAD


def test_an_unconverged_or_flagged_solve_is_marked():
    assert _status(_row(converged=0))["solver"].level == LEVEL_WARN
    flagged = _status(_row(command_mismatch=1))["solver"]
    assert flagged.level == LEVEL_BAD and "command_mismatch" in flagged.text


def test_a_latched_fault_names_its_cause():
    fields = _status(_row(hold=2, clik_ran=0, fault_latched=1, fault_cause=1, qp_fail_streak=5))
    assert fields["fault"].text.startswith("LATCHED — QP fail streak")
    assert fields["fault"].level == LEVEL_BAD
    assert _status(_row(qp_fail_streak=2))["fault"].level == LEVEL_WARN


def test_active_limits_are_named():
    fields = _status(_row(accel_rows_binding=2, fb_saturated=1, brake_active=4))
    assert "2 binding" in fields["limits"].text
    assert "feedback capped" in fields["limits"].text and "braking" in fields["limits"].text
    assert fields["limits"].level == LEVEL_WARN


def test_a_task_that_was_not_computed_shows_no_error():
    fields = _status(_row(left_hand_valid=0, left_hand_err_lin=0.0, left_hand_err_ang=0.0))
    assert fields[task_status_key("left_hand", "error")].text == PLACEHOLDER
    assert fields[task_status_key("right_hand", "error")].text != PLACEHOLDER


def test_a_moving_goal_and_a_refused_goal_show_on_the_task_row():
    fields = _status(
        _row(
            right_hand_traj_active=1,
            right_hand_goal_sequence=3,
            right_hand_reject_unknown_frame=1,
            right_hand_drop_unusable=2,
        )
    )
    goal = fields[task_status_key("right_hand", "goal")]
    assert goal.text == "#3  ·  moving" and goal.level == LEVEL_WARN
    counters = fields[task_status_key("right_hand", "counters")]
    assert "rejected 1" in counters.text and "dropped 2" in counters.text
    assert "unknown_frame 1" in counters.text and "unusable 2" in counters.text
    assert counters.level == LEVEL_WARN
    assert fields[task_status_key("left_hand", "counters")].level == LEVEL_OK


def test_counters_are_found_by_prefix_so_a_renamed_cause_still_counts():
    """An earlier version of the log wrote ``drop_near_pi`` where this one
    writes ``drop_unusable``. A list of names would read that file as zero."""
    values = _row()
    del values["right_hand_drop_unusable"]
    values["right_hand_drop_near_pi"] = "3"
    counters = _status(values)[task_status_key("right_hand", "counters")]
    assert "dropped 3" in counters.text and "near_pi 3" in counters.text


def test_a_task_goal_sent_to_a_group_topic_is_surfaced():
    fault = _status(_row(group_goal_rejects=2))["fault"]
    assert "group topic 2" in fault.text and fault.level == LEVEL_WARN


def test_a_row_missing_columns_does_not_raise():
    fields = _status({"tick": "7", "hold": "0", "clik_ran": "1"})
    assert fields["tick"].text.startswith("solved")
    assert fields[task_status_key("right_hand", "error")].text == PLACEHOLDER


def test_the_controller_row_comes_from_list_controllers():
    assert _status()["cm"].level == LEVEL_IDLE
    assert "has not reported" in _status()["cm"].text
    active = CmView("active", True, False, 0, 0)
    assert _status(cm=active)["cm"].text.startswith("active")
    assert _status(cm=active)["cm"].level == LEVEL_OK
    inactive = CmView("inactive", False, False, 0, 0)
    assert _status(cm=inactive)["cm"].level == LEVEL_IDLE
    faulted = CmView("active", True, True, 1, 2)
    field = _status(cm=faulted)["cm"]
    assert "FAULT LATCHED" in field.text and field.level == LEVEL_BAD
    assert "reject 1 / drop 2" in field.text


def test_catalog_entries_carry_the_fault_latch_and_mailbox_counters():
    cs = ControllerState()
    cs.name = "DemoDualArmController"
    cs.type = DUALARM_CONFIG_KEY
    cs.state = "active"
    cs.is_active = True
    cs.has_latched_fault = True
    cs.target_reject_count = 3
    cs.target_drop_count = 4
    (entry,) = build_entries([cs], ())
    assert entry.has_latched_fault is True
    assert (entry.target_reject_count, entry.target_drop_count) == (3, 4)
    (quiet,) = build_entries([ControllerState()], ())
    assert quiet.has_latched_fault is False


# ── The producer's source is the owner — pin the copies here to it ───────────

_PKG = Path(__file__).resolve().parents[1]
_POD_HEADER = _PKG / "include" / "integrated_bringup" / "logging" / "dualarm_diag_log_pod.hpp"
_PARAMETERS = _PKG / "src" / "controllers" / "dualarm" / "parameters.cpp"

_HOLD_LABELS = {
    "kNone": "solved",
    "kEstop": "E-STOP",
    "kFault": "fault latch",
    "kUnreadable": "body unreadable",
    "kNotSeeded": "not seeded",
    "kNoModel": "no model",
}
_FAULT_LABELS = {
    "kNone": "none",
    "kQpFailStreak": "QP fail streak",
    "kTrackError": "tracking error",
    "kSeedOutsideBox": "seed outside box",
}

# Columns build_status reads by name. Fixed ones first, then the per-task
# suffixes (`reject_` / `drop_` are prefixes of a family).
_FIXED_COLUMNS_READ = (
    "tick",
    "hold",
    "clik_ran",
    "fault_latched",
    "fault_cause",
    "reseeded",
    "reached_solve",
    "converged",
    "rejected_input",
    "non_finite",
    "command_mismatch",
    "accel_rows_violated",
    "brake_box_empty",
    "status",
    "iterations",
    "solve_time_us",
    "accel_rows",
    "accel_rows_binding",
    "fb_saturated",
    "rot_near_pi",
    "brake_active",
    "brake_static_infeasible",
    "qp_fail_streak",
    "track_err",
    "group_goal_rejects",
)
_TASK_SUFFIXES_READ = (
    "_valid",
    "_traj_active",
    "_goal_sequence",
    "_err_lin",
    "_err_ang",
    "_goals_accepted",
    "_reject_",
    "_drop_",
)


def _source(path: Path) -> str:
    if not path.exists():  # pragma: no cover - installed without the source tree
        pytest.skip(f"controller source not present at {path}")
    return path.read_text()


def _enumerators(text, enum_name):
    body = text.split(f"enum class {enum_name} ", 1)[1].split("{", 1)[1].split("};", 1)[0]
    body = re.sub(r"//[^\n]*", "", body)
    return [tok.split("=")[0].strip() for tok in body.split(",") if tok.strip()]


def test_hold_and_fault_names_follow_the_enums():
    text = _source(_POD_HEADER)
    holds = _enumerators(text, "Hold")
    assert [_HOLD_LABELS[e] for e in holds] == list(dualarm.HOLD_NAMES)
    # _tick_field marks exactly these two as needing the operator.
    assert (holds.index("kEstop"), holds.index("kFault")) == (1, 2)
    faults = _enumerators(text, "FaultCause")
    assert [_FAULT_LABELS[e] for e in faults] == list(dualarm.FAULT_CAUSE_NAMES)


def test_the_test_row_and_the_reader_use_columns_the_writer_emits():
    writer = _source(_POD_HEADER).split("inline void WriteDualArmDiagLogHeader(", 1)[1]
    writer = writer.split("\n}\n", 1)[0]
    for column in _FIXED_COLUMNS_READ:
        assert re.search(rf'[",]{column}[",]', writer), f"writer does not emit '{column}'"
    for suffix in _TASK_SUFFIXES_READ:
        assert suffix in writer, f"writer emits no per-task '{suffix}' column"
    # ...and _row() above is made of exactly those, so the status tests exercise
    # the names the file really has.
    row = _row()
    fixed = {k for k in row if not k.startswith(TASKS)}
    assert fixed == set(_FIXED_COLUMNS_READ)
    assert dualarm.DIAG_LOG_MSG_TYPE in _source(_POD_HEADER)


def test_gain_parameter_names_are_the_ones_the_controller_declares(shipped):
    text = _source(_PARAMETERS)
    # tasks.<task>.<field> and posture.<group>.gain are assembled; the pieces
    # are literal.
    assert '"tasks." + task + "." + field' in text
    assert '"posture." + group + ".gain"' in text
    for field in ("gain_linear", "gain_angular"):
        assert f'TaskParam(task, "{field}")' in text
    for row in gain_rows(shipped):
        if row.param.startswith("trajectory."):
            assert f'"{row.param}"' in text, row.param
            cap = f'declare_cap("{row.param}"' in text
            assert cap == row.read_only, f"{row.param}: read-only flag disagrees"
