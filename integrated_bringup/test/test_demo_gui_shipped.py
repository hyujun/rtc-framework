"""Shipped hand postures (demo_gui.shipped) and the GUI's load of them.

The load rule is a pure function (``choose``); the GUI's half — moving a file
aside, seeding it, recording the seed — runs here on a stub instance against
tmp files, never the workspace's logging root.
"""

from __future__ import annotations

import json
import math
import shutil
from pathlib import Path
from types import MethodType, SimpleNamespace

import pytest
import yaml

from integrated_bringup.demo_gui import shipped as s
from integrated_bringup.demo_gui.app import DemoControllerGUI
from integrated_bringup.demo_gui.config import default_presets_for
from integrated_bringup.demo_sequence.files import FileProblem

_CONFIG = Path(__file__).resolve().parents[1] / "config"


def _preset(positions, side="right", **extra):
    return {"type": "open", "positions_deg": positions, "grasp_time": 1.0, "side": side, **extra}


SHIPPED = {"a": _preset([1.0, 2.0]), "b": _preset([3.0, 4.0])}
EDITED = {"a": _preset([1.0, 2.5]), "b": _preset([3.0, 4.0])}
NEWER = {"a": _preset([1.5, 2.0]), "b": _preset([3.0, 4.0])}
ROSTER = {"open_flat": _preset([0.0, 0.0])}


# ── The load rule ────────────────────────────────────────────────────────────


@pytest.mark.parametrize(
    ("state", "working", "seed", "shipped", "action", "aside", "differs", "records"),
    [
        # no working copy → seed
        (s.MISSING, None, None, SHIPPED, s.SEED, False, False, True),
        # unreadable / the other hand's → moved aside, then seed
        (s.CORRUPT, None, None, SHIPPED, s.SEED, True, False, True),
        (
            s.REFUSED,
            {"a": _preset([0.0, 0.0], side="left")},
            None,
            SHIPPED,
            s.SEED,
            True,
            False,
            True,
        ),
        # a default roster written before anything shipped → seed
        (s.PRESENT, ROSTER, None, SHIPPED, s.SEED, False, False, True),
        # the shipped copy, seed not yet recorded → keep, record it
        (s.PRESENT, SHIPPED, None, SHIPPED, s.KEEP, False, False, True),
        # the shipped copy, seed recorded → keep, nothing to write
        (s.PRESENT, SHIPPED, s.content_hash(SHIPPED), SHIPPED, s.KEEP, False, False, False),
        # unedited since its seed, shipped changed (a pull) → seed the new one
        (s.PRESENT, SHIPPED, s.content_hash(SHIPPED), NEWER, s.SEED, False, False, True),
        # edited → kept, and said to differ — also when shipped changed since
        (s.PRESENT, EDITED, s.content_hash(SHIPPED), SHIPPED, s.KEEP, False, True, False),
        (s.PRESENT, EDITED, s.content_hash(SHIPPED), NEWER, s.KEEP, False, True, False),
        # an edit that emptied the file is an edit
        (s.PRESENT, {}, s.content_hash(SHIPPED), SHIPPED, s.KEEP, False, True, False),
    ],
)
def test_the_load_rule(state, working, seed, shipped, action, aside, differs, records):
    choice = s.choose(state, working, seed, shipped, stale=(ROSTER,))
    assert (choice.action, choice.set_aside, choice.differs) == (action, aside, differs)
    assert bool(choice.seed_hash) is records
    if records:
        assert choice.seed_hash == s.content_hash(shipped)


def test_the_hash_ignores_key_order_and_layout():
    reordered = json.loads(json.dumps({"b": SHIPPED["b"], "a": SHIPPED["a"]}, indent=4))
    assert s.content_hash(reordered) == s.content_hash(SHIPPED)
    assert s.content_hash(EDITED) != s.content_hash(SHIPPED)


def test_read_working_classifies_the_file(tmp_path):
    path = tmp_path / "p.json"
    assert s.read_working(str(path), "right")[0] == s.MISSING
    path.write_text("{")
    assert s.read_working(str(path), "right")[0] == s.CORRUPT
    path.write_text("[1]")
    assert s.read_working(str(path), "right")[0] == s.CORRUPT
    path.write_text(json.dumps({"x": _preset([0, 0], side="left")}))
    assert s.read_working(str(path), "right")[0] == s.REFUSED
    path.write_text(json.dumps(SHIPPED))
    assert s.read_working(str(path), "right") == (s.PRESENT, SHIPPED)
    path.write_text(json.dumps({"steps": "no"}))
    assert s.read_working(str(path), sequence=True)[0] == s.CORRUPT


# ── Validation of a preset table as a release ────────────────────────────────

LIMITS = ((-90.0, 90.0), (-10.0, 120.0))


def test_a_valid_table_passes():
    assert s.validate_presets(SHIPPED, "f", "right", 2, LIMITS) == SHIPPED


@pytest.mark.parametrize(
    ("table", "match"),
    [
        ({}, "holds no preset"),
        ({"a": _preset([1.0, 2.0], side="left")}, "another hand"),
        ({"a": {k: v for k, v in _preset([1.0, 2.0]).items() if k != "side"}}, "no 'side: right'"),
        ({"a": _preset([1.0])}, "not 2 finite"),
        ({"a": _preset([1.0, math.nan])}, "not 2 finite"),
        ({"a": _preset([1.0, True])}, "not 2 finite"),
        ({"a": _preset([1.0, 121.0])}, "joint 1 at 121.0"),
        ({"a": _preset([-90.1, 0.0])}, "joint 0 at -90.1"),
        ({"a": {**_preset([1.0, 2.0]), "type": "grip"}}, "type 'grip'"),
        ({"a": {**_preset([1.0, 2.0]), "grasp_time": 0.0}}, "grasp_time"),
        ({"a": {**_preset([1.0, 2.0]), "grasp_time": math.inf}}, "grasp_time"),
        # a hand-only robot has no arm to aim
        (
            {"a": _preset([1.0, 2.0], controller="c", robot_goal_type="joint", robot_target=[])},
            "robot_target",
        ),
    ],
)
def test_a_table_that_cannot_play_is_refused(table, match):
    with pytest.raises(FileProblem, match=match):
        s.validate_presets(table, "f", "right", 2, LIMITS)


def test_a_position_on_a_limit_read_from_radians_is_inside_it():
    # π rad is 179.9999998 deg in a YAML that spells 3.14159265; the GUI's
    # 180.0 is on that limit, not past it.
    limits = ((-math.degrees(3.14159265), math.degrees(3.14159265)),) * 2
    s.validate_presets({"a": _preset([180.0, -180.0])}, "f", "right", 2, limits)
    with pytest.raises(FileProblem):
        s.validate_presets({"a": _preset([180.01, 0.0])}, "f", "right", 2, limits)


def test_robot_fields_are_checked_on_an_arm():
    joint = _preset([1.0, 2.0], controller="c", robot_goal_type="joint", robot_target=[0.0] * 6)
    task = {**joint, "robot_goal_type": "task"}
    s.validate_presets({"a": joint, "b": task}, "f", "right", 2, None, arm_dof=6)
    for bad in ({**joint, "robot_target": [0.0] * 5}, {**joint, "controller": ""}):
        with pytest.raises(FileProblem):
            s.validate_presets({"a": bad}, "f", "right", 2, None, arm_dof=6)


def test_limits_are_read_by_joint_name(tmp_path):
    base = tmp_path / "_base.yaml"
    base.write_text(
        yaml.safe_dump(
            {
                "/**": {
                    "ros__parameters": {
                        "devices": {
                            "g": {
                                "joint_state_names": ["j0", "j1"],
                                "joint_limits": {
                                    "position_lower": [-1.0, -0.5],
                                    "position_upper": [1.0, 2.0],
                                },
                            }
                        }
                    }
                }
            }
        )
    )
    # Asked in the other order: the limits follow the names, not the slots.
    lo_hi = s.hand_limits_deg(str(base), "g", ("j1", "j0"))
    flat = [v for pair in lo_hi for v in pair]
    assert flat == pytest.approx([math.degrees(v) for v in (-0.5, 2.0, -1.0, 1.0)])
    with pytest.raises(FileProblem, match="no limits for"):
        s.hand_limits_deg(str(base), "g", ("j0", "j9"))


def test_a_sequence_must_play_on_the_presets():
    good = {"steps": [{"preset": "a", "time_s": 1.0}], "tolerance_deg": 2.0}
    s.validate_sequence(good, "q", SHIPPED, 2)
    with pytest.raises(FileProblem, match="no longer exists"):
        s.validate_sequence({"steps": [{"preset": "zz", "time_s": 1.0}]}, "q", SHIPPED, 2)
    with pytest.raises(FileProblem, match="no steps"):
        s.validate_sequence({"steps": []}, "q", SHIPPED, 2)


# ── Calibration the postures were recorded under ─────────────────────────────


def _driver(tmp_path, params, name="drv.yaml"):
    path = tmp_path / name
    path.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}))
    return str(path)


def _recorded(signs=(1.0, -1.0), offsets=(0.0, 5.0)):
    return s.Calibration("drv.yaml", ("j0", "j1"), signs, offsets)


def test_the_driver_in_use_matches_or_names_the_joint(tmp_path):
    flat = _driver(
        tmp_path,
        {
            "joint_state_names": ["j0", "j1"],
            "joint_direction_signs": [1.0, -1.0],
            "joint_position_offsets_deg": [0.0, 5.0],
        },
    )
    assert s.calibration_mismatch(_recorded(), s.driver_calibration(flat, "right")) == ""
    problem = s.calibration_mismatch(
        _recorded(offsets=(0.0, 4.0)), s.driver_calibration(flat, "right")
    )
    assert "j1 joint_position_offsets_deg 5" in problem


def test_the_driver_side_keys_win_and_absent_keys_are_the_node_defaults(tmp_path):
    per_hand = _driver(
        tmp_path,
        {
            "joint_state_names": ["j0", "j1"],
            "joint_position_offsets_deg": [9.0, 9.0],
            "left": {"joint_position_offsets_deg": [0.0, 5.0]},
        },
    )
    in_use = s.driver_calibration(per_hand, "left")
    assert in_use.joint_position_offsets_deg == (0.0, 5.0)
    assert in_use.joint_direction_signs == (1.0, 1.0)  # absent → +1
    assert s.calibration_mismatch(_recorded(signs=(1.0, 1.0)), in_use) == ""


def test_check_calibration_reads_the_shipped_record(tmp_path):
    (tmp_path / "r" / "demo_gui").mkdir(parents=True)
    _driver(
        tmp_path / "r",
        {"joint_state_names": ["j0", "j1"], "joint_position_offsets_deg": [0.0, 5.0]},
    )
    record = {
        "driver_config": "drv.yaml",
        "joint_names": ["j0", "j1"],
        "joint_direction_signs": [1.0, 1.0],
        "joint_position_offsets_deg": [0.0, 5.0],
    }
    cal = tmp_path / "r" / "demo_gui" / s.CALIBRATION_FILE
    cal.write_text(yaml.safe_dump(record))
    assert s.check_calibration(str(tmp_path), "r", "") == ""
    cal.write_text(yaml.safe_dump({**record, "joint_position_offsets_deg": [0.0, 0.0]}))
    assert "j1" in s.check_calibration(str(tmp_path), "r", "")
    # nothing recorded → nothing to warn about
    assert s.check_calibration(str(tmp_path), "other", "") == ""


# ── Every shipped copy in this package plays on its robot ────────────────────


def _shipped_dirs():
    return sorted(p.parent.name for p in _CONFIG.glob(f"*/{s.SHIPPED_DIR}"))


@pytest.mark.parametrize("robot", _shipped_dirs())
def test_every_shipped_copy_plays_on_its_robot(robot):
    # Properties only — the values are the rig's and change with it. The
    # limits are read from the robot's _base.yaml as it is now.
    presets, _ = s.load_shipped(str(_CONFIG), s.ShippedProfile.for_robot(robot))
    assert presets, f"config/{robot}/{s.SHIPPED_DIR} ships no {s.PRESETS_FILE}"
    # The record of the calibration is readable (whether it matches the driver
    # is the GUI's warning and promote's refusal, not a test).
    s.read_calibration(str(_CONFIG / robot / s.SHIPPED_DIR / s.CALIBRATION_FILE))


def test_working_notes_say_when_the_run_plays_another_copy(tmp_path):
    # A copy of a shipped robot's files: the shipped side is read as installed.
    robot = _shipped_dirs()[0]
    config = tmp_path / "config"
    shutil.copytree(_CONFIG / robot, config / robot)
    presets, sequence = s.load_shipped(str(config), s.ShippedProfile.for_robot(robot))
    p_path, q_path = tmp_path / "p.json", tmp_path / "q.json"
    p_path.write_text(json.dumps(presets))
    q_path.write_text(json.dumps(sequence))
    assert s.working_notes(str(config), robot, str(p_path), str(q_path)) == []
    edited = dict(presets)
    edited.pop(next(iter(edited)))
    p_path.write_text(json.dumps(edited))
    notes = s.working_notes(str(config), robot, str(p_path), str(q_path))
    assert len(notes) == 1 and "p.json differs from the shipped copy" in notes[0]
    # Nothing shipped → nothing to say.
    assert s.working_notes(str(tmp_path / "none"), robot, str(p_path), str(q_path)) == []


# ── The GUI's half, on tmp files ─────────────────────────────────────────────


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(("info", m))

    def warn(self, m):
        self.lines.append(("warn", m))

    def error(self, m):
        self.lines.append(("error", m))


def _gui(tmp_path, shipped_presets=SHIPPED, shipped_sequence=None):
    log = _Log()
    gui = SimpleNamespace(
        _profile=SimpleNamespace(side="right", preset_token=lambda: "p2"),
        _shape=SimpleNamespace(hand_dof=2),
        _preset_path=str(tmp_path / "hand_presets_p2.json"),
        _sequence_path=str(tmp_path / "demo_gui_sequences_p2.json"),
        _shipped_presets=shipped_presets,
        _shipped_sequence=shipped_sequence,
        _shipped_notes=[],
        get_logger=lambda: log,
        log=log,
    )
    for name in (
        "_apply_shipped_choice",
        "_load_presets",
        "_load_presets_from_shipped",
        "_load_sequence",
        "_save_presets_to_file",
    ):
        setattr(gui, name, MethodType(getattr(DemoControllerGUI, name), gui))
    return gui


def _read(path):
    return json.loads(Path(path).read_text())


def test_the_gui_seeds_a_missing_copy_and_records_the_seed(tmp_path):
    gui = _gui(tmp_path)
    assert gui._load_presets() == SHIPPED
    assert _read(gui._preset_path) == SHIPPED
    assert s.read_seed_hash(gui._preset_path) == s.content_hash(SHIPPED)


def test_the_gui_follows_a_shipped_change_only_while_unedited(tmp_path):
    gui = _gui(tmp_path)
    gui._load_presets()
    gui._shipped_presets = NEWER
    assert gui._load_presets() == NEWER  # unedited → follows the pull
    Path(gui._preset_path).write_text(json.dumps(EDITED))  # an operator save
    gui._shipped_presets = SHIPPED
    assert gui._load_presets() == EDITED
    assert _read(gui._preset_path) == EDITED
    assert any("differs from shipped" in m for lvl, m in gui.log.lines if lvl == "warn")
    assert gui._shipped_notes


def test_the_gui_sets_a_corrupt_or_refused_copy_aside(tmp_path):
    gui = _gui(tmp_path)
    Path(gui._preset_path).write_text("{")
    assert gui._load_presets() == SHIPPED
    assert Path(gui._preset_path + ".refused").read_text() == "{"
    left = {"x": _preset([0.0, 0.0], side="left")}
    Path(gui._preset_path).write_text(json.dumps(left))
    assert gui._load_presets() == SHIPPED
    assert _read(gui._preset_path + ".refused.1") == left  # the first one is kept


def test_the_gui_replaces_an_unedited_default_roster(tmp_path):
    gui = _gui(tmp_path)
    Path(gui._preset_path).write_text(json.dumps(default_presets_for(2, "p2")))
    assert gui._load_presets() == SHIPPED


def test_without_a_shipped_copy_the_gui_loads_as_before(tmp_path):
    gui = _gui(tmp_path, shipped_presets=None)
    assert gui._load_presets() == default_presets_for(2, "p2")
    assert not Path(gui._preset_path + s.SEED_SUFFIX).exists()


SEQ = {"steps": [{"preset": "a", "time_s": 1.0, "dwell_s": 0.0}], "tolerance_deg": 2.0}


def test_the_gui_seeds_the_sequence_only_when_it_plays(tmp_path):
    gui = _gui(tmp_path, shipped_sequence=SEQ)
    gui._presets = gui._load_presets()
    seq = gui._load_sequence()
    assert [step.preset for step in seq.steps] == ["a"]
    assert _read(gui._sequence_path) == SEQ
    # A cleared list is an edit: kept, not refilled.
    Path(gui._sequence_path).write_text(json.dumps({"steps": [], "tolerance_deg": 2.0}))
    assert gui._load_sequence().steps == ()
    # Not seeded when a step's preset is not in the presets in use.
    Path(gui._sequence_path).unlink()
    gui._presets = {"b": SHIPPED["b"]}
    assert gui._load_sequence().steps == ()
    assert not Path(gui._sequence_path).exists()
