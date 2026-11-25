"""demo_controller_sequence without ROS: the files, the readiness table, the run.

The orchestrator is driven through a fake ``Io`` whose observation a test sets
tick by tick, so every row of the operator document's table and every way a
run ends is pinned here; ``test_demo_sequence_ros`` covers the rclpy wiring.
"""

from __future__ import annotations

import json
import math
import re
import shutil
from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from integrated_bringup.demo_gui.sequence import ResolvedStep, SequenceRunner
from integrated_bringup.demo_sequence import cli, env, files as f
from integrated_bringup.demo_sequence.gates import DwellWatch, disk_verdict, drift_threshold_deg
from integrated_bringup.demo_sequence.orchestrator import (
    CONFIRM,
    DISK_SAMPLE_S,
    RUNNING,
    SPEED_TIMEOUT_S,
    Orchestrator,
    RunConfig,
)
from integrated_bringup.demo_sequence.readiness import (
    READY,
    REFUSE,
    WAIT,
    Observation,
    assess,
    runtime_abort,
)

CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
NAMES = ("j0", "j1")


# ── Files ────────────────────────────────────────────────────────────────────


# The run reads a robot's shipped copy (config/<robot>/demo_gui/ of the
# installed package): each test builds one under tmp_path from p2's profile
# files, with release presets taken from p2's own shipped table.
_SHIPPED_P2 = json.loads((CONFIG_DIR / "p2" / "demo_gui" / "hand_presets.json").read_text())
# Any two of them: the names below are this file's, the bodies a release's.
_OPEN, _CLOSE = list(_SHIPPED_P2.values())[:2]
PRESETS = {"open": _OPEN, "close": _CLOSE}
SEQUENCE = {
    "steps": [
        {"preset": "close", "time_s": 1.0, "dwell_s": 2.0},
        {"preset": "open", "time_s": 1.0, "dwell_s": 0.0},
    ],
    "tolerance_deg": 2.0,
}
MISSING = object()


def _ship(tmp_path: Path, presets: object = PRESETS, sequence: object = SEQUENCE) -> str:
    """A config dir holding p2's profile with ``presets`` / ``sequence`` as its
    shipped copy (MISSING: no such file; a str: the file's raw text)."""
    config = tmp_path / "config"
    shutil.copytree(CONFIG_DIR / "p2", config / "p2")
    shipped = config / "p2" / "demo_gui"
    for name, data in (("hand_presets.json", presets), ("sequence.json", sequence)):
        path = shipped / name
        if data is MISSING:
            path.unlink()
        else:
            path.write_text(data if isinstance(data, str) else json.dumps(data))
    return str(config)


def test_inputs_are_the_shipped_copy_resolved(tmp_path):
    inputs = f.load_run_inputs(_ship(tmp_path), "p2")
    assert [s.preset for s in inputs.steps] == ["close", "open"]
    assert inputs.steps[0].target_rad == pytest.approx(
        tuple(math.radians(v) for v in _CLOSE["positions_deg"])
    )
    assert inputs.tolerance_deg == 2.0


def test_inputs_write_nothing(tmp_path):
    config = _ship(tmp_path)
    before = sorted(p.relative_to(tmp_path) for p in tmp_path.rglob("*"))
    f.load_run_inputs(config, "p2")
    assert sorted(p.relative_to(tmp_path) for p in tmp_path.rglob("*")) == before


@pytest.mark.parametrize(
    "presets,sequence,needle",
    [
        (MISSING, SEQUENCE, "does not exist"),
        ({"open": {**_OPEN, "side": "left"}}, SEQUENCE, "another hand"),
        (PRESETS, {"steps": []}, "no steps"),
        (PRESETS, {"steps": [{"preset": "open", "time_s": 0.0}]}, "time_s"),
        (PRESETS, {"steps": [{"preset": "open", "time_s": 601.0}]}, "time_s"),
        (PRESETS, {"steps": [{"preset": "open", "time_s": 1.0, "dwell_s": -1}]}, "dwell_s"),
        (PRESETS, {"steps": [{"preset": "open", "time_s": True}]}, "time_s"),
        (PRESETS, {"steps": ["open"]}, "not an object"),
        (PRESETS, {"steps": [{"time_s": 1.0}]}, "names no preset"),
        (PRESETS, {"steps": [{"preset": "gone", "time_s": 1.0}]}, "no longer exists"),
        (PRESETS, {**SEQUENCE, "tolerance_deg": 40.0}, "tolerance_deg"),
        (
            {"open": {**_OPEN, "positions_deg": [0.0]}},
            {"steps": [{"preset": "open", "time_s": 1}]},
            "positions_deg is not 10 finite numbers",
        ),
    ],
)
def test_inputs_refuse_what_the_gui_would_clamp_or_drop(tmp_path, presets, sequence, needle):
    with pytest.raises(f.FileProblem, match=needle):
        f.load_run_inputs(_ship(tmp_path, presets, sequence), "p2")


def test_a_broken_json_names_the_file(tmp_path):
    with pytest.raises(f.FileProblem, match=r"sequence.json cannot be read"):
        f.load_run_inputs(_ship(tmp_path, sequence="{"), "p2")


# ── Arguments and environment ───────────────────────────────────────────────


@pytest.mark.parametrize("text,seconds", [("9h", 32400), ("90m", 5400), ("45s", 45), ("600", 600)])
def test_durations(text, seconds):
    assert cli.parse_duration(text) == seconds


@pytest.mark.parametrize(
    "argv",
    [
        ["--robot", "p2", "--mode", "loop", "--duration", "30s", "--yes"],
        ["--robot", "p2", "--mode", "loop", "--duration", "11h", "--yes"],
        [
            "--robot",
            "p2",
            "--mode",
            "loop",
            "--duration",
            "1h",
            "--hold-abort-after",
            "0.05",
            "--yes",
        ],
    ],
)
def test_out_of_range_arguments_exit_2(argv):
    assert cli.main(argv) == 2


@pytest.mark.parametrize(
    "argv",
    [
        ["--robot", "p2", "--mode", "once", "--duration", "1h"],
        ["--robot", "p3", "--mode", "loop"],
        # The run plays the shipped copy: there is no other file to point it at.
        ["--robot", "p2", "--mode", "loop", "--duration", "1h", "--logging-root", "/x"],
    ],
)
def test_once_and_unknown_robots_are_refused(argv):
    with pytest.raises(SystemExit) as exc:
        cli.main(argv)
    assert exc.value.code == 2


def _seq(*rows) -> dict:
    return {"steps": [{"preset": "open", "time_s": 1.0, **r} for r in rows], "tolerance_deg": 2.0}


def _parse(*rows):
    return f.parse_sequence(_seq(*rows), "/s.json")[0]


def test_a_switch_point_is_read_into_the_step():
    steps = _parse({"next_at_pct": 1}, {"next_at_pct": 99}, {})
    assert [s.next_at_pct for s in steps] == [1, 99, 100]


@pytest.mark.parametrize(
    ("value", "needle"),
    [
        (True, "whole number"),
        (50.5, "whole number"),
        (50.0, "whole number"),
        ("50", "whole number"),
        (None, "whole number"),
        (0, "outside"),
        (-3, "outside"),
        (101, "outside"),
        (100, "omit next_at_pct"),
    ],
)
def test_a_malformed_switch_point_is_refused(value, needle):
    with pytest.raises(f.FileProblem, match=needle):
        _parse({"next_at_pct": value}, {})


def test_a_sequence_without_a_settling_step_is_refused_on_every_path(tmp_path):
    rows = ({"next_at_pct": 50}, {"next_at_pct": 20})
    with pytest.raises(f.FileProblem, match="no 100 % step"):
        _parse(*rows)
    with pytest.raises(f.FileProblem, match="no 100 % step"):
        f.load_run_inputs(_ship(tmp_path, sequence=_seq(*rows)), "p2")


def test_a_shipped_wave_loads_into_resolved_steps(tmp_path):
    seq = _seq({"preset": "close", "next_at_pct": 40}, {"preset": "open"})
    inputs = f.load_run_inputs(_ship(tmp_path, sequence=seq), "p2")
    assert [s.next_at_pct for s in inputs.steps] == [40, 100]


def test_a_sequence_without_a_settling_step_exits_2(tmp_path, capsys, monkeypatch):
    seq = _seq({"preset": "close", "next_at_pct": 40})
    monkeypatch.setattr(cli, "shipped_config_dir", lambda: _ship(tmp_path, sequence=seq))
    monkeypatch.setattr(env, "logging_root", lambda: (str(tmp_path), ""))
    argv = ["--robot", "p2", "--mode", "loop", "--duration", "1h", "--yes"]
    assert cli.main(argv) == 2
    assert "no 100 % step" in capsys.readouterr().err


def test_a_missing_file_exits_2_before_ros(tmp_path, capsys, monkeypatch):
    monkeypatch.setattr(cli, "shipped_config_dir", lambda: _ship(tmp_path, sequence=MISSING))
    monkeypatch.setattr(env, "logging_root", lambda: (str(tmp_path), ""))
    argv = ["--robot", "p2", "--mode", "loop", "--duration", "1h", "--yes"]
    assert cli.main(argv) == 2
    err = capsys.readouterr().err
    assert "sequence.json does not exist" in err and "build.sh" in err


def test_without_a_terminal_start_must_be_given(capsys, monkeypatch):
    monkeypatch.setattr("sys.stdin.isatty", lambda: False)
    argv = ["--robot", "p2", "--mode", "loop", "--duration", "1h"]
    assert cli.main(argv) == 2
    assert "--yes" in capsys.readouterr().err


@pytest.mark.parametrize(
    "given,expected_rmw,uri_set",
    [
        ({}, env.CYCLONE, True),
        ({"RMW_IMPLEMENTATION": env.CYCLONE}, env.CYCLONE, True),
        ({"RMW_IMPLEMENTATION": env.CYCLONE, "CYCLONEDDS_URI": "file:///x"}, env.CYCLONE, False),
        ({"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp"}, "rmw_fastrtps_cpp", False),
    ],
)
def test_rmw_is_aligned_with_the_robot_launch(given, expected_rmw, uri_set):
    environ = dict(given)
    notes = env.align_rmw(environ, uri=lambda: "file:///stack.xml")
    assert environ["RMW_IMPLEMENTATION"] == expected_rmw
    assert (environ.get("CYCLONEDDS_URI") == "file:///stack.xml") is uri_set
    if expected_rmw != env.CYCLONE:
        assert any("WARNING" in n for n in notes)


def test_link_topics_match_the_shipped_driver_yaml():
    from integrated_bringup.demo_sequence.ros_io import LINK_TOPICS

    for robot, topic in LINK_TOPICS.items():
        doc = yaml.safe_load((CONFIG_DIR / robot / "udp_hand_node_p2.yaml").read_text())
        assert doc["/**"]["ros__parameters"]["link_status_topic"] == topic


def test_every_installed_entry_script_has_a_shebang():
    # CMake installs them with install(PROGRAMS): without the line `ros2 run`
    # fails with "Exec format error" (this CLI's first bring-up did).
    cmake = (CONFIG_DIR.parent / "CMakeLists.txt").read_text()
    scripts = sorted(set(re.findall(r"PROGRAMS (scripts/\S+\.py)", cmake)))
    assert "scripts/demo_controller_sequence.py" in scripts
    for script in scripts:
        first = (CONFIG_DIR.parent / script).read_text().splitlines()[0]
        assert first == "#!/usr/bin/env python3", script


# ── Readiness table ──────────────────────────────────────────────────────────

GOOD = Observation(
    now=0.0,
    cm_nodes=1,
    param_nodes=1,
    active_pubs=1,
    estop_pubs=1,
    active="hand_joint_controller",
    estop=False,
    joint_age=0.01,
    steady_s=5.0,
    roster_ok=True,
    positions=(0.0, 0.0),
    link_pubs=1,
    link=True,
    goal_pubs=1,
    goal_matched=1,
    mode="tracking",
    cap=10.0,
)


def test_a_complete_stack_is_ready():
    assert assess(GOOD, cm_seen=True).kind == READY
    assert runtime_abort(GOOD, link_required=True) == ""


@pytest.mark.parametrize(
    "change,needle",
    [
        (
            {
                "cm_nodes": 0,
                "param_nodes": 0,
                "active_pubs": 0,
                "estop_pubs": 0,
                "joint_age": None,
            },
            "controller manager",
        ),
        ({"joint_age": None}, "joint_states"),
        ({"steady_s": 1.0}, "steady"),
        ({"mode": None}, "parameters"),
        ({"cap": None}, "parameters"),
        ({"goal_matched": 0}, "subscriber"),
        ({"link": False}, "link"),
        ({"link": None}, "link"),
    ],
)
def test_waiting_rows_name_what_is_missing(change, needle):
    verdict = assess(replace(GOOD, **change), cm_seen=False)
    assert verdict.kind == WAIT
    assert needle in verdict.reason


@pytest.mark.parametrize(
    "change,cm_seen,needle",
    [
        ({"estop": True}, False, "E-STOP"),
        ({"other_hand": True}, False, "other hand"),
        ({"cm_nodes": 2}, False, "exactly one"),
        ({"param_nodes": 2}, False, "exactly one"),
        ({"estop_pubs": 2}, False, "exactly one"),
        ({"cm_nodes": 0}, True, "disappeared"),
        ({"active": "demo_joint_controller"}, False, "not hand_joint_controller"),
        ({"mode": "damping"}, False, "mode"),
        ({"goal_pubs": 2}, False, "another publisher"),
        ({"roster_ok": False}, False, "--robot"),
    ],
)
def test_refusing_rows(change, cm_seen, needle):
    verdict = assess(replace(GOOD, **change), cm_seen=cm_seen)
    assert verdict.kind == REFUSE
    assert needle in verdict.reason


def test_a_visible_controller_that_never_answers_points_at_the_rmw():
    assert "RMW_IMPLEMENTATION" in assess(replace(GOOD, mode=None), cm_seen=True).reason


def test_a_latched_value_without_its_publisher_is_no_evidence():
    stale = replace(GOOD, active_pubs=0, active="demo_joint_controller", estop_pubs=0, estop=True)
    assert assess(stale, cm_seen=False).kind == WAIT


@pytest.mark.parametrize(
    "change,link_required,needle",
    [
        ({"joint_age": 1.2}, True, "joint_states"),
        ({"active_pubs": 0}, True, "gone"),
        ({"goal_matched": 0}, True, "subscribes"),
        ({"link": False}, True, "link"),
        ({"link_pubs": 0}, True, "link"),
        ({"estop": True}, True, "E-STOP"),
        ({"mode": "damping"}, True, "mode"),
        ({"active": "demo_joint_controller"}, True, "active controller"),
        ({"goal_pubs": 2}, True, "another publisher"),
        ({"cm_nodes": 0}, True, "controller manager"),
        ({"estop_pubs": 0}, True, "E-STOP status has no publisher"),
    ],
)
def test_runtime_aborts(change, link_required, needle):
    assert needle in runtime_abort(replace(GOOD, **change), link_required)


def test_no_link_publisher_is_fine_where_none_was_required():
    assert runtime_abort(replace(GOOD, link_pubs=0, link=None), link_required=False) == ""


# ── Watches ──────────────────────────────────────────────────────────────────


def test_drift_threshold():
    assert drift_threshold_deg(2.0) == 7.0
    assert drift_threshold_deg(5.0) == 15.0


def test_a_short_drift_is_recorded_a_long_one_aborts():
    watch = DwellWatch(tolerance_deg=2.0, abort_after_s=1.0)
    big = math.radians(20.0)
    assert watch.feed(0.0, 1, "close", big, "j1") == ""
    assert watch.feed(0.5, 1, "close", big, "j1") == ""
    assert watch.feed(0.6, 1, "close", 0.0, "j1") == ""
    assert len(watch.records) == 1 and watch.records[0].joint == "j1"
    assert watch.feed(1.0, 1, "close", big, "j1") == ""
    assert "not holding" in watch.feed(2.1, 1, "close", big, "j1")


def test_disk_verdict():
    assert disk_verdict(10**10, 1e3, 3600) == ""
    assert "GB free" in disk_verdict(10**6, 1e5, 3600)


# ── The run (fake Io) ────────────────────────────────────────────────────────


class FakeIo:
    def __init__(self, obs: Observation) -> None:
        self.t = 0.0
        self.obs = obs
        self.lines: list[str] = []
        self.said: list[str] = []
        self.published: list[tuple] = []
        self.speed_sets: list[float] = []
        self.answer: bool | None = True  # how a speed set answers
        self.service_up = True
        self.dropped = 0
        self._interrupts = 0
        self._terminated = False
        self.summaries: list[dict] = []
        self.summary_fails = False
        self.free = 10**12
        self.bytes = 0

    def now(self):
        return self.t

    def wall(self):
        return f"t={self.t:.1f}"

    def observe(self):
        return replace(self.obs, now=self.t)

    def set_speed(self, speed):
        if not self.service_up:
            return None
        self.speed_sets.append(speed)
        return len(self.speed_sets)

    def speed_result(self, handle):
        return self.answer

    def drop_speed(self, handle):
        self.dropped += 1

    def publish(self, target):
        self.published.append(tuple(target))

    def interrupts(self):
        return self._interrupts

    def terminated(self):
        return self._terminated

    def read_line(self):
        return self.lines.pop(0) if self.lines else None

    def say(self, text):
        self.said.append(text)

    def environment(self):
        return "ROS_DOMAIN_ID=-"

    def log_bytes(self):
        return self.bytes

    def free_bytes(self):
        return self.free

    def write_summary(self, data):
        if self.summary_fails:
            raise OSError("disk full")
        self.summaries.append(data)


STEPS = (
    ResolvedStep("close", (0.5, 1.0), 1.0, 2.0),
    ResolvedStep("open", (0.0, 0.0), 1.0, 0.0),
)


def _cfg(**kw) -> RunConfig:
    inputs = f.RunInputs(STEPS, 2.0, "/p.json", "/s.json")
    base = {
        "robot": "p2",
        "joint_names": NAMES,
        "inputs": inputs,
        "duration_s": 3600.0,
        "assume_yes": True,
    }
    base.update(kw)
    return RunConfig(**base)


def _ticks(orch: Orchestrator, io: FakeIo, seconds: float, dt: float = 0.02, follow=True):
    """Advance; with ``follow`` the fake hand is at the last published target."""
    end = io.t + seconds
    while io.t < end and orch.outcome is None:
        orch.step()
        if follow and io.published:
            io.obs = replace(io.obs, positions=io.published[-1])
        io.t += dt
    return orch.outcome


def _running(io: FakeIo, **cfg) -> Orchestrator:
    orch = Orchestrator(io, _cfg(**cfg))
    while orch.state != RUNNING and orch.outcome is None and io.t < DISK_SAMPLE_S + 1.0:
        _ticks(orch, io, 0.02)
    assert orch.state == RUNNING, io.said
    return orch


def test_nothing_is_published_before_running():
    io = FakeIo(GOOD)
    orch = Orchestrator(io, _cfg(assume_yes=False))
    _ticks(orch, io, DISK_SAMPLE_S + 1.0)
    assert orch.state == CONFIRM and io.published == [] and io.speed_sets == []
    io.lines.append("yes\n")
    _ticks(orch, io, 0.1)
    assert orch.state == CONFIRM and io.published == []
    io.lines.append("start\n")
    _ticks(orch, io, 0.1)
    assert orch.state == RUNNING


def test_a_step_is_published_after_its_speed_set_answers():
    io = FakeIo(GOOD)
    io.answer = None
    orch = _running(io)
    _ticks(orch, io, 0.04)
    assert io.speed_sets and io.published == []
    io.answer = True
    _ticks(orch, io, 0.04)
    assert io.published == [STEPS[0].target_rad]


def test_a_refused_speed_set_sends_no_goal():
    io = FakeIo(GOOD)
    io.answer = False
    orch = _running(io)
    out = _ticks(orch, io, 0.1)
    assert out.code == 1 and "refused" in out.reason and io.published == []


def test_an_unanswered_speed_set_sends_no_goal_and_a_late_answer_is_ignored():
    io = FakeIo(GOOD)
    io.answer = None
    orch = _running(io)
    out = _ticks(orch, io, SPEED_TIMEOUT_S + 0.2)
    assert out.code == 1 and "unanswered" in out.reason and io.dropped == 1
    io.answer = True
    orch.step()
    assert io.published == []


def test_the_loop_runs_and_stops_at_the_end_of_a_cycle_when_the_duration_is_over():
    io = FakeIo(GOOD)
    orch = _running(io, duration_s=60.0)
    out = _ticks(orch, io, 200.0)
    assert out.code == 0
    cycles = orch.runner.cycle
    assert cycles >= 2
    # every cycle sent both steps, and nothing after the last cycle
    assert len(io.published) == 2 * cycles
    assert io.summaries[-1]["exit_code"] == 0
    assert io.summaries[-1]["steps"][0]["reached"] == cycles


def test_the_duration_expiring_in_a_dwell_finishes_that_cycle():
    io = FakeIo(GOOD)
    orch = _running(io, duration_s=60.0)
    # The duration counts from START (after the disk sample), not from t = 0.
    while orch.runner.state != "dwelling" or io.t < orch._clock.ends:
        _ticks(orch, io, 0.02)
    sent = len(io.published)
    out = _ticks(orch, io, 30.0)
    assert out.code == 0
    assert len(io.published) == sent + 1  # the cycle's second step, then nothing


def test_no_parameter_service_sends_no_goal_and_says_so():
    io = FakeIo(GOOD)
    io.service_up = False
    orch = _running(io)
    out = _ticks(orch, io, 0.1)
    assert out.code == 1 and "not available" in out.reason and io.published == []


def test_a_stop_names_its_cause():
    io = FakeIo(GOOD)
    orch = _running(io, duration_s=60.0)
    out = _ticks(orch, io, 200.0)
    assert out.code == 0 and "duration is over" in out.reason
    assert "stopped at the end of cycle" in out.reason


def test_a_disk_stop_at_a_cycle_end_is_not_a_success():
    io = FakeIo(GOOD)
    orch = _running(io, heartbeat_s=1.0)
    io.bytes, io.free = 10**9, 10**6  # the logs grew fast: the heartbeat's re-check fails
    out = _ticks(orch, io, 60.0)
    assert out.code == 1 and out.reason.startswith("disk:")
    assert io.summaries[-1]["end_reason"].startswith("disk:")


def test_a_flickering_stack_still_meets_the_startup_timeout():
    io = FakeIo(GOOD)
    orch = Orchestrator(io, _cfg(startup_timeout_s=3.0))
    while orch.outcome is None and io.t < 60.0:
        ready = int(io.t / 0.5) % 2 == 0  # ready half of every second
        io.obs = replace(GOOD, joint_age=0.01 if ready else None)
        orch.step()
        io.t += 0.02
    assert orch.outcome is not None and orch.outcome.code == 3, "it waited past the bound"
    assert io.t < 10.0 and io.published == []


def test_ctrl_c_once_ends_at_the_cycle_end_twice_at_once():
    io = FakeIo(GOOD)
    orch = _running(io)
    io._interrupts = 1
    _ticks(orch, io, 0.5)
    assert orch.outcome is None
    io._interrupts = 2
    out = _ticks(orch, io, 0.1)
    assert out.code == 130


def test_sighup_stops_at_once_with_a_summary():
    io = FakeIo(GOOD)
    orch = _running(io)
    io._terminated = True
    out = _ticks(orch, io, 0.1)
    assert out.code == 130 and io.summaries[-1]["exit_code"] == 130


def test_ctrl_c_at_the_prompt_sends_nothing():
    io = FakeIo(GOOD)
    orch = Orchestrator(io, _cfg(assume_yes=False))
    _ticks(orch, io, DISK_SAMPLE_S + 1.0)
    io._interrupts = 1
    out = _ticks(orch, io, 0.1)
    assert out.code == 130 and io.published == [] and io.summaries == []


@pytest.mark.parametrize(
    "change,needle",
    [
        ({"mode": "damping"}, "mode"),
        ({"goal_pubs": 2}, "another publisher"),
        ({"estop": True}, "E-STOP"),
        ({"link": False}, "link"),
    ],
)
def test_a_change_while_running_aborts_and_sends_nothing_more(change, needle):
    io = FakeIo(GOOD)
    orch = _running(io)
    _ticks(orch, io, 1.5)
    sent = len(io.published)
    io.obs = replace(io.obs, **change)
    out = _ticks(orch, io, 0.1)
    assert out.code == 1 and needle in out.reason and len(io.published) == sent


def test_joint_states_stopping_in_a_dwell_aborts():
    io = FakeIo(GOOD)
    orch = _running(io)
    while orch.runner.state != "dwelling":
        _ticks(orch, io, 0.02)
    io.obs = replace(io.obs, joint_age=1.1)
    out = _ticks(orch, io, 0.1, follow=False)
    assert out.code == 1 and "joint_states" in out.reason


def test_a_loop_pause_with_fresh_samples_does_not_abort():
    # The ROS side stamps samples in their callbacks and drains them before
    # judging: a 1.2 s stall of this process still sees fresh joint_states.
    io = FakeIo(GOOD)
    orch = _running(io)
    io.t += 1.2
    _ticks(orch, io, 0.1)
    assert orch.outcome is None


def test_a_long_dwell_drift_aborts_a_short_one_is_recorded():
    io = FakeIo(GOOD)
    orch = _running(io)
    while orch.runner.state != "dwelling":
        _ticks(orch, io, 0.02)
    pushed = (STEPS[0].target_rad[0], STEPS[0].target_rad[1] + math.radians(20))
    io.obs = replace(io.obs, positions=pushed)
    _ticks(orch, io, 0.5, follow=False)
    io.obs = replace(io.obs, positions=STEPS[0].target_rad)
    _ticks(orch, io, 0.1, follow=False)
    assert orch.outcome is None and len(orch.watch.records) == 1
    _ticks(orch, io, 10.0)  # next cycle's dwell
    while orch.runner.state != "dwelling":
        _ticks(orch, io, 0.02)
    io.obs = replace(io.obs, positions=pushed)
    out = _ticks(orch, io, 1.5, follow=False)
    assert out.code == 1 and "not holding" in out.reason


def test_a_summary_failure_mid_run_continues_the_final_one_exits_4():
    io = FakeIo(GOOD)
    orch = _running(io, heartbeat_s=1.0)
    io.summary_fails = True
    _ticks(orch, io, 3.0)
    assert orch.outcome is None and any("summary not written" in s for s in io.said)
    io._interrupts = 2
    out = _ticks(orch, io, 0.1)
    assert out.code == 4


def test_not_ready_until_the_deadline_exits_3_with_the_environment():
    io = FakeIo(replace(GOOD, cm_nodes=0, joint_age=None))
    out = _ticks(Orchestrator(io, _cfg(startup_timeout_s=3.0)), io, 4.0)
    assert out.code == 3 and "ROS_DOMAIN_ID" in out.reason and io.published == []


def test_a_cm_that_appears_then_vanishes_exits_3_at_once():
    io = FakeIo(replace(GOOD, joint_age=None))
    orch = Orchestrator(io, _cfg())
    _ticks(orch, io, 0.5)
    io.obs = replace(io.obs, cm_nodes=0)
    out = _ticks(orch, io, 0.1)
    assert out.code == 3 and "disappeared" in out.reason


def test_a_run_that_does_not_fit_the_disk_is_refused():
    io = FakeIo(GOOD)
    io.free = 10**9
    orch = Orchestrator(io, _cfg(duration_s=36000.0))

    def grow():
        io.bytes += 100_000  # 5 MB/s at dt 0.02

    while orch.outcome is None and io.t < 10:
        grow()
        _ticks(orch, io, 0.02)
    assert orch.outcome.code == 3 and "GB free" in orch.outcome.reason


def test_link_was_absent_at_start_so_it_is_not_required():
    io = FakeIo(replace(GOOD, link_pubs=0, link=None))
    orch = _running(io)
    out = _ticks(orch, io, 5.0)
    assert out is None and io.published


# ── SequenceRunner: the cycle-end stop the CLI relies on ─────────────────────


def test_runner_stop_at_cycle_end_finishes_the_pass():
    runner = SequenceRunner()
    assert runner.start(STEPS, "loop", 2.0, NAMES) == ""
    t = 0.0
    sent = []
    pos = (0.0, 0.0)
    runner.stop_at_cycle_end()
    while runner.running and t < 30:
        step = runner.tick(t, pos, 0.0)
        if step is not None:
            sent.append(step.preset)
            runner.on_sent(runner.token, t)
            pos = step.target_rad
        t += 0.05
    assert sent == ["close", "open"] and not runner.aborted and runner.cycle == 1


# ── The shipped copy is what plays (field PCs) ──────────────────────────────


def _working_copy(root: Path, presets: object, sequence: object) -> None:
    """The GUI's working copy in a logging root — what the run used to read."""
    (root / "hand_presets_p2.json").write_text(json.dumps(presets))
    (root / "demo_gui_sequences_p2.json").write_text(json.dumps(sequence))


@pytest.mark.parametrize("working", ["missing", "other"])
def test_the_run_plays_the_shipped_copy_whatever_the_working_copy_holds(
    tmp_path, monkeypatch, working
):
    """The field-PC contract: pull + build + this command plays what was
    committed. A PC's stale or edited GUI working copy changes nothing."""
    from integrated_bringup.demo_gui import shipped as s
    from integrated_bringup.demo_sequence import ros_io

    root = tmp_path / "logging_data"
    root.mkdir()
    if working == "other":
        _working_copy(root, {"open": {**_OPEN, "positions_deg": [0.0] * 10}}, SEQUENCE)
    config = _ship(tmp_path)
    monkeypatch.setattr(cli, "shipped_config_dir", lambda: config)
    monkeypatch.setattr(env, "logging_root", lambda: (str(root), ""))
    seen = {}

    def run(args, profile, inputs, log_root):
        seen.update(inputs=inputs, root=log_root)
        return 0

    monkeypatch.setattr(ros_io, "run", run)
    argv = ["--robot", "p2", "--mode", "loop", "--duration", "1h", "--yes"]
    assert cli.main(argv) == 0
    inputs = seen["inputs"]
    assert [x.preset for x in inputs.steps] == ["close", "open"]
    assert inputs.steps[0].target_rad == pytest.approx(
        tuple(math.radians(v) for v in _CLOSE["positions_deg"])
    )
    assert inputs.posture_code == s.posture_code(PRESETS, SEQUENCE)
    assert inputs.preset_path == str(Path(config, "p2", "demo_gui", "hand_presets.json").resolve())
    assert inputs.calibration is not None and seen["root"] == str(root)


def test_the_installed_share_is_followed_to_the_files_it_links(tmp_path):
    """A symlink install links each file: the run names what it read."""
    real = Path(_ship(tmp_path / "src"))
    config = tmp_path / "share" / "config"
    for path in sorted(real.rglob("*")):
        target = config / path.relative_to(real)
        if path.is_dir():
            target.mkdir(parents=True)
        else:
            target.symlink_to(path)
    inputs = f.load_run_inputs(str(config), "p2")
    assert inputs.sequence_path == str(real / "p2" / "demo_gui" / "sequence.json")


def _git(cwd: Path, *args: str) -> str:
    import subprocess

    return subprocess.run(
        ["git", "-c", "user.name=t", "-c", "user.email=t@t", *args],
        cwd=cwd,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def test_the_commit_line_names_the_last_commit_and_local_changes(tmp_path):
    repo = tmp_path / "repo"
    config = Path(_ship(repo))
    _git(repo, "init", "-q")
    _git(repo, "add", ".")
    _git(repo, "commit", "-q", "-m", "postures")
    head = _git(repo, "log", "-1", "--format=%h %cs")
    paths = tuple(
        str(config / "p2" / "demo_gui" / n) for n in ("hand_presets.json", "sequence.json")
    )
    assert f.provenance(paths) == head
    Path(paths[1]).write_text(json.dumps({**SEQUENCE, "tolerance_deg": 3.0}))
    assert f.provenance(paths) == f"{head} + local changes"
    # A shipped file git does not track yet (promote has just written it).
    _git(repo, "rm", "-q", "--cached", paths[1])
    _git(repo, "commit", "-q", "-m", "untrack")
    assert f.provenance(paths).endswith(" + local changes")


def test_no_git_no_commit_line(tmp_path):
    config = Path(_ship(tmp_path))
    paths = tuple(
        str(config / "p2" / "demo_gui" / n) for n in ("hand_presets.json", "sequence.json")
    )
    assert f.provenance(paths) == ""


# ── The running driver's calibration ────────────────────────────────────────


def _calibration(offsets=(0.0, 5.0), names=NAMES):
    from integrated_bringup.demo_gui.shipped import Calibration

    return Calibration("drv.yaml", names, (1.0, 1.0), tuple(offsets))


RECORDED = _calibration()


def test_a_driver_on_the_recorded_calibration_is_ready():
    obs = replace(GOOD, driver_nodes=1, driver_calibration=_calibration())
    assert assess(obs, True, RECORDED).kind == READY


def test_a_driver_on_another_calibration_refuses_and_says_restart():
    obs = replace(GOOD, driver_nodes=1, driver_calibration=_calibration((0.0, -5.0)))
    verdict = assess(obs, True, RECORDED)
    assert verdict.kind == REFUSE
    assert "j1 joint_position_offsets_deg -5 (recorded 5)" in verdict.reason
    assert "restart the bring-up" in verdict.reason


def test_a_driver_not_read_yet_is_waited_for():
    verdict = assess(replace(GOOD, driver_nodes=1), True, RECORDED)
    assert verdict.kind == WAIT and "hand driver calibration" in verdict.reason


def test_a_driver_whose_parameters_are_no_calibration_refuses():
    obs = replace(GOOD, driver_nodes=1, driver_problem="/udp_hand_node has no joint_state_names")
    verdict = assess(obs, True, RECORDED)
    assert verdict.kind == REFUSE and "cannot be read" in verdict.reason


@pytest.mark.parametrize(
    "obs,recorded",
    [
        # A sim: no driver node and no link_status — nothing to compare.
        (replace(GOOD, link_pubs=0, link=None), RECORDED),
        # No record ships beside the postures: not checked.
        (replace(GOOD, driver_nodes=1), None),
    ],
)
def test_the_calibration_is_not_checked_without_a_driver_or_a_record(obs, recorded):
    assert assess(obs, True, recorded).kind == READY


def test_the_link_alone_marks_a_driver_whose_node_is_not_seen_yet():
    """link_status is the driver's topic: no READY without its calibration."""
    assert assess(replace(GOOD, driver_nodes=0), True, RECORDED).kind == WAIT


def test_a_calibration_mismatch_exits_3_before_anything_is_sent():
    obs = replace(GOOD, driver_nodes=1, driver_calibration=_calibration((1.0, 5.0)))
    io = FakeIo(obs)
    inputs = replace(_cfg().inputs, calibration=RECORDED)
    out = _ticks(Orchestrator(io, _cfg(inputs=inputs)), io, 1.0)
    assert out.code == 3 and "j0 joint_position_offsets_deg" in out.reason
    assert io.published == []


# ── What the plan screen and the summary name ───────────────────────────────


def _plan_and_summary(**inputs):
    io = FakeIo(replace(GOOD, driver_nodes=1, driver_calibration=_calibration()))
    run_inputs = replace(_cfg().inputs, calibration=RECORDED, **inputs)
    orch = _running(io, inputs=run_inputs)
    plan = next(t for t in io.said if t.startswith("robot p2:"))
    return plan, orch.summary()


def test_the_plan_and_the_summary_name_the_postures():
    plan, summary = _plan_and_summary(posture_code="1a2b3c4d", provenance="0123abc 2026-11-25")
    assert "posture code 1a2b3c4d" in plan
    assert "commit   0123abc 2026-11-25" in plan
    assert "presets  /p.json" in plan and "sequence /s.json" in plan
    assert "calibration the running driver's = the recorded one" in plan
    assert summary["posture_code"] == "1a2b3c4d"
    assert summary["commit"] == "0123abc 2026-11-25"
    assert (summary["preset_file"], summary["sequence_file"]) == ("/p.json", "/s.json")


def test_no_commit_line_when_git_cannot_tell():
    plan, summary = _plan_and_summary(posture_code="1a2b3c4d", provenance="")
    assert "commit" not in plan and summary["commit"] == ""


def test_a_missing_calibration_record_refuses_with_the_build_hint(tmp_path):
    config = Path(_ship(tmp_path))
    (config / "p2" / "demo_gui" / "calibration.yaml").unlink()
    with pytest.raises(f.FileProblem, match=r"calibration.yaml does not exist .*build.sh"):
        f.load_run_inputs(str(config), "p2")


def test_the_wrong_hand_on_the_domain_is_named_before_its_calibration():
    """Another hand's stack also runs another calibration: its own message
    (the domain) is the one that says what to fix."""
    obs = replace(
        GOOD, driver_nodes=1, driver_calibration=_calibration((9.0, 9.0)), other_hand=True
    )
    verdict = assess(obs, True, RECORDED)
    assert verdict.kind == REFUSE and "ROS_DOMAIN_ID" in verdict.reason


def test_two_hand_drivers_on_the_domain_refuse():
    obs = replace(GOOD, driver_nodes=2, driver_calibration=_calibration())
    verdict = assess(obs, True, RECORDED)
    assert verdict.kind == REFUSE and "2 hand drivers" in verdict.reason


# ── A wave: steps that hand over before they arrive ─────────────────────────

WAVE = (
    ResolvedStep("close", (0.5, 1.0), 1.0, 0.0, 50),
    ResolvedStep("open", (0.0, 0.0), 1.0, 0.0),
)


def _wave_run(io: FakeIo) -> Orchestrator:
    inputs = f.RunInputs(WAVE, 2.0, "/p.json", "/s.json")
    return _running(io, inputs=inputs)


def test_a_switched_step_is_not_reached_and_records_where_the_hand_was():
    io = FakeIo(GOOD)  # the hand rests at (0, 0): far from the first target
    orch = _wave_run(io)
    _ticks(orch, io, 3.0, follow=False)
    first = orch.summary()["steps"][0]
    assert first["switched"] >= 1 and first["reached"] == 0
    assert first["reach_s_mean"] is None
    assert first["switch_error_deg_max"] > 30.0  # ~57 deg off: no arrival was needed
    second = orch.summary()["steps"][1]
    assert second["reached"] >= 1 and second["switched"] == 0
    # The first goal went out, then the second at 50 % of 1 s, not after 1 s.
    assert io.published[:2] == [WAVE[0].target_rad, WAVE[1].target_rad]


def test_the_lag_is_the_publish_after_the_switch_due_time():
    io = FakeIo(GOOD)
    orch = _wave_run(io)
    while not io.published:
        _ticks(orch, io, 0.02, follow=False)
    io.answer = None  # the next speed set takes a while to answer
    while orch.runner.index != 1:
        _ticks(orch, io, 0.02, follow=False)
    _ticks(orch, io, 0.3, follow=False)
    io.answer = True
    _ticks(orch, io, 0.1, follow=False)
    step = orch.summary()["steps"][0]
    assert len(io.published) >= 2 and step["switched"] == 1
    assert 0.28 <= step["lag_s_mean"] <= 0.45 and step["lag_s_max"] == pytest.approx(
        step["lag_s_mean"], abs=1e-3
    )


def test_an_arrival_only_run_keeps_the_old_summary_values():
    io = FakeIo(GOOD)
    orch = _running(io)
    _ticks(orch, io, 6.0)
    for row in orch.summary()["steps"]:
        assert row["switched"] == 0 and row["lag_s_mean"] is None and row["reached"] >= 1


def test_the_plan_shows_the_switch_point_only_where_it_is_set():
    io = FakeIo(replace(GOOD, driver_nodes=1, driver_calibration=_calibration()))
    inputs = f.RunInputs(WAVE, 2.0, "/p.json", "/s.json", calibration=RECORDED)
    _running(io, inputs=inputs)
    lines = next(t for t in io.said if t.startswith("robot p2:")).splitlines()
    assert "next at  50 %" in next(x for x in lines if " 1. close" in x)
    assert "next at" not in next(x for x in lines if " 2. open" in x)
