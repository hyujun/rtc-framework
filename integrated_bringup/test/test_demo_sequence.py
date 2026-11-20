"""demo_controller_sequence without ROS: the files, the readiness table, the run.

The orchestrator is driven through a fake ``Io`` whose observation a test sets
tick by tick, so every row of the operator document's table and every way a
run ends is pinned here; ``test_demo_sequence_ros`` covers the rclpy wiring.
"""

from __future__ import annotations

import json
import math
import re
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


def _write(root: Path, presets: object, sequence: object, token: str = "p2") -> None:
    (root / f"hand_presets_{token}.json").write_text(json.dumps(presets))
    (root / f"demo_gui_sequences_{token}.json").write_text(json.dumps(sequence))


PRESETS = {
    "open": {"positions_deg": [0.0, 0.0], "side": "right"},
    "close": {"positions_deg": [30.0, 60.0]},
}
SEQUENCE = {
    "steps": [
        {"preset": "close", "time_s": 1.0, "dwell_s": 2.0},
        {"preset": "open", "time_s": 1.0, "dwell_s": 0.0},
    ],
    "tolerance_deg": 2.0,
}


def test_inputs_resolve_and_list_the_untagged_presets(tmp_path):
    _write(tmp_path, PRESETS, SEQUENCE)
    inputs = f.load_run_inputs(str(tmp_path), "p2", "right", 2)
    assert [s.preset for s in inputs.steps] == ["close", "open"]
    assert inputs.steps[0].target_rad == pytest.approx((math.radians(30), math.radians(60)))
    assert inputs.untagged == ("close",)
    assert inputs.tolerance_deg == 2.0


def test_inputs_write_nothing(tmp_path):
    _write(tmp_path, PRESETS, SEQUENCE)
    before = sorted(p.name for p in tmp_path.iterdir())
    f.load_run_inputs(str(tmp_path), "p2", "right", 2)
    assert sorted(p.name for p in tmp_path.iterdir()) == before


@pytest.mark.parametrize(
    "presets,sequence,needle",
    [
        (None, SEQUENCE, "does not exist"),
        ({"open": {"positions_deg": [0, 0], "side": "left"}}, SEQUENCE, "another hand"),
        (PRESETS, {"steps": []}, "no steps"),
        (PRESETS, {"steps": [{"preset": "open", "time_s": 0.0}]}, "time_s"),
        (PRESETS, {"steps": [{"preset": "open", "time_s": 601.0}]}, "time_s"),
        (PRESETS, {"steps": [{"preset": "open", "time_s": 1.0, "dwell_s": -1}]}, "dwell_s"),
        (PRESETS, {"steps": [{"preset": "open", "time_s": True}]}, "time_s"),
        (PRESETS, {"steps": ["open"]}, "not an object"),
        (PRESETS, {"steps": [{"time_s": 1.0}]}, "names no preset"),
        (PRESETS, {"steps": [{"preset": "gone", "time_s": 1.0}]}, "no longer exists"),
        (PRESETS, {**SEQUENCE, "tolerance_deg": 40.0}, "tolerance_deg"),
        ({"open": {"positions_deg": [0.0]}}, {"steps": [{"preset": "open", "time_s": 1}]}, "DoF"),
    ],
)
def test_inputs_refuse_what_the_gui_would_clamp_or_drop(tmp_path, presets, sequence, needle):
    if presets is None:
        (tmp_path / "demo_gui_sequences_p2.json").write_text(json.dumps(sequence))
    else:
        _write(tmp_path, presets, sequence)
    with pytest.raises(f.FileProblem, match=needle):
        f.load_run_inputs(str(tmp_path), "p2", "right", 2)


def test_a_broken_json_names_the_file(tmp_path):
    _write(tmp_path, PRESETS, SEQUENCE)
    (tmp_path / "demo_gui_sequences_p2.json").write_text("{")
    with pytest.raises(f.FileProblem, match="demo_gui_sequences_p2.json is not JSON"):
        f.load_run_inputs(str(tmp_path), "p2", "right", 2)


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
        ["--robot", "p2", "--mode", "loop", "--duration", "1h", "--logging-root", "rel", "--yes"],
    ],
)
def test_out_of_range_arguments_exit_2(argv):
    assert cli.main(argv) == 2


@pytest.mark.parametrize(
    "argv",
    [["--robot", "p2", "--mode", "once", "--duration", "1h"], ["--robot", "p3", "--mode", "loop"]],
)
def test_once_and_unknown_robots_are_refused(argv):
    with pytest.raises(SystemExit) as exc:
        cli.main(argv)
    assert exc.value.code == 2


def test_a_missing_file_exits_2_before_ros(tmp_path, capsys):
    argv = ["--robot", "p2", "--mode", "loop", "--duration", "1h", "--yes"]
    assert cli.main([*argv, "--logging-root", str(tmp_path)]) == 2
    assert "does not exist" in capsys.readouterr().err


def test_without_a_terminal_start_must_be_given(tmp_path, capsys, monkeypatch):
    monkeypatch.setattr("sys.stdin.isatty", lambda: False)
    argv = ["--robot", "p2", "--mode", "loop", "--duration", "1h"]
    assert cli.main([*argv, "--logging-root", str(tmp_path)]) == 2
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
    inputs = f.RunInputs(STEPS, 2.0, "/p.json", "/s.json", ())
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
