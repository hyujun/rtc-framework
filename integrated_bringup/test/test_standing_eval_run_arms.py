"""tools/standing_eval/run_arms.py (not installed — read from the source tree).

The launch arguments each arm gets: without ``--mpc-threads`` they are the ones
the tool always passed; with it, every arm launched under the MPC layout also
names the simulation worker profile and the solver's thread count. The two
event arms (reentry, estop): the order of their service calls, how each reply
is read, which reply ends the run, and the windows cut from the recording.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest
import yaml

_PATH = Path(__file__).resolve().parents[1] / "tools" / "standing_eval" / "run_arms.py"
_SPEC = importlib.util.spec_from_file_location("standing_eval_run_arms", _PATH)
run_arms = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(run_arms)

POOL = ["layout_profile:=mpc_par_sim", "mpc_threads:=2"]


@pytest.mark.parametrize(
    ("arm", "mpc_layout", "expected"),
    [
        ("servo", False, []),
        ("lowpd", False, []),
        ("loco", False, []),
        ("mpc", False, ["enable_mpc:=true"]),
        ("mpc_fb", False, ["enable_mpc:=true", "sim_overlay:=mpc_riccati"]),
        ("servo", True, ["enable_mpc:=true"]),
        ("loco", True, ["enable_mpc:=true", "sim_overlay:=mpc_off"]),
        ("mpc", True, ["enable_mpc:=true"]),
        # the event arms stand on the latched pose like loco
        ("reentry", False, []),
        ("estop", False, []),
        ("reentry", True, ["enable_mpc:=true", "sim_overlay:=mpc_off"]),
        ("estop", True, ["enable_mpc:=true", "sim_overlay:=mpc_off"]),
        # the walk arms walk on the MPC's plan, with or without the layout flag
        ("walk", False, ["enable_mpc:=true"]),
        ("walk", True, ["enable_mpc:=true"]),
        ("walk_estop", False, ["enable_mpc:=true"]),
        ("walk_estop", True, ["enable_mpc:=true"]),
    ],
)
def test_without_threads_the_arguments_are_unchanged(arm, mpc_layout, expected):
    assert run_arms.launch_args(arm, mpc_layout, {}) == expected
    assert run_arms.launch_args(arm, mpc_layout, {}, None) == expected


@pytest.mark.parametrize(
    ("arm", "mpc_layout", "expected"),
    [
        ("servo", False, []),  # no MPC layout: nothing to seat
        ("loco", False, []),
        ("mpc", False, ["enable_mpc:=true", *POOL]),
        ("mpc_fb", False, ["enable_mpc:=true", *POOL, "sim_overlay:=mpc_riccati"]),
        # one CPU layout for every arm of an --mpc-layout evaluation
        ("servo", True, ["enable_mpc:=true", *POOL]),
        ("loco", True, ["enable_mpc:=true", *POOL, "sim_overlay:=mpc_off"]),
        ("walk", False, ["enable_mpc:=true", *POOL]),
        ("walk_estop", True, ["enable_mpc:=true", *POOL]),
    ],
)
def test_threads_add_the_worker_profile_under_the_mpc_layout(arm, mpc_layout, expected):
    assert run_arms.launch_args(arm, mpc_layout, {}, 2) == expected


def test_an_arm_overlay_still_replaces_its_own():
    assert run_arms.launch_args("mpc_fb", False, {"mpc_fb": "/x.yaml"}, 3) == [
        "enable_mpc:=true",
        "layout_profile:=mpc_par_sim",
        "mpc_threads:=3",
        "sim_overlay:=/x.yaml",
    ]


def test_the_inekf_source_puts_its_overlay_on_every_arm():
    overlays = run_arms.source_overlays(["mpc", "walk"], False, {}, "inekf")
    assert overlays == {"mpc": "estimator_inekf", "walk": "estimator_inekf"}
    assert run_arms.launch_args("walk", False, overlays) == [
        "enable_mpc:=true",
        "sim_overlay:=estimator_inekf",
    ]
    assert run_arms.source_overlays(["mpc", "walk"], False, {"x": "y"}, "truth") == {"x": "y"}


@pytest.mark.parametrize(
    ("arm", "mpc_layout", "own"),
    [
        ("mpc_fb", False, "mpc_riccati"),  # its own overlay
        ("loco", True, "mpc_off"),  # mpc_off under the MPC layout
    ],
)
def test_the_inekf_source_merges_into_an_arm_that_has_an_overlay(tmp_path, arm, mpc_layout, own):
    overlays = run_arms.source_overlays([arm], mpc_layout, {}, "inekf", tmp_path, "g1_loco")
    merged = Path(overlays[arm])
    assert merged == tmp_path / f"{arm}_inekf.yaml"
    params = yaml.safe_load(merged.read_text())["integrated_rt_controller"]["ros__parameters"]
    loco = params["locomotion_controller"]
    assert loco["estimator"]["source"] == "inekf"  # the estimator overlay's
    assert (
        loco["mpc"]["riccati"]["enabled"] is True
        if own == "mpc_riccati"
        else (loco["mpc"]["enabled"] is False)
    )  # the arm's own
    meta = json.loads(merged.with_suffix(".meta.json").read_text())
    assert [Path(c["path"]).stem for c in meta["components"]] == [own, "estimator_inekf"]
    assert run_arms.launch_args(arm, mpc_layout, overlays)[-1] == f"sim_overlay:={merged}"


def test_the_inekf_source_merges_a_given_overlay_and_refuses_a_conflict(tmp_path):
    given = tmp_path / "cand.yaml"
    given.write_text(
        "integrated_rt_controller:\n  ros__parameters:\n    locomotion_controller:\n"
        "      mpc:\n        max_policy_age_s: 0.35\n"
    )
    overlays = run_arms.source_overlays(
        ["walk"], False, {"walk": str(given)}, "inekf", tmp_path / "o", "g1_loco"
    )
    loco = yaml.safe_load(Path(overlays["walk"]).read_text())["integrated_rt_controller"][
        "ros__parameters"
    ]["locomotion_controller"]
    assert loco["mpc"]["max_policy_age_s"] == 0.35 and loco["estimator"]["source"] == "inekf"
    # A candidate that sets the source itself to something else cannot be merged.
    given.write_text(
        "integrated_rt_controller:\n  ros__parameters:\n    locomotion_controller:\n"
        "      estimator:\n        source: truth\n"
    )
    with pytest.raises(ValueError, match="estimator"):
        run_arms.source_overlays(
            ["walk"], False, {"walk": str(given)}, "inekf", tmp_path / "o", "g1_loco"
        )
    # A name that resolves to nothing is refused, not skipped.
    with pytest.raises(ValueError, match="no_such_overlay"):
        run_arms.source_overlays(
            ["walk"], False, {"walk": "no_such_overlay"}, "inekf", tmp_path / "o", "g1_loco"
        )


DECLARED = "    'mpc_threads':\n        (default: '')\n"


def _seat_one_worker(value: str, profile: str) -> int:
    """mpc_solver_threads on a host whose profile seats one worker (6 cores)."""
    if int(value) < 1:
        raise RuntimeError(f"mpc_threads {value} is below 1")
    if int(value) > 2:
        raise RuntimeError(f"mpc_threads {value} needs {int(value) - 1} MPC worker slot(s)")
    return int(value)


def _main(threads: str, shown: str, out: Path, monkeypatch) -> int:
    """main() up to its first run (-k 0), the host and the launch's arguments stubbed."""
    monkeypatch.setattr(run_arms, "mpc_solver_threads", _seat_one_worker)
    monkeypatch.setattr(
        run_arms, "sh", lambda cmd, timeout=30.0: type("Shown", (), {"stdout": shown})()
    )
    argv = ["--out", str(out), "--runtime-yaml", "x", "--logging-root", "y", "-k", "0"]
    return run_arms.main([*argv, "--mpc-threads", threads])


@pytest.mark.parametrize(
    ("threads", "shown", "message"),
    [
        ("0", DECLARED, "is below 1"),
        ("3", DECLARED, "needs 2 MPC worker slot"),
        ("2", "    'enable_mpc':\n", "declares no mpc_threads argument"),
    ],
)
def test_an_unusable_thread_count_is_refused_before_the_first_run(
    threads, shown, message, monkeypatch, capsys, tmp_path
):
    with pytest.raises(SystemExit):
        _main(threads, shown, tmp_path, monkeypatch)
    assert message in capsys.readouterr().err


def test_a_seated_thread_count_on_a_launch_that_takes_it_passes(monkeypatch, tmp_path):
    assert _main("2", DECLARED, tmp_path, monkeypatch) == 0


def test_the_solver_thread_count_is_read_off_the_controller_log(tmp_path):
    log = tmp_path / "launch.log"
    log.write_text("[INFO] boot\n")
    assert run_arms.solver_threads(log) is None
    log.write_text(
        "[locomotion] MPC thread started: core=3 prio=60 11.43 Hz, 1 solver thread(s)\n"
        "[locomotion] MPC thread started: core=3 prio=60 11.43 Hz, 2 solver thread(s)\n"
    )
    assert run_arms.solver_threads(log) == 2


# ── The event arms ─────────────────────────────────────────────────────────

SWITCH_OK = "response:\nrtc_msgs.srv.SwitchController_Response(ok=True, message='switched')\n"
SWITCH_REFUSED = "response:\nrtc_msgs.srv.SwitchController_Response(ok=False, message='no')\n"
TRIGGER_OK = "response:\nstd_srvs.srv.Trigger_Response(success=True, message='raised')\n"
TRIGGER_NO = "response:\nstd_srvs.srv.Trigger_Response(success=False, message='?')\n"
CLEAR_OK = (
    'response:\nrtc_msgs.srv.ClearEstop_Response(ok=True, message="post-clear switch to '
    "'DemoJointController': ok; E-STOP cleared (operator_request)\")\n"
)
CLEAR_NO_SWITCH = "response:\nrtc_msgs.srv.ClearEstop_Response(ok=True, message='cleared')\n"
RESET_OK = "response:\nstd_srvs.srv.Trigger_Response(success=True, message='reset')\n"
RESET_NO = "response:\nstd_srvs.srv.Trigger_Response(success=False, message='?')\n"
RUNTIME_YAML = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "g1_loco"
    / "controllers"
    / "locomotion"
    / "runtime.yaml"
)


class FakeClock:
    def __init__(self):
        self.t = 100.0
        self.sleeps: list[float] = []

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.t += seconds

    def now(self):
        return self.t

    def epoch(self):
        return 1000.0 + self.t


def _args(**over):
    base = {
        "rtf": None,
        "duration": 10.0,
        "settle": 2.0,
        "lead": 1.0,
        "hold_lead": 2.0,
        "hold": 10.0,
        "sensor_topic": "/g1/sim_sensors",
        "joint_controller": "demo_joint_controller",
        "joint_controller_class": "DemoJointController",
        "z_ref": 0.76,
        "upright_wait": 4.0,
        "body_group": "g1",
        "runtime_yaml": RUNTIME_YAML,
        "cmd_topic": "/locomotion_controller/g1/cmd_vel",
        "stop_window": 3.0,
        "estop_at": 8.0,
        "run_out": None,
        "twist_proc": None,
        "script": "walk",
    }
    base.update(over)
    return type("Args", (), base)()


class FakeShell:
    """`sh` as the scripts see it: canned replies per service, the calls kept."""

    def __init__(self, replies: dict[str, str], active: list[str], pub_rc: int = 0):
        self.replies = replies
        self.active = active  # what active_controller_name answers, call after call
        self.pub_rc = pub_rc  # what `ros2 topic pub` exits with
        self.calls: list[list[str]] = []

    def __call__(self, cmd, timeout=30.0):
        self.calls.append(cmd)
        rc = 0
        if cmd[:3] == ["ros2", "service", "call"]:
            out = self.replies.get(cmd[3], "")
        elif cmd[:3] == ["ros2", "topic", "echo"] and "/rtc_cm/active_controller_name" in cmd:
            out = self.active.pop(0) if self.active else ""
        elif cmd[:3] == ["ros2", "topic", "pub"]:
            out, rc = "", self.pub_rc
        else:
            out = ""
        return type("Done", (), {"stdout": out, "stderr": "", "returncode": rc})()


def _services(shell: FakeShell) -> list[str]:
    return [c[3] for c in shell.calls if c[:3] == ["ros2", "service", "call"]]


def test_the_estop_script_calls_trigger_clear_switch_in_order_and_marks_each(monkeypatch):
    shell = FakeShell(
        {
            "/rtc_cm/trigger_estop": TRIGGER_OK,
            "/rtc_cm/clear_estop": CLEAR_OK,
            "/rtc_cm/switch_controller": SWITCH_OK,
        },
        active=["demo_joint_controller", "locomotion_controller"],
    )
    monkeypatch.setattr(run_arms, "sh", shell)
    monkeypatch.setattr(run_arms.time, "sleep", lambda s: None)
    note: dict = {}
    clock = FakeClock()
    assert run_arms.run_events("estop", _args(), note, clock, t0=100.0)
    assert _services(shell) == [
        "/rtc_cm/trigger_estop",
        "/rtc_cm/clear_estop",
        "/rtc_cm/switch_controller",
    ]
    names = [e["name"] for e in note["events"]]
    assert names == ["trigger_estop", "clear_estop", "switch_loco", "window_start", "window_end"]
    by = {e["name"]: e for e in note["events"]}
    assert by["clear_estop"]["post_clear_switch_ok"] is True
    assert by["clear_estop"]["active_controller"] == "demo_joint_controller"
    assert by["switch_loco"]["ok"] is True
    assert by["switch_loco"]["active_controller"] == "locomotion_controller"
    # one active_controller_name read per switch, one after the clear
    assert sum("/rtc_cm/active_controller_name" in c for c in shell.calls) == 2
    # the wall-clock stamp of each mark, on the recorder's axis
    assert [e["sim_stamp"] - e["wall_s"] for e in note["events"]] == [1100.0] * 5
    assert by["trigger_estop"]["wall_s"] == 2.0
    assert by["window_start"]["wall_s"] == 2.0 + 10.0 + 2.0 + 1.0
    assert by["window_end"]["wall_s"] == by["window_start"]["wall_s"] + 10.0
    assert "error" not in note
    assert run_arms.script_duration("estop", _args()) == 25.0


def test_the_estop_script_scales_its_waits_by_the_rtf(monkeypatch):
    shell = FakeShell(
        {
            "/rtc_cm/trigger_estop": TRIGGER_OK,
            "/rtc_cm/clear_estop": CLEAR_OK,
            "/rtc_cm/switch_controller": SWITCH_OK,
        },
        active=["demo_joint_controller", "locomotion_controller"],
    )
    monkeypatch.setattr(run_arms, "sh", shell)
    monkeypatch.setattr(run_arms.time, "sleep", lambda s: None)
    clock = FakeClock()
    assert run_arms.run_events("estop", _args(rtf=0.5), {}, clock, t0=100.0)
    assert clock.sleeps == [4.0, 20.0, 4.0, 2.0, 20.0]


@pytest.mark.parametrize(
    ("replies", "active", "error"),
    [
        ({"/rtc_cm/trigger_estop": TRIGGER_NO}, [], "did not report success"),
        (
            {"/rtc_cm/trigger_estop": TRIGGER_OK, "/rtc_cm/clear_estop": CLEAR_NO_SWITCH},
            ["demo_joint_controller"],
            "post-clear switch",
        ),
        (
            {"/rtc_cm/trigger_estop": TRIGGER_OK, "/rtc_cm/clear_estop": CLEAR_OK},
            ["locomotion_controller"],
            "active controller is 'locomotion_controller'",
        ),
        (
            {
                "/rtc_cm/trigger_estop": TRIGGER_OK,
                "/rtc_cm/clear_estop": CLEAR_OK,
                "/rtc_cm/switch_controller": SWITCH_REFUSED,
            },
            ["demo_joint_controller"],
            "switch to the locomotion controller failed",
        ),
    ],
)
def test_the_estop_script_stops_at_the_first_unexpected_reply(replies, active, error, monkeypatch):
    shell = FakeShell(replies, active=list(active))
    monkeypatch.setattr(run_arms, "sh", shell)
    monkeypatch.setattr(run_arms.time, "sleep", lambda s: None)
    note: dict = {}
    assert not run_arms.run_events("estop", _args(), note, FakeClock(), t0=100.0)
    assert error in note["error"]
    assert not any(e["name"].startswith("window") for e in note["events"])


REENTRY_REPLIES = {
    "/rtc_cm/trigger_estop": TRIGGER_OK,
    "/rtc_cm/clear_estop": CLEAR_OK,
    "/sim/reset": RESET_OK,
    "/rtc_cm/switch_controller": SWITCH_OK,
}


def test_the_reentry_script_stops_stands_the_robot_up_and_switches_back(monkeypatch):
    shell = FakeShell(REENTRY_REPLIES, active=["demo_joint_controller", "locomotion_controller"])
    monkeypatch.setattr(run_arms, "sh", shell)
    monkeypatch.setattr(run_arms.time, "sleep", lambda s: None)
    note: dict = {}
    clock = FakeClock()
    assert run_arms.run_events("reentry", _args(), note, clock, t0=100.0)
    assert _services(shell) == [
        "/rtc_cm/trigger_estop",
        "/rtc_cm/clear_estop",
        "/sim/reset",
        "/rtc_cm/switch_controller",
    ]
    pub = next(c for c in shell.calls if c[:3] == ["ros2", "topic", "pub"])
    assert pub[-3:-1] == ["/demo_joint_controller/g1/joint_goal", "rtc_msgs/msg/RobotTarget"]
    assert "-w" in pub and "--once" in pub
    assert "goal_type: joint" in pub[-1]
    assert "left_hip_pitch_joint" in pub[-1] and "-0.3" in pub[-1]
    assert [e["name"] for e in note["events"]] == [
        "window_a_start",
        "window_a_end",
        "trigger_estop",
        "clear_estop",
        "stand_goal",
        "sim_reset",
        "switch_loco",
        "window_b_start",
        "window_b_end",
    ]
    assert clock.sleeps == [10.0, 10.0, 4.0, 2.0, 1.0, 10.0]
    assert run_arms.script_duration("reentry", _args()) == 37.0


@pytest.mark.parametrize(
    ("replies", "pub_rc", "error", "last"),
    [
        (REENTRY_REPLIES, 1, "stand goal was not published", "stand_goal"),
        (
            {**REENTRY_REPLIES, "/sim/reset": RESET_NO},
            0,
            "/sim/reset did not report success",
            "sim_reset",
        ),
        (
            {**REENTRY_REPLIES, "/rtc_cm/clear_estop": CLEAR_NO_SWITCH},
            0,
            "post-clear switch",
            "clear_estop",
        ),
    ],
)
def test_the_reentry_script_stops_at_the_step_that_failed(
    replies, pub_rc, error, last, monkeypatch
):
    shell = FakeShell(replies, active=["demo_joint_controller"], pub_rc=pub_rc)
    monkeypatch.setattr(run_arms, "sh", shell)
    monkeypatch.setattr(run_arms.time, "sleep", lambda s: None)
    note: dict = {}
    assert not run_arms.run_events("reentry", _args(), note, FakeClock(), t0=100.0)
    assert error in note["error"]
    assert note["events"][-1]["name"] == last
    assert "/rtc_cm/switch_controller" not in _services(shell)


def test_the_stand_goal_is_the_shipped_stand_pose_on_the_body_roster():
    names, pose = run_arms.stand_goal(RUNTIME_YAML, "demo_joint_controller", "g1")
    assert len(names) == len(pose) == 23
    assert names[0] == "left_hip_pitch_joint"
    assert pose[:5] == [-0.3, 0.0, 0.0, 0.6, -0.3]


def test_the_stand_goal_refuses_a_roster_it_cannot_match(tmp_path):
    profile = tmp_path / "p"
    (profile / "controllers" / "locomotion").mkdir(parents=True)
    runtime = profile / "controllers" / "locomotion" / "runtime.yaml"
    runtime.write_text("{}\n")
    (profile / "controllers" / "j.yaml").write_text(
        "j: {estop: {arm_safe_position: [0.1, 0.2]}}\n"
    )
    (profile / "_base.yaml").write_text("x: {devices: {g1: {joint_state_names: [a, b, c]}}}\n")
    with pytest.raises(ValueError, match="2 stand-pose values for 3 joints"):
        run_arms.stand_goal(runtime, "j", "g1")
    with pytest.raises(ValueError, match="no devices.arm"):
        run_arms.stand_goal(runtime, "j", "arm")


@pytest.mark.parametrize(
    ("reply", "cls", "expected"),
    [
        ("post-clear switch to 'DemoJointController': ok; cleared", "DemoJointController", True),
        ("post-clear switch to 'DemoJointController': ok; cleared", None, True),
        ("post-clear switch to 'OtherController': ok; cleared", "DemoJointController", False),
        (
            "post-clear switch to 'DemoJointController': already active; x",
            "DemoJointController",
            False,
        ),
        ("post-clear switch to 'DemoJointController': FAILED (x) — E-STOP latched", None, False),
        ("E-STOP cleared (operator_request)", None, False),
    ],
)
def test_the_clear_reply_is_read_for_the_post_clear_switch(reply, cls, expected):
    assert run_arms.post_clear_switch_ok(reply, cls) is expected


def test_the_windows_are_cut_from_the_recording_between_their_marks(tmp_path):
    from rtc_tools.analysis import stance_metrics as sm

    samples = [
        sm.Sample(
            t=float(i) * 0.1,
            pos=(0.0, 0.0, 0.76 - (0.4 if 2.0 <= i * 0.1 <= 3.0 else 0.0)),
            quat=(1.0, 0.0, 0.0, 0.0),
            feet=((0.0, 0.1, 0.0), (0.0, -0.1, 0.0)),
            loads=(200.0, 200.0),
        )
        for i in range(60)
    ]
    csv = tmp_path / "stance.csv"
    sm.write_csv(csv, samples, 2)
    events = [
        {"name": "window_a_start", "sim_stamp": 0.0},
        {"name": "window_a_end", "sim_stamp": 1.5},
        {"name": "switch_joint", "sim_stamp": 1.6},
        {"name": "window_b_start", "sim_stamp": 2.0},
        {"name": "window_b_end", "sim_stamp": 3.0},
        {"name": "window_c_start", "sim_stamp": None},
        {"name": "window_c_end", "sim_stamp": 5.0},
    ]
    w = run_arms.window_metrics(csv, events, z_ref=0.76)
    assert set(w) == {"window_a", "window_b", "window_c"}
    assert w["window_a"]["fallen"] is False
    assert w["window_a"]["num_samples"] == 16
    assert w["window_b"]["fallen"] is True
    assert "error" in w["window_c"]
    assert run_arms.window_metrics(tmp_path / "missing.csv", events, 0.76) == {}


# ── The walk arms ──────────────────────────────────────────────────────────

# A stand-in for twist_script.py: its start line at once, then it rests until
# its own end (`--rtf` read as the seconds it lasts) or SIGINT, exit 0.
FAKE_TWIST = """
import json, signal, sys, time
signal.signal(signal.SIGINT, lambda *_: (print(json.dumps({"event": "end"}), flush=True), sys.exit(0)))
print(json.dumps({"event": "start", "wall": 1.5, "argv": sys.argv[1:]}), flush=True)
time.sleep(float(sys.argv[sys.argv.index("--rtf") + 1]))
print(json.dumps({"event": "end"}), flush=True)
"""


def _fake_twist(tmp_path, monkeypatch) -> None:
    script = tmp_path / "fake_twist.py"
    script.write_text(FAKE_TWIST)
    monkeypatch.setattr(run_arms, "TWIST_SCRIPT", script)


def test_the_walk_script_starts_the_command_marks_the_stop_window_and_waits_for_its_end(
    monkeypatch, tmp_path
):
    _fake_twist(tmp_path, monkeypatch)
    args = _args(run_out=tmp_path, rtf=0.1)  # the fake lasts 0.1 s
    note: dict = {}
    clock = FakeClock()
    assert run_arms.run_events("walk", args, note, clock, t0=100.0)
    assert [e["name"] for e in note["events"]] == [
        "twist_start",
        "stop_start",
        "stop_end",
        "twist_end",
    ]
    by = {e["name"]: e for e in note["events"]}
    assert by["twist_start"]["script_wall"] == 1.5
    assert by["twist_end"]["rc"] == 0
    # 26 s of script: the stop window is its last 3, all scaled by 1 / rtf
    assert clock.sleeps == pytest.approx([230.0, 30.0])
    assert run_arms.script_duration("walk", args) == pytest.approx(26.0)
    assert args.twist_proc is None
    started = run_arms.read_events(tmp_path / "twist.log")[0]
    assert started["argv"][:2] == ["--topic", "/locomotion_controller/g1/cmd_vel"]
    assert started["argv"][-2:] == ["--script", "walk"]


@pytest.mark.parametrize("script", ["turn", "curve", "turn_slow", "curve_slow"])
def test_the_walk_script_passes_its_command_script_and_keeps_the_run_length(
    script, monkeypatch, tmp_path
):
    _fake_twist(tmp_path, monkeypatch)
    args = _args(run_out=tmp_path, rtf=0.1, script=script)
    clock = FakeClock()
    assert run_arms.run_events("walk", args, {}, clock, t0=100.0)
    assert run_arms.read_events(tmp_path / "twist.log")[0]["argv"][-2:] == ["--script", script]
    assert clock.sleeps == pytest.approx([230.0, 30.0])
    assert run_arms.script_duration("walk", args) == pytest.approx(26.0)


def test_the_walk_estop_script_stops_the_command_after_the_hold(monkeypatch, tmp_path):
    _fake_twist(tmp_path, monkeypatch)
    shell = FakeShell({"/rtc_cm/trigger_estop": TRIGGER_OK}, active=[])
    monkeypatch.setattr(run_arms, "sh", shell)
    args = _args(run_out=tmp_path, rtf=100.0)  # the fake would rest 100 s: stopped instead
    note: dict = {}
    clock = FakeClock()
    assert run_arms.run_events("walk_estop", args, note, clock, t0=100.0)
    assert [e["name"] for e in note["events"]] == [
        "twist_start",
        "trigger_estop",
        "hold_start",
        "hold_end",
        "twist_stop",
    ]
    assert _services(shell) == ["/rtc_cm/trigger_estop"]
    assert clock.sleeps == pytest.approx([8.0 / 100.0, 10.0 / 100.0])
    assert run_arms.script_duration("walk_estop", args) == pytest.approx(18.0)
    assert args.twist_proc is None


def test_a_command_script_that_never_starts_ends_the_walk(monkeypatch, tmp_path):
    script = tmp_path / "no_start.py"
    script.write_text("import sys; sys.exit(3)\n")
    monkeypatch.setattr(run_arms, "TWIST_SCRIPT", script)
    note: dict = {}
    assert not run_arms.run_events("walk", _args(run_out=tmp_path), note, FakeClock(), t0=100.0)
    assert note["error"] == "the command script did not start"
    assert note["events"][0]["rc"] == 3


def test_a_process_deaf_to_sigint_is_killed():
    proc = run_arms.subprocess.Popen(
        [
            run_arms.sys.executable,
            "-c",
            "import signal, time; signal.signal(signal.SIGINT, signal.SIG_IGN); "
            "print('ready', flush=True); time.sleep(30)",
        ],
        stdout=run_arms.subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    assert proc.stdout.readline().strip() == "ready"  # its handler is in place
    assert run_arms.stop_group(proc, timeout=0.5) == -9
    proc.stdout.close()
    assert run_arms.stop_group(None) is None


# ── provenance ────────────────────────────────────────────────────────────


def _tree(root, files):
    for rel, text in files.items():
        (root / rel).parent.mkdir(parents=True, exist_ok=True)
        (root / rel).write_text(text)


def test_an_install_that_matches_the_checkout_has_the_same_config_hash(tmp_path):
    files = {"p/_base.yaml": "a: 1\n", "p/controllers/x.yaml": "b: 2\n"}
    _tree(tmp_path / "src", files)
    _tree(tmp_path / "inst", {**files, "p/left_by_an_older_build.yaml": "c: 3\n"})
    h = run_arms.config_hashes("p", tmp_path / "inst", tmp_path / "src")
    assert h["files"] == 2 and h["source_sha256"] is not None
    assert h["installed_sha256"] == h["source_sha256"]  # the leftover file is not the checkout's


def test_a_stale_installed_value_changes_the_config_hash(tmp_path):
    _tree(tmp_path / "src", {"p/_base.yaml": "a: 1\n", "p/controllers/x.yaml": "b: 2\n"})
    _tree(tmp_path / "inst", {"p/_base.yaml": "a: 1\n", "p/controllers/x.yaml": "b: 3\n"})
    h = run_arms.config_hashes("p", tmp_path / "inst", tmp_path / "src")
    assert h["installed_sha256"] not in (None, h["source_sha256"])


def test_a_file_missing_from_the_install_leaves_no_installed_hash(tmp_path):
    _tree(tmp_path / "src", {"p/_base.yaml": "a: 1\n", "p/new.yaml": "b: 2\n"})
    _tree(tmp_path / "inst", {"p/_base.yaml": "a: 1\n"})
    assert (
        run_arms.config_hashes("p", tmp_path / "inst", tmp_path / "src")["installed_sha256"]
        is None
    )


def test_the_config_hash_tells_a_moved_value_from_the_same_bytes(tmp_path):
    _tree(tmp_path / "a", {"p/x.yaml": "1", "p/y.yaml": "2"})
    _tree(tmp_path / "b", {"p/x.yaml": "2", "p/y.yaml": "1"})
    files = [Path("x.yaml"), Path("y.yaml")]
    assert run_arms.tree_sha256(tmp_path / "a" / "p", files) != run_arms.tree_sha256(
        tmp_path / "b" / "p", files
    )


def test_the_overlay_record_carries_the_hash_and_the_generators_note(tmp_path):
    overlay = tmp_path / "cand.yaml"
    overlay.write_text("/**:\n  ros__parameters: {}\n")
    assert run_arms.overlay_record(["enable_mpc:=true"], "g1_loco") is None
    plain = run_arms.overlay_record([f"sim_overlay:={overlay}"], "g1_loco")
    assert plain == {"path": str(overlay), "sha256": run_arms.file_sha256(overlay)}
    (tmp_path / "cand.meta.json").write_text('{"theta": {"state": 0.5}}')
    noted = run_arms.overlay_record([f"sim_overlay:={overlay}"], "g1_loco")
    assert noted["meta"] == {"theta": {"state": 0.5}}
    overlay.write_text("/**:\n  ros__parameters: {a: 1}\n")
    assert (
        run_arms.overlay_record([f"sim_overlay:={overlay}"], "g1_loco")["sha256"]
        != plain["sha256"]
    )


def test_the_urdf_is_found_in_the_parameter_tree():
    tree = {"/**": {"ros__parameters": {"x": 1, "urdf": {"package": "p", "path": "a/b.urdf"}}}}
    assert run_arms.find_urdf(tree) == {"package": "p", "path": "a/b.urdf"}
    assert run_arms.find_urdf({"urdf": "a string"}) is None


def test_a_run_whose_plant_is_not_in_the_log_has_no_model_match(tmp_path):
    base = tmp_path / "_base.yaml"
    base.write_text("urdf: {package: robot_descriptions, path: nope.urdf}\n")
    assert "error" in run_arms.model_record(base, "no plant here")
    unreadable = run_arms.model_record(base, "[MuJoCoSimulator] Loaded '/nope.xml'  nq=1")
    assert "error" in unreadable and "mass_kg" not in unreadable


def test_the_checkout_state_is_a_commit_and_a_flag():
    state = run_arms.git_state()
    assert len(state["commit"]) == 40 and isinstance(state["dirty"], bool)
    assert run_arms.git_state(Path("/"))["commit"] is None


# ── scenarios ─────────────────────────────────────────────────────────────


def test_an_arm_named_once_keeps_its_overlay_as_given(tmp_path):
    out = run_arms.arm_overlays(["walk"], ["walk=mpc_riccati"], None, tmp_path, "g1_loco")
    assert out == {"walk": "mpc_riccati"}
    assert not list(tmp_path.iterdir())  # nothing merged, nothing written
    with pytest.raises(ValueError, match="ARM=NAME"):
        run_arms.arm_overlays(["walk"], ["walk"], None, tmp_path, "g1_loco")


def test_several_overlays_of_one_arm_are_merged(tmp_path):
    cand = tmp_path / "cand.yaml"
    cand.write_text(
        "integrated_rt_controller:\n  ros__parameters:\n    locomotion_controller:\n"
        "      mpc:\n        max_policy_age_s: 0.35\n"
    )
    out = run_arms.arm_overlays(
        ["walk"], [f"walk={cand}", "walk=mpc_riccati"], None, tmp_path / "o", "g1_loco"
    )
    merged = yaml.safe_load(Path(out["walk"]).read_text())
    mpc = merged["integrated_rt_controller"]["ros__parameters"]["locomotion_controller"]["mpc"]
    assert mpc == {"max_policy_age_s": 0.35, "riccati": {"enabled": True}}
    cand.write_text(
        "integrated_rt_controller:\n  ros__parameters:\n    locomotion_controller:\n"
        "      mpc:\n        riccati:\n          enabled: false\n"
    )
    with pytest.raises(ValueError, match="riccati"):
        run_arms.arm_overlays(
            ["walk"], [f"walk={cand}", "walk=mpc_riccati"], None, tmp_path / "o", "g1_loco"
        )


def test_friction_is_a_piece_of_every_arms_overlay(tmp_path):
    out = run_arms.arm_overlays(["mpc", "walk"], ["walk=mpc_riccati"], 0.4, tmp_path, "g1_loco")
    alone = yaml.safe_load(Path(out["mpc"]).read_text())  # the friction piece by itself
    override = alone["mujoco_simulator"]["ros__parameters"]["solver"]["contact_override"]
    assert override == {"enable": True, "o_friction": [0.4, 0.4, 0.005, 0.0001, 0.0001]}
    both = yaml.safe_load(Path(out["walk"]).read_text())
    assert both["mujoco_simulator"]["ros__parameters"]["solver"]["contact_override"] == override
    assert "integrated_rt_controller" in both
    with pytest.raises(ValueError, match="friction"):
        run_arms.arm_overlays(["walk"], [], 0.0, tmp_path, "g1_loco")


def test_the_push_reaches_the_command_script_as_given():
    import argparse

    none = argparse.Namespace(push=None, push_body=None)
    assert run_arms.push_arguments(none) == []
    args = argparse.Namespace(push=[6.0, 0.2, 40.0, 0.0, 0.0], push_body="pelvis")
    assert run_arms.push_arguments(args) == [
        "--push", "6.0", "0.2", "40.0", "0.0", "0.0", "--push-body", "pelvis",
    ]  # fmt: skip


@pytest.mark.parametrize(
    "extra",
    [
        ["--push", "6", "0.2", "40", "0", "0"],  # no body
        ["--push", "15.9", "0.2", "40", "0", "0", "--push-body", "pelvis"],  # runs into the stop
        ["--push", "6", "0", "40", "0", "0", "--push-body", "pelvis"],  # no duration
        ["--friction", "-1"],
    ],
)
def test_a_scenario_that_cannot_run_is_refused_before_the_first_run(tmp_path, extra):
    base = ["--out", str(tmp_path), "--arms", "walk", "--logging-root", str(tmp_path)]
    with pytest.raises(SystemExit):
        run_arms.main(base + extra)
    assert not (tmp_path / "walk_0").exists()


# ── --mpc-sync (lock-step) ─────────────────────────────────────────────────


@pytest.mark.parametrize(
    ("arm", "mpc_layout", "expected"),
    [
        ("servo", False, []),  # no MPC: nothing to sync
        ("loco", True, ["enable_mpc:=true", "mpc_sync:=lockstep", "sim_overlay:=mpc_off"]),
        ("mpc", False, ["enable_mpc:=true", "mpc_sync:=lockstep"]),
        ("walk", False, ["enable_mpc:=true", "mpc_sync:=lockstep"]),
    ],
)
def test_mpc_sync_goes_to_the_arms_launched_with_the_mpc(arm, mpc_layout, expected):
    assert run_arms.launch_args(arm, mpc_layout, {}, None, "lockstep") == expected
    # unset: the arguments the tool always passed
    assert "mpc_sync:=lockstep" not in run_arms.launch_args(arm, mpc_layout, {})


def test_mpc_sync_follows_the_thread_pool_arguments():
    got = run_arms.launch_args("mpc", True, {}, 2, "free_run")
    assert got == ["enable_mpc:=true", *POOL, "mpc_sync:=free_run"]


def test_lockstep_is_the_default_and_needs_no_mpc_arm(monkeypatch, tmp_path):
    # the sim launch defaults to lockstep; the tools follow it for every arm
    # (sim time is the right axis with or without the MPC), free_run opts out
    seen = {}
    monkeypatch.setattr(
        run_arms, "run_once", lambda arm, k, args: seen.update(sync=args.mpc_sync) or {}
    )
    argv = [
        "--out",
        str(tmp_path),
        "--runtime-yaml",
        "x",
        "--logging-root",
        "y",
        "-k",
        "1",
        "--arms",
        "servo",
    ]
    run_arms.main(argv)
    assert seen["sync"] == "lockstep"
    run_arms.main([*argv, "--mpc-sync", "free_run"])
    assert seen["sync"] == "free_run"


class _FakeSim:
    """A sim time that advances `step` per poll on a fake wall clock `dt` per read."""

    def __init__(self, step, wall_dt=0.1, first=100.0):
        self.sim = None if step is None else first
        self.step = step
        self.wall = 0.0
        self.wall_dt = wall_dt

    def poll(self, timeout_s):
        if self.step is not None:
            self.sim += self.step
        return self.sim

    def clock(self):
        self.wall += self.wall_dt
        return self.wall


def test_sim_clock_sleeps_until_the_sim_time_has_advanced():
    fake = _FakeSim(step=0.5)
    clock = run_arms.SimClock(poll=fake.poll, wall=fake.clock)
    clock.sleep(2.0)
    # the start is the first reading (100.5): returns at the first poll ≥ 2.0 s later
    assert fake.sim == pytest.approx(102.5)


def test_sim_clock_survives_a_sim_reset_during_the_sleep():
    # 100.5, 101.0, then /sim/reset: 0.0, 0.5, 1.0, 1.5 — only the forward steps count
    readings = iter([100.5, 101.0, 0.0, 0.5, 1.0, 1.5, 2.0])
    fake = _FakeSim(step=0.0)
    clock = run_arms.SimClock(poll=lambda _t: next(readings), wall=fake.clock)
    clock.sleep(2.0)
    assert next(readings) == 2.0  # returned at 1.5: 0.5 (pre-reset) + 1.5 (post-reset)


def test_sim_clock_raises_when_the_sim_does_not_advance_within_the_wall_bound():
    fake = _FakeSim(step=0.0)  # a stalled sim
    clock = run_arms.SimClock(poll=fake.poll, wall=fake.clock)
    with pytest.raises(run_arms.SimClockTimeout, match="advanced 0.000 s of the 1.000 s"):
        clock.sleep(1.0)
    assert fake.wall > 10.0 * 1.0 + 5.0  # not before 10 × 1 + 5 wall s
    dead = _FakeSim(step=None)  # no message at all
    with pytest.raises(run_arms.SimClockTimeout):
        run_arms.SimClock(poll=dead.poll, wall=dead.clock).sleep(1.0)


def test_lockstep_events_sleep_sim_seconds_whatever_the_rtf(monkeypatch):
    shell = FakeShell(
        {
            "/rtc_cm/trigger_estop": TRIGGER_OK,
            "/rtc_cm/clear_estop": CLEAR_OK,
            "/rtc_cm/switch_controller": SWITCH_OK,
        },
        active=["demo_joint_controller", "locomotion_controller"],
    )
    monkeypatch.setattr(run_arms, "sh", shell)
    monkeypatch.setattr(run_arms.time, "sleep", lambda s: None)
    clock = FakeClock()
    assert run_arms.run_events("estop", _args(rtf=0.5, mpc_sync="lockstep"), {}, clock, t0=100.0)
    assert clock.sleeps == [2.0, 10.0, 2.0, 1.0, 10.0]  # not divided by the rtf


def test_the_twist_script_gets_the_sim_clock_under_lockstep(monkeypatch, tmp_path):
    seen = []

    class Proc:
        def poll(self):
            return None

    monkeypatch.setattr(run_arms.subprocess, "Popen", lambda cmd, **kw: seen.append(cmd) or Proc())
    monkeypatch.setattr(run_arms, "read_events", lambda path: [{"event": "start", "wall": 1.0}])
    for sync, flags in (("free_run", ["--rtf", "1.0"]), ("lockstep", ["--clock", "sim"])):
        args = _args(mpc_sync=sync, run_out=tmp_path, push=None, push_body=None)
        assert run_arms._twist_start(args)[0]
        cmd = seen[-1]
        assert flags[0] in cmd and cmd[cmd.index(flags[0]) + 1] == flags[1]
        assert ("--rtf" in cmd) == (sync == "free_run")
