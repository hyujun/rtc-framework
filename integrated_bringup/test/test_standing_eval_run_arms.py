"""tools/standing_eval/run_arms.py (not installed — read from the source tree).

The launch arguments each arm gets: without ``--mpc-threads`` they are the ones
the tool always passed; with it, every arm launched under the MPC layout also
names the simulation worker profile and the solver's thread count. The two
event arms (reentry, estop): the order of their service calls, how each reply
is read, which reply ends the run, and the windows cut from the recording.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

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
STAMP = "sec: 12\nnanosec: 500000000\n---\n"


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
    }
    base.update(over)
    return type("Args", (), base)()


class FakeShell:
    """`sh` as the scripts see it: canned replies per service, the calls kept."""

    def __init__(self, replies: dict[str, str], active: list[str]):
        self.replies = replies
        self.active = active  # what active_controller_name answers, call after call
        self.calls: list[list[str]] = []

    def __call__(self, cmd, timeout=30.0):
        self.calls.append(cmd)
        if cmd[:3] == ["ros2", "service", "call"]:
            out = self.replies.get(cmd[3], "")
        elif cmd[:3] == ["ros2", "topic", "echo"] and "/rtc_cm/active_controller_name" in cmd:
            out = self.active.pop(0) if self.active else ""
        elif cmd[:3] == ["ros2", "topic", "echo"]:
            out = STAMP
        else:
            out = ""
        return type("Done", (), {"stdout": out, "stderr": "", "returncode": 0})()


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


def test_the_reentry_script_switches_away_and_back_between_two_windows(monkeypatch):
    shell = FakeShell(
        {"/rtc_cm/switch_controller": SWITCH_OK},
        active=["demo_joint_controller", "locomotion_controller"],
    )
    monkeypatch.setattr(run_arms, "sh", shell)
    monkeypatch.setattr(run_arms.time, "sleep", lambda s: None)
    note: dict = {}
    clock = FakeClock()
    assert run_arms.run_events("reentry", _args(), note, clock, t0=100.0)
    assert _services(shell) == ["/rtc_cm/switch_controller"] * 2
    requests = [c[5] for c in shell.calls if c[:3] == ["ros2", "service", "call"]]
    assert "demo_joint_controller" in requests[0]
    assert "locomotion_controller" in requests[1]
    names = [e["name"] for e in note["events"]]
    assert names == [
        "window_a_start",
        "window_a_end",
        "switch_joint",
        "switch_loco",
        "window_b_start",
        "window_b_end",
    ]
    assert clock.sleeps == [10.0, 2.0, 1.0, 10.0]
    assert run_arms.script_duration("reentry", _args()) == 23.0


def test_the_reentry_script_stops_when_the_switch_back_is_refused(monkeypatch):
    shell = FakeShell({"/rtc_cm/switch_controller": SWITCH_REFUSED}, active=[])
    monkeypatch.setattr(run_arms, "sh", shell)
    monkeypatch.setattr(run_arms.time, "sleep", lambda s: None)
    note: dict = {}
    assert not run_arms.run_events("reentry", _args(), note, FakeClock(), t0=100.0)
    assert "switch to the joint controller" in note["error"]
    assert [e["name"] for e in note["events"]] == [
        "window_a_start",
        "window_a_end",
        "switch_joint",
    ]


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
