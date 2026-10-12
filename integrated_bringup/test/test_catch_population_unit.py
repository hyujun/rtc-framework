"""catch_population_trials (#821): one unit from bringup to status, over stand-in processes.

``test_catch_population_trials.py`` holds what the command decides; this file runs
what it DOES — the processes it starts, in their own groups, and what it leaves in
the unit directory — without a sim. A stand-in ``ros2`` on the PATH plays the four
things a unit starts (the sim's launch, the estimator's launch, the controller
switch, the trial driver): each prints what the real one prints and leaves what it
leaves, as the scenario says. The controller's mirror, the one thing a unit asks
over ROS itself, is planted. The unit's waits run fifty times faster.

What a real sim adds — that the launch arguments reach the controller, that the
real processes stop on SIGINT, the real session — is checked in real runs.
"""

import os
import signal
import stat
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
from test_catch_population_trials import GRID, NLP, _unit_state

from integrated_bringup import catch_population_trials as cpt

# What `ros2 <verb> <what>` does in a unit. The scenario is in FAKE_* variables.
STUB = r"""#!/bin/bash
hold() { trap 'exit 0' INT TERM; while true; do sleep 0.02; "$@"; done; }
case "$1 $2" in
  "launch integrated_bringup")
    echo "ros2 $*"
    log=$FAKE_SESSION/controllers/demo_catching_controller
    mkdir -p "$log" "$FAKE_SESSION/sim"
    printf 't,mode\n0.0,1\n' > "$log/catching_diag.csv"
    [ -n "$FAKE_FOREIGN" ] && echo x > "$log/$FAKE_FOREIGN"
    echo "[integrated_rt_controller-2] [INFO] [1.0] [integrated_rt_controller]: Session dir: $FAKE_SESSION (max_sessions=60)"
    cat "$FAKE_LAUNCH_LOG"
    later() {
      [ -f "$FAKE_DIR/key" ] && { cat "$FAKE_DIR/key"; rm "$FAKE_DIR/key"; }
      [ -f "$FAKE_DIR/sim_leaves" ] && exit 0
    }
    hold later ;;
  "pkg prefix") echo "$FAKE_PREFIX" ;;
  "launch ball_perception_sim")
    echo "ros2 $*"
    [ "$FAKE_ESTIMATOR" == "silent" ] || echo "[sim_estimator]: activated epoch activation-1"
    hold true ;;
  "service call")
    echo "response:"
    echo "rtc_msgs.srv.SwitchController_Response(ok=${FAKE_SWITCH:-True}, message='switched')" ;;
  "run integrated_bringup")
    echo "ros2 $*" > "$FAKE_DIR/driver_command"
    mkdir -p "$4"; echo '[]' > "$4/trial_results.json"
    cat "$FAKE_TRIALS_LOG"
    n=$(cat "$FAKE_DIR/driver_runs" 2>/dev/null || echo 0); echo $((n + 1)) > "$FAKE_DIR/driver_runs"
    rc=$(echo "${FAKE_DRIVER_RC:-0}" | cut -d, -f$((n + 1))); rc=${rc:-0}
    [ "$rc" == "hang" ] && hold true
    if [ -n "$FAKE_KEY_AT_END" ]; then  # returns once the sim has logged the key
      echo "$FAKE_KEY_AT_END" > "$FAKE_DIR/key"
      while [ -f "$FAKE_DIR/key" ]; do sleep 0.02; done
    fi
    exit "$rc" ;;
  *) echo "the stand-in ros2 has no '$*'" >&2; exit 99 ;;
esac
"""

SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
FLAP = ["TRACKING", "ARMED"] * 80 + ["TRACKING"]
CYCLE = ["TRACKING", "APPROACH", "COMMITTED", "CLOSING", "DECEL", "HOLD", "RETREAT", "ARMED"]


def _throws(*mode_lists):
    return "".join(
        f"[INFO] [1.0] [catching_sim_trials]: trial {i}: modes={modes!r} outcome=None "
        "final_mode=TRACKING\n"
        for i, modes in enumerate(mode_lists)
    )


class Rig:
    """A result folder, a stand-in ``ros2`` and the context of one unit."""

    def __init__(self, tmp_path, monkeypatch, arm):
        self.tmp, self.env, self.arm = tmp_path, monkeypatch, arm
        bin_dir = tmp_path / "bin"
        bin_dir.mkdir()
        stub = bin_dir / "ros2"
        stub.write_text(STUB)
        stub.chmod(stub.stat().st_mode | stat.S_IXUSR)
        monkeypatch.setenv("PATH", f"{bin_dir}{os.pathsep}{os.environ['PATH']}")
        self.fake = tmp_path / "fake"
        self.fake.mkdir()
        self.ball_ws = tmp_path / "ball_ws"
        (self.ball_ws / "install").mkdir(parents=True)
        (self.ball_ws / "install" / "local_setup.bash").write_text("")
        self.profile = tmp_path / "profile.json"
        self.profile.write_text("{}")
        self.logging_root = tmp_path / "logging_data"
        self.session = self.logging_root / "261011_2332"
        self.expect, self.mirror, log = _unit_state("ur5e_p1b", arm)
        (self.fake / "launch.log").write_text(log)
        (self.fake / "trials.log").write_text(_throws(*[CYCLE] * 6))
        self.set(
            FAKE_DIR=self.fake,
            FAKE_SESSION=self.session,
            FAKE_LAUNCH_LOG=self.fake / "launch.log",
            FAKE_TRIALS_LOG=self.fake / "trials.log",
            FAKE_PREFIX=self.ball_ws / "install" / "ball_perception_sim",
        )
        self.chunk = tmp_path / arm.name / "throws" / "chunk_000.json"
        self.chunk.parent.mkdir(parents=True)
        self.chunk.write_text('{"throws": []}')
        self.unit_dir = tmp_path / "sim" / arm.name / "seed_8210" / "chunk_000"
        self.spec = cpt.UnitSpec(arm, 8210, self.chunk, self.unit_dir, 250)
        self.ctx = cpt.RunContext(
            robot="ur5e_p1b",
            config_path=tmp_path / "catch_population.yaml",
            config_sha256="c0" * 32,
            share_config_dir=tmp_path,
            sim_root=tmp_path / "sim",
            viewer=True,
            end_on_ball_low=True,
            host_watch="abort",
            throws_limit=None,
            estimator_ws=self.ball_ws,
            profile=self.profile,
            domain_id=88,
            logging_root=str(self.logging_root),
            state_logs=("ur5e_state.csv", "p1b_state.csv"),
            residual=(r"/nonexistent/[^ ]*mujoco_simulator_node",),
            revisions={
                "rtc_framework": ("unknown", "unknown", "unknown"),
                "ball_perception": ("9005aab", "0", ""),
            },
        )
        # The unit's waits, fifty times faster — and, so that a loaded host does not
        # decide a test, five times as many looks for a process to come up. The free
        # minute has its own tests.
        fast = SimpleNamespace(sleep=lambda s: time.sleep(s / 50), monotonic=time.monotonic)
        monkeypatch.setattr(cpt, "time", fast)
        monkeypatch.setattr(cpt, "STARTUP_TIMEOUT_S", 5 * cpt.STARTUP_TIMEOUT_S)
        monkeypatch.setattr(cpt, "ESTIMATOR_TIMEOUT_S", 5 * cpt.ESTIMATOR_TIMEOUT_S)
        monkeypatch.setattr(cpt, "wait_for_free_minute", lambda *a, **k: "261011_2332")
        monkeypatch.setattr(cpt, "read_mirror", lambda: (self.mirror, "tennis"))
        self.units: list[cpt.UnitRun] = []

    def set(self, **values):
        for name, value in values.items():
            self.env.setenv(name, str(value))

    def unit(self) -> cpt.UnitRun:
        self.units.append(cpt.UnitRun(self.spec, self.expect, self.ctx))
        return self.units[-1]

    def run(self) -> str:
        return self.unit().run()

    def read(self, name: str) -> str:
        return (self.unit_dir / name).read_text()

    def conditions(self) -> dict:
        return cpt.read_conditions(self.unit_dir)

    def nothing_left_running(self) -> bool:
        groups = [g for u in self.units for g in (u.launch, u.estimator, u.driver) if g]
        return not any(g.alive() for g in groups)


@pytest.fixture
def rig(tmp_path, monkeypatch):
    return Rig(tmp_path, monkeypatch, NLP)


# ── a unit that throws ───────────────────────────────────────────────────────
def test_a_unit_runs_from_bringup_to_done(rig):
    assert rig.run() == "DONE"
    assert rig.read("status") == "DONE\n"
    assert rig.nothing_left_running()
    # The bringup: the arm as launch arguments, no overlay file.
    launched = rig.read("launch.log").splitlines()[0]
    assert "launch integrated_bringup sim_ur5e_p1b.launch.py" in launched
    assert "catch_search_mode:=nlp catch_segment_mode:=mpc_docking" in launched
    assert "enable_viewer:=true" in launched and "max_log_sessions:=60" in launched
    assert "sim_overlay" not in launched
    # The estimator: its own workspace, the unit's seed.
    est = rig.read("est.log")
    assert "launch ball_perception_sim sim_estimator.launch.py" in est
    assert f"profile_path:={rig.profile}" in est and "simulator_seed:=8210" in est
    assert "producer_revision:=9005aab simulator_revision:=unknown" in est
    assert "ok=True" in rig.read("switch.log")
    # The driver: this unit's chunk, into this unit's trials directory.
    driver = (rig.fake / "driver_command").read_text()
    assert f"catching_sim_trials {rig.unit_dir / 'trials'} --profile ur5e_p1b" in driver
    assert f"--throws-file {rig.chunk} --host-watch abort --arm nlp_mpc_docking" in driver
    assert driver.strip().endswith("--end-on-ball-low")
    assert (rig.unit_dir / "trials" / "trial_results.json").is_file()
    assert "trial 5: modes=" in rig.read("trials.log")


def test_a_done_unit_holds_its_session_and_the_original_is_gone(rig):
    assert rig.run() == "DONE"
    copied = rig.unit_dir / "session" / "controllers" / "demo_catching_controller"
    assert (copied / "catching_diag.csv").read_text() == "t,mode\n0.0,1\n"
    assert not rig.session.exists()
    assert rig.read("session.txt") == f"{rig.session}\n"
    assert rig.read("session_size.txt").endswith(f"\t{rig.session}\n")
    assert not (rig.unit_dir / "session_failed").exists()


def test_a_unit_records_what_it_ran_and_what_it_was_told_to_expect(rig):
    rig.run()
    cond = rig.conditions()
    assert cond["robot"] == "ur5e_p1b" and cond["arm"] == "nlp_mpc_docking"
    assert (cond["n"], cond["seed"]) == ("250", "8210")
    assert cond["throws_file"] == str(rig.chunk)
    assert cond["throws_file_sha256"] == cpt.sha256_file(rig.chunk)
    # The arm it was told to expect — what summarize.py holds against the mirror.
    assert (cond["expect_search"], cond["expect_mode"], cond["expect_ball"]) == (
        "nlp",
        "mpc_docking",
        "tennis",
    )
    assert cond["condition"] == "mpc_docking" and cond["expect_kv"] == ""
    assert cond["launch_arguments"] == "catch_search_mode:=nlp catch_segment_mode:=mpc_docking"
    assert cond["end_on_ball_low"] == "1 margin_m=default grace_s=default"
    assert (cond["viewer"], cond["probe_dump"], cond["host_watch"]) == ("on", "off", "abort")
    assert cond["ros_domain_id"] == "88" and cond["planner_budget_s"] == "0.040"
    # No repository to ask: recorded as unknown, and the unit ran.
    assert cond["rtc_framework_rev"] == "unknown" and cond["ball_perception_rev"] == "9005aab"
    assert cond["profile_sha256"] == cpt.sha256_file(rig.profile)
    assert cond["population_config_sha256"] == "c0" * 32
    for key in ("date_start", "date_end", "loadavg_start", "loadavg_end"):
        assert cond[key], key
    assert "throws_limit" not in cond
    # The mirror as the unit tools parse it, and the startup lines.
    mirror = rig.read("mirror.txt")
    assert "planner.search.mode: String value is: nlp\n" in mirror
    assert mirror.endswith("ball_type: String value is: tennis\n")
    assert rig.read("commit.txt") == "commit at t_c − 0.390 s\n"
    assert rig.read("segment_startup.txt").startswith("mpc_docking segment planner ready: up to 9")


def test_the_units_overlay_is_a_record_the_unit_tools_can_read(rig):
    """summarize.py --unit reads what a unit changed from the file conditions.txt names
    as its overlay: here that is the launch arguments once more, in overlay form."""
    import yaml

    rig.run()
    record = Path(rig.conditions()["overlay"])
    assert record == rig.unit_dir / "launch_overrides.yaml"
    planner = yaml.safe_load(record.read_text())["integrated_rt_controller"]["ros__parameters"]
    planner = planner["demo_catching_controller"]["catching"]["planner"]
    assert planner == {"search": {"mode": "nlp"}, "segment": {"mode": "mpc_docking"}}


def test_a_limited_unit_says_so(rig):
    rig.ctx = cpt.dataclasses.replace(rig.ctx, throws_limit=5, viewer=False, end_on_ball_low=False)
    rig.spec = cpt.dataclasses.replace(rig.spec, n_throws=5)
    assert rig.run() == "DONE"
    cond = rig.conditions()
    assert (cond["n"], cond["throws_limit"], cond["viewer"]) == ("5", "5", "off")
    assert "end_on_ball_low" not in cond
    driver = (rig.fake / "driver_command").read_text()
    assert "--limit 5" in driver and "--end-on-ball-low" not in driver
    assert "enable_viewer:=false" in rig.read("launch.log")


# ── a unit that does not get to throw ────────────────────────────────────────
def test_a_failed_bringup_ends_the_unit_before_anything_else_starts(rig):
    refusing = (
        "[integrated_rt_controller-2] [ERROR] [1.0] [integrated_bringup.demo_catching_controller]: "
        "refusing to configure: no `catching:` section.\n"
    )
    (rig.fake / "launch.log").write_text(refusing)
    assert rig.run() == "FAIL:startup"
    assert rig.read("startup_failure.txt") == refusing
    assert rig.nothing_left_running()
    assert not (rig.unit_dir / "est.log").exists() and not (rig.unit_dir / "trials.log").exists()
    # A failed attempt keeps its session with the attempt.
    assert (rig.unit_dir / "session_failed" / "sim").is_dir() and not rig.session.exists()


def test_a_bringup_that_leaves_without_a_word_is_a_failed_startup(rig):
    (rig.fake / "launch.log").write_text("")
    (rig.fake / "sim_leaves").write_text("")
    assert rig.run() == "FAIL:startup"
    assert not (rig.unit_dir / "startup_failure.txt").exists()


def test_a_controller_that_did_not_load_the_arm_is_refused_before_a_throw(rig):
    """The launch arguments' failure is silent (a key one level off is created and never
    read): the controller then runs the shipped pair, and its mirror says so."""
    _, shipped_pair, log = _unit_state("ur5e_p1b", GRID)
    rig.mirror = shipped_pair
    (rig.fake / "launch.log").write_text(log)
    status = rig.run()
    assert status.startswith("FAIL:mirror mode_mirror search_mirror mode_log")
    assert "planner.search.mode: String value is: grid\n" in rig.read("mirror.txt")
    assert rig.nothing_left_running()
    assert not (rig.unit_dir / "est.log").exists() and not (rig.unit_dir / "trials.log").exists()


def test_a_shipped_value_that_is_off_is_named(rig):
    rig.mirror["planner.freeze.T_freeze"] = 0.36
    assert rig.run() == "FAIL:mirror T_freeze"


def test_a_controller_that_does_not_answer_is_a_failure_of_its_own(rig, monkeypatch):
    def unread():
        raise RuntimeError("/demo_catching_controller/get_parameters is not there")

    monkeypatch.setattr(cpt, "read_mirror", unread)
    assert rig.run() == "FAIL:mirror unread"
    assert "is not there" in rig.read("mirror.txt") and rig.nothing_left_running()


def test_an_estimator_that_does_not_activate_ends_the_unit(rig):
    rig.set(FAKE_ESTIMATOR="silent")
    assert rig.run() == "FAIL:estimator not activated"
    assert rig.nothing_left_running() and not (rig.unit_dir / "trials.log").exists()


def test_an_estimator_from_another_workspace_is_refused(rig):
    rig.set(FAKE_PREFIX="/opt/other/install/ball_perception_sim")
    assert rig.run() == "FAIL:ball_perception not from BALL_SIM_WS"
    assert "WRONG_PREFIX /opt/other/install/ball_perception_sim" in rig.read("est.log")


def test_a_merged_install_of_the_estimator_is_its_workspace_too(rig):
    rig.set(FAKE_PREFIX=rig.ball_ws / "install")
    assert rig.run() == "DONE"


def test_a_switch_the_manager_refuses_ends_the_unit(rig):
    rig.set(FAKE_SWITCH="False")
    assert rig.run() == "FAIL:switch"
    assert rig.nothing_left_running() and not (rig.unit_dir / "trials.log").exists()


def test_an_old_trials_directory_is_never_thrown_over(rig):
    (rig.unit_dir / "trials").mkdir(parents=True)
    assert rig.run() == "FAIL:trials dir exists (move the old unit aside)"
    assert not (rig.unit_dir / "launch.log").exists()


# ── an abnormal start ────────────────────────────────────────────────────────
def test_a_key_in_the_viewer_at_the_start_ends_the_unit_before_a_throw(rig):
    """2026-10-11: `G` landed in the viewer as it opened. That unit threw 82 throws."""
    with open(rig.fake / "launch.log", "a") as log:
        log.write("[mujoco_simulator_node-1] [Viewer] World gravity OFF\n")
    assert rig.run() == "FAIL:abnormal_start"
    assert rig.read("abnormal_start.txt") == (
        "a key in the viewer changed the sim: [Viewer] World gravity OFF\n"
    )
    assert rig.nothing_left_running()
    assert not (rig.unit_dir / "mirror.txt").exists()
    assert not (rig.unit_dir / "trials.log").exists()
    assert (rig.unit_dir / "session_failed").is_dir()


def test_a_flapping_start_ends_the_unit_after_its_first_throw(rig):
    (rig.fake / "trials.log").write_text(_throws(FLAP))
    rig.set(FAKE_DRIVER_RC="hang")  # the driver would go on for the whole chunk
    assert rig.run() == "FAIL:abnormal_start"
    assert "throw 0 went through 161 modes" in rig.read("abnormal_start.txt")
    assert rig.nothing_left_running()
    assert "date_end" in rig.conditions()


def test_a_key_in_the_viewer_while_the_driver_throws_ends_the_unit(rig):
    rig.set(FAKE_DRIVER_RC="hang")
    unit = rig.unit()
    original = cpt.watch_verdict
    looks = []

    def watch(**look):
        looks.append(look)
        if len(looks) == 3:  # the unit is throwing: now the key is pressed
            (rig.fake / "key").write_text("[mujoco_simulator_node-1] [Viewer] Contacts DISABLED\n")
        return original(**look)

    rig.env.setattr(cpt, "watch_verdict", watch)
    assert unit.run() == "FAIL:abnormal_start"
    assert "[Viewer] Contacts DISABLED" in rig.read("abnormal_start.txt")
    assert rig.nothing_left_running()


def test_a_key_pressed_under_the_drivers_last_throw_still_counts(rig):
    """The driver returned 0: the throws it threw in the changed sim are not the unit's."""
    rig.set(FAKE_KEY_AT_END="[mujoco_simulator_node-1] [Viewer] World gravity OFF")
    assert rig.run() == "FAIL:abnormal_start"


# ── the driver's end ─────────────────────────────────────────────────────────
def test_a_busy_host_is_the_drivers_exit_code_three(rig):
    rig.set(FAKE_DRIVER_RC="3")
    assert rig.run() == "FAIL:host_busy"
    assert rig.nothing_left_running() and (rig.unit_dir / "session_failed").is_dir()


def test_any_other_exit_code_is_the_units_status(rig):
    rig.set(FAKE_DRIVER_RC="1")
    assert rig.run() == "FAIL:trials rc=1"


def test_a_driver_that_runs_out_of_time_is_stopped(rig, monkeypatch):
    rig.set(FAKE_DRIVER_RC="hang")
    monkeypatch.setattr(cpt, "driver_timeout_s", lambda n: 0.2)
    assert rig.run() == "FAIL:trials rc=124"
    assert rig.nothing_left_running()


def test_a_sim_that_leaves_under_the_driver_ends_the_unit(rig):
    rig.set(FAKE_DRIVER_RC="hang")
    unit = rig.unit()
    original = cpt.watch_verdict

    def watch(**look):
        (rig.fake / "sim_leaves").write_text("")
        return original(**look)

    rig.env.setattr(cpt, "watch_verdict", watch)
    assert unit.run() == "FAIL:sim exited"
    assert rig.nothing_left_running()


# ── the session ──────────────────────────────────────────────────────────────
def test_a_foreign_file_in_the_session_refuses_the_unit(rig):
    rig.set(FAKE_FOREIGN="iiwa7_state.csv")
    assert rig.run() == "FAIL:contaminated foreign file iiwa7_state.csv"
    assert not (rig.unit_dir / "session").exists()
    assert (rig.unit_dir / "session_failed").is_dir() and not rig.session.exists()


def test_a_bringup_that_names_no_session_copies_and_removes_nothing(rig):
    rig.set(FAKE_SESSION=rig.tmp / "not_a_session")  # the log line then names no session
    assert rig.run() == "FAIL:no session directory in the launch log"
    assert rig.read("session.txt") == "\n"
    assert (rig.tmp / "not_a_session" / "sim").is_dir()  # left where it is


# ── a signal ─────────────────────────────────────────────────────────────────
def test_a_signal_ends_the_unit_and_everything_it_started(rig, monkeypatch):
    unit = rig.unit()

    def signalled():
        raise cpt.Interrupted(2)  # what the run's handler raises on SIGINT

    monkeypatch.setattr(cpt, "read_mirror", signalled)
    with pytest.raises(cpt.Interrupted):
        unit.run()
    assert rig.read("status") == "FAIL:signal\n"
    assert unit.launch is not None and rig.nothing_left_running()
    assert (rig.unit_dir / "session_failed").is_dir()


def test_a_signal_under_the_driver_stops_the_driver_too(rig):
    rig.set(FAKE_DRIVER_RC="hang")
    unit = rig.unit()

    def watch(**look):
        raise cpt.Interrupted(15)

    rig.env.setattr(cpt, "watch_verdict", watch)
    with pytest.raises(cpt.Interrupted):
        unit.run()
    assert rig.read("status") == "FAIL:signal\n"
    assert unit.driver is not None and unit.estimator is not None
    assert rig.nothing_left_running()


def test_a_defect_of_the_command_does_not_leave_a_sim_up(rig, monkeypatch):
    def broken():
        raise KeyError("planner.search.mode")

    monkeypatch.setattr(cpt, "read_mirror", broken)
    with pytest.raises(KeyError):
        rig.run()
    assert rig.read("status").startswith("FAIL:error KeyError")
    assert rig.nothing_left_running()


# ── retry ────────────────────────────────────────────────────────────────────
class Said:
    def __init__(self):
        self.lines = []

    def say(self, text):
        self.lines.append(text)


def _throw_unit(rig, retries=3):
    said = Said()
    done = cpt.throw_unit(said, rig.spec, rig.expect, rig.ctx, retries)
    return done, said.lines


def test_a_rig_failure_is_moved_aside_and_thrown_again(rig):
    rig.set(FAKE_DRIVER_RC="3,3,0")
    done, said = _throw_unit(rig)
    assert done and rig.read("status") == "DONE\n"
    for k in (1, 2):
        aside = rig.unit_dir.with_name(f"chunk_000.fail{k}")
        assert (aside / "status").read_text() == "FAIL:host_busy\n"
        assert (aside / "session_failed").is_dir()  # kept as evidence, with its session
    assert [line.split(" load")[0] for line in said if line.startswith("start")] == [
        "start nlp_mpc_docking/seed_8210/chunk_000 try 1",
        "start nlp_mpc_docking/seed_8210/chunk_000 try 2",
        "start nlp_mpc_docking/seed_8210/chunk_000 try 3",
    ]
    assert "status=FAIL:host_busy" in said[1] and "status=DONE" in said[-1]


def test_a_rig_failure_is_given_up_after_the_configured_bringups(rig):
    rig.set(FAKE_DRIVER_RC="3,3,3")
    done, said = _throw_unit(rig, retries=2)
    assert not done and rig.read("status") == "FAIL:host_busy\n"
    assert (rig.fake / "driver_runs").read_text().strip() == "2"
    assert said[-1] == (
        "nlp_mpc_docking/seed_8210/chunk_000 given up after 2 bringups: 'FAIL:host_busy'"
    )


def test_a_failure_the_throws_could_cause_is_not_thrown_again(rig):
    rig.set(FAKE_DRIVER_RC="1,0")
    done, said = _throw_unit(rig)
    assert not done and rig.read("status") == "FAIL:trials rc=1\n"
    assert (rig.fake / "driver_runs").read_text().strip() == "1"
    assert said[-1] == (
        "nlp_mpc_docking/seed_8210/chunk_000 not retried: 'FAIL:trials rc=1' is not a rig failure"
    )
    # Starting the command again is a person's decision: the unit is then thrown anew.
    done, said = _throw_unit(rig)
    assert done and rig.unit_dir.with_name("chunk_000.fail1").is_dir()


def test_a_done_unit_is_skipped(rig):
    assert _throw_unit(rig)[0]
    done, said = _throw_unit(rig)
    assert done and said == []
    assert (rig.fake / "driver_runs").read_text().strip() == "1"


def test_a_unit_done_under_another_limit_is_neither_skipped_nor_thrown_over(rig):
    assert _throw_unit(rig)[0]
    rig.spec = cpt.dataclasses.replace(rig.spec, n_throws=5)
    done, said = _throw_unit(rig)
    assert not done and "is DONE but it threw n=250, this run throws 5" in said[0]
    assert (rig.fake / "driver_runs").read_text().strip() == "1"
    assert rig.read("status") == "DONE\n"


def test_a_stop_file_keeps_a_failed_unit_from_being_thrown_again(rig):
    rig.set(FAKE_DRIVER_RC="3,0")
    said = Said()
    original = cpt.UnitRun.run

    def run_then_stop(self):
        status = original(self)
        (rig.tmp / "STOP").write_text("")
        return status

    rig.env.setattr(cpt.UnitRun, "run", run_then_stop)
    assert not cpt.throw_unit(said, rig.spec, rig.expect, rig.ctx, 3)
    assert (rig.fake / "driver_runs").read_text().strip() == "1"
    assert rig.read("status") == "FAIL:host_busy\n"


# ── the whole command ────────────────────────────────────────────────────────
CONFIG = Path(__file__).resolve().parents[1] / "config" / "ur5e_p1b"


class Command:
    """A result folder stage 1 would have left for one arm, and the command's arguments."""

    def __init__(self, rig: Rig, chunks=1):
        from rtc_tools.analysis.catching_throw_list import write_throw_list

        self.rig = rig
        tmp, env = rig.tmp, rig.env
        # The robot is the folder the configuration file is in.
        robot = tmp / "given" / "ur5e_p1b"
        robot.mkdir(parents=True)
        for name in ("catch_population.yaml", "mujoco_simulator.yaml"):
            (robot / name).write_text((CONFIG / name).read_text())
        self.config = robot / "catch_population.yaml"
        self.result = tmp / "result"
        throws_dir = self.result / "nlp_mpc_docking" / "throws"
        throws_dir.mkdir(parents=True)
        throws = [
            {"throw_id": i, "pos": [2.0, 0.0, 1.0], "vel": [-3.0, 0.0, 4.0]} for i in range(4)
        ]
        meta = {"plan_set_sha256": "5e" * 32, "plan_set_throws": 4, "n_throws": 4}
        write_throw_list(throws_dir / "sample.json", throws, meta)
        for k in range(chunks):
            write_throw_list(throws_dir / f"chunk_{k:03d}.json", throws[2 * k : 2 * k + 2], meta)
        self.unit_dir = self.result / "sim" / "nlp_mpc_docking" / "seed_8210" / "chunk_000"
        # The launch's logging root is the one the command resolves: this environment's.
        prefix = tmp / "install"
        prefix.mkdir()
        env.setenv("COLCON_PREFIX_PATH", str(prefix))
        rig.set(FAKE_SESSION=tmp / "logging_data" / "261011_2332")
        env.setenv("ROS_DOMAIN_ID", os.environ.get("ROS_DOMAIN_ID", "0"))  # restored after
        env.setattr(cpt, "analyse_unit", self.analyse)
        self.analysed = []
        self.args = ["--config", str(self.config), "--out", str(self.result)]
        self.args += ["--estimator-ws", str(rig.ball_ws), "--profile", str(rig.profile)]
        self.args += ["--arms", "nlp_mpc_docking", "--domain-id", "88"]

    def analyse(self, unit, config_dir, python):
        """Stands in for the analysis: every throw of the unit's chunk, the first caught."""
        import json

        self.analysed.append(Path(unit).name)
        chunk = self.result / "nlp_mpc_docking" / "throws" / f"{Path(unit).name}.json"
        (Path(unit) / "ct").mkdir(exist_ok=True)
        lines = ["idx,throw_id,invalid_reason,plan_verdict,truth_success"]
        for idx, throw in enumerate(json.loads(chunk.read_text())["throws"]):
            lines.append(f"{idx},{throw['throw_id']},,published,{idx == 0}")
        (Path(unit) / "ct" / "catching_trials.csv").write_text("\n".join(lines) + "\n")
        (Path(unit) / "ct" / "vec.csv").write_text("idx\n")
        return True

    def main(self, *more):
        return cpt.main([*self.args, *more])

    def progress(self) -> str:
        return (self.result / "sim" / "progress.log").read_text()


@pytest.fixture
def command(rig):
    from ament_index_python.packages import PackageNotFoundError, get_package_prefix

    try:
        get_package_prefix("rtc_mujoco_sim")
    except PackageNotFoundError:
        pytest.skip("this workspace has no rtc_mujoco_sim: the command refuses to throw")
    saved = {number: signal.getsignal(number) for number in SIGNALS}
    yield Command(rig)
    for number, handler in saved.items():
        signal.signal(number, handler)


def test_the_command_throws_analyses_and_collects(command):
    assert command.main() == 0
    assert (command.unit_dir / "status").read_text() == "DONE\n"
    # The session is where the command's environment says the launch writes it.
    assert (command.unit_dir / "session" / "controllers").is_dir()
    assert not (command.rig.tmp / "logging_data" / "261011_2332").exists()
    assert command.analysed == ["chunk_000"]
    rows = (command.result / "nlp_mpc_docking" / "sim_throws.csv").read_text().splitlines()
    assert rows[0].startswith("arm,seed,chunk,idx,throw_id") and len(rows) == 3
    assert (command.result / "nlp_mpc_docking" / "throw_catch_set.json").is_file()
    progress = command.progress()
    assert "1 units of ur5e_p1b (nlp_mpc_docking), viewer on, domain 88" in progress
    assert "start nlp_mpc_docking/seed_8210/chunk_000 try 1" in progress
    assert "seed 8210: thrown 2, invalid 0, pair published 2, caught 1 (1 chunks)" in progress
    # The run's domain is the unit's and the children's.
    assert os.environ["ROS_DOMAIN_ID"] == "88"
    assert cpt.read_conditions(command.unit_dir)["ros_domain_id"] == "88"
    # Started again: nothing is thrown twice.
    assert command.main() == 0
    assert (command.rig.fake / "driver_runs").read_text().strip() == "1"


def test_the_command_takes_the_seeds_the_viewer_and_the_limit_from_its_arguments(command):
    assert command.main("--seeds", "7", "--viewer", "off", "--throws-limit", "1") == 0
    unit = command.result / "sim" / "nlp_mpc_docking" / "seed_7" / "chunk_000"
    cond = cpt.read_conditions(unit)
    assert (cond["seed"], cond["viewer"], cond["n"], cond["throws_limit"]) == (
        "7",
        "off",
        "1",
        "1",
    )
    assert "simulator_seed:=7" in (unit / "est.log").read_text()


def test_a_signal_ends_the_run_and_collects_nothing(command):
    command.rig.set(FAKE_DRIVER_RC="hang")
    original = cpt.watch_verdict

    def watch(**look):
        os.kill(os.getpid(), signal.SIGINT)  # Ctrl-C while the driver throws
        return original(**look)

    command.rig.env.setattr(cpt, "watch_verdict", watch)
    assert command.main() == 130
    assert (command.unit_dir / "status").read_text() == "FAIL:signal\n"
    assert (command.unit_dir / "session_failed").is_dir()
    assert not (command.result / "nlp_mpc_docking" / "sim_throws.csv").exists()
    assert "signal 2: the running unit was ended" in command.progress()
    # A second Ctrl-C does not cut the shutdown: the handler lets one signal in.
    assert signal.getsignal(signal.SIGINT) is signal.SIG_IGN


def test_a_stop_file_ends_the_run_before_the_next_bringup(command):
    (command.result / "STOP").write_text("")
    assert command.main() == 1
    assert not command.unit_dir.exists()
    assert "STOP file: leaving before nlp_mpc_docking/seed_8210/chunk_000" in command.progress()


def test_a_unit_that_stays_failed_makes_the_run_incomplete_and_is_not_collected(command):
    command.rig.set(FAKE_DRIVER_RC="1")
    assert command.main() == 1
    assert "not collected: nlp_mpc_docking/seed_8210/chunk_000 (FAIL:trials rc=1)" in (
        command.progress()
    )
    assert command.analysed == []


def test_collect_only_throws_nothing(command):
    assert command.main() == 0
    (command.result / "nlp_mpc_docking" / "sim_throws.csv").unlink()
    (command.unit_dir / "ct" / "vec.csv").unlink()
    assert (
        cpt.main(["--config", str(command.config), "--out", str(command.result), "--collect-only"])
        == 0
    )
    assert command.analysed == ["chunk_000", "chunk_000"]  # what was not analysed is
    assert (command.result / "nlp_mpc_docking" / "sim_throws.csv").is_file()
    assert (command.rig.fake / "driver_runs").read_text().strip() == "1"


def test_collect_only_refuses_a_unit_that_threw_another_file(command):
    assert command.main() == 0
    conditions = command.unit_dir / "conditions.txt"
    conditions.write_text(
        conditions.read_text().replace("throws_file_sha256: ", "throws_file_sha256: 00")
    )
    assert command.main("--collect-only") == 1
    assert "collect nlp_mpc_docking REFUSED" in command.progress()
    assert "nlp_mpc_docking/seed_8210/chunk_000: it threw another file" in command.progress()


@pytest.mark.parametrize(
    ("change", "message"),
    [
        (["--arms", "grid_closed_form"], "no arm 'grid_closed_form'"),
        (["--out", "/nonexistent/result"], "no result folder /nonexistent/result"),
        (["--arms", "grid_mpc"], "no chunk files for ['grid_mpc']"),
        (["--estimator-ws", "/nonexistent/ws"], "no colcon workspace at /nonexistent/ws"),
        (["--profile", "/nonexistent/p.json"], "no estimator profile /nonexistent/p.json"),
    ],
)
def test_a_run_that_cannot_be_what_it_says_is_refused_before_a_bringup(
    command, capsys, change, message
):
    with pytest.raises(SystemExit) as refused:
        command.main(*change)  # the later argument is the one argparse keeps
    assert refused.value.code == 2 and message in capsys.readouterr().err
    assert not command.unit_dir.exists()


def test_a_configuration_stage_one_did_not_judge_with_is_refused(command, capsys):
    import yaml

    document = yaml.safe_load(command.config.read_text())
    document["population"]["speed_max_m_s"] = 14
    (command.result / "config.yaml").write_text(yaml.safe_dump(document))
    with pytest.raises(SystemExit):
        command.main()
    assert "differs in: population" in capsys.readouterr().err
    assert not command.unit_dir.exists()
