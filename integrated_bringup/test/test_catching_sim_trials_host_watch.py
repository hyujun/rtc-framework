"""The sim-trial runner's host load watch (#601).

A sim that runs slower than the wall makes the controller drop a healthy ball
as ``BALL_STALE`` (D-S8-17), so a loaded run measures the host. What is
pinned here:

* the verdict is read off the truth rows the runner already records — a slow
  stretch inside an otherwise real-time trial is found, a real-time trial is
  not, and the rows of a measured unloaded run (2026-09-27, receive jitter
  included) stay far from the threshold;
* the runner does what its mode says — ``abort`` ends the run on the loaded
  throw and says so in ``run_meta.json``, ``warn`` goes on, ``off`` writes no
  key at all;
* the cause report never names the runner's own family: these tests run under
  ``pytest`` (under ``colcon test``), which is exactly what the pattern looks
  for.
"""

import json
import os

import pytest

from integrated_bringup.catching_sim_trials import (
    EXIT_HOST_BUSY,
    HOST_WATCH_MODES,
    HostWatch,
    busy_processes,
    command_line,
    parse_args,
    process_family,
    process_table,
    run_trials,
    truth_rtf,
    write_run_meta,
)

DT = 0.01  # the sim's truth period (projectile_ball.publish.sample_rate_hz 100)


def truth_rows(segments, wall0=1000.0, stamp0=1000.0005):
    """Truth rows for ``segments`` of ``(sim seconds, rtf)``, one row per DT of sim time."""
    rows, wall, stamp = [], wall0, stamp0
    for span, rtf in segments:
        for _ in range(round(span / DT)):
            rows.append((wall, stamp, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
            stamp += DT
            wall += DT / rtf
    return rows


class FakeLogger:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(("info", msg))

    def error(self, msg):
        self.lines.append(("error", msg))


class FakeNode:
    """The driver's surface :func:`run_trials` uses, with a scripted sim speed per throw."""

    def __init__(self, segments_per_throw):
        self.segments = segments_per_throw
        self.truth_rows = []
        self.thrown = []
        self.log = FakeLogger()

    def get_logger(self):
        return self.log

    def home(self):
        return {"err_q_at_throw": 0.0}

    def throw(self, idx, throw):
        self.thrown.append(idx)
        self.truth_rows = truth_rows(self.segments[idx])
        return {"idx": idx, **throw, "accepted": True, "n_truth_rows": len(self.truth_rows)}

    def spin_for(self, seconds):
        pass


REAL_TIME = [(3.0, 1.0)]
LOADED = [(1.0, 1.0), (0.5, 0.5), (1.5, 1.0)]  # half speed for 0.5 s of sim time
COLCON = [(4242, 1, "/usr/bin/python3 /usr/bin/colcon test --packages-select rtc_base")]


def run(mode, segments, table=COLCON):
    node = FakeNode(segments)
    watch = HostWatch(mode=mode, table=lambda: table, own_pid=os.getpid())
    results = []
    run_trials(node, [{"pos": (1.0, 0.0, 0.2)}] * len(segments), {"m": 1}, watch, results)
    return node, watch, results


def test_a_slow_stretch_is_found_and_a_real_time_trial_is_not():
    assert truth_rtf(truth_rows(REAL_TIME))["host_rtf_min"] == pytest.approx(1.0, abs=1e-6)
    loaded = truth_rtf(truth_rows(LOADED))
    assert loaded["host_rtf_min"] == pytest.approx(0.5, abs=1e-6)
    # The ball looked 0.5 s older than its stamp after the slow stretch: the
    # quantity the controller's age check reads.
    assert loaded["host_lag_max_ms"] == pytest.approx(500.0 - 0.5, abs=1.0)
    # Whole-trial RTF would hide it: 3 s of sim in 3.5 s of wall is 0.857, and a
    # shorter stretch hides further. The window is what finds it.
    short = truth_rtf(truth_rows([(5.0, 1.0), (0.25, 0.5), (5.0, 1.0)]))
    assert short["host_rtf_min"] < 0.6


def test_rows_that_decide_nothing_are_not_called_load():
    assert truth_rtf([]) == {"host_rtf_min": None, "host_lag_max_ms": None}
    assert truth_rtf(truth_rows([(DT, 1.0)]))["host_rtf_min"] is None
    # Too short for one half window: a lag, but no speed.
    brief = truth_rtf(truth_rows([(0.1, 1.0)]))
    assert brief["host_rtf_min"] is None and brief["host_lag_max_ms"] is not None
    node, watch, results = run("abort", [[], REAL_TIME])
    assert [r["host_busy"] for r in results] == [False, False]
    assert not watch.aborted


def test_receive_jitter_of_an_unloaded_run_stays_far_from_the_threshold():
    # Receive − stamp of an unloaded run measured 0.1–4.0 ms (S8-G unit, 8
    # throws, 2026-09-27). The worst case for a window is its two end rows
    # pulled apart by the whole spread.
    rows = truth_rows(REAL_TIME)
    jittered = [
        (w + (0.004 if i % 25 == 24 else 0.0001 if i % 25 == 0 else 0.0006), *r)
        for i, (w, *r) in enumerate(rows)
    ]
    assert truth_rtf(jittered)["host_rtf_min"] > 0.98


def test_abort_ends_the_run_on_the_loaded_throw_and_records_why(tmp_path):
    node, watch, results = run("abort", [REAL_TIME, LOADED, REAL_TIME])
    assert node.thrown == [0, 1]  # the third throw never happened
    assert [r["host_busy"] for r in results] == [False, True]
    assert watch.aborted
    write_run_meta(str(tmp_path), {"n_throws": 3}, watch)
    meta = json.loads((tmp_path / "run_meta.json").read_text())
    assert meta["n_throws"] == 3
    hw = meta["host_watch"]
    assert hw["mode"] == "abort" and hw["aborted"] is True
    assert hw["rtf_min"] == 0.95 and hw["window_s"] == 0.25
    (found,) = hw["detections"]
    assert found["idx"] == 1
    assert found["host_rtf_min"] == pytest.approx(0.5, abs=1e-6)
    assert found["processes"] == ["4242 " + COLCON[0][2]]
    errors = [m for level, m in node.log.lines if level == "error"]
    assert any("host load" in m and "colcon test" in m for m in errors)
    assert any("--host-watch abort" in m for m in errors)
    assert EXIT_HOST_BUSY != 0


def test_warn_records_the_same_finding_and_throws_on():
    node, watch, results = run("warn", [REAL_TIME, LOADED, REAL_TIME])
    assert node.thrown == [0, 1, 2]
    assert [r["host_busy"] for r in results] == [False, True, False]
    assert not watch.aborted
    assert [d["idx"] for d in watch.detections] == [1]


def test_a_slow_sim_with_no_build_on_the_host_is_still_load():
    # The RTF is the verdict; the process list only names a likely cause.
    node, watch, results = run("abort", [LOADED], table=[])
    assert watch.aborted
    assert watch.detections[0]["processes"] == []
    assert any("none" in m for level, m in node.log.lines if level == "error")


def test_off_changes_nothing_that_was_written_before(tmp_path):
    node, watch, results = run("off", [REAL_TIME, LOADED])
    assert node.thrown == [0, 1]
    before = {"idx", "pos", "accepted", "n_truth_rows", "err_q_at_throw", "controller_mirror"}
    assert all(set(r) == before for r in results)
    meta = {"args": {"seed": 1}, "n_throws": 2}
    write_run_meta(str(tmp_path), meta, watch)
    assert json.loads((tmp_path / "run_meta.json").read_text()) == meta
    assert not watch.detections and not watch.aborted
    # ...and the watch adds exactly these keys when it is on.
    on = run("warn", [REAL_TIME])[2][0]
    assert set(on) - before == {"host_rtf_min", "host_lag_max_ms", "host_busy"}


def test_the_runner_does_not_report_its_own_family():
    me = os.getpid()
    table = [
        (1, 0, "/sbin/init"),
        (10, 1, "/usr/bin/python3 /usr/bin/colcon test"),  # the runner's test session
        (11, 10, "/usr/bin/ctest -C Release"),
        (12, 11, "/usr/bin/python3 -m pytest test_catching_sim_trials_host_watch.py"),
        (me, 12, "python3 catching_sim_trials"),
        (20, me, "pytest spawned-by-the-runner"),
        (21, 20, "colcon build grandchild"),
        (30, 1, "/usr/bin/python3 /usr/bin/colcon build --packages-up-to rtc_tools"),
        (31, 30, "/usr/bin/ctest"),
        (40, 1, "bash -c sleep 5"),
    ]
    assert process_family(table, me) == {1, 10, 11, 12, me, 20, 21}
    assert busy_processes(table, me) == [
        "30 /usr/bin/python3 /usr/bin/colcon build --packages-up-to rtc_tools",
        "31 /usr/bin/ctest",
    ]


def test_a_command_that_only_mentions_a_test_runner_is_not_one():
    # A shell left behind by another session: its -c script greps for the
    # runners' names, it runs none of them.
    watcher = command_line(
        b"/bin/bash\0-c\0while true; do ps -eo args | grep -E 'colcon test|ctest|pytest'; "
        b"sleep 10; done\0"
    )
    commit = command_line(b"git\0commit\0-m\0fix the pytest fixture\0")
    real = command_line(b"/usr/bin/python3\0/usr/bin/colcon\0test\0--packages-select\0rtc_tools\0")
    table = [
        (1, 0, "/sbin/init"),
        (50, 1, watcher),
        (51, 1, commit),
        (52, 1, real),
        (53, 52, command_line(b"/usr/bin/python3\0-m\0pytest\0--tb=short\0")),
    ]
    assert watcher == "/bin/bash -c <text>"
    assert busy_processes(table, os.getpid()) == [
        "52 /usr/bin/python3 /usr/bin/colcon test --packages-select rtc_tools",
        "53 /usr/bin/python3 -m pytest --tb=short",
    ]


def test_a_process_that_rewrote_its_title_is_read_by_that_title():
    # setproctitle leaves one string without separators: it is the command,
    # not a text the command carries.
    title = command_line(b"colcon test --packages-select rtc_tools\0")
    assert title == "colcon test --packages-select rtc_tools"
    assert busy_processes([(1, 0, "/sbin/init"), (60, 1, title)], os.getpid()) == [
        "60 colcon test --packages-select rtc_tools"
    ]


def test_the_real_process_table_holds_this_process_and_excludes_its_pytest():
    table = process_table()
    pids = {p for p, _, _ in table}
    assert os.getpid() in pids and os.getppid() in pids
    # Under pytest the pattern matches this very process (or its parent);
    # nothing in the report may be in the family.
    family = process_family(table, os.getpid())
    assert all(int(line.split()[0]) not in family for line in busy_processes(table, os.getpid()))


def test_the_cli_defaults_to_warn_with_the_analysers_window_and_threshold():
    args = parse_args(["out"])
    assert (args.host_watch, args.host_rtf_min, args.host_window) == ("warn", 0.95, 0.25)
    assert parse_args(["out", "--host-watch", "abort"]).host_watch == "abort"
    assert set(HOST_WATCH_MODES) == {"off", "warn", "abort"}
    with pytest.raises(ValueError, match="not one of"):
        HostWatch(mode="loud")
