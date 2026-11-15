"""tools/standing_eval/run_arms.py (not installed — read from the source tree).

The launch arguments each arm gets: without ``--mpc-threads`` they are the ones
the tool always passed; with it, every arm launched under the MPC layout also
names the simulation worker profile and the solver's thread count.
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
