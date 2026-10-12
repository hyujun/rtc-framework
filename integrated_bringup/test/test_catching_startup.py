"""integrated_bringup.catching_startup: what a unit driver reads off a bringup's logs.

The startup and mode checks are the ones ``tools/catching_eval/check_startup.py`` and
``check_segment_mode.py`` held before #821 (their tests moved here with them, every
assertion kept); the two files are shims now, and the script tests below still run
THEM — what ``run_unit.sh`` calls. The abnormal-start checks are new: both are held
against excerpts shaped like the 2026-10-11 unit that came up without gravity (a key
in the viewer) and like the units that ran normally beside it.
"""

import subprocess
import sys
from pathlib import Path

import pytest

from integrated_bringup import catching_startup as st

TOOLS = Path(__file__).resolve().parents[1] / "tools" / "catching_eval"


# ── which mode did the controller log? (check_segment_mode.py) ───────────────

_MPC_LINE = (
    "[INFO] [1.0] [demo_catching_controller]: segment mode: mpc — takes a plan with its first "
    "segment and follows the MPC's segments from APPROACH to the end of the stop"
)
_CF_LINE = (
    "[INFO] [1.0] [demo_catching_controller]: segment mode: closed_form — the RT tick makes "
    "the reference itself"
)
_OTHER = (
    "[INFO] [1.0] [demo_catching_controller]: planner enabled: wake timeout 0.1 s, budget 0.2 s"
)


@pytest.mark.parametrize(
    ("expected", "log", "passes"),
    [
        ("closed_form", _OTHER + "\n" + _CF_LINE + "\n", True),
        ("mpc", _OTHER + "\n" + _MPC_LINE + "\n", True),
        # positive controls: the case the old check passed — the mpc line is absent
        # and the closed_form line is absent too (a controller that never configured).
        ("closed_form", _OTHER + "\n", False),
        ("closed_form", _OTHER + "\n" + _MPC_LINE + "\n", False),
        ("mpc", _OTHER + "\n" + _CF_LINE + "\n", False),
        ("mpc", _OTHER + "\n", False),
        ("closed_form", "", False),
        ("mpc", "", False),
        # both lines: no controller configures twice into one launch.log
        ("mpc", _MPC_LINE + "\n" + _CF_LINE + "\n", False),
        ("closed_form", _MPC_LINE + "\n" + _CF_LINE + "\n", False),
        # the old spelling says nothing about the mode
        ("mpc", "[demo_catching_controller]: DECEL law: mpc — takes a plan\n", False),
    ],
)
def test_the_segment_mode_check_needs_the_expected_line_present(expected, log, passes):
    assert (st.check(log, expected) is None) is passes


def test_the_segment_mode_check_refuses_an_unknown_expected_mode():
    assert "not one of" in st.check(_MPC_LINE, "decel")


def test_the_segment_mode_script_exits_on_the_verdict(tmp_path):
    log = tmp_path / "launch.log"
    log.write_text(_OTHER + "\n" + _CF_LINE + "\n")
    script = TOOLS / "check_segment_mode.py"
    run = [sys.executable, str(script), str(log)]
    assert subprocess.run([*run, "closed_form"], capture_output=True).returncode == 0
    refused = subprocess.run([*run, "mpc"], capture_output=True, text=True)
    assert refused.returncode == 1 and "no 'segment mode: mpc' line" in refused.stderr
    log.write_text(_OTHER + "\n")  # nothing logged: both expectations fail
    assert subprocess.run([*run, "closed_form"], capture_output=True).returncode == 1
    assert subprocess.run([*run, "mpc", str(tmp_path / "missing.log")]).returncode == 2
    assert subprocess.run([*run[:2], str(tmp_path / "missing.log"), "mpc"]).returncode == 1


_DOCKING_LINE = (
    "[INFO] [1.0] [demo_catching_controller]: segment mode: mpc_docking — the docking planner"
)
_GRID_LINE = "[INFO] [1.0] [demo_catching_controller]: search mode: grid — enumerates the grid"
_NLP_LINE = "[INFO] [1.0] [demo_catching_controller]: search mode: nlp — solves the NLP"


def test_the_segment_mode_check_knows_mpc_docking_and_not_as_mpc():
    log = _OTHER + "\n" + _DOCKING_LINE + "\n"
    assert st.check(log, "mpc_docking") is None
    # `mpc_docking` must not read as `mpc` (the regex alternation order)
    assert st.logged_modes(log) == {"mpc_docking"}
    assert st.check(log, "mpc") is not None
    assert st.check(_OTHER + "\n" + _MPC_LINE + "\n", "mpc_docking") is not None


@pytest.mark.parametrize(
    ("expected", "log", "passes"),
    [
        ("grid", _GRID_LINE + "\n", True),
        ("nlp", _NLP_LINE + "\n", True),
        ("grid", _NLP_LINE + "\n", False),
        ("nlp", _GRID_LINE + "\n", False),
        ("grid", _OTHER + "\n", False),  # a controller that never configured is silent
        ("grid", _GRID_LINE + "\n" + _NLP_LINE + "\n", False),
    ],
)
def test_the_search_mode_check_needs_the_expected_line_present(expected, log, passes):
    assert (st.check_search(log, expected) is None) is passes


def test_the_search_mode_check_refuses_an_unknown_expected_mode():
    assert "not one of" in st.check_search(_GRID_LINE, "annealing")


def test_the_segment_mode_script_checks_the_search_line_with_the_flag(tmp_path):
    log = tmp_path / "launch.log"
    log.write_text("\n".join([_OTHER, _DOCKING_LINE, _NLP_LINE]) + "\n")
    run = [sys.executable, str(TOOLS / "check_segment_mode.py"), str(log), "mpc_docking"]
    assert subprocess.run(run, capture_output=True).returncode == 0  # CLI unchanged
    assert subprocess.run([*run, "--search", "nlp"], capture_output=True).returncode == 0
    refused = subprocess.run([*run, "--search", "grid"], capture_output=True, text=True)
    assert refused.returncode == 1 and "no 'search mode: grid' line" in refused.stderr
    assert subprocess.run([*run, "--bogus", "nlp"], capture_output=True).returncode == 2


# ── has the controller come up, or has the bringup failed? (check_startup.py) ─

_READY = (
    "[integrated_rt_controller-2] [INFO] [1.0] [integrated_bringup.demo_catching_controller]: "
    "supervisor: trials enabled — commit at t_c − 0.370 s"
)
# What a healthy controller logs while it configures, before the ready line (#793).
_REACH_BOUND = (
    "[integrated_rt_controller-2] [INFO] [1.0] [integrated_bringup.demo_catching_controller]: "
    "search: reach bound 1.1169 m about [0.0000 0.0000 0.1625] (model world, 6 joints) — a "
    "candidate whose IK target is farther than that plus the IK's position tolerance is refused "
    "without the IK (too_far)"
)
_REFUSING = (
    "[integrated_rt_controller-2] [ERROR] [1.0] [integrated_bringup.demo_catching_controller]: "
    "refusing to configure: no `catching:` section."
)
_CONFIG_LOAD = (
    "[integrated_rt_controller-2] [ERROR] [1.0] [integrated_rt_controller]: Config load failed "
    "for 'demo_catching_controller' (pkg=integrated_bringup): bad YAML"
)
_WILL_REFUSE = (
    "[integrated_rt_controller-2] [WARN] [1.0] [integrated_bringup.demo_catching_controller]: "
    "DISABLED: … This controller will refuse to activate; the robot still comes up."
)


@pytest.mark.parametrize(
    ("log", "state"),
    [
        ("", st.WAITING),
        (_OTHER + "\n", st.WAITING),
        # The race this check was written for: the poll sees the reach-bound line and
        # not yet the ready line. The old pattern (`refus` anywhere) failed the unit here.
        (_OTHER + "\n" + _REACH_BOUND + "\n", st.WAITING),
        (_REACH_BOUND + "\n" + _READY + "\n", st.READY),
        (_READY + "\n", st.READY),
        # positive controls: a failed bringup is still a failure
        (_REACH_BOUND + "\n" + _REFUSING + "\n", st.FAILED),
        (_CONFIG_LOAD + "\n", st.FAILED),
        (_WILL_REFUSE + "\n", st.FAILED),
        # a line with no level at all (the launch's own output) counts
        ("bring_up_failed: integrated_rt_controller\n", st.FAILED),
        # ready wins: the controller came up, whatever was logged on the way
        (_WILL_REFUSE + "\n" + _READY + "\n", st.READY),
    ],
)
def test_the_startup_check_tells_a_failed_bringup_from_a_healthy_refusal_line(log, state):
    assert st.startup_state(log) == state


def test_the_startup_check_names_the_failure_lines_only():
    log = "\n".join([_OTHER, _REACH_BOUND, _REFUSING, _CONFIG_LOAD]) + "\n"
    assert st.failure_lines(log) == [_REFUSING, _CONFIG_LOAD]


def test_the_startup_script_exits_on_the_state(tmp_path):
    log = tmp_path / "launch.log"
    run = [sys.executable, str(TOOLS / "check_startup.py"), str(log)]
    assert subprocess.run(run, capture_output=True).returncode == 2  # no log yet
    log.write_text(_REACH_BOUND + "\n")
    assert subprocess.run(run, capture_output=True).returncode == 2
    log.write_text(_REACH_BOUND + "\n" + _REFUSING + "\n")
    failed = subprocess.run(run, capture_output=True, text=True)
    assert failed.returncode == 1 and failed.stderr.strip() == _REFUSING
    log.write_text(_REACH_BOUND + "\n" + _READY + "\n")
    assert subprocess.run(run, capture_output=True).returncode == 0


# ── the two mains, asked directly (the scripts above are shims over them) ────


def test_the_two_shims_hold_no_check_of_their_own():
    assert "import startup_main as main" in (TOOLS / "check_startup.py").read_text()
    assert "import segment_mode_main as main" in (TOOLS / "check_segment_mode.py").read_text()


def test_the_startup_main_returns_the_state(tmp_path, capsys):
    log = tmp_path / "launch.log"
    assert st.startup_main(["check_startup.py", str(log)]) == 2  # no log yet
    log.write_text(_REACH_BOUND + "\n")
    assert st.startup_main(["check_startup.py", str(log)]) == 2
    log.write_text(_REACH_BOUND + "\n" + _REFUSING + "\n")
    capsys.readouterr()
    assert st.startup_main(["check_startup.py", str(log)]) == 1
    assert capsys.readouterr().err.strip() == _REFUSING
    log.write_text(_REACH_BOUND + "\n" + _READY + "\n")
    assert st.startup_main(["check_startup.py", str(log)]) == 0
    assert st.startup_main(["check_startup.py"]) == 2


def test_the_segment_mode_main_returns_the_verdict(tmp_path, capsys):
    log = tmp_path / "launch.log"
    log.write_text("\n".join([_OTHER, _DOCKING_LINE, _NLP_LINE]) + "\n")
    run = ["check_segment_mode.py", str(log)]
    assert st.segment_mode_main([*run, "mpc_docking"]) == 0
    assert st.segment_mode_main([*run, "mpc_docking", "--search", "nlp"]) == 0
    capsys.readouterr()
    assert st.segment_mode_main([*run, "mpc_docking", "--search", "grid"]) == 1
    assert "no 'search mode: grid' line" in capsys.readouterr().err
    assert st.segment_mode_main([*run, "mpc"]) == 1
    assert "no 'segment mode: mpc' line" in capsys.readouterr().err
    assert st.segment_mode_main([*run, "mpc_docking", "--bogus", "nlp"]) == 2
    assert st.segment_mode_main(["check_segment_mode.py", str(tmp_path / "no.log"), "mpc"]) == 1


def test_the_modes_are_the_ones_the_launch_arguments_take():
    from integrated_bringup import catch_modes

    assert set(st.MODES) == set(catch_modes.SEGMENT_MODES)
    assert {"grid", "nlp"} == set(catch_modes.SEARCH_MODES)


# ── abnormal start: a key in the viewer (#821) ───────────────────────────────

# The viewer's banner as the simulator prints it at start, and what it logged on
# 2026-10-11 when `G` and `O` were pressed in its window as it opened.
_VIEWER_BANNER = "\n".join(
    [
        "[mujoco_simulator_node-1] [Viewer] Ready — press F1 in the window for help",
        "[mujoco_simulator_node-1]   Simulation : Space=pause  +/-=speed  Right=step  R=reset  O=object",
        "[mujoco_simulator_node-1]   Physics    : G=gravity  N=contacts",
        "[mujoco_simulator_node-1]   Solver     : I=integrator  S=solver  ]/[=iterations  F4=stats",
        "[mujoco_simulator_node-1]   Frames     : B=link frames  J=joint axes  Shift+J=joint frames",
    ]
)
_GRAVITY_OFF = "[mujoco_simulator_node-1] [Viewer] World gravity OFF"
_AFTER = "\n".join(
    [
        "[mujoco_simulator_node-1] [ObjectPool] spawned 'apple' at (-0.472, -0.131, 0.160)",
        "[mujoco_simulator_node-1] [INFO] [1.5] [mujoco_simulator]: [MuJoCoSimulatorNode] "
        "Simulator running=true  steps=500  sim_time=1.00 s  rtf=2.7x",
    ]
)


def test_a_normal_bringup_names_no_key():
    assert st.viewer_interference(_VIEWER_BANNER + "\n" + _AFTER + "\n") == []
    assert st.viewer_interference("") == []


def test_the_gravity_key_of_the_abnormal_unit_is_found():
    log = "\n".join([_VIEWER_BANNER, _GRAVITY_OFF, _AFTER]) + "\n"
    assert st.viewer_interference(log) == ["[Viewer] World gravity OFF"]


@pytest.mark.parametrize(
    "line",
    [
        "[Viewer] World gravity OFF",
        "[Viewer] World gravity ON",
        "[Viewer] Contacts DISABLED",
        "[Viewer] Contacts ENABLED",
        "[Viewer] Integrator → RK4",
        "[Viewer] Solver → CG",
        "[Viewer] Solver iterations → 200",
    ],
)
def test_every_key_that_changes_the_physics_is_a_sign(line):
    assert st.viewer_interference(f"[mujoco_simulator_node-1] {line}\n") == [line]


@pytest.mark.parametrize(
    "line",
    [
        "[Viewer] Camera → Free",
        "[Viewer] Camera → Tracking body 3",
        "[Viewer] Link frames ON",
        "[Viewer] Joint frames OFF",
        "[Viewer] Fingertip wrench arrows ON",
        "[Viewer] Selected body 12",
        "[Viewer] Screenshot saved: shot.png",
        "[Viewer] Closed",
    ],
)
def test_a_key_that_only_changes_the_picture_is_not(line):
    assert st.viewer_interference(f"[mujoco_simulator_node-1] {line}\n") == []


def test_a_key_pressed_twice_is_two_lines():
    """Gravity off and on again: the throws between the two flew without it."""
    log = "x\n[sim] [Viewer] World gravity OFF\ny\n[sim] [Viewer] World gravity ON\n"
    assert st.viewer_interference(log) == [
        "[Viewer] World gravity OFF",
        "[Viewer] World gravity ON",
    ]


# ── abnormal start: a throw that never ends (#821) ───────────────────────────

_PREFIX = "[INFO] [1791726035.271574798] [catching_sim_trials]: "
# The driver's two lines per throw. The aligned line carries lists too and must not
# be read as a mode list.
_ALIGNED = (
    _PREFIX + "trial {idx}: aligned {{'armed_at_throw': True, 'home_start_q': [-0.0003, "
    "-0.5608], 'home_total_s': 0.3, 'q_at_throw': [-0.00038, -0.56081], "
    "'err_q_at_throw': 0.0, 'err_qd_at_throw': 0.0}}\n"
)
# Mode lists of the abnormal unit (p1b nlp x mpc_docking, 2026-10-11): the ball never
# came down, so the controller saw it for the whole 12 s cap ...
_FLAP = ["TRACKING", "ARMED"] * 80 + ["TRACKING"]
# ... or the cycle opened on it and aborted a second after APPROACH.
_ABORTED = ["TRACKING", "APPROACH", "COMMITTED", "CLOSING", "ABORT_SAFE", "RETREAT", "ARMED"]
# Mode lists of the unit that threw the same chunk normally.
_NO_CYCLE = ["TRACKING", "ARMED", "TRACKING"]
_CYCLE = [
    "TRACKING",
    "APPROACH",
    "COMMITTED",
    "CLOSING",
    "DECEL",
    "HOLD",
    "RETREAT",
    "ARMED",
    "TRACKING",
    "ARMED",
    "TRACKING",
]


def _trials_log(*throws):
    """A driver log of ``throws`` = ``(mode list, outcome)``, in throw order."""
    text = ""
    for idx, (modes, outcome) in enumerate(throws):
        text += _ALIGNED.format(idx=idx)
        text += _PREFIX + f"trial {idx}: modes={modes!r} outcome={outcome} final_mode=TRACKING\n"
    return text


_ABNORMAL_UNIT = _trials_log(
    (_FLAP, "None"), (_FLAP, "None"), (_ABORTED, "ABORTED"), (_FLAP, "None"), (_ABORTED, "ABORTED")
)
_NORMAL_UNIT = _trials_log(
    (_NO_CYCLE, "None"),
    (_CYCLE, "MISSED"),
    (_CYCLE[:9], "CAPTURED"),
    (_NO_CYCLE + ["ARMED", "TRACKING"], "None"),
    (_CYCLE, "MISSED"),
)


def test_the_drivers_throw_lines_are_read_as_index_count_and_outcome():
    assert st.throw_mode_counts(_NORMAL_UNIT) == [
        (0, 3, "None"),
        (1, 11, "MISSED"),
        (2, 9, "CAPTURED"),
        (3, 5, "None"),
        (4, 11, "MISSED"),
    ]
    assert st.throw_mode_counts(_ABNORMAL_UNIT)[0] == (0, 161, "None")
    # A throw still in flight has its aligned line only.
    assert st.throw_mode_counts(_ALIGNED.format(idx=0)) == []


def test_the_abnormal_unit_is_ended_on_its_first_throw():
    state, why = st.flapping_start(_ABNORMAL_UNIT)
    assert state == st.FAILED
    assert "throw 0" in why and "161 modes" in why
    # Its first finished throw is enough: the unit need not run the other four.
    assert st.flapping_start(_trials_log((_FLAP, "None")))[0] == st.FAILED


def test_the_normal_unit_is_let_go_after_its_first_throws():
    assert st.flapping_start(_NORMAL_UNIT) == (st.READY, None)
    # Before the fifth throw nothing is said yet, and nothing at all before the first.
    assert st.flapping_start(_trials_log(*[(_NO_CYCLE, "None")] * 4)) == (st.WAITING, None)
    assert st.flapping_start("") == (st.WAITING, None)
    assert st.flapping_start(_ALIGNED.format(idx=0)) == (st.WAITING, None)


def test_an_outcome_never_ends_a_unit():
    """Five ABORTED throws are what the arm did with five throws — a finding, not a rig
    failure: a unit ended (and thrown again) for them would choose its outcomes."""
    assert st.flapping_start(_trials_log(*[(_ABORTED, "ABORTED")] * 5)) == (st.READY, None)


def test_the_flap_is_looked_for_in_the_first_throws_only():
    """A long list at the sixth throw is not a START — a key in the viewer later in a
    unit is the launch log's to say (viewer_interference)."""
    log = _trials_log(*[(_CYCLE, "MISSED")] * st.ABNORMAL_START_THROWS, (_FLAP, "None"))
    assert st.flapping_start(log) == (st.READY, None)
    early = _trials_log(*[(_CYCLE, "MISSED")] * (st.ABNORMAL_START_THROWS - 1), (_FLAP, "None"))
    assert st.flapping_start(early)[0] == st.FAILED


def test_the_longest_normal_throw_is_far_under_the_flap_count():
    # The longest mode list of the 4,527 throws that ran normally: 15 entries.
    longest = ["TRACKING", "ARMED"] * 7 + ["TRACKING"]
    assert st.flapping_start(_trials_log(*[(longest, "None")] * 5)) == (st.READY, None)
    at = ["TRACKING", "ARMED"] * (st.FLAP_MODES // 2)
    assert st.flapping_start(_trials_log((at, "None")))[0] == st.FAILED
    assert st.flapping_start(_trials_log((at[:-1], "None")))[0] == st.WAITING
