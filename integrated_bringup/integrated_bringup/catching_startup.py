"""Has a catching sim unit come up, in the modes it was asked for, and is it the sim it should be?

What a unit driver reads off the logs of a sim bringup before and while it
throws. ``catch_population_trials`` imports the checks; ``tools/catching_eval/
check_startup.py`` and ``check_segment_mode.py`` are shims over
:func:`startup_main` / :func:`segment_mode_main`, so ``run_unit.sh`` asks the
same functions. Every check is a function of the log text: a test feeds it a
synthetic log, and nothing here imports ROS.

**Startup** (#747). The controller writes ``supervisor: trials enabled`` once it
can throw. A failed bringup is one of ``Config load failed`` /
``bring_up_failed`` / ``refus…`` on a line that is NOT an ``[INFO]`` line. The
level matters: a healthy controller says "… is refused without the IK
(too_far)" at INFO while it configures, a few tens of milliseconds before the
ready line, and a poll that fell between the two used to end the unit as
``FAIL:startup`` before a throw. The lines that do mean a failed bringup are
WARN / ERROR / FATAL lines (the controller's ``refusing to configure`` / ``will
refuse to activate``, the manager's ``Config load failed``) or carry no level at
all (the launch's own output).

**Modes** (#711). Every mode is said at configure — ``segment mode: mpc
— …``, ``search mode: nlp — …`` — so the expected line has to be PRESENT: the
absence of the other ones proves nothing, a controller that never configured is
just as silent.

**Abnormal start** (#821). A unit can come up, pass every check above and still
not be the sim it should be. On 2026-10-11 a key press landed in the MuJoCo
viewer as it opened (``G`` — world gravity — and ``O``): the ball then flew
without gravity (12 s after a launch it was 35 m up), the controller flapped
TRACKING ↔ ARMED some 80 times until each throw ran into its cap, or aborted a
second after APPROACH, no throw reached CAPTURED or MISSED, and the unit ran 82
throws before a person stopped it. Two signs, either of which ends the unit:

* the cause — a ``[Viewer]`` line of a key that changes the physics
  (:func:`viewer_interference`), at any time of the unit;
* the symptom, whatever caused it — a throw whose mode list is far longer than a
  throw's can be (:func:`flapping_start`), among the first
  ``ABNORMAL_START_THROWS`` throws the driver logs.

Neither depends on what the arm did with a throw, so a driver may throw such a
unit again (a rig failure, like a busy host).
"""

from __future__ import annotations

import re
import sys

from integrated_bringup.catch_modes import SEARCH_MODES, SEGMENT_MODES

# ── startup ──────────────────────────────────────────────────────────────────

READY, FAILED, WAITING = "ready", "failed", "waiting"
READY_LINE = "supervisor: trials enabled"
_FAILURE = re.compile(r"Config load failed|bring_up_failed|refus")
_INFO = re.compile(r"\[INFO\]")


def failure_lines(log_text: str) -> list[str]:
    """The lines of the log that say the bringup failed."""
    return [
        line for line in log_text.splitlines() if _FAILURE.search(line) and not _INFO.search(line)
    ]


def startup_state(log_text: str) -> str:
    """``ready`` once the ready line is there (whatever else is), else ``failed`` on a
    failure line, else ``waiting``."""
    if READY_LINE in log_text:
        return READY
    return FAILED if failure_lines(log_text) else WAITING


# ── modes ────────────────────────────────────────────────────────────────────

MODES = SEGMENT_MODES
# `mpc_docking` before `mpc`: the alternation takes the first that matches.
_LINE = re.compile(r"segment mode: (mpc_docking|mpc|closed_form)\b")
_SEARCH_LINE = re.compile(r"search mode: (grid|nlp)\b")


def logged_modes(log_text: str) -> set[str]:
    """The segment modes the log's ``segment mode: <mode>`` lines name."""
    return set(_LINE.findall(log_text))


def logged_search_modes(log_text: str) -> set[str]:
    """The search modes the log's ``search mode: <mode>`` lines name."""
    return set(_SEARCH_LINE.findall(log_text))


def check_search(log_text: str, expected: str) -> str | None:
    """``None`` when the log names search mode ``expected`` and no other, else why not."""
    if expected not in SEARCH_MODES:
        return f"expected search mode {expected!r} is not one of {SEARCH_MODES}"
    seen = logged_search_modes(log_text)
    if expected not in seen:
        return (
            f"no 'search mode: {expected}' line in the log (search modes logged: {sorted(seen)})"
        )
    if seen - {expected}:
        return f"the log also names {sorted(seen - {expected})}, expected only {expected}"
    return None


def check(log_text: str, expected: str) -> str | None:
    """``None`` when the log names segment mode ``expected`` and no other, else why not."""
    if expected not in MODES:
        return f"expected mode {expected!r} is not one of {MODES}"
    seen = logged_modes(log_text)
    if expected not in seen:
        return f"no 'segment mode: {expected}' line in the log (modes logged: {sorted(seen)})"
    if seen - {expected}:
        return f"the log also names {sorted(seen - {expected})}, expected only {expected}"
    return None


# ── abnormal start ───────────────────────────────────────────────────────────

# The viewer's keys that change what the simulator computes, as viewer_callbacks.cpp
# (rtc_mujoco_sim) logs them: G, N, I, S, ] and [. The camera, frame and rendering
# keys log too and change nothing a throw depends on. Space (pause), +/- (speed),
# R (reset), K (ball launch) and O (object) log nothing: the symptom check and the
# driver's host watch are what is left for those.
_VIEWER_PHYSICS = re.compile(
    r"\[Viewer\] (?:World gravity (?:ON|OFF)|Contacts (?:ENABLED|DISABLED)"
    r"|Integrator → \S+|Solver → \S+|Solver iterations → \d+)"
)


def viewer_interference(log_text: str) -> list[str]:
    """What the launch log says a key in the viewer did to the physics, in log order
    (``[Viewer] World gravity OFF``, …). A key pressed twice is still two lines: the
    throws between them flew in another sim."""
    return [match.group(0) for match in _VIEWER_PHYSICS.finditer(log_text)]


# The driver's line per finished throw (catching_sim_trials: TrialDriver.throw).
_THROW_LINE = re.compile(r"trial (\d+): modes=\[([^\]]*)\] outcome=(\w+) final_mode=(\w+)")
# A throw's mode list this long is not a throw. A cycle is a dozen transitions:
# the longest list among the 4,527 throws of the units that ran normally on
# 2026-10-11 and 10-12 (both robots, both arms) has 15 entries; the 45 flapping
# throws of the abnormal unit have 159 to 173 (its other 37, all ABORTED, 10 at most).
FLAP_MODES = 40
# How many of the driver's first throws the symptom is looked for in.
ABNORMAL_START_THROWS = 5


def throw_mode_counts(trials_log_text: str) -> list[tuple[int, int, str]]:
    """``(throw index, number of modes, outcome)`` of every throw the driver's log has
    finished, in log order."""
    out = []
    for idx, modes, outcome, _ in _THROW_LINE.findall(trials_log_text):
        out.append((int(idx), len(re.findall(r"'[A-Z_]+'", modes)), outcome))
    return out


def flapping_start(
    trials_log_text: str, *, flap_modes: int = FLAP_MODES, throws: int = ABNORMAL_START_THROWS
) -> tuple[str, str | None]:
    """Is the unit's start the flapping one? ``(state, why)``:

    * ``("failed", why)`` as soon as one of the first ``throws`` finished throws went
      through ``flap_modes`` modes or more — the ball of that throw never ended;
    * ``("ready", None)`` once ``throws`` throws are finished and none did;
    * ``("waiting", None)`` before that.

    Only the count is read, never the outcome: an ABORTED throw is something the arm
    did, and a unit ended on it would choose which outcomes are recorded.
    """
    counts = throw_mode_counts(trials_log_text)[:throws]
    for idx, n_modes, outcome in counts:
        if n_modes >= flap_modes:
            return FAILED, (
                f"throw {idx} went through {n_modes} modes (outcome {outcome}); a throw's "
                f"cycle is under {flap_modes}"
            )
    return (READY if len(counts) >= throws else WAITING), None


# ── the two scripts run_unit.sh calls ────────────────────────────────────────


def _read(path: str) -> str:
    with open(path, encoding="utf-8", errors="replace") as handle:
        return handle.read()


def startup_main(argv: list[str]) -> int:
    """``check_startup.py <launch.log>``: exit 0 ready, 1 failed (the first failure line on
    stderr), 2 neither yet — a log that cannot be read is "not yet" (the launch has not
    written it)."""
    if len(argv) != 2:
        print(f"usage: {argv[0]} <launch.log>", file=sys.stderr)
        return 2
    try:
        text = _read(argv[1])
    except OSError:
        return 2
    state = startup_state(text)
    if state == FAILED:
        print(failure_lines(text)[0], file=sys.stderr)
    return {READY: 0, FAILED: 1, WAITING: 2}[state]


def segment_mode_main(argv: list[str]) -> int:
    """``check_segment_mode.py <launch.log> <mode> [--search <mode>]``: exit 0 when the log
    names exactly the expected segment mode (and, with ``--search``, search mode), 1
    otherwise (the reason on stderr), 2 on a bad command line."""
    args = list(argv)
    search = None
    if len(args) == 5 and args[3] == "--search":
        search = args[4]
        args = args[:3]
    if len(args) != 3:
        print(
            f"usage: {argv[0]} <launch.log> <mpc|closed_form|mpc_docking> [--search <grid|nlp>]",
            file=sys.stderr,
        )
        return 2
    try:
        text = _read(argv[1])
    except OSError as err:
        print(f"cannot read {argv[1]}: {err}", file=sys.stderr)
        return 1
    why = check(text, args[2])
    if why is None and search is not None:
        why = check_search(text, search)
    if why:
        print(why, file=sys.stderr)
        return 1
    return 0
