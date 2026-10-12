"""Throw the throw population's chunks at the sim and collect what was caught (#821, stage 2).

    ros2 run integrated_bringup catch_population_trials \\
        --config <catch_population.yaml> --out <result folder> --estimator-ws <colcon ws>

Stage 1 (``rtc_tools catch_population``) left, per arm, the throws to throw as
chunk files (``<result>/<arm>/throws/chunk_NNN.json``). This command throws
them: for every estimator seed and every arm, **one chunk is one unit is one
sim bringup**, and what the finished units caught becomes the arm's
``sim_throws.csv`` and ``throw_catch_set.json`` (``catch_population_collect``).
The configuration file and the result folder are stage 1's
(``rtc_tools.utils.catch_population``); the robot is the folder the file is in.

A unit, in order — every step is what ``tools/catching_eval/run_unit.sh`` does,
whose header is the history of why:

1. **Not in the minute of the previous launch**, nor in a minute that already
   has a session: a session directory is minute-named, and two bringups in one
   would share it.
2. **Bringup**: ``ros2 launch integrated_bringup sim_<robot>.launch.py`` with the
   arm's two modes as launch arguments (``integrated_bringup.catch_modes`` — no
   overlay file), the viewer per ``sim.viewer``, the sim lanes on, no CPU shield
   and ``max_log_sessions:=60`` (the launch prunes the session tree; a long run
   must not lose the session of the unit before).
3. **Readiness** (``catching_startup.startup_state``), up to 120 s.
4. **The mirror is read back before a single throw.** The launch arguments
   reach the controller through a path that creates whatever key it is given, so
   the unit asks the running controller what it loaded: the two modes, and the
   values that must be the SHIPPED ones — the lead (``T_arm``, ``lead_enable``),
   the commit window (``T_freeze``), the CLIK form, the arm's segment grid
   (``n_pre_max``; the docking planner's stop ``n_nodes`` / ``dt_s``) and the
   sim's ball. What "shipped" is comes from the installed configuration
   (:func:`shipped_expectation`), not from a table in this file. The startup
   lines must say the same modes and grid (:func:`mirror_failures`).
5. **Estimator**: ``ball_perception_sim``'s ``sim_estimator.launch.py`` from its
   OWN workspace, sourced in the shell that starts it (two workspaces, MD-17);
   the seed is the estimator's alone — the sim takes none and the throws are a
   file, so two units of one chunk under two seeds differ in the estimator's
   noise only.
6. **Controller switch**, then ``catching_sim_trials --throws-file <chunk>``.
   While it throws, the unit is watched (:func:`watch_verdict`): the driver's
   time limit, an abnormal start (``catching_startup``), a sim that left.
7. **Shutdown** in a fixed order — driver, estimator, sim — each group with
   SIGINT and a wait before SIGKILL, then whatever of the three executables is
   left (:func:`residual_pids`).
8. **The session** the sim wrote is checked for foreign files (a test run that
   started in the launch's minute writes fixture CSVs into the same directory),
   copied into the unit and removed. A unit that fails keeps its session as
   ``session_failed``.

A unit directory holds what ``run_unit.sh`` leaves (``status`` = ``DONE`` |
``FAIL:<why>``, ``conditions.txt``, ``mirror.txt``, ``launch.log``, ``est.log``,
``trials.log``, ``trials/``, ``session/``, ``ct/`` …), so the tools that read a
unit read these. ``launch_overrides.yaml`` is the launch arguments once more in
overlay form: ``conditions.txt`` names it as the unit's ``overlay``, which is
where ``summarize.py --unit`` looks for what the unit changed.

**Resume and retry** (``run_all.sh``'s rule). A unit whose status is ``DONE`` is
skipped. A failed unit directory is moved aside (``<chunk>.failN``, kept as
evidence) and thrown again — once when the command is started (a person's
decision), and after that only for a RIG failure, at most ``sim.retries``
bringups: ``host_busy`` (the sim under 0.95 of real time), ``estimator not
activated``, ``startup`` and ``abnormal_start``. A rig failure does not depend
on what the arm did with a throw; any other failure stays where it failed and
the run goes on — throwing a unit again for a reason its own throws could cause
would choose which outcomes are recorded. Units are ordered seed by seed, and
inside a seed chunk k of every arm before chunk k+1, so a run that ends early
has every arm. A file ``<result>/STOP`` ends the run before the next bringup;
SIGINT / SIGTERM / SIGHUP end the running unit (``FAIL:signal``) and everything
it started.

**Installed packages are enough.** No git repository is needed: the revisions
written into ``conditions.txt`` are ``unknown`` when there is no source tree to
ask, and a dirty tree is recorded, not refused. The environment is the caller's
(ROS and this workspace already sourced); only the estimator's workspace is
named (``--estimator-ws`` or ``BALL_SIM_WS``). Nothing is read from a source
path: the shipped values come from the package share, the session directory
from the launch's own log line and logging root (``rtc_tools.utils.session_dir``).

The analysis (``catching_tc_vector``) runs after each unit, when no sim is up,
and never beside one. ``--collect-only`` throws nothing: it analyses what is
not analysed and rebuilds the two result files from the unit directories
present — which is also how units copied in from another PC are merged.

Everything a test can ask without a sim is a function of its arguments: the
plan, the command lines, the expectations and their verdict, the watch verdict,
the retry step, the session checks. The process handling at the bottom is thin.
"""

from __future__ import annotations

import argparse
import contextlib
import dataclasses
import datetime
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from collections.abc import Mapping, Sequence
from pathlib import Path

import yaml

from integrated_bringup import catching_startup as startup
from integrated_bringup.catch_modes import (
    CONTROLLER as CATCHING,
    SEARCH_ARGUMENT,
    SEGMENT_ARGUMENT,
    catch_mode_overrides,
)
from integrated_bringup.catch_population_collect import (
    DONE,
    CollectError,
    analyse_unit,
    collect_arm,
    needs_analysis,
    read_conditions,
    sha256_file,
    sim_root,
    unit_status,
    units_present,
)

PACKAGE = "integrated_bringup"
JOINT = "demo_joint_controller"
CONTROLLER_NODE = f"/{CATCHING}/{CATCHING}"
SIMULATOR_NODE = "/mujoco_simulator"
BALL_TYPE_PARAMETER = "projectile_ball.ball_type"
ESTIMATOR_PACKAGE = "ball_perception_sim"
ESTIMATOR_PROFILE = ("share", ESTIMATOR_PACKAGE, "config", "sim_profile.catching.json")
ESTIMATOR_SOURCE = ("src", "ball_perception")
UNKNOWN = "unknown"
DEFAULT_DOMAIN_ID = 88
MAX_LOG_SESSIONS = 60

STARTUP_TIMEOUT_S = 120
ESTIMATOR_TIMEOUT_S = 15
SWITCH_TIMEOUT_S = 30
DRIVER_STOP_S = 10
ESTIMATOR_STOP_S = 20
LAUNCH_STOP_S = 20
WATCH_PERIOD_S = 2.0

# Statuses a unit is thrown again for (the module docstring says why these and no other).
RIG_FAILURES = ("host_busy", "estimator not activated", "startup", "abnormal_start")
RETRY_ON = re.compile(r"^FAIL:(" + "|".join(re.escape(why) for why in RIG_FAILURES) + r")$")
EXIT_HOST_BUSY = 3  # catching_sim_trials: --host-watch abort
EXIT_TIMEOUT = 124  # what `timeout` returns, kept as the status of a driver that ran out

# The controller mirrors a unit records (mirror.txt), in run_unit.sh's order. A name the
# running selection does not declare is recorded as "Parameter not set".
MIRROR_NAMES = (
    "joint_cmd.lag.T_arm",
    "joint_cmd.lag.lead_enable",
    "planner.freeze.T_freeze",
    "reference.omega",
    "reference.a_max",
    "reference.v_max",
    "control.dt",
    "prediction.dt_expected",
    "io.n_min",
    "planner.search.grid.slice.dt",
    "planner.segment.mpc.horizon.n_nodes",
    "planner.segment.mpc.horizon.dt_s",
    "planner.segment.mpc.approach.n_pre_max",
    "planner.segment.mpc.approach.dt_pre_s",
    "planner.segment.mpc.approach.rest_tol",
    "planner.segment.mpc.replan.k_max",
    "planner.segment.mpc.replan.same_point",
    "planner.segment.mpc.budget.first_s",
    "planner.segment.mpc.budget.replan_s",
    "planner.segment.mpc.publish.catch_pos_err_max",
    "planner.segment.mpc.publish.slack_max",
    "planner.segment.mpc.eta_tau",
    "planner.segment.mpc.m_q",
    "planner.segment.mpc.catch.gamma_ref",
    "planner.segment.mpc.catch.w_v_par",
    "planner.segment.mpc.catch.w_v_perp",
    "planner.segment.mpc.catch.w_axis",
    "planner.segment.mpc.catch.kappa",
    "planner.segment.mpc.catch.sigma_floor",
    "planner.segment.mpc.catch.w_max",
    "planner.segment.mpc.catch.w_const",
    "planner.segment.mpc.catch.sigma_ref",
    "planner.search.grid.gamma.eta_v",
    "planner.search.grid.time.margin",
    "planner.search.grid.slice.t_lead_min",
    "joint_cmd.accel_constraint",
    "planner.segment.mode",
    "planner.search.mode",
    "planner.segment.mpc.switch_margin",
    "planner.segment.mpc_docking.approach.n_pre_max",
    "planner.segment.mpc_docking.stop.n_nodes",
    "planner.segment.mpc_docking.stop.dt_s",
    "planner.segment.mpc_docking.budget.first_s",
    "planner.segment.mpc_docking.budget.replan_s",
    "planner.search.nlp.budget.budget_s",
    "planner.search.nlp.budget.solve_s",
    "planner.search.nlp.budget.max_solves",
    "planner.segment.mpc.eta_v",
    "planner.search.grid.reference.omega",
    "planner.search.grid.reference.a_max",
    "planner.search.grid.reference.v_max",
)


class Interrupted(BaseException):
    """A signal ended the run (raised from the handler, in the main thread)."""


# ── the plan ─────────────────────────────────────────────────────────────────


@dataclasses.dataclass(frozen=True)
class UnitSpec:
    """One unit to throw: an arm's chunk under one estimator seed."""

    arm: object  # rtc_tools.utils.catch_population.Arm
    seed: int
    chunk: Path  # the chunk file
    unit_dir: Path
    n_throws: int  # what the unit throws: the file's count, or the limit when lower

    @property
    def name(self) -> str:
        return f"{self.arm.name}/seed_{self.seed}/{self.chunk.stem}"


def chunk_index(chunk: Path) -> int:
    """``chunk_003.json`` → 3."""
    return int(chunk.stem.rsplit("_", 1)[1])


def plan_units(
    arms: Sequence,
    seeds: Sequence[int],
    chunks: Mapping[str, Sequence[Path]],
    counts: Mapping[Path, int],
    unit_dir,
    *,
    max_chunks: int | None = None,
    throws_limit: int | None = None,
) -> list[UnitSpec]:
    """The units of a run, in the order they are thrown.

    Seed by seed; inside a seed, chunk k of every arm (in ``arms`` order) before
    chunk k+1 — a run that ends early then has every arm. ``chunks`` maps an
    arm's name to its chunk files, ``counts`` a chunk file to its throw count,
    ``unit_dir(arm name, seed, chunk stem)`` is the layout's. ``max_chunks`` keeps
    the chunks whose index is below it; ``throws_limit`` cuts every unit.
    """
    indices = sorted({chunk_index(c) for arm in arms for c in chunks.get(arm.name, ())})
    if max_chunks is not None:
        indices = [k for k in indices if k < max_chunks]
    by_index = {arm.name: {chunk_index(c): c for c in chunks.get(arm.name, ())} for arm in arms}
    units = []
    for seed in seeds:
        for k in indices:
            for arm in arms:
                chunk = by_index[arm.name].get(k)
                if chunk is None:
                    continue
                n = counts[chunk]
                if throws_limit is not None:
                    n = min(n, throws_limit)
                units.append(UnitSpec(arm, seed, chunk, unit_dir(arm.name, seed, chunk.stem), n))
    return units


def next_step(status: str | None, attempt: int) -> str:
    """What the ``attempt``-th look (from 1) at a unit of ``status`` does.

    ``"done"`` — it is DONE; ``"run"`` — throw it (a failed directory is moved
    aside first); ``"stay"`` — it failed for a reason that is not a rig failure
    and this run already threw it: it is left where it failed. The first look
    throws whatever is not DONE: starting the command again is a person's
    decision to throw a failed unit anew.
    """
    if status == DONE:
        return "done"
    if attempt > 1 and not RETRY_ON.match(status or ""):
        return "stay"
    return "run"


def failed_name(unit_dir: Path) -> Path:
    """The first free ``<unit>.failN`` beside ``unit_dir``."""
    k = 1
    while unit_dir.with_name(f"{unit_dir.name}.fail{k}").exists():
        k += 1
    return unit_dir.with_name(f"{unit_dir.name}.fail{k}")


def done_for_another_run(unit_dir: Path, spec: UnitSpec) -> str | None:
    """Why a DONE unit is not this run's unit, or None. Such a unit is neither
    skipped as thrown nor thrown over: a person moves it."""
    cond = read_conditions(unit_dir)
    if cond.get("n") != str(spec.n_throws):
        return f"it threw n={cond.get('n')}, this run throws {spec.n_throws} (--throws-limit?)"
    if cond.get("throws_file_sha256") != sha256_file(spec.chunk):
        return f"it threw another file than this folder's {spec.chunk.name}"
    return None


def stage1_mismatch(config, copy_path: Path) -> list[str]:
    """The sections of ``config`` that differ from the copy stage 1 left in the result
    folder (``config.yaml``), ``sim`` apart — that one is this stage's, and a PC may
    throw other seeds. Empty when there is no copy to compare with."""
    from rtc_tools.utils.catch_population import ConfigError, parse_config

    if not copy_path.is_file():
        return []
    try:
        document = yaml.safe_load(copy_path.read_text(encoding="utf-8"))
        judged = parse_config(document, config.path)
    except (OSError, yaml.YAMLError, ConfigError) as exc:
        return [f"{copy_path.name} cannot be read as a configuration ({exc})"]
    sections = ("arms", "ball", "timing", "judge", "population", "sample")
    return [name for name in sections if getattr(judged, name) != getattr(config, name)]


# ── what the controller must have loaded ─────────────────────────────────────


@dataclasses.dataclass(frozen=True)
class Expectation:
    """The mirror values and startup lines one arm's unit must show."""

    search: str
    segment: str
    t_arm: float
    lead_enable: bool
    t_freeze: float
    accel_constraint: str
    ball_type: str
    # This arm's segment planner (None for closed_form, which has none): the approach
    # grid, and the node count the mirror shows — the horizon's for mpc, the stop
    # part's for mpc_docking, whose stop step is checked as well.
    n_pre_max: int | None = None
    dt_pre_s: float | None = None
    n_nodes: int | None = None
    stop_dt_s: float | None = None

    @property
    def mirror_prefix(self) -> str:
        return f"planner.segment.{self.segment}"

    @property
    def n_nodes_mirror(self) -> str:
        part = "horizon" if self.segment == "mpc" else "stop"
        return f"{self.mirror_prefix}.{part}.n_nodes"

    @property
    def grid_line(self) -> str | None:
        """The startup line that says this arm's approach grid."""
        if self.n_pre_max is None:
            return None
        said = f"up to {self.n_pre_max} x {self.dt_pre_s:.3f} s"
        if self.segment == "mpc":
            return f"MPC segment planner approach grid: {said}"
        return f"mpc_docking segment planner ready: {said}"


def _leaf(tree: Mapping, path: Sequence[str], source: str):
    node = tree
    for part in path:
        if not isinstance(node, Mapping) or part not in node:
            raise ValueError(f"{source}: no key {'.'.join(path)}")
        node = node[part]
    return node


def shipped_ball_type(simulator_yaml: Path) -> str:
    """``projectile_ball.ball_type`` of a robot's shipped simulator configuration."""
    with open(simulator_yaml, encoding="utf-8") as handle:
        document = yaml.safe_load(handle) or {}
    for node in document.values():
        params = node.get("ros__parameters") if isinstance(node, Mapping) else None
        ball = params.get("projectile_ball") if isinstance(params, Mapping) else None
        if isinstance(ball, Mapping) and "ball_type" in ball:
            return str(ball["ball_type"])
    raise ValueError(f"{simulator_yaml}: no projectile_ball.ball_type")


def shipped_expectation(config_dir: Path, simulator_yaml: Path, arm) -> Expectation:
    """What a unit of ``arm`` must read back: the shipped values of ``config_dir`` (a
    robot's installed configuration folder) with the arm's two modes.

    The controller file is read composed, as the controller manager hands it over
    (``load_controller_config``: the main file plus its ``include:`` fragments —
    the stop grid lives in one). ``ValueError`` names a key that is not there: a
    unit that cannot say what it expects is not started.
    """
    from rtc_tools.utils.controller_config import load_controller_config

    path = Path(config_dir) / "controllers" / f"{CATCHING}.yaml"
    catching = load_controller_config(path, config_key=CATCHING)[CATCHING].get("catching") or {}
    source = str(path)
    planner: dict = {}
    if arm.segment != "closed_form":
        segment = ("planner", "segment", arm.segment)
        nodes = ("horizon",) if arm.segment == "mpc" else ("stop",)
        planner = {
            "n_pre_max": int(_leaf(catching, (*segment, "approach", "n_pre_max"), source)),
            "dt_pre_s": float(_leaf(catching, (*segment, "approach", "dt_pre_s"), source)),
            "n_nodes": int(_leaf(catching, (*segment, *nodes, "n_nodes"), source)),
        }
        if arm.segment == "mpc_docking":
            planner["stop_dt_s"] = float(_leaf(catching, (*segment, "stop", "dt_s"), source))
    return Expectation(
        search=arm.search,
        segment=arm.segment,
        t_arm=float(_leaf(catching, ("joint_cmd", "lag", "T_arm"), source)),
        lead_enable=bool(_leaf(catching, ("joint_cmd", "lag", "lead_enable"), source)),
        t_freeze=float(_leaf(catching, ("planner", "freeze", "T_freeze"), source)),
        accel_constraint=str(_leaf(catching, ("joint_cmd", "accel_constraint"), source)),
        ball_type=shipped_ball_type(simulator_yaml),
        **planner,
    )


def _same_number(got, want) -> bool:
    if isinstance(got, bool) or not isinstance(got, int | float) or want is None:
        return False
    return abs(float(got) - float(want)) < 1e-9


_BUDGET_LINE = re.compile(r"planner enabled: wake timeout [0-9.]+ s, budget ([0-9.]+) s")
FOLLOWS_SEGMENTS_LINE = "takes a plan with its first segment"


def planner_budget(log_text: str) -> str | None:
    """The planner's search budget [s] as its startup line says it (not a parameter:
    only ``planner enabled: … budget X s`` does)."""
    match = _BUDGET_LINE.search(log_text)
    return match.group(1) if match else None


def mirror_failures(mirror: Mapping, ball_type, launch_log: str, expect: Expectation) -> list[str]:
    """What of ``expect`` the running controller does not show — empty for a unit that
    may throw. ``mirror`` is ``{mirror name: value | None}`` as read off the controller,
    ``ball_type`` the simulator's parameter, ``launch_log`` the bringup's log.

    The words are ``run_unit.sh``'s: ``T_arm`` ``lead`` ``ball_type`` ``T_freeze``
    ``clik_form`` ``mode_mirror`` ``search_mirror`` (the mirror), ``mode_log`` (the
    ``segment mode:`` / ``search mode:`` startup lines), ``n_pre_max`` ``n_nodes``
    ``stop_dt`` (the arm's segment grid), ``approach_grid`` ``not_e1f09_binary`` (the
    planner's startup lines) and ``no_budget_line``.
    """
    why = []
    if not _same_number(mirror.get("joint_cmd.lag.T_arm"), expect.t_arm):
        why.append("T_arm")
    if mirror.get("joint_cmd.lag.lead_enable") is not expect.lead_enable:
        why.append("lead")
    if ball_type != expect.ball_type:
        why.append("ball_type")
    if not _same_number(mirror.get("planner.freeze.T_freeze"), expect.t_freeze):
        why.append("T_freeze")
    if mirror.get("joint_cmd.accel_constraint") != expect.accel_constraint:
        why.append("clik_form")
    if mirror.get("planner.segment.mode") != expect.segment:
        why.append("mode_mirror")
    if mirror.get("planner.search.mode") != expect.search:
        why.append("search_mirror")
    if startup.check(launch_log, expect.segment) or startup.check_search(
        launch_log, expect.search
    ):
        why.append("mode_log")
    if expect.n_pre_max is not None:
        got = mirror.get(f"{expect.mirror_prefix}.approach.n_pre_max")
        if isinstance(got, bool) or got != expect.n_pre_max:
            why.append("n_pre_max")
        got = mirror.get(expect.n_nodes_mirror)
        if isinstance(got, bool) or got != expect.n_nodes:
            why.append("n_nodes")
        if expect.stop_dt_s is not None and not _same_number(
            mirror.get(f"{expect.mirror_prefix}.stop.dt_s"), expect.stop_dt_s
        ):
            why.append("stop_dt")
        if expect.grid_line not in launch_log:
            why.append("approach_grid")
        if FOLLOWS_SEGMENTS_LINE not in launch_log:
            why.append("not_e1f09_binary")
    if planner_budget(launch_log) is None:
        why.append("no_budget_line")
    return why


def ros2_param_text(value) -> str:
    """A value as ``ros2 param get`` prints it — the format of ``mirror.txt``, which the
    unit tools parse (``<name>: String value is: nlp``)."""
    if value is None:
        return "Parameter not set"
    if isinstance(value, bool):
        return f"Boolean value is: {value}"
    if isinstance(value, int):
        return f"Integer value is: {value}"
    if isinstance(value, float):
        return f"Double value is: {value}"
    if isinstance(value, str):
        return f"String value is: {value}"
    return f"Double values are: {list(value)}"


def mirror_text(mirror: Mapping, ball_type) -> str:
    """``mirror.txt``: every recorded mirror, then the simulator's ball."""
    lines = [f"{name}: {ros2_param_text(mirror.get(name))}" for name in MIRROR_NAMES]
    lines.append(f"ball_type: {ros2_param_text(ball_type)}")
    return "\n".join(lines) + "\n"


def overrides_as_overlay(overrides: Mapping[str, str]) -> dict:
    """The launch's controller overrides (``catch_modes``) as the params tree an overlay
    file would hold — the unit's record of what it changed, in the form the unit tools
    read (``launch_overrides.yaml``)."""
    tree: dict = {}
    for key, value in overrides.items():
        node = tree
        parts = key.split(".")
        for part in parts[:-1]:
            node = node.setdefault(part, {})
        node[parts[-1]] = value
    return {"integrated_rt_controller": {"ros__parameters": tree}}


# ── command lines ────────────────────────────────────────────────────────────


def launch_file(robot: str) -> str:
    return f"sim_{robot}.launch.py"


def launch_command(robot: str, arm, viewer: bool) -> list[str]:
    """The sim bringup of one unit."""
    return [
        "ros2",
        "launch",
        PACKAGE,
        launch_file(robot),
        f"enable_viewer:={'true' if viewer else 'false'}",
        "use_cpu_affinity:=false",
        "enable_mpc:=true",
        "sim_lanes:=true",
        f"{SEARCH_ARGUMENT}:={arm.search}",
        f"{SEGMENT_ARGUMENT}:={arm.segment}",
        f"max_log_sessions:={MAX_LOG_SESSIONS}",
    ]


# Runs in a shell of its own: the estimator's workspace is sourced there and nowhere
# else. `ros2 pkg prefix` must then name that workspace (its package folder, or the
# install root of a merged install) — an estimator found elsewhere on the path is
# another build. $1 is the workspace; the rest are the launch arguments.
_ESTIMATOR_SCRIPT = f"""
source "$1/install/local_setup.bash"
prefix=$(ros2 pkg prefix {ESTIMATOR_PACKAGE})
[ "$prefix" == "$1/install/{ESTIMATOR_PACKAGE}" ] || [ "$prefix" == "$1/install" ] \\
  || {{ echo "WRONG_PREFIX $prefix"; exit 1; }}
exec ros2 launch {ESTIMATOR_PACKAGE} sim_estimator.launch.py "${{@:2}}"
"""
WRONG_PREFIX = "WRONG_PREFIX"
ESTIMATOR_ACTIVE = "activated epoch"


def estimator_command(
    workspace: Path, profile: Path, seed: int, *, producer_revision: str, simulator_revision: str
) -> list[str]:
    """The estimator of one unit, from its own workspace."""
    return [
        "bash",
        "-c",
        _ESTIMATOR_SCRIPT,
        "estimator",
        str(workspace),
        f"profile_path:={profile}",
        f"producer_revision:={producer_revision}",
        f"simulator_revision:={simulator_revision}",
        f"simulator_seed:={seed}",
    ]


def estimator_profile(workspace: Path) -> Path:
    """The estimator's shipped catching profile in ``workspace`` (an isolated or a
    merged install); the isolated path when neither is there, for the message."""
    install = Path(workspace) / "install"
    isolated = install.joinpath(ESTIMATOR_PACKAGE, *ESTIMATOR_PROFILE)
    merged = install.joinpath(*ESTIMATOR_PROFILE)
    return merged if merged.is_file() and not isolated.is_file() else isolated


def switch_command() -> list[str]:
    request = (
        f"{{activate_controllers: [{CATCHING}], deactivate_controllers: [{JOINT}], "
        "strictness: 1, timeout: {sec: 3}}"
    )
    return [
        "ros2",
        "service",
        "call",
        "/rtc_cm/switch_controller",
        "rtc_msgs/srv/SwitchController",
        request,
    ]


def switched(reply_text: str) -> bool:
    """Whether ``ros2 service call``'s output carries the switch's ``ok=True``."""
    return "ok=True" in reply_text


def driver_command(
    trials_dir: Path,
    robot: str,
    chunk: Path,
    arm_label: str,
    *,
    limit: int | None,
    end_on_ball_low: bool,
    host_watch: str,
) -> list[str]:
    """``catching_sim_trials`` over one chunk file."""
    command = [
        "ros2",
        "run",
        PACKAGE,
        "catching_sim_trials",
        str(trials_dir),
        "--profile",
        robot,
        "--throws-file",
        str(chunk),
    ]
    if limit is not None:
        command += ["--limit", str(limit)]
    command += ["--host-watch", host_watch, "--arm", arm_label]
    if end_on_ball_low:
        command.append("--end-on-ball-low")
    return command


def driver_timeout_s(n_throws: int) -> int:
    """How long a unit's driver may run: 30 s a throw (a throw is 12 s at the cap) and
    two minutes."""
    return n_throws * 30 + 120


# ── what the logs and the session say ────────────────────────────────────────

_SESSION_LINE = re.compile(r"Session dir: (/\S*/\d{6}_\d{4})\b")
_SESSION_NAME = re.compile(r"logging_data/(\d{6}_\d{4})\b")
_COMMIT_LINE = re.compile(r"commit at t_c − [0-9.]* s")
_SEGMENT_STARTUP = re.compile(
    r"MPC segment planner (?:ready|approach grid)|mpc_docking segment planner ready"
)


def session_dir_from_log(log_text: str, logging_root: str) -> str | None:
    """The session directory of the bringup whose log this is.

    The RT node's own line (``Session dir: <path>``) when it is there; otherwise
    the first ``logging_data/<YYMMDD_HHMM>`` the log names, under ``logging_root``
    — the launch's root, which a caller in the launch's environment and directory
    gets from ``rtc_tools.utils.session_dir.resolve_logging_root``. None when the
    log names no session: the caller must not then copy or remove anything.
    """
    match = _SESSION_LINE.search(log_text)
    if match:
        return match.group(1)
    match = _SESSION_NAME.search(log_text)
    if match:
        return os.path.join(logging_root, match.group(1))
    return None


def commit_line(log_text: str) -> str:
    match = _COMMIT_LINE.search(log_text)
    return match.group(0) if match else ""


def segment_startup_lines(log_text: str) -> list[str]:
    """The segment planner's startup lines, without the logger prefix."""
    return [
        re.sub(rf"^.*{CATCHING}\]: ", "", line)
        for line in log_text.splitlines()
        if _SEGMENT_STARTUP.search(line)
    ]


def state_log_names(controller_tree: Mapping) -> tuple[str, ...]:
    """The device state CSVs the catching controller writes: one per device of its
    ``topics`` (``<device>_state.csv``)."""
    return tuple(f"{device}_state.csv" for device in (controller_tree.get("topics") or {}))


CONTROLLER_LOGS = ("catching_diag.csv", "planner_events.csv", "nlp_candidates.csv")


def session_contamination(session_dir: Path, state_logs: Sequence[str]) -> str | None:
    """Why the catching controller's logs in ``session_dir`` are not one unit's, or None.

    A ``colcon test`` that starts in the launch's minute can write fixture CSVs into
    the same time-named session: a CSV whose first row does not have its header's
    field count, or a file that is not one of the controller's logs, refuses the
    unit. The words are ``run_unit.sh``'s.
    """
    folder = Path(session_dir) / "controllers" / CATCHING
    if not folder.is_dir():
        return None
    known = {*CONTROLLER_LOGS, *state_logs}
    for path in sorted(folder.iterdir()):
        if path.suffix == ".csv" and path.is_file():
            with open(path, encoding="utf-8", errors="replace") as handle:
                header, row = handle.readline(), handle.readline()
            fields, first = header.rstrip("\n").count(","), row.rstrip("\n").count(",")
            if row.strip() and fields != first:
                return f"contaminated {path.name} header {fields + 1} rows {first + 1}"
    for path in sorted(folder.iterdir()):
        if path.name not in known:
            return f"contaminated foreign file {path.name}"
    return None


def _viewer_key(keys: Sequence[str]) -> str:
    return f"a key in the viewer changed the sim: {keys[0]}"


def _error_status(exc: BaseException) -> str:
    return (f"FAIL:error {type(exc).__name__}: {exc}".splitlines() or ["FAIL:error"])[0]


def watch_verdict(
    *,
    driver_rc: int | None,
    elapsed_s: float,
    timeout_s: float,
    viewer_keys: Sequence[str],
    start: tuple[str, str | None],
    sim_alive: bool,
) -> tuple[str, str | None]:
    """What the unit does at one look while its driver throws: ``(what, why)``.

    ``("abnormal", why)`` — a physics key in the viewer (``viewer_keys``: the launch
    log's lines so far) or a flapping start (``start``:
    ``catching_startup.flapping_start``); ``("ended", None)`` — the driver returned
    (``driver_rc``); ``("sim_exited", …)`` — the sim is gone under a driver that is
    not; ``("timeout", …)``; else ``("go", None)``. The abnormal sign is read first:
    throws thrown in a changed sim are not the unit's, whatever the driver returned.
    """
    if viewer_keys:
        return "abnormal", _viewer_key(viewer_keys)
    if start[0] == startup.FAILED:
        return "abnormal", start[1]
    if driver_rc is not None:
        return "ended", None
    if not sim_alive:
        return "sim_exited", "the sim's launch left while the driver was throwing"
    if elapsed_s >= timeout_s:
        return "timeout", f"the driver ran {timeout_s:.0f} s"
    return "go", None


def residual_pids(
    table: Sequence[tuple[int, int, str]], patterns: Sequence[str], spare: set[int]
) -> list[int]:
    """The processes of ``table`` (``catching_sim_trials.process_table`` rows) whose
    command line STARTS with one of ``patterns`` (regular expressions: an executable
    under a workspace) and that are not in ``spare`` — this command's own family,
    whose arguments may name the same paths."""
    compiled = [re.compile(pattern) for pattern in patterns]
    return [
        pid
        for pid, _, line in table
        if pid not in spare and any(rx.match(line) for rx in compiled)
    ]


def residual_patterns(prefixes: Mapping[str, str], estimator_ws: str) -> list[str]:
    """What is killed if it outlives a unit: the simulator and the RT node of THIS
    workspace (``prefixes``: executable name → the install prefix it runs from) and
    the estimator of its own — never another workspace's, which may run its own."""
    patterns = [
        rf"{re.escape(prefix)}/[^ ]*{re.escape(name)}" for name, prefix in prefixes.items()
    ]
    patterns.append(rf"{re.escape(str(estimator_ws))}/[^ ]*ball_perception")
    return patterns


# ── processes ────────────────────────────────────────────────────────────────


def revision(tree: str | None) -> tuple[str, str, str]:
    """``(short revision, number of changed files, their names)`` of the git tree at
    ``tree`` — ``unknown`` three times when there is none to ask (an installed
    package has no repository)."""
    if not tree or not os.path.isdir(tree):
        return UNKNOWN, UNKNOWN, UNKNOWN

    def git(*words: str) -> str | None:
        try:
            done = subprocess.run(
                ["git", "-C", tree, *words],
                stdin=subprocess.DEVNULL,
                capture_output=True,
                text=True,
                check=False,
            )
        except OSError:
            return None
        return done.stdout if done.returncode == 0 else None

    rev = git("rev-parse", "--short", "HEAD")
    if rev is None or not rev.strip():
        return UNKNOWN, UNKNOWN, UNKNOWN
    dirty = (git("status", "--porcelain") or "").splitlines()
    return rev.strip(), str(len(dirty)), ";".join(dirty)


def own_source_tree() -> str | None:
    """Where this module's file really is, when that is a source tree (a symlink
    install); None for a copied install, which has no repository to ask."""
    from ament_index_python.packages import PackageNotFoundError, get_package_prefix

    here = os.path.dirname(os.path.realpath(__file__))
    try:
        prefix = os.path.realpath(get_package_prefix(PACKAGE))
    except PackageNotFoundError:
        return here
    return None if os.path.commonpath([here, prefix]) == prefix else here


def minute_stamp() -> str:
    """The name a session opened now would get."""
    return datetime.datetime.now().strftime("%y%m%d_%H%M")


def wait_for_free_minute(stamp_file: Path, logging_root: str, *, sleep=time.sleep) -> str:
    """Wait until now is not the minute of the previous launch (``stamp_file``) nor a
    minute ``logging_root`` already has a session of; then write it to ``stamp_file``."""
    try:
        last = stamp_file.read_text(encoding="utf-8").strip()
    except OSError:
        last = ""
    while True:
        now = minute_stamp()
        if now != last and not os.path.isdir(os.path.join(logging_root, now)):
            break
        sleep(2)
    stamp_file.parent.mkdir(parents=True, exist_ok=True)
    stamp_file.write_text(now + "\n", encoding="utf-8")
    return now


class Group:
    """A child in a session of its own, so that it and whatever it starts are one
    process group: the shell leaving does not stop a launch, a signal to the group
    does. Its output goes to ``log``."""

    def __init__(self, command: Sequence[str], log: Path) -> None:
        self.log = log
        with open(log, "wb") as out:
            self.proc = subprocess.Popen(
                list(command),
                stdin=subprocess.DEVNULL,
                stdout=out,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        self.pgid = self.proc.pid

    def poll(self) -> int | None:
        return self.proc.poll()

    def alive(self) -> bool:
        """Whether any process of the group is left (the leader is reaped here)."""
        self.proc.poll()
        try:
            os.killpg(self.pgid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    def stop(self, wait_s: int) -> bool:
        """SIGINT the group, wait ``wait_s`` for it to leave, then SIGKILL. True when it
        had to be killed."""
        if not self.alive():
            return False
        with contextlib.suppress(ProcessLookupError):
            os.killpg(self.pgid, signal.SIGINT)
        for _ in range(wait_s):
            if not self.alive():
                return False
            time.sleep(1)
        with contextlib.suppress(ProcessLookupError):
            os.killpg(self.pgid, signal.SIGKILL)
        with contextlib.suppress(subprocess.TimeoutExpired):
            self.proc.wait(timeout=5)
        return True


class LogTail:
    """A growing log read in whole lines: ``new()`` is what was written since the last
    call, up to the last newline (a line still being written waits for its end)."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self.offset = 0

    def new(self) -> str:
        try:
            with open(self.path, "rb") as handle:
                handle.seek(self.offset)
                data = handle.read()
        except OSError:
            return ""
        end = data.rfind(b"\n") + 1
        self.offset += end
        return data[:end].decode("utf-8", errors="replace")


def _read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""


def loadavg() -> str:
    return " ".join(_read_text(Path("/proc/loadavg")).split()[:3])


def read_mirror(timeout_s: float = 10.0) -> tuple[dict, object]:
    """The running controller's mirror (``{name: value | None}`` over ``MIRROR_NAMES``)
    and the simulator's ball type. ``RuntimeError`` when a node does not answer."""
    # ROS imports stay here so everything above is importable without a ROS graph.
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from rclpy.executors import SingleThreadedExecutor

    from integrated_bringup.catching_sim_trials import _parameter_value, mirror_from_replies

    # A context of its own: nothing of rclpy outlives the read, and rclpy installs no
    # signal handler for a context that is not the default one — the run's own stay.
    context = rclpy.Context()
    rclpy.init(context=context)
    try:
        node = rclpy.create_node("catch_population_trials", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        def parameters(target: str, names: Sequence[str]) -> dict:
            client = node.create_client(GetParameters, f"{target}/get_parameters")
            if not client.wait_for_service(timeout_sec=timeout_s):
                raise RuntimeError(f"{target}/get_parameters is not there")

            def ask(batch):
                request = GetParameters.Request()
                request.names = list(batch)
                future = client.call_async(request)
                executor.spin_until_future_complete(future, timeout_sec=timeout_s)
                reply = future.result() if future.done() else None
                if reply is None:
                    raise RuntimeError(f"{target}/get_parameters did not answer")
                return [(value.type, _parameter_value(value)) for value in reply.values]

            return mirror_from_replies(names, ask)

        mirror = parameters(CONTROLLER_NODE, MIRROR_NAMES)
        ball = parameters(SIMULATOR_NODE, [BALL_TYPE_PARAMETER])[BALL_TYPE_PARAMETER]
        executor.shutdown()
        node.destroy_node()
        return mirror, ball
    finally:
        rclpy.shutdown(context=context)


@dataclasses.dataclass(frozen=True)
class RunContext:
    """What every unit of a run shares."""

    robot: str
    config_path: Path
    config_sha256: str
    share_config_dir: Path  # the installed configuration folder of the robot
    sim_root: Path  # <result>/sim
    viewer: bool
    end_on_ball_low: bool
    host_watch: str
    throws_limit: int | None
    estimator_ws: Path
    profile: Path
    domain_id: int
    logging_root: str
    state_logs: tuple[str, ...]
    residual: tuple[str, ...]
    revisions: Mapping[str, tuple[str, str, str]]  # "rtc_framework" / "ball_perception"


class UnitRun:
    """One unit, from the free minute to its status (the module docstring's steps)."""

    def __init__(self, spec: UnitSpec, expect: Expectation, ctx: RunContext) -> None:
        self.spec, self.expect, self.ctx = spec, expect, ctx
        self.out = spec.unit_dir
        self.launch: Group | None = None
        self.estimator: Group | None = None
        self.driver: Group | None = None
        self.launch_tail: LogTail | None = None
        self.session: str | None = None

    # ── records ──────────────────────────────────────────────────────────
    def _note(self, name: str, text: str) -> None:
        (self.out / name).write_text(text, encoding="utf-8")

    def _condition(self, **lines) -> None:
        with open(self.out / "conditions.txt", "a", encoding="utf-8") as handle:
            for key, value in lines.items():
                handle.write(f"{key}: {value}\n")

    def _write_conditions(self) -> None:
        spec, ctx = self.spec, self.ctx
        overrides = catch_mode_overrides(spec.arm.search, spec.arm.segment)
        overlay = self.out / "launch_overrides.yaml"
        overlay.write_text(
            "# What this unit's launch arguments wrote into the controller configuration\n"
            f"# ({SEARCH_ARGUMENT}, {SEGMENT_ARGUMENT}) — a record in overlay form, not an input.\n"
            + yaml.safe_dump(overrides_as_overlay(overrides), sort_keys=False),
            encoding="utf-8",
        )
        rev, dirty, files = ctx.revisions["rtc_framework"]
        lines = {
            "date_start": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
            "robot": ctx.robot,
            "condition": spec.arm.segment,
            "arm": spec.arm.name,
            "overlay": overlay,
            "launch_arguments": (
                f"{SEARCH_ARGUMENT}:={spec.arm.search} {SEGMENT_ARGUMENT}:={spec.arm.segment}"
            ),
            "n": spec.n_throws,
            "seed": spec.seed,
            "throws_file": spec.chunk,
            "throws_file_sha256": sha256_file(spec.chunk),
        }
        if ctx.throws_limit is not None:
            lines["throws_limit"] = ctx.throws_limit
        if ctx.end_on_ball_low:
            lines["end_on_ball_low"] = "1 margin_m=default grace_s=default"
        lines.update(
            {
                "rtc_framework_rev": rev,
                "rtc_framework_dirty": dirty,
                "rtc_framework_dirty_files": files,
                "ball_perception_rev": ctx.revisions["ball_perception"][0],
                "profile": ctx.profile,
                "profile_sha256": sha256_file(ctx.profile),
                "population_config": ctx.config_path,
                "population_config_sha256": ctx.config_sha256,
                "loadavg_start": loadavg(),
                "ros_domain_id": ctx.domain_id,
                "expect_mode": spec.arm.segment,
                "expect_kv": "",
                "expect_search": spec.arm.search,
                "expect_ball": self.expect.ball_type,
                "viewer": "on" if ctx.viewer else "off",
                "probe_dump": "off",
                "host_watch": ctx.host_watch,
            }
        )
        self._note("conditions.txt", "")
        self._condition(**lines)

    # ── shutdown ─────────────────────────────────────────────────────────
    def cleanup(self) -> None:
        """Stop what the unit started, in order, then kill what is left of it."""
        from integrated_bringup.catching_sim_trials import process_family, process_table

        for group, wait_s in (
            (self.driver, DRIVER_STOP_S),
            (self.estimator, ESTIMATOR_STOP_S),
            (self.launch, LAUNCH_STOP_S),
        ):
            if group is not None and group.stop(wait_s):
                with open(self.out / "cleanup.log", "a", encoding="utf-8") as handle:
                    handle.write(f"killed group {group.pgid} after {wait_s} s\n")
        if self.launch is None:
            return
        time.sleep(2)
        table = process_table()
        for pid in residual_pids(table, self.ctx.residual, process_family(table, os.getpid())):
            with contextlib.suppress(ProcessLookupError, PermissionError):
                os.kill(pid, signal.SIGKILL)
        time.sleep(1)

    def keep_failed_session(self) -> None:
        """A failed attempt keeps its session WITH the attempt: left in the logging
        root it is an orphan nothing names."""
        if self.session and os.path.isdir(self.session):
            shutil.move(self.session, self.out / "session_failed")

    # ── the unit ─────────────────────────────────────────────────────────
    def run(self) -> str:
        """Throw the unit; returns the status it wrote. A signal ends it as
        ``FAIL:signal`` and goes on up (:class:`Interrupted`)."""
        self.out.mkdir(parents=True, exist_ok=True)
        (self.out / "status").unlink(missing_ok=True)
        try:
            try:
                status = self._run()
            except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
                status = self._stop(_error_status(exc))
            self._note("status", status + "\n")
            if status != DONE:
                self.keep_failed_session()
            return status
        except Interrupted:
            # Wherever the signal fell — in a step, in a shutdown already under way —
            # the whole shutdown runs again from the top (the handler lets no second
            # signal in).
            self._fail("FAIL:signal")
            raise
        except BaseException as exc:
            # A defect of this command: the sim it started must still not stay up.
            self._fail(_error_status(exc))
            raise

    def _fail(self, status: str) -> None:
        self.cleanup()
        self._note("status", status + "\n")
        self.keep_failed_session()

    def _stop(self, status: str) -> str:
        self.cleanup()
        return status

    def _run(self) -> str:
        spec, ctx, out = self.spec, self.ctx, self.out
        if (out / "trials").exists():
            return "FAIL:trials dir exists (move the old unit aside)"
        wait_for_free_minute(ctx.sim_root / ".last_minute", ctx.logging_root)
        self._write_conditions()

        log_path = out / "launch.log"
        self.launch = Group(launch_command(ctx.robot, spec.arm, ctx.viewer), log_path)
        state = startup.WAITING
        for _ in range(STARTUP_TIMEOUT_S):
            state = startup.startup_state(_read_text(log_path))
            if state != startup.WAITING or self.launch.poll() is not None:
                break
            time.sleep(1)
        log = _read_text(log_path)
        self.session = session_dir_from_log(log, ctx.logging_root)
        self._note("session.txt", (self.session or "") + "\n")
        if state != startup.READY:
            failures = startup.failure_lines(log)
            if failures:
                self._note("startup_failure.txt", failures[0] + "\n")
            return self._stop("FAIL:startup")
        time.sleep(3)

        # From here to the unit's end every line of the launch log is read once for a
        # key in the viewer: the same tail goes on under the driver.
        self.launch_tail = LogTail(log_path)
        keys = startup.viewer_interference(self.launch_tail.new())
        if keys:
            self._note("abnormal_start.txt", _viewer_key(keys) + "\n")
            return self._stop("FAIL:abnormal_start")
        log = _read_text(log_path)
        self._note("commit.txt", commit_line(log) + "\n")
        try:
            mirror, ball_type = read_mirror()
        except RuntimeError as exc:
            self._note("mirror.txt", f"unread: {exc}\n")
            return self._stop("FAIL:mirror unread")
        self._note("mirror.txt", mirror_text(mirror, ball_type))
        self._note("segment_startup.txt", "".join(f"{x}\n" for x in segment_startup_lines(log)))
        self._condition(planner_budget_s=planner_budget(log) or "")
        why = mirror_failures(mirror, ball_type, log, self.expect)
        if why:
            return self._stop("FAIL:mirror " + " ".join(why))

        est_log = out / "est.log"
        self.estimator = Group(
            estimator_command(
                ctx.estimator_ws,
                ctx.profile,
                spec.seed,
                producer_revision=ctx.revisions["ball_perception"][0],
                simulator_revision=ctx.revisions["rtc_framework"][0],
            ),
            est_log,
        )
        for _ in range(ESTIMATOR_TIMEOUT_S):
            if ESTIMATOR_ACTIVE in _read_text(est_log) or self.estimator.poll() is not None:
                break
            time.sleep(1)
        est = _read_text(est_log)
        if WRONG_PREFIX in est:
            return self._stop("FAIL:ball_perception not from BALL_SIM_WS")
        if ESTIMATOR_ACTIVE not in est:
            return self._stop("FAIL:estimator not activated")
        time.sleep(2)

        # The call waits for the service: a manager that never answers must end the unit.
        try:
            with open(out / "switch.log", "wb") as handle:
                subprocess.run(
                    switch_command(),
                    stdin=subprocess.DEVNULL,
                    stdout=handle,
                    stderr=subprocess.STDOUT,
                    check=False,
                    timeout=SWITCH_TIMEOUT_S,
                )
        except subprocess.TimeoutExpired:
            return self._stop("FAIL:switch")
        if not switched(_read_text(out / "switch.log")):
            return self._stop("FAIL:switch")

        what, why_not = self._throw()
        self._condition(
            loadavg_end=loadavg(),
            date_end=datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        )
        rc = self.driver.poll()
        self.cleanup()
        if what == "abnormal":
            self._note("abnormal_start.txt", f"{why_not}\n")
            return "FAIL:abnormal_start"
        if what == "sim_exited":
            return "FAIL:sim exited"
        if what == "timeout":
            return f"FAIL:trials rc={EXIT_TIMEOUT}"
        if rc == EXIT_HOST_BUSY:
            return "FAIL:host_busy"
        if rc != 0:
            return f"FAIL:trials rc={rc}"
        return self._keep_session()

    def _throw(self) -> tuple[str, str | None]:
        """Run the driver over the chunk and watch the unit while it throws
        (:func:`watch_verdict`'s ``(what, why)`` of the look that ended it)."""
        spec, ctx, out = self.spec, self.ctx, self.out
        trials_log = out / "trials.log"
        self.driver = Group(
            driver_command(
                out / "trials",
                ctx.robot,
                spec.chunk,
                spec.arm.name,
                limit=ctx.throws_limit,
                end_on_ball_low=ctx.end_on_ball_low,
                host_watch=ctx.host_watch,
            ),
            trials_log,
        )
        began = time.monotonic()
        keys: list[str] = []
        start: tuple[str, str | None] = (startup.WAITING, None)
        while True:
            # The exit code first, the logs after: what an ended driver wrote is then
            # all there when it is read.
            rc = self.driver.poll()
            keys += startup.viewer_interference(self.launch_tail.new())
            if start[0] == startup.WAITING:
                start = startup.flapping_start(_read_text(trials_log))
            what, why = watch_verdict(
                driver_rc=rc,
                elapsed_s=time.monotonic() - began,
                timeout_s=driver_timeout_s(spec.n_throws),
                viewer_keys=keys,
                start=start,
                sim_alive=self.launch.poll() is None,
            )
            if what == "go":
                time.sleep(WATCH_PERIOD_S)
                continue
            if what != "ended":
                self.driver.stop(DRIVER_STOP_S)
            return what, why

    def _keep_session(self) -> str:
        """Check the session the unit wrote, copy it into the unit and drop the original."""
        out, session = self.out, self.session
        # No session named in the launch log: there is nothing that may be copied or
        # removed (an empty path once made the workspace root the "session").
        if not session or not os.path.isdir(session):
            return "FAIL:no session directory in the launch log"
        size = sum(f.stat().st_size for f in Path(session).rglob("*") if f.is_file())
        self._note("session_size.txt", f"{size / 1e6:.1f} MB\t{session}\n")
        why = session_contamination(Path(session), self.ctx.state_logs)
        if why:
            return f"FAIL:{why}"
        try:
            shutil.copytree(session, out / "session", dirs_exist_ok=True)
        except (OSError, shutil.Error):
            return "FAIL:session copy"
        diag = out / "session" / "controllers" / CATCHING / "catching_diag.csv"
        if not diag.is_file() or diag.stat().st_size == 0:
            return "FAIL:no diag in copy"
        shutil.rmtree(session)
        return DONE


# ── the run ──────────────────────────────────────────────────────────────────


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "--config",
        required=True,
        help="the population's configuration file (config/<robot>/catch_population.yaml)",
    )
    parser.add_argument("--out", required=True, help="the result folder stage 1 wrote")
    parser.add_argument(
        "--estimator-ws",
        default=os.environ.get("BALL_SIM_WS"),
        help="the estimator's colcon workspace (default: $BALL_SIM_WS)",
    )
    parser.add_argument(
        "--profile",
        default=os.environ.get("PROFILE"),
        help="the estimator's profile (default: $PROFILE, then its shipped catching profile)",
    )
    parser.add_argument("--arms", nargs="+", help="throw these arms only (<search>_<segment>)")
    parser.add_argument(
        "--seeds",
        nargs="+",
        type=int,
        help="estimator seeds to throw under (default: the configuration's sim.estimator_seeds)",
    )
    parser.add_argument("--max-chunks", type=int, help="throw only the chunks 0..N-1 of every arm")
    parser.add_argument(
        "--throws-limit", type=int, help="throw only the first N throws of every chunk (a smoke)"
    )
    parser.add_argument(
        "--viewer",
        choices=("on", "off"),
        help="the MuJoCo viewer (default: the configuration's sim.viewer)",
    )
    parser.add_argument(
        "--domain-id",
        type=int,
        default=int(os.environ.get("EVAL_DOMAIN", DEFAULT_DOMAIN_ID)),
        help=f"ROS_DOMAIN_ID of the run (default: $EVAL_DOMAIN, then {DEFAULT_DOMAIN_ID})",
    )
    parser.add_argument(
        "--collect-only",
        action="store_true",
        help="throw nothing: analyse the finished units and rebuild the result files",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the units of this run and their command lines, start nothing",
    )
    args = parser.parse_args(argv)
    for name in ("max_chunks", "throws_limit"):
        value = getattr(args, name)
        if value is not None and value < 1:
            parser.error(f"--{name.replace('_', '-')} must be >= 1")
    return args


class Run:
    """The run's log: every line goes to the terminal and to ``<result>/sim/progress.log``."""

    def __init__(self, sim_root: Path) -> None:
        self.sim_root = sim_root

    def say(self, text: str) -> None:
        line = f"{datetime.datetime.now():%H:%M:%S} {text}"
        print(line, flush=True)
        self.sim_root.mkdir(parents=True, exist_ok=True)
        with open(self.sim_root / "progress.log", "a", encoding="utf-8") as handle:
            handle.write(line + "\n")


def analyse_pending(run: Run, layout, arms: Sequence, share_config_dir: Path) -> None:
    """Analyse every DONE unit of ``arms`` that has no ``ct/vec.csv`` (no sim is up)."""
    from integrated_bringup.catching_tc_vector import analysis_python

    python = None
    for arm in arms:
        for unit in units_present(sim_root(layout), arm.name):
            if not needs_analysis(unit.path):
                continue
            python = python or analysis_python()
            if not analyse_unit(unit.path, share_config_dir, python):
                run.say(f"analysis FAILED {unit.name} (see ct/log.txt)")


def collect(run: Run, config, layout, arms: Sequence) -> bool:
    """Rebuild every arm's result files; True when all of them could be."""
    ok = True
    for arm in arms:
        try:
            result = collect_arm(
                layout,
                robot=config.robot,
                arm=arm.name,
                search=arm.search,
                segment=arm.segment,
                thrown_set=arm.throw_set,
            )
        except CollectError as exc:
            run.say(f"collect {arm.name} REFUSED: {exc}")
            ok = False
            continue
        for line in result.lines():
            run.say(line)
    return ok


def _refuse(message: str):
    print(f"catch_population_trials: {message}", file=sys.stderr)
    raise SystemExit(2)


def _installed(config) -> tuple[Path, Path]:
    """The robot's installed configuration folder and the package's launch folder."""
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

    try:
        share = Path(get_package_share_directory(PACKAGE))
    except PackageNotFoundError:
        _refuse(f"the package {PACKAGE} is not in this environment (source its workspace)")
    return share / "config" / config.robot, share / "launch"


def _estimator(args) -> tuple[Path, Path]:
    """The estimator's workspace and profile of this run, or the reason there is none."""
    if not args.estimator_ws:
        _refuse("name the estimator's colcon workspace: --estimator-ws or BALL_SIM_WS")
    workspace = Path(args.estimator_ws).resolve()
    if not (workspace / "install" / "local_setup.bash").is_file():
        _refuse(f"no colcon workspace at {workspace} (no install/local_setup.bash)")
    profile = Path(args.profile).resolve() if args.profile else estimator_profile(workspace)
    if not profile.is_file():
        _refuse(f"no estimator profile {profile}")
    return workspace, profile


def _context(config, layout, args, *, share_config_dir, viewer, estimator, state_logs):
    """What the units of this run share (:class:`RunContext`), read off the environment
    the command was started in."""
    from ament_index_python.packages import PackageNotFoundError, get_package_prefix

    from rtc_tools.utils.session_dir import resolve_logging_root

    workspace, profile = estimator
    try:
        prefixes = {
            "mujoco_simulator_node": get_package_prefix("rtc_mujoco_sim"),
            "integrated_rt_controller": get_package_prefix(PACKAGE),
        }
    except PackageNotFoundError as exc:
        _refuse(f"a package of the sim is not in this environment: {exc}")
    return RunContext(
        robot=config.robot,
        config_path=config.path,
        config_sha256=config.sha256,
        share_config_dir=share_config_dir,
        sim_root=sim_root(layout),
        viewer=viewer,
        end_on_ball_low=config.sim.end_on_ball_low,
        host_watch=config.sim.host_watch,
        throws_limit=args.throws_limit,
        estimator_ws=workspace,
        profile=profile,
        domain_id=args.domain_id,
        # The launch is started from this directory in this environment: its root is ours.
        logging_root=resolve_logging_root(),
        state_logs=state_logs,
        residual=tuple(residual_patterns(prefixes, str(workspace))),
        revisions={
            "rtc_framework": revision(own_source_tree()),
            "ball_perception": revision(str(workspace.joinpath(*ESTIMATOR_SOURCE))),
        },
    )


def print_plan(units: Sequence[UnitSpec], ctx: RunContext) -> None:
    """``--dry-run``: every unit of the run, where it stands and what would be started."""
    for spec in units:
        print(f"{spec.name}  n={spec.n_throws}  status={unit_status(spec.unit_dir)}")
        commands = (
            launch_command(ctx.robot, spec.arm, ctx.viewer),
            estimator_command(
                ctx.estimator_ws,
                ctx.profile,
                spec.seed,
                producer_revision=ctx.revisions["ball_perception"][0],
                simulator_revision=ctx.revisions["rtc_framework"][0],
            )[3:],
            driver_command(
                spec.unit_dir / "trials",
                ctx.robot,
                spec.chunk,
                spec.arm.name,
                limit=ctx.throws_limit,
                end_on_ball_low=ctx.end_on_ball_low,
                host_watch=ctx.host_watch,
            ),
        )
        for command in commands:
            print("  " + " ".join(command))


def main(argv=None) -> int:
    from rtc_tools.utils.catch_population import ConfigError, ResultLayout, load_config

    args = parse_args(argv)
    try:
        config = load_config(args.config)
        arms = [config.arm(name) for name in args.arms] if args.arms else list(config.arms)
    except ConfigError as exc:
        _refuse(str(exc))
    layout = ResultLayout(Path(args.out).resolve())
    if not layout.root.is_dir():
        _refuse(f"no result folder {layout.root} (stage 1 writes it)")
    differing = stage1_mismatch(config, layout.config)
    if differing:
        _refuse(
            f"{config.path} is not the configuration {layout.config} was judged with "
            f"(differs in: {', '.join(differing)})"
        )
    share_config_dir, launch_dir = _installed(config)
    run = Run(sim_root(layout))

    if args.collect_only:
        analyse_pending(run, layout, arms, share_config_dir)
        return 0 if collect(run, config, layout, arms) else 1

    from rtc_tools.analysis.catching_throw_list import read_throw_list_file
    from rtc_tools.utils.controller_config import load_controller_config

    chunks = {arm.name: layout.chunks(arm.name) for arm in arms}
    missing = [arm.name for arm in arms if not chunks[arm.name]]
    if missing:
        _refuse(f"no chunk files for {missing} under {layout.root} (stage 1 writes <arm>/throws/)")
    if not (launch_dir / launch_file(config.robot)).is_file():
        _refuse(f"{PACKAGE} has no {launch_file(config.robot)} — no sim bringup of {config.robot}")
    try:
        counts = {
            c: len(read_throw_list_file(c).throws) for files in chunks.values() for c in files
        }
        # Before a unit is started: a unit that cannot say what it expects is not one.
        expect = {
            arm.name: shipped_expectation(
                share_config_dir, share_config_dir / config.ball.config.name, arm
            )
            for arm in arms
        }
        controller = load_controller_config(
            share_config_dir / "controllers" / f"{CATCHING}.yaml", config_key=CATCHING
        )[CATCHING]
    except (OSError, ValueError) as exc:
        _refuse(str(exc))
    units = plan_units(
        arms,
        args.seeds or config.sim.estimator_seeds,
        chunks,
        counts,
        layout.unit_dir,
        max_chunks=args.max_chunks,
        throws_limit=args.throws_limit,
    )
    ctx = _context(
        config,
        layout,
        args,
        share_config_dir=share_config_dir,
        viewer=config.sim.viewer if args.viewer is None else args.viewer == "on",
        estimator=_estimator(args),
        state_logs=state_log_names(controller),
    )
    if args.dry_run:
        print_plan(units, ctx)
        return 0

    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)

    def on_signal(signum, _frame):
        # Once: the unit's shutdown must not be cut by a second Ctrl-C.
        for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(number, signal.SIG_IGN)
        raise Interrupted(signum)

    for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(number, on_signal)

    run.say(
        f"{len(units)} units of {config.robot} ({', '.join(a.name for a in arms)}), viewer "
        f"{'on' if ctx.viewer else 'off'}, domain {args.domain_id}, sessions under "
        f"{ctx.logging_root}"
    )
    complete = True
    try:
        for spec in units:
            if layout.stop_file.exists():
                run.say(f"STOP file: leaving before {spec.name}")
                complete = False
                break
            if not throw_unit(run, spec, expect[spec.arm.name], ctx, config.sim.retries):
                complete = False
            # Between two units no sim is up: the one moment the analysis may run.
            analyse_pending(run, layout, [spec.arm], share_config_dir)
    except Interrupted as exc:
        run.say(f"signal {exc.args[0]}: the running unit was ended, nothing is collected")
        return 130
    return 0 if collect(run, config, layout, arms) and complete else 1


def throw_unit(run: Run, spec: UnitSpec, expect: Expectation, ctx: RunContext, retries: int):
    """Throw one unit until it is DONE or may not be thrown again; True when it is DONE
    (:func:`next_step` is the rule, ``retries`` the bringups it may take)."""
    stop = ctx.sim_root.parent / "STOP"
    for attempt in range(1, retries + 1):
        status = unit_status(spec.unit_dir)
        step = next_step(status, attempt)
        if step == "done":
            other = done_for_another_run(spec.unit_dir, spec)
            if other:
                run.say(f"{spec.name} is DONE but {other} — not thrown; move it aside")
            return other is None
        if step == "stay":
            run.say(f"{spec.name} not retried: '{status}' is not a rig failure")
            return False
        if attempt > 1 and stop.exists():
            return False
        if spec.unit_dir.exists():
            spec.unit_dir.rename(failed_name(spec.unit_dir))
        run.say(f"start {spec.name} try {attempt} load {loadavg()}")
        began = time.monotonic()
        status = UnitRun(spec, expect, ctx).run()
        run.say(f"end {spec.name} status={status} wall={time.monotonic() - began:.0f}s")
    if status != DONE:
        run.say(f"{spec.name} given up after {retries} bringups: '{status}'")
    return status == DONE
