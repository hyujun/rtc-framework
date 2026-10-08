"""Multi-frame CLIK controller support for demo_controller_gui.

``demo_dualarm_controller`` steers SEVERAL task frames in one solve, and three
things about it do not fit the single-arm panels:

  - **what exists is per run.** Which tasks, which goal frames and which posture
    groups there are is whatever the controller's YAML configured, and the names
    are part of its interface: the goal topic is ``<task>/task_goal`` and the
    gain parameters are ``tasks.<task>.gain_linear`` / ``posture.<group>.gain``.
    Nothing here lists a name. ``parse_dualarm_config`` reads the same YAML the
    controller configures from, and every table below is built from that;
  - **a task goal has its own topic and its own frame.** The arm panel's task
    half publishes ``goal_type: task`` on the device-group topic, which this
    controller refuses. A goal here is six numbers stated in a frame the
    operator picks, sent ONCE (a re-send restarts the trajectory);
  - **there is no state topic.** What the solve is doing is read from the tail
    of the controller's own per-tick CSV log, which is flushed as it is
    written. That makes the readout local to the machine the controller runs
    on, and it is withheld — never frozen — once the file stops growing.

Pure Python — no Tk, no rclpy — so it is unit-testable without a display or a
ROS graph, following ``demo_gui.pull`` / ``demo_gui.task_frame``.

Public surface (imported by app.py):
- DUALARM_CONFIG_KEY, DualArmSpec, TaskSpec, PostureGroupSpec
- dualarm_config_path, load_dualarm_spec, parse_dualarm_config
- GainRow, gain_rows, gain_group_layout
- goal_topic, frame_choices, goal_frame_id, parse_goal_entries,
  format_goal_entries, frame_note, euler_note
- MeasuredPose, MeasuredPoses, measured_child_frame, copy_allowed
- resolve_session_dir, diag_csv_path, DiagSample, DiagTail
- CmView, StatusField, build_status, status_keys
"""

from __future__ import annotations

import math
import os
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass

from .pull import PLACEHOLDER

# Registry / config key of the controller, and with it the namespace of its
# topics and of its parameter node.
DUALARM_CONFIG_KEY = "demo_dualarm_controller"

# `logs[].msg_type` of the per-tick record. Owner:
# include/integrated_bringup/logging/dualarm_diag_log_pod.hpp (kDualArmDiagLogMsgType).
DIAG_LOG_MSG_TYPE = "integrated_bringup/DualArmDiagLog"

# The controller broadcasts a link's MEASURED pose under `<link>_actual`.
# Owner: src/support/owned_topics.cpp (MakeActualChildFrame).
_ACTUAL_SUFFIX = "_actual"


# ── What the controller was configured with ──────────────────────────────────


@dataclass(frozen=True)
class TaskSpec:
    """One SE3 task of the solve."""

    name: str  # goal topic segment and parameter segment
    frame: str  # the link the task steers
    base_frame: str  # the frame its target and error are taken in
    gain_linear: float  # 1/s
    gain_angular: float  # 1/s


@dataclass(frozen=True)
class PostureGroupSpec:
    name: str
    joints: tuple[str, ...]
    gain: float  # 1/s


@dataclass(frozen=True)
class DualArmSpec:
    tasks: tuple[TaskSpec, ...]
    # Frames a goal may be expressed in besides its task's own base frame.
    target_frames: tuple[str, ...]
    posture_groups: tuple[PostureGroupSpec, ...]
    # trajectory.<key> -> configured value; the `*_max` keys are read-only caps.
    trajectory: Mapping[str, float]
    # Instance (file stem) of the per-tick log, "" when the run does not log it.
    diag_instance: str


# Order the trajectory rows are shown in: each speed next to its own cap.
_TRAJECTORY_KEYS = (
    "linear_speed",
    "linear_speed_max",
    "angular_speed",
    "angular_speed_max",
    "hand_speed",
    "hand_speed_max",
)


def dualarm_config_path(share_dir: str, robot: str) -> str:
    """The controller YAML the ``robot`` bringup configures from."""
    return os.path.join(share_dir, "config", robot, "controllers", f"{DUALARM_CONFIG_KEY}.yaml")


def parse_dualarm_config(doc) -> DualArmSpec:
    """Read the tasks, goal frames, posture groups and speeds out of a parsed
    controller YAML.

    Raises ``ValueError`` naming what is missing. The controller itself refuses
    to configure from such a file, so there is no partial result worth showing:
    a panel built from half a config would offer a goal for a task that does
    not exist.
    """
    try:
        root = doc[DUALARM_CONFIG_KEY]
        clik = root["clik"]
        tasks = tuple(
            TaskSpec(
                name=str(t["name"]),
                frame=str(t["frame"]),
                base_frame=str(t["base_frame"]),
                gain_linear=float(t["gain_linear"]),
                gain_angular=float(t["gain_angular"]),
            )
            for t in clik["tasks"]
        )
        target_frames = tuple(str(f) for f in clik["target_frames"])
        posture_groups = tuple(
            PostureGroupSpec(
                name=str(g["name"]),
                joints=tuple(str(j) for j in g["joints"]),
                gain=float(g["gain"]),
            )
            for g in clik["posture_groups"]
        )
        trajectory = {key: float(root["trajectory"][key]) for key in _TRAJECTORY_KEYS}
        logs = root.get("logs") or ()
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"{DUALARM_CONFIG_KEY} config is not usable: {exc!r}") from exc
    if not tasks:
        raise ValueError(f"{DUALARM_CONFIG_KEY} config declares no task")
    names = [t.name for t in tasks]
    if len(set(names)) != len(names):
        raise ValueError(f"{DUALARM_CONFIG_KEY} config repeats a task name: {names}")
    diag_instance = ""
    for entry in logs:
        if isinstance(entry, Mapping) and entry.get("msg_type") == DIAG_LOG_MSG_TYPE:
            diag_instance = str(entry.get("instance", ""))
            break
    return DualArmSpec(
        tasks=tasks,
        target_frames=target_frames,
        posture_groups=posture_groups,
        trajectory=trajectory,
        diag_instance=diag_instance,
    )


def load_dualarm_spec(path: str) -> DualArmSpec | None:
    """Parse the controller YAML at ``path``; ``None`` when the bringup ships
    none (a robot without this controller). A file that is there but unusable
    raises ``ValueError`` — see ``parse_dualarm_config``."""
    if not os.path.isfile(path):
        return None
    import yaml

    try:
        with open(path) as f:
            doc = yaml.safe_load(f)
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"{path}: {exc}") from exc
    return parse_dualarm_config(doc)


# ── Gains ────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class GainRow:
    """One scalar parameter of the controller, as a row of the Gains panel."""

    label: str  # unique within the controller — the panel's dispatch key
    param: str  # declared ROS parameter name
    default: float
    read_only: bool
    group: str  # title of the box it is shown in


_POSTURE_GROUP_TITLE = "Posture gain [1/s]"
_TRAJECTORY_GROUP_TITLE = "Trajectory speed"


def _task_group_title(task: str) -> str:
    return f"Task {task} gain [1/s]"


def gain_rows(spec: DualArmSpec) -> tuple[GainRow, ...]:
    """Every runtime parameter the controller declares, in panel order.

    Parameter names: src/controllers/dualarm/parameters.cpp (DeclareParameters).
    The defaults are the YAML's, which is what the controller seeds them with;
    Load Gain replaces them with the live values.
    """
    rows: list[GainRow] = []
    for task in spec.tasks:
        group = _task_group_title(task.name)
        rows.append(
            GainRow(
                f"{task.name} linear",
                f"tasks.{task.name}.gain_linear",
                task.gain_linear,
                False,
                group,
            )
        )
        rows.append(
            GainRow(
                f"{task.name} angular",
                f"tasks.{task.name}.gain_angular",
                task.gain_angular,
                False,
                group,
            )
        )
    for posture in spec.posture_groups:
        rows.append(
            GainRow(
                posture.name,
                f"posture.{posture.name}.gain",
                posture.gain,
                False,
                _POSTURE_GROUP_TITLE,
            )
        )
    for key in _TRAJECTORY_KEYS:
        rows.append(
            GainRow(
                key,
                f"trajectory.{key}",
                spec.trajectory[key],
                key.endswith("_max"),
                _TRAJECTORY_GROUP_TITLE,
            )
        )
    return tuple(rows)


def gain_group_layout(spec: DualArmSpec) -> list[list[str]]:
    """Rows of group boxes: the tasks side by side, then posture and speeds."""
    layout = [[_task_group_title(t.name) for t in spec.tasks]]
    layout.append(
        ([_POSTURE_GROUP_TITLE] if spec.posture_groups else []) + [_TRAJECTORY_GROUP_TITLE]
    )
    return layout


# ── Task goals ───────────────────────────────────────────────────────────────

GOAL_AXIS_LABELS = ("X (m)", "Y (m)", "Z (m)", "Roll (deg)", "Pitch (deg)", "Yaw (deg)")


def goal_topic(task: str) -> str:
    return f"/{DUALARM_CONFIG_KEY}/{task}/task_goal"


def frame_choices(spec: DualArmSpec, task: TaskSpec) -> tuple[str, ...]:
    """Frames a goal for ``task`` may be stated in: its own base frame first,
    then the configured goal frames."""
    rest = tuple(f for f in spec.target_frames if f != task.base_frame)
    return (task.base_frame, *rest)


def goal_frame_id(task: TaskSpec, selected: str) -> str:
    """``header.frame_id`` for a goal stated in ``selected``.

    The task's own base frame goes out as the empty string. That is the one
    spelling the controller accepts for it unconditionally — a name is looked up
    in the configured goal frames, and the base frame need not be one of them.
    """
    return "" if selected == task.base_frame else selected


def parse_goal_entries(texts: Sequence[str]) -> list[float]:
    """Six entry strings (m, m, m, deg, deg, deg) -> ``task_target`` (m, rad).

    Raises ``ValueError`` on anything that is not six finite numbers: the
    controller counts a non-finite goal as a reject, and the operator should
    hear about a typo before it becomes a counter.
    """
    if len(texts) != 6:
        raise ValueError(f"a task goal has 6 values, got {len(texts)}")
    try:
        values = [float(t) for t in texts]
    except ValueError:
        raise ValueError("a task goal needs six numbers: an entry is empty or not one") from None
    if not all(math.isfinite(v) for v in values):
        raise ValueError("a task goal value is not finite")
    return [*values[:3], *(math.radians(v) for v in values[3:])]


def format_goal_entries(xyz: Sequence[float], rpy: Sequence[float]) -> list[str]:
    """A pose (m, rad) as the six entry strings (m, deg)."""
    return [f"{v:.4f}" for v in xyz] + [f"{math.degrees(v):.4f}" for v in rpy]


def frame_note(task: TaskSpec, selected: str) -> str:
    """What stating a goal in another frame than the task's own means.

    The controller converts such a goal into the base frame ONCE, on the tick it
    applies it, and holds that. Seen from the frame the operator typed it in,
    the hand then follows the base frame wherever a later goal takes it — which
    reads as the goal not being held unless it is said.
    """
    if selected == task.base_frame:
        return ""
    return (
        f"Stated in '{selected}', held in '{task.base_frame}': converted once, when the goal "
        f"is applied. If '{task.base_frame}' moves afterwards, the pose seen from "
        f"'{selected}' moves with it."
    )


# Within this of ±90° pitch the ZYX roll and yaw turn about nearly the same
# axis. The goal format is the controller's; the note only tells the operator
# why two step buttons seem to do one thing.
_EULER_MARGIN_RAD = math.radians(5.0)


def euler_note(pitch_rad: float) -> str:
    """A one-line warning when ``pitch_rad`` is next to the ZYX singularity."""
    if abs(abs(pitch_rad) - math.pi / 2.0) > _EULER_MARGIN_RAD:
        return ""
    return (
        f"pitch {math.degrees(pitch_rad):+.1f}° is next to ±90°: roll and yaw turn about "
        "nearly the same axis here"
    )


# ── Measured poses (from the controller's transforms) ────────────────────────


def measured_child_frame(task: TaskSpec) -> str:
    """TF child frame carrying the measured pose of the task's frame."""
    return task.frame + _ACTUAL_SUFFIX


@dataclass(frozen=True)
class MeasuredPose:
    parent: str  # the frame the pose is expressed in
    xyz: tuple[float, float, float]  # m
    rpy: tuple[float, float, float]  # rad, ZYX
    stamp_s: float  # monotonic arrival time


class MeasuredPoses:
    """Latest measured pose per TF child frame.

    Written from the rclpy executor thread (``observe``; ``clear`` when the
    active controller changes) and read from the Tk thread (``get``). Each
    write stores or rebinds one immutable value, so a reader never sees half a
    pose.
    """

    # A pose older than this is not "current" — the controller broadcasts every
    # tick it runs, so silence this long means it stopped.
    MAX_AGE_S = 1.0

    def __init__(self) -> None:
        self._poses: dict[str, MeasuredPose] = {}

    def clear(self) -> None:
        """Forget everything — the poses belong to the controller that sent them."""
        self._poses = {}

    def observe(self, child: str, pose: MeasuredPose) -> None:
        self._poses[child] = pose

    def get(self, child: str, now_s: float) -> MeasuredPose | None:
        pose = self._poses.get(child)
        if pose is None or now_s - pose.stamp_s > self.MAX_AGE_S:
            return None
        return pose


def copy_allowed(selected_frame: str, pose: MeasuredPose | None) -> bool:
    """May the measured pose be copied into a goal stated in ``selected_frame``?

    Only when it is expressed in that very frame. The GUI has the pose in the
    transforms' parent frame and nowhere else; turning it into another frame
    needs a transform no topic carries, and a guessed one would send the hand
    somewhere while reading as "stay where you are".
    """
    return pose is not None and pose.parent == selected_frame


# ── The per-tick log as a status source ──────────────────────────────────────


def diag_csv_path(session_dir: str, instance: str) -> str:
    return os.path.join(session_dir, "controllers", DUALARM_CONFIG_KEY, f"{instance}.csv")


def resolve_session_dir(
    explicit: str | None, env_session: str | None, logging_root: str, instance: str = ""
) -> tuple[str | None, str]:
    """The session directory the running controller logs into, and how it was
    found: ``--session``, then ``$RTC_SESSION_DIR``, then a search of the
    logging root.

    The search takes the newest session that HAS this controller's log
    (``instance``), not simply the newest directory: any tool that starts a
    session creates a newer one, and following it would blank the readout of a
    controller that is still running. With no such session it falls back to the
    newest directory, so the readout can say which file it is waiting for. It
    is re-made on every poll — a GUI started before the bringup finds the new
    session as soon as its log exists.
    """
    if explicit:
        return explicit, "--session"
    if env_session:
        return env_session, "$RTC_SESSION_DIR"
    from rtc_tools.utils.session_dir import list_session_dirs

    names = list_session_dirs(logging_root)
    if not names:
        return None, f"no session under {logging_root}"
    if instance:
        for name in reversed(names):
            session = os.path.join(logging_root, name)
            if os.path.isfile(diag_csv_path(session, instance)):
                return session, "newest session with this log"
    return os.path.join(logging_root, names[-1]), "newest session"


@dataclass(frozen=True)
class DiagSample:
    values: Mapping[str, str]  # column name -> the cell, unparsed
    mtime_s: float  # wall clock of the file's last write


class DiagTail:
    """Reads the last complete row of a CSV that is still being written.

    Only the tail is read — the file grows by one row per control tick and is
    polled a few times a second. The header is kept per file (path + inode), so
    a new session is picked up without a restart.
    """

    def __init__(self, tail_bytes: int = 16384) -> None:
        self._tail_bytes = int(tail_bytes)
        self._ident: tuple[str, int] | None = None
        self._header: list[str] = []
        self._header_end = 0

    def read(self, path: str) -> DiagSample | None:
        """The newest complete row, or ``None`` when there is none to read
        (no file, header only, or a file that cannot be opened)."""
        try:
            st = os.stat(path)
            with open(path, "rb") as f:
                ident = (path, st.st_ino)
                if ident != self._ident or st.st_size < self._header_end:
                    self._ident = None
                    first = f.readline()
                    if not first.endswith(b"\n"):
                        return None  # the header itself is still being written
                    self._header = first.decode(errors="replace").strip().split(",")
                    self._header_end = f.tell()
                    self._ident = ident
                start = max(self._header_end, st.st_size - self._tail_bytes)
                f.seek(start)
                chunk = f.read()
        except OSError:
            return None
        lines = chunk.split(b"\n")
        # The last piece is either empty (the chunk ended on a newline) or a row
        # still being written; the first is the back half of a row whenever the
        # read started mid-file. Neither is a row.
        candidates = lines[1:-1] if start > self._header_end else lines[:-1]
        for raw in reversed(candidates):
            cells = raw.decode(errors="replace").strip().split(",")
            if len(cells) == len(self._header):
                return DiagSample(dict(zip(self._header, cells, strict=True)), st.st_mtime)
        return None


# DualArmDiagLogPod::Hold / ::FaultCause wire values, in enum order. Pinned to
# the C++ enums by test_demo_gui_dualarm.py, which reads the POD header.
HOLD_NAMES = (
    "solved",
    "E-STOP",
    "fault latch",
    "body unreadable",
    "not seeded",
    "no model",
)
FAULT_CAUSE_NAMES = ("none", "QP fail streak", "tracking error", "seed outside box")

# Per-tick flags that mark the solve call as abnormal when set.
_FAILURE_FLAGS = (
    "rejected_input",
    "non_finite",
    "command_mismatch",
    "accel_rows_violated",
    "brake_box_empty",
)

# The log is written every tick the controller runs and flushed as it goes; a
# file this long without a write is a controller that is not running.
STALE_AFTER_S = 1.0

# Status levels, mapped to colours by the Tk side.
LEVEL_OK = "ok"
LEVEL_WARN = "warn"
LEVEL_BAD = "bad"
LEVEL_IDLE = "idle"


@dataclass(frozen=True)
class StatusField:
    text: str
    level: str = LEVEL_IDLE


@dataclass(frozen=True)
class CmView:
    """What /rtc_cm/list_controllers says about the controller."""

    state: str
    is_active: bool
    has_latched_fault: bool
    target_reject_count: int
    target_drop_count: int


# Fixed status rows; each task adds the three from `task_status_keys`.
STATUS_KEYS = ("cm", "log", "tick", "solver", "limits", "fault")
STATUS_LABELS = {
    "cm": "Controller",
    "log": "Log",
    "tick": "Tick",
    "solver": "Solve",
    "limits": "Limits",
    "fault": "Fault",
}
TASK_STATUS_COLUMNS = ("error", "goal", "counters")


def task_status_key(task: str, column: str) -> str:
    return f"task/{task}/{column}"


def status_keys(task_names: Iterable[str]) -> tuple[str, ...]:
    """Every key ``build_status`` returns for these tasks, in display order."""
    return STATUS_KEYS + tuple(
        task_status_key(t, c) for t in task_names for c in TASK_STATUS_COLUMNS
    )


def _num(values: Mapping[str, str], key: str) -> float | None:
    """A cell as a finite number, ``None`` when it is absent or does not parse."""
    try:
        value = float(values[key])
    except (KeyError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _int(values: Mapping[str, str], key: str) -> int:
    value = _num(values, key)
    return int(value) if value is not None else 0


def _name(table: Sequence[str], code: int) -> str:
    return table[code] if 0 <= code < len(table) else f"code {code}"


def _counter_sum(values: Mapping[str, str], prefix: str) -> tuple[int, list[str]]:
    """Sum of the columns starting with ``prefix`` and the names of the nonzero
    ones. By prefix, not by a list: the set of causes has changed between
    versions of the log and may again."""
    total = 0
    nonzero: list[str] = []
    for key in values:
        if key.startswith(prefix):
            count = _int(values, key)
            total += count
            if count:
                nonzero.append(f"{key[len(prefix) :]} {count}")
    return total, nonzero


# /rtc_cm/list_controllers is polled every 5 s. Three missed polls is a
# controller manager that is not answering, and its last answer is not its state.
CM_STALE_AFTER_S = 15.0


def _cm_field(cm: CmView | None, cm_age_s: float | None) -> StatusField:
    if cm_age_s is not None and cm_age_s > CM_STALE_AFTER_S:
        return StatusField(
            f"/rtc_cm/list_controllers has not answered for {cm_age_s:.0f} s — withheld",
            LEVEL_IDLE,
        )
    if cm is None:
        return StatusField("/rtc_cm/list_controllers has not reported it", LEVEL_IDLE)
    parts = [cm.state]
    if cm.has_latched_fault:
        parts.append("FAULT LATCHED")
    parts.append(f"mailbox reject {cm.target_reject_count} / drop {cm.target_drop_count}")
    if cm.has_latched_fault:
        level = LEVEL_BAD
    elif cm.is_active:
        level = LEVEL_OK
    else:
        level = LEVEL_IDLE
    return StatusField("  ·  ".join(parts), level)


def _tick_field(values: Mapping[str, str]) -> StatusField:
    hold = _int(values, "hold")
    tick = _int(values, "tick")
    if hold == 0:
        text = f"solved  ·  tick {tick}"
        if _int(values, "reseeded"):
            text += "  ·  reseeded"
        return StatusField(text, LEVEL_OK)
    # E-STOP and the fault latch need the operator; the others pass by themselves.
    level = LEVEL_BAD if hold in (1, 2) else LEVEL_WARN
    return StatusField(f"HOLD — {_name(HOLD_NAMES, hold)}  ·  tick {tick}", level)


def _solver_field(values: Mapping[str, str]) -> StatusField:
    if not _int(values, "clik_ran"):
        return StatusField(PLACEHOLDER)
    flags = [f for f in _FAILURE_FLAGS if _int(values, f)]
    if not _int(values, "reached_solve"):
        # The call ran and was turned away before the QP: there is no solve
        # time, iteration count or status to show.
        text = "input refused before the solve"
        if flags:
            text += f"  ·  {', '.join(flags)}"
        return StatusField(text, LEVEL_BAD)
    converged = bool(_int(values, "converged"))
    text = (
        f"{'converged' if converged else 'NOT converged'}  ·  status {_int(values, 'status')}"
        f"  ·  {_int(values, 'iterations')} iter  ·  {_int(values, 'solve_time_us')} µs"
    )
    if flags:
        return StatusField(f"{text}  ·  {', '.join(flags)}", LEVEL_BAD)
    return StatusField(text, LEVEL_OK if converged else LEVEL_WARN)


def _limits_field(values: Mapping[str, str]) -> StatusField:
    if not _int(values, "reached_solve"):
        return StatusField(PLACEHOLDER)
    binding = _int(values, "accel_rows_binding")
    text = f"accel rows {_int(values, 'accel_rows')} ({binding} binding)"
    active = [
        name
        for name, key in (
            ("feedback capped", "fb_saturated"),
            ("rotation near π", "rot_near_pi"),
            ("braking", "brake_active"),
            ("brake infeasible", "brake_static_infeasible"),
        )
        if _int(values, key)
    ]
    if active:
        return StatusField(f"{text}  ·  {', '.join(active)}", LEVEL_WARN)
    return StatusField(text, LEVEL_WARN if binding else LEVEL_OK)


def _fault_field(values: Mapping[str, str]) -> StatusField:
    streak = _int(values, "qp_fail_streak")
    if _int(values, "fault_latched"):
        cause = _name(FAULT_CAUSE_NAMES, _int(values, "fault_cause"))
        return StatusField(f"LATCHED — {cause}  ·  fail streak {streak}", LEVEL_BAD)
    text = f"none  ·  fail streak {streak}"
    track_err = _num(values, "track_err")
    if _int(values, "clik_ran") and track_err is not None:
        text += f"  ·  track err {track_err:.4f} rad"
    return StatusField(text, LEVEL_WARN if streak else LEVEL_OK)


def _task_fields(values: Mapping[str, str], task: str) -> dict[str, StatusField]:
    fields: dict[str, StatusField] = {}
    err_lin = _num(values, f"{task}_err_lin")
    err_ang = _num(values, f"{task}_err_ang")
    if _int(values, f"{task}_valid") and err_lin is not None and err_ang is not None:
        fields["error"] = StatusField(
            f"{err_lin * 1000.0:.2f} mm  ·  {math.degrees(err_ang):.3f}°", LEVEL_OK
        )
    else:
        # Not computed this tick. The cell holds a zero, which is not an error of zero.
        fields["error"] = StatusField(PLACEHOLDER)
    moving = bool(_int(values, f"{task}_traj_active"))
    fields["goal"] = StatusField(
        f"#{_int(values, f'{task}_goal_sequence')}  ·  {'moving' if moving else 'settled'}",
        LEVEL_WARN if moving else LEVEL_OK,
    )
    rejects, reject_names = _counter_sum(values, f"{task}_reject_")
    drops, drop_names = _counter_sum(values, f"{task}_drop_")
    text = (
        f"accepted {_int(values, f'{task}_goals_accepted')}  ·  "
        f"rejected {rejects}  ·  dropped {drops}"
    )
    detail = reject_names + drop_names
    if detail:
        text += f"  ({', '.join(detail)})"
    fields["counters"] = StatusField(text, LEVEL_WARN if detail else LEVEL_OK)
    return fields


def build_status(
    task_names: Sequence[str],
    sample: DiagSample | None,
    now_wall_s: float,
    cm: CmView | None,
    source: str,
    cm_age_s: float | None = None,
) -> dict[str, StatusField]:
    """Every status row, keyed as ``status_keys(task_names)``.

    ``source`` is the log's path when ``sample`` is given and the reason there
    is none otherwise. A sample whose file has stopped growing is treated as no
    sample for everything but the ``log`` row: the last row of a controller
    that is no longer running is not its state. ``cm_age_s`` is how long ago
    ``cm`` was reported; an old one is withheld the same way.
    """
    fields: dict[str, StatusField] = {
        key: StatusField(PLACEHOLDER) for key in status_keys(task_names)
    }
    fields["cm"] = _cm_field(cm, cm_age_s)
    if sample is None:
        fields["log"] = StatusField(source, LEVEL_IDLE)
        return fields
    age_s = max(0.0, now_wall_s - sample.mtime_s)
    if age_s > STALE_AFTER_S:
        fields["log"] = StatusField(
            f"{source}  ·  last row {age_s:.0f} s ago — not running, values withheld",
            LEVEL_IDLE,
        )
        return fields
    fields["log"] = StatusField(source, LEVEL_OK)
    values = sample.values
    fields["tick"] = _tick_field(values)
    fields["solver"] = _solver_field(values)
    fields["limits"] = _limits_field(values)
    fault = _fault_field(values)
    group_rejects = _int(values, "group_goal_rejects")
    if group_rejects:
        fault = StatusField(
            f"{fault.text}  ·  task goals on a group topic {group_rejects}",
            fault.level if fault.level != LEVEL_OK else LEVEL_WARN,
        )
    fields["fault"] = fault
    for task in task_names:
        for column, field in _task_fields(values, task).items():
            fields[task_status_key(task, column)] = field
    return fields
