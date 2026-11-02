"""Per-hand pose list and ordered replay of the hand joint-command GUI.

No rclpy, no Qt. A ``PoseSequencer`` sits beside one ``HandController``: it
keeps that hand's poses (targets in degrees, keyed by joint name), edits the
list, reads and writes the pose file and, during replay, is the only writer
of the controller's targets. Publishing, slew and every safety stop stay in
``HandController.tick``; ``step`` runs right after ``tick`` in the same
publish period and only decides whether to move on to the next pose.

- Save stores what the sliders show: the target while the hand is enabled,
  the measured position while it is disabled.
- Replay (``play``) needs the hand enabled, a non-empty list and every pose
  inside the current slider ranges — checked up front, nothing is clamped.
  It walks the list in order: set the pose as target, wait until the command
  has reached it (exactly — ``tick`` snaps), hold ``dwell_s``, next. The
  last pose is held when the list ends. ``go_to`` is one pose without dwell.
- ``stop`` freezes the hand where it is (target <- command). A hand that is
  disabled during replay (by the user or automatically) ends the replay in
  the same step; ``message`` then carries the controller's reason.
- While ``playing`` the view locks the sliders, the ranges and the list.
- Pose file (YAML, one file per hand)::

      hand: <label>
      poses:
        - name: <text>
          dwell_s: <0 .. MAX_DWELL_S>
          joints_deg: {<joint name>: <deg>, ...}   # every joint of the hand

  Loading validates the whole file against the hand's joint names and
  rejects it as a whole on the first problem: unknown keys, ``hand`` other
  than the label, ``joints_deg`` that does not name exactly the hand's
  joints, a value beyond ``ABS_LIMIT_DEG``. Ranges are checked at play, not
  at load. Duplicate joint keys are not detected (``yaml.safe_load`` keeps
  the last one).

GUI YAML keys (parsed by ``joint_command_model``): ``replay_dwell_s`` — the
dwell written into a pose when it is saved (default 1.0, 0 .. MAX_DWELL_S);
``poses_files`` — ``{<label>: <path>}``, loaded once that hand's joint names
are known, relative paths resolved against the config file's directory.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

from rtc_tools.gui.joint_command_model import (
    ABS_LIMIT_DEG,
    MAX_DWELL_S,
    HandController,
    _number as _config_number,
    dwell_error,
)

_FILE_KEYS = {"hand", "poses"}
_POSE_KEYS = {"name", "dwell_s", "joints_deg"}


class PoseError(ValueError):
    """A pose file does not fit this hand."""


@dataclass
class Pose:
    name: str
    dwell_s: float
    # joint name -> deg, in the hand's joint order
    joints_deg: dict[str, float]


def _number(value, what: str) -> float:
    return _config_number(value, what, PoseError)


def parse_pose_file(data, label: str, joint_names: tuple[str, ...]) -> list[Pose]:
    """Validate a parsed pose-file mapping for the hand ``label``; raises PoseError."""
    if not isinstance(data, dict):
        raise PoseError("the pose file must be a mapping with 'hand' and 'poses'")
    unknown = set(data) - _FILE_KEYS
    if unknown:
        raise PoseError(f"unknown keys: {sorted(unknown)}")
    if data.get("hand") != label:
        raise PoseError(f"hand is {data.get('hand')!r}, this panel is {label!r}")
    entries = data.get("poses")
    if entries is None:
        entries = []
    if not isinstance(entries, list):
        raise PoseError("poses must be a list")
    poses = []
    for n, entry in enumerate(entries, start=1):
        what = f"poses[{n}]"
        if not isinstance(entry, dict):
            raise PoseError(f"{what} must be a mapping")
        unknown = set(entry) - _POSE_KEYS
        if unknown:
            raise PoseError(f"{what}: unknown keys {sorted(unknown)}")
        name = entry.get("name")
        if not isinstance(name, str) or not name.strip():
            raise PoseError(f"{what}: name must be a non-empty string, got {name!r}")
        dwell = _number(entry.get("dwell_s"), f"{what}.dwell_s")
        reason = dwell_error(dwell)
        if reason:
            raise PoseError(f"{what}: {reason}")
        joints = entry.get("joints_deg")
        if not isinstance(joints, dict):
            raise PoseError(f"{what}.joints_deg must map joint name to degrees")
        missing = [j for j in joint_names if j not in joints]
        extra = sorted(set(joints) - set(joint_names))
        if missing or extra:
            raise PoseError(
                f"{what} '{name}': joints_deg does not match the hand — "
                f"missing {missing}, unknown {extra}"
            )
        values = {}
        for joint in joint_names:
            deg = _number(joints[joint], f"{what}.joints_deg.{joint}")
            if abs(deg) > ABS_LIMIT_DEG:
                raise PoseError(
                    f"{what}.joints_deg.{joint} = {deg:g} is beyond ±{ABS_LIMIT_DEG:g} deg"
                )
            values[joint] = deg
        poses.append(Pose(name.strip(), dwell, values))
    return poses


def pose_file_data(label: str, poses: list[Pose]) -> dict:
    """The mapping written as the pose file of hand ``label``."""
    return {
        "hand": label,
        "poses": [
            {"name": p.name, "dwell_s": p.dwell_s, "joints_deg": dict(p.joints_deg)} for p in poses
        ],
    }


class PoseSequencer:
    """Pose list, file I/O and the replay of one hand (see the module docstring).

    Every action returns the refusal as text (None = done) and leaves the same
    text — or a short outcome — in ``message`` for the panel to show.
    """

    def __init__(self, ctrl: HandController, default_dwell_s: float) -> None:
        self.ctrl = ctrl
        self.default_dwell_s = default_dwell_s
        self.poses: list[Pose] = []
        self.file_path: Path | None = None
        self.dirty = False
        # bumped on every change of the list; the view rebuilds its table on it
        self.revision = 0
        self.playing = False
        # index of the pose being driven while playing
        self.current: int | None = None
        self.message = ""
        self._single = False
        self._due_s: float | None = None

    # ── list editing ─────────────────────────────────────────────────────

    def _refuse(self, reason: str) -> str:
        self.message = reason
        return reason

    def _done(self, note: str) -> None:
        self.message = note

    def _changed(self) -> None:
        self.dirty = True
        self.revision += 1

    def _current_deg(self) -> list[float] | str:
        """What the sliders show now (deg), or why there is nothing to save."""
        ctrl = self.ctrl
        if not ctrl.ready:
            return "no joint_states received yet"
        if ctrl.enabled:
            return list(ctrl.target_deg)
        measured = ctrl.measured_deg()
        if measured is None:
            return "no joint_states received yet"
        if not all(math.isfinite(d) for d in measured):
            return "measured position is not finite"
        return measured

    def _index_error(self, index: int | None) -> str | None:
        if index is None or not 0 <= index < len(self.poses):
            return "no pose selected"
        return None

    def save_pose(self) -> str | None:
        """Append the current slider values as a new pose."""
        if self.playing:
            return self._refuse("replay in progress")
        values = self._current_deg()
        if isinstance(values, str):
            return self._refuse(values)
        pose = Pose(
            f"pose {len(self.poses) + 1}",
            self.default_dwell_s,
            dict(zip(self.ctrl.joint_names, values, strict=True)),
        )
        self.poses.append(pose)
        self._changed()
        self._done(f"saved '{pose.name}'")
        return None

    def update_pose(self, index: int | None) -> str | None:
        """Overwrite the joints of pose ``index`` with the current slider values."""
        if self.playing:
            return self._refuse("replay in progress")
        reason = self._index_error(index)
        if reason:
            return self._refuse(reason)
        values = self._current_deg()
        if isinstance(values, str):
            return self._refuse(values)
        pose = self.poses[index]
        pose.joints_deg = dict(zip(self.ctrl.joint_names, values, strict=True))
        self._changed()
        self._done(f"updated '{pose.name}'")
        return None

    def delete_pose(self, index: int | None) -> str | None:
        if self.playing:
            return self._refuse("replay in progress")
        reason = self._index_error(index)
        if reason:
            return self._refuse(reason)
        pose = self.poses.pop(index)
        self._changed()
        self._done(f"deleted '{pose.name}'")
        return None

    def move_pose(self, index: int | None, delta: int) -> str | None:
        """Move pose ``index`` by ``delta`` places (-1 = up, +1 = down)."""
        if self.playing:
            return self._refuse("replay in progress")
        reason = self._index_error(index)
        if reason:
            return self._refuse(reason)
        new = index + delta
        if not 0 <= new < len(self.poses):
            return self._refuse("already at the end of the list")
        self.poses.insert(new, self.poses.pop(index))
        self._changed()
        self._done(f"moved '{self.poses[new].name}' to #{new + 1}")
        return None

    def rename_pose(self, index: int | None, name: str) -> str | None:
        if self.playing:
            return self._refuse("replay in progress")
        reason = self._index_error(index)
        if reason:
            return self._refuse(reason)
        name = name.strip()
        if not name:
            return self._refuse("a pose name must not be empty")
        self.poses[index].name = name
        self._changed()
        self._done(f"renamed to '{name}'")
        return None

    def set_dwell(self, index: int | None, dwell_s: float) -> str | None:
        if self.playing:
            return self._refuse("replay in progress")
        reason = self._index_error(index) or dwell_error(dwell_s)
        if reason:
            return self._refuse(reason)
        self.poses[index].dwell_s = float(dwell_s)
        self._changed()
        self._done(f"'{self.poses[index].name}' dwell {dwell_s:g} s")
        return None

    # ── replay ───────────────────────────────────────────────────────────

    def _range_error(self, indices: range) -> str | None:
        """The first pose value outside the current slider ranges, or None."""
        for index in indices:
            pose = self.poses[index]
            for name, (lo, hi) in zip(self.ctrl.joint_names, self.ctrl.ranges, strict=True):
                deg = pose.joints_deg[name]
                if not lo <= deg <= hi:
                    return (
                        f"pose '{pose.name}': {name} {deg:.1f} deg is outside "
                        f"the range [{lo:g}, {hi:g}]"
                    )
        return None

    def _start(self, index: int, single: bool) -> str | None:
        ctrl = self.ctrl
        if self.playing:
            return self._refuse("replay in progress")
        if not self.poses:
            return self._refuse("no poses")
        reason = self._index_error(index)
        if reason:
            return self._refuse(reason)
        if not ctrl.enabled:
            return self._refuse("hand is not enabled")
        reason = self._range_error(range(index, index + 1) if single else range(len(self.poses)))
        if reason:
            return self._refuse(reason)
        self.playing = True
        self._single = single
        self._drive(index)
        if self.playing:
            self._done(f"{'going to' if single else 'playing from'} '{self.poses[index].name}'")
        return None if self.playing else self.message

    def play(self) -> str | None:
        """Replay the whole list from the first pose; returns the refusal, or None."""
        return self._start(0, single=False)

    def go_to(self, index: int | None) -> str | None:
        """Move to one pose and end there (no dwell); returns the refusal, or None."""
        if index is None:
            return self._refuse("no pose selected")
        return self._start(index, single=True)

    def _drive(self, index: int) -> None:
        """Make pose ``index`` the controller's target."""
        self.current = index
        self._due_s = None
        pose = self.poses[index]
        for i, name in enumerate(self.ctrl.joint_names):
            deg = pose.joints_deg[name]
            # cannot differ after _range_error, kept so a clamp is never silent
            if self.ctrl.set_target(i, deg) != deg:
                self._finish(f"replay aborted: '{pose.name}' {name} {deg:g} deg was clamped")
                return

    def _finish(self, note: str) -> None:
        self.playing = False
        self.current = None
        self._due_s = None
        self._done(note)

    def stop(self) -> None:
        """End the replay now; the hand stops where its command is."""
        if not self.playing:
            return
        ctrl = self.ctrl
        name = self.poses[self.current].name
        if ctrl.enabled:
            for i, deg in enumerate(ctrl.command_deg):
                ctrl.set_target(i, deg)
        self._finish(f"stopped on the way to '{name}'")

    def step(self, now_s: float) -> None:
        """Advance the replay; call right after ``HandController.tick``."""
        if not self.playing:
            return
        ctrl = self.ctrl
        if not ctrl.enabled:
            self._finish(f"replay stopped: {ctrl.status}")
            return
        if ctrl.command_deg != ctrl.target_deg:
            return
        pose = self.poses[self.current]
        if self._single:
            self._finish(f"reached '{pose.name}'")
            return
        if self._due_s is None:
            self._due_s = now_s + pose.dwell_s
        if now_s < self._due_s:
            return
        following = self.current + 1
        if following >= len(self.poses):
            self._finish(f"replay done — holding '{pose.name}'")
        else:
            self._drive(following)

    # ── files ────────────────────────────────────────────────────────────

    def load_file(self, path: str | Path) -> str | None:
        """Replace the list with the poses of ``path``; returns the refusal, or None."""
        path = Path(path)
        if self.playing:
            return self._refuse("replay in progress")
        if not self.ctrl.ready:
            return self._refuse("no joint_states received yet — joint names unknown")
        try:
            with open(path, encoding="utf-8") as stream:
                data = yaml.safe_load(stream)
            poses = parse_pose_file(data, self.ctrl.label, self.ctrl.joint_names)
        except (OSError, UnicodeError) as exc:
            return self._refuse(f"cannot read {path}: {getattr(exc, 'strerror', None) or exc}")
        except yaml.YAMLError as exc:
            return self._refuse(f"{path.name} is not valid YAML: {exc}")
        except PoseError as exc:
            return self._refuse(f"{path.name}: {exc}")
        self.poses = poses
        self.file_path = path
        self.dirty = False
        self.revision += 1
        self._done(f"loaded {len(poses)} poses from {path.name}")
        return None

    def save_file(self, path: str | Path) -> str | None:
        """Write the list to ``path`` (YAML); returns the refusal, or None."""
        path = Path(path)
        if self.playing:
            return self._refuse("replay in progress")
        if not self.ctrl.ready:
            return self._refuse("no joint_states received yet — joint names unknown")
        text = yaml.safe_dump(pose_file_data(self.ctrl.label, self.poses), sort_keys=False)
        try:
            path.write_text(text, encoding="utf-8")
        except (OSError, UnicodeError) as exc:
            return self._refuse(f"cannot write {path}: {getattr(exc, 'strerror', None) or exc}")
        self.file_path = path
        self.dirty = False
        self._done(f"saved {len(self.poses)} poses to {path.name}")
        return None


__all__ = [
    "MAX_DWELL_S",
    "Pose",
    "PoseError",
    "PoseSequencer",
    "parse_pose_file",
    "pose_file_data",
]
