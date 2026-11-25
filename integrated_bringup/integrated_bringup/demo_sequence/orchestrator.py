"""The run, start to exit code — rclpy-free; the ROS side is an ``Io``.

``Orchestrator.step()`` is called by a loop that never blocks (``cli``); each
call looks at the stack once and moves the state machine:

    WAIT_ROS → CONFIRM → RUNNING → done
       ↑__________|        (graceful stop at the end of a cycle, or abort)

Nothing is published before RUNNING. In RUNNING a step goes out as: speed set
(async) → its answer → this tick's stack checks → publish. A speed set that is
refused or unanswered ends the run without the goal (the GUI sends the goal
anyway; an unattended run does not).
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, field
from typing import Protocol

from ..demo_gui.preset_speed import HandSpeedPlan, plan_hand_speed
from ..demo_gui.sequence import (
    DWELLING,
    LEFT_SWITCHED,
    MODE_LOOP,
    MOVING,
    ResolvedStep,
    SequenceRunner,
)
from .files import RunInputs
from .gates import DwellWatch, disk_verdict, worst_error
from .readiness import READY, REFUSE, Observation, assess, driver_present, runtime_abort

EXIT_DONE = 0
EXIT_ABORT = 1
EXIT_USAGE = 2
EXIT_NOT_READY = 3
EXIT_SUMMARY = 4
EXIT_INTERRUPTED = 130

WAIT_ROS = "wait_ros"
CONFIRM = "confirm"
RUNNING = "running"
DONE = "done"

SPEED_TIMEOUT_S = 1.0  # the GUI's PRESET_SPEED_TIMEOUT_MS
WAIT_REPORT_S = 5.0
DISK_SAMPLE_S = 5.0  # the log growth is measured over at least this, before START
START_WORD = "start"


class Io(Protocol):
    def now(self) -> float: ...
    def wall(self) -> str: ...
    def observe(self) -> Observation: ...
    def set_speed(self, speed: float) -> object | None: ...  # None: no parameter service
    def speed_result(self, handle: object) -> bool | None: ...
    def drop_speed(self, handle: object) -> None: ...
    def publish(self, target_rad: Sequence[float]) -> None: ...
    def interrupts(self) -> int: ...
    def terminated(self) -> bool: ...
    def read_line(self) -> str | None: ...
    def say(self, text: str) -> None: ...
    def environment(self) -> str: ...
    def log_bytes(self) -> int: ...
    def free_bytes(self) -> int: ...
    def write_summary(self, data: dict) -> None: ...


@dataclass(frozen=True)
class RunConfig:
    robot: str
    joint_names: tuple[str, ...]  # the preset / goal order
    inputs: RunInputs
    duration_s: float
    startup_timeout_s: float = 60.0
    hold_abort_after_s: float = 1.0
    assume_yes: bool = False
    heartbeat_s: float = 60.0


@dataclass
class StepStats:
    preset: str
    reached: int = 0
    reach_s_sum: float = 0.0
    reach_s_max: float = 0.0
    error_deg_max: float = 0.0
    # Left by switch (next_at_pct < 100): no arrival was checked, so the pose
    # at the switch and how late the next goal went out are what is recorded.
    switched: int = 0
    switch_error_deg_max: float = 0.0
    lag_s_sum: float = 0.0
    lag_s_max: float = 0.0
    lagged: int = 0  # switches whose next goal was published (lag known)

    def add_switch(self, error_deg: float) -> None:
        self.switched += 1
        self.switch_error_deg_max = max(self.switch_error_deg_max, error_deg)

    def add_lag(self, lag_s: float) -> None:
        self.lagged += 1
        self.lag_s_sum += lag_s
        self.lag_s_max = max(self.lag_s_max, lag_s)

    def add(self, reach_s: float, error_deg: float) -> None:
        self.reached += 1
        self.reach_s_sum += reach_s
        self.reach_s_max = max(self.reach_s_max, reach_s)
        self.error_deg_max = max(self.error_deg_max, error_deg)


@dataclass
class _Pending:
    token: int
    step: ResolvedStep
    plan: HandSpeedPlan
    handle: object
    deadline: float


@dataclass
class _Clock:
    started: float = 0.0
    ends: float = 0.0
    heartbeat_at: float = 0.0
    move_started: float = 0.0
    # (index, switch_due) of a step left by switch whose next goal is not out yet.
    switch_lag: tuple[int, float] | None = None


@dataclass
class Outcome:
    code: int
    reason: str
    summary: dict = field(default_factory=dict)


class Orchestrator:
    def __init__(self, io: Io, cfg: RunConfig) -> None:
        self.io, self.cfg = io, cfg
        self.state = WAIT_ROS
        self.outcome: Outcome | None = None
        self.runner = SequenceRunner()
        self.watch = DwellWatch(cfg.inputs.tolerance_deg, cfg.hold_abort_after_s)
        self.stats = [StepStats(s.preset) for s in cfg.inputs.steps]
        self.sent = 0
        self._cm_seen = False
        # Time spent not ready, summed: a stack that flickers READY must not
        # restart the --startup-timeout bound each time.
        self._waited = 0.0
        self._last_look: float | None = None
        self._reported = -math.inf
        self._disk0: tuple[float, int] | None = None
        self._rate = 0.0
        self._prompted = False
        self._link_required = False
        self._pending: _Pending | None = None
        self._graceful = ""
        self._graceful_code = EXIT_DONE
        self._clock = _Clock()
        self._started_wall = ""
        self._last_obs: Observation | None = None

    # ── Loop entry ───────────────────────────────────────────────────────────

    def step(self) -> Outcome | None:
        if self.outcome is not None:
            return self.outcome
        obs = self.io.observe()
        self._last_obs = obs
        if self.state in (WAIT_ROS, CONFIRM):
            self._before_start(obs)
        elif self.state == RUNNING:
            self._running(obs)
        return self.outcome

    # ── Before RUNNING: nothing is ever published here ───────────────────────

    def _before_start(self, obs: Observation) -> None:
        now = self.io.now()
        since = 0.0 if self._last_look is None else now - self._last_look
        self._last_look = now
        if self.io.terminated() or self.io.interrupts() > 0:
            self._finish(EXIT_INTERRUPTED, "stopped before the start — nothing was sent")
            return
        self._cm_seen = self._cm_seen or obs.cm_nodes > 0
        verdict = assess(obs, self._cm_seen, self.cfg.inputs.calibration)
        if verdict.kind == REFUSE:
            self._finish(EXIT_NOT_READY, verdict.reason)
            return
        if verdict.kind != READY:
            self._waited += since
            if self.state == CONFIRM:
                self.io.say(f"not ready any more — {verdict.reason}")
                self.state, self._prompted, self._disk0 = WAIT_ROS, False, None
            if self._waited >= self.cfg.startup_timeout_s:
                self._finish(
                    EXIT_NOT_READY,
                    f"not ready after {self.cfg.startup_timeout_s:.0f} s — {verdict.reason}"
                    f" [{self.io.environment()}]",
                )
            elif now - self._reported >= WAIT_REPORT_S:
                self._reported = now
                self.io.say(f"{verdict.reason} [{self.io.environment()}]")
            return
        # READY: measure the log growth, then ask (or go, with --yes).
        self.state = CONFIRM
        if self._disk0 is None:
            self._disk0 = (now, self.io.log_bytes())
            return
        t0, b0 = self._disk0
        if now - t0 < DISK_SAMPLE_S:
            return
        if not self._prompted:
            self._rate = max(self.io.log_bytes() - b0, 0) / (now - t0)
            self._prompted = True
            self.io.say(self._plan_text(obs))
            if not self.cfg.assume_yes:
                self.io.say(f"type '{START_WORD}' and Enter to begin (Ctrl-C to quit)")
        if not self.cfg.assume_yes:
            line = self.io.read_line()
            if line is None:
                return
            if line.strip() != START_WORD:
                self.io.say(f"'{line.strip()}' is not '{START_WORD}' — still waiting")
                return
        if problem := disk_verdict(self.io.free_bytes(), self._rate, self.cfg.duration_s):
            self._finish(EXIT_NOT_READY, problem)
            return
        self._start(obs, now)

    def _plan_text(self, obs: Observation) -> str:
        inputs = self.cfg.inputs
        lines = [
            f"robot {self.cfg.robot}: {len(inputs.steps)} steps, loop for "
            f"{self.cfg.duration_s / 3600:.2f} h, tolerance {inputs.tolerance_deg:.1f} deg",
            f"  posture code {inputs.posture_code} — compare it with the code announced "
            "for this robot",
            *([f"  commit   {inputs.provenance}"] if inputs.provenance else []),
            f"  presets  {inputs.preset_path}",
            f"  sequence {inputs.sequence_path}",
            f"  calibration {self._calibration_text(obs)}",
            f"  speed cap {obs.cap:.2f} rad/s, hand link "
            + ("required" if obs.link_pubs > 0 else "not published (sim)"),
            f"  logs grow {self._rate / 1e3:.0f} kB/s, {self.io.free_bytes() / 1e9:.1f} GB free",
        ]
        for i, s in enumerate(inputs.steps, start=1):
            blend = f"  next at {s.next_at_pct:3d} %" if s.next_at_pct < 100 else ""
            lines.append(
                f"  {i:2d}. {s.preset:24s} move {s.time_s:6.2f} s  dwell {s.dwell_s:6.2f} s{blend}"
            )
        return "\n".join(lines)

    def _calibration_text(self, obs: Observation) -> str:
        if self.cfg.inputs.calibration is None:
            return "not checked (no record)"
        if not driver_present(obs):
            return "not checked (no hand driver — sim)"
        return "the running driver's = the recorded one"

    def _start(self, obs: Observation, now: float) -> None:
        self._link_required = obs.link_pubs > 0
        problem = self.runner.start(
            self.cfg.inputs.steps,
            MODE_LOOP,
            self.cfg.inputs.tolerance_deg,
            self.cfg.joint_names,
        )
        if problem:
            self._finish(EXIT_NOT_READY, problem)
            return
        self.state = RUNNING
        self._clock = _Clock(now, now + self.cfg.duration_s, now + self.cfg.heartbeat_s)
        self._started_wall = self.io.wall()
        self.io.say("RUNNING")
        self._write_summary(final=False)

    # ── RUNNING ──────────────────────────────────────────────────────────────

    def _running(self, obs: Observation) -> None:
        now = self.io.now()
        if self.io.terminated() or self.io.interrupts() >= 2:
            self._finish(EXIT_INTERRUPTED, "stopped at once by the operator")
            return
        if self.io.interrupts() == 1:
            self._request_graceful("Ctrl-C: stopping at the end of this cycle (again = now)")
        if now >= self._clock.ends:
            self._request_graceful("the duration is over: stopping at the end of this cycle")
        if reason := runtime_abort(obs, self._link_required):
            self._finish(EXIT_ABORT, reason)
            return
        if self._watch_dwell(obs, now):
            return
        if self._pending is not None and not self._complete_send(obs, now):
            return
        before = (self.runner.state, self.runner.index)
        step = self.runner.tick(now, obs.positions, obs.joint_age)
        self._note_arrival(before, obs, now)
        if not self.runner.running:
            if self.runner.aborted:
                self._finish(EXIT_ABORT, self.runner.result)
            else:
                # Why it stopped (duration, Ctrl-C, disk), not only where.
                self._finish(self._graceful_code, f"{self._graceful} — {self.runner.result}")
            return
        if step is not None:
            self._begin_send(step, obs, now)
        if now >= self._clock.heartbeat_at:
            self._heartbeat(now)

    def _request_graceful(self, why: str, code: int = EXIT_DONE) -> None:
        """Stop at the end of the cycle; ``code`` is the exit it then gives
        (a stop the run did not plan — the disk — is not a success)."""
        if not self._graceful:
            self._graceful, self._graceful_code = why, code
            self.runner.stop_at_cycle_end()
            self.io.say(why)

    def _watch_dwell(self, obs: Observation, now: float) -> bool:
        if self.runner.state != DWELLING:
            self.watch.settle(now)
            return False
        target = self.cfg.inputs.steps[self.runner.index]
        error, joint = worst_error(target.target_rad, obs.positions, self.cfg.joint_names)
        if reason := self.watch.feed(now, self.runner.index + 1, target.preset, error, joint):
            self._finish(EXIT_ABORT, reason)
            return True
        return False

    def _begin_send(self, step: ResolvedStep, obs: Observation, now: float) -> None:
        distance = max(
            (abs(t - m) for t, m in zip(step.target_rad, obs.positions, strict=False)), default=0.0
        )
        plan = plan_hand_speed(distance, step.time_s, obs.cap)
        handle = self.io.set_speed(plan.speed)
        if handle is None:
            self._finish(
                EXIT_ABORT,
                f"the controller's parameter service is not available — '{step.preset}' not sent",
            )
            return
        self._pending = _Pending(self.runner.token, step, plan, handle, now + SPEED_TIMEOUT_S)

    def _complete_send(self, obs: Observation, now: float) -> bool:
        """False when the run ended or the send is still waiting."""
        pending = self._pending
        assert pending is not None
        result = self.io.speed_result(pending.handle)
        if result is None:
            if now <= pending.deadline:
                return False
            self.io.drop_speed(pending.handle)
            self._pending = None
            self._finish(
                EXIT_ABORT,
                f"trajectory_speed set for '{pending.step.preset}' unanswered after "
                f"{SPEED_TIMEOUT_S:.1f} s — goal not sent",
            )
            return False
        self._pending = None
        if not result:
            self._finish(
                EXIT_ABORT,
                f"trajectory_speed {pending.plan.speed:.3f} refused for '{pending.step.preset}'"
                " — goal not sent",
            )
            return False
        if not self.runner.awaiting(pending.token):
            return True  # stopped meanwhile — nothing goes out
        # This tick's stack checks have passed (runtime_abort ran first).
        self.io.publish(pending.step.target_rad)
        # The tick's `now` predates the speed-set answer: the lag is measured
        # at the publish itself.
        published = self.io.now()
        if (lag := self._clock.switch_lag) is not None:
            self.stats[lag[0]].add_lag(max(published - lag[1], 0.0))
            self._clock.switch_lag = None
        self.sent += 1
        self.runner.on_sent(pending.token, now, "", pending.plan.min_duration)
        self._clock.move_started = now
        return True

    def _note_arrival(self, before: tuple[str, int], obs: Observation, now: float) -> None:
        state, index = before
        if state != MOVING or (self.runner.state == MOVING and self.runner.index == index):
            return
        if self.runner.aborted:
            return
        step = self.cfg.inputs.steps[index]
        error, _ = worst_error(step.target_rad, obs.positions, self.cfg.joint_names)
        if self.runner.left_by == LEFT_SWITCHED:
            self.stats[index].add_switch(math.degrees(error))
            if self.runner.left_due is not None:
                self._clock.switch_lag = (index, self.runner.left_due)
            return
        self.stats[index].add(now - self._clock.move_started, math.degrees(error))

    def _heartbeat(self, now: float) -> None:
        self._clock.heartbeat_at = now + self.cfg.heartbeat_s
        elapsed = now - self._clock.started
        self.io.say(
            f"[{elapsed / 3600:5.2f} h] cycle {self.runner.cycle + 1}, {self.sent} goals, "
            f"{len(self.watch.records)} drifts — {self.runner.status(now)}"
        )
        self._write_summary(final=False)
        if not self._graceful:
            rate = max(self.io.log_bytes() - (self._disk0 or (0, 0))[1], 0) / max(
                now - (self._disk0 or (now - 1, 0))[0], 1.0
            )
            remaining = max(self._clock.ends - now, 0.0)
            if problem := disk_verdict(self.io.free_bytes(), rate, remaining):
                self._request_graceful(
                    f"disk: {problem} — stopping at the end of this cycle", EXIT_ABORT
                )

    # ── End ──────────────────────────────────────────────────────────────────

    def _finish(self, code: int, reason: str) -> None:
        if self.runner.running:
            self.runner.stop(reason)
        if self._pending is not None:
            self.io.drop_speed(self._pending.handle)
            self._pending = None
        self.state = DONE
        if self._started_wall and not self._write_summary(final=True, code=code, reason=reason):
            code, reason = EXIT_SUMMARY, f"{reason} (the final summary could not be written)"
        self.io.say(f"exit {code}: {reason}")
        self.outcome = Outcome(code, reason, self.summary(code, reason))

    def summary(self, code: int | None = None, reason: str = "") -> dict:
        now = self.io.now()
        inputs = self.cfg.inputs
        return {
            "robot": self.cfg.robot,
            "started": self._started_wall,
            "updated": self.io.wall(),
            "elapsed_s": round(max(now - self._clock.started, 0.0), 1),
            "duration_s": self.cfg.duration_s,
            "posture_code": inputs.posture_code,
            "commit": inputs.provenance,
            "preset_file": inputs.preset_path,
            "sequence_file": inputs.sequence_path,
            "tolerance_deg": inputs.tolerance_deg,
            "cycles_completed": self.runner.cycle,
            "goals_sent": self.sent,
            "steps": [
                {
                    "step": i + 1,
                    "preset": s.preset,
                    "reached": s.reached,
                    "reach_s_mean": round(s.reach_s_sum / s.reached, 3) if s.reached else None,
                    "reach_s_max": round(s.reach_s_max, 3),
                    "final_error_deg_max": round(s.error_deg_max, 3),
                    "switched": s.switched,
                    "switch_error_deg_max": round(s.switch_error_deg_max, 3),
                    "lag_s_mean": round(s.lag_s_sum / s.lagged, 3) if s.lagged else None,
                    "lag_s_max": round(s.lag_s_max, 3),
                }
                for i, s in enumerate(self.stats)
            ],
            "drifts": len(self.watch.records),
            "drift_records": [vars(r) for r in self.watch.records[-50:]],
            "exit_code": code,
            "end_reason": reason,
        }

    def _write_summary(self, *, final: bool, code: int | None = None, reason: str = "") -> bool:
        try:
            self.io.write_summary(self.summary(code, reason))
            return True
        except OSError as exc:
            self.io.say(f"WARNING: summary not written: {exc}")
            return not final
