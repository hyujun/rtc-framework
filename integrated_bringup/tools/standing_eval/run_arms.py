#!/usr/bin/env python3
"""Stand the floating-base sim profile under its arms and record each run.

One run = one fresh simulator on one time axis: launch (the profile's initial
joint controller holds the keyframe on its MJCF servo gains) → ``--settle`` s
→ the arm's switch step → ``--lead`` s → the stance recorder from its first
message for ``--duration`` s. The arms differ only in the switch step:

  servo   nothing (the MJCF servo holds the robot)            — the baseline
  lowpd   the joint controller's primary-group PD gains are set to the
          locomotion controller's (``pd_feedforward.kp/kd`` parameters), no
          feedforward                                          — PD alone
  loco    /rtc_cm/switch_controller to the locomotion controller; the call
          must answer ok and the active controller must be it  — the latched pose
  mpc     the same switch, the launch with enable_mpc:=true: the controller
          stands on its MPC's plan
  mpc_fb  as mpc, with the plan's Riccati feedback on (sim_overlay:=mpc_riccati)
  reentry the loco switch, then an EVENT SCRIPT under one long recording,
          the operator's way back after a stop: window A (``--duration``) →
          /rtc_cm/trigger_estop → ``--hold`` → /rtc_cm/clear_estop (the CM
          hands back to the joint controller, checked as in estop) → the robot
          stood up again: the joint controller's goal set to the stand pose
          (``estop.arm_safe_position`` of its shipped YAML, the stand
          keyframe's joints) → ``--upright-wait`` → /sim/reset (the whole scene
          back to that keyframe, upright and still) → ``--settle`` → switch to
          the locomotion controller → ``--lead`` → window B (``--duration``)
                                                                    — re-entry
  estop   the loco switch, then: ``--hold-lead`` → /rtc_cm/trigger_estop (its
          reply must say success) → ``--hold`` under the CM's hold →
          /rtc_cm/clear_estop (its reply must say ok and name the post-clear
          switch to the joint controller; the active controller must then be
          ``--joint-controller``) → ``--settle`` → switch to the locomotion
          controller → ``--lead`` → a window (``--duration``)   — the operator stop
  walk    the loco switch with the MPC on (enable_mpc:=true, never mpc_off),
          then the walking command script (twist_script.py, the sim-second
          table rtc_tools.analysis.walk_metrics.COMMAND_SCRIPTS[``--script``],
          default WALK_SCRIPT, published on ``--cmd-topic``): its start is marked once the controller's
          subscription matched; the stop window is its last ``--stop-window``
          s; the run ends when the script has rested             — walking
  walk_estop as walk, and ``--estop-at`` s into the script /rtc_cm/trigger_estop
          (its reply must say success) → a ``hold`` window of ``--hold`` s →
          the script stopped; no clear (its check runs on the wall clock)
                                                                    — a stop while walking

The event arms start the recorder in the background for the whole script
(stopping it once the script is done) and mark every event in run.json with
its wall offset from the recorder's start and its wall-clock stamp — the
recorder CSV's time axis, since the sim stamps its sensor messages with the
wall clock; the windows are judged from the CSV between their marks
(``windows`` in run.json, the stance_metrics numbers), beside the recorder's
own JSON over the whole run. A step whose reply is not what the script expects
ends the run with ``error`` in run.json and a non-zero exit. reentry and
estop run with the plain launch (no MPC) or, under ``--mpc-layout``, with
sim_overlay:=mpc_off like loco; the walk arms always run under the MPC. The
recorder and the command script are stopped whatever ends the script (an
error, an exception): the script first, then the recorder, then the launch.

With ``--mpc-layout`` every arm runs under enable_mpc:=true (the same CPU
layout), and loco (and the two event arms) turn the MPC off again with
sim_overlay:=mpc_off — the control arm of an MPC evaluation. ``--mpc-threads N`` solves on N threads:
every arm launched with enable_mpc:=true also gets layout_profile:=mpc_par_sim
(the simulation worker profile, whose worker cores are the absent drivers'
slots) and mpc_threads:=N. A pool needs mpc_main's SCHED_FIFO (realtime
rights on the host); without them activation is refused. The tool refuses up
front a count the profile cannot seat on this host and a ``--launch`` that
declares no ``mpc_threads`` (the argument would be ignored and the run would
still be recorded as N threads). ``--rtf R`` runs the sim at R × real time
(``max_rtf:=R``): settle, lead and the recording are sim seconds scaled by
1 / R (the sim stamps its sensor messages with the wall clock, so the
recorder's duration, its wall limit and its JSON's times are wall seconds), so
a host slower than the control PCs gives the MPC the solve time per simulated
second that a fast one would. ``max_rtf`` is a ceiling: check the sim reached
it (the CM's timing lines report sim seconds per wall interval).

``--source inekf`` runs every arm on the estimated base (sim_overlay:=
estimator_inekf); an arm that already runs with an overlay gets the two merged
into one file under ``<out>/overlays/`` (one overlay per launch; two that set
one key to different values are refused up front). run.json records the source.

Each run's recorder CSV / JSON and the controller log session land in
``<out>/<arm>_<k>/``. Run it under ``repo_scripts/scripts/with_verify_hold.sh``
so no build starts between runs, with the workspace environment sourced.
"""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import asdict
from pathlib import Path

import yaml

from rtc_tools.analysis import stance_metrics as sm, walk_metrics as wm
from rtc_tools.analysis.weight_overlay import WeightOverlayError, merge_overlay_files
from rtc_tools.launch.cpu_shield import mpc_solver_threads
from rtc_tools.validation.model_match import model_match

LOCO = "locomotion_controller"
ARMS = ("servo", "loco", "lowpd", "mpc", "mpc_fb", "reentry", "estop", "walk", "walk_estop")
# arms whose switch step activates the locomotion controller without its MPC
# under --mpc-layout (the walk arms activate it with its MPC, always)
LOCO_ARMS = ("loco", "reentry", "estop")
WALK_ARMS = ("walk", "walk_estop")
# a script (SCRIPTS) under a background recording
EVENT_ARMS = ("reentry", "estop", *WALK_ARMS)
# The state-source overlay of ``--source inekf`` (config/g1_loco/sim_overlays/).
INEKF_OVERLAY = "estimator_inekf"
TWIST_SCRIPT = Path(__file__).resolve().parent / "twist_script.py"
TWIST_START_TIMEOUT_S = 90.0  # [wall] the script's wait for the controller's subscription
WORKER_PROFILE = "mpc_par_sim"  # the simulation worker profile (--mpc-threads)
OPERATOR_ESTOP_REASON = "operator_request"  # the CM's reason for /rtc_cm/trigger_estop
# The event arms' recorder is stopped (SIGINT) once the script is done; its
# own --duration is only a bound, this much past the script's sleeps: every
# call is a `ros2` CLI call (0.5-1.5 s of start-up and discovery each) the
# sleeps do not count.
RECORDER_MARGIN_S = 30.0
RECORDER_GRACE_S = 1.0  # recording past the last mark before the stop


def default_overlay(arm: str, mpc_layout: bool) -> str | None:
    """The sim_overlay an arm runs with when none is given (module docstring)."""
    if arm in LOCO_ARMS and mpc_layout:
        return "mpc_off"
    return {"mpc_fb": "mpc_riccati"}.get(arm)


def source_overlays(
    arms: list[str],
    mpc_layout: bool,
    overlays: dict[str, str],
    source: str,
    merge_dir: Path | None = None,
    profile: str = "",
) -> dict[str, str]:
    """`overlays` with the state source's overlay on every arm (``--source``).

    truth is the shipped source: nothing changes. inekf runs each arm with
    sim_overlay:=estimator_inekf. A launch takes one overlay, so an arm that
    already has one (its own or a given one) gets the two merged into
    ``<merge_dir>/<arm>_inekf.yaml`` (rtc_tools weight_overlay: a leaf the two
    set to different values is an error; the components and their hashes go to
    the file's ``.meta.json``).
    """
    out = dict(overlays)
    for arm in arms if source == "inekf" else ():
        own = overlays.get(arm, default_overlay(arm, mpc_layout))
        if not own:
            out[arm] = INEKF_OVERLAY
            continue
        if merge_dir is None:
            raise ValueError(
                f"--source {source}: arm {arm} runs with sim_overlay {own}; no merge dir"
            )
        from integrated_bringup.sim_overlay import resolve_sim_overlay

        merged = merge_dir / f"{arm}_{source}.yaml"
        try:
            merge_overlay_files(
                [Path(resolve_sim_overlay(o, profile)) for o in (own, INEKF_OVERLAY)], merged
            )
        except (WeightOverlayError, RuntimeError) as e:
            raise ValueError(f"--source {source}: arm {arm}: {e}") from e
        out[arm] = str(merged)
    return out


def launch_args(
    arm: str, mpc_layout: bool, overlays: dict[str, str], mpc_threads: int | None = None
) -> list[str]:
    """The arm's launch arguments (module docstring); `overlays` replaces an arm's overlay."""
    under_mpc = arm in ("mpc", "mpc_fb", *WALK_ARMS) or mpc_layout
    overlay = overlays.get(arm, default_overlay(arm, mpc_layout))
    pool = (
        [f"layout_profile:={WORKER_PROFILE}", f"mpc_threads:={mpc_threads}"]
        if under_mpc and mpc_threads is not None
        else []
    )
    return (
        (["enable_mpc:=true"] if under_mpc else [])
        + pool
        + ([f"sim_overlay:={overlay}"] if overlay else [])
    )


def check_mpc_threads(p: argparse.ArgumentParser, threads: int, launch: str) -> None:
    """Refuse ``--mpc-threads`` before the first run rather than once per run.

    The launch would refuse a count its profile cannot seat — after the tool
    has waited out the sensor-topic timeout, on every arm and every k.
    """
    try:
        mpc_solver_threads(str(threads), WORKER_PROFILE)
    except RuntimeError as e:
        p.error(str(e))
    shown = sh(["ros2", "launch", "integrated_bringup", launch, "--show-args"], timeout=60.0)
    if "'mpc_threads':" not in shown.stdout:
        p.error(f"{launch} declares no mpc_threads argument")


def sh(cmd: list[str], timeout: float = 30.0) -> subprocess.CompletedProcess:
    """Run a command; a timeout is a failed result (rc 124), not an exception."""
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, check=False)
    except subprocess.TimeoutExpired as exc:
        out = exc.stdout.decode() if isinstance(exc.stdout, bytes) else (exc.stdout or "")
        return subprocess.CompletedProcess(cmd, 124, out, "timeout")


def wait_topic(topic: str, timeout: float) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        r = sh(["ros2", "topic", "echo", "--once", topic], timeout=15.0)
        if r.returncode == 0 and r.stdout.strip():
            return True
        time.sleep(0.5)
    return False


def active_controller() -> str:
    r = sh(
        ["ros2", "topic", "echo", "--once", "--field", "data", "/rtc_cm/active_controller_name"]
    )
    return r.stdout.strip().splitlines()[0].strip() if r.stdout.strip() else ""


def gains(runtime_yaml: Path) -> tuple[list[float], list[float]]:
    pd = yaml.safe_load(runtime_yaml.read_text())[LOCO]["pd_feedforward"]
    return [float(x) for x in pd["kp"]], [float(x) for x in pd["kd"]]


def service_call(service: str, srv_type: str, request: str) -> str:
    """``ros2 service call``; its stdout (the reply's fields print as `name=value`)."""
    return sh(["ros2", "service", "call", service, srv_type, request]).stdout


def switch_controller(name: str) -> tuple[bool, str, str]:
    """Activate `name`; (the reply said ok AND it is the active controller, the reply, the active one)."""
    req = f"{{activate_controllers: ['{name}'], deactivate_controllers: [], strictness: 1}}"
    out = service_call("/rtc_cm/switch_controller", "rtc_msgs/srv/SwitchController", req)
    if "ok=True" not in out:
        return False, out.strip()[-300:], ""
    time.sleep(0.3)
    active = active_controller()
    return active == name, out.strip()[-300:], active


def trigger_estop() -> tuple[bool, str]:
    """The operator stop; (the reply said success, the reply)."""
    out = service_call("/rtc_cm/trigger_estop", "std_srvs/srv/Trigger", "{}")
    return "success=True" in out, out.strip()[-300:]


def clear_estop(reason_ack: str) -> tuple[bool, str]:
    """Clear the latch; (the reply said ok, the reply)."""
    out = service_call(
        "/rtc_cm/clear_estop", "rtc_msgs/srv/ClearEstop", f"{{reason_ack: '{reason_ack}'}}"
    )
    return "ok=True" in out, out.strip()[-400:]


def post_clear_switch_ok(reply: str, joint_controller_class: str | None) -> bool:
    """The clear's reply reports the post-clear switch (estop_on_clear) as ok.

    The CM names the controller by its class (`Name()`), not its config key;
    with `joint_controller_class` None only the ok is checked.
    """
    marker = "post-clear switch to '"
    i = reply.find(marker)
    if i < 0:
        return False
    rest = reply[i + len(marker) :]
    name, sep, tail = rest.partition("': ")
    if not sep or not tail.startswith("ok"):
        return False
    return joint_controller_class is None or name == joint_controller_class


def switch_step(arm: str, args: argparse.Namespace, note: dict) -> bool:
    if arm == "servo":
        return True
    if arm == "lowpd":
        kp, kd = gains(args.runtime_yaml)
        for name, vals in (("pd_feedforward.kp", kp), ("pd_feedforward.kd", kd)):
            r = sh(["ros2", "param", "set", args.joint_node, name, json.dumps(vals)])
            note[name] = r.stdout.strip() or r.stderr.strip()
            if "successful" not in r.stdout.lower():
                return False
        return True
    ok, reply, active = switch_controller(LOCO)
    note["switch"] = reply
    note["active_controller"] = active
    return ok


class WallClock:
    """The event scripts' clock (a test substitutes its own).

    `now` is the monotonic clock the marks' offsets are on; `epoch` is the
    wall clock, which is the recorder's time axis: the simulator stamps its
    sensor messages with it (module docstring), so a mark's `sim_stamp` is
    read here instead of from a message.
    """

    def sleep(self, seconds: float) -> None:
        time.sleep(seconds)

    def now(self) -> float:
        return time.monotonic()

    def epoch(self) -> float:
        return time.time()


def _switch_loco(args: argparse.Namespace) -> tuple[bool, dict, str]:
    ok, reply, active = switch_controller(LOCO)
    return (
        ok,
        {"active_controller": active, "reply": reply},
        "the switch to the locomotion controller failed",
    )


def stand_goal(
    runtime_yaml: Path, joint_controller: str, group: str
) -> tuple[list[str], list[float]]:
    """The joint controller's stand pose: its ``estop.arm_safe_position`` (the
    stand keyframe's joints), named by the profile's ``devices.<group>`` roster.

    Both files are the profile's own, found from the runtime fragment's place
    (``config/<profile>/controllers/locomotion/runtime.yaml``).
    """
    profile = runtime_yaml.resolve().parents[2]
    ctrl = yaml.safe_load((profile / "controllers" / f"{joint_controller}.yaml").read_text())
    pose = [float(x) for x in ctrl[joint_controller]["estop"]["arm_safe_position"]]

    def find_devices(node):
        if isinstance(node, dict):
            if isinstance(node.get("devices"), dict) and group in node["devices"]:
                return node["devices"][group]
            for v in node.values():
                found = find_devices(v)
                if found is not None:
                    return found
        return None

    device = find_devices(yaml.safe_load((profile / "_base.yaml").read_text()))
    if device is None:
        raise ValueError(f"{profile}/_base.yaml has no devices.{group}")
    names = [str(n) for n in device["joint_state_names"]]
    if len(names) != len(pose):
        raise ValueError(f"{len(pose)} stand-pose values for {len(names)} joints of {group}")
    return names, pose


def _stand_goal(args: argparse.Namespace) -> tuple[bool, dict, str]:
    names, pose = stand_goal(args.runtime_yaml, args.joint_controller, args.body_group)
    topic = f"/{args.joint_controller}/{args.body_group}/joint_goal"
    msg = (
        "{goal_type: joint, joint_names: ["
        + ", ".join(names)
        + "], joint_target: ["
        + ", ".join(repr(x) for x in pose)
        + "]}"
    )
    # -w 1: the active joint controller owns the topic; wait for it, then once.
    r = sh(["ros2", "topic", "pub", "--once", "-w", "1", topic, "rtc_msgs/msg/RobotTarget", msg])
    return r.returncode == 0, {"topic": topic}, f"the stand goal was not published on {topic}"


def _sim_reset(args: argparse.Namespace) -> tuple[bool, dict, str]:
    out = service_call("/sim/reset", "std_srvs/srv/Trigger", "{}")
    return (
        "success=True" in out,
        {"reply": out.strip()[-200:]},
        "/sim/reset did not report success",
    )


def _trigger_estop(args: argparse.Namespace) -> tuple[bool, dict, str]:
    ok, reply = trigger_estop()
    return ok, {"reply": reply}, "trigger_estop did not report success"


def _clear_estop(args: argparse.Namespace) -> tuple[bool, dict, str]:
    ok, reply = clear_estop(OPERATOR_ESTOP_REASON)
    switched = post_clear_switch_ok(reply, args.joint_controller_class)
    active = active_controller()
    fields = {"post_clear_switch_ok": switched, "active_controller": active, "reply": reply}
    if not ok:
        return False, fields, "clear_estop did not report ok"
    if not switched:
        return False, fields, "the clear's reply does not report the post-clear switch as ok"
    if active != args.joint_controller:
        return False, fields, f"after the clear the active controller is {active!r}"
    return True, fields, ""


def stop_group(proc: subprocess.Popen | None, timeout: float = 60.0) -> int | None:
    """SIGINT to the process's group, then SIGKILL after `timeout`; its exit code."""
    if proc is None:
        return None
    if proc.poll() is None:
        with contextlib.suppress(ProcessLookupError):
            os.killpg(proc.pid, signal.SIGINT)
    try:
        return proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        with contextlib.suppress(ProcessLookupError):
            os.killpg(proc.pid, signal.SIGKILL)
        return proc.wait()


def read_events(path: Path) -> list[dict]:
    """The JSON lines of a command script's log (anything else is skipped)."""
    out = []
    if path.is_file():
        for line in path.read_text().splitlines():
            try:
                out.append(json.loads(line))
            except ValueError:
                continue
    return out


def push_arguments(args: argparse.Namespace) -> list[str]:
    """The command script's push arguments (``--push``); none without a push."""
    push = getattr(args, "push", None)  # absent on a caller that never parsed --push
    if push is None:
        return []
    return ["--push", *(str(x) for x in push), "--push-body", args.push_body]


def friction_overlay(mu: float, sim_node: str) -> dict:
    """A params overlay setting the scene's sliding friction to `mu` for every contact.

    The simulator's contact override replaces each contact's parameters with
    one set; the others are written at the simulator's defaults (MuJoCo's own),
    so on a scene that does not set them only the sliding friction changes.
    """
    return {
        sim_node: {
            "ros__parameters": {
                "solver": {
                    "contact_override": {
                        "enable": True,
                        "o_friction": [float(mu), float(mu), 0.005, 0.0001, 0.0001],
                    }
                }
            }
        }
    }


def arm_overlays(
    arms: list[str], given: list[str], friction: float | None, merge_dir: Path, profile: str,
    sim_node: str = "mujoco_simulator",
) -> dict[str, str]:  # fmt: skip
    """{arm: its one overlay} from the ``--overlay ARM=…`` list and ``--friction``.

    An arm named once keeps that value as given. An arm with several pieces
    (named more than once, or once plus the friction piece) gets them merged
    into ``<merge_dir>/<arm>.yaml`` — a launch takes one overlay.
    """
    pieces: dict[str, list[str]] = {}
    for item in given:
        arm, _, value = item.partition("=")
        if not value:
            raise ValueError(f"--overlay {item}: expected ARM=NAME-or-PATH")
        pieces.setdefault(arm, []).append(value)
    if friction is not None:
        if not (friction > 0.0):
            raise ValueError(f"--friction must be > 0, got {friction}")
        merge_dir.mkdir(parents=True, exist_ok=True)
        path = merge_dir / "friction.yaml"
        path.write_text(yaml.safe_dump(friction_overlay(friction, sim_node)))
        for arm in arms:
            pieces.setdefault(arm, []).append(str(path))
    out: dict[str, str] = {}
    for arm, values in pieces.items():
        if len(values) == 1:
            out[arm] = values[0]
            continue
        from integrated_bringup.sim_overlay import resolve_sim_overlay

        merged = merge_dir / f"{arm}.yaml"
        try:
            merge_overlay_files([Path(resolve_sim_overlay(v, profile)) for v in values], merged)
        except (WeightOverlayError, RuntimeError) as e:
            raise ValueError(f"--overlay {arm}: {e}") from e
        out[arm] = str(merged)
    return out


def _twist_start(args: argparse.Namespace) -> tuple[bool, dict, str]:
    """Start the command script; ok once it reports its start (the subscriber matched)."""
    log_path = args.run_out / "twist.log"
    with open(log_path, "w") as log:
        args.twist_proc = subprocess.Popen(
            [
                sys.executable,
                str(TWIST_SCRIPT),
                "--topic",
                args.cmd_topic,
                "--rtf",
                str(args.rtf or 1.0),
                "--script",
                args.script,
                *push_arguments(args),
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    end = time.monotonic() + TWIST_START_TIMEOUT_S
    while time.monotonic() < end:
        exited = args.twist_proc.poll() is not None  # before the read: its last line counts
        started = [e for e in read_events(log_path) if e.get("event") == "start"]
        if started:
            return True, {"script_wall": started[0].get("wall")}, ""
        if exited:
            break
        time.sleep(0.05)
    return False, {"rc": args.twist_proc.poll()}, "the command script did not start"


def _twist_end(args: argparse.Namespace) -> tuple[bool, dict, str]:
    """The script's own end (its last sleep and the rest commands)."""
    try:
        rc = args.twist_proc.wait(timeout=60.0)
    except subprocess.TimeoutExpired:
        rc = stop_group(args.twist_proc)
    args.twist_proc = None
    return rc == 0, {"rc": rc}, "the command script did not end cleanly"


def _twist_stop(args: argparse.Namespace) -> tuple[bool, dict, str]:
    """Stop the script now (it rests on the way out)."""
    rc = stop_group(args.twist_proc)
    args.twist_proc = None
    return rc is not None, {"rc": rc}, "the command script was not running"


def walk_until_stop_window(args: argparse.Namespace) -> float:
    """The script's length before its last ``--stop-window`` s [s, sim]."""
    return wm.script_length(wm.COMMAND_SCRIPTS[args.script]) - args.stop_window


# The event scripts (module docstring), one step per entry: ("sleep", its
# length), ("window", its name[, its length] — `--duration` long without one,
# marked at both ends), ("call", its mark name, the action). A length is the
# argument holding it or a function of the arguments. An action returns
# (ok, the fields its mark carries, what failed). The recorder's length is
# derived from the same table (script_duration).
SCRIPTS = {
    "reentry": (
        ("window", "window_a"),
        ("call", "trigger_estop", _trigger_estop),
        ("sleep", "hold"),
        ("call", "clear_estop", _clear_estop),
        ("call", "stand_goal", _stand_goal),
        ("sleep", "upright_wait"),
        ("call", "sim_reset", _sim_reset),
        ("sleep", "settle"),
        ("call", "switch_loco", _switch_loco),
        ("sleep", "lead"),
        ("window", "window_b"),
    ),
    "estop": (
        ("sleep", "hold_lead"),
        ("call", "trigger_estop", _trigger_estop),
        ("sleep", "hold"),
        ("call", "clear_estop", _clear_estop),
        ("sleep", "settle"),
        ("call", "switch_loco", _switch_loco),
        ("sleep", "lead"),
        ("window", "window"),
    ),
    "walk": (
        ("call", "twist_start", _twist_start),
        ("sleep", walk_until_stop_window),
        ("window", "stop", "stop_window"),
        ("call", "twist_end", _twist_end),
    ),
    "walk_estop": (
        ("call", "twist_start", _twist_start),
        ("sleep", "estop_at"),
        ("call", "trigger_estop", _trigger_estop),
        ("window", "hold", "hold"),
        ("call", "twist_stop", _twist_stop),
    ),
}


def step_length(args: argparse.Namespace, length) -> float:
    """A step's length [s, sim]: the argument it names, or the function of the arguments."""
    return length(args) if callable(length) else getattr(args, length)


def window_length(args: argparse.Namespace, step: tuple) -> float:
    """A window step's length [s, sim]: its own, or `--duration`."""
    return step_length(args, step[2]) if len(step) > 2 else args.duration


def run_events(arm: str, args: argparse.Namespace, note: dict, clock, t0: float) -> bool:
    """The arm's event script (SCRIPTS) with the recorder already running.

    Every call and every window end appends a mark to note["events"] — its
    name, the wall offset from `t0` (the recorder's start) and the wall-clock
    stamp (`sim_stamp`, the recorder CSV's time axis); a call whose reply is
    not the expected one stops the script with note["error"]. The windows'
    marks bound the judged intervals.
    """
    wall = 1.0 / args.rtf if args.rtf else 1.0
    events: list[dict] = note.setdefault("events", [])

    def mark(name: str, **fields) -> None:
        events.append(
            {"name": name, "wall_s": clock.now() - t0, "sim_stamp": clock.epoch(), **fields}
        )

    for step in SCRIPTS[arm]:
        if step[0] == "sleep":
            clock.sleep(step_length(args, step[1]) * wall)
        elif step[0] == "window":
            mark(f"{step[1]}_start")
            clock.sleep(window_length(args, step) * wall)
            mark(f"{step[1]}_end")
        else:
            ok, fields, what = step[2](args)
            mark(step[1], ok=ok, **fields)
            if not ok:
                note["error"] = what
                return False
    return True


def script_duration(arm: str, args: argparse.Namespace) -> float:
    """The event script's length [s, sim]: its sleeps and windows (SCRIPTS)."""
    total = 0.0
    for step in SCRIPTS[arm]:
        if step[0] == "sleep":
            total += step_length(args, step[1])
        elif step[0] == "window":
            total += window_length(args, step)
    return total


def window_metrics(csv_path: Path, events: list[dict], z_ref: float) -> dict:
    """stance_metrics over the CSV between each window's start and end marks."""
    if not csv_path.is_file():
        return {}
    samples = sm.read_csv(csv_path)
    marks = {e["name"]: e.get("sim_stamp") for e in events}
    out: dict = {}
    for name in sorted({n[: -len("_start")] for n in marks if n.endswith("_start")}):
        t_start, t_end = marks.get(f"{name}_start"), marks.get(f"{name}_end")
        if t_start is None or t_end is None:
            out[name] = {"error": "a window mark has no sim stamp"}
            continue
        inside = [s for s in samples if t_start <= s.t <= t_end]
        try:
            m = sm.compute(inside, settle_s=0.0, force_threshold_n=5.0, z_ref=z_ref)
        except ValueError as e:
            out[name] = {"error": str(e)}
            continue
        out[name] = {"t_start": t_start, "t_end": t_end, **asdict(m)}
    return out


def solver_threads(launch_log: Path) -> int | None:
    """The thread count the MPC thread started with, from the controller's log line."""
    found = re.findall(r"MPC thread started: .*?(\d+) solver thread\(s\)", launch_log.read_text())
    return int(found[-1]) if found else None


def newest_session(root: Path, after: float) -> Path | None:
    """The session a run wrote: the one holding the newest CSV written after `after`.

    Session folders are named to the minute, so two runs can share one and the
    folder's own mtime does not move when a file inside it is rewritten.
    """
    if not root.is_dir():
        return None
    best, best_t = None, after
    for d in root.iterdir():
        if not d.is_dir():
            continue
        for f in d.rglob("*.csv"):
            t = f.stat().st_mtime
            if t >= best_t:
                best, best_t = d, t
    return best


# ── provenance ────────────────────────────────────────────────────────────

REPO = Path(__file__).resolve().parents[3]
CONFIG_SOURCE = Path(__file__).resolve().parents[2] / "config"
PLANT_LINE = re.compile(r"\[MuJoCoSimulator\] Loaded '([^']+)'")


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tree_sha256(root: Path, files: list[Path]) -> str | None:
    """One hash of `files` (paths relative to `root`) read under `root`; None if one is missing."""
    digest = hashlib.sha256()
    for rel in files:
        path = root / rel
        if not path.is_file():
            return None
        digest.update(str(rel).encode() + b"\0" + path.read_bytes() + b"\0")
    return digest.hexdigest()


def config_hashes(profile: str, installed_root: Path, source_root: Path = CONFIG_SOURCE) -> dict:
    """The profile's configuration as the launch read it against the checkout's.

    The launch reads the installed copy, and parts of an install tree are
    copies made at build time rather than links: a build older than the
    checkout runs other values than the ones the commit names. The file list is
    the checkout's, so a file an older build left behind does not count.
    """
    source = source_root / profile
    files = sorted(p.relative_to(source) for p in source.rglob("*") if p.is_file())
    return {
        "profile": profile,
        "files": len(files),
        "source_sha256": tree_sha256(source, files),
        "installed_sha256": tree_sha256(installed_root / profile, files),
    }


def overlay_record(launch_args: list[str], profile: str) -> dict | None:
    """The run's overlay file: its path, hash and the generator's note beside it, if any.

    The note is ``<overlay>.meta.json`` (what the overlay was generated from —
    the search parameters of a tuning candidate).
    """
    values = [a.split(":=", 1)[1] for a in launch_args if a.startswith("sim_overlay:=")]
    if not values:
        return None
    from integrated_bringup.sim_overlay import resolve_sim_overlay

    path = Path(resolve_sim_overlay(values[-1], profile))
    record: dict = {"path": str(path), "sha256": file_sha256(path)}
    meta = path.with_suffix(".meta.json")
    if meta.is_file():
        record["meta"] = json.loads(meta.read_text())
    return record


def find_urdf(node) -> dict | None:
    """The first ``urdf: {package, path}`` mapping of a parameter tree."""
    if isinstance(node, dict):
        urdf = node.get("urdf")
        if isinstance(urdf, dict) and "package" in urdf and "path" in urdf:
            return urdf
        for value in node.values():
            found = find_urdf(value)
            if found is not None:
                return found
    return None


def model_record(base_yaml: Path, launch_log: str) -> dict:
    """The controller's URDF against the plant the simulator loaded (model_match)."""
    from ament_index_python.packages import get_package_share_directory

    loaded = PLANT_LINE.search(launch_log)
    urdf = find_urdf(yaml.safe_load(base_yaml.read_text()))
    if loaded is None or urdf is None:
        return {"error": "no plant line in the launch log" if loaded is None else "no urdf"}
    urdf_path = Path(get_package_share_directory(urdf["package"])) / urdf["path"]
    plant = Path(loaded.group(1))
    try:
        match = model_match(urdf_path, plant, urdf.get("root_joint_type") == "floating")
    except Exception as exc:  # an unverified model is recorded as such, never as a match
        return {
            "urdf": str(urdf_path),
            "plant": str(plant),
            "error": f"{type(exc).__name__}: {exc}",
        }
    return {
        "urdf": str(urdf_path),
        "plant": str(plant),
        "plant_sha256": file_sha256(plant),
        **match,
    }


def git_state(repo: Path = REPO) -> dict:
    """The checkout's commit and whether anything tracked or unignored differs from it."""

    def git(*cmd: str) -> str | None:
        r = subprocess.run(["git", "-C", str(repo), *cmd], capture_output=True, text=True)
        return r.stdout.strip() if r.returncode == 0 else None

    status = git("status", "--porcelain")
    return {"commit": git("rev-parse", "HEAD"), "dirty": None if status is None else bool(status)}


def provenance(profile: str, launch_args: list[str], launch_log: str) -> dict:
    """What a judge needs to tell this run's build, configuration and plant from another's."""
    from ament_index_python.packages import get_package_share_directory

    installed = Path(get_package_share_directory("integrated_bringup")) / "config"
    return {
        **git_state(),
        "config": config_hashes(profile, installed),
        "overlay": overlay_record(launch_args, profile),
        "model_match": model_record(installed / profile / "_base.yaml", launch_log),
    }


def run_once(arm: str, k: int, args: argparse.Namespace) -> dict:
    out = args.out / f"{arm}_{k}"
    out.mkdir(parents=True, exist_ok=True)
    extra = launch_args(arm, args.mpc_layout, args.overlay, args.mpc_threads) + args.launch_arg
    wall = 1.0
    if args.rtf:
        extra.append(f"max_rtf:={args.rtf}")
        wall = 1.0 / args.rtf
    # The run's own settings the judge (walk_gate.py) reads back.
    note: dict = {
        "arm": arm,
        "k": k,
        "launch_args": extra,
        "rtf": args.rtf,
        "z_ref": args.z_ref,
        "stop_window": args.stop_window,
        "source": args.source,
        "script": args.script,
        # The disturbances the run was given (None = none), for the judge.
        "scenario": {
            "push": None
            if args.push is None
            else dict(
                zip(("t_s", "duration_s", "fx", "fy", "fz"), args.push, strict=True),
                body=args.push_body,
                point=[0.0, 0.0, 0.0],
            ),
            "friction": args.friction,
        },
    }
    args.run_out = out
    args.twist_proc = None
    t_start = time.time()
    with open(out / "launch.log", "w") as log:
        launch = subprocess.Popen(
            ["ros2", "launch", "integrated_bringup", args.launch, "enable_viewer:=false", *extra],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            if not wait_topic(args.sensor_topic, 90.0):
                note["error"] = "no sensor topic"
                return note
            time.sleep(args.settle * wall)
            note["switch_ok"] = switch_step(arm, args, note)
            if not note["switch_ok"]:
                note["error"] = "switch step failed"
                return note
            time.sleep(args.lead * wall)
            event_arm = arm in EVENT_ARMS
            record_s = (
                script_duration(arm, args) + RECORDER_MARGIN_S if event_arm else args.duration
            ) * wall
            recorder_cmd = [
                "ros2",
                "run",
                "rtc_tools",
                "run_stance_metrics",
                "--sensor-topic",
                args.sensor_topic,
                "--pos",
                "pelvis_pos",
                "--quat",
                "pelvis_quat",
                "--foot-pos",
                "left_sole_pos",
                "right_sole_pos",
                "--contact",
                "left_sole_contact",
                "right_sole_contact",
                "--contact-sign",
                "-1",
                "--duration",
                str(record_s),
                "--settle",
                "0",
                "--z-ref",
                str(args.z_ref),
                "--start-now",
                "--csv",
                str(out / "stance.csv"),
                "--json",
                str(out / "stance.json"),
            ]
            if event_arm:
                # The recorder in the background for the whole script; the
                # script's marks window the CSV afterwards.
                with open(out / "recorder.log", "w") as rec_log:
                    # Its own session: `ros2 run` does not pass SIGINT on —
                    # the group gets it, and the recorder writes its CSV and
                    # JSON on that interrupt.
                    recorder = subprocess.Popen(
                        recorder_cmd,
                        stdout=rec_log,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,
                    )
                    try:
                        t0 = time.monotonic()
                        note["events_ok"] = run_events(arm, args, note, WallClock(), t0)
                        time.sleep(RECORDER_GRACE_S * wall)
                    finally:
                        # The script first (it rests the command), then the recorder —
                        # the recorder whatever stopping the script raised.
                        try:
                            if args.twist_proc is not None:
                                note["twist_rc"] = stop_group(args.twist_proc)
                                args.twist_proc = None
                        finally:
                            note["recorder_rc"] = stop_group(recorder)
                if arm in WALK_ARMS:
                    note["twist_events"] = read_events(out / "twist.log")
                note["windows"] = window_metrics(out / "stance.csv", note["events"], args.z_ref)
            else:
                rec = sh(recorder_cmd, timeout=args.duration * 4 * wall + 60)
                note["recorder_rc"] = rec.returncode
                (out / "recorder.log").write_text(rec.stdout + rec.stderr)
            note["active_controller_end"] = active_controller()
        finally:
            stop_group(launch, timeout=20.0)
            session = newest_session(args.logging_root, t_start)
            if session is not None:
                # Only this run's files: a folder shared with an earlier run keeps its too.
                for f in session.rglob("*"):
                    if f.is_file() and f.stat().st_mtime >= t_start:
                        dst = out / "session" / f.relative_to(session)
                        dst.parent.mkdir(parents=True, exist_ok=True)
                        shutil.copy2(f, dst)
                note["session"] = session.name
            # What the controller ran, beside what the tool asked for.
            note["solver_threads"] = solver_threads(out / "launch.log")
            try:
                note["provenance"] = provenance(
                    args.profile, extra, (out / "launch.log").read_text(errors="replace")
                )
            except Exception as exc:  # the run's data is kept; the judge refuses it
                note["provenance_error"] = f"{type(exc).__name__}: {exc}"
            (out / "run.json").write_text(json.dumps(note, indent=2) + "\n")
    return note


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--out", type=Path, required=True)
    p.add_argument(
        "--arms",
        nargs="+",
        default=["servo", "loco", "lowpd"],
        choices=ARMS,
    )
    p.add_argument(
        "--mpc-layout",
        action="store_true",
        help="every arm under enable_mpc:=true; loco with sim_overlay:=mpc_off",
    )
    p.add_argument(
        "--mpc-threads",
        type=int,
        default=None,
        metavar="N",
        help="MPC solver threads, on the simulation worker profile (module docstring)",
    )
    p.add_argument("-k", type=int, default=5, help="runs per arm")
    p.add_argument("--launch", default="sim_g1_loco.launch.py")
    p.add_argument(
        "--profile",
        help="the launch's config profile (config/<profile>); default: from --launch "
        "(sim_<profile>.launch.py)",
    )
    p.add_argument(
        "--runtime-yaml",
        type=Path,
        required=True,
        help="the locomotion controller's runtime fragment (its PD gains)",
    )
    p.add_argument(
        "--overlay",
        action="append",
        default=[],
        metavar="ARM=OVERLAY",
        help="the arm's sim_overlay (a shipped name or a path), replacing its own; "
        "repeat it for one arm to merge several into one file",
    )
    p.add_argument(
        "--push",
        type=float,
        nargs=5,
        metavar=("T", "DUR", "FX", "FY", "FZ"),
        help="walk arms: one push at T sim seconds into the command script, for DUR sim "
        "seconds, a world-axis force [N] at the origin of --push-body",
    )
    p.add_argument("--push-body", help="the simulated body --push acts on (as the scene names it)")
    p.add_argument(
        "--friction",
        type=float,
        help="the scene's sliding friction for every contact (the simulator's contact "
        "override, merged into each arm's overlay)",
    )
    p.add_argument(
        "--launch-arg",
        action="append",
        default=[],
        metavar="NAME:=VALUE",
        help="passed to every launch (e.g. max_log_sessions:=200 while sessions are collected)",
    )
    p.add_argument("--joint-node", default="/demo_joint_controller/demo_joint_controller")
    p.add_argument(
        "--joint-controller",
        default="demo_joint_controller",
        help="the joint controller's config key (the switch service, active_controller_name)",
    )
    p.add_argument(
        "--joint-controller-class",
        default="DemoJointController",
        help="its class name, as the clear's reply names the post-clear switch target",
    )
    p.add_argument(
        "--hold-lead", type=float, default=2.0, help="estop: recording before the stop [s]"
    )
    p.add_argument(
        "--hold", type=float, default=10.0, help="estop / reentry: time under the hold [s]"
    )
    p.add_argument(
        "--upright-wait",
        type=float,
        default=4.0,
        help="reentry: the joint controller's time to reach the stand goal before /sim/reset [s]",
    )
    p.add_argument(
        "--body-group", default="g1", help="reentry: the body device group (its roster)"
    )
    p.add_argument(
        "--cmd-topic",
        default="/locomotion_controller/g1/cmd_vel",
        help="walk arms: the controller's Twist topic",
    )
    p.add_argument(
        "--stop-window",
        type=float,
        default=3.0,
        help="walk: the judged stand at the script's end, its last seconds [s, sim]",
    )
    p.add_argument(
        "--estop-at",
        type=float,
        default=8.0,
        help="walk_estop: the stop's time into the command script [s, sim]",
    )
    p.add_argument("--sensor-topic", default="/g1/sim_sensors")
    p.add_argument("--z-ref", type=float, default=0.764431)
    p.add_argument("--settle", type=float, default=2.0)
    p.add_argument("--lead", type=float, default=1.0)
    p.add_argument("--duration", type=float, default=10.0)
    p.add_argument(
        "--rtf", type=float, default=None, help="max_rtf for the sim (module docstring)"
    )
    p.add_argument("--logging-root", type=Path, required=True)
    p.add_argument(
        "--script",
        choices=sorted(wm.COMMAND_SCRIPTS),
        default="walk",
        help="the walk arms' command script (walk_metrics.COMMAND_SCRIPTS; walk = WALK_SCRIPT)",
    )
    p.add_argument(
        "--source",
        choices=("truth", "inekf"),
        default="truth",
        help="the controller's state source; inekf = sim_overlay:=estimator_inekf on every arm",
    )
    args = p.parse_args(argv)
    if args.profile is None:
        named = re.fullmatch(r"sim_(.+)\.launch\.py", args.launch)
        if named is None:
            p.error(f"--profile is needed: --launch {args.launch} is not sim_<profile>.launch.py")
        args.profile = named.group(1)
    # The stop window inside the script's rest; the walking stop while it walks.
    script = wm.COMMAND_SCRIPTS[args.script]
    walking_s = wm.script_length(script[:-1])
    if not 0.0 < args.stop_window <= script[-1].length_s:
        p.error(f"--stop-window must be in (0, {script[-1].length_s}] s")
    if not 0.0 < args.estop_at < walking_s:
        p.error(f"--estop-at must be in (0, {walking_s}) s — inside the walking part")
    if args.mpc_threads is not None:
        check_mpc_threads(p, args.mpc_threads, args.launch)
    if args.push is not None:
        t_push, dur = args.push[0], args.push[1]
        if not args.push_body:
            p.error("--push needs --push-body")
        if not (t_push >= 0.0 and dur > 0.0 and t_push + dur <= walking_s):
            p.error(f"--push: T and T + DUR must be inside the walking part [0, {walking_s}] s")
    try:
        args.overlay = arm_overlays(
            args.arms, args.overlay, args.friction, args.out / "overlays", args.profile
        )
        args.overlay = source_overlays(
            args.arms,
            args.mpc_layout,
            args.overlay,
            args.source,
            args.out / "overlays",
            args.profile,
        )
    except ValueError as e:
        p.error(str(e))
    notes = []
    for k in range(args.k):
        for arm in args.arms:
            note = run_once(arm, k, args)
            notes.append(note)
            print(json.dumps(note), flush=True)
    (args.out / "runs.json").write_text(json.dumps(notes, indent=2) + "\n")
    return 0 if all(n.get("recorder_rc") == 0 and n.get("events_ok", True) for n in notes) else 1


if __name__ == "__main__":
    sys.exit(main())
