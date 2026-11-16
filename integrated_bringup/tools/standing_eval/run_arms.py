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

The two event arms start the recorder in the background for the whole script
(stopping it once the script is done) and mark every event in run.json with
its wall offset from the recorder's start and its wall-clock stamp — the
recorder CSV's time axis, since the sim stamps its sensor messages with the
wall clock; the windows are judged from the CSV between their marks
(``windows`` in run.json, the stance_metrics numbers), beside the recorder's
own JSON over the whole run. A step whose reply is not what the script expects
ends the run with ``error`` in run.json and a non-zero exit. They run with the plain launch (no MPC) or, under
``--mpc-layout``, with sim_overlay:=mpc_off like loco.

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

Each run's recorder CSV / JSON and the controller log session land in
``<out>/<arm>_<k>/``. Run it under ``repo_scripts/scripts/with_verify_hold.sh``
so no build starts between runs, with the workspace environment sourced.
"""

from __future__ import annotations

import argparse
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

from rtc_tools.analysis import stance_metrics as sm
from rtc_tools.launch.cpu_shield import mpc_solver_threads

LOCO = "locomotion_controller"
ARMS = ("servo", "loco", "lowpd", "mpc", "mpc_fb", "reentry", "estop")
# arms whose switch step activates the locomotion controller
LOCO_ARMS = ("loco", "reentry", "estop")
EVENT_ARMS = ("reentry", "estop")  # a script (SCRIPTS) under a background recording
WORKER_PROFILE = "mpc_par_sim"  # the simulation worker profile (--mpc-threads)
OPERATOR_ESTOP_REASON = "operator_request"  # the CM's reason for /rtc_cm/trigger_estop
# The event arms' recorder is stopped (SIGINT) once the script is done; its
# own --duration is only a bound, this much past the script's sleeps: every
# call is a `ros2` CLI call (0.5-1.5 s of start-up and discovery each) the
# sleeps do not count.
RECORDER_MARGIN_S = 30.0
RECORDER_GRACE_S = 1.0  # recording past the last mark before the stop


def launch_args(
    arm: str, mpc_layout: bool, overlays: dict[str, str], mpc_threads: int | None = None
) -> list[str]:
    """The arm's launch arguments (module docstring); `overlays` replaces an arm's overlay."""
    under_mpc = arm in ("mpc", "mpc_fb") or mpc_layout
    default = {"mpc_fb": "mpc_riccati"}.get(arm)
    if arm in LOCO_ARMS and mpc_layout:
        default = "mpc_off"
    overlay = overlays.get(arm, default)
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


# The event scripts (module docstring), one step per entry: ("sleep", the
# argument holding its length), ("window", its name — `--duration` long,
# marked at both ends), ("call", its mark name, the action). An action returns
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
}


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
            clock.sleep(getattr(args, step[1]) * wall)
        elif step[0] == "window":
            mark(f"{step[1]}_start")
            clock.sleep(args.duration * wall)
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
    return sum(
        getattr(args, step[1]) if step[0] == "sleep" else args.duration
        for step in SCRIPTS[arm]
        if step[0] in ("sleep", "window")
    )


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


def run_once(arm: str, k: int, args: argparse.Namespace) -> dict:
    out = args.out / f"{arm}_{k}"
    out.mkdir(parents=True, exist_ok=True)
    extra = launch_args(arm, args.mpc_layout, args.overlay, args.mpc_threads) + args.launch_arg
    wall = 1.0
    if args.rtf:
        extra.append(f"max_rtf:={args.rtf}")
        wall = 1.0 / args.rtf
    note: dict = {"arm": arm, "k": k, "launch_args": extra}
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
                    t0 = time.monotonic()
                    note["events_ok"] = run_events(arm, args, note, WallClock(), t0)
                    time.sleep(RECORDER_GRACE_S * wall)
                    if recorder.poll() is None:
                        os.killpg(recorder.pid, signal.SIGINT)
                    try:
                        note["recorder_rc"] = recorder.wait(timeout=60)
                    except subprocess.TimeoutExpired:
                        os.killpg(recorder.pid, signal.SIGKILL)
                        note["recorder_rc"] = recorder.wait()
                note["windows"] = window_metrics(out / "stance.csv", note["events"], args.z_ref)
            else:
                rec = sh(recorder_cmd, timeout=args.duration * 4 * wall + 60)
                note["recorder_rc"] = rec.returncode
                (out / "recorder.log").write_text(rec.stdout + rec.stderr)
            note["active_controller_end"] = active_controller()
        finally:
            os.killpg(launch.pid, signal.SIGINT)
            try:
                launch.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait()
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
        help="the arm's sim_overlay (a shipped name or a path), replacing its own",
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
    p.add_argument("--sensor-topic", default="/g1/sim_sensors")
    p.add_argument("--z-ref", type=float, default=0.764431)
    p.add_argument("--settle", type=float, default=2.0)
    p.add_argument("--lead", type=float, default=1.0)
    p.add_argument("--duration", type=float, default=10.0)
    p.add_argument(
        "--rtf", type=float, default=None, help="max_rtf for the sim (module docstring)"
    )
    p.add_argument("--logging-root", type=Path, required=True)
    args = p.parse_args(argv)
    if args.mpc_threads is not None:
        check_mpc_threads(p, args.mpc_threads, args.launch)
    args.overlay = dict(o.split("=", 1) for o in args.overlay)
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
