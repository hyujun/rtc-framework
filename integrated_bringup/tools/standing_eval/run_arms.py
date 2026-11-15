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

With ``--mpc-layout`` every arm runs under enable_mpc:=true (the same CPU
layout), and loco turns the MPC off again with sim_overlay:=mpc_off — the
control arm of an MPC evaluation. ``--mpc-threads N`` solves on N threads:
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
from pathlib import Path

import yaml

from rtc_tools.launch.cpu_shield import mpc_solver_threads

LOCO = "locomotion_controller"
ARMS = ("servo", "loco", "lowpd", "mpc", "mpc_fb")
WORKER_PROFILE = "mpc_par_sim"  # the simulation worker profile (--mpc-threads)


def launch_args(
    arm: str, mpc_layout: bool, overlays: dict[str, str], mpc_threads: int | None = None
) -> list[str]:
    """The arm's launch arguments (module docstring); `overlays` replaces an arm's overlay."""
    under_mpc = arm in ("mpc", "mpc_fb") or mpc_layout
    default = {"mpc_fb": "mpc_riccati", "loco": "mpc_off" if mpc_layout else None}.get(arm)
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
    req = f"{{activate_controllers: ['{LOCO}'], deactivate_controllers: [], strictness: 1}}"
    r = sh(
        [
            "ros2",
            "service",
            "call",
            "/rtc_cm/switch_controller",
            "rtc_msgs/srv/SwitchController",
            req,
        ]
    )
    note["switch"] = r.stdout.strip()[-300:]
    if "ok=True" not in r.stdout:
        return False
    time.sleep(0.3)
    note["active_controller"] = active_controller()
    return note["active_controller"] == LOCO


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
            rec = sh(
                [
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
                    str(args.duration * wall),
                    "--settle",
                    "0",
                    "--z-ref",
                    str(args.z_ref),
                    "--start-now",
                    "--csv",
                    str(out / "stance.csv"),
                    "--json",
                    str(out / "stance.json"),
                ],
                timeout=args.duration * 4 * wall + 60,
            )
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
    return 0 if all(n.get("recorder_rc") == 0 for n in notes) else 1


if __name__ == "__main__":
    sys.exit(main())
