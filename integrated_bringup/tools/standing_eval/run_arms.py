#!/usr/bin/env python3
"""Stand the floating-base sim profile under three arms and record each run.

One run = one fresh simulator on one time axis: launch (the profile's initial
joint controller holds the keyframe on its MJCF servo gains) → ``--settle`` s
→ the arm's switch step → ``--lead`` s → the stance recorder from its first
message for ``--duration`` s. The arms differ only in the switch step:

  servo   nothing (the MJCF servo holds the robot)            — the baseline
  lowpd   the joint controller's primary-group PD gains are set to the
          locomotion controller's (``pd_feedforward.kp/kd`` parameters), no
          feedforward                                          — PD alone
  loco    /rtc_cm/switch_controller to the locomotion controller; the call
          must answer ok and the active controller must be it  — the treatment

Each run's recorder CSV / JSON and the controller log session land in
``<out>/<arm>_<k>/``. Run it under ``repo_scripts/scripts/with_verify_hold.sh``
so no build starts between runs, with the workspace environment sourced.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml

LOCO = "locomotion_controller"


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
    note: dict = {"arm": arm, "k": k}
    t_start = time.time()
    with open(out / "launch.log", "w") as log:
        launch = subprocess.Popen(
            ["ros2", "launch", "integrated_bringup", args.launch, "enable_viewer:=false"],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            if not wait_topic(args.sensor_topic, 90.0):
                note["error"] = "no sensor topic"
                return note
            time.sleep(args.settle)
            note["switch_ok"] = switch_step(arm, args, note)
            if not note["switch_ok"]:
                note["error"] = "switch step failed"
                return note
            time.sleep(args.lead)
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
                    str(args.duration),
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
                timeout=args.duration * 4 + 60,
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
            (out / "run.json").write_text(json.dumps(note, indent=2) + "\n")
    return note


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--arms", nargs="+", default=["servo", "loco", "lowpd"])
    p.add_argument("-k", type=int, default=5, help="runs per arm")
    p.add_argument("--launch", default="sim_g1_loco.launch.py")
    p.add_argument(
        "--runtime-yaml",
        type=Path,
        required=True,
        help="the locomotion controller's runtime fragment (its PD gains)",
    )
    p.add_argument("--joint-node", default="/demo_joint_controller/demo_joint_controller")
    p.add_argument("--sensor-topic", default="/g1/sim_sensors")
    p.add_argument("--z-ref", type=float, default=0.764431)
    p.add_argument("--settle", type=float, default=2.0)
    p.add_argument("--lead", type=float, default=1.0)
    p.add_argument("--duration", type=float, default=10.0)
    p.add_argument("--logging-root", type=Path, required=True)
    args = p.parse_args(argv)
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
