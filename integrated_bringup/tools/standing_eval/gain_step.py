#!/usr/bin/env python3
"""Hanging step response of a PD servo gain set — the first check of a gain choice.

The robot hangs (its floating base pinned in the air, like a crane), the
servos run the candidate PD gains with no feedforward and no gravity
compensation, and each actuated joint in turn is commanded a step of
``--step`` rad from the keyframe pose while the others hold it. Per joint the
report says how much of the step was covered after ``--window`` seconds and
the overshoot past it. The pass rule (both are CLI flags so a change shows on
the command line): coverage >= ``--min-coverage`` and overshoot <=
``--max-overshoot``.

Gains come from a controller YAML's ``pd_feedforward`` section (``kp`` / ``kd``
in actuator order) or from ``--kp`` / ``--kd`` (one value for every joint).
The step's sign points away from the nearer joint limit.

Run with the venv's python (it has mujoco):
    gain_step.py --mjcf <scene.xml> --gains-yaml <runtime.yaml> --json out.json
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np
import yaml


def load_gains(path: Path, key: str) -> tuple[list[float], list[float]]:
    doc = yaml.safe_load(path.read_text())
    section = doc[key]["pd_feedforward"]
    return [float(x) for x in section["kp"]], [float(x) for x in section["kd"]]


def run(args: argparse.Namespace) -> dict:
    model = mujoco.MjModel.from_xml_path(str(args.mjcf))
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, args.keyframe)
    if key < 0:
        raise SystemExit(f"no keyframe '{args.keyframe}'")
    nu = model.nu
    if args.gains_yaml is not None:
        kp, kd = load_gains(args.gains_yaml, args.controller_key)
    else:
        kp, kd = [args.kp] * nu, [args.kd] * nu
    if len(kp) != nu or len(kd) != nu:
        raise SystemExit(f"{len(kp)} / {len(kd)} gains for {nu} actuators")
    for a in range(nu):
        model.actuator_gainprm[a, 0] = kp[a]
        model.actuator_biasprm[a, 0] = 0.0
        model.actuator_biasprm[a, 1] = -kp[a]
        model.actuator_biasprm[a, 2] = -kd[a]
    model.body_gravcomp[:] = 0.0

    joint_of = [int(model.actuator_trnid[a, 0]) for a in range(nu)]
    qadr = [int(model.jnt_qposadr[j]) for j in joint_of]
    steps = int(round(args.window / model.opt.timestep))
    results = []
    for a in range(nu):
        mujoco.mj_resetDataKeyframe(model, data, key)
        data.qpos[2] += args.hang_height
        base = data.qpos[:7].copy()
        q0 = data.qpos[qadr[a]]
        lo, hi = model.jnt_range[joint_of[a]]
        sign = 1.0 if (hi - q0) >= (q0 - lo) else -1.0
        target = q0 + sign * args.step
        data.ctrl[:] = model.key_ctrl[key]
        data.ctrl[a] = target
        peak = 0.0
        for _ in range(steps):
            mujoco.mj_step(model, data)
            data.qpos[:7] = base  # the crane: the base does not move
            data.qvel[:6] = 0.0
            peak = max(peak, sign * (data.qpos[qadr[a]] - q0))
        covered = sign * (data.qpos[qadr[a]] - q0) / args.step
        overshoot = max(0.0, peak / args.step - 1.0)
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a)
        ok = covered >= args.min_coverage and overshoot <= args.max_overshoot
        results.append(
            {
                "actuator": name,
                "kp": kp[a],
                "kd": kd[a],
                "coverage": covered,
                "overshoot": overshoot,
                "pass": bool(ok and math.isfinite(covered)),
            }
        )
    return {
        "mjcf": str(args.mjcf),
        "step_rad": args.step,
        "window_s": args.window,
        "min_coverage": args.min_coverage,
        "max_overshoot": args.max_overshoot,
        "all_pass": all(r["pass"] for r in results),
        "coverage_min": float(np.min([r["coverage"] for r in results])),
        "overshoot_max": float(np.max([r["overshoot"] for r in results])),
        "joints": results,
    }


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--mjcf", type=Path, required=True)
    p.add_argument("--keyframe", default="stand")
    p.add_argument("--gains-yaml", type=Path, default=None)
    p.add_argument("--controller-key", default="locomotion_controller")
    p.add_argument("--kp", type=float, default=None)
    p.add_argument("--kd", type=float, default=None)
    p.add_argument("--step", type=float, default=0.2, help="[rad]")
    p.add_argument("--window", type=float, default=2.0, help="[s]")
    p.add_argument("--hang-height", type=float, default=0.5, help="base raised by [m]")
    p.add_argument("--min-coverage", type=float, default=0.5)
    p.add_argument("--max-overshoot", type=float, default=0.2)
    p.add_argument("--json", type=Path, default=None)
    args = p.parse_args(argv)
    if args.gains_yaml is None and (args.kp is None or args.kd is None):
        p.error("give --gains-yaml or both --kp and --kd")
    report = run(args)
    text = json.dumps(report, indent=2)
    if args.json is not None:
        args.json.write_text(text + "\n")
    print(json.dumps({k: v for k, v in report.items() if k != "joints"}, indent=2))
    for r in report["joints"]:
        flag = "ok " if r["pass"] else "BAD"
        print(
            f"  {flag} {r['actuator']:<28} cover {r['coverage']:.2f} overshoot {r['overshoot']:.2f}"
        )
    return 0 if report["all_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
