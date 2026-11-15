#!/usr/bin/env python3
"""Hanging step response of a PD servo gain set — the first check of a gain choice.

The robot hangs (its floating base welded in place, contacts off, like a crane), the
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


def pinned_model(mjcf: Path, base_body: str) -> mujoco.MjModel:
    """The scene with its floating base welded to the world and contacts off.

    Resetting the free joint after each step is not a pin: inside the step the
    base falls, and the legs fall with it — weightless, so a gain that sags
    under gravity would pass. The weld holds the base at its reference pose
    (qpos0) during the step.
    """
    spec = mujoco.MjSpec.from_file(str(mjcf))
    weld = spec.add_equality()
    weld.type = mujoco.mjtEq.mjEQ_WELD
    weld.objtype = mujoco.mjtObj.mjOBJ_BODY
    weld.name1 = base_body
    model = spec.compile()
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    return model


def run(args: argparse.Namespace) -> dict:
    model = pinned_model(args.mjcf, args.base_body)
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
        if model.actuator_trntype[a] != mujoco.mjtTrn.mjTRN_JOINT:
            raise SystemExit(f"actuator {a} does not drive a joint")
        model.actuator_gaintype[a] = mujoco.mjtGain.mjGAIN_FIXED
        model.actuator_biastype[a] = mujoco.mjtBias.mjBIAS_AFFINE
        model.actuator_gainprm[a, 0] = kp[a]  # force = kp·ctrl − kp·q − kd·q̇
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
        data.qpos[:7] = model.qpos0[:7]  # the weld's reference pose
        data.qvel[:] = 0.0
        data.ctrl[:] = [data.qpos[q] for q in qadr]  # every other joint holds where it is
        q0 = data.qpos[qadr[a]]
        lo, hi = model.jnt_range[joint_of[a]]
        sign = 1.0 if (hi - q0) >= (q0 - lo) else -1.0
        step = args.step
        if model.jnt_limited[joint_of[a]]:
            step = min(step, (hi - q0) if sign > 0 else (q0 - lo))
        target = q0 + sign * step
        lo_c, hi_c = model.actuator_ctrlrange[a]
        if model.actuator_ctrllimited[a]:
            target = min(max(target, lo_c), hi_c)
        step = abs(target - q0)
        data.ctrl[a] = target
        peak = 0.0
        diverged = False
        for _ in range(steps):
            mujoco.mj_step(model, data)
            x = sign * (data.qpos[qadr[a]] - q0)
            if not math.isfinite(x) or data.warning[mujoco.mjtWarning.mjWARN_BADQACC].number:
                diverged = True
                break
            peak = max(peak, x)
        covered = sign * (data.qpos[qadr[a]] - q0) / step if step > 0 and not diverged else 0.0
        overshoot = max(0.0, peak / step - 1.0) if step > 0 and not diverged else 0.0
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a)
        ok = not diverged and covered >= args.min_coverage and overshoot <= args.max_overshoot
        results.append(
            {
                "actuator": name,
                "kp": kp[a],
                "kd": kd[a],
                "step_rad": step,
                "coverage": covered,
                "overshoot": overshoot,
                "diverged": diverged,
                "pass": bool(ok),
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
    p.add_argument("--base-body", default="pelvis", help="the floating base body to weld")
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
