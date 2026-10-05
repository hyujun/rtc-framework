#!/usr/bin/env python3
"""E0-F04 (#647) condition set: profiles, overlays, the condition table and the unit plan.

Writes into $DATA/conditions (DATA has no default — the data directory of the
evaluation, outside the repository):
  <COND>/profile.json         ball_perception's shipped catching profile, `prediction` changed
  <COND>/overlay_<robot>.yaml the shipped catch_lead_on leaves + the three grid keys
  conditions.tsv              cond, robot, profile, overlay, dt, n_min, points
  plan.txt                    the unit order, unless it exists

The shipped profile is read from $BALL_SIM_WS (the estimator's colcon
workspace), the shipped controller config and overlays from the source tree
this file is in. The plan lines name CONDITIONS (`<dir> <robot> <cond> <n>
<seed>`): they are not run_all.sh's plan format, which names an overlay path.
The leaf rule is test_catch_lead_overlays' `_unread_leaves`, loaded from
integrated_bringup/test by path — renaming that test breaks this script.

L-50 IS the shipped configuration: the installed profile itself and the shipped
overlay by name, so L-50 is E0-F02's configuration with the profile moved.
Refuses to write anything that fails: horizon an exact ns multiple of the step,
n_min = ceil(horizon_min / dt) + 1 <= points <= kCap, every overlay leaf a
shipped controller key of the same type (test_catch_lead_overlays' rule).
"""

import copy
import importlib.util
import json
import math
import os
import random
import sys
from pathlib import Path

import yaml

from rtc_tools.utils.catching_keys import reject_renamed_keys
from rtc_tools.utils.controller_config import load_controller_config

REPO = str(Path(__file__).resolve().parents[3])
BWS = os.environ.get("BALL_SIM_WS", "")
DATA = os.environ.get("DATA", "")
SHIPPED_PROFILE = (
    f"{BWS}/install/ball_perception_sim/share/ball_perception_sim/config/sim_profile.catching.json"
)
OUT = f"{DATA}/conditions"

CONTROLLER = "demo_catching_controller"
ROBOTS = {"p1b": "ur5e_p1b", "leap": "iiwa7_leap"}
K_CAP = 40
HORIZON_MIN_S = 0.51
# name: (horizon_s, points) — formulation §1.7
CONDITIONS = {
    "L-50": (1.0, 20),
    "L-40": (1.0, 25),
    "L-31": (1.0, 32),
    "L-25": (1.0, 40),
    "M-50": (0.75, 15),
    "M-31": (0.75, 24),
    "M-25": (0.75, 30),
    "M-19": (0.75, 40),
}
SEEDS = {"p1b": (601, 602), "leap": (701, 702)}
N_THROWS = 50
ORDER_SEED = 647


def grid(horizon_s, points):
    h_ns = round(horizon_s * 1e9)
    if h_ns % points:
        sys.exit(f"horizon {horizon_s} s is not a multiple of {points} points in ns")
    step_ns = h_ns // points
    step_s = step_ns / 1e9
    if round(step_s * 1e9) != step_ns:
        sys.exit(f"step {step_s} does not round-trip to {step_ns} ns")
    n_min = math.ceil(HORIZON_MIN_S / step_s) + 1
    if not n_min <= points <= K_CAP:
        sys.exit(f"n_min {n_min} <= points {points} <= kCap {K_CAP} fails")
    return step_s, n_min


def leaf_checker():
    path = f"{REPO}/integrated_bringup/test/test_catch_lead_overlays.py"
    spec = importlib.util.spec_from_file_location("lead_overlays", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod._unread_leaves


def main():
    if not DATA or not BWS:
        sys.exit(
            "DATA (the evaluation's data directory) and BALL_SIM_WS (the estimator's workspace) must be set"
        )
    unread = leaf_checker()
    with open(SHIPPED_PROFILE) as f:
        shipped_profile = json.load(f)
    rows = []
    for cond, (horizon_s, points) in CONDITIONS.items():
        step_s, n_min = grid(horizon_s, points)
        os.makedirs(f"{OUT}/{cond}", exist_ok=True)
        if cond == "L-50":
            p = shipped_profile["prediction"]
            assert (p["horizon_s"], p["step_s"], p["max_points"]) == (
                horizon_s,
                step_s,
                points,
            ), p
            profile_path = SHIPPED_PROFILE
        else:
            prof = copy.deepcopy(shipped_profile)
            prof["prediction"] = {
                "horizon_s": horizon_s,
                "step_s": step_s,
                "max_points": points,
            }
            profile_path = f"{OUT}/{cond}/profile.json"
            with open(profile_path, "w") as f:
                json.dump(prof, f, indent=1)
                f.write("\n")
        for short, robot in ROBOTS.items():
            cfg = f"{REPO}/integrated_bringup/config/{robot}"
            # The shipped config is four files (MD-90): read it composed, as CM does.
            shipped_ctrl = load_controller_config(
                f"{cfg}/controllers/demo_catching_controller.yaml", config_key=CONTROLLER
            )[CONTROLLER]
            ctrl = shipped_ctrl["catching"]
            reject_renamed_keys(ctrl, source=f"{cfg}/controllers/demo_catching_controller.yaml")
            ship = (
                ctrl["prediction"]["dt_expected"],
                ctrl["io"]["n_min"],
                ctrl["planner"]["search"]["grid"]["slice"]["dt"],
            )
            assert ship == (0.05, 12, 0.05), (robot, ship)
            if cond == "L-50":
                overlay = "catch_lead_on"
            else:
                with open(f"{cfg}/sim_overlays/catch_lead_on.yaml") as f:
                    ov = yaml.safe_load(f)
                c = ov["integrated_rt_controller"]["ros__parameters"]["demo_catching_controller"][
                    "catching"
                ]
                reject_renamed_keys(c, source=f"{cfg}/sim_overlays/catch_lead_on.yaml")
                c.setdefault("prediction", {})["dt_expected"] = step_s
                c.setdefault("io", {})["n_min"] = n_min
                c.setdefault("planner", {}).setdefault("search", {}).setdefault(
                    "grid", {}
                ).setdefault("slice", {})["dt"] = step_s
                problems = unread(
                    ov["integrated_rt_controller"]["ros__parameters"]["demo_catching_controller"],
                    shipped_ctrl,
                )
                if problems:
                    sys.exit(f"{cond} {robot}: {problems}")
                overlay = f"{OUT}/{cond}/overlay_{short}.yaml"
                with open(overlay, "w") as f:
                    f.write(
                        f"# E0-F04 {cond} ({robot}): shipped catch_lead_on + the prediction grid.\n"
                    )
                    yaml.safe_dump(ov, f, sort_keys=False)
            rows.append(
                (
                    cond,
                    short,
                    profile_path,
                    overlay,
                    repr(step_s),
                    str(n_min),
                    str(points),
                )
            )
    with open(f"{OUT}/conditions.tsv", "w") as f:
        f.write("cond\trobot\tprofile\toverlay\tdt\tn_min\tpoints\n")
        for r in rows:
            f.write("\t".join(r) + "\n")

    plan = f"{OUT}/plan.txt"
    if os.path.exists(plan):
        print(f"{plan} exists — not rewritten")
    else:
        rng = random.Random(ORDER_SEED)
        lines = [
            "# <dir> <robot> <cond> <n> <seed> — seed blocks; conditions in a fixed random order per block"
        ]
        for block in range(2):
            order = list(CONDITIONS)
            rng.shuffle(order)
            lines.append(f"# block {block + 1}: {' '.join(order)}")
            for cond in order:
                for short in ROBOTS:
                    seed = SEEDS[short][block]
                    lines.append(f"{cond}_{short}_{seed} {short} {cond} {N_THROWS} {seed}")
        with open(plan, "w") as f:
            f.write("\n".join(lines) + "\n")
    for r in rows:
        print("\t".join(r[:2] + r[4:]))


if __name__ == "__main__":
    main()
