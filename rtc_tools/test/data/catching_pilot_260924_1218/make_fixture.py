#!/usr/bin/env python3
"""Cut the catching_trials golden fixture out of the S8 pilot session.

The pilot (dynamic_catching S8, session ``260924_1218``: 15 reference + 10
perturbed throws at the ``ur5e_p1b`` sim, lead off, clock and contact lanes on,
non-RT dev PC) left ~700 MB of CSVs. ``test_catching_trials.py`` needs only a
few columns inside the 25 trial windows, so this script keeps exactly that:

* ``session/controllers/<ctl>/<diag>.csv.gz`` — the columns
  ``catching_trials.diag_columns`` reads, and only the rows the analysis reads:
  inside each trial window, every tick from APPROACH through HOLD (± a few
  ticks for the velocity derivative), every ``ref_valid`` tick (± 1, so a
  ``ref_saturated`` streak can neither split nor merge), every mode or hand
  phase transition, and RETREAT thinned to one tick per truth sample (10 ms —
  the truth-success check reads FK only at truth sample times);
* ``session/controllers/<ctl>/planner_events.csv.gz`` — whole (74 kB);
* ``session/sim/clock_lane.csv.gz`` — the active (in-flight) rows of each
  launch, which is all ``clock_phase.read_lane`` segments on, plus the
  ``launch_seq`` 0 header row;
* ``session/sim/ball_contact_lane.csv.gz`` — the episodes that touch a robot
  body (the ball's floor and table episodes are never read);
* ``trials/`` — ``trial_results.json`` with ``truth_csv`` rewritten to the bare
  file name, and every ``truth_trial_NN.csv`` gzipped;
* ``robot.urdf`` — the xacro-expanded URDF the pilot analysed;
* ``config/`` — only the keys ``catching_trials.load_profile`` reads, copied
  from the robot profile (test data may carry robot values; the module may not).

Left out: the hand device log (its ``effort_*`` only feeds ``hand_effort_max``,
which is not part of the golden) — the tool skips a missing device log.

Floats are written with 10 significant digits (1e-7 s on a ~100 s time axis,
1e-10 rad), truth stamps with 1 µs; the golden test checks the fixture
reproduces the full-session numbers within its tolerances.

Usage (from the repo root, env sourced)::

    python3 rtc_tools/test/data/catching_pilot_260924_1218/make_fixture.py \\
        ~/ros2_ws/rtc_ws/logging_data/260924_1218 \\
        ~/.claude/plans/dynamic-catching-S8-tools/pilot_260924_1218 \\
        integrated_bringup/config/ur5e_p1b

where the second argument holds ``trials/``, ``clock_lane_trim.csv``,
``ball_contact_lane.csv`` and ``p1b.urdf`` (the pilot wrote its lanes and
trials outside the session tree; S8-A's ``sim_lanes:=true`` puts the lanes under
``<session>/sim/``).
"""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

from rtc_tools.analysis.catching_trials import (
    MODE_APPROACH,
    MODE_HOLD,
    MODE_RETREAT,
    _csv_header,
    _ros_params,
    diag_columns,
    load_profile,
    load_trials,
    urdf_robot_links,
)

HERE = Path(__file__).resolve().parent
MARGIN_S = 0.1  # > the analysis window margin (0.05 s)
FLOAT_FORMAT = "%.10g"
DERIVATIVE_PAD = 5  # ticks kept around the moving span (np.gradient needs ±1)
RETREAT_STRIDE = 5  # one RETREAT tick per 10 ms truth sample at 2 ms ticks


def _windows(trials_dir: Path) -> list[tuple[float, float]]:
    trials, _ = load_trials(trials_dir)
    return [(t.t_launch - MARGIN_S, t.t_end + MARGIN_S) for t in trials if t.accepted]


def _in_windows(t: pd.Series, windows) -> pd.Series:
    keep = pd.Series(False, index=t.index)
    for lo, hi in windows:
        keep |= (t >= lo) & (t <= hi)
    return keep


def _dilate(mask: np.ndarray, pad: int) -> np.ndarray:
    out = mask.copy()
    for shift in range(1, pad + 1):
        out[shift:] |= mask[:-shift]
        out[:-shift] |= mask[shift:]
    return out


def _diag_rows(diag: pd.DataFrame, windows) -> np.ndarray:
    """The rows catching_trials reads (see the module docstring)."""
    mode = diag["mode"].to_numpy(int)
    phase = diag["hand_phase"].to_numpy(int)
    inside = _in_windows(diag["t_relative_s"], windows).to_numpy()
    moving = _dilate((mode >= MODE_APPROACH) & (mode <= MODE_HOLD), DERIVATIVE_PAD)
    valid = _dilate(diag["ref_valid"].to_numpy(int) != 0, 1)
    change = np.zeros(len(diag), dtype=bool)
    change[1:] = (np.diff(mode) != 0) | (np.diff(phase) != 0)
    change = _dilate(change, 1)
    retreat = (mode == MODE_RETREAT) & (np.arange(len(diag)) % RETREAT_STRIDE == 0)
    return inside & (moving | valid | change | retreat)


def _write_gz(df: pd.DataFrame, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(
        path,
        index=False,
        float_format=FLOAT_FORMAT,
        compression={"method": "gzip", "mtime": 0},  # reproducible bytes
    )


def _config(config_dir: Path, controller: str, out: Path) -> None:
    base = yaml.safe_load((config_dir / "_base.yaml").read_text())
    params = _ros_params(base)
    urdf = params["urdf"]
    (out / "controllers").mkdir(parents=True, exist_ok=True)
    header = f"# Cut by make_fixture.py from {config_dir.name}/"
    base_min = {
        "/**": {
            "ros__parameters": {
                "urdf": {
                    "package": urdf["package"],
                    "path": urdf["path"],
                    "extra_frames": {"catch_frame": urdf["extra_frames"]["catch_frame"]},
                }
            }
        }
    }
    (out / "_base.yaml").write_text(
        f"{header}_base.yaml\n" + yaml.safe_dump(base_min, sort_keys=False)
    )
    ctl_doc = yaml.safe_load((config_dir / "controllers" / f"{controller}.yaml").read_text())
    node = ctl_doc[controller]
    io = node["catching"]["io"]
    ctl_min = {
        controller: {
            "catching": {
                "core": {"ball": node["catching"]["core"]["ball"]},
                "io": {"arm_base_frame": io["arm_base_frame"], "base_T_world": io["base_T_world"]},
            },
            "topics": node["topics"],
            "logs": node["logs"],
        }
    }
    (out / "controllers" / f"{controller}.yaml").write_text(
        f"{header}controllers/{controller}.yaml\n" + yaml.safe_dump(ctl_min, sort_keys=False)
    )
    sim = yaml.safe_load((config_dir / "mujoco_simulator.yaml").read_text())
    (sim_node,) = [k for k, v in sim.items() if isinstance(v, dict) and "ros__parameters" in v]
    ball = sim[sim_node]["ros__parameters"]["projectile_ball"]
    sim_min = {
        sim_node: {
            "ros__parameters": {
                "projectile_ball": {"mass_kg": ball["mass_kg"], "radius_m": ball["radius_m"]}
            }
        }
    }
    (out / "mujoco_simulator.yaml").write_text(
        f"{header}mujoco_simulator.yaml\n" + yaml.safe_dump(sim_min, sort_keys=False)
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("session", type=Path)
    ap.add_argument("pilot_dir", type=Path, help="trials/, lanes and the expanded URDF")
    ap.add_argument("config_dir", type=Path)
    ap.add_argument("--out", type=Path, default=HERE)
    args = ap.parse_args()

    profile = load_profile(args.config_dir, session=args.session)
    for stale in ("session", "trials", "config"):
        shutil.rmtree(args.out / stale, ignore_errors=True)
    windows = _windows(args.pilot_dir / "trials")
    src = args.session / "controllers" / profile.controller
    dst = args.out / "session" / "controllers" / profile.controller

    diag_csv = src / f"{profile.diag_log}.csv"
    diag = pd.read_csv(diag_csv, usecols=diag_columns(_csv_header(diag_csv)), low_memory=False)
    _write_gz(diag[_diag_rows(diag, windows)], dst / f"{diag_csv.name}.gz")

    events = pd.read_csv(src / "planner_events.csv", low_memory=False)
    _write_gz(events, dst / "planner_events.csv.gz")

    lane = pd.read_csv(args.pilot_dir / "clock_lane_trim.csv")
    active = lane["ball_active"].astype(int) == 1
    first = lane.index == lane.index[0]
    _write_gz(lane[active | first], args.out / "session" / "sim" / "clock_lane.csv.gz")
    contacts = pd.read_csv(args.pilot_dir / "ball_contact_lane.csv")
    robot = set(urdf_robot_links((args.pilot_dir / "p1b.urdf").read_text()))
    contacts = contacts[contacts["first_body"].isin(robot)]
    _write_gz(contacts, args.out / "session" / "sim" / "ball_contact_lane.csv.gz")

    trials_out = args.out / "trials"
    trials_out.mkdir(parents=True, exist_ok=True)
    records = json.loads((args.pilot_dir / "trials" / "trial_results.json").read_text())
    for rec in records:
        if rec.get("truth_csv"):
            name = Path(rec["truth_csv"]).name
            rec["truth_csv"] = name
            truth = pd.read_csv(args.pilot_dir / "trials" / name)
            for col in ("wall_recv_s", "stamp_s"):  # epoch seconds: keep 1 µs
                truth[col] = truth[col].map("{:.6f}".format)
            _write_gz(truth, trials_out / f"{name}.gz")
    (trials_out / "trial_results.json").write_text(json.dumps(records, indent=1) + "\n")

    shutil.copyfile(args.pilot_dir / "p1b.urdf", args.out / "robot.urdf")
    _config(args.config_dir, profile.controller, args.out / "config")
    total = sum(p.stat().st_size for p in args.out.rglob("*") if p.is_file())
    print(f"fixture written to {args.out} ({total / 1e6:.2f} MB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
