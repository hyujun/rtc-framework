"""Resolve the ``sim_lanes:=`` launch argument to simulator parameter overrides.

Shared by the per-profile sim launch files that run catching trials. The two
lanes are the measurement substrate of every S8 trial run — the clock-phase
lane is the D-3 covariate and the ball-contact lane the sim truth that
``catching_trials`` reads by default from ``<session>/sim/``. A profile that
lacks the argument cannot produce that truth, so the wiring lives here once
rather than as a copy per launch file that can drift.
"""

from __future__ import annotations

import os

LANES = ("clock_lane", "ball_contact_lane")


def sim_lanes_argument_description() -> str:
    return (
        "Write the sim clock-phase lane and the ball contact truth lane to "
        "<session_dir>/sim/{clock_lane,ball_contact_lane}.csv (S8 trial runs; "
        "catching_trials joins them with the controller CSVs)."
    )


def sim_lane_overrides(value: str, session_dir: str) -> dict:
    """``sim_lanes:=`` → the simulator overrides that turn both lanes on, or {}.

    The lanes are node parameters read at start-up, off by default, and refuse
    an empty csv_path. Pointing them into the SESSION tree keeps every trial
    run's lanes next to the controller CSVs they are joined with — a path in an
    overlay file would make every run write the same file. The caller places
    the result in the CLI dict, so it wins over an overlay's lane block.
    A value that is neither true nor false RAISES rather than running unlaned.
    """
    value = value.strip().lower()
    if value in ("true", "1", "yes"):
        lane_dir = os.path.join(session_dir, "sim")
        overrides: dict = {}
        for lane in LANES:
            overrides[f"{lane}.enabled"] = True
            overrides[f"{lane}.csv_path"] = os.path.join(lane_dir, f"{lane}.csv")
        return overrides
    if value in ("", "false", "0", "no"):
        return {}
    raise RuntimeError(f"sim_lanes must be true or false, got {value!r}")
