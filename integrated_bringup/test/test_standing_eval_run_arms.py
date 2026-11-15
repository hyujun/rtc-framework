"""tools/standing_eval/run_arms.py (not installed — read from the source tree).

The launch arguments each arm gets: without ``--mpc-threads`` they are the ones
the tool always passed; with it, every arm launched under the MPC layout also
names the simulation worker profile and the solver's thread count.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1] / "tools" / "standing_eval"
sys.path.insert(0, str(TOOLS))
import run_arms  # noqa: E402

POOL = ["layout_profile:=mpc_par_sim", "mpc_threads:=2"]


@pytest.mark.parametrize(
    ("arm", "mpc_layout", "expected"),
    [
        ("servo", False, []),
        ("lowpd", False, []),
        ("loco", False, []),
        ("mpc", False, ["enable_mpc:=true"]),
        ("mpc_fb", False, ["enable_mpc:=true", "sim_overlay:=mpc_riccati"]),
        ("servo", True, ["enable_mpc:=true"]),
        ("loco", True, ["enable_mpc:=true", "sim_overlay:=mpc_off"]),
        ("mpc", True, ["enable_mpc:=true"]),
    ],
)
def test_without_threads_the_arguments_are_unchanged(arm, mpc_layout, expected):
    assert run_arms.launch_args(arm, mpc_layout, {}) == expected
    assert run_arms.launch_args(arm, mpc_layout, {}, None) == expected


@pytest.mark.parametrize(
    ("arm", "mpc_layout", "expected"),
    [
        ("servo", False, []),  # no MPC layout: nothing to seat
        ("loco", False, []),
        ("mpc", False, ["enable_mpc:=true", *POOL]),
        ("mpc_fb", False, ["enable_mpc:=true", *POOL, "sim_overlay:=mpc_riccati"]),
        # one CPU layout for every arm of an --mpc-layout evaluation
        ("servo", True, ["enable_mpc:=true", *POOL]),
        ("loco", True, ["enable_mpc:=true", *POOL, "sim_overlay:=mpc_off"]),
    ],
)
def test_threads_add_the_worker_profile_under_the_mpc_layout(arm, mpc_layout, expected):
    assert run_arms.launch_args(arm, mpc_layout, {}, 2) == expected


def test_an_arm_overlay_still_replaces_its_own():
    assert run_arms.launch_args("mpc_fb", False, {"mpc_fb": "/x.yaml"}, 3) == [
        "enable_mpc:=true",
        "layout_profile:=mpc_par_sim",
        "mpc_threads:=3",
        "sim_overlay:=/x.yaml",
    ]


def test_threads_below_one_are_refused(capsys):
    with pytest.raises(SystemExit):
        run_arms.main(
            [
                "--out",
                "/nonexistent",
                "--runtime-yaml",
                "x",
                "--logging-root",
                "y",
                "--mpc-threads",
                "0",
                "-k",
                "0",
            ]
        )
    assert "--mpc-threads must be at least 1" in capsys.readouterr().err
