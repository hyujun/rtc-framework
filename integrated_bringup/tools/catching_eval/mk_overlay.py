#!/usr/bin/env python3
"""Catching sim evaluation (first used for E1-F06, G-1, #632): write one arm's sim overlay.

    mk_overlay.py <out.yaml> <p1b|leap> <closed_form|mpc|mpc_docking> [grid|nlp [tennis|beanbag|hard]]

The overlay states the arm and nothing else: the stop law
(``planner.segment.mode: <closed_form|mpc|mpc_docking>``), for both arms — the
shipped default is not what tells them apart. Every other key of the run is the
robot's shipped one, the sim arm's lead included (it is in the shipped
controller YAML since #821; this tool used to copy it from the
``catch_lead_on`` overlay, which is gone).
The optional fourth argument is the catch-point search: given, the overlay also
states ``planner.search.mode: <grid|nlp>``; omitted, it states no search key (the
shipped default, grid, applies). ``nlp`` with ``closed_form`` is refused: the
NLP search hands its result to a segment planner and the closed-form law has none.
The optional fifth argument is the sim's ball (#746): given, the overlay also states
``mujoco_simulator.ros__parameters.projectile_ball.ball_type`` — the simulator's key, not
the controller's; omitted, it states no ball (the simulator's default, tennis, applies).
The ball changes the sim alone: the controller, the estimator's profile and the hand's
identified sets stay what the robot ships.
Nothing else — in particular no ``gamma_ref``
(the shipped value is what G-1 compares; the plan line checks it in the
mirror with EXPECT_KV). After writing, the leaves are re-read: anything but
the arm's keys is refused. The robot argument names the unit's robot in the
file's first line; the keys are the same for both.

``catch_population_trials`` (#821) needs no overlay: the sim launches take the
two modes as arguments. This tool stays for ``run_unit.sh``.
"""

import sys
from pathlib import Path

import yaml

SEGMENT_MODES = ("closed_form", "mpc", "mpc_docking")
SEARCH_MODES = ("grid", "nlp")
# rtc_mujoco_sim's projectile_ball.ball_type values (ParseProjectileBallType).
BALLS = ("tennis", "beanbag", "hard")
BALL_KEY = ("mujoco_simulator", "ros__parameters", "projectile_ball", "ball_type")
ROBOTS = {"p1b": "ur5e_p1b", "leap": "iiwa7_leap"}
CATCHING = (
    "integrated_rt_controller",
    "ros__parameters",
    "demo_catching_controller",
    "catching",
)


def leaves(node, prefix=()):
    if isinstance(node, dict):
        for k, v in node.items():
            yield from leaves(v, (*prefix, k))
    else:
        yield prefix, node


def main(argv):
    if len(argv) not in (3, 4, 5):
        raise SystemExit(__doc__)
    out, short, mode = argv[:3]
    search = argv[3] if len(argv) >= 4 else None
    ball = argv[4] if len(argv) == 5 else None
    if short not in ROBOTS or mode not in SEGMENT_MODES:
        raise SystemExit(__doc__)
    if search is not None and search not in SEARCH_MODES:
        raise SystemExit(__doc__)
    if ball is not None and ball not in BALLS:
        raise SystemExit(__doc__)
    if search == "nlp" and mode == "closed_form":
        raise SystemExit(
            "refused: planner.search.mode: nlp with planner.segment.mode: closed_form "
            "is not an allowed pair"
        )
    expected = {(*CATCHING, "planner", "segment", "mode"): mode}
    if search is not None:
        expected[(*CATCHING, "planner", "search", "mode")] = search
    if ball is not None:
        expected[BALL_KEY] = ball
    doc: dict = {}
    for path, value in expected.items():
        node = doc
        for k in path[:-1]:
            node = node.setdefault(k, {})
        node[path[-1]] = value
    keys = "planner.segment.mode" + (" + planner.search.mode" if search else "")
    keys += " + projectile_ball.ball_type" if ball else ""
    label = " ".join(x for x in (mode, search, ball) if x)
    with open(out, "w") as f:
        f.write(f"# catching eval overlay: {ROBOTS[short]} {label} = shipped + {keys}\n")
        yaml.safe_dump(doc, f, sort_keys=False)
    got = dict(leaves(yaml.safe_load(Path(out).read_text())))
    if got != expected:
        Path(out).unlink()
        raise SystemExit(f"refused: the overlay's leaves are not the arm key(s): {got}")
    print(f"{out}: {len(got)} arm key(s)")


if __name__ == "__main__":
    main(sys.argv[1:])
