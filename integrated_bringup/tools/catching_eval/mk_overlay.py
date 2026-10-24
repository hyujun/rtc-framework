#!/usr/bin/env python3
"""Catching sim evaluation (#632): write one arm's sim overlay.

    mk_overlay.py <out.yaml> <p1b|leap> <closed_form|mpc|mpc_docking> [grid|nlp [tennis|beanbag|hard]]
        [--set <key>=<value>]...

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

``--set <key>=<value>`` (#822, repeatable) also states one controller value, for
a unit that measures what a value does. The key is a mirror name that IS the
value's path under ``catching`` (``planner.segment.mpc.approach.n_pre_max``) — a
key the robot's shipped controller tree (its ``include:`` fragments composed)
does not have is refused, and so is an alias mirror (``reference.omega``), which
names no path. The value is read as YAML would type it (``true`` · ``8`` · ``0.15``
· ``1e-3`` as a number, not the string PyYAML 1.1 reads) and must be of the
shipped value's type: ``6`` for a float key is refused (a ROS double parameter
refuses an integer). Refused too: the two mode keys (the positional arguments
state them), the docking planner's keys (its grid is bound to the search's —
``search_nlp.yaml`` — and is not checked here) and an approach grid over the
payload's 24 nodes. The overlay's second line and the last line printed are
``EXPECT_KV=…`` in the form ``ros2 param get`` prints the values, which
``run_unit.sh`` reads back (and takes from the header when EXPECT_KV is empty).

``catch_population_trials`` (#821) needs no overlay: the sim launches take the
two modes as arguments. This tool stays for ``run_unit.sh``.
"""

import math
import re
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


CONTROLLER = "demo_catching_controller"
REPO = Path(__file__).resolve().parents[3]
MODE_KEYS = ("planner.segment.mode", "planner.search.mode")
DOCKING_PREFIX = "planner.segment.mpc_docking."
# The trajectory payload's node cap (rtc_controllers trajectory.hpp kMaxSegmentNodes).
MAX_SEGMENT_NODES = 24


def parse_value(text):
    """A ``--set`` value as YAML 1.2 types it: bool, int, float (``1e-3`` too), else str."""
    if text.lower() in ("true", "false"):
        return text.lower() == "true"
    if re.fullmatch(r"[+-]?\d+", text):
        return int(text)
    try:
        value = float(text)
    except ValueError:
        return text
    if not math.isfinite(value):
        raise SystemExit(f"refused: --set value {text!r} is not a finite number")
    return value


def mirror_text(value):
    """``value`` as ``ros2 param get`` prints it after ``… value is: ``."""
    if isinstance(value, bool):
        return "True" if value else "False"
    return repr(value) if isinstance(value, float) else str(value)


def parse_sets(argv):
    """``(positional arguments, [(key, text), ...])`` of a command line with ``--set``."""
    rest, sets = [], []
    args = iter(argv)
    for arg in args:
        if arg == "--set" or arg.startswith("--set="):
            item = arg[len("--set=") :] if arg.startswith("--set=") else next(args, "")
            key, sep, text = item.partition("=")
            if not sep or not key or not text:
                raise SystemExit(f"refused: --set takes <key>=<value>, got {item!r}")
            sets.append((key, text))
        else:
            rest.append(arg)
    return rest, sets


def checked_sets(robot, sets):
    """``{key: value}`` of the ``--set`` items, each a shipped catching value of the
    same type (module docstring), or SystemExit naming every item refused."""
    from rtc_tools.utils.controller_config import load_controller_config, unread_leaves

    shipped = load_controller_config(
        REPO / "integrated_bringup" / "config" / robot / "controllers" / f"{CONTROLLER}.yaml",
        config_key=CONTROLLER,
    )[CONTROLLER]
    values, tree, why = {}, {}, []
    for key, text in sets:
        value = parse_value(text)
        if key in values:
            why.append(f"{key}: set twice")
        elif key in MODE_KEYS:
            why.append(f"{key}: a mode — the positional arguments state it")
        elif key.startswith(DOCKING_PREFIX):
            why.append(f"{key}: the docking planner's keys are not set here")
        elif (
            key.endswith(".approach.n_pre_max")
            and isinstance(value, int)
            and value > MAX_SEGMENT_NODES
        ):
            why.append(f"{key}: {value} nodes is over the payload's {MAX_SEGMENT_NODES}")
        values[key] = value
        node = tree
        parts = key.split(".")
        for part in parts[:-1]:
            node = node.setdefault(part, {})
            if not isinstance(node, dict):
                why.append(f"{key}: under another --set's value")
                break
        else:
            node[parts[-1]] = value
    why += unread_leaves({"catching": tree}, shipped)
    if why:
        raise SystemExit("refused --set: " + "; ".join(why))
    return values


def leaves(node, prefix=()):
    if isinstance(node, dict):
        for k, v in node.items():
            yield from leaves(v, (*prefix, k))
    else:
        yield prefix, node


def main(argv):
    argv, sets = parse_sets(argv)
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
    values = checked_sets(ROBOTS[short], sets) if sets else {}
    for key, value in values.items():
        expected[(*CATCHING, *key.split("."))] = value
    expect_kv = ";".join(f"{key}={mirror_text(value)}" for key, value in values.items())
    doc: dict = {}
    for path, value in expected.items():
        node = doc
        for k in path[:-1]:
            node = node.setdefault(k, {})
        node[path[-1]] = value
    keys = "planner.segment.mode" + (" + planner.search.mode" if search else "")
    keys += " + projectile_ball.ball_type" if ball else ""
    keys += "".join(f" + {key}" for key in values)
    label = " ".join(x for x in (mode, search, ball) if x)
    with open(out, "w") as f:
        f.write(f"# catching eval overlay: {ROBOTS[short]} {label} = shipped + {keys}\n")
        if expect_kv:
            f.write(f"# EXPECT_KV={expect_kv}\n")
        yaml.safe_dump(doc, f, sort_keys=False)
    got = dict(leaves(yaml.safe_load(Path(out).read_text())))
    # A value is the same when its type is too: 6 and 6.0 are two parameters.
    typed = {path: (type(value), value) for path, value in got.items()}
    if typed != {path: (type(value), value) for path, value in expected.items()}:
        Path(out).unlink()
        raise SystemExit(f"refused: the overlay's leaves are not the arm key(s): {got}")
    print(f"{out}: {len(got)} arm key(s)")
    if expect_kv:
        print(f"EXPECT_KV={expect_kv}")


if __name__ == "__main__":
    main(sys.argv[1:])
