#!/usr/bin/env python3
"""Catching sim evaluation (first used for E1-F06, G-1, #632): write one arm's sim overlay.

    mk_overlay.py <out.yaml> <p1b|leap> <mpc|closed_form>

The overlay IS the repo's ``sim_overlays/catch_lead_on.yaml`` of the robot
(read from the source tree, every leaf kept), plus the stop law of the arm:
``planner.segment.mode: <mpc|closed_form>``. The overlay states the mode for
both arms — the shipped default is not what tells them apart.
Nothing else — in particular no ``gamma_ref``
(the shipped value is what G-1 compares; the plan line checks it in the
mirror with EXPECT_KV). After writing, the leaves are re-read and compared
with the repo file's; any difference is refused.
"""

import sys
from pathlib import Path

import yaml

from rtc_tools.utils.catching_keys import reject_renamed_keys

# integrated_bringup/config of the source tree this file is in.
REPO = Path(__file__).resolve().parents[2] / "config"
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
    out, short, mode = argv
    if short not in ROBOTS or mode not in ("mpc", "closed_form"):
        raise SystemExit(__doc__)
    src = REPO / ROBOTS[short] / "sim_overlays" / "catch_lead_on.yaml"
    base = yaml.safe_load(src.read_text())
    doc = yaml.safe_load(src.read_text())
    c = doc
    for k in CATCHING:
        c = c[k]
    # An overlay that still writes a pre-#711 key would be copied through and the
    # mode key below would sit beside it, not replace it.
    reject_renamed_keys(c, source=str(src))
    c.setdefault("planner", {}).setdefault("segment", {})["mode"] = mode
    with open(out, "w") as f:
        f.write(
            f"# catching eval overlay: {ROBOTS[short]} {mode} = {src.name} + planner.segment.mode\n"
        )
        yaml.safe_dump(doc, f, sort_keys=False)
    got = dict(leaves(yaml.safe_load(Path(out).read_text())))
    want = dict(leaves(base))
    extra = {k: v for k, v in got.items() if k not in want}
    expect_extra = {(*CATCHING, "planner", "segment", "mode"): mode}
    if any(got.get(k) != v for k, v in want.items()) or extra != expect_extra:
        Path(out).unlink()
        raise SystemExit(f"refused: leaves differ from {src} + the mode key: extra {extra}")
    if any("gamma_ref" in k for k in got):
        Path(out).unlink()
        raise SystemExit("refused: gamma_ref in the overlay")
    print(f"{out}: {len(want)} catch_lead_on leaves + {len(extra)} mode key")


if __name__ == "__main__":
    main(sys.argv[1:])
