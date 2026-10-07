#!/usr/bin/env python3
"""Catching sim evaluation (first used for E1-F06, G-1, #632): write one arm's sim overlay.

    mk_overlay.py <out.yaml> <p1b|leap> <closed_form|mpc|mpc_docking> [grid|nlp]

The overlay IS the repo's ``sim_overlays/catch_lead_on.yaml`` of the robot
(read from the source tree, every leaf kept), plus the stop law of the arm:
``planner.segment.mode: <closed_form|mpc|mpc_docking>``. The overlay states the
mode for both arms — the shipped default is not what tells them apart.
The optional fourth argument is the catch-point search: given, the overlay also
states ``planner.search.mode: <grid|nlp>``; omitted, it states no search key (the
shipped default, grid, applies). ``nlp`` with ``closed_form`` is refused: the
NLP search hands its result to a segment planner and the closed-form law has none.
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
SEGMENT_MODES = ("closed_form", "mpc", "mpc_docking")
SEARCH_MODES = ("grid", "nlp")
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
    if len(argv) not in (3, 4):
        raise SystemExit(__doc__)
    out, short, mode = argv[:3]
    search = argv[3] if len(argv) == 4 else None
    if short not in ROBOTS or mode not in SEGMENT_MODES:
        raise SystemExit(__doc__)
    if search is not None and search not in SEARCH_MODES:
        raise SystemExit(__doc__)
    if search == "nlp" and mode == "closed_form":
        raise SystemExit(
            "refused: planner.search.mode: nlp with planner.segment.mode: closed_form "
            "is not an allowed pair"
        )
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
    if search is not None:
        c["planner"].setdefault("search", {})["mode"] = search
    keys = "planner.segment.mode" + (" + planner.search.mode" if search else "")
    label = f"{mode} {search}" if search else mode
    with open(out, "w") as f:
        f.write(f"# catching eval overlay: {ROBOTS[short]} {label} = {src.name} + {keys}\n")
        yaml.safe_dump(doc, f, sort_keys=False)
    got = dict(leaves(yaml.safe_load(Path(out).read_text())))
    want = dict(leaves(base))
    extra = {k: v for k, v in got.items() if k not in want}
    expect_extra = {(*CATCHING, "planner", "segment", "mode"): mode}
    if search is not None:
        expect_extra[(*CATCHING, "planner", "search", "mode")] = search
    if any(got.get(k) != v for k, v in want.items()) or extra != expect_extra:
        Path(out).unlink()
        raise SystemExit(f"refused: leaves differ from {src} + the mode key(s): extra {extra}")
    if any("gamma_ref" in k for k in got):
        Path(out).unlink()
        raise SystemExit("refused: gamma_ref in the overlay")
    print(f"{out}: {len(want)} catch_lead_on leaves + {len(extra)} mode key(s)")


if __name__ == "__main__":
    main(sys.argv[1:])
