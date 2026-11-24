#!/usr/bin/env python3
"""Promote demo_controller_gui's hand postures to the shipped copy.

    python3 integrated_bringup/tools/demo_postures/promote.py --robot p2 --check
    python3 integrated_bringup/tools/demo_postures/promote.py --robot p2 --robot p2_left

Writes config/<robot>/demo_gui/ of the checkout this file is in (not the
installed share), from the GUI's working copy in the logging root. Needs the
workspace sourced. The logic is integrated_bringup.demo_gui.promote; the
operator guide is docs/demo-controller-gui.md §Shipped postures.
"""

import sys
from pathlib import Path

from integrated_bringup.demo_gui.promote import main

if __name__ == "__main__":
    sys.exit(main(sys.argv[1:], str(Path(__file__).resolve().parents[2] / "config")))
