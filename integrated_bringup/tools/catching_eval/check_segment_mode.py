#!/usr/bin/env python3
"""Which segment mode did the controller log at configure? (#711)

``check_segment_mode.py <launch.log> <mpc|closed_form|mpc_docking> [--search <grid|nlp>]``
exits 0 when the controller's startup lines say exactly the expected mode, 1 otherwise
(the reason on stderr). With ``--search`` the ``search mode: <grid|nlp> — …`` line is
checked the same way, besides the segment line. ``run_unit.sh`` calls this.

A thin shim: the checks are ``integrated_bringup.catching_startup`` (installed, so
that ``catch_population_trials`` asks the same functions — its docstring says why the
expected line has to be PRESENT).
"""

import sys

from integrated_bringup.catching_startup import segment_mode_main as main

if __name__ == "__main__":
    sys.exit(main(sys.argv))
