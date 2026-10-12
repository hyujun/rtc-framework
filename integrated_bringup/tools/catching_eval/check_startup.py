#!/usr/bin/env python3
"""Has the catching controller come up, or has the bringup failed? (#747)

``check_startup.py <launch.log>`` exits 0 when the log has the line the controller
writes once it can throw (``supervisor: trials enabled``), 1 when the bringup has
failed (the first failure line on stderr), 2 when neither is there yet
(``run_unit.sh`` polls it once a second).

A thin shim: the check is ``integrated_bringup.catching_startup`` (installed, so
that ``catch_population_trials`` asks the same functions — its docstring says what
a failure line is and why the level matters).
"""

import sys

from integrated_bringup.catching_startup import startup_main as main

if __name__ == "__main__":
    sys.exit(main(sys.argv))
