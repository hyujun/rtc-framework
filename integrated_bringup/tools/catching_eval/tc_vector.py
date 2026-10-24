#!/usr/bin/env python3
"""Catching sim evaluation: the t_c gap as VECTORS (along / across the ball's travel).

    tc_vector.py <unit_dir> <config_dir> <out.csv> [<ct_out_dir>]

Runs rtc_tools.analysis.catching_trials over <unit>/session + <unit>/trials and
writes one row per committed trial. ``analyse_unit.sh`` calls this.

A thin shim: the analysis is ``integrated_bringup.catching_tc_vector`` (installed,
so that ``catch_population_trials`` runs the same code after a unit — its docstring
says what the columns are and why the interpreter is the distribution's python).
"""

import sys

from integrated_bringup.catching_tc_vector import main

if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
