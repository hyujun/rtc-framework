#!/usr/bin/env python3
"""Thin entrypoint shim — implementation lives in
``integrated_bringup.catching_sim_trials``."""

import sys

from integrated_bringup.catching_sim_trials import main

if __name__ == "__main__":
    sys.exit(main())
