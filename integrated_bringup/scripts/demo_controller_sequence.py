#!/usr/bin/env python3
"""Thin entrypoint shim — implementation lives in
``integrated_bringup.demo_sequence`` (see cli.py)."""

import sys

from integrated_bringup.demo_sequence.cli import main

if __name__ == "__main__":
    sys.exit(main())
