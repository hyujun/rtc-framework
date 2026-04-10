#!/usr/bin/env python3
"""Merge per-package compile_commands.json from colcon build into one."""

import json
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parent.parent.parent  # rtc_ws/
BUILD_DIR = WORKSPACE / "build"
OUTPUT = Path(__file__).resolve().parent / "build" / "compile_commands.json"


def main():
    merged = []
    for cc in sorted(BUILD_DIR.rglob("compile_commands.json")):
        with open(cc) as f:
            merged.extend(json.load(f))
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    with open(OUTPUT, "w") as f:
        json.dump(merged, f, indent=2)
    print(f"Merged {len(merged)} entries -> {OUTPUT}")


if __name__ == "__main__":
    main()
