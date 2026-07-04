#!/usr/bin/env python3
"""Produce a single compile_commands.json at repo-root for clangd / IDE.

colcon-cmake already aggregates every package's compile_commands.json into
``<ws>/build/compile_commands.json`` (its CompileCommandsEventHandler runs
whenever CMake is invoked). We copy that aggregate to repo-root, next to
``.clangd`` (which sets ``CompilationDatabase: .``). Keeping the DB at repo-root
— rather than pointing clangd at ``<ws>/build`` — avoids creating a ``build/``
folder here, which would falsely signal that ``colcon build`` in the repo is
fine. colcon must always run from ws-root (see .claude/rules/colcon-cwd.md).

Fallback: if the colcon aggregate is absent (older colcon, or a build that
never invoked CMake), merge the per-package ``build/<pkg>/compile_commands.json``
files directly. This intentionally does NOT recurse (``rglob``): recursion would
pull in BOTH the workspace-level aggregate AND every per-package file, doubling
each entry. ``glob('*/compile_commands.json')`` matches only per-package DBs —
the top-level aggregate (no intermediate dir) and ``_deps/`` nested DBs are
excluded by construction.
"""

import json
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent
WORKSPACE = REPO_ROOT.parent.parent  # <ws>/src/rtc-framework -> <ws>/
BUILD_DIR = WORKSPACE / "build"
AGGREGATE = BUILD_DIR / "compile_commands.json"
OUTPUT = REPO_ROOT / "compile_commands.json"


def load_aggregate():
    """Return colcon's workspace-level DB entries, or None if absent/invalid."""
    if not AGGREGATE.is_file():
        return None
    try:
        return json.loads(AGGREGATE.read_text())
    except (json.JSONDecodeError, OSError) as exc:
        print(f"warning: ignoring unreadable {AGGREGATE}: {exc}", file=sys.stderr)
        return None


def merge_per_package():
    """Merge each build/<pkg>/compile_commands.json (direct children only)."""
    entries = []
    for cc in sorted(BUILD_DIR.glob("*/compile_commands.json")):
        try:
            entries.extend(json.loads(cc.read_text()))
        except (json.JSONDecodeError, OSError) as exc:
            print(f"warning: skipping {cc}: {exc}", file=sys.stderr)
    return entries


def write_atomic(entries):
    """Write via temp + os.replace so clangd never reads a half-written file."""
    tmp = OUTPUT.parent / (OUTPUT.name + ".tmp")
    # No indent: the DB is gitignored (never diffed) and can reach several MB —
    # compact output roughly halves the size and speeds clangd's initial load.
    tmp.write_text(json.dumps(entries, separators=(",", ":")))
    os.replace(tmp, OUTPUT)


def main():
    entries = load_aggregate()
    source = "colcon aggregate"
    if entries is None:
        entries = merge_per_package()
        source = "per-package merge"

    if not entries:
        print(
            f"error: no compile_commands.json found under {BUILD_DIR} — leaving "
            f"existing {OUTPUT.name} untouched (build the workspace first)",
            file=sys.stderr,
        )
        return 1

    write_atomic(entries)
    print(f"Wrote {len(entries)} entries -> {OUTPUT} ({source})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
