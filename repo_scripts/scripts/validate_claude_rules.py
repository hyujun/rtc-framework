#!/usr/bin/env python3
"""Validate `.claude/rules/*.md` path-scoped rules against the real tree.

A path-scoped rule whose `paths:` globs match nothing is a rule that never
loads. That failure is silent by construction: the file is present, readable,
and reviewable, and the only symptom is guidance quietly not arriving. This
repo has already paid for the same shape once (a constitution copy that stopped
at 7 of 9 RT rules, #213), which is why a rule that matches zero files is an
error here rather than a warning.

This is the *computational* half of the sensor pair. It answers "could this
rule ever fire" without a session. The other half is the InstructionsLoaded
hook (`.claude/hooks/log-instructions-loaded.sh`), which answers "did it
actually fire" and is the ground truth -- glob semantics here are a faithful
but independent reimplementation of the runtime matcher, so agreement is not
guaranteed by construction. When the two disagree, the runtime log wins.

Semantics implemented, per the documented `paths` contract:
  - `**/*.ext` matches that extension in any directory
  - `dir/**/*` matches everything under dir/
  - `*.md` matches only the project root
  - brace expansion (`*.{ts,tsx}`) multiplies patterns
  - a pattern that cannot be read as a glob matches nothing without taking the
    rule's other patterns down with it

Exit status: 0 clean, 1 findings, 2 usage error.
"""

from __future__ import annotations

import argparse
import glob
import re
import sys
from pathlib import Path

# Directories that are build output or vendored trees. Counting matches inside
# them would let a rule look alive on the strength of files that are not
# source -- e.g. a stale install/ copy keeping a since-renamed glob "green".
PRUNE_DIRS = {"build", "install", "log", "deps", ".git", "__pycache__", "node_modules"}

FRONTMATTER_RE = re.compile(r"\A---\s*\n(.*?)\n---\s*(?:\n|\Z)", re.DOTALL)


def find_repo_root(start: Path) -> Path:
    for candidate in [start, *start.parents]:
        if (candidate / ".git").exists():
            return candidate
    return start


def parse_frontmatter(text: str) -> tuple[dict, bool]:
    """Return (frontmatter_mapping, had_frontmatter).

    Deliberately a narrow parser rather than PyYAML: the only shape that
    matters here is `paths:` followed by a block list of quoted scalars, and a
    hard dependency would make this gate fail-open on boxes without PyYAML --
    the opposite of what a silent-failure detector should do.
    """
    m = FRONTMATTER_RE.match(text)
    if not m:
        return {}, False

    data: dict[str, list[str]] = {}
    current_key: str | None = None
    for raw in m.group(1).splitlines():
        line = raw.rstrip()
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        item = re.match(r"^\s*-\s+(.*)$", line)
        if item and current_key:
            data[current_key].append(item.group(1).strip().strip("\"'"))
            continue
        key = re.match(r"^([A-Za-z_][A-Za-z0-9_-]*)\s*:\s*(.*)$", line)
        if key:
            current_key = key.group(1)
            inline = key.group(2).strip()
            data[current_key] = []
            if inline and not inline.startswith("#"):
                data[current_key].append(inline.strip("\"'"))
    return data, True


def expand_braces(pattern: str) -> list[str]:
    """Expand `{a,b}` groups, leftmost first. No expansion -> [pattern]."""
    m = re.search(r"\{([^{}]*)\}", pattern)
    if not m:
        return [pattern]
    out: list[str] = []
    for alt in m.group(1).split(","):
        out.extend(expand_braces(pattern[: m.start()] + alt + pattern[m.end() :]))
    return out


def is_pruned(rel_path: str) -> bool:
    return any(part in PRUNE_DIRS for part in Path(rel_path).parts)


def count_matches(root: Path, pattern: str) -> int:
    """Files under root matching one already-brace-expanded glob."""
    try:
        hits = glob.glob(pattern, root_dir=str(root), recursive=True)
    except (ValueError, re.error):
        # An unreadable pattern matches nothing; it must not abort the rest.
        return 0
    return sum(1 for h in hits if not is_pruned(h) and (root / h).is_file())


def check_rule(root: Path, rule_path: Path) -> list[str]:
    findings: list[str] = []
    rel = rule_path.relative_to(root).as_posix()
    text = rule_path.read_text(encoding="utf-8", errors="replace")

    fm, had_fm = parse_frontmatter(text)
    if not had_fm:
        # Legal and documented: a rule with no frontmatter loads unconditionally
        # every session. That is a context-budget decision, not a defect, so it
        # is reported at info level only.
        print(f"{rel}: [info] no frontmatter — loads unconditionally every session")
        return findings

    patterns = fm.get("paths") or []
    if not patterns:
        print(f"{rel}: [info] no `paths:` — loads unconditionally every session")
        return findings

    dead: list[str] = []
    total = 0
    for pattern in patterns:
        matched = sum(count_matches(root, p) for p in expand_braces(pattern))
        total += matched
        if matched == 0:
            dead.append(pattern)

    if total == 0:
        findings.append(
            f"{rel}: [R1] every `paths:` glob matches zero files — this rule can never load. "
            f"Patterns: {', '.join(patterns)}"
        )
    elif dead:
        # Not an error: rt-path.md deliberately carries redundant anchors, and a
        # pattern for a layout that does not exist yet is a defensible hedge.
        # Still worth printing, because a typo looks exactly like a hedge.
        print(
            f"{rel}: [info] {len(dead)} of {len(patterns)} pattern(s) match nothing "
            f"(rule still loads via the others): {', '.join(dead)}"
        )
    return findings


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--root", default=None, help="repo root (default: auto-detect)")
    ap.add_argument(
        "--files",
        nargs="*",
        default=None,
        help="specific rule files to check (default: all under .claude/rules/)",
    )
    args = ap.parse_args()

    root = Path(args.root).resolve() if args.root else find_repo_root(Path.cwd())

    if args.files:
        rules = [Path(f).resolve() for f in args.files]
    else:
        rules_dir = root / ".claude" / "rules"
        rules = sorted(rules_dir.rglob("*.md")) if rules_dir.is_dir() else []

    rules = [r for r in rules if r.is_file()]
    if not rules:
        print("no .claude/rules/*.md to validate")
        return 0

    findings: list[str] = []
    for rule in rules:
        try:
            findings.extend(check_rule(root, rule))
        except OSError as exc:  # unreadable file is itself a finding
            findings.append(f"{rule}: [R0] unreadable ({exc})")

    if findings:
        for f in findings:
            print(f, file=sys.stderr)
        print(f"\n{len(findings)} rule finding(s)", file=sys.stderr)
        return 1

    print(f"claude rules validation clean ({len(rules)} rule file(s))")
    return 0


if __name__ == "__main__":
    sys.exit(main())
