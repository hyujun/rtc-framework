"""Which files under the checkout colcon would build -- the scope of the repo-wide gates.

validate_test_domains.py and validate_test_registration.py walk the checkout
with rglob rather than with colcon's package discovery, so they must leave out
what colcon leaves out, or they judge copies colcon never builds or tests.
colcon-recursive-crawl skips every directory whose name starts with "." and
every directory holding a COLCON_IGNORE file. Missing the first rule cost a
false FAIL: an agent worktree under .claude/worktrees/ duplicated every
package, and the domain gate reported each ROS_DOMAIN_ID claim as owned by two
packages (#945).

build/ install/ log/ are named as well. colcon marks them with COLCON_IGNORE,
but a tree misplaced inside the repo (AGENTS.md §9.1) is not guaranteed to
carry the marker. __pycache__ holds no package, so skipping it only saves a
walk.
"""

from __future__ import annotations

from pathlib import Path

SKIP_DIRS = frozenset({"build", "install", "log", "__pycache__"})
IGNORE_MARKER = "COLCON_IGNORE"


def is_skipped_dir(name: str) -> bool:
    """A directory name colcon's crawl never enters (hidden) or a build tree."""
    return name.startswith(".") or name in SKIP_DIRS


def in_colcon_scope(path: Path, root: Path) -> bool:
    """True when colcon's crawl from `root` would reach the file `path`.

    Every directory between `root` (exclusive) and the file must be one the
    crawl enters, and none of them, `root` included, may hold COLCON_IGNORE.
    """
    rel_dirs = path.relative_to(root).parent.parts
    if any(is_skipped_dir(part) for part in rel_dirs):
        return False
    d = root
    for part in (None, *rel_dirs):
        if part is not None:
            d = d / part
        if (d / IGNORE_MARKER).exists():
            return False
    return True


def self_test_failures() -> list[str]:
    """The cases both gates rely on, on a throwaway tree."""
    import tempfile

    failures = []
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        cases = {
            "pkg/package.xml": True,
            "group/pkg/package.xml": True,
            ".claude/worktrees/agent-x/pkg/package.xml": False,
            ".git/pkg/package.xml": False,
            ".venv/lib/pkg/package.xml": False,
            "build/pkg/package.xml": False,
            "ignored/pkg/package.xml": False,
            "pkg/.hidden_test/test_x.py": False,
        }
        for rel in cases:
            (root / rel).parent.mkdir(parents=True, exist_ok=True)
            (root / rel).write_text("\n")
        (root / "ignored" / IGNORE_MARKER).write_text("")
        for rel, want in cases.items():
            got = in_colcon_scope(root / rel, root)
            if got != want:
                failures.append(f"colcon scope of {rel}: expected {want}, got {got}")
    return failures
