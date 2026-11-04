"""Vendored-asset byte-identity gate — ``robots/**/MANIFEST.sha256``.

An upstream robot description copied into ``robot_descriptions/robots/<name>/``
in whole (WD-13 of the humanoid plan, #974) carries a ``MANIFEST.sha256`` in
``sha256sum`` format, written at the copy. The README of the robot says which
upstream and which revision. This test re-hashes every listed file and fails
on any drift, and on any file in that directory the manifest does not name —
a silently edited or added file is exactly what the manifest exists to catch,
and "upstream 과 바이트 동일" is only a claim until something checks it.

``hashlib`` only, so it runs under colcon's ``/usr/bin/python3`` as well as
the venv — unlike ``test_real_model_pairs.py`` there is no ``importorskip``
here, and a skip would not be a pass.

Regenerate after an intentional upstream refresh (from inside the vendored
directory, then update the robot README)::

    find . -type f ! -name MANIFEST.sha256 | sed 's|^\\./||' | LC_ALL=C sort \\
      | xargs sha256sum > MANIFEST.sha256
"""

from __future__ import annotations

import hashlib
from pathlib import Path

import pytest
from conftest import robots_dir

ROBOTS = robots_dir()
MANIFESTS = sorted(ROBOTS.rglob("MANIFEST.sha256")) if ROBOTS else []


def _parse(manifest: Path) -> dict[str, str]:
    """``<sha256>  <relative path>`` per line (sha256sum format)."""
    entries: dict[str, str] = {}
    for line in manifest.read_text().splitlines():
        if not line.strip():
            continue
        digest, _, rel = line.partition("  ")
        assert len(digest) == 64 and rel, f"{manifest}: malformed line {line!r}"
        entries[rel] = digest
    return entries


def test_at_least_one_manifest_is_present():
    """게이트가 조용히 0 개를 돌면 그건 통과가 아니라 미실행이다."""
    assert ROBOTS is not None, "robot_descriptions/robots 를 찾지 못했다"
    assert MANIFESTS, "robots/**/MANIFEST.sha256 이 하나도 없다"


@pytest.mark.parametrize(
    "manifest", MANIFESTS, ids=[str(m.relative_to(ROBOTS).parent) for m in MANIFESTS]
)
def test_vendored_files_match_manifest(manifest: Path):
    root = manifest.parent
    entries = _parse(manifest)
    assert entries, f"{manifest} is empty"

    drifted = []
    for rel, expected in entries.items():
        path = root / rel
        assert path.is_file(), f"{manifest}: listed file missing: {rel}"
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        if actual != expected:
            drifted.append(rel)
    assert not drifted, f"{manifest}: content differs from the manifest: {drifted}"

    on_disk = {
        str(p.relative_to(root))
        for p in root.rglob("*")
        if p.is_file() and p.name != "MANIFEST.sha256"
    }
    unlisted = sorted(on_disk - set(entries))
    assert not unlisted, f"{manifest}: files not named by the manifest: {unlisted}"
