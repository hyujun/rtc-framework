"""The shipped ``ur5e_p1b`` profile on its test model, for tests that hand a tool
a profile directory and let it open the model.

The shipped profile loads its model from ``hand_description``, a separate project
this workspace does not depend on and whose files are not public: a test that
resolved it would fail wherever that project is absent (#952). A copy of the
profile whose ``urdf`` block names the test model instead —
``robot_descriptions/robots/p1b_surrogate/``, a synthetic hand with the real
one's link and joint names and loop topology but made-up geometry — is what
these tests hand the tools. Every other file and key is the shipped one, so a
change there still moves the tests; a number that depends on the hand's
geometry says nothing about the real robot.
"""

from __future__ import annotations

import functools
import shutil
import tempfile
from pathlib import Path

import pytest
import yaml

SHIPPED = Path(__file__).resolve().parents[2] / "integrated_bringup" / "config" / "ur5e_p1b"
TEST_URDF = "robots/p1b_surrogate/urdf/ur5e_with_p1b_surrogate.urdf.xacro"
TEST_CLOSURE = "robots/p1b_surrogate/urdf/ur5e_with_p1b_surrogate.closure.yaml"

_KEEP: list[tempfile.TemporaryDirectory] = []


def retarget_model(config_dir: Path) -> None:
    """Point the ``urdf`` block of ``config_dir/_base.yaml`` at the test model, in place."""
    base = config_dir / "_base.yaml"
    doc = yaml.safe_load(base.read_text())
    urdf = doc["/**"]["ros__parameters"]["urdf"]
    assert urdf["package"] == "hand_description", (
        f"{base}: the shipped model is no longer hand_description's — this copy has "
        "nothing left to retarget, so the helper is stale"
    )
    urdf.update(package="robot_descriptions", path=TEST_URDF, closure_path=TEST_CLOSURE)
    base.write_text(yaml.safe_dump(doc, sort_keys=False))


@functools.cache
def _copy() -> Path:
    holder = tempfile.TemporaryDirectory(prefix="ur5e_p1b_test_profile_")
    _KEEP.append(holder)
    dest = Path(holder.name) / "ur5e_p1b"
    shutil.copytree(SHIPPED, dest)
    retarget_model(dest)
    return dest


def ur5e_p1b_test_profile() -> Path:
    """A process-wide copy of the shipped profile on the test model (do not edit it)."""
    if not SHIPPED.is_dir():
        pytest.skip("integrated_bringup config is not beside this checkout")
    return _copy()
