"""The scene rule shared by the tests that compile a shipped MuJoCo scene.

They compile only scenes in ``robot_descriptions``. A profile whose scene is in
another package (``ur5e_p1b`` and ``g1_p1b`` load theirs from ``hand_description``,
a separate project this workspace does not depend on) is not a case: a test here
never resolves a package outside the repo (#952).
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory

SCENE_PACKAGE = "robot_descriptions"
_PREFIX = f"package://{SCENE_PACKAGE}/"


def is_scene_case(model_path: str) -> bool:
    """Whether a ``model_path`` is a scene these tests compile."""
    return model_path.startswith(_PREFIX)


def not_a_case_reason(model_path: str) -> str:
    return (
        f"{model_path!r} is not in {SCENE_PACKAGE}: these tests compile only "
        f"{SCENE_PACKAGE} scenes (#952)"
    )


def scene_file(model_path: str) -> str:
    """A :data:`SCENE_PACKAGE` ``package://`` URI -> the installed file (asserted to exist)."""
    assert is_scene_case(model_path), not_a_case_reason(model_path)
    path = os.path.join(get_package_share_directory(SCENE_PACKAGE), model_path[len(_PREFIX) :])
    assert os.path.isfile(path), f"{model_path!r} is not installed at {path}"
    return path
