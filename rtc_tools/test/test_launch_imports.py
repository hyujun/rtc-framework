"""Packaging smoke test: every public ``rtc_tools.launch`` symbol used by
``integrated_bringup`` launch files must be importable from the installed
``rtc_tools`` package.

If ``setup.py`` ever stops shipping a sub-package (forgotten ``find_packages``
update, deleted ``__init__.py``, renamed module), this test fails at
``colcon test`` time — long before a launch file would ``ImportError`` at
``ros2 launch`` time on a fresh install.

The symbols listed below are the exact ones imported by
``integrated_bringup/launch/{robot_ur5e_p1a,sim_ur5e_p1a,sim_iiwa7_leap}.launch.py``.
"""

from __future__ import annotations

import importlib

import pytest


def test_trace_action_helper_importable() -> None:
    """``make_trace_action`` is used by sim*.launch.py / robot_ur5e_p1a.launch.py for
    opt-in ros2_tracing capture.

    ``trace_action`` depends on the ROS 2 ``launch`` package, which is unavailable
    outside the colcon environment — skip when that is the case so the test
    suite still passes for plain ``pytest`` runs while validating packaging
    under ``colcon test``.
    """
    if importlib.util.find_spec("launch") is None:
        pytest.skip("ROS 2 'launch' package unavailable (not in colcon env)")
    from rtc_tools.launch.trace_action import make_trace_action

    assert callable(make_trace_action)


def test_thread_layout_helpers_importable() -> None:
    """The four ``get_*_core`` accessors are used by the launch files."""
    from rtc_tools.launch.thread_layout import (
        get_arm_driver_core,
        get_hand_driver_core,
        get_sim_core,
        get_viewer_core,
    )

    for fn in (get_arm_driver_core, get_hand_driver_core, get_sim_core, get_viewer_core):
        assert callable(fn)
        result = fn(12)  # canonical tier
        assert isinstance(result, int)


def test_session_dir_helpers_importable() -> None:
    """``session_dir`` helpers are used by both robot_ur5e_p1a.launch.py and sim*.launch.py."""
    from rtc_tools.utils.session_dir import (
        cleanup_old_sessions,
        create_session_dir,
        resolve_logging_root,
    )

    for fn in (cleanup_old_sessions, create_session_dir, resolve_logging_root):
        assert callable(fn)
