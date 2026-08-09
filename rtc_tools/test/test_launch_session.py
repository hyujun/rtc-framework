"""Unit tests for :mod:`rtc_tools.launch.session` (#402).

Deliberately self-contained: CI's Python Test job installs only ``rtc_tools``,
``rtc_msgs`` and ``rtc_digital_twin``, so a test here that resolved
``integrated_bringup`` through ``ament_index`` would pass locally (where every
package is installed) and fail only in CI. The launch files' side of this
wiring is covered by ``integrated_bringup/test/test_launch_session_wiring.py``.
"""

from __future__ import annotations

import os

import pytest
from launch import LaunchContext
from launch.actions import SetEnvironmentVariable

from rtc_tools.launch.session import (
    MAX_SESSIONS_ARG,
    SESSION_DIR_CONFIG,
    max_log_sessions_from_yaml,
    open_session,
)
from rtc_tools.utils.session_dir import is_session_dir_name


@pytest.fixture
def logging_root(monkeypatch, tmp_path):
    """Point ``resolve_logging_root`` at ``tmp_path`` — and prove it took.

    ``open_session`` deletes directories. A redirect that silently stopped
    working would show up as missing developer data, never as a red test, so
    the assertion is part of the fixture rather than a comment.
    """
    prefix = tmp_path / "install"
    prefix.mkdir()
    monkeypatch.setenv("COLCON_PREFIX_PATH", str(prefix))
    root = tmp_path / "logging_data"
    root.mkdir()

    from rtc_tools.utils.session_dir import resolve_logging_root

    assert resolve_logging_root() == str(root), (
        "logging root escaped the sandbox; this test prunes session directories"
    )
    return root


def _context(**configurations) -> LaunchContext:
    context = LaunchContext()
    context.launch_configurations.update(configurations)
    return context


def _seed(root, count: int) -> list[str]:
    """Create ``count`` session directories that sort before any real one."""
    names = [f"2601{index:02d}_1200" for index in range(count)]
    for name in names:
        (root / name).mkdir()
    return names


# ── max_log_sessions_from_yaml ──────────────────────────────────────────────


def test_reads_the_node_parameter_out_of_a_config_file(tmp_path):
    path = tmp_path / "_base.yaml"
    path.write_text("/**:\n  ros__parameters:\n    max_log_sessions: 30\n")
    assert max_log_sessions_from_yaml(str(path)) == 30


def test_missing_key_is_loud(tmp_path):
    """No silent default: a fallback here would restore the min() trap.

    The whole point of reading the file is that the launch argument and the RT
    node's parameter carry the same number.
    """
    path = tmp_path / "_base.yaml"
    path.write_text("/**:\n  ros__parameters:\n    control_rate: 500.0\n")
    with pytest.raises(KeyError, match=MAX_SESSIONS_ARG):
        max_log_sessions_from_yaml(str(path))


def test_missing_file_is_loud(tmp_path):
    with pytest.raises(OSError):
        max_log_sessions_from_yaml(str(tmp_path / "does_not_exist.yaml"))


# ── open_session ────────────────────────────────────────────────────────────


def test_creates_the_session_and_publishes_it(logging_root):
    context = _context(**{MAX_SESSIONS_ARG: "10"})
    session = open_session(context)

    assert os.path.isdir(session.directory)
    assert is_session_dir_name(os.path.basename(session.directory))
    assert os.path.dirname(session.directory) == str(logging_root)
    # Consumers built at description-build time read the path back through this
    # launch configuration; there is no other channel for them.
    assert context.launch_configurations[SESSION_DIR_CONFIG] == session.directory


def test_exports_the_session_and_run_id_to_child_processes(logging_root):
    session = open_session(_context(**{MAX_SESSIONS_ARG: "10"}))

    exported = {}
    for action in session.actions:
        assert isinstance(action, SetEnvironmentVariable)
        exported[action.name[0].perform(LaunchContext())] = action.value[0].perform(
            LaunchContext()
        )

    assert exported["RTC_SESSION_DIR"] == session.directory
    # Same-minute restarts share a session directory, so the run id is the only
    # boundary inside an appended timing CSV (#376).
    assert exported["RTC_RUN_ID"].isdigit()


def test_prunes_down_to_the_requested_count(logging_root):
    _seed(logging_root, 5)
    session = open_session(_context(**{MAX_SESSIONS_ARG: "2"}))

    remaining = sorted(p.name for p in logging_root.iterdir())
    assert len(remaining) == 2
    # This run's session is the newest, so pruning must never take it.
    assert os.path.basename(session.directory) in remaining


def test_reports_the_count_it_pruned_with(logging_root):
    """The RT node prunes the same tree again and must get this same number.

    Two independent numbers is what made the effective retention
    ``min(launch, node)`` before #402.
    """
    assert open_session(_context(**{MAX_SESSIONS_ARG: "7"})).max_sessions == 7


def test_an_undeclared_argument_is_loud(logging_root):
    """Positive control for the ordering constraint the prologue introduced.

    An ``OpaqueFunction`` placed above the ``DeclareLaunchArgument`` reads the
    argument back as unset. Defaulting here instead would prune to some number
    nobody chose, and the launch would look fine.
    """
    with pytest.raises(RuntimeError, match="DeclareLaunchArgument"):
        open_session(_context())
    assert not list(logging_root.iterdir()), "a rejected call must not create a session"


def test_a_non_integer_count_is_loud(logging_root):
    with pytest.raises(RuntimeError, match="Invalid"):
        open_session(_context(**{MAX_SESSIONS_ARG: "ten"}))
