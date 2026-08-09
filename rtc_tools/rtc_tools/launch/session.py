"""Open a bringup run's session directory from inside a launch context (#402).

Why this is not a call in ``generate_launch_description()``
----------------------------------------------------------
``create_session_dir`` makes directories and ``cleanup_old_sessions`` deletes
them, so *where* a launch calls the pair decides whether merely **building** a
launch description touches the user's ``logging_data``. Both robot launches used
to call them from the body of ``generate_launch_description()``, which meant
``ros2 launch --show-args``, a linter, or the launch-evaluation sensor mutated
the real workspace without launching anything: measured on a tree seeded with 14
sessions, one directory created and five deleted per call. The sim launches were
already a level down, inside their ``launch_setup`` ``OpaqueFunction``.

Moving the pair behind a launch *context* is also what makes the retention
policy configurable at all: a launch argument can only be read through a
context, which is why the robot launches hardcoded ``10`` while the sim launches
could honour ``max_log_sessions``.

Retention has two actors, not one
---------------------------------
The RT controller node prunes the same tree **again** in ``on_configure``, using
its own ``max_log_sessions`` ROS parameter (``rtc_controller_manager``'s
``rt_controller_node_params.cpp``; the YAML that sets it is the same file this
module reads). Two actors with independent numbers made the effective retention
``min(launch value, node parameter)`` — raising the YAML to 30 still left 10,
because the launch had already pruned to its hardcoded 10, and lowering the sim
launch argument below the YAML worked while raising it above did nothing.

So the launch argument's default is read out of the node's own YAML
(:func:`max_log_sessions_from_yaml`) and the resolved value is handed back to
the node as a parameter override. Untouched, both actors see the YAML value;
overridden on the command line, both see the override. One knob, and the YAML
stays the single source of truth.
"""

from __future__ import annotations

from typing import NamedTuple

import yaml
from launch.actions import SetEnvironmentVariable

from rtc_tools.utils.session_dir import (
    cleanup_old_sessions,
    create_session_dir,
    generate_run_id,
    resolve_logging_root,
)

#: Launch argument carrying the retention count.
MAX_SESSIONS_ARG = "max_log_sessions"

#: Launch configuration the opened session path is published under. Actions
#: built at description-build time (a node's parameter dict, for instance)
#: cannot capture the path as a string, so they read it back as a
#: ``LaunchConfiguration`` substitution resolved at their own execution time.
SESSION_DIR_CONFIG = "session_dir"


def max_log_sessions_from_yaml(path: str) -> int:
    """Read ``max_log_sessions`` out of a ``/**: ros__parameters:`` config file.

    Mirrors :func:`rtc_tools.launch.cm_rt_params.control_rate_from_yaml`,
    including its refusal to fall back on a default: the entire reason for
    reading the file is that the launch argument and the RT node's parameter
    must agree, and a silent default here would quietly restore the ``min()``
    trap the read exists to remove.
    """
    with open(path) as handle:
        data = yaml.safe_load(handle) or {}
    params = (data.get("/**", {}) or {}).get("ros__parameters", {}) or {}
    if MAX_SESSIONS_ARG not in params:
        raise KeyError(f"{MAX_SESSIONS_ARG} not found in {path}")
    return int(params[MAX_SESSIONS_ARG])


class OpenedSession(NamedTuple):
    """What :func:`open_session` hands back to a launch file.

    ``max_sessions`` is returned rather than left implicit because the RT node
    has to receive the very same number — see this module's docstring.
    """

    directory: str
    max_sessions: int
    actions: list


def open_session(
    context,
    *,
    max_sessions_arg: str = MAX_SESSIONS_ARG,
    session_dir_config: str = SESSION_DIR_CONFIG,
) -> OpenedSession:
    """Create this run's session directory, prune old ones, publish the path.

    Call from inside an ``OpaqueFunction`` (or anywhere else holding a launch
    context) placed **after** the ``max_log_sessions`` declaration and
    **before** every consumer of the session path. Getting that order wrong
    raises here rather than silently pruning to some other number: an
    undeclared argument reads back as the empty string, which is rejected.

    The path is published two ways because its consumers differ. Actions built
    at description-build time read it back as the ``session_dir`` launch
    configuration; child processes get it from ``$RTC_SESSION_DIR`` via the
    returned actions.
    """
    raw = str(context.launch_configurations.get(max_sessions_arg, "")).strip()
    if not raw:
        raise RuntimeError(
            f"{max_sessions_arg} is unset. Declare it with DeclareLaunchArgument "
            "and open the session after that declaration — an OpaqueFunction "
            "placed before it cannot read the argument."
        )
    try:
        max_sessions = int(raw)
    except ValueError:
        raise RuntimeError(
            f"Invalid {max_sessions_arg}={raw!r}: expected an integer count of "
            "session folders to keep."
        ) from None

    logging_root = resolve_logging_root()
    directory = create_session_dir(logging_root)
    cleanup_old_sessions(logging_root, max_sessions)
    context.launch_configurations[session_dir_config] = directory

    return OpenedSession(
        directory=directory,
        max_sessions=max_sessions,
        actions=[
            SetEnvironmentVariable(name="RTC_SESSION_DIR", value=directory),
            # 세션 디렉토리는 분 해상도라 같은 분의 재기동이 같은 디렉토리를 얻고,
            # 타이밍 CSV 는 거기에 append 된다. 이 런 ID 가 그 파일 안의 런 경계이며,
            # 한 launch 의 모든 노드가 같은 값을 받아야 CSV 간 join 이 성립한다 (#376).
            SetEnvironmentVariable(name="RTC_RUN_ID", value=generate_run_id()),
        ],
    )
