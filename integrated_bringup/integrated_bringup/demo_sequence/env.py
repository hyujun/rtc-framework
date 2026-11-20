"""Process environment of the CLI: where its files are, and which RMW it talks.

Both are settled before rclpy is imported. The robot launch runs every node on
``rmw_cyclonedds_cpp`` with the CM's ``cyclone_dds.xml`` but sets them only for
its own processes, so a CLI started from a fresh terminal would otherwise come
up on the default RMW and might never see the stack.
"""

from __future__ import annotations

import os
from collections.abc import MutableMapping

CYCLONE = "rmw_cyclonedds_cpp"


def logging_root(explicit: str | None) -> tuple[str, str]:
    """(the logging root, "") or ("", why there is none).

    Only a root the workspace names counts (``workspace_logging_root``): the
    session helpers' last fallback, ``$PWD/logging_data``, is wherever the
    terminal happens to be, and a run reading its presets from there reads
    files nobody made.
    """
    if explicit is not None:
        if not os.path.isdir(explicit):
            return "", f"--logging-root {explicit} is not a directory"
        return explicit, ""
    from rtc_tools.utils.session_dir import workspace_logging_root

    root = workspace_logging_root()
    if root is None:
        return "", (
            "no colcon workspace in the environment or above this directory — source "
            "the workspace, or pass --logging-root"
        )
    return root, ""


def cyclone_uri() -> str:
    from ament_index_python.packages import get_package_share_directory

    share = get_package_share_directory("rtc_controller_manager")
    return "file://" + os.path.join(share, "config", "cyclone_dds.xml")


def align_rmw(environ: MutableMapping[str, str], uri=cyclone_uri) -> list[str]:
    """Bring ``environ`` to the robot launch's RMW; returns what was done.

    Unset → cyclone with the stack's URI. Cyclone without a URI → the URI.
    Anything else is left alone with a warning: the operator chose it.
    """
    rmw = environ.get("RMW_IMPLEMENTATION", "")
    notes = []
    if not rmw:
        environ["RMW_IMPLEMENTATION"] = CYCLONE
        notes.append(f"RMW_IMPLEMENTATION unset — using {CYCLONE}, as the robot launch does")
        rmw = CYCLONE
    if rmw == CYCLONE and not environ.get("CYCLONEDDS_URI"):
        environ["CYCLONEDDS_URI"] = uri()
        notes.append(f"CYCLONEDDS_URI unset — using {environ['CYCLONEDDS_URI']}")
    elif rmw != CYCLONE:
        notes.append(
            f"WARNING: RMW_IMPLEMENTATION={rmw}, the robot launch uses {CYCLONE} — "
            "if the stack is not found, unset it"
        )
    return notes


def describe(environ: MutableMapping[str, str]) -> str:
    """The DDS environment in one line, for the waiting messages."""
    keys = (
        "ROS_DOMAIN_ID",
        "RMW_IMPLEMENTATION",
        "CYCLONEDDS_URI",
        "ROS_AUTOMATIC_DISCOVERY_RANGE",
        "ROS_STATIC_PEERS",
    )
    return " ".join(f"{k}={environ.get(k, '') or '-'}" for k in keys)
