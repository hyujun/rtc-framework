"""Resolve the ``sim_overlay:=`` launch argument to a params-file path.

Shared by the per-profile sim launch files. It lives here rather than being
copied into each one because the failure it guards against is silent: an
overlay that does not resolve and is skipped runs the SHIPPED scene under a
command line naming a different one, and every measurement taken from that run
describes the wrong scene. A second transcription is a second chance to get
that wrong, and the copy that drifts is the one nobody re-reads.

(The asymmetry this module removes is the same shape as the missing controller
YAML that kept two of three profiles from booting: a facility present on one
profile and absent on another, where the absence reads as "not supported" only
after someone has already trusted the flag.)
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory


def resolve_sim_overlay(value: str, profile: str) -> str | None:
    """``sim_overlay:=`` → an existing params-file path, or None when unset.

    A bare name is looked up among ``profile``'s shipped overlays; anything with
    a path separator or a YAML suffix is taken as a path. A value that resolves
    to nothing RAISES rather than falling back — see the module docstring.
    """
    value = value.strip()
    if not value:
        return None
    overlay_dir = os.path.join(
        get_package_share_directory("integrated_bringup"), "config", profile, "sim_overlays"
    )
    if os.sep in value or value.endswith((".yaml", ".yml")):
        path = os.path.abspath(os.path.expanduser(value))
    else:
        path = os.path.join(overlay_dir, value + ".yaml")
    if not os.path.isfile(path):
        shipped = (
            sorted(f[: -len(".yaml")] for f in os.listdir(overlay_dir) if f.endswith(".yaml"))
            if os.path.isdir(overlay_dir)
            else []
        )
        raise RuntimeError(
            f"sim_overlay '{value}' resolves to {path}, which does not exist. "
            f"Shipped overlays for {profile}: {shipped} (or pass a path to a params YAML)."
        )
    return path


def sim_overlay_argument_description(profile: str) -> str:
    """The ``DeclareLaunchArgument`` description, so both launches say the same."""
    return (
        f"Extra params YAML applied after config/{profile}/mujoco_simulator.yaml "
        "and before the other command-line overrides, to both nodes. A bare name "
        f"(e.g. inference_pole) resolves to config/{profile}/sim_overlays/<name>.yaml; "
        "a value containing '/' or ending in .yaml is a path. An unresolvable value "
        "fails the launch. Empty = the shipped scene."
    )
