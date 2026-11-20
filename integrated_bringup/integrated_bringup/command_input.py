"""The controller parameter layers of a locomotion sim launch, and ``command_input:=``.

``controller_param_layers`` is the order the launch hands the RT node its
parameter files in; later layers win. ``command_input:=joy`` adds the
profile's shipped ``sim_overlays/joy_command.yaml`` right after the
``enable_mpc`` mapping and before a ``sim_overlay:=`` (so a user overlay still
wins). ``twist`` — the default — adds nothing, so the default launch builds the
same list it did before the argument existed.
"""

from __future__ import annotations

from integrated_bringup.sim_overlay import resolve_sim_overlay

COMMAND_INPUTS = ("twist", "joy")
JOY_OVERLAY = "joy_command"


def command_input_overlay(value: str, profile: str) -> str | None:
    """``command_input:=`` → the overlay file to add, or None for twist."""
    value = value.strip() or "twist"
    if value not in COMMAND_INPUTS:
        raise ValueError(f"command_input must be one of {COMMAND_INPUTS}, got '{value}'")
    if value == "twist":
        return None
    return resolve_sim_overlay(JOY_OVERLAY, profile)


def controller_param_layers(
    base_config,
    ctrl_config,
    sim_config,
    mpc_enabled: bool | None,
    command_overlay,
    sim_overlay,
) -> list:
    """Shipped YAMLs, then the enable_mpc mapping, the command input, the user overlay."""
    layers = [base_config, ctrl_config, sim_config]
    if mpc_enabled is not None:
        layers.append({"locomotion_controller.mpc.enabled": mpc_enabled})
    if command_overlay is not None:
        layers.append(command_overlay)
    if sim_overlay is not None:
        layers.append(sim_overlay)
    return layers
