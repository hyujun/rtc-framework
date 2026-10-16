"""Virtual TCP panel logic for demo_controller_gui (#847) — Tk- and rclpy-free.

The task, compliance and joint controllers declare ``virtual_tcp_mode``,
``virtual_tcp_offset`` and ``virtual_tcp_orientation`` as ROS parameters and
accept a set only while the controller is inactive
(``integrated_bringup/support/virtual_tcp_params.hpp`` owns that contract).
This module holds what the Control tab's Virtual TCP panel decides, so it can be
tested without a display or a ROS graph:

- which controllers own the parameters (``VTCP_OWNERS`` — defined here only),
- how a ``get_parameters`` answer becomes a value the panel can show,
- what the operator typed, validated before it is sent,
- whether Apply is offered for the selected controller.

The Apply gate is a hint for the operator, not the authority. The controller's
callback refuses a set while active, and the GUI shows that refusal verbatim;
the gate only spares the round trip and says why up front.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from rcl_interfaces.msg import ParameterType

VTCP_MODE_PARAM = "virtual_tcp_mode"
VTCP_OFFSET_PARAM = "virtual_tcp_offset"
VTCP_ORIENTATION_PARAM = "virtual_tcp_orientation"

# Request order for get_parameters; parse_entries reads the answer in this order.
VTCP_PARAMS: tuple[str, str, str] = (VTCP_MODE_PARAM, VTCP_OFFSET_PARAM, VTCP_ORIENTATION_PARAM)

# The controller's whitelist, in VirtualTcpMode enum order.
VTCP_MODES: tuple[str, ...] = ("disabled", "centroid", "weighted", "constant")

# Controllers that declare the three parameters. The only definition — the
# test cross-checks it against the C++ parameters.cpp files that call
# DeclareVirtualTcpParameters.
VTCP_OWNERS: frozenset[str] = frozenset(
    {"demo_joint_controller", "demo_task_controller", "demo_compliance_controller"}
)

HINT_NOT_OWNER = "no virtual TCP parameters on this controller"
HINT_ACTIVE_UNKNOWN = "active controller not known yet — Apply waits for it"
HINT_ACTIVE = "active — switch to another controller to edit"
HINT_INACTIVE = "inactive — Apply takes effect on the next activation"


@dataclass(frozen=True)
class VtcpConfigView:
    """One controller's virtual TCP parameters as the panel shows them."""

    mode: str
    offset: tuple[float, float, float]  # [x, y, z] in the TCP frame [m]
    orientation: tuple[float, float, float]  # [roll, pitch, yaw] in the TCP frame [rad]


def _triple(values: Sequence[float]) -> tuple[float, float, float] | None:
    vals = [float(v) for v in values]
    if len(vals) != 3 or not all(math.isfinite(v) for v in vals):
        return None
    return (vals[0], vals[1], vals[2])


def parse_entries(values: Sequence) -> VtcpConfigView | None:
    """Turn a ``get_parameters(VTCP_PARAMS)`` answer into a view.

    ``values`` are ``rcl_interfaces/msg/ParameterValue`` (duck-typed: ``type``,
    ``string_value``, ``double_array_value``) in ``VTCP_PARAMS`` order.

    Returns None when anything is not what the controller declares: a missing
    entry, PARAMETER_NOT_SET (a controller older than #847), a wrong type, an
    array that is not three finite numbers, or a mode outside the whitelist. The
    panel then shows "unknown" rather than a guess.
    """
    if len(values) != len(VTCP_PARAMS):
        return None
    mode_v, offset_v, rpy_v = values
    if mode_v.type != ParameterType.PARAMETER_STRING or mode_v.string_value not in VTCP_MODES:
        return None
    double_array = ParameterType.PARAMETER_DOUBLE_ARRAY
    if offset_v.type != double_array or rpy_v.type != double_array:
        return None
    offset = _triple(offset_v.double_array_value)
    rpy = _triple(rpy_v.double_array_value)
    if offset is None or rpy is None:
        return None
    return VtcpConfigView(mode=mode_v.string_value, offset=offset, orientation=rpy)


def parse_inputs(
    mode: str, offset_texts: Sequence[str], rpy_texts: Sequence[str]
) -> tuple[VtcpConfigView | None, str]:
    """Validate what the operator typed. Returns ``(view, "")`` or ``(None, why)``.

    Checks the same things the controller does (whitelist, three finite values)
    so an obvious typo is answered without a round trip. The controller checks
    again; this is not the gate.
    """
    if mode not in VTCP_MODES:
        return None, "select a mode first (Load reads the current one)"
    parsed = []
    for label, texts in (("offset", offset_texts), ("rpy", rpy_texts)):
        try:
            triple = _triple([float(t) for t in texts])
        except ValueError:
            return None, f"{label}: enter three numbers"
        if triple is None:
            return None, f"{label}: enter three finite numbers"
        parsed.append(triple)
    return VtcpConfigView(mode=mode, offset=parsed[0], orientation=parsed[1]), ""


def format_value(v: float) -> str:
    """Text for one value, exact on the way back.

    ``repr`` is the shortest string that parses to the same double. Apply sends
    all three keys from the input widgets, so a rounded seed would rewrite the
    values the operator did not touch (0.12345 → 0.1235) and move the control
    point on a mode-only edit.
    """
    return repr(float(v))


def format_readout(view: VtcpConfigView | None) -> str:
    """One line for the panel's readout of the controller's current values."""
    if view is None:
        return "current: unknown"
    off = ", ".join(format_value(v) for v in view.offset)
    rpy = ", ".join(format_value(v) for v in view.orientation)
    return f"current: {view.mode} · offset [{off}] m · rpy [{rpy}] rad"


def seed_due(seeded: tuple[str, VtcpConfigView | None] | None, ctrl: str, view) -> bool:
    """Whether the input widgets should be overwritten with ``view``.

    Only when the controller's values (or the selected controller) changed since
    the last seed, so a re-read that returns the same values — the one after a
    refused Apply — leaves an unapplied edit in place.

    "Unknown" for the controller already seeded is not a change: it is the
    window between dropping the cache and the re-read landing, and a catalog
    poll that paints it there must not blank the inputs (the re-read would then
    refill them and the edit would be lost). Unknown still blanks the inputs
    when the selection moved to another controller.
    """
    if view is None and seeded is not None and seeded[0] == ctrl:
        return False
    return seeded != (ctrl, view)


def apply_enabled(selected: str, active_name: str, owners: frozenset[str]) -> tuple[bool, str]:
    """Whether Apply is offered for ``selected``, and the hint that says why.

    Fail-closed on an unknown active controller (no ``active_controller_name``
    message yet): offering Apply then could send a set to the controller that is
    in fact running, which the controller would refuse anyway — but the panel
    would have invited it.
    """
    if selected not in owners:
        return False, HINT_NOT_OWNER
    if not active_name:
        return False, HINT_ACTIVE_UNKNOWN
    if selected == active_name:
        return False, HINT_ACTIVE
    return True, HINT_INACTIVE
