"""Generate the ``controller_manager`` RT parameter file for the UR arm driver.

Why a generated file and not ``taskset``
----------------------------------------
``ros2_control_node`` runs its ``read()``/``update()``/``write()`` loop in a
dedicated thread (upstream ``controller_manager``), while ``main()`` stays on
the executor spin. ``taskset -cp`` takes a TID and reaches only that main
thread, so a launch-time process pin moves the *executor* and leaves the RT loop
roaming — and because the loop thread is created before any post-launch timer
can fire, no amount of retiming fixes it. The runtime verifier read the same
main thread and reported PASS, so the gap was invisible from both ends
(issue #343).

``controller_manager`` applies its own ``cpu_affinity`` / ``thread_priority``
parameters *from inside* that loop thread, which is the only place the right TID
is knowable. Feeding them is a pure launch-argument change: ``ur_control.launch.py``
declares ``update_rate_config_file`` and passes it as the first parameter file of
the node, so RTC hands it this generated file instead of the upstream default
(``<ur_robot_driver>/config/<ur_type>_update_rate.yaml``, which carries only
``update_rate``). No upstream patch, no third copy of the slot map.

The file is written into the session directory so a run's actual RT contract is
recoverable next to its logs.

Contract notes
--------------
* ``cpu_affinity`` takes a **logical CPU id**, not a slot index. It is resolved
  through :func:`rtc_tools.launch.pinning.resolve_logical_cpu`, i.e. by
  ``rt_common.sh::slot_to_logical_cpu`` — see the "no third mirror" section of
  :mod:`rtc_tools.launch.pinning`. An unresolvable slot omits the key rather
  than guessing a number (issue #163).
* ``update_rate`` is derived from the robot config's ``control_rate`` so the CM
  loop and the RTC controller cannot silently disagree; the two were never
  cross-checked before.
* ``lock_memory`` asks the node for ``mlockall``, which the runtime verifier
  confirms through the process's ``VmLck``.
"""

from __future__ import annotations

import os

import yaml

from rtc_tools.launch.pinning import resolve_logical_cpu

# Upstream ``controller_manager`` defaults ``thread_priority`` to 50; RTC's
# layout places the arm driver's loop below every in-process RT thread
# (rt_control 90 > rt_callback 70 > mpc 60/55 > arm 50), so the default is also
# the intended value. Named here because the runtime verifier checks the RT
# thread carries exactly this priority — the two must move together.
DEFAULT_THREAD_PRIORITY = 50

# Basename inside the session directory. Kept stable so a support request can
# ask for "the controller_manager_rt.yaml in the session dir".
CM_RT_PARAM_FILENAME = "controller_manager_rt.yaml"


def _yaml_scalar(value: bool | int) -> str:
    """Render one parameter value as a YAML scalar.

    Not ``yaml.safe_dump``: dumping a bare scalar produces a whole *document*
    (``true\\n...\\n``), whose end marker corrupts the file when spliced into a
    mapping. ``bool`` is checked first because it is a subclass of ``int`` — a
    ``lock_memory: 1`` would be a parameter-type mismatch at node start.
    """
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(int(value))


def control_rate_from_yaml(path: str) -> float:
    """Read ``control_rate`` (Hz) out of a ``/**: ros__parameters:`` config file.

    The robot's ``_base.yaml`` is the SSoT for the control rate; this reads it at
    launch-build time so the generated ``update_rate`` tracks it. Raises rather
    than defaulting: a wrong rate here desynchronises the CM loop from the RTC
    controller at every tick, which is far worse than a launch that refuses to
    start.
    """
    with open(path) as handle:
        data = yaml.safe_load(handle) or {}
    params = (data.get("/**", {}) or {}).get("ros__parameters", {}) or {}
    if "control_rate" not in params:
        raise KeyError(f"control_rate not found in {path}")
    return float(params["control_rate"])


def update_rate_from_control_rate(control_rate: float | int | str) -> int:
    """Convert a ``control_rate`` in Hz to ``controller_manager``'s integer rate.

    ``control_rate`` is a float in YAML (``500.0``) while ``update_rate`` is an
    ``int`` parameter. A fractional rate cannot be represented and would be
    silently truncated by the parameter server, so it is rejected instead.
    """
    rate = float(control_rate)
    if not rate.is_integer() or rate <= 0:
        raise ValueError(
            f"control_rate={control_rate!r} cannot be expressed as controller_manager's "
            "integer update_rate (must be a positive whole number of Hz)"
        )
    return int(rate)


def build_cm_rt_params(
    *,
    control_rate: float | int | str,
    slot: int | None = None,
    thread_priority: int = DEFAULT_THREAD_PRIORITY,
    lock_memory: bool = True,
) -> dict:
    """Build the ``controller_manager`` parameter dict, resolving ``slot`` if given.

    ``slot=None`` omits ``cpu_affinity`` entirely — the shape used when
    ``use_cpu_affinity:=false``, and the fallback when the slot cannot be
    resolved to a logical CPU. The remaining keys are scheduling and memory
    settings, not affinity, so they stay in both shapes.
    """
    params: dict = {"update_rate": update_rate_from_control_rate(control_rate)}

    if slot is not None:
        logical_cpu = resolve_logical_cpu(slot)
        if logical_cpu is not None:
            params["cpu_affinity"] = logical_cpu

    params["thread_priority"] = int(thread_priority)
    params["lock_memory"] = bool(lock_memory)
    return params


def write_cm_rt_param_file(
    dest_dir: str,
    *,
    control_rate: float | int | str,
    slot: int | None = None,
    thread_priority: int = DEFAULT_THREAD_PRIORITY,
    lock_memory: bool = True,
    filename: str = CM_RT_PARAM_FILENAME,
) -> tuple[str, dict]:
    """Write the parameter file and return ``(path, params)``.

    ``params`` is returned so the caller can log what was actually applied —
    specifically whether ``cpu_affinity`` made it in, which is the one key that
    can drop out (unresolvable slot, or affinity disabled).

    Write errors propagate. The session directory is created by the launch file
    itself and already holds the run's logs, so a failure here means the run has
    no usable output either; falling back to the upstream default file would
    quietly restore the unpinned RT loop this file exists to fix.
    """
    params = build_cm_rt_params(
        control_rate=control_rate,
        slot=slot,
        thread_priority=thread_priority,
        lock_memory=lock_memory,
    )

    lines = [
        "# controller_manager RT parameters — GENERATED, do not edit.",
        "# Written per run by rtc_tools.launch.cm_rt_params; consumed by",
        "# ur_control.launch.py's update_rate_config_file argument (issue #343).",
        "#",
        f"# update_rate mirrors control_rate from the robot config ({params['update_rate']} Hz).",
    ]
    if "cpu_affinity" in params:
        lines.append(
            f"# cpu_affinity is a LOGICAL cpu id, resolved from slot {slot} by"
            " rt_common.sh::slot_to_logical_cpu."
        )
    else:
        lines.append(
            "# cpu_affinity is absent: affinity disabled, or the slot could not be"
            " resolved to a logical cpu (pinning is skipped, never guessed)."
        )
    lines.append("controller_manager:")
    lines.append("  ros__parameters:")
    for key, value in params.items():
        lines.append(f"    {key}: {_yaml_scalar(value)}")

    path = os.path.join(dest_dir, filename)
    with open(path, "w") as handle:
        handle.write("\n".join(lines) + "\n")
    return path, params
