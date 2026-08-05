"""Python mirror of ``rtc::SelectThreadConfigs`` core-tier dispatch.

The tier table itself is GENERATED into :mod:`rtc_tools.launch.thread_layout_generated`
from ``repo_scripts/config/thread_layout.yaml`` -- the same manifest that
generates the C++ tier constants and the shell helpers (issue #153 M1). This
module is the hand-written adapter: it keeps the stable, launch-facing API
(:class:`ThreadLayout` and the ``get_*_core`` accessors) on top of that table.

Launch files need the same per-tier decisions the C++ side makes, for
``taskset`` pinning of external driver and simulator processes.

Drift between the three languages is caught by the generator's ``--check``
mode, not by a test in this package::

    python3 repo_scripts/scripts/gen_thread_layout.py --check

``test/test_thread_layout.py`` is a different sensor: it pins the expected
per-tier values as hand-written literals, so it catches a *generator* bug that
``--check`` cannot see (a wrong table matches itself in all three languages).

``cpu_core == -1`` is a sentinel meaning "do not pin" -- the launch script
treats it as a no-op (skip the taskset call entirely). ``rt_callback_core`` is
exposed so launch files can co-pin the DDS receive thread to the same core as
the ``rt_callback`` RT thread for cache locality (layout v4 invariant).

Every value here is a *slot index*, not a kernel logical CPU id; resolving a
slot to a logical CPU is the launch helper's job (``pin_*_to_slot``).
"""

from __future__ import annotations

import os
import subprocess
from dataclasses import dataclass

from rtc_tools.launch.thread_layout_generated import ROLE_SPECS, tier_for_cores


@dataclass(frozen=True)
class ThreadLayout:
    arm_driver_core: int
    hand_driver_core: int
    sim_thread_core: int
    viewer_core: int
    rt_callback_core: int


def get_physical_cpu_count() -> int:
    """Return the physical (non-SMT) CPU count.

    Mirrors ``rtc::GetPhysicalCpuCount`` and ``rt_common.sh::get_physical_cores``.
    Prefers ``lscpu -p=Core,Socket`` and applies the same row filter as the
    shell helper: drop ``#``-prefixed headers and blank lines, then count
    unique ``(Core,Socket)`` tuples. Falls back to ``os.cpu_count()`` only if
    lscpu is unavailable. The shell and Python paths must agree because both
    feed the same tier selection.
    """
    try:
        result = subprocess.run(
            ["lscpu", "-p=Core,Socket"],
            capture_output=True,
            text=True,
            check=False,
            timeout=2.0,
        )
        if result.returncode == 0:
            unique_cores = set()
            for line in result.stdout.splitlines():
                if line.startswith("#") or not line.strip():
                    continue
                unique_cores.add(line.strip())
            if unique_cores:
                return len(unique_cores)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass

    return os.cpu_count() or 1


def select_thread_layout(physical_cores: int | None = None) -> ThreadLayout:
    """Return the ``ThreadLayout`` for the given (or detected) physical core count.

    Mirrors ``rtc::SelectThreadConfigsForCoreCount()``; both read the same
    manifest, so a tier change only has to be made once.
    """
    if physical_cores is None:
        physical_cores = get_physical_cpu_count()

    specs = ROLE_SPECS[tier_for_cores(physical_cores)]
    return ThreadLayout(
        arm_driver_core=specs["arm_driver"].slot,
        hand_driver_core=specs["hand_driver"].slot,
        sim_thread_core=specs["sim_thread"].slot,
        viewer_core=specs["viewer"].slot,
        rt_callback_core=specs["rt_callback"].slot,
    )


def get_arm_driver_core(physical_cores: int | None = None) -> int:
    """Core index for the external arm driver process (``taskset`` target)."""
    return select_thread_layout(physical_cores).arm_driver_core


def get_hand_driver_core(physical_cores: int | None = None) -> int:
    """Core index for the external hand driver process (``taskset`` target)."""
    return select_thread_layout(physical_cores).hand_driver_core


def get_sim_core(physical_cores: int | None = None) -> int:
    """Core index for the MuJoCo physics thread; ``-1`` means no pinning."""
    return select_thread_layout(physical_cores).sim_thread_core


def get_viewer_core(physical_cores: int | None = None) -> int:
    """Core index for the GLFW viewer thread; ``-1`` means no pinning."""
    return select_thread_layout(physical_cores).viewer_core


def get_rt_callback_core(physical_cores: int | None = None) -> int:
    """Core index for the ``rt_callback`` RT thread; DDS recv co-pins here."""
    return select_thread_layout(physical_cores).rt_callback_core
