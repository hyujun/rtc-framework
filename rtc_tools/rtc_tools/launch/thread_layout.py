"""Python mirror of ``rtc::SelectThreadConfigs`` core-tier dispatch.

C++ SSoT lives in ``rtc_base/include/rtc_base/threading/thread_config.hpp`` and
``thread_utils.hpp::SelectThreadConfigs``. Launch files (Python) need to make
the same per-tier decisions for ``taskset`` pinning of external driver and
simulator processes, so this module re-encodes the same tier breakpoints and
returns the matching ``cpu_core`` values.

Drift between this module and the C++ SSoT is caught by
``test/test_thread_layout.py`` — the test fixture pins the expected mapping
per tier; any divergence (here or in ``thread_config.hpp``) breaks the test.

Tier source (layout v4.1):
  - 4-core fallback  : arm=0,  hand=0,  sim=-1, viewer=-1, rt_callback=2
  - 6-core (degraded): arm=4,  hand=4,  sim=-1, viewer=-1, rt_callback=2
  - 8-core           : arm=4,  hand=5,  sim=-1, viewer=-1, rt_callback=2
  - 10-core          : arm=5,  hand=6,  sim=-1, viewer=-1, rt_callback=2
  - 12-core          : arm=6,  hand=7,  sim=-1, viewer=-1, rt_callback=2
  - 14-core          : arm=6,  hand=7,  sim=-1, viewer=-1, rt_callback=2
  - 16-core+         : arm=6,  hand=7,  sim=-1, viewer=-1, rt_callback=2

``cpu_core == -1`` is a sentinel meaning "do not pin" — the launch script
treats it as a no-op (skip taskset call entirely). ``rt_callback_core`` is
exposed so launch files can co-pin the DDS receive thread to the same core
as the ``rt_callback`` RT thread for cache locality (layout v4 invariant).
"""

from __future__ import annotations

import os
import subprocess
from dataclasses import dataclass


@dataclass(frozen=True)
class ThreadLayout:
    arm_driver_core: int
    hand_driver_core: int
    sim_thread_core: int
    viewer_core: int
    rt_callback_core: int


_TIERS: tuple[tuple[int, ThreadLayout], ...] = (
    (
        16,
        ThreadLayout(
            arm_driver_core=6,
            hand_driver_core=7,
            sim_thread_core=-1,
            viewer_core=-1,
            rt_callback_core=2,
        ),
    ),
    (
        14,
        ThreadLayout(
            arm_driver_core=6,
            hand_driver_core=7,
            sim_thread_core=-1,
            viewer_core=-1,
            rt_callback_core=2,
        ),
    ),
    (
        12,
        ThreadLayout(
            arm_driver_core=6,
            hand_driver_core=7,
            sim_thread_core=-1,
            viewer_core=-1,
            rt_callback_core=2,
        ),
    ),
    (
        10,
        ThreadLayout(
            arm_driver_core=5,
            hand_driver_core=6,
            sim_thread_core=-1,
            viewer_core=-1,
            rt_callback_core=2,
        ),
    ),
    (
        8,
        ThreadLayout(
            arm_driver_core=4,
            hand_driver_core=5,
            sim_thread_core=-1,
            viewer_core=-1,
            rt_callback_core=2,
        ),
    ),
    (
        6,
        ThreadLayout(
            arm_driver_core=4,
            hand_driver_core=4,
            sim_thread_core=-1,
            viewer_core=-1,
            rt_callback_core=2,
        ),
    ),
)
_FALLBACK_4CORE = ThreadLayout(
    arm_driver_core=0,
    hand_driver_core=0,
    sim_thread_core=-1,
    viewer_core=-1,
    rt_callback_core=2,
)


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

    Mirrors ``rtc::SelectThreadConfigs()`` tier breakpoints. Any change to the
    C++ tier table must be reflected here (and caught by the drift test).
    """
    if physical_cores is None:
        physical_cores = get_physical_cpu_count()

    for threshold, layout in _TIERS:
        if physical_cores >= threshold:
            return layout
    return _FALLBACK_4CORE


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
