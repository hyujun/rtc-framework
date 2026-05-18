"""Python mirror of ``rtc::SelectThreadConfigs`` core-tier dispatch.

C++ SSoT lives in ``rtc_base/include/rtc_base/threading/thread_config.hpp`` and
``thread_utils.hpp::SelectThreadConfigs``. Launch files (Python) need to make
the same per-tier decisions for ``taskset`` pinning of external driver and
simulator processes, so this module re-encodes the same tier breakpoints and
returns the matching ``cpu_core`` values.

Drift between this module and the C++ SSoT is caught by
``test/test_thread_layout.py`` — the test fixture pins the expected mapping
per tier; any divergence (here or in ``thread_config.hpp``) breaks the test.

Tier source (Phase 5 layout v3):
  - 4-core fallback  : arm=0, hand=0, sim=-1, viewer=-1
  - 6-core (degraded): arm=1, hand=1, sim=-1, viewer=-1
  - 8-core           : arm=6, hand=5, sim=7,  viewer=-1
  - 10-core          : arm=7, hand=6, sim=9,  viewer=-1
  - 12-core          : arm=8, hand=7, sim=10, viewer=-1
  - 14-core          : arm=8, hand=7, sim=10, viewer=-1
  - 16-core+         : arm=13, hand=12, sim=15, viewer=-1

``cpu_core == -1`` is a sentinel meaning "do not pin" — the launch script
treats it as a no-op (skip taskset call entirely).
"""

from __future__ import annotations

import subprocess
from dataclasses import dataclass


@dataclass(frozen=True)
class ThreadLayout:
    arm_driver_core: int
    hand_driver_core: int
    sim_thread_core: int
    viewer_core: int


_TIERS: tuple[tuple[int, ThreadLayout], ...] = (
    (
        16,
        ThreadLayout(arm_driver_core=13, hand_driver_core=12, sim_thread_core=15, viewer_core=-1),
    ),
    (14, ThreadLayout(arm_driver_core=8, hand_driver_core=7, sim_thread_core=10, viewer_core=-1)),
    (12, ThreadLayout(arm_driver_core=8, hand_driver_core=7, sim_thread_core=10, viewer_core=-1)),
    (10, ThreadLayout(arm_driver_core=7, hand_driver_core=6, sim_thread_core=9, viewer_core=-1)),
    (8, ThreadLayout(arm_driver_core=6, hand_driver_core=5, sim_thread_core=7, viewer_core=-1)),
    (6, ThreadLayout(arm_driver_core=1, hand_driver_core=1, sim_thread_core=-1, viewer_core=-1)),
)
_FALLBACK_4CORE = ThreadLayout(
    arm_driver_core=0, hand_driver_core=0, sim_thread_core=-1, viewer_core=-1
)


def get_physical_cpu_count() -> int:
    """Return the physical (non-SMT) CPU count.

    Mirrors ``rtc::GetPhysicalCpuCount`` and ``rt_common.sh::get_physical_cores``.
    Prefers ``lscpu -p=Core,Socket`` (matches the shell helper); falls back to
    ``os.cpu_count()`` only if lscpu is unavailable. The shell and Python paths
    must agree because both feed the same tier selection.
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

    import os

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
