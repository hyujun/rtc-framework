"""Drift test: Python ``select_thread_layout`` ≡ C++ ``SelectThreadConfigs``.

The expected per-tier values are hard-coded here as the *contract* between the
Python helper and the C++ SSoT. If either side changes a tier breakpoint or a
``cpu_core`` value, this test fails — preventing silent launch-time drift.

The C++ SSoT is ``rtc_base/include/rtc_base/threading/thread_config.hpp`` plus
``rtc_base/include/rtc_base/threading/thread_utils.hpp::SelectThreadConfigs``.
"""

from __future__ import annotations

import pytest

from rtc_tools.launch.thread_layout import (
    ThreadLayout,
    get_arm_driver_core,
    get_hand_driver_core,
    get_physical_cpu_count,
    get_sim_core,
    get_viewer_core,
    select_thread_layout,
)

# Per-tier expected mapping. Mirrors the table at the top of
# rtc_tools/launch/thread_layout.py and the C++ kXxxConfigNCore constants.
_EXPECTED: dict[int, ThreadLayout] = {
    4: ThreadLayout(arm_driver_core=0, hand_driver_core=0, sim_thread_core=-1, viewer_core=-1),
    6: ThreadLayout(arm_driver_core=1, hand_driver_core=1, sim_thread_core=-1, viewer_core=-1),
    8: ThreadLayout(arm_driver_core=6, hand_driver_core=5, sim_thread_core=7, viewer_core=-1),
    10: ThreadLayout(arm_driver_core=7, hand_driver_core=6, sim_thread_core=9, viewer_core=-1),
    12: ThreadLayout(arm_driver_core=8, hand_driver_core=7, sim_thread_core=10, viewer_core=-1),
    14: ThreadLayout(arm_driver_core=8, hand_driver_core=7, sim_thread_core=10, viewer_core=-1),
    16: ThreadLayout(arm_driver_core=13, hand_driver_core=12, sim_thread_core=15, viewer_core=-1),
}


@pytest.mark.parametrize("ncpu, expected", sorted(_EXPECTED.items()))
def test_tier_layout_matches_cpp_ssot(ncpu: int, expected: ThreadLayout) -> None:
    """Each canonical tier returns the expected ``ThreadLayout``."""
    assert select_thread_layout(ncpu) == expected


def test_below_minimum_tier_uses_4core_fallback() -> None:
    """1–3 physical cores degrade to the 4-core fallback layout."""
    fallback = _EXPECTED[4]
    for ncpu in (1, 2, 3):
        assert select_thread_layout(ncpu) == fallback


def test_intermediate_cores_round_down_to_lower_tier() -> None:
    """A 9-core box uses the 8-core layout, 11 uses 10, 13 uses 12, 15 uses 14."""
    assert select_thread_layout(9) == _EXPECTED[8]
    assert select_thread_layout(11) == _EXPECTED[10]
    assert select_thread_layout(13) == _EXPECTED[12]
    assert select_thread_layout(15) == _EXPECTED[14]


def test_high_core_count_clamps_to_16core() -> None:
    """24-/32-/64-core boxes use the 16-core layout (highest tier)."""
    for ncpu in (24, 32, 64):
        assert select_thread_layout(ncpu) == _EXPECTED[16]


def test_helper_accessors_agree_with_layout() -> None:
    """The ``get_*_core`` accessors return the matching fields."""
    for ncpu, expected in _EXPECTED.items():
        assert get_arm_driver_core(ncpu) == expected.arm_driver_core
        assert get_hand_driver_core(ncpu) == expected.hand_driver_core
        assert get_sim_core(ncpu) == expected.sim_thread_core
        assert get_viewer_core(ncpu) == expected.viewer_core


def test_physical_cpu_count_is_positive() -> None:
    """The runtime probe returns a sane positive count on the test host."""
    assert get_physical_cpu_count() >= 1
