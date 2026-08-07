"""Value oracle for the Python side of the generated thread layout.

**This test does not read the C++ side, and does not claim to.** It used to say
it caught "C++ drift", which was never true: it imports
``rtc_tools.launch.thread_layout`` and compares against the ``_EXPECTED`` table
below — no subprocess, no C++ parsing — so a C++-only change passed it (issue
#153 M1 re-measured this at HEAD ``f7bbf1e4``).

Cross-language drift is now caught by the generator instead. All three
encodings (C++ constants, shell helpers, this Python mirror) are generated from
``repo_scripts/config/thread_layout.yaml``, and::

    python3 repo_scripts/scripts/gen_thread_layout.py --check

fails if any checked-in artifact is stale or hand-edited. That runs in CI and
as ``repo_scripts``' ``test_gen_thread_layout`` test.

What ``--check`` *cannot* catch is a wrong manifest or a generator bug: a bad
table matches itself in all three languages. That is this file's job. The
values below are deliberately **hand-written literals**, never derived from the
manifest — turning them into generated values would make the test compare a
table against itself and remove every mutation it can detect. The C++ and shell
sides keep their own literal oracles for the same reason
(``rtc_base/test/test_mpc_thread_config.cpp``,
``repo_scripts/test/test_rt_common.sh``).
"""

from __future__ import annotations

import pytest

from rtc_tools.launch.thread_layout import (
    ThreadLayout,
    get_arm_driver_core,
    get_hand_driver_core,
    get_physical_cpu_count,
    get_rt_callback_core,
    get_sim_core,
    get_viewer_core,
    select_thread_layout,
)

# Per-tier expected mapping — hand-written on purpose (see the module docstring).
# These are the same values the C++ kXxxConfigNCore constants and the shell
# get_{arm,hand}_driver_slot helpers carry; each language pins them separately so
# a single wrong manifest entry cannot pass everywhere at once.
_EXPECTED: dict[int, ThreadLayout] = {
    4: ThreadLayout(
        arm_driver_core=0,
        hand_driver_core=0,
        sim_thread_core=-1,
        viewer_core=-1,
        rt_callback_core=2,
    ),
    6: ThreadLayout(
        arm_driver_core=4,
        hand_driver_core=4,
        sim_thread_core=-1,
        viewer_core=-1,
        rt_callback_core=2,
    ),
    8: ThreadLayout(
        arm_driver_core=4,
        hand_driver_core=5,
        sim_thread_core=-1,
        viewer_core=-1,
        rt_callback_core=2,
    ),
    10: ThreadLayout(
        arm_driver_core=4,
        hand_driver_core=5,
        sim_thread_core=-1,
        viewer_core=-1,
        rt_callback_core=2,
    ),
    12: ThreadLayout(
        arm_driver_core=4,
        hand_driver_core=5,
        sim_thread_core=-1,
        viewer_core=-1,
        rt_callback_core=2,
    ),
    14: ThreadLayout(
        arm_driver_core=4,
        hand_driver_core=5,
        sim_thread_core=-1,
        viewer_core=-1,
        rt_callback_core=2,
    ),
    16: ThreadLayout(
        arm_driver_core=4,
        hand_driver_core=5,
        sim_thread_core=-1,
        viewer_core=-1,
        rt_callback_core=2,
    ),
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
        assert get_rt_callback_core(ncpu) == expected.rt_callback_core


def test_physical_cpu_count_is_positive() -> None:
    """The runtime probe returns a sane positive count on the test host."""
    assert get_physical_cpu_count() >= 1
