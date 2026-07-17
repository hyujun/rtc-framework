"""Drift test: launch pinning resolves *slot* -> *logical CPU* via rt_common.sh.

``thread_layout`` returns slot indices; ``rtc_tools.launch.pinning`` must hand
those slots to ``rt_common.sh::slot_to_logical_cpu`` before ``taskset``, exactly
as C++ ``ApplyThreadConfig`` does through ``SlotToLogicalCpu``. Passing the raw
slot to ``taskset`` is issue #163 — on a hybrid NUC 13 Pro it pins the driver
and DDS threads onto the RT cluster's own cores.

This test builds mock sysfs topologies and executes the *actual* bash prelude
that ``pinning`` emits, then asserts the resolved logical CPU matches the
topology's known ``PHYSICAL_CORE_SLOTS`` mapping. It therefore locks both sides
at once: if the Python side stops sending the slot through the translation, or
the bash mapping drifts from the C++ ``physical_core_slots`` ordering, the
concrete per-tier expectations below fail.

The two existing drift axes are complementary:
  * ``test_thread_layout.py``            — tier -> slot   (vs thread_config.hpp)
  * ``repo_scripts/test_rt_common.sh``   — slot -> logical (vs cpu_topology.hpp)
This test ties them together at the launch boundary that consumes both.
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
from pathlib import Path

import pytest

from rtc_tools.launch import pinning
from rtc_tools.launch.thread_layout import select_thread_layout

# rt_common.sh from the source tree (this file: rtc_tools/test/ -> repo root).
_REPO_ROOT = Path(__file__).resolve().parents[2]
_RT_COMMON = _REPO_ROOT / "repo_scripts" / "scripts" / "lib" / "rt_common.sh"

_HAVE_DEPS = _RT_COMMON.is_file() and shutil.which("bash") is not None
_skip_no_deps = pytest.mark.skipif(
    not _HAVE_DEPS, reason="bash and repo_scripts/rt_common.sh required"
)


def _add_cpu(root: Path, cpu: int, core_id: int, max_khz: int) -> None:
    d = root / "devices" / "system" / "cpu" / f"cpu{cpu}"
    (d / "topology").mkdir(parents=True, exist_ok=True)
    (d / "cpufreq").mkdir(parents=True, exist_ok=True)
    (d / "topology" / "physical_package_id").write_text("0\n")
    (d / "topology" / "core_id").write_text(f"{core_id}\n")
    (d / "cpufreq" / "cpuinfo_max_freq").write_text(f"{max_khz}\n")


def _build_nuc13(root: Path) -> None:
    """Raptor Lake-P: 4 P-cores (HT, cpu0-7) + 8 E-cores (cpu8-15).

    Matches repo_scripts test_rt_common.sh::test_physical_core_slots_nuc13_hybrid;
    PHYSICAL_CORE_SLOTS == 0 2 4 6 8 9 10 11 12 13 14 15.
    """
    for i in range(4):
        _add_cpu(root, i * 2, i, 5000000)
        _add_cpu(root, i * 2 + 1, i, 5000000)
    for i in range(8):
        _add_cpu(root, 8 + i, 4 + i, 3800000)
    types = root / "devices" / "system" / "cpu" / "types"
    (types / "intel_core").mkdir(parents=True, exist_ok=True)
    (types / "intel_atom").mkdir(parents=True, exist_ok=True)
    (types / "intel_core" / "cpus").write_text("0,1,2,3,4,5,6,7")
    (types / "intel_atom" / "cpus").write_text("8,9,10,11,12,13,14,15")
    (root / "cpuinfo").write_text(
        "processor\t: 0\nvendor_id\t: GenuineIntel\n"
        "cpu family\t: 6\nmodel\t\t: 186\nflags\t\t: fpu tsc hybrid pae\n"
    )


def _build_ryzen_5600x(root: Path) -> None:
    """AMD Ryzen 5 5600X: 6C/12T, siblings (0,6)(1,7)... → identity slots.

    Non-hybrid SMT-on where PHYSICAL_CORE_SLOTS == 0 1 2 3 4 5. Represents this
    repo's dev box: the fix must be a no-op here (slot == logical).
    """
    for cpu in range(12):
        _add_cpu(root, cpu, cpu % 6, 4600000)
    (root / "cpuinfo").write_text(
        "processor\t: 0\nvendor_id\t: AuthenticAMD\nflags\t\t: fpu tsc pae\n"
    )


def _build_smt_off_8core(root: Path) -> None:
    """8 physical cores, no SMT → identity PHYSICAL_CORE_SLOTS == 0..7."""
    for cpu in range(8):
        _add_cpu(root, cpu, cpu, 3600000)
    (root / "cpuinfo").write_text(
        "processor\t: 0\nvendor_id\t: GenuineIntel\nflags\t\t: fpu tsc pae\n"
    )


# (label, builder, physical_core_count, {slot: expected_logical_cpu})
_FIXTURES = [
    ("nuc13_4p8e", _build_nuc13, 12, {0: 0, 1: 2, 2: 4, 3: 6, 6: 10, 7: 11}),
    ("ryzen_5600x", _build_ryzen_5600x, 6, {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5}),
    ("smt_off_8core", _build_smt_off_8core, 8, {0: 0, 2: 2, 4: 4, 5: 5, 7: 7}),
]


def _resolve_via_launch(slot: int, sysfs_root: Path, monkeypatch) -> int:
    """Run the exact bash prelude ``pinning`` emits and return the resolved CPU.

    Points ``rt_common_path`` at the source tree and ``RTC_SYSFS_ROOT`` at the
    mock so no installed workspace or real hardware is touched.
    """
    monkeypatch.setattr(pinning, "rt_common_path", lambda: str(_RT_COMMON))
    snippet = pinning._resolve_slot_prelude("drift_probe", slot) + 'echo "CPU=$CPU"'
    result = subprocess.run(
        ["bash", "-c", snippet],
        capture_output=True,
        text=True,
        env={
            "PATH": os.environ.get("PATH", ""),
            "RTC_SYSFS_ROOT": str(sysfs_root),
            "RTC_PROC_CPUINFO": str(sysfs_root / "cpuinfo"),
            "RTC_HYBRID_SANITY": "0",
        },
    )
    assert result.returncode == 0, f"prelude failed: {result.stderr}"
    for line in result.stdout.splitlines():
        if line.startswith("CPU="):
            return int(line[len("CPU=") :])
    raise AssertionError(f"no CPU= line in output:\n{result.stdout}")


@_skip_no_deps
@pytest.mark.parametrize("label, builder, ncpu, expected", _FIXTURES)
def test_launch_pins_resolve_slot_to_logical(
    label, builder, ncpu, expected, tmp_path, monkeypatch
):
    """Each fixture: the launch prelude resolves every slot to its logical CPU."""
    root = tmp_path / label
    builder(root)
    for slot, want in expected.items():
        got = _resolve_via_launch(slot, root, monkeypatch)
        assert got == want, f"{label}: slot {slot} -> logical {got}, expected {want}"


@_skip_no_deps
def test_nuc13_actual_layout_slots_land_off_the_rt_cluster(tmp_path, monkeypatch):
    """Regression guard for #163 on the real control PC (NUC 13 Pro, 12-core).

    The arm/hand driver and rt_callback-copin slots must resolve to E-cores /
    the P2 core — NOT the raw slot values, which are rt_control (2) and
    mpc_main's P-core + sibling (6, 7).
    """
    root = tmp_path / "nuc13"
    _build_nuc13(root)
    layout = select_thread_layout(12)

    arm = _resolve_via_launch(layout.arm_driver_core, root, monkeypatch)
    hand = _resolve_via_launch(layout.hand_driver_core, root, monkeypatch)
    dds = _resolve_via_launch(layout.rt_callback_core, root, monkeypatch)

    # Documented NUC13 mapping (issue #163).
    assert (arm, hand, dds) == (10, 11, 4)
    # The bug would have pinned to the raw slots — assert we did NOT.
    assert arm != layout.arm_driver_core  # slot 6 (mpc_main P-core)
    assert hand != layout.hand_driver_core  # slot 7 (mpc_main HT sibling)
    assert dds != layout.rt_callback_core  # slot 2 (rt_control P-core)


@_skip_no_deps
def test_sentinel_slot_is_left_unpinned(tmp_path, monkeypatch):
    """slot -1 (sim/viewer no-pin sentinel) resolves to -1, never a real CPU."""
    root = tmp_path / "nuc13"
    _build_nuc13(root)
    assert _resolve_via_launch(-1, root, monkeypatch) == -1


# ── DDS-pin RTC-owned thread filter (issue #163 Phase 3) ─────────────────────

_THREAD_CONFIG_HPP = (
    _REPO_ROOT / "rtc_base" / "include" / "rtc_base" / "threading" / "thread_config.hpp"
)


def _thread_config_names() -> set[str]:
    """Every ``.name = "..."`` value declared in thread_config.hpp."""
    text = _THREAD_CONFIG_HPP.read_text()
    return set(re.findall(r'\.name = "([^"]+)"', text))


def test_rtc_owned_names_mirror_thread_config_hpp():
    """The DDS-pin exclusion set is an exact mirror of the C++ thread names.

    If a thread is renamed (or added) in thread_config.hpp without updating
    RTC_OWNED_THREAD_NAMES, pin_dds_threads_to_slot could yank it off its
    dedicated core — the exact class of bug as #163. Lock the two together.
    """
    assert _thread_config_names() == pinning.RTC_OWNED_THREAD_NAMES


def test_dds_pin_snippet_excludes_every_rtc_thread():
    """The rendered DDS-pin command carries the full exclusion list and applies it.

    nrt_logging / nrt_callback are SCHED_OTHER; the pre-#163 FIFO-only filter let
    them through. Assert the name filter is present and complete so they (and the
    RT threads) are skipped.
    """
    action = pinning.pin_dds_threads_to_slot(
        "integrated_rt_controller", "integrated_rt_controller", 2
    )
    snippet = action.cmd[2][0].text
    # The exclusion is applied via a case-match against $COMM.
    assert 'case "$RTC_OWNED" in *" $COMM "*) continue' in snippet
    # Every RTC-owned name — especially the SCHED_OTHER nrt_* pair — is listed.
    for name in pinning.RTC_OWNED_THREAD_NAMES:
        assert f" {name} " in snippet, f"{name} missing from RTC_OWNED list"


# ── Shield adopt action (issue #151) ─────────────────────────────────────────


def test_adopt_snippet_resolves_pid_and_calls_cpu_shield():
    """The adopt action pgreps the PID and delegates the cset move to cpu_shield.sh.

    The cset knowledge lives in cpu_shield.sh (SSoT); the launch helper only
    resolves the newest matching PID and hands it to ``adopt``. Assert both.
    """
    action = pinning.adopt_process_into_shield(
        "integrated_rt_controller", "integrated_rt_controller"
    )
    snippet = action.cmd[2][0].text
    assert 'PID=$(pgrep -nf "integrated_rt_controller")' in snippet
    assert '"$SHIELD" adopt "$PID"' in snippet
    # Guards: missing PID / missing script / password-required sudo all exit 0.
    assert "shield adopt skipped" in snippet
    assert action.cmd[2][0].text.count("exit 0") >= 2


def test_adopt_gated_toggles_use_cpu_affinity_condition():
    """gated=True carries the use_cpu_affinity IfCondition; gated=False drops it.

    A caller gating ACTIVATE on the adopt action's OnProcessExit needs it to fire
    (and exit) even when use_cpu_affinity:=false, so gated=False must leave the
    action unconditional.
    """
    assert pinning.adopt_process_into_shield("x", "x", gated=True).condition is not None
    assert pinning.adopt_process_into_shield("x", "x", gated=False).condition is None


def test_adopt_rejects_shell_injection_via_label():
    """Embedded double quotes in interpolated fields are rejected (defense in depth)."""
    with pytest.raises(ValueError):
        pinning.adopt_process_into_shield('bad"name', "grep")
    with pytest.raises(ValueError):
        pinning.adopt_process_into_shield("label", 'gr"ep')
