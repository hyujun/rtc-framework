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


@pytest.fixture
def _stub_script_paths(monkeypatch):
    """Render snippet-shape assertions without an installed ``repo_scripts``.

    ``rt_common_path`` / ``cpu_shield_path`` resolve the installed scripts through
    ``ament_index`` (``get_package_share_directory("repo_scripts")``). The CI
    Python-test job installs only ``rtc_tools`` / ``rtc_msgs`` / ``rtc_digital_twin``,
    so that lookup raises ``PackageNotFoundError``. Tests that only assert on the
    rendered bash *structure* (not the resolved path) stub both helpers; the
    production launch always runs with ``repo_scripts`` installed.
    """
    monkeypatch.setattr(pinning, "rt_common_path", lambda: "/opt/rtc/rt_common.sh")
    monkeypatch.setattr(pinning, "cpu_shield_path", lambda: "/opt/rtc/cpu_shield.sh")


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


def test_dds_pin_snippet_excludes_every_rtc_thread(_stub_script_paths):
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


# ── Process-pin thread scope: all_threads flag (issue #245) ──────────────────


def test_process_pin_defaults_to_main_thread_only(_stub_script_paths):
    """Default (arm driver) pins the main thread only — taskset -cp, never -a.

    Widening the UR arm driver to every thread would sweep its RTDE workers onto
    one E-core; that trade-off stays gated on NUC13 measurements (#163 Phase 4),
    so the default must remain main-thread-only.
    """
    snippet = (
        pinning.pin_process_to_slot("UR ros2_control_node", "ros2_control_node", 6).cmd[2][0].text
    )
    assert 'taskset -cp "$CPU" "$PID"' in snippet
    assert "-acp" not in snippet
    assert "main thread" in snippet


def test_process_pin_all_threads_uses_taskset_dash_a(_stub_script_paths):
    """all_threads=True (hand driver) sweeps every TID — taskset -acp.

    The hand CommLoop RT thread + failure detector self-set cpu_core=-1 and are
    created in on_activate before the pin timer fires, so only -a reaches them
    (#245). Assert the flag flips the taskset scope.
    """
    snippet = (
        pinning.pin_process_to_slot("udp_hand_node", "udp_hand_node", 7, all_threads=True)
        .cmd[2][0]
        .text
    )
    assert 'taskset -acp "$CPU" "$PID"' in snippet
    assert "all threads" in snippet


# ── Shield adopt action (issue #151) ─────────────────────────────────────────


def test_adopt_snippet_resolves_pid_and_calls_cpu_shield(_stub_script_paths):
    """The adopt action pgreps the PID and delegates the cset move to cpu_shield.sh.

    The cset knowledge lives in cpu_shield.sh (SSoT); the launch helper only
    resolves the newest matching PID and hands it to ``adopt``. Assert both.
    """
    action = pinning.adopt_process_into_shield(
        "integrated_rt_controller", "integrated_rt_controller"
    )
    snippet = action.cmd[2][0].text
    # comm-exact match (pgrep -nx) on the 15-char comm — never the command line,
    # so it cannot resolve to the launch's own wrapper bash / forks (#151).
    assert 'pgrep -nx "integrated_rt_c"' in snippet
    assert '"$SHIELD" adopt "$PID"' in snippet
    # Guards: missing PID / missing script / password-required sudo all exit 0.
    assert "shield adopt skipped" in snippet
    assert action.cmd[2][0].text.count("exit 0") >= 2


def test_adopt_no_sudo_distinguishes_active_shield_from_shield_off(_stub_script_paths):
    """No passwordless sudo splits into a loud ERROR vs a benign WARNING (#151).

    When adopt cannot run and a cset shield is ACTIVE, the CM is stuck in the
    "system" cpuset and its RT self-pin EINVALs — that is a hard failure the log
    must surface, not the harmless "shield off, full affinity" skip. Assert the
    branch detects an active cset "user" cpuset and escalates to ERROR, while the
    fall-through stays a WARNING.
    """
    snippet = (
        pinning.adopt_process_into_shield("integrated_rt_controller", "integrated_rt_controller")
        .cmd[2][0]
        .text
    )
    # Active-shield detection reuses the status-safe (no-sudo) cset probe.
    assert 'cset shield -s 2>/dev/null | grep -q "user"' in snippet
    # Active shield + no sudo → loud ERROR naming the EINVAL failure mode.
    assert "[RT] ERROR:" in snippet
    assert "setaffinity EINVAL" in snippet
    # No active shield → benign WARNING, CM keeps full affinity.
    assert "CM keeps full affinity" in snippet


def test_adopt_gated_toggles_use_cpu_affinity_condition(_stub_script_paths):
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


# ── pgrep self-match guard (issue #151) ──────────────────────────────────────


def test_comm_pattern_truncates_to_15_chars():
    # comm is TASK_COMM_LEN-1 = 15 chars: the node's /proc/<pid>/comm.
    assert pinning._comm_pattern("integrated_rt_controller") == "integrated_rt_c"
    assert pinning._comm_pattern("ros2_control_node") == "ros2_control_no"
    assert pinning._comm_pattern("udp_hand_node") == "udp_hand_node"


def test_all_pgrep_helpers_match_comm_not_cmdline(_stub_script_paths):
    """Every pgrep-based action resolves the PID by comm (pgrep -nx), never -f.

    Command-line matching self-matched the launch's own wrapper bash and its
    transient sub-shell forks on NUC13 (#151). comm matching cannot: a wrapper's
    comm is bash/sh/pgrep/cat, never the node's exec name.
    """
    cases = [
        (
            pinning.pin_process_to_slot("UR ros2_control_node", "ros2_control_node", 6),
            "ros2_control_no",
        ),
        (
            pinning.pin_dds_threads_to_slot(
                "integrated_rt_controller", "integrated_rt_controller", 2
            ),
            "integrated_rt_c",
        ),
        (
            pinning.adopt_process_into_shield(
                "integrated_rt_controller", "integrated_rt_controller"
            ),
            "integrated_rt_c",
        ),
    ]
    for action, comm in cases:
        snippet = action.cmd[2][0].text
        assert f'pgrep -nx "{comm}"' in snippet, f"expected comm match: {snippet}"
        assert "pgrep -f" not in snippet, f"cmdline pgrep leaked: {snippet}"
