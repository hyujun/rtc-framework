"""``taskset`` pinning actions that resolve slot indices to logical CPU ids.

``ThreadConfig::cpu_core`` — and therefore everything ``thread_layout`` returns —
is a **slot index**, not a kernel logical CPU id. C++ ``ApplyThreadConfig``
translates it through ``CpuTopology::physical_core_slots``
(``thread_utils.hpp::SlotToLogicalCpu``); the shell side translates it through
``rt_common.sh::slot_to_logical_cpu``. Launch files must do the same before
handing a number to ``taskset``, or the pin lands on an unrelated CPU.

On a hybrid NUC 13 Pro (Raptor Lake-P, 4P+8E) the two differ sharply::

    PHYSICAL_CORE_SLOTS = 0 2 4 6 8 9 10 11 12 13 14 15
    slot 2 -> logical 4     slot 6 -> logical 10     slot 7 -> logical 11

Passing the raw slot pins the rt_callback co-pin onto logical 2 (rt_control's
P-core, SCHED_FIFO 90) and the arm/hand drivers onto logical 6/7 (mpc_main's
P-core and its SMT sibling) — the exact cores the RT layout works to protect.
See issue #163; the equivalent shell-side fix is commit e93d0d4.

Design — no third mirror
------------------------
Two translation implementations already exist and are drift-tested against each
other (``repo_scripts/test/test_rt_common.sh`` fixtures pin the C++
``physical_core_slots`` ordering). Re-deriving the slot map in Python would make
a third. Instead the generated ``bash -c`` snippet sources ``rt_common.sh`` and
calls ``slot_to_logical_cpu`` directly, so responsibility splits cleanly:

* ``thread_layout`` (Python) owns **tier -> slot**  — drift-tested against
  ``thread_config.hpp`` by ``test/test_thread_layout.py``.
* ``rt_common.sh`` (bash) owns **slot -> logical** — drift-tested against
  ``cpu_topology.hpp`` by ``test_rt_common.sh``.

``integrated_bringup`` already declares ``<exec_depend>repo_scripts</exec_depend>``
and ``<exec_depend>rtc_tools</exec_depend>``, so this adds no new dependency.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

# ``cpu_core == -1``: "do not pin" sentinel from ``select_thread_layout``.
NO_PIN_SENTINEL = -1

# Thread names that ``ApplyThreadConfig`` (thread_utils.hpp) already places on a
# dedicated core via ``pthread_setname_np``. ``pin_dds_threads_to_slot`` must
# never move these — doing so overrides the RT layout it works to protect. This
# is a 1:1 mirror of the ``.name`` fields in
# ``rtc_base/include/rtc_base/threading/thread_config.hpp`` and is drift-tested
# against that header (test_pinning_slot_to_logical.py). Names are ≤ 15 chars so
# they survive ``pthread_setname_np`` truncation and match ``/proc/*/comm``
# verbatim. Process-level names (arm_driver/hand_driver/sim_thread/viewer) live
# in separate processes and never appear in a co-pinned process's task list, but
# are included so the set stays an exact mirror of the header.
RTC_OWNED_THREAD_NAMES = frozenset(
    {
        "rt_control",
        "rt_callback",
        "mpc_main",
        "mpc_worker_0",
        "mpc_worker_1",
        "nrt_logging",
        "nrt_callback",
        "arm_driver",
        "hand_driver",
        "sim_thread",
        "viewer",
    }
)


def rt_common_path() -> str:
    """Absolute path to the installed ``rt_common.sh``.

    ``repo_scripts`` installs its scripts under ``lib/<pkg>`` and its sourceable
    helpers under ``lib/<pkg>/lib`` (see ``repo_scripts/CMakeLists.txt``), so the
    share directory is walked up twice — the same derivation the launch files
    already use to locate ``cpu_shield.sh``.
    """
    share = get_package_share_directory("repo_scripts")
    prefix = os.path.dirname(os.path.dirname(share))
    return os.path.join(prefix, "lib", "repo_scripts", "lib", "rt_common.sh")


def cpu_shield_path() -> str:
    """Absolute path to the installed ``cpu_shield.sh`` (``lib/repo_scripts/``).

    Same prefix derivation as :func:`rt_common_path`; the scripts install one
    directory shallower than the sourceable helpers.
    """
    share = get_package_share_directory("repo_scripts")
    prefix = os.path.dirname(os.path.dirname(share))
    return os.path.join(prefix, "lib", "repo_scripts", "cpu_shield.sh")


def _reject_quotes(**fields: str) -> None:
    """Reject embedded double quotes in values interpolated into ``bash -c``.

    Defense in depth: current callers pass hardcoded literals, but a future
    caller must not be able to inject shell syntax through a label.
    """
    for key, value in fields.items():
        if '"' in value:
            raise ValueError(f"embedded double quote disallowed in {key}={value!r}")


def _pgrep_pattern(process_grep: str) -> str:
    """Bracket the first character so ``pgrep -f`` never matches the launch's
    own helper.

    These actions run as ``bash -c '… pgrep -nf "<process_grep>" …'``, so the
    helper's *own* command line contains ``<process_grep>`` verbatim. ``pgrep
    -f`` matches full command lines, and ``-n`` returns the newest — the
    just-spawned helper — so a bare pattern resolves to the wrapper bash (which
    then exits) instead of the target node. On NUC13 this made ``adopt`` move a
    transient bash into the shield while the real controller stayed in "system"
    and its RT pins EINVAL'd (issue #151); the DDS co-pin hit the same wall.

    Wrapping the first char as a regex class — ``integrated_rt_controller`` ->
    ``[i]ntegrated_rt_controller`` — is the classic ``ps | grep '[p]attern'``
    trick: the regex still matches the real process's literal command line, but
    the helper's command line now contains the *bracketed* text, which the
    regex does not match. So every pgrep-based helper excludes itself and its
    siblings, leaving only the true target.
    """
    if not process_grep:
        return process_grep
    return f"[{process_grep[0]}]{process_grep[1:]}"


def _resolve_pid_snippet(process_grep: str) -> str:
    """Bash that leaves the target *node's* PID in ``$PID`` (empty if absent).

    ``pgrep -nf`` alone is not enough. The bracket trick (:func:`_pgrep_pattern`)
    stops the pattern from matching itself, but the launch helper's command line
    *also* carries the raw process name in its human-readable ``label`` / warning
    strings (e.g. ``"integrated_rt_controller not found …"``), so ``pgrep -f``
    still matches the wrapper bash through that occurrence and ``-n`` returns it —
    a shell, not the node (issue #151: adopt moved ``comm='bash'``).

    So: enumerate every match, then keep the newest whose ``/proc/<pid>/comm`` is
    not a shell or the shield script. The real node's comm is the (truncated)
    executable name (``integrated_rt_c``); every self-/sibling-match is ``bash``.
    The comm filter is the load-bearing guard; the bracketed pattern just trims
    the candidate set.
    """
    pat = _pgrep_pattern(process_grep)
    return (
        'PID=""; '
        f'for _p in $(pgrep -f "{pat}" 2>/dev/null); do '
        '  _c=$(cat "/proc/$_p/comm" 2>/dev/null || echo ""); '
        '  case "$_c" in bash|sh|dash|cpu_shield.sh) continue ;; esac; '
        '  PID="$_p"; '
        "done; "
    )


def _skip_action(message: str) -> ExecuteProcess:
    """An action that only logs ``message`` — used for the no-pin sentinel."""
    return ExecuteProcess(
        cmd=["bash", "-c", f'echo "{message}"'],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_cpu_affinity")),
    )


def _resolve_slot_prelude(label: str, slot: int) -> str:
    """Bash prelude that leaves the resolved logical CPU id in ``$CPU``.

    Falls back to *skipping the pin* (never to the raw slot) when ``rt_common.sh``
    is missing: pinning to an unverified number is what issue #163 is about, so a
    skipped pin with a warning is strictly safer than a wrong pin.

    ``slot_to_logical_cpu`` itself degrades to identity when the topology cannot
    be read (container without sysfs), matching ``SlotToLogicalCpu``'s fallback.
    """
    return (
        f'RTC_RT_COMMON="{rt_common_path()}"; '
        'if [ ! -f "$RTC_RT_COMMON" ]; then '
        f'  echo "[RT] WARNING: rt_common.sh not found at $RTC_RT_COMMON — '
        f'{label} pinning skipped (slot {slot} left unresolved)"; exit 0; '
        "fi; "
        '. "$RTC_RT_COMMON"; '
        "detect_hybrid_capability >/dev/null 2>&1 || true; "
        f"CPU=$(slot_to_logical_cpu {slot}); "
        'if [ -z "$CPU" ]; then '
        f'  echo "[RT] WARNING: slot_to_logical_cpu {slot} returned empty — '
        f'{label} pinning skipped"; exit 0; '
        "fi; "
    )


def pin_process_to_slot(label: str, process_grep: str, slot: int) -> ExecuteProcess:
    """Pin the newest process matching ``process_grep`` to ``slot``'s logical CPU.

    ``slot < 0`` is the "no pinning" sentinel from ``select_thread_layout``; the
    returned action then only logs the skip, so launch files need no own guard.

    NOTE: ``taskset -p`` (no ``-a``) sets the affinity of the *main thread only*
    — ``sched_setaffinity`` takes a TID, not a process. Existing worker threads
    keep their inherited mask. Whether to widen this to ``-a`` is gated on NUC13
    E-core measurements (issue #163 Phase 4), because pinning the whole UR driver
    onto one E-core may regress its 500 Hz RTDE loop.
    """
    _reject_quotes(label=label, process_grep=process_grep)

    if slot < 0:
        return _skip_action(f"[RT] {label}: cpu_core={slot}, no taskset pinning")

    return ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            _resolve_slot_prelude(label, slot)
            + _resolve_pid_snippet(process_grep)
            + 'if [ -z "$PID" ]; then '
            f'  echo "[RT] WARNING: {label} not found — CPU pinning skipped"; exit 0; '
            "fi; "
            'taskset -cp "$CPU" "$PID" >/dev/null 2>&1 && '
            f'  echo "[RT] {label} (PID=$PID) pinned to slot {slot} -> logical CPU $CPU" || '
            f'  echo "[RT] WARNING: {label} (PID=$PID) taskset to logical CPU $CPU failed"',
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_cpu_affinity")),
    )


def adopt_process_into_shield(
    label: str, process_grep: str, *, gated: bool = True
) -> ExecuteProcess:
    """Move the newest process matching ``process_grep`` into the cset shield.

    ``cset shield --cpu`` (cpu_shield.sh ``on``) *creates* the isolated "user"
    cpuset but sweeps existing tasks into "system"; a process launched afterwards
    inherits "system" and is barred from the RT logical CPUs. Its
    ``pthread_setaffinity_np`` then returns EINVAL and ``thread_utils.hpp`` aborts
    the whole thread config — the RT thread loses its pin *and* SCHED_FIFO. This
    action hands the PID to ``cpu_shield.sh adopt``, which moves it (and all its
    threads) into "user" so RT/nrt self-pins land in-set. Issue #151.

    Must complete *before* the controller activates and spawns its RT threads
    (which then inherit the "user" cpuset). Callers gate ACTIVATE on this action's
    ``OnProcessExit`` for a deterministic order rather than racing the async sudo
    call against thread creation. Defensive by design: skips (warns, never fails
    the launch) when the process is gone, the script is missing, sudo needs a
    password, or no shield is active (the ``adopt`` subcommand itself no-ops in
    the last case, so shield-off runs keep the CM at full affinity and its
    existing self-pin path is unchanged).

    ``gated=False`` drops the ``use_cpu_affinity`` :class:`IfCondition` so the
    action always runs (and always exits): a caller gating ACTIVATE on its exit
    needs it to fire even when ``use_cpu_affinity:=false``, where the bash body
    finds no shield and exits 0 immediately.
    """
    _reject_quotes(label=label, process_grep=process_grep)

    shield = cpu_shield_path()
    kwargs = {}
    if gated:
        kwargs["condition"] = IfCondition(LaunchConfiguration("use_cpu_affinity"))
    return ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            f'SHIELD="{shield}"; ' + _resolve_pid_snippet(process_grep) + 'if [ -z "$PID" ]; then '
            f'  echo "[RT] WARNING: {label} not found — shield adopt skipped"; exit 0; '
            "fi; "
            'if [ ! -f "$SHIELD" ]; then '
            f'  echo "[RT] WARNING: cpu_shield.sh not found at $SHIELD — '
            f'{label} shield adopt skipped"; exit 0; '
            "fi; "
            "if sudo -n true 2>/dev/null; then "
            '  sudo "$SHIELD" adopt "$PID"; '
            "else "
            f'  echo "[RT] WARNING: sudo requires a password — {label} shield adopt '
            'skipped (RT pin relies on shield being off)"; '
            "fi",
        ],
        output="screen",
        **kwargs,
    )


def pin_dds_threads_to_slot(label: str, process_grep: str, slot: int) -> ExecuteProcess:
    """Co-pin a node's DDS / aux threads onto ``slot``'s logical CPU.

    Layout v4.1 co-pins the DDS receive thread with ``rt_callback`` so message
    dispatch and state-callback execution share L1/L2. Only threads that RTC did
    NOT place are moved: a thread is left alone if its ``/proc/*/comm`` is in
    ``RTC_OWNED_THREAD_NAMES`` *or* it runs under SCHED_FIFO. The name check is
    the load-bearing one — ``nrt_logging`` / ``nrt_callback`` are SCHED_OTHER, so
    the old FIFO-only filter yanked them off their dedicated cores onto the
    rt_callback core (issue #163). The FIFO check stays as a belt-and-suspenders
    guard for any RT thread whose name was not set. The main thread (TID == PID)
    is covered by the same loop, so it too is protected by the name filter.
    """
    _reject_quotes(label=label, process_grep=process_grep)

    if slot < 0:
        return _skip_action(f"[RT] {label}: cpu_core={slot}, no DDS thread pinning")

    owned = " ".join(sorted(RTC_OWNED_THREAD_NAMES))
    return ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            _resolve_slot_prelude(label, slot)
            + _resolve_pid_snippet(process_grep)
            + 'if [ -z "$PID" ]; then '
            f'  echo "[RT] WARNING: {label} not found — DDS thread pinning skipped"; exit 0; '
            "fi; "
            f'RTC_OWNED=" {owned} "; '
            "PINNED=0; "
            "for TID in $(ls /proc/$PID/task/ 2>/dev/null); do "
            '  COMM=$(cat /proc/$PID/task/$TID/comm 2>/dev/null || echo ""); '
            '  case "$RTC_OWNED" in *" $COMM "*) continue ;; esac; '
            '  POLICY=$(chrt -p $TID 2>/dev/null | grep -o "SCHED_FIFO" || echo ""); '
            '  if [ -n "$POLICY" ]; then continue; fi; '
            '  taskset -cp "$CPU" "$TID" 2>/dev/null && PINNED=$((PINNED+1)); '
            "done; "
            f'echo "[RT] {label} (PID=$PID): $PINNED DDS/aux threads pinned to '
            f'slot {slot} -> logical CPU $CPU"',
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_cpu_affinity")),
    )
