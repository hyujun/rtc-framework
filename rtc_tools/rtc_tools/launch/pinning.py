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
import subprocess

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
        "nrt_logging",
        "nrt_callback",
        # Shares nrt_callback's core but self-pins under its own name since
        # issue #349 D15. It must be listed here for the same reason as the
        # rest: the DDS sweep moves by exclusion, so a thread missing from
        # this set is a thread the sweep is free to drag off its slot.
        "nrt_publish",
        "arm_driver",
        "hand_driver",
        "sim_thread",
        "viewer",
    }
)

# CycloneDDS' own threads, as they appear in ``/proc/*/comm``.
#
# This is a *verification* set, not a selection set: the sweep still decides
# what to move by exclusion (RTC_OWNED_THREAD_NAMES + SCHED_FIFO), which is what
# issue #163 tuned. This set answers the different question of whether the sweep
# actually covered the DDS threads it exists for — previously the sweep reported
# only a count, and nothing anywhere checked which threads that count contained.
#
# Measured, not read out of the library (issue #164). ``strings libddsc.so``
# shows the prefix ``dq.builtin``; the thread's real ``comm`` is ``dq.builtins``,
# so a set built from the binary would silently never match. The measurement
# also established that CycloneDDS creates these at domain init and adds none
# afterwards: three rounds of participant discovery/matching/departure plus
# eight nodes created late in the same process produced zero new TIDs. That is
# why the one-shot sweep is sound in kind and this file does not re-pin
# periodically — see the issue for the probe and its limits (rclpy process,
# loopback, CycloneDDS 0.10.5; FastDDS not measured).
#
# ``recvMC`` appears only when multicast is enabled (the robot configs set
# ``AllowMulticast=spdp``), so absence is not an error — the sweep reports which
# of these are present and unpinned, never that a name is missing outright.
# All names are < 15 chars, so ``/proc/*/comm`` truncation does not apply.
CYCLONEDDS_THREAD_NAMES = frozenset(
    {
        "recv",
        "recvMC",
        "recvUC",
        "tev",
        "gc",
        "dq.builtins",
        "dq.user",
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


def _comm_pattern(process_grep: str) -> str:
    """The process *comm* (kernel thread name) for ``process_grep``.

    ``comm`` is the executable basename truncated to ``TASK_COMM_LEN - 1`` = 15
    chars (``integrated_rt_controller`` -> ``integrated_rt_c``). Callers pass the
    executable name, so the comm is its 15-char prefix.
    """
    return process_grep[:15]


def _resolve_pid_snippet(process_grep: str) -> str:
    """Bash that leaves the target *node's* PID in ``$PID`` (empty if absent).

    Match on ``comm`` (``pgrep -nx``), NOT the command line (``pgrep -f``). Every
    cmdline-based approach self-matched on NUC13 (issue #151): the launch helper
    runs as ``bash -c '… <name> …'`` and carries the raw process name in its
    ``pgrep`` arg AND its label / warning strings, so ``pgrep -f`` matched the
    wrapper bash (and its transient ``$(…)`` sub-shell forks, which then exited
    before the move) instead of the node. Bracketing the pattern fixed the pgrep
    arg but not the label; a comm filter still raced the sub-shell forks.

    ``comm`` sidesteps all of it: the node's comm is the truncated exec name
    (``integrated_rt_c``); every wrapper / fork is ``bash`` / ``sh`` / ``pgrep`` /
    ``cat``, none of which equals the target comm. ``-x`` demands an exact comm
    match and ``-n`` takes the newest, so only the real node is ever returned.
    """
    comm = _comm_pattern(process_grep)
    return f'PID=$(pgrep -nx "{comm}" 2>/dev/null); '


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


def resolve_logical_cpu(slot: int) -> int | None:
    """Resolve a slot index to its kernel logical CPU id *now*, via ``rt_common.sh``.

    The eager twin of :func:`_resolve_slot_prelude`: same translation, same "no
    third mirror" rule (module docstring) — the number comes out of
    ``rt_common.sh::slot_to_logical_cpu`` and is never re-derived in Python — but
    the answer is needed at launch-*build* time instead of inside a spawned
    action. ``controller_manager``'s native ``cpu_affinity`` parameter takes a
    logical CPU id and must be written into a YAML file before the driver
    process exists, so the bash-snippet form cannot serve it (issue #343).

    Returns ``None`` — never a guess — for the no-pin sentinel, a missing
    helper, or a failed lookup. Callers must then omit the pin entirely: writing
    an unverified number into ``cpu_affinity`` is exactly what issue #163 was.
    """
    if slot < 0:
        return None

    rt_common = rt_common_path()
    if not os.path.isfile(rt_common):
        return None

    # ``int(slot)`` also serves as the injection guard for the interpolation.
    script = (
        f'. "{rt_common}" || exit 1; '
        "detect_hybrid_capability >/dev/null 2>&1 || true; "
        f"slot_to_logical_cpu {int(slot)}"
    )
    try:
        completed = subprocess.run(
            ["bash", "-c", script],
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if completed.returncode != 0:
        return None
    try:
        cpu = int(completed.stdout.strip())
    except ValueError:
        return None
    return cpu if cpu >= 0 else None


def pin_process_to_slot(label: str, process_grep: str, slot: int) -> ExecuteProcess:
    """Pin the newest process matching ``process_grep`` to ``slot``'s logical CPU.

    ``slot < 0`` is the "no pinning" sentinel from ``select_thread_layout``; the
    returned action then only logs the skip, so launch files need no own guard.

    ``taskset -p`` pins the *main thread only* (``sched_setaffinity`` takes a TID,
    not a process). Existing worker threads keep their inherited mask, and
    threads created *later* inherit the new one. Correct only where the main
    thread is the work: the **MuJoCo simulator**, whose physics stepping runs on
    it, is the only remaining caller.

    This scope is **not** usable for a process whose real-time work lives in a
    worker thread — it pins the wrong TID while looking successful. The UR arm
    driver was pinned this way until issue #343: ``ros2_control_node``'s main
    thread is the (idle) executor and its 500 Hz loop is a separate ``cm_thread``
    created before this timer fires, so the pin never reached it and the verifier,
    reading the same main thread, reported PASS. It now uses
    ``controller_manager``'s native ``cpu_affinity`` parameter, which is applied
    *inside* that thread — see :mod:`rtc_tools.launch.cm_rt_params`.

    An ``all_threads`` flag rendering ``taskset -acp`` existed for the hand driver
    until issue #345. It is gone rather than merely unused: the sweep it produced
    is now actively wrong. The hand process pins its own threads, and the aux lane
    (``hand_aux_io``) deliberately sits on a *different* core, so a blanket
    all-thread sweep would drag it back onto the hand_driver core — undoing the
    split it exists to protect. Use :func:`pin_dds_threads_to_slot`, which skips
    threads RTC placed itself.
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
            f'taskset -cp "$CPU" "$PID" >/dev/null 2>&1 && '
            f'  echo "[RT] {label} (PID=$PID, main thread) pinned to slot {slot} '
            f'-> logical CPU $CPU" || '
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
    call against thread creation.

    **The exit code is the contract** (issue #344). Benign skips exit 0 and the
    caller should proceed: the process is gone, the script is missing, or no
    shield is active (the ``adopt`` subcommand itself no-ops in the last case, so
    shield-off runs keep the CM at full affinity and its existing self-pin path
    is unchanged).

    Non-zero means the adopt was *needed and failed*, and the caller must not
    activate. Two ways to get there: passwordless sudo is unavailable while a cset
    shield is already active (e.g. relaunching after a manual ``sudo cpu_shield.sh
    on``), or ``cpu_shield.sh adopt`` itself failed to move the process. Either
    way the CM stays in the "system" cpuset, and its self-pin to the shielded RT
    cores then EINVALs — losing affinity *and* SCHED_FIFO, because
    ``ApplyThreadConfig`` returns before it sets the policy. isolcpus isolation is
    *not* this trap: explicit ``sched_setaffinity`` to an isolcpus core succeeds,
    so that path falls through to the benign warning.

    :func:`rtc_tools.launch.cpu_shield.shield_adopt_then_activate` is the wiring
    that consumes this contract; launch files should use it rather than calling
    this directly, so the ordering and the exit-code check stay in one place.

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
            # Exit status of the `if` is this command's, so cpu_shield.sh's
            # non-zero adopt failure reaches the caller unchanged (issue #344).
            '  sudo "$SHIELD" adopt "$PID"; '
            'elif command -v cset >/dev/null 2>&1 && cset shield -s 2>/dev/null | grep -q "user"; then '
            f'  echo "[RT] ERROR: cset CPU shield is ACTIVE but passwordless sudo is unavailable — '
            f"{label} stays in the system cpuset and its RT thread pinning WILL fail "
            "(setaffinity EINVAL, issue #151). Configure passwordless sudo (install.sh "
            'setup_rt_sudoers + realtime group), or release the shield: sudo $SHIELD off"; '
            # Non-zero: this is the one branch that is NOT a benign skip, so a
            # caller gating ACTIVATE on this action must be able to tell it apart
            # from "no shield, nothing to do" below (issue #344).
            "  exit 1; "
            "else "
            f'  echo "[RT] WARNING: sudo requires a password — {label} shield adopt '
            'skipped (no active cset shield — CM keeps full affinity)"; '
            "fi",
        ],
        output="screen",
        **kwargs,
    )


def pin_dds_threads_to_slot(
    label: str,
    process_grep: str,
    slot: int,
    *,
    extra_protected_names: frozenset[str] | set[str] = frozenset(),
) -> ExecuteProcess:
    """Co-pin a node's DDS / aux threads onto ``slot``'s logical CPU.

    ``extra_protected_names`` adds process-specific thread names to the exclusion
    set for this call only. ``RTC_OWNED_THREAD_NAMES`` cannot absorb them: it is
    drift-locked to the ``.name`` fields in ``thread_config.hpp``
    (``test_rtc_owned_names_mirror_thread_config_hpp``), and a package-local
    thread has no entry there by definition. ``udp_hand_node``'s ``hand_aux_io``
    is the case this exists for — it is SCHED_OTHER and named outside the C++
    SSoT, so both the name filter and the FIFO guard would miss it and the sweep
    would pull it off the aux slot onto the hand_driver core (issue #345).

    Layout v4.1 co-pins the DDS receive thread with ``rt_callback`` so message
    dispatch and state-callback execution share L1/L2. Only threads that RTC did
    NOT place are moved: a thread is left alone if its ``/proc/*/comm`` is in
    ``RTC_OWNED_THREAD_NAMES`` *or* it runs under SCHED_FIFO. The name check is
    the load-bearing one — ``nrt_logging`` / ``nrt_callback`` are SCHED_OTHER, so
    the old FIFO-only filter yanked them off their dedicated cores onto the
    rt_callback core (issue #163). The FIFO check stays as a belt-and-suspenders
    guard for any RT thread whose name was not set. The main thread (TID == PID)
    is covered by the same loop, so it too is protected by the name filter.

    The sweep is one-shot on purpose. CycloneDDS creates its threads at domain
    init and adds none for later discovery, matching, or in-process node
    creation (measured — see ``CYCLONEDDS_THREAD_NAMES``), so a repeat pass has
    nothing to catch, while repeating an *exclusion* filter would drag every
    later non-RTC thread onto this core — the #163 regression again.

    What was missing is not repetition but evidence: this used to report a bare
    count, and nothing checked what that count contained. It now also names the
    CycloneDDS threads it moved and warns about any that were present and left
    behind — which is what a timer firing before domain init, or a CycloneDDS
    rename, would actually look like. The durable verdict lives in
    ``verify_rt_runtime.sh`` (issue #164).
    """
    _reject_quotes(label=label, process_grep=process_grep)

    if slot < 0:
        return _skip_action(f"[RT] {label}: cpu_core={slot}, no DDS thread pinning")

    for name in extra_protected_names:
        _reject_quotes(**{f"extra_protected_name[{name}]": name})
    owned = " ".join(sorted(RTC_OWNED_THREAD_NAMES | set(extra_protected_names)))
    dds = " ".join(sorted(CYCLONEDDS_THREAD_NAMES))
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
            f'DDS_NAMES=" {dds} "; '
            'PINNED=0; DDS_PINNED=""; DDS_MISSED=""; '
            "for TID in $(ls /proc/$PID/task/ 2>/dev/null); do "
            '  COMM=$(cat /proc/$PID/task/$TID/comm 2>/dev/null || echo ""); '
            '  IS_DDS=""; '
            '  case "$DDS_NAMES" in *" $COMM "*) IS_DDS=1 ;; esac; '
            # A DDS thread landing in either skip branch is the interesting
            # case: it means the name filter or the FIFO guard is shadowing a
            # thread this sweep exists to move, so record it rather than
            # silently dropping it out of the count.
            '  case "$RTC_OWNED" in *" $COMM "*) '
            '    [ -n "$IS_DDS" ] && DDS_MISSED="$DDS_MISSED $COMM(rtc-owned)"; continue ;; esac; '
            '  POLICY=$(chrt -p $TID 2>/dev/null | grep -o "SCHED_FIFO" || echo ""); '
            '  if [ -n "$POLICY" ]; then '
            '    [ -n "$IS_DDS" ] && DDS_MISSED="$DDS_MISSED $COMM(fifo)"; continue; '
            "  fi; "
            '  if taskset -cp "$CPU" "$TID" >/dev/null 2>&1; then '
            "    PINNED=$((PINNED+1)); "
            '    [ -n "$IS_DDS" ] && DDS_PINNED="$DDS_PINNED $COMM"; '
            "  else "
            '    [ -n "$IS_DDS" ] && DDS_MISSED="$DDS_MISSED $COMM(taskset-failed)"; '
            "  fi; "
            "done; "
            f'echo "[RT] {label} (PID=$PID): $PINNED DDS/aux threads pinned to '
            f'slot {slot} -> logical CPU $CPU"; '
            f'echo "[RT] {label}: CycloneDDS threads co-pinned:${{DDS_PINNED:- (none)}}"; '
            # Absence of a name is not reported: recvMC only exists with
            # multicast, and the set is a superset by design. Only a DDS thread
            # that was seen and NOT moved is a defect.
            'if [ -n "$DDS_MISSED" ]; then '
            f'  echo "[RT] WARNING: {label}: CycloneDDS threads present but NOT co-pinned:'
            '$DDS_MISSED — DDS dispatch will not share cache with rt_callback"; '
            "fi; "
            'if [ -z "$DDS_PINNED" ]; then '
            f'  echo "[RT] WARNING: {label}: no CycloneDDS thread was co-pinned. Either the '
            "sweep ran before DDS domain init, or the thread names changed (expected one of:"
            f' {dds}). See issue #164."; '
            "fi",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_cpu_affinity")),
    )
