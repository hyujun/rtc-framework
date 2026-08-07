"""cset shield actions shared by every bringup launch (issues #151, #233, #344).

Two blocks used to be hand-copied into all five launch files — the shield-enable
``ExecuteProcess`` and the ``configure -> activate`` lifecycle chain — and both
drifted. ``robot_ur5e_p1b`` got the cset-aware shield detection (#151) and the
adopt step; ``robot_ur5e_p1a`` and the three sim launches kept the original
isolcpus-only detection and activate straight out of ``configuring -> inactive``.
Nothing tested that a launch file was wired correctly, so three consecutive fixes
landed in one twin only. This module is the single copy, and
``integrated_bringup/test/test_launch_shield_wiring.py`` pins that every launch
uses it.

Why the chain has to be here, not inline
----------------------------------------
The order is load-bearing, not stylistic. ``cpu_shield.sh on`` creates the "user"
cpuset but sweeps existing tasks into "system", so a controller started afterwards
inherits "system" and is barred from the RT cores. Its ``pthread_setaffinity_np``
then returns EINVAL, and ``ApplyThreadConfig`` (thread_utils.hpp) returns *before*
setting SCHED_FIFO and before ``pthread_setname_np`` — so the 500 Hz loop ends up
fair-scheduled, unpinned, and **unnamed**. The missing name is what made this so
quiet: ``verify_rt_runtime.sh`` finds threads by name, so it could not see the
threads it was meant to judge. Adopting the process into "user" *before* it spawns
those threads is the fix, and gating ACTIVATE on the adopt action's exit is what
makes the order deterministic instead of a race against an async sudo call.

``fail_closed`` (issue #344)
----------------------------
When the adopt genuinely fails while a shield is active, activating anyway means
running real-robot motion control on CFS. The chain therefore refuses to emit
ACTIVATE unless the adopt action exits 0, leaving the controller ``inactive`` and
the failure on screen. Benign no-ops still exit 0 and still activate: no shield,
no ``cset``, process already gone, or ``use_cpu_affinity:=false``. Only the
dangerous case — active shield, adopt failed — holds the lifecycle back.
"""

from __future__ import annotations

from launch.actions import EmitEvent, ExecuteProcess, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

from rtc_tools.launch.pinning import adopt_process_into_shield, cpu_shield_path

SHIELD_MODES = ("--robot", "--sim")


def enable_cpu_shield(
    mode: str, *, log_prefix: str = "[RT]", gated: bool = True, enable_mpc=None
) -> ExecuteProcess:
    """Enable the cset CPU shield for ``mode`` unless one is already active.

    ``mode`` is ``--robot`` or ``--sim``; both shield the same CM-spanning range
    (``cpu_shield.sh::compute_shield_cores`` funnels both to
    ``get_cm_shield_cpus``), so the flag only selects the mode-specific extras.

    ``enable_mpc`` (issue #350) carries the launch's tri-state ``enable_mpc``
    argument — a substitution, a plain string, or ``None`` — and selects the
    layout *profile* the shield is built for. Only an explicit false disables
    MPC: an empty value means "defer to the controller YAML", and deferring must
    keep reserving the MPC cores, because the YAML can be overridden and a
    controller switch can spawn the MPC thread mid-session. Reserving cores
    nobody uses wastes them; not reserving cores someone does use puts a
    SCHED_FIFO thread on an unshielded core, so the tie breaks toward reserving.

    It is passed as a positional argument to ``bash -c`` rather than baked into
    the script text so both launch styles can use one call shape: the robot
    launches have no ``OpaqueFunction`` and so cannot ``perform()`` a
    ``LaunchConfiguration`` while building their description.

    The active-shield test runs ``cpu_shield.sh check`` **before** falling back
    to ``/sys/devices/system/cpu/isolated``. That order is the #151 fix: a cset
    shield writes nothing to ``isolated`` — that file is for isolcpus — so an
    isolcpus-only test reads an active cset shield as "not active" and re-runs
    ``on`` on every launch.

    ``check`` rather than a local ``cset shield -s | grep user`` (issue #349
    D14): presence is not currency. After a layout change the old, wider shield
    is still "present", so the grep skipped the reconfigure and the cores were
    never returned — while the log said the shield was active. ``check``
    compares the active mask against the desired one and exits non-zero when
    they differ *or when it cannot tell*, so the fall-through re-applies. It
    needs no root, which is why this stays a cheap pre-test rather than always
    paying the sudo path.

    ``gated=False`` drops the ``use_cpu_affinity`` :class:`IfCondition`, for
    callers that already decided in Python whether to build this action at all.
    """
    if mode not in SHIELD_MODES:
        raise ValueError(f"mode must be one of {SHIELD_MODES}, got {mode!r}")

    shield = cpu_shield_path()
    kwargs = {}
    if gated:
        kwargs["condition"] = IfCondition(LaunchConfiguration("use_cpu_affinity"))

    return ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            f'SHIELD="{shield}"; '
            'if [ ! -f "$SHIELD" ]; then '
            f'  echo "{log_prefix} WARNING: cpu_shield.sh not found: $SHIELD"; exit 0; '
            "fi; "
            # Tri-state enable_mpc -> layout profile. Only an explicit false
            # opts out; "" (defer to YAML) keeps the MPC reservation.
            'PROFILE="mpc_on"; '
            'case "$(echo "${1:-}" | tr "[:upper:]" "[:lower:]")" in '
            '  false|0|no) PROFILE="mpc_off" ;; '
            "esac; "
            "ISOLATED=$(cat /sys/devices/system/cpu/isolated 2>/dev/null); "
            "if command -v cset >/dev/null 2>&1 && "
            f'   "$SHIELD" check {mode} --profile "$PROFILE" >/dev/null 2>&1; then '
            f'  echo "{log_prefix} CPU shield already active and current (profile: $PROFILE)"; '
            'elif [ -n "$ISOLATED" ]; then '
            f'  echo "{log_prefix} CPU shield already active: Core $ISOLATED isolated (isolcpus)"; '
            "else "
            f'  echo "{log_prefix} CPU shield not active — enabling {mode} mode '
            '(profile: $PROFILE)..."; '
            "  if sudo -n true 2>/dev/null; then "
            f'    sudo "$SHIELD" on {mode} --profile "$PROFILE"; '
            "  else "
            f'    echo "{log_prefix} WARNING: sudo requires a password — skipping CPU shield. '
            "Configure passwordless sudo for cpu_shield.sh or run: "
            f'sudo $SHIELD on {mode} --profile $PROFILE"; '
            "  fi; "
            "fi",
            "rtc_cpu_shield",  # $0 for the script above
            enable_mpc if enable_mpc is not None else "",
        ],
        output="screen",
        **kwargs,
    )


def activate_entities(lifecycle_node):
    """The EmitEvent list that drives ``lifecycle_node`` to ACTIVATE."""
    return [
        EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=lambda node: node == lifecycle_node,
                transition_id=Transition.TRANSITION_ACTIVATE,
            )
        )
    ]


def activate_or_refuse(lifecycle_node, returncode: int, *, label: str, log_prefix: str):
    """Entities to emit once the adopt action has exited with ``returncode``.

    The fail-closed decision, kept out of the launch plumbing so it can be tested
    as what it is: a two-branch rule about whether the robot is allowed to move.

    Non-zero means the adopt was needed and failed (see
    :func:`rtc_tools.launch.pinning.adopt_process_into_shield` for which exits are
    benign). Holding the controller in ``inactive`` is the safe direction — the
    alternative is 500 Hz motion control on CFS, unpinned. The launch stays up so
    the state can be inspected.
    """
    if returncode == 0:
        return activate_entities(lifecycle_node)
    return [
        LogInfo(
            msg=(
                f"{log_prefix} ERROR: shield adopt failed (exit {returncode}) — refusing to "
                f"ACTIVATE {label}. Its RT threads would lose both CPU affinity and "
                "SCHED_FIFO (setaffinity EINVAL, issue #151). Fix passwordless sudo "
                "(install.sh setup_rt_sudoers), or release the shield with "
                "'sudo cpu_shield.sh off', then relaunch."
            )
        )
    ]


def shield_adopt_then_activate(
    lifecycle_node,
    *,
    label: str = "integrated_rt_controller",
    process_grep: str = "integrated_rt_controller",
    log_prefix: str = "[RT]",
    fail_closed: bool = True,
):
    """Wire ``configuring -> inactive`` -> shield adopt -> ACTIVATE.

    Returns ``(adopt_action, adopt_after_configure, activate_after_adopt)``. The
    caller adds the two handlers to its launch description and keeps owning what
    triggers CONFIGURE — that differs by launch (the robot files gate it on the
    readiness-gate process exiting, the sim files on the MuJoCo node reaching
    ``active``) and is not part of this contract.

    The adopt runs after configure rather than at process start because the RT
    threads are spawned in ``on_activate``: the move only helps if it lands
    between the two.

    With ``fail_closed`` (default), ACTIVATE is emitted only when the adopt action
    exits 0 — see the module docstring for which failures are benign.
    """
    adopt_action = adopt_process_into_shield(label, process_grep, gated=False)

    adopt_after_configure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lifecycle_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg=f"{log_prefix} configured — adopting {label} into CPU shield"),
                adopt_action,
            ],
        )
    )

    if fail_closed:

        def _on_adopt_exit(event, context):
            return activate_or_refuse(
                lifecycle_node, event.returncode, label=label, log_prefix=log_prefix
            )

        on_exit = _on_adopt_exit
    else:
        on_exit = activate_entities(lifecycle_node)

    activate_after_adopt = RegisterEventHandler(
        OnProcessExit(target_action=adopt_action, on_exit=on_exit)
    )

    return adopt_action, adopt_after_configure, activate_after_adopt
