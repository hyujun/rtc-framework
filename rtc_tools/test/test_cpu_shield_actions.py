"""Shared cset-shield launch actions (issues #151, #233, #344).

Two properties matter here and neither is visible by reading a launch file:

* the shield-active probe checks ``cset`` **before** ``/sys/.../isolated``. A
  cset shield writes nothing to that file, so an isolcpus-only probe reads an
  active shield as inactive and re-runs ``on`` every launch (#151).
* ACTIVATE is emitted only when the adopt action exited 0. Getting this wrong is
  invisible in a green test run and only shows up as a robot doing 500 Hz motion
  control on CFS, so the callable is exercised directly at both exit codes.
"""

from __future__ import annotations

import pytest
from launch.actions import EmitEvent
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

from rtc_tools.launch import cpu_shield, pinning


@pytest.fixture(autouse=True)
def _stub_script_path(monkeypatch):
    """Resolve script paths without an installed ``repo_scripts``.

    Both modules must be stubbed, and the second one is easy to miss: the chain
    calls ``pinning.adopt_process_into_shield``, which resolves the path through
    ``pinning``'s own reference — patching only this module's name leaves that
    call hitting the real ``ament_index``. The CI Python-test job installs just
    ``rtc_tools`` / ``rtc_msgs`` / ``rtc_digital_twin``, so the lookup raises
    ``PackageNotFoundError`` there while passing on a full local workspace.
    """
    monkeypatch.setattr(cpu_shield, "cpu_shield_path", lambda: "/opt/stub/cpu_shield.sh")
    monkeypatch.setattr(pinning, "cpu_shield_path", lambda: "/opt/stub/cpu_shield.sh")
    monkeypatch.setattr(pinning, "rt_common_path", lambda: "/opt/stub/rt_common.sh")


def _text(action) -> str:
    return action.cmd[2][0].text


def _lifecycle_node() -> LifecycleNode:
    """A real LifecycleNode — OnStateTransition rejects anything else.

    Construction only; nothing here launches it.
    """
    return LifecycleNode(
        package="integrated_bringup",
        executable="integrated_rt_controller",
        name="integrated_rt_controller",
        namespace="",
    )


def _on_exit_of(handler):
    """The ``on_exit`` value the handler was built with.

    launch stores it under one of two private names depending on what it was
    given — a callable lands in ``__on_event``, a static entity list in
    ``__actions_on_event`` — and that difference is exactly what the fail-closed
    wiring test needs to see.
    """
    inner = handler.event_handler
    return getattr(
        inner,
        "_OnActionEventBase__on_event",
        getattr(inner, "_OnActionEventBase__actions_on_event", None),
    )


# ── enable_cpu_shield ────────────────────────────────────────────────────────


@pytest.mark.parametrize("mode", ["--robot", "--sim"])
def test_shield_probe_is_cset_aware_before_isolcpus(mode):
    snippet = _text(cpu_shield.enable_cpu_shield(mode))

    cset_probe = snippet.index(f'"$SHIELD" check {mode}')
    isolcpus_probe = snippet.index('elif [ -n "$ISOLATED" ]')
    # Order is the fix, not a style choice: the isolcpus branch must be the
    # fallback, never the sole test (issue #151).
    assert cset_probe < isolcpus_probe
    assert f"on {mode}" in snippet


@pytest.mark.parametrize("mode", ["--robot", "--sim"])
def test_shield_probe_tests_currency_not_mere_presence(mode):
    """The probe must ask "is the shield *right*", not "is a shield up".

    This assertion replaced one that pinned the literal
    ``cset shield -s | grep -q "user"``. That probe answered presence only, so
    after a layout change the previous, wider shield satisfied it, the
    reconfigure was skipped, and the cores were never returned — while the
    launch log reported the shield as active (issue #349 D14). Delegating to
    ``cpu_shield.sh check`` keeps the mask comparison in one place and makes
    "cannot tell" fall through to a re-apply.
    """
    snippet = _text(cpu_shield.enable_cpu_shield(mode))

    assert 'grep -q "user"' not in snippet, "presence-only probe is back"
    assert f'"$SHIELD" check {mode}' in snippet


@pytest.mark.parametrize("mode", ["--robot", "--sim"])
def test_shield_probe_carries_the_layout_profile(mode):
    """Both the currency probe and the apply must name the profile (#350).

    They have to agree: a ``check`` that asks about the MPC-on mask while ``on``
    builds the MPC-off one would reconfigure the shield on every single launch,
    and the reverse would report a stale wide shield as current.
    """
    snippet = _text(cpu_shield.enable_cpu_shield(mode))

    assert f'"$SHIELD" check {mode} --profile "$PROFILE"' in snippet
    assert f'sudo "$SHIELD" on {mode} --profile "$PROFILE"' in snippet


def test_shield_profile_defaults_to_reserving_mpc_cores():
    """Only an explicit false opts out; empty means "defer to YAML" (#350).

    The tie breaks toward reserving because the two errors are not symmetric.
    Reserving cores nobody uses wastes them; failing to reserve cores someone
    does use puts a SCHED_FIFO thread on a core the shield just handed back to
    the system cpuset.
    """
    snippet = _text(cpu_shield.enable_cpu_shield("--robot"))

    assert 'PROFILE="mpc_on"' in snippet
    assert 'PROFILE="mpc_off"' in snippet
    # The opt-out is keyed off the launch argument, not off anything the host
    # can observe — MPC activation is a controller-YAML decision.
    assert "false|0|no" in snippet


@pytest.mark.parametrize("value", ["", "false", "true"])
def test_shield_passes_enable_mpc_as_a_positional_argument(value):
    """The tri-state value rides argv, so both launch styles share one call.

    The robot launches build their description without an ``OpaqueFunction``, so
    they cannot ``perform()`` a ``LaunchConfiguration`` — baking the resolved
    string into the script text would work for the sim launches only.
    """
    action = cpu_shield.enable_cpu_shield("--robot", enable_mpc=value)

    assert action.cmd[3][0].text == "rtc_cpu_shield"  # $0 for bash -c
    assert action.cmd[4][0].text == value


def test_shield_accepts_an_unresolved_substitution():
    """The robot launches pass a ``LaunchConfiguration``, resolved at run time.

    A plain-string-only contract would silently work in the sim tests and fail
    on the robot files, which are the ones that cannot resolve it early.
    """
    action = cpu_shield.enable_cpu_shield("--robot", enable_mpc=LaunchConfiguration("enable_mpc"))

    assert isinstance(action.cmd[4][0], LaunchConfiguration)


def test_shield_without_enable_mpc_still_builds():
    """Omitting the keyword must not crash — it reserves, like it always did."""
    action = cpu_shield.enable_cpu_shield("--robot")

    assert action.cmd[4][0].text == ""


def test_shield_rejects_unknown_mode():
    with pytest.raises(ValueError):
        cpu_shield.enable_cpu_shield("--robot-ish")


def test_shield_log_prefix_is_parameterised():
    assert "[SIM]" in _text(cpu_shield.enable_cpu_shield("--sim", log_prefix="[SIM]"))
    assert "[RT]" in _text(cpu_shield.enable_cpu_shield("--robot"))


def test_shield_missing_script_is_benign():
    snippet = _text(cpu_shield.enable_cpu_shield("--robot"))
    assert "cpu_shield.sh not found" in snippet
    assert "exit 0" in snippet


def test_shield_gating_toggles_condition():
    assert cpu_shield.enable_cpu_shield("--robot", gated=True).condition is not None
    assert cpu_shield.enable_cpu_shield("--robot", gated=False).condition is None


# ── shield_adopt_then_activate ───────────────────────────────────────────────


def test_chain_returns_adopt_and_two_handlers():
    node = _lifecycle_node()
    adopt, after_configure, after_adopt = cpu_shield.shield_adopt_then_activate(node)

    # gated=False: the caller gates ACTIVATE on this action's exit, so it has to
    # run (and exit) even when use_cpu_affinity:=false.
    assert adopt.condition is None
    assert after_configure is not None and after_adopt is not None


def test_fail_closed_activates_only_on_zero_exit():
    """Exit 0 activates; anything else must not."""
    node = _lifecycle_node()

    ok = cpu_shield.activate_or_refuse(node, 0, label="cm", log_prefix="[RT]")
    assert len(ok) == 1
    assert isinstance(ok[0], EmitEvent)

    refused = cpu_shield.activate_or_refuse(node, 1, label="cm", log_prefix="[RT]")
    # No EmitEvent at all — the controller stays inactive.
    assert not any(isinstance(entity, EmitEvent) for entity in refused)


def test_refusal_message_names_the_remedy():
    refused = cpu_shield.activate_or_refuse(
        _lifecycle_node(), 1, label="integrated_rt_controller", log_prefix="[RT]"
    )
    # LogInfo normalises its msg into a substitution list.
    msg = "".join(part.text for part in refused[0].msg)
    # An operator reading only this line must know what broke and what to do.
    assert "refusing to ACTIVATE" in msg
    assert "SCHED_FIFO" in msg
    assert "cpu_shield.sh off" in msg


def test_fail_closed_is_the_default():
    """Every launch calls this with no flag, so the default *is* the contract.

    Pinned separately from the explicit-argument test below: flipping the default
    to False would silently return all five launches to fail-open while every
    explicitly-parameterised test kept passing.
    """
    _, _, after_adopt = cpu_shield.shield_adopt_then_activate(_lifecycle_node())
    assert callable(_on_exit_of(after_adopt))


def test_fail_closed_installs_a_callable_and_opt_out_does_not():
    """The chain must actually consult the exit code, not just own the rule.

    fail_closed=True installs the callable form of ``on_exit`` (launch passes it
    the ProcessExited event); fail_closed=False installs a static entity list,
    which fires on any exit code — the pre-#344 behaviour.
    """
    node = _lifecycle_node()

    _, _, closed = cpu_shield.shield_adopt_then_activate(node, fail_closed=True)
    _, _, opened = cpu_shield.shield_adopt_then_activate(node, fail_closed=False)

    assert callable(_on_exit_of(closed))
    assert not callable(_on_exit_of(opened))
    assert any(isinstance(entity, EmitEvent) for entity in _on_exit_of(opened))


def test_adopt_targets_the_named_process():
    node = _lifecycle_node()
    adopt, _, _ = cpu_shield.shield_adopt_then_activate(
        node, label="integrated_rt_controller", process_grep="integrated_rt_controller"
    )
    # comm is truncated to 15 chars by the kernel; the helper must match that.
    assert 'pgrep -nx "integrated_rt_c"' in _text(adopt)
