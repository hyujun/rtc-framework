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
from launch.substitutions import LaunchConfiguration, PythonExpression
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


@pytest.mark.parametrize(
    ("enable_mpc", "expected"),
    [
        ("false", cpu_shield.MPC_OFF),
        ("False", cpu_shield.MPC_OFF),
        ("0", cpu_shield.MPC_OFF),
        ("no", cpu_shield.MPC_OFF),
        (" FALSE ", cpu_shield.MPC_OFF),
        ("", cpu_shield.MPC_ON),
        ("true", cpu_shield.MPC_ON),
        ("1", cpu_shield.MPC_ON),
        (None, cpu_shield.MPC_ON),
    ],
)
def test_only_an_explicit_false_opts_out(enable_mpc, expected):
    """Empty means "defer to the controller YAML", and deferring keeps reserving.

    The two errors are not symmetric. Reserving cores nobody uses wastes them;
    failing to reserve cores someone does use puts a SCHED_FIFO thread on a core
    the shield just handed back to the system cpuset. Anything that is not a
    recognised false therefore reserves.
    """
    assert cpu_shield.mpc_layout_profile(enable_mpc) == expected


def test_profile_mapping_survives_an_unresolved_substitution():
    """The robot launches map a ``LaunchConfiguration``, resolved at run time.

    A string-only contract would pass every sim test and fail on exactly the two
    launches that cannot resolve the value while building their description.
    """
    profile = cpu_shield.mpc_layout_profile(LaunchConfiguration("enable_mpc"))

    assert isinstance(profile, PythonExpression)
    # Both outcomes have to be reachable from the emitted expression, otherwise
    # it collapses to a constant and the launch argument stops mattering.
    expression = "".join(
        part.text if hasattr(part, "text") else str(part) for part in profile.expression
    )
    assert cpu_shield.MPC_OFF in expression
    assert cpu_shield.MPC_ON in expression


@pytest.mark.parametrize("profile", [cpu_shield.MPC_ON, cpu_shield.MPC_OFF])
def test_shield_passes_the_profile_as_a_positional_argument(profile):
    """The id rides argv, so both launch styles share one call shape."""
    action = cpu_shield.enable_cpu_shield("--robot", layout_profile=profile)

    assert action.cmd[3][0].text == "rtc_cpu_shield"  # $0 for bash -c
    assert action.cmd[4][0].text == profile


def test_shield_accepts_an_unresolved_profile():
    action = cpu_shield.enable_cpu_shield(
        "--robot", layout_profile=cpu_shield.mpc_layout_profile(LaunchConfiguration("enable_mpc"))
    )

    assert isinstance(action.cmd[4][0], PythonExpression)


def test_shield_without_a_profile_reserves():
    """Omitting the keyword must not crash — it reserves, like it always did."""
    action = cpu_shield.enable_cpu_shield("--robot")

    assert action.cmd[4][0].text == cpu_shield.MPC_ON


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
