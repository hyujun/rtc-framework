"""compare_mjcf_urdf must read an MJCF the way MuJoCo compiles it (#686).

The tool parses MJCF as text.  Wherever its reading departs from what MuJoCo
compiles, it reports on a model the simulator never runs — a false mismatch,
or a real one printed with the wrong value.

Every synthetic case below is a small MJCF with the values MuJoCo gives it
written out as literals.  Two things are asserted against those literals:

* the tool reads them — no ``mujoco`` needed, so this runs wherever colcon
  runs pytest;
* MuJoCo compiles them — skipped where ``mujoco`` is not importable (colcon's
  ``/usr/bin/python3``; run it from the workspace ``.venv``).

The literal in the middle is what keeps the first half honest on a host that
cannot compile, and the second half is what keeps the literal honest.

The last section drops the literals and compares the tool with the compiled
model directly, joint by joint, on the MJCF files this repository ships.
"""

from __future__ import annotations

import itertools
import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import pytest

from rtc_tools.validation.compare_mjcf_urdf import (
    _compute_mjcf_world_frames,
    _detect_joint_types,
    compare,
    main,
    parse_mjcf,
)

# ═══════════════════════════════════════════════════════════════════════════
# Synthetic MJCF: one body `b` carrying one joint `j`
# ═══════════════════════════════════════════════════════════════════════════

RADIAN = '<compiler angle="radian"/>'
INERTIAL = '<inertial pos="0 0 0" mass="1" diaginertia="0.3 0.2 0.1"/>'


def _mjcf(
    *,
    compiler: str = RADIAN,
    defaults: str = "",
    wrap: str = "{}",
    body: str = "",
    joint: str = "",
    inertial: str = INERTIAL,
    actuators: str = "",
) -> str:
    """Every argument is an XML fragment; the defaults give a bare hinge.

    ``wrap`` nests the body: ``'<body name="g" childclass="c1">{}</body>'``.
    """
    body_xml = f"""\
<body name="b" pos="0.1 0.2 0.3" {body}>
      {inertial}
      <joint name="j" {joint}/>
    </body>"""
    return f"""\
<mujoco model="semantics">
  {compiler}
  {defaults}
  <worldbody>
    {wrap.format(body_xml)}
  </worldbody>
  <actuator>
    {actuators}
  </actuator>
</mujoco>
"""


def _classes(**classes: str) -> str:
    """Sibling default classes under an empty main: ``_classes(c1="<joint .../>")``."""
    inner = "".join(f'<default class="{name}">{body}</default>' for name, body in classes.items())
    return f"<default>{inner}</default>"


@dataclass(frozen=True)
class Case:
    """An MJCF and what MuJoCo makes of its joint ``j``."""

    xml: str
    #: Enforced position limits, radians or metres; ``(0, 0)`` when unlimited.
    range: tuple[float, float] = (0.0, 0.0)
    #: Torque the actuators can apply to the joint, as ``(lower, upper)``;
    #: ``None`` when nothing limits it.
    torque: tuple[float, float] | None = None
    armature: float = 0.0
    #: Joint axis and anchor in the world frame (the body is not rotated, so
    #: the axis is also the one in the body frame).
    axis: tuple[float, float, float] = (0.0, 0.0, 1.0)
    anchor: tuple[float, float, float] = (0.1, 0.2, 0.3)
    hinge: bool = True
    #: The body sits under a <frame>.  The tool does not follow frame poses,
    #: so it must have NO world frame for the joint rather than a wrong one.
    under_frame: bool = False
    #: The actuator force depends on the joint state, so no control input
    #: drives it to a fixed value.
    state_dependent_force: bool = False


MOTOR = '<motor joint="j"/>'

CASES: dict[str, Case] = {
    # ── joint-side limit: <joint actuatorfrcrange> ──
    "motor_without_any_limit": Case(_mjcf(actuators=MOTOR)),
    "joint_actuatorfrcrange": Case(
        _mjcf(joint='actuatorfrcrange="-50 50"', actuators=MOTOR),
        torque=(-50.0, 50.0),
    ),
    # MuJoCo enforces the actuator's and the joint's bound; the smaller wins.
    # Two fixtures, because either "the joint side wins" or "the actuator side
    # wins" passes one of them.
    "actuator_bound_below_joint_bound": Case(
        _mjcf(
            joint='actuatorfrcrange="-50 50"',
            actuators='<motor joint="j" forcerange="-40 40"/>',
        ),
        torque=(-40.0, 40.0),
    ),
    "joint_bound_below_actuator_bound": Case(
        _mjcf(
            joint='actuatorfrcrange="-50 50"',
            actuators='<motor joint="j" forcerange="-60 60"/>',
        ),
        torque=(-50.0, 50.0),
    ),
    # The joint range is the smaller one here, so reading it despite
    # actuatorfrclimited="false" shows as 30 instead of 40.
    "actuatorfrclimited_false": Case(
        _mjcf(
            joint='actuatorfrcrange="-30 30" actuatorfrclimited="false"',
            actuators='<motor joint="j" forcerange="-40 40"/>',
        ),
        torque=(-40.0, 40.0),
    ),
    # The limit is a property of the joint: it stands with no actuator at all.
    "actuatorless_joint_keeps_its_own_force_range": Case(
        _mjcf(joint='actuatorfrcrange="-50 50"'),
        torque=(-50.0, 50.0),
    ),
    # ── default classes ──
    # The top-level <default> is "main": it reaches a joint that names no class.
    "main_default_reaches_a_classless_joint": Case(
        _mjcf(
            defaults='<default><joint armature="0.3" range="-1 1" axis="1 0 0"'
            ' actuatorfrcrange="-9 9"/></default>',
            actuators=MOTOR,
        ),
        range=(-1.0, 1.0),
        torque=(-9.0, 9.0),
        armature=0.3,
        axis=(1.0, 0.0, 0.0),
    ),
    # A nested class is NOT main.  Nothing in c1 applies here — neither to the
    # joint nor to the actuator, both of which name no class.
    "nested_class_does_not_reach_a_classless_joint": Case(
        _mjcf(
            defaults=_classes(
                c1='<joint armature="0.1" range="-1 1" axis="1 0 0" type="slide"'
                ' actuatorfrcrange="-5 5"/><general forcerange="-9 9"/>'
            ),
            actuators=MOTOR,
        ),
    ),
    "sibling_class_does_not_leak": Case(
        _mjcf(
            defaults=_classes(
                c1='<joint armature="0.1" range="-1 1" actuatorfrcrange="-5 5"/>',
                c2='<joint damping="1"/>',
            ),
            joint='class="c2"',
            actuators=MOTOR,
        ),
    ),
    # c3 sets nothing of its own.  Everything comes down the chain main → c2 →
    # c3, past a sibling (c1) that a "first nested class" shortcut would pick.
    "class_inherits_down_its_parent_chain": Case(
        _mjcf(
            defaults='<default><joint axis="1 0 0"/>'
            '<default class="c1"><joint armature="0.1" range="-1 1"/></default>'
            '<default class="c2"><joint armature="0.3" range="-2 2"/>'
            '<motor forcerange="-22 22"/>'
            '<default class="c3"><joint damping="2"/></default></default></default>',
            joint='class="c3"',
            actuators='<motor joint="j" class="c3"/>',
        ),
        range=(-2.0, 2.0),
        torque=(-22.0, 22.0),
        armature=0.3,
        axis=(1.0, 0.0, 0.0),
    ),
    # A nested class copies its parent AFTER all of the parent's own elements,
    # even one written below the nested block.
    "nested_class_sees_parent_elements_written_after_it": Case(
        _mjcf(
            defaults='<default><default class="c1"><joint damping="1"/></default>'
            '<joint armature="0.4"/></default>',
            joint='class="c1"',
        ),
        armature=0.4,
    ),
    "body_childclass": Case(
        _mjcf(
            defaults=_classes(
                c1='<joint damping="1"/>',
                c2='<joint armature="0.2" axis="0 1 0" actuatorfrcrange="-22 22"/>',
            ),
            body='childclass="c2"',
            actuators=MOTOR,
        ),
        torque=(-22.0, 22.0),
        armature=0.2,
        axis=(0.0, 1.0, 0.0),
    ),
    "childclass_of_an_ancestor_body": Case(
        _mjcf(
            defaults=_classes(c1='<joint armature="0.1" range="-1 1" type="slide"/>'),
            wrap='<body name="g" childclass="c1">{}</body>',
        ),
        range=(-1.0, 1.0),
        armature=0.1,
        hinge=False,
    ),
    "childclass_of_a_frame": Case(
        _mjcf(
            defaults=_classes(c1='<joint armature="0.1" range="-1 1"/>'),
            wrap='<frame childclass="c1">{}</frame>',
        ),
        range=(-1.0, 1.0),
        armature=0.1,
        under_frame=True,
    ),
    "own_class_beats_childclass": Case(
        _mjcf(
            defaults=_classes(c1='<joint armature="0.1"/>', c2='<joint armature="0.2"/>'),
            body='childclass="c2"',
            joint='class="c1"',
        ),
        armature=0.1,
    ),
    # axis and pos are defaults like any other joint attribute.
    "joint_pos_and_axis_from_the_class": Case(
        _mjcf(defaults=_classes(c1='<joint pos="0 0 0.1" axis="0 1 0"/>'), joint='class="c1"'),
        axis=(0.0, 1.0, 0.0),
        anchor=(0.1, 0.2, 0.4),
    ),
    "joint_attribute_beats_its_class": Case(
        _mjcf(
            defaults=_classes(c1='<joint armature="0.1"/>'),
            joint='class="c1" armature="0.7"',
        ),
        armature=0.7,
    ),
    # ── actuator classes ──
    "actuator_reads_its_own_class": Case(
        _mjcf(
            defaults=_classes(c1='<general forcerange="-33 33"/>'),
            actuators='<motor joint="j" class="c1"/>',
        ),
        torque=(-33.0, 33.0),
    ),
    "actuator_class_inherits_from_main": Case(
        _mjcf(
            defaults='<default><general forcerange="-33 33"/>'
            '<default class="c1"><joint armature="1"/></default></default>',
            actuators='<motor joint="j" class="c1"/>',
        ),
        torque=(-33.0, 33.0),
    ),
    # Actuators sit outside the body tree: the body's childclass is not theirs.
    "childclass_does_not_reach_the_actuator": Case(
        _mjcf(
            defaults=_classes(c1='<motor forcerange="-9 9"/>'),
            body='childclass="c1"',
            actuators=MOTOR,
        ),
    ),
    # <general> and <motor> in a <default> write the same slot; the later wins.
    "actuator_shortcuts_share_one_default": Case(
        _mjcf(
            defaults=_classes(c1='<general forcerange="-10 10"/><motor forcerange="-20 20"/>'),
            actuators='<general joint="j" class="c1"/>',
        ),
        torque=(-20.0, 20.0),
    ),
    # A class's actuator defaults are not the joint's: with no actuator MuJoCo
    # applies them to nothing.
    "actuatorless_joint_ignores_its_class_forcerange": Case(
        _mjcf(
            defaults=_classes(c1='<general forcerange="-150 150"/>'),
            joint='class="c1"',
        ),
    ),
    # ── gear ──
    # forcerange is in actuator space; the joint feels gear times it.
    "gear_scales_the_actuator_bound": Case(
        _mjcf(actuators='<motor joint="j" forcerange="-40 40" gear="2"/>'),
        torque=(-80.0, 80.0),
    ),
    "gear_from_the_class_default": Case(
        _mjcf(
            defaults=_classes(c1='<general forcerange="-10 10" gear="3"/>'),
            actuators='<motor joint="j" class="c1"/>',
        ),
        torque=(-30.0, 30.0),
    ),
    # ...but the joint's own range is already in joint space.
    "gear_does_not_scale_the_joint_bound": Case(
        _mjcf(
            joint='actuatorfrcrange="-50 50"',
            actuators='<motor joint="j" forcerange="-40 40" gear="2"/>',
        ),
        torque=(-50.0, 50.0),
    ),
    "negative_gear_flips_the_range": Case(
        _mjcf(actuators='<motor joint="j" forcerange="-30 50" gear="-2"/>'),
        torque=(-100.0, 60.0),
    ),
    # ── *limited flags: a range that is not enforced is not a limit ──
    "limited_false": Case(_mjcf(joint='range="-1 1" limited="false"')),
    "limited_false_from_the_class_default": Case(
        _mjcf(defaults='<default><joint limited="false"/></default>', joint='range="-1 1"'),
    ),
    # With autolimits off the flag is the only thing that enforces the range.
    "limited_true_without_autolimits": Case(
        _mjcf(
            compiler='<compiler angle="radian" autolimits="false"/>',
            joint='range="-1 1" limited="true"',
        ),
        range=(-1.0, 1.0),
    ),
    "forcelimited_false": Case(
        _mjcf(actuators='<motor joint="j" forcerange="-40 40" forcelimited="false"/>'),
    ),
    "ctrllimited_false": Case(
        _mjcf(actuators='<motor joint="j" ctrlrange="-4 4" ctrllimited="false"/>'),
    ),
    # ── ctrlrange: a torque bound only on a pure gain ──
    "motor_ctrlrange": Case(
        _mjcf(actuators='<motor joint="j" ctrlrange="-4 4"/>'),
        torque=(-4.0, 4.0),
    ),
    "pure_general_gain_times_ctrlrange": Case(
        _mjcf(actuators='<general joint="j" gainprm="5" ctrlrange="-4 4"/>'),
        torque=(-20.0, 20.0),
    ),
    "motor_ctrlrange_times_gear": Case(
        _mjcf(actuators='<motor joint="j" ctrlrange="-4 4" gear="3"/>'),
        torque=(-12.0, 12.0),
    ),
    "motor_ctrlrange_above_forcerange": Case(
        _mjcf(actuators='<motor joint="j" ctrlrange="-4 4" forcerange="-3 3"/>'),
        torque=(-3.0, 3.0),
    ),
    # A position servo's force is kp * (ctrl - q): the clamp on ctrl says
    # nothing about its size.  Reading ctrlrange here would report 1.
    "position_servo_ctrlrange_is_not_a_torque_bound": Case(
        _mjcf(actuators='<position joint="j" kp="100" ctrlrange="-1 1"/>'),
        state_dependent_force=True,
    ),
    "affine_general_ctrlrange_is_not_a_torque_bound": Case(
        _mjcf(
            actuators='<general joint="j" gainprm="100" biastype="affine"'
            ' biasprm="0 -100 0" ctrlrange="-1 1"/>'
        ),
        state_dependent_force=True,
    ),
    "position_servo_forcerange_still_bounds": Case(
        _mjcf(actuators='<position joint="j" kp="100" ctrlrange="-1 1" forcerange="-7 7"/>'),
        torque=(-7.0, 7.0),
    ),
    # The bias a class default set is part of the actuator unless the element
    # resets it: <motor> does, <general> does not.
    "motor_resets_an_inherited_bias": Case(
        _mjcf(
            defaults=_classes(c1='<position kp="5"/>'),
            actuators='<motor joint="j" class="c1" ctrlrange="-4 4"/>',
        ),
        torque=(-4.0, 4.0),
    ),
    "general_keeps_an_inherited_bias": Case(
        _mjcf(
            defaults=_classes(c1='<position kp="5"/>'),
            actuators='<general joint="j" class="c1" ctrlrange="-4 4"/>',
        ),
        state_dependent_force=True,
    ),
    # ── <compiler angle>: degrees unless stated, and only for angles ──
    "hinge_range_is_degrees_by_default": Case(
        _mjcf(compiler="", joint='range="-90 45"'),
        range=(-math.pi / 2, math.pi / 4),
    ),
    "hinge_range_from_a_class_default_is_degrees_too": Case(
        _mjcf(compiler="", defaults=_classes(c1='<joint range="-90 45"/>'), joint='class="c1"'),
        range=(-math.pi / 2, math.pi / 4),
    ),
    "slide_range_is_a_length": Case(
        _mjcf(compiler="", joint='type="slide" range="-90 45"'),
        range=(-90.0, 45.0),
        hinge=False,
    ),
    "slide_type_from_a_class_default_is_a_length_too": Case(
        _mjcf(
            compiler="",
            defaults=_classes(c1='<joint type="slide" range="-90 45"/>'),
            joint='class="c1"',
        ),
        range=(-90.0, 45.0),
        hinge=False,
    ),
    "force_and_ctrl_ranges_are_never_angles": Case(
        _mjcf(
            compiler="",
            joint='actuatorfrcrange="-50 50"',
            actuators='<motor joint="j" ctrlrange="-90 90"/>',
        ),
        torque=(-50.0, 50.0),
    ),
    "compiler_elements_accumulate": Case(
        _mjcf(
            compiler='<compiler angle="radian"/><compiler autolimits="true"/>',
            joint='range="-1 1"',
        ),
        range=(-1.0, 1.0),
    ),
    # ── asymmetric ranges ──
    "asymmetric_joint_force_range": Case(
        _mjcf(joint='actuatorfrcrange="-30 50"', actuators=MOTOR),
        torque=(-30.0, 50.0),
    ),
    "asymmetric_actuator_forcerange": Case(
        _mjcf(actuators='<motor joint="j" forcerange="-30 50"/>'),
        torque=(-30.0, 50.0),
    ),
    # ── several actuators on one joint: their forces add ──
    "two_actuators_add_up": Case(
        _mjcf(actuators='<motor joint="j" forcerange="-40 40"/>' * 2),
        torque=(-80.0, 80.0),
    ),
    "one_unbounded_actuator_unbounds_the_joint": Case(
        _mjcf(actuators='<motor joint="j" forcerange="-40 40"/>' + MOTOR),
    ),
    "joint_bound_clamps_the_sum": Case(
        _mjcf(
            joint='actuatorfrcrange="-50 50"',
            actuators='<motor joint="j" forcerange="-40 40"/>' * 2,
        ),
        torque=(-50.0, 50.0),
    ),
}

CASE_IDS = sorted(CASES)


def _write(tmp_path, name: str, text: str):
    path = tmp_path / name
    path.write_text(text)
    return path


@pytest.mark.parametrize("name", CASE_IDS)
def test_tool_reads_what_mujoco_compiles(name, tmp_path):
    case = CASES[name]
    mjcf = _write(tmp_path, "m.xml", case.xml)
    _, joints = parse_mjcf(mjcf, {"b"}, {"j"})
    jp = joints["j"]

    assert (jp.lower, jp.upper) == pytest.approx(case.range, abs=1e-12)
    # No limit reads as 0 — the tool has one number for "none" and "zero".
    assert (jp.effort_lower, jp.effort) == pytest.approx(case.torque or (0.0, 0.0), abs=1e-12)
    assert jp.armature == pytest.approx(case.armature, abs=1e-12)

    assert jp.axis == pytest.approx(case.axis, abs=1e-12)
    frames = _compute_mjcf_world_frames(mjcf, {"j"})
    if case.under_frame:
        assert "j" not in frames
    else:
        anchor, axis = frames["j"]
        assert axis == pytest.approx(case.axis, abs=1e-12)
        assert anchor == pytest.approx(case.anchor, abs=1e-12)

    urdf = _write(tmp_path, "m.urdf", _urdf())
    kind = _detect_joint_types(mjcf, urdf, {"j"})["j"]
    assert kind == ("revolute" if case.hinge else "other")


# ═══════════════════════════════════════════════════════════════════════════
# What the compiled model says
# ═══════════════════════════════════════════════════════════════════════════


def _interval_scaled(k: float, interval: tuple[float, float]) -> tuple[float, float]:
    lo, hi = k * interval[0], k * interval[1]
    return (lo, hi) if lo <= hi else (hi, lo)


def _compiled_range(model, joint_id: int) -> tuple[float, float]:
    if not model.jnt_limited[joint_id]:
        return (0.0, 0.0)
    return tuple(float(v) for v in model.jnt_range[joint_id])


def _compiled_torque_range(model, joint_id: int) -> tuple[float, float] | None:
    """Torque limit of a joint, from the compiled model's fields.

    For each actuator attached to the joint: its ``forcerange`` if
    ``forcelimited``; for a pure gain (fixed gain, no bias, no dynamics) also
    ``gain * ctrlrange`` if ``ctrllimited``; both carried to the joint by
    ``gear``.  The actuators add up.  The joint's ``actuatorfrcrange`` clamps
    the sum if ``actuatorfrclimited``.  ``None`` when nothing limits it.
    """
    import mujoco

    lo, hi = -math.inf, math.inf
    attached = [
        a
        for a in range(model.nu)
        if model.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT
        and model.actuator_trnid[a, 0] == joint_id
    ]
    if attached:
        lo = hi = 0.0
    for a in attached:
        a_lo, a_hi = -math.inf, math.inf
        if model.actuator_forcelimited[a]:
            a_lo, a_hi = (float(v) for v in model.actuator_forcerange[a])
        pure_gain = (
            model.actuator_gaintype[a] == mujoco.mjtGain.mjGAIN_FIXED
            and model.actuator_biastype[a] == mujoco.mjtBias.mjBIAS_NONE
            and model.actuator_dyntype[a] == mujoco.mjtDyn.mjDYN_NONE
        )
        if pure_gain and model.actuator_ctrllimited[a]:
            c_lo, c_hi = _interval_scaled(
                float(model.actuator_gainprm[a, 0]),
                tuple(float(v) for v in model.actuator_ctrlrange[a]),
            )
            a_lo, a_hi = max(a_lo, c_lo), min(a_hi, c_hi)
        a_lo, a_hi = _interval_scaled(float(model.actuator_gear[a, 0]), (a_lo, a_hi))
        lo, hi = lo + a_lo, hi + a_hi
    if model.jnt_actfrclimited[joint_id]:
        j_lo, j_hi = (float(v) for v in model.jnt_actfrcrange[joint_id])
        lo, hi = max(lo, j_lo), min(hi, j_hi)
    return (lo, hi) if math.isfinite(lo) and math.isfinite(hi) else None


#: Far past every range in CASES, so each actuator is driven into its clamps.
_SATURATING_CTRL = 1.0e6
#: A torque this large was not clamped by anything.
_UNLIMITED_TORQUE = 1.0e5


def _measured_torque_range(model, joint_id: int) -> tuple[float, float] | None:
    """Extremes of the actuator torque on a joint over saturating controls.

    Independent of ``_compiled_torque_range``: nothing here reads a range or
    a gear, MuJoCo's own forward pass applies them.
    """
    import mujoco

    data = mujoco.MjData(model)
    dof = model.jnt_dofadr[joint_id]
    seen = []
    for signs in itertools.product((-1.0, 1.0), repeat=model.nu):
        data.ctrl[:] = [s * _SATURATING_CTRL for s in signs]
        mujoco.mj_forward(model, data)
        seen.append(float(data.qfrc_actuator[dof]))
    lo, hi = min(seen), max(seen)
    return None if max(-lo, hi) > _UNLIMITED_TORQUE else (lo, hi)


def _approx_interval(interval: tuple[float, float] | None):
    return None if interval is None else pytest.approx(interval, abs=1e-9)


@pytest.mark.parametrize("name", CASE_IDS)
def test_mujoco_compiles_the_expected_values(name):
    mujoco = pytest.importorskip("mujoco")
    case = CASES[name]
    model = mujoco.MjModel.from_xml_string(case.xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "j")

    assert _compiled_range(model, jid) == pytest.approx(case.range, abs=1e-9)
    assert _compiled_torque_range(model, jid) == _approx_interval(case.torque)
    assert float(model.dof_armature[model.jnt_dofadr[jid]]) == pytest.approx(case.armature)
    assert [float(v) for v in data.xaxis[jid]] == pytest.approx(case.axis, abs=1e-9)
    assert [float(v) for v in data.xanchor[jid]] == pytest.approx(case.anchor, abs=1e-9)
    assert (model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_HINGE) == case.hinge


#: Cases whose limit a saturating control can show: an actuator drives the
#: joint, and its force does not depend on the joint state.
MEASURABLE_IDS = [
    name
    for name in CASE_IDS
    if 'joint="j"' in CASES[name].xml.split("<actuator>")[1]
    and not CASES[name].state_dependent_force
]


@pytest.mark.parametrize("name", MEASURABLE_IDS)
def test_mujoco_applies_the_expected_torque(name):
    """The limit is not only a field of the model — it is the torque MuJoCo
    stops at.  This is what makes ``_compiled_torque_range`` usable as the
    oracle on real models, where the fields are all there is to read."""
    mujoco = pytest.importorskip("mujoco")
    case = CASES[name]
    model = mujoco.MjModel.from_xml_string(case.xml)
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "j")

    assert _measured_torque_range(model, jid) == _approx_interval(case.torque)


# ═══════════════════════════════════════════════════════════════════════════
# The reading reaches the report
# ═══════════════════════════════════════════════════════════════════════════

URDF_INERTIA = 'ixx="0.3" iyy="0.2" izz="0.1" ixy="0" ixz="0" iyz="0"'


def _urdf(*, effort: float = 0.0, inertia: str = URDF_INERTIA) -> str:
    """The URDF counterpart of ``_mjcf()``'s body and joint."""
    return f"""\
<robot name="semantics">
  <link name="world"/>
  <link name="b">
    <inertial>
      <mass value="1"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia {inertia}/>
    </inertial>
  </link>
  <joint name="j" type="revolute">
    <parent link="world"/>
    <child link="b"/>
    <origin xyz="0.1 0.2 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="0" effort="{effort}" velocity="1"/>
  </joint>
</robot>
"""


def _report(tmp_path, capsys, mjcf_text: str, urdf_text: str) -> str:
    compare(
        _write(tmp_path, "m.xml", mjcf_text),
        _write(tmp_path, "m.urdf", urdf_text),
        link_map={"b": "b"},
        joint_names=["j"],
    )
    return capsys.readouterr().out


class TestJointForceRangeInTheReport:
    MJCF = CASES["joint_actuatorfrcrange"].xml

    def test_a_real_difference_is_reported_with_the_value_mujoco_enforces(self, tmp_path, capsys):
        """50 against a URDF 35 is a mismatch before and after the joint range
        is read — what changes is the number printed (it used to be 0)."""
        out = _report(tmp_path, capsys, self.MJCF, _urdf(effort=35))
        assert "EFFORT MISMATCH:  MJCF=50  URDF=35" in out

    def test_equal_limits_are_not_a_mismatch(self, tmp_path, capsys):
        out = _report(tmp_path, capsys, self.MJCF, _urdf(effort=50))
        assert "effort: 50  OK" in out
        assert "EFFORT MISMATCH" not in out


class TestAsymmetricForceRangeInTheReport:
    """A URDF effort bounds both directions alike.  An MJCF range that does
    not is a difference whichever end the URDF value happens to equal."""

    MJCF = CASES["asymmetric_joint_force_range"].xml  # -30 .. 50

    @pytest.mark.parametrize("urdf_effort", [50, 30])
    def test_it_is_a_mismatch_against_either_end(self, tmp_path, capsys, urdf_effort):
        out = _report(tmp_path, capsys, self.MJCF, _urdf(effort=urdf_effort))
        assert f"EFFORT MISMATCH:  MJCF=[-30, 50] (asymmetric)  URDF={urdf_effort}" in out
        assert "effort: " not in out

    def test_a_symmetric_range_keeps_the_single_value_report(self, tmp_path, capsys):
        out = _report(tmp_path, capsys, CASES["joint_actuatorfrcrange"].xml, _urdf(effort=50))
        assert "asymmetric" not in out


class TestActuatorlessJointInTheReport:
    def test_a_urdf_effort_with_no_actuator_behind_it_is_a_mismatch(self, tmp_path, capsys):
        """The class says 150 and so does the URDF — but nothing drives the
        joint in the simulator, and the report has to say that."""
        mjcf = CASES["actuatorless_joint_ignores_its_class_forcerange"].xml
        out = _report(tmp_path, capsys, mjcf, _urdf(effort=150))
        assert "EFFORT MISMATCH:  MJCF=0  URDF=150" in out


def test_the_mjcf_class_option_is_gone(tmp_path, capsys, monkeypatch):
    """There is no "root class" to pick once the default tree is read as a
    tree; a value other than the old auto-detected one only ever misread it."""
    monkeypatch.setattr("sys.argv", ["compare_mjcf_urdf", "--mjcf-class", "c1"])
    with pytest.raises(SystemExit) as exit_info:
        main()
    assert exit_info.value.code == 2
    assert "unrecognized arguments: --mjcf-class" in capsys.readouterr().err


# ═══════════════════════════════════════════════════════════════════════════
# <inertial fullinertia>
# ═══════════════════════════════════════════════════════════════════════════

# ixx iyy izz ixy ixz iyz.  The off-diagonals are non-zero and all different:
# on an axis-aligned tensor a reader that takes the first three numbers, or
# takes the six in row-major order, would still land on the right moments.
FULLINERTIA = "0.5 0.4 0.3 0.05 -0.03 0.02"
#: Eigenvalues of that tensor, descending.
FULLINERTIA_MOMENTS = (0.5225715114, 0.3890349006, 0.2883935880)
MJCF_FULLINERTIA = _mjcf(inertial=f'<inertial pos="0 0 0" mass="1" fullinertia="{FULLINERTIA}"/>')
URDF_SAME_TENSOR = 'ixx="0.5" iyy="0.4" izz="0.3" ixy="0.05" ixz="-0.03" iyz="0.02"'


class TestFullInertia:
    def test_tool_reads_the_principal_moments(self, tmp_path):
        pytest.importorskip("numpy")  # the eigenvalues need it, on both sides
        links, _ = parse_mjcf(_write(tmp_path, "m.xml", MJCF_FULLINERTIA), {"b"}, {"j"})
        assert links["b"].diag_inertia == pytest.approx(FULLINERTIA_MOMENTS, abs=1e-9)

    def test_mujoco_compiles_the_same_moments(self):
        mujoco = pytest.importorskip("mujoco")
        model = mujoco.MjModel.from_xml_string(MJCF_FULLINERTIA)
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "b")
        compiled = sorted((float(v) for v in model.body_inertia[bid]), reverse=True)
        assert compiled == pytest.approx(FULLINERTIA_MOMENTS, abs=1e-9)

    def test_the_same_tensor_on_both_sides_matches(self, tmp_path, capsys):
        pytest.importorskip("numpy")
        out = _report(tmp_path, capsys, MJCF_FULLINERTIA, _urdf(inertia=URDF_SAME_TENSOR))
        assert "principal moments: [0.522572, 0.389035, 0.288394]  OK" in out
        assert "INERTIA MISMATCH" not in out

    def test_a_different_tensor_is_reported_with_the_moments_mujoco_carries(
        self, tmp_path, capsys
    ):
        """It used to print ``MJCF=[0, 0, 0]`` — and to pass whenever the URDF
        moments were under the tolerance."""
        pytest.importorskip("numpy")
        out = _report(tmp_path, capsys, MJCF_FULLINERTIA, _urdf())
        assert "INERTIA MISMATCH" in out
        assert "MJCF=[0.522572, 0.389035, 0.288394]" in out


# ═══════════════════════════════════════════════════════════════════════════
# The MJCF files this repository ships: tool == compiled model, joint by joint
#
# No literals here — the compiled model is the oracle.  A file enters by having
# a named <joint> in its own text; what a file only pulls in through <include>
# the tool does not read, so there is nothing of the tool's to check.
# ═══════════════════════════════════════════════════════════════════════════


def _robot_descriptions_dir() -> Path | None:
    """In-source first: these are the files the repository owns."""
    in_source = Path(__file__).resolve().parents[2] / "robot_descriptions"
    if in_source.is_dir():
        return in_source
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("robot_descriptions"))
    except Exception:
        return None


def _text_joint_names(path: Path) -> set[str]:
    """Named <joint> elements under <worldbody>, or empty if not an MJCF."""
    try:
        root = ET.parse(path).getroot()
    except (ET.ParseError, OSError):
        return set()
    if root.tag != "mujoco":
        return set()
    return {
        joint.get("name")
        for worldbody in root.findall("worldbody")
        for joint in worldbody.iter("joint")
        if joint.get("name")
    }


def _shipped_mjcf_with_joints() -> list[Path]:
    base = _robot_descriptions_dir()
    if base is None:
        return []
    found = []
    # os.walk, not rglob: a symlink install exposes directories as links.
    for dirpath, _dirs, files in os.walk(base, followlinks=True):
        for filename in files:
            path = Path(dirpath) / filename
            if path.suffix == ".xml" and _text_joint_names(path):
                found.append(path)
    return sorted(found)


SHIPPED_ROOT = _robot_descriptions_dir()
SHIPPED_MJCF = _shipped_mjcf_with_joints()


def test_shipped_mjcf_files_were_found():
    """Zero files would make every test below vacuous rather than red."""
    assert SHIPPED_ROOT is not None, "robot_descriptions not found"
    assert SHIPPED_MJCF, f"no MJCF with a named <joint> under {SHIPPED_ROOT}"


def _tool_reading(quantity: str, jp):
    if quantity == "range":
        return (jp.lower, jp.upper)
    if quantity == "torque":
        # The tool spells "no limit" as (0, 0).
        return (jp.effort_lower, jp.effort)
    return jp.armature


def _compiled_reading(quantity: str, model, joint_id: int):
    if quantity == "range":
        return _compiled_range(model, joint_id)
    if quantity == "torque":
        return _compiled_torque_range(model, joint_id) or (0.0, 0.0)
    return float(model.dof_armature[model.jnt_dofadr[joint_id]])


@pytest.mark.parametrize("quantity", ["range", "torque", "armature"])
@pytest.mark.parametrize(
    "path", SHIPPED_MJCF, ids=[str(p.relative_to(SHIPPED_ROOT)) for p in SHIPPED_MJCF]
)
def test_shipped_mjcf_reads_as_compiled(path, quantity):
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(path))
    _, joints = parse_mjcf(path, set(), _text_joint_names(path))

    compared = 0
    differing = []
    for jid in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
        if name not in joints or model.jnt_type[jid] not in (
            mujoco.mjtJoint.mjJNT_HINGE,
            mujoco.mjtJoint.mjJNT_SLIDE,
        ):
            continue
        compared += 1
        tool = _tool_reading(quantity, joints[name])
        compiled = _compiled_reading(quantity, model, jid)
        if tool != pytest.approx(compiled, rel=1e-9, abs=1e-12):
            differing.append(f"{name}: tool={tool} compiled={compiled}")

    assert compared, f"{path}: no hinge/slide joint of the text is in the compiled model"
    assert not differing, (
        f"{path}: {quantity} differs from the compiled model on "
        f"{len(differing)} of {compared} joint(s)\n  " + "\n  ".join(differing)
    )
