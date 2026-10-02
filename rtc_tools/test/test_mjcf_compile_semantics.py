"""compare_mjcf_urdf must read an MJCF the way MuJoCo compiles it (#686).

The tool parses MJCF as text.  Wherever its reading departs from what MuJoCo
compiles, it reports on a model the simulator never runs — a false mismatch,
or a real one printed with the wrong value.

Every case below is a small MJCF with the values MuJoCo gives it written out
as literals.  Two things are asserted against those literals:

* the tool reads them — no ``mujoco`` needed, so this runs wherever colcon
  runs pytest;
* MuJoCo compiles them — skipped where ``mujoco`` is not importable (colcon's
  ``/usr/bin/python3``; run it from the workspace ``.venv``).

The literal in the middle is what keeps the first half honest on a host that
cannot compile, and the second half is what keeps the literal honest.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

import pytest

from rtc_tools.validation.compare_mjcf_urdf import compare, parse_mjcf

# ═══════════════════════════════════════════════════════════════════════════
# Synthetic MJCF: one body `b` carrying one joint `j`
# ═══════════════════════════════════════════════════════════════════════════

RADIAN = '<compiler angle="radian"/>'
INERTIAL = '<inertial pos="0 0 0" mass="1" diaginertia="0.3 0.2 0.1"/>'


def _mjcf(
    *,
    compiler: str = RADIAN,
    defaults: str = "",
    body: str = "",
    joint: str = "",
    inertial: str = INERTIAL,
    actuators: str = "",
) -> str:
    """Every argument is an XML fragment; the defaults give a bare hinge."""
    return f"""\
<mujoco model="semantics">
  {compiler}
  {defaults}
  <worldbody>
    <body name="b" pos="0.1 0.2 0.3" {body}>
      {inertial}
      <joint name="j" {joint}/>
    </body>
  </worldbody>
  <actuator>
    {actuators}
  </actuator>
</mujoco>
"""


@dataclass(frozen=True)
class Case:
    """An MJCF and what MuJoCo makes of its joint ``j``."""

    xml: str
    #: Torque the actuators can apply to the joint, as ``(lower, upper)``;
    #: ``None`` when nothing limits it.
    torque: tuple[float, float] | None = None
    #: Whether driving the actuators into saturation shows ``torque``.
    measure_torque: bool = True


CASES: dict[str, Case] = {
    # ── joint-side limit: <joint actuatorfrcrange> ──
    "motor_without_any_limit": Case(_mjcf(actuators='<motor joint="j"/>')),
    "joint_actuatorfrcrange": Case(
        _mjcf(joint='actuatorfrcrange="-50 50"', actuators='<motor joint="j"/>'),
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
}

CASE_IDS = sorted(CASES)


def _write(tmp_path, name: str, text: str):
    path = tmp_path / name
    path.write_text(text)
    return path


@pytest.mark.parametrize("name", CASE_IDS)
def test_tool_reads_what_mujoco_compiles(name, tmp_path):
    case = CASES[name]
    _, joints = parse_mjcf(_write(tmp_path, "m.xml", case.xml), {"b"}, {"j"})

    # No limit reads as 0 — the tool has one number for "none" and "zero".
    expected = case.torque[1] if case.torque is not None else 0.0
    assert joints["j"].effort == pytest.approx(expected, abs=1e-12)


# ═══════════════════════════════════════════════════════════════════════════
# What the compiled model says
# ═══════════════════════════════════════════════════════════════════════════


def _interval_scaled(k: float, interval: tuple[float, float]) -> tuple[float, float]:
    lo, hi = k * interval[0], k * interval[1]
    return (lo, hi) if lo <= hi else (hi, lo)


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
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "j")

    assert _compiled_torque_range(model, jid) == _approx_interval(case.torque)


@pytest.mark.parametrize("name", [n for n in CASE_IDS if CASES[n].measure_torque])
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
