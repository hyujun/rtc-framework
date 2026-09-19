"""derive_accel_limits (dynamic_catching S2.5, plan §9) on synthetic models.

The oracles are closed forms, not the tool's own |M|-bound:

* a 1-DoF pendulum: s* = (η τ_max − m g l) / I at the worst angle,
* RNEA over every sign pattern for a 2-link arm (independent code path),
* a MuJoCo pendulum for the mj_inverse check (skipped without ``mujoco`` —
  colcon's /usr/bin/python3 has none; run with the workspace .venv python).
"""

from __future__ import annotations

import hashlib
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

pin = pytest.importorskip("pinocchio")

from rtc_tools.analysis import derive_accel_limits as dal  # noqa: E402

G = 9.81

# Point mass m at distance l from a revolute joint about y; gravity along −z,
# so the gravity torque is m g l |cos q| with q measured from horizontal (x).
PENDULUM_M = 2.0
PENDULUM_L = 0.5
PENDULUM_IC = 0.01  # about the COM, so I = m l² + Ic about the joint


def pendulum_urdf() -> str:
    return f"""<?xml version="1.0"?>
<robot name="pendulum">
  <link name="base"/>
  <link name="bar">
    <inertial>
      <origin xyz="{PENDULUM_L} 0 0"/>
      <mass value="{PENDULUM_M}"/>
      <inertia ixx="{PENDULUM_IC}" iyy="{PENDULUM_IC}" izz="{PENDULUM_IC}"
               ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="bar"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5" upper="1.5" effort="50" velocity="2"/>
  </joint>
</robot>
"""


def two_link_urdf() -> str:
    def link(name: str, x: float, m: float) -> str:
        return f"""  <link name="{name}">
    <inertial><origin xyz="{x} 0 0"/><mass value="{m}"/>
      <inertia ixx="0.02" iyy="0.02" izz="0.02" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>"""

    return f"""<?xml version="1.0"?>
<robot name="two_link">
  <link name="base"/>
{link("l1", 0.2, 3.0)}
{link("l2", 0.15, 1.5)}
{link("hand", 0.05, 0.3)}
  <joint name="j1" type="revolute"><parent link="base"/><child link="l1"/>
    <axis xyz="0 1 0"/><limit lower="-2.5" upper="2.5" effort="80" velocity="2"/></joint>
  <joint name="j2" type="revolute"><parent link="l1"/><child link="l2"/>
    <origin xyz="0.8 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="2.5" effort="30" velocity="3"/></joint>
  <joint name="finger" type="revolute"><parent link="l2"/><child link="hand"/>
    <origin xyz="0.3 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="2" velocity="5"/></joint>
</robot>
"""


def write_config(tmp_path: Path, group: str, names, tau, vmax, name="robot.yaml", **extra):
    limits = {"max_torque": list(tau), "max_velocity": list(vmax), **extra}
    doc = {
        "/**": {
            "ros__parameters": {
                "devices": {group: {"joint_state_names": list(names), "joint_limits": limits}}
            }
        }
    }
    p = tmp_path / name
    p.write_text(yaml.safe_dump(doc))
    return p


def run(tmp_path, urdf_text, config, group, *extra) -> tuple[int, dict]:
    urdf = tmp_path / "model.urdf"
    urdf.write_text(urdf_text)
    out = tmp_path / "out.yaml"
    code = dal.main(
        ["--robot-config", str(config), "--group", group, "--urdf", str(urdf), "--out", str(out)]
        + list(extra)
    )
    return code, yaml.safe_load(out.read_text())["derived_accel_limits"][group]


def test_pendulum_matches_closed_form(tmp_path):
    eta, tau_max = 0.8, 50.0
    cfg = write_config(tmp_path, "arm", ["j1"], [tau_max], [2.0])
    code, entry = run(
        tmp_path, pendulum_urdf(), cfg, "arm", "--eta-tau", str(eta), "--samples", "20000"
    )
    assert code == dal.EXIT_OK
    inertia = PENDULUM_M * PENDULUM_L**2 + PENDULUM_IC
    # Worst angle is horizontal (|cos q| = 1, q = 0 is inside ±1.5); a 1-DoF arm
    # has no velocity-product term, so s* = (η τ − m g l) / I.
    exact = (eta * tau_max - PENDULUM_M * G * PENDULUM_L) / inertia
    got = entry["qdd_max"][0]
    assert got >= exact - 1e-6, "min over samples cannot undercut the true minimum"
    assert got <= exact * 1.001, "20000 samples should find the horizontal pose"
    assert entry["adopted"] is True
    assert entry["provisional"] is True


def test_refinement_reaches_closed_form_with_few_samples(tmp_path):
    """5 random samples land far from the horizontal pose (the sample minimum
    misses the closed form by ~1 %); the bounded local refinement must still
    reach it, and never raise s above the sample minimum."""
    eta, tau_max = 0.8, 50.0
    cfg = write_config(tmp_path, "arm", ["j1"], [tau_max], [2.0])
    _, entry = run(tmp_path, pendulum_urdf(), cfg, "arm", "--eta-tau", str(eta), "--samples", "5")
    prov = entry["provenance"]
    inertia = PENDULUM_M * PENDULUM_L**2 + PENDULUM_IC
    exact = (eta * tau_max - PENDULUM_M * G * PENDULUM_L) / inertia
    assert prov["s_sampled_min"] > exact * (1 + 1e-4), "fixture: samples must miss the optimum"
    assert prov["s_star"] <= prov["s_sampled_min"]
    assert prov["s_star"] == pytest.approx(exact, rel=1e-5)


def test_box_survives_rnea_for_every_sign_pattern(tmp_path):
    cfg = write_config(tmp_path, "arm", ["j1", "j2"], [80.0, 30.0], [2.0, 3.0])
    code, entry = run(
        tmp_path,
        two_link_urdf(),
        cfg,
        "arm",
        "--eta-tau",
        "0.8",
        "--samples",
        "3000",
        "--check",
        "rnea",
        "--check-states",
        "300",
    )
    assert code == dal.EXIT_OK
    check = entry["cross_checks"][0]
    assert check["method"] == "rnea"
    assert check["patterns"] == 4
    assert check["pass"] and check["worst_ratio"] <= 1.0
    # The hand joint is outside the arm group: its coupling is reported, not bounded.
    assert entry["provenance"]["max_abs_M_arm_hand"] > 0.0


def test_bound_is_tight_at_the_worst_sign_pattern():
    """At q̇ = 0 the |M|-bound is attained: for the binding row b, the pattern
    σ_j = sign(M_bj)·sign(g_b) makes RNEA give exactly |τ_b| = η τ_max,b. A
    signed-M (instead of |M|) bound would overshoot there. q2 = 2.5 rad makes
    the off-diagonal M_12 negative so the two differ."""
    model = pin.buildModelFromXML(two_link_urdf())
    data = model.createData()
    idx = dal.arm_index(model, ["j1", "j2"])
    tau_max = np.array([80.0, 30.0])
    eta = 0.8
    q = pin.neutral(model)
    q[idx.q] = [0.3, 2.5]
    v = np.zeros(model.nv)
    weights = np.ones(2)
    s, b, _, _ = dal.per_state_scale(model, data, q, v, idx, eta * tau_max, weights)
    m_full = pin.crba(model, data, q)
    m_full = np.triu(m_full) + np.triu(m_full, 1).T
    assert m_full[idx.v[0], idx.v[1]] < 0.0, "fixture must have a negative off-diagonal"
    g = pin.computeGeneralizedGravity(model, data, q)
    sigma = np.sign(m_full[idx.v[b], idx.v]) * np.sign(g[idx.v[b]])
    a = np.zeros(model.nv)
    a[idx.v] = sigma * s * weights
    tau = pin.rnea(model, data, q, v, a)
    assert abs(tau[idx.v[b]]) == pytest.approx(eta * tau_max[b], rel=1e-9)


def test_degenerate_budget_is_not_adopted(tmp_path):
    # η τ below the pendulum's gravity torque m g l ≈ 9.8 N·m.
    cfg = write_config(tmp_path, "arm", ["j1"], [50.0], [2.0])
    code, entry = run(
        tmp_path, pendulum_urdf(), cfg, "arm", "--eta-tau", "0.1", "--samples", "2000"
    )
    assert code == dal.EXIT_DEGENERATE
    assert entry["adopted"] is False
    assert entry["provenance"]["infeasible_ratio"] > 0.0
    assert any("tau_dyn <= 0" in r for r in entry["degenerate_reasons"])


def test_min_accel_floor_is_degenerate(tmp_path):
    cfg = write_config(tmp_path, "arm", ["j1"], [50.0], [2.0])
    code, entry = run(
        tmp_path,
        pendulum_urdf(),
        cfg,
        "arm",
        "--eta-tau",
        "0.8",
        "--samples",
        "500",
        "--min-accel",
        "1e6",
    )
    assert code == dal.EXIT_DEGENERATE
    assert entry["adopted"] is False


def test_tau_max_weights_scale_the_box(tmp_path):
    cfg = write_config(tmp_path, "arm", ["j1", "j2"], [80.0, 30.0], [2.0, 3.0])
    _, entry = run(
        tmp_path,
        two_link_urdf(),
        cfg,
        "arm",
        "--eta-tau",
        "0.8",
        "--samples",
        "500",
        "--weights",
        "tau_max",
    )
    a = np.asarray(entry["qdd_max"])
    assert a[1] / a[0] == pytest.approx(30.0 / 80.0, rel=1e-4)
    assert entry["provenance"]["weight_mode"] == "tau_max"


def test_provenance_records_inputs(tmp_path):
    cfg = write_config(
        tmp_path,
        "arm",
        ["j1", "j2"],
        [80.0, 30.0],
        [2.0, 3.0],
        position_lower=[-1.0, -3.0],
        position_upper=[1.0, 3.0],
    )
    text = two_link_urdf()
    _, entry = run(
        tmp_path, text, cfg, "arm", "--eta-tau", "0.9", "--samples", "400", "--seed", "7"
    )
    prov = entry["provenance"]
    assert prov["urdf_sha256"] == hashlib.sha256(text.encode()).hexdigest()
    assert prov["joint_names"] == ["j1", "j2"]
    assert prov["eta_tau"] == 0.9
    assert prov["samples"] == 400 and prov["seed"] == 7
    assert sum(prov["binding_count"].values()) == 400
    # Sampling box = URDF (±2.5) ∩ robot config ([-1, 1], [-3, 3]).
    assert prov["q_range"]["lower"] == [-1.0, -2.5]
    assert prov["q_range"]["upper"] == [1.0, 2.5]
    assert prov["qd_range_abs"] == [2.0, 3.0]
    assert math.isfinite(prov["s_star"])


def test_later_config_file_overrides(tmp_path):
    base = write_config(tmp_path, "arm", ["j1"], [50.0], [2.0], name="base.yaml")
    over = write_config(tmp_path, "arm", ["j1"], [40.0], [1.0], name="over.yaml")
    params = dal.load_robot_params([base, over])
    spec = dal.arm_spec_from_params(params, "arm")
    assert spec.tau_max.tolist() == [40.0]
    assert spec.v_max.tolist() == [1.0]


def test_rejects_bad_inputs(tmp_path):
    cfg = write_config(tmp_path, "arm", ["j1"], [50.0], [2.0])
    with pytest.raises(SystemExit):
        run(tmp_path, pendulum_urdf(), cfg, "nope", "--eta-tau", "0.8")
    with pytest.raises(SystemExit):
        run(tmp_path, pendulum_urdf(), cfg, "arm", "--eta-tau", "1.5")
    bad = write_config(tmp_path, "arm", ["not_a_joint"], [50.0], [2.0], name="bad.yaml")
    with pytest.raises(SystemExit):
        run(tmp_path, pendulum_urdf(), bad, "arm", "--eta-tau", "0.8")


# ── MuJoCo cross-check (needs the .venv interpreter) ──────────────────────────

PENDULUM_MJCF = f"""
<mujoco>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body name="bar">
      <joint name="j1" type="hinge" axis="0 1 0" range="-1.5 1.5" armature="0.0"/>
      <inertial pos="{PENDULUM_L} 0 0" mass="{PENDULUM_M}"
                diaginertia="{PENDULUM_IC} {PENDULUM_IC} {PENDULUM_IC}"/>
      <geom type="sphere" size="0.02" pos="{PENDULUM_L} 0 0" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="j1" forcelimited="true" forcerange="LO HI" gear="GEAR"/>
  </actuator>
</mujoco>
"""


# The derived box needs up to η τ_max = 40 N·m of joint torque in both
# directions. Joint torque = gear · actuator force, per direction.
@pytest.mark.parametrize(
    ("lo", "hi", "gear", "expect_pass"),
    [
        (-50.0, 50.0, 1.0, True),
        (-5.0, 5.0, 1.0, False),
        (-5.0, 50.0, 1.0, False),  # asymmetric: too weak in the negative direction
        (-5.0, 5.0, 10.0, True),  # a 10:1 gear makes 5 N of force 50 N·m
        (-50.0, 50.0, 0.1, False),  # a 1:10 gear leaves 5 N·m
    ],
)
def test_mujoco_check_uses_geared_directional_range(tmp_path, lo, hi, gear, expect_pass):
    pytest.importorskip("mujoco")
    mjcf = tmp_path / "pendulum.xml"
    mjcf.write_text(
        PENDULUM_MJCF.replace("LO", str(lo)).replace("HI", str(hi)).replace("GEAR", str(gear))
    )
    cfg = write_config(tmp_path, "arm", ["j1"], [50.0], [2.0])
    code, entry = run(
        tmp_path,
        pendulum_urdf(),
        cfg,
        "arm",
        "--eta-tau",
        "0.8",
        "--samples",
        "2000",
        "--check",
        "mujoco",
        "--mjcf",
        str(mjcf),
        "--check-states",
        "50",
    )
    check = entry["cross_checks"][0]
    assert check["pass"] is expect_pass
    assert code == (dal.EXIT_OK if expect_pass else dal.EXIT_CHECK_FAILED)
    assert entry["adopted"] is expect_pass
    assert "contact" in check["disabled"]
