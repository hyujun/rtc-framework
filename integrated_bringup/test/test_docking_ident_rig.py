"""tools/docking_ident/rig.py on a hand small enough to compute by hand.

**This file needs the ``mujoco`` module and therefore runs nowhere automatic.**
A local ``colcon test`` runs pytest under ``/usr/bin/python3``, which has no
``mujoco`` (it is a pip package of the workspace venv), so every test here
skips there. Run it deliberately after touching the rig::

    .venv/bin/python -m pytest \\
        src/rtc-framework/integrated_bringup/test/test_docking_ident_rig.py

(with the workspace environment sourced — the rig imports ``rtc_tools``).

The hand is a flat palm with one sliding finger on a one-joint arm; a post
that is NOT part of the hand stands beside it. The catch frame's origin is
where the ball rests on the palm, so a ball coming straight down the approach
axis touches the palm exactly when its free flight reaches ``s = 0`` — which
pins the frame, the timing of the arrival and the line the ball is put on.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "integrated_bringup" / "tools" / "docking_ident"))
import rig_config as rc  # noqa: E402

try:
    import mujoco
except ImportError:
    mujoco = None
else:
    import rig as rg

# Every test is SKIPPED without mujoco, not the module: a file that collects
# nothing makes pytest exit 5, which colcon reports as a failed test.
pytestmark = pytest.mark.skipif(
    mujoco is None,
    reason=(
        "mujoco absent — this file is manual-only: colcon runs pytest under /usr/bin/python3. "
        "Run: .venv/bin/python -m pytest <this file>"
    ),
)

RADIUS = 0.0335
PALM_TOP = 0.01  # palm box half-height: its top face in the palm frame
POST = (0.0, 0.3, 0.9)  # world

SCENE = """
<mujoco>
  <option timestep="0.002"{option}/>
  <worldbody>
    <body name="arm_link" pos="0 0 0.5">
      <joint name="arm_j" type="hinge" axis="0 1 0"/>
      <geom name="arm_g" type="capsule" fromto="0 0 0 0 0 0.15" size="0.02"/>
      <body name="palm" pos="0 0 0.2">
        <geom name="palm_g" type="box" size="0.05 0.05 0.01"/>
        {cage}
        <body name="finger" pos="0.08 0 0.03">
          <joint name="finger_j" type="slide" axis="1 0 0" range="-0.05 0.05"/>
          <geom name="finger_g" type="box" size="0.01 0.05 0.01"/>
        </body>
      </body>
    </body>
    <body name="post" pos="{post}">
      <geom name="post_g" type="sphere" size="0.03"/>
    </body>
  </worldbody>
  <actuator>
    <position name="arm_a" joint="arm_j" kp="100" kv="10"/>
    <position name="finger_a" joint="finger_j" kp="200" kv="5"/>{extra_actuator}
  </actuator>
</mujoco>
"""

# Five walls closing the ball in at the catch point, 2 mm clear on every side.
_INNER = RADIUS + 0.002
_Z = PALM_TOP + RADIUS
CAGE = "".join(
    f'<geom name="cage{i}" type="box" pos="{x} {y} {z}" size="{sx} {sy} {sz}"/>'
    for i, (x, y, z, sx, sy, sz) in enumerate(
        [
            (_INNER + 0.004, 0, _Z, 0.004, 0.05, 0.04),
            (-_INNER - 0.004, 0, _Z, 0.004, 0.05, 0.04),
            (0, _INNER + 0.004, _Z, 0.05, 0.004, 0.04),
            (0, -_INNER - 0.004, _Z, 0.05, 0.004, 0.04),
            (0, 0, _Z + _INNER + 0.004, 0.05, 0.05, 0.004),
        ]
    )
)


def make_config(tmp_path, *, option="", cage="", extra_actuator="", **overrides) -> rc.RigConfig:
    scene = tmp_path / "scene.xml"
    scene.write_text(
        SCENE.format(
            option=option,
            cage=cage,
            extra_actuator=extra_actuator,
            post=" ".join(map(str, POST)),
        )
    )
    fields = {
        "profile": "synthetic",
        "model_path": str(scene),
        "control_period": 0.002,
        "n_substeps": 3,
        "solver": rc._ros_parameters(rc.SIM_DEFAULTS)["solver"],
        "use_yaml_servo_gains": True,
        "groups": (
            rc.SimGroup("arm", ("arm_j",), (500.0,), (50.0,)),
            rc.SimGroup("hand", ("finger_j",), (300.0,), (10.0,)),
        ),
        "arm_joints": ("arm_j",),
        "arm_pose": (0.0,),
        "hand_joints": ("finger_j",),
        "q_pre": (0.0,),
        "q_close": (-0.04,),
        "caging_mask": (True,),
        "eta_close": 0.5,
        "t_close_e2e": 0.05,
        "catch_parent": "palm",
        "catch_xyz": (0.0, 0.0, PALM_TOP + RADIUS),
        "catch_rpy": (0.0, 0.0, 0.0),
        "ball": rc.BallConfig("tennis", RADIUS, 0.057, 1, 1),
    }
    fields.update(overrides)
    return rc.RigConfig(**fields)


# ── build_model: the simulator's load-time steps ──────────────────────────────


def _id(model, kind, name):
    return mujoco.mj_name2id(model, kind, name)


def test_build_lays_the_simulator_over_the_model(tmp_path):
    cfg = make_config(tmp_path)
    model, applied = rg.build_model(cfg)
    assert model.opt.timestep == pytest.approx(0.002 / 3)
    assert applied["substep_s"] == pytest.approx(0.002 / 3)

    # Solver: the simulator's defaults, none of which this scene writes.
    assert model.opt.cone == mujoco.mjtCone.mjCONE_ELLIPTIC
    assert model.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    assert model.opt.iterations == cfg.solver["iterations"]
    assert model.opt.tolerance == pytest.approx(cfg.solver["tolerance"])
    assert model.opt.enableflags & int(mujoco.mjtEnableBit.mjENBL_MULTICCD)
    assert model.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_ISLAND)  # island: false
    assert not model.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_WARMSTART)
    assert applied["solver"]["cone"] == "elliptic" and applied["solver"]["multiccd"] is True

    # Servo gains: kp into the gain and the position bias, kd into the velocity bias.
    arm = _id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "arm_a")
    assert model.actuator_gainprm[arm, 0] == 500.0
    assert model.actuator_biasprm[arm, :3].tolist() == [0.0, -500.0, -50.0]
    finger = _id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "finger_a")
    assert model.actuator_biasprm[finger, :3].tolist() == [0.0, -300.0, -10.0]

    # Gravity compensation: the bodies the groups' joints move and the jointless
    # body between them — not the ball, not the post.
    compensated = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
        for b in range(model.nbody)
        if model.body_gravcomp[b] == 1.0
    }
    assert compensated == {"arm_link", "palm", "finger"}
    assert applied["gravcomp_bodies"] == 3 and model.ngravcomp == 3

    # The ball: explicit inertia and the contact the simulator computes.
    geom = _id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{rc.BALL_BODY}_geom")
    body = _id(model, mujoco.mjtObj.mjOBJ_BODY, rc.BALL_BODY)
    inertia_ratio, restitution, sliding, torsional, rolling = rc.BALL_PRESETS["tennis"]
    assert model.body_mass[body] == pytest.approx(0.057)
    assert model.body_inertia[body] == pytest.approx([inertia_ratio * 0.057 * RADIUS**2] * 3)
    omega = math.pi / (rc.CONTACT_SUBSTEPS * 0.002 / 3)
    zeta = rc.damping_ratio_for(restitution)
    assert model.geom_solref[geom] == pytest.approx([-(omega**2), -2 * zeta * omega])
    assert model.geom_solimp[geom] == pytest.approx(rc.BALL_SOLIMP)
    assert model.geom_friction[geom] == pytest.approx(
        [sliding, torsional * RADIUS, rolling * RADIUS]
    )
    assert model.geom_condim[geom] == rc.BALL_CONDIM
    assert model.geom_priority[geom] == rc.BALL_PRIORITY
    assert model.opt.gravity.tolist() == [0.0, 0.0, 0.0]


def test_an_option_the_scene_file_writes_is_kept(tmp_path):
    cfg = make_config(tmp_path, option=' cone="pyramidal" iterations="7"')
    model, applied = rg.build_model(cfg)
    assert model.opt.cone == mujoco.mjtCone.mjCONE_PYRAMIDAL and model.opt.iterations == 7
    assert "cone" not in applied["solver"] and "iterations" not in applied["solver"]
    assert applied["solver"]["ls_iterations"] == cfg.solver["ls_iterations"]


def test_without_the_gain_override_the_models_gains_stay(tmp_path):
    model, applied = rg.build_model(make_config(tmp_path, use_yaml_servo_gains=False))
    arm = _id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "arm_a")
    assert model.actuator_gainprm[arm, 0] == 100.0 and applied["servo_gains"] == {}


def test_configs_the_simulator_would_refuse_are_refused(tmp_path):
    with pytest.raises(ValueError, match="physics_timestep"):
        rg.build_model(make_config(tmp_path, control_period=0.001))
    with pytest.raises(ValueError, match="servo_kp"):
        rg.build_model(
            make_config(
                tmp_path,
                groups=(
                    rc.SimGroup("arm", ("arm_j",), (500.0, 1.0), (50.0,)),
                    rc.SimGroup("hand", ("finger_j",), (300.0,), (10.0,)),
                ),
            )
        )
    with pytest.raises(ValueError, match="not in the model"):
        rg.DockingRig(make_config(tmp_path, hand_joints=("thumb_j",)))
    with pytest.raises(ValueError, match="ball_type"):
        rg.build_model(make_config(tmp_path, ball=rc.BallConfig("marble", RADIUS, 0.057, 1, 1)))


def test_the_ball_rebounds_near_the_presets_restitution(tmp_path):
    measured = rg.measure_restitution(make_config(tmp_path))
    # The simulator's own note: within about 0.04 of the target at these substeps.
    assert measured == pytest.approx(rc.BALL_PRESETS["tennis"][1], abs=0.05)


# ── The rig ───────────────────────────────────────────────────────────────────


@pytest.fixture
def rig(tmp_path):
    return rg.DockingRig(make_config(tmp_path))


def test_the_catch_frame_is_where_the_config_puts_it(rig):
    assert rig.at_rest() and rig.hand_error() < 1e-6
    assert rig.origin == pytest.approx([0.0, 0.0, 0.7 + PALM_TOP + RADIUS], abs=1e-6)
    assert rig.rot == pytest.approx(np.eye(3), abs=1e-6)
    # A frame turned over about x points its approach axis down.
    assert rg.rpy_matrix((math.pi, 0.0, 0.0)) == pytest.approx(np.diag([1.0, -1.0, -1.0]))
    contact = rig.first_contact_on_axis()
    assert contact["body"] == "palm" and contact["on_hand"]
    assert -0.0015 < contact["s"] <= 0.0
    assert contact["point"] == pytest.approx([0.0, 0.0, -RADIUS], abs=2e-3)


@pytest.mark.parametrize("c", [0.5, 1.0, 2.5])
def test_the_ball_reaches_the_origin_plane_when_the_trial_says(rig, c):
    # The close comes 200 ms after the arrival and the finger is out of the
    # way: the first thing the ball meets is the palm, at s = 0, at t = 0.
    out = rig.fly_in((0.0, 0.0), c, 0.2)
    assert out["stray"] is None
    assert abs(out["s_first"]) < 2 * c * rig.h + 1e-4
    assert abs(out["t_first"]) < 2.5 * rig.h
    assert out["rho_first"] == pytest.approx(0.0, abs=1e-3)  # the hand has not moved yet
    assert not out["held"] and out["why"] == "left"  # an open palm holds nothing


def test_the_close_is_commanded_t_close_minus_delta_o_before_the_arrival(rig):
    # Choose delta_o so that the ball arrives while the finger is half way: the
    # closure progress at the first touch then tells when the close was sent.
    halfway = rig.empty_close_time()
    delta_o = rig.cfg.t_close_e2e - halfway
    out = rig.fly_in((0.0, 0.0), 1.0, delta_o)

    def progress_after(seconds: float) -> float:
        rig.restore()
        rig._command_close()
        for _ in range(int(round(seconds / rig.h))):
            mujoco.mj_step(rig.model, rig.data)
        return rig.progress()

    lead = rig.cfg.t_close_e2e - delta_o  # command -> arrival
    expected = progress_after(lead)
    assert 0.3 < expected < 0.7
    # Within the progress of one control period either side (the command is on
    # a tick; the touch is seen at the start of a substep).
    period = rig.h * rig.nsub
    assert (
        progress_after(lead - period) - 1e-3
        <= out["rho_first"]
        <= progress_after(lead + period) + 1e-3
    )
    # A much later close would find the finger still open at the touch.
    assert rig.fly_in((0.0, 0.0), 1.0, delta_o + 0.04)["rho_first"] < expected - 0.2


def test_the_catch_frame_follows_the_arm_and_its_own_rotation(tmp_path):
    tilt, yaw = 0.3, math.pi / 2
    turned = rg.DockingRig(
        make_config(tmp_path, arm_pose=(tilt,), catch_rpy=(0.0, 0.0, yaw), q_close=(0.04,))
    )
    ry = np.array(
        [[math.cos(tilt), 0, math.sin(tilt)], [0, 1, 0], [-math.sin(tilt), 0, math.cos(tilt)]]
    )
    rz = np.array(
        [[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]
    )
    assert turned.rot == pytest.approx(ry @ rz, abs=1e-4)
    assert turned.origin == pytest.approx(
        np.array([0.0, 0.0, 0.5]) + ry @ np.array([0.0, 0.0, 0.2 + PALM_TOP + RADIUS]), abs=1e-4
    )
    # The approach axis is the tilted palm's normal: straight down it, the ball
    # still meets the palm at s = 0, on time.
    out = turned.fly_in((0.0, 0.0), 1.0, 0.2)
    assert abs(out["s_first"]) < 2 * turned.h + 1e-4 and abs(out["t_first"]) < 2.5 * turned.h
    # An acceleration is given in that frame too: along its approach axis the
    # ball still comes down the palm's normal, only sooner.
    s_pass, a_s = 0.04, -9.81
    fast = turned.fly_in((0.0, 0.0), 1.0, 0.2, s_pass=s_pass, accel=(0.0, 0.0, a_s))
    tau = (1.0 - math.sqrt(1.0 - 2.0 * a_s * s_pass)) / a_s
    assert fast["t_first"] == pytest.approx(tau - s_pass, abs=2.5 * turned.h)
    assert abs(fast["s_first"]) < 3 * turned.h + 1e-4
    # Sideways it is the FRAME's axes that count. The finger stands on the
    # palm's +x, which the yaw makes the frame's −y: pulled that way from 0.1
    # above the palm the ball runs into it, pulled along the frame's +x (the
    # palm's +y, where nothing stands) it passes the hand by.
    assert turned.fly_in((0.0, 0.0), 1.0, 0.2, s_pass=0.1, accel=(0.0, -20.0, 0.0))["s_first"]
    beside = turned.fly_in((0.0, 0.0), 1.0, 0.2, s_pass=0.1, accel=(20.0, 0.0, 0.0))
    assert beside["s_first"] is None and beside["why"] == "missed"


# ── A ball that accelerates in the catch frame ────────────────────────────────


def test_no_acceleration_is_the_straight_flight(rig):
    straight = rig.fly_in((0.01, -0.005), 1.3, 0.004, (0.1, 0.05), 0.02)
    assert rig.fly_in((0.01, -0.005), 1.3, 0.004, (0.1, 0.05), 0.02, (0.0, 0.0, 0.0)) == straight
    assert straight["accel"] == [0.0, 0.0, 0.0]


@pytest.mark.parametrize(
    ("c", "a_s"), [(1.0, -9.81), (1.0, 6.0), (0.3, -9.81)], ids=["toward", "away", "slow"]
)
def test_the_ball_accelerates_from_the_named_plane_on(rig, c, a_s):
    # It crosses s_pass at speed c when the line does, so it meets the palm
    # (s = 0) the time tau after that with c tau − a_s tau² / 2 = s_pass —
    # earlier than the line's arrival when it speeds up, later when it slows.
    s_pass = 0.04
    out = rig.fly_in((0.0, 0.0), c, 0.2, s_pass=s_pass, accel=(0.0, 0.0, a_s))
    tau = (c - math.sqrt(c * c - 2.0 * a_s * s_pass)) / a_s
    assert c * tau - 0.5 * a_s * tau**2 == pytest.approx(s_pass)
    assert out["t_first"] == pytest.approx(tau - s_pass / c, abs=2.5 * rig.h)
    assert (out["t_first"] < -5 * rig.h) == (a_s < 0.0)
    assert abs(out["s_first"]) < 2 * (c - a_s * tau) * rig.h + 1e-4
    assert out["stray"] is None and out["accel"] == [0.0, 0.0, a_s]
    assert not rig.model.opt.gravity.any()  # only while the trial runs


def test_a_sideways_acceleration_bends_the_line_below_the_named_plane_only(rig):
    # The ball is aimed at the catch point on the plane 0.1 above it and
    # pulled along +y at 20 m/s²: by the palm, 0.1 s later, it is 0.1 m off —
    # beside the palm's 50 mm half-width. Without the pull it lands on it.
    straight = rig.fly_in((0.0, 0.0), 1.0, 0.2, s_pass=0.1)
    assert straight["s_first"] is not None
    out = rig.fly_in((0.0, 0.0), 1.0, 0.2, s_pass=0.1, accel=(0.0, 20.0, 0.0))
    assert out["s_first"] is None and out["why"] == "missed"
    # Named on the origin plane, the same pull starts where the palm is: the
    # flight down to the first contact is the straight one, to the digit.
    late = rig.fly_in((0.0, 0.0), 1.0, 0.2, accel=(0.0, 20.0, 0.0))
    for key in ("s_first", "t_first", "rho_first"):
        assert late[key] == straight[key]


def test_the_same_trial_gives_the_same_result(rig):
    parked = rig.parked.qpos.copy()
    first = rig.fly_in((0.01, -0.005), 1.3, 0.004, (0.1, 0.05))
    again = rig.fly_in((0.01, -0.005), 1.3, 0.004, (0.1, 0.05))
    assert first == again
    rig.restore()
    assert np.array_equal(rig.data.qpos, parked) and rig.data.time == 0.0
    assert rig.model.geom_contype[rig.ball_geom] == 0  # parked: contacts off


def test_a_ball_aimed_beside_the_hand_is_missed(rig):
    out = rig.fly_in((0.0, 0.15), 1.0, 0.0)
    assert out["s_first"] is None and out["why"] == "missed" and not out["held"]


def test_something_that_is_not_the_hand_on_the_way_is_named(rig):
    post = np.asarray(POST) - rig.origin  # the post in the catch frame
    # Straight down onto the post...
    assert rig.fly_in(post[:2], 1.0, 0.0)["stray"] == "post"
    # ...and a tilted line through the catch point that passes through it: the
    # ball is at rho − (nu/c)(s − s_pass) above the plane it is aimed on.
    c = 1.0
    nu = -post[:2] * c / post[2]
    out = rig.fly_in((0.0, 0.0), c, 0.0, nu)
    assert out["stray"] == "post"
    assert rig.fly_in((0.0, 0.0), c, 0.0, -nu)["stray"] is None  # the mirror line misses it
    # The same line, named by the point where it crosses s = 0.1 instead.
    crossing = -(nu / c) * 0.1
    assert rig.fly_in(crossing, c, 0.0, nu, s_pass=0.1)["stray"] == "post"


def test_the_contact_field_is_the_palm_and_the_post(rig):
    post = np.asarray(POST) - rig.origin
    ss = np.round(np.arange(-0.005, 0.0205, 0.001), 6)
    hand, stray = rig.contact_field([0.0, 0.2], [0.0], ss)
    assert hand[0, 0, ss <= -0.001].all() and not hand[0, 0, ss >= 0.001].any()
    assert not hand[1].any() and not stray.any()
    hand, stray = rig.contact_field([post[0]], [post[1]], [post[2]])
    assert stray[0, 0, 0] and not hand[0, 0, 0]
    # The field leaves the rig parked.
    assert np.array_equal(rig.data.qpos, rig.parked.qpos)


def test_closure_progress_runs_from_the_preshape_to_the_closed_pose(rig):
    rig.restore()
    assert rig.progress() == pytest.approx(0.0, abs=1e-3)
    elapsed = rig.empty_close_time()
    assert elapsed is not None and 0.0 < elapsed < 1.0
    # It is the first substep at which rho has reached eta.
    rig.restore()
    rig._command_close()
    steps = int(round(elapsed / rig.h))
    for _ in range(steps - 1):
        mujoco.mj_step(rig.model, rig.data)
    assert rig.progress() < rig.cfg.eta_close
    mujoco.mj_step(rig.model, rig.data)
    assert rig.progress() >= rig.cfg.eta_close


# ── The verdict ───────────────────────────────────────────────────────────────


def test_the_shake_tells_a_caged_ball_from_a_resting_one(tmp_path, rig):
    rig.restore()
    rig.put_ball((0.0, 0.0, 0.0))
    assert rig._shake() > rg.SLIP_M  # on an open palm: gone in the first direction that lifts it
    assert rig.model.opt.gravity.tolist() == [0.0, 0.0, 0.0]

    caged = rg.DockingRig(make_config(tmp_path, cage=CAGE))
    caged.restore()
    caged.put_ball((0.0, 0.0, 0.0))
    slip = caged._shake()
    assert slip < 0.006 < rg.SLIP_M  # 2 mm of clearance each way
    # Gravity along each catch axis in turn: the ball did get pushed around.
    assert slip > 0.001
