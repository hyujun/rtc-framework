"""The docking identification rig: the sim's ball flown into the preshape hand.

dynamic_catching E1-F15 (#741). The planner's capture set is a property of a
hand, a posture pair and a ball, and it is measured here: the arm is held at
the wait pose, the hand settles at ``q_pre``, a ball arrives along a straight
line in the catch frame, and ``q_close`` is commanded at an instant fixed
relative to that arrival. Whether the ball is then HELD is the verdict.

THE SAME PHYSICS AS THE BRING-UP. A rig that loaded the MJCF alone would not
be the simulator: ``rtc_mujoco_sim`` lays its solver options, its servo gains
and gravity compensation over the model when it loads it, and builds the ball
itself. :func:`build_model` repeats every one of those steps from the values
rig_config.py read — :attr:`DockingRig.applied` lists what was set, and the
self-check prints it. The ball's material constants are the one thing that
lives only in C++ (``rtc_mujoco_sim/src/projectile_ball.cpp``); they are
copied in rig_config.py and a test compares the copy with that file.

WHAT IS NOT THE BRING-UP, on purpose:

* no gravity while the ball flies — the approach is a straight line in the
  catch frame, which is what the planner's relative state is at one instant;
* the hand does not move — the ball carries the whole relative velocity.
  That equals a moving hand only while the hand moves at constant velocity;
* the close command is a step applied on a control tick, with no controller,
  no transport and no estimator in between.

TIME. ``delta_o`` is (instant the closure is nominally complete) − (instant the
ball centre would reach the catch frame's origin plane in free flight), with
"nominally complete" = command tick + the shipped ``T_close_e2e``: exactly the
quantity the hand sequencer controls when it commands at ``t_c − T_close_e2e``.

THE VERDICT (kept from the tool this replaces): the scene plays out 1 s after
the arrival, then gravity is applied along ±each catch-frame axis for 0.25 s
from that state. Held = the ball stayed within 0.25 m and slipped at most
20 mm in every direction.

Needs the ``mujoco`` python module (the workspace environment has it).
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from rig_config import (
    BALL_BODY,
    BALL_CONDIM,
    BALL_PARK,
    BALL_PRESETS,
    BALL_PRIORITY,
    BALL_SOLIMP,
    CONTACT_SUBSTEPS,
    RigConfig,
    damping_ratio_for,
)

from rtc_tools.analysis.hand_close import joint_progress

# ── The verdict ──
PLAY_S = 1.0  # after the arrival, before the shake
SHAKE_S = 0.25  # per direction
SHAKE_G = 9.81
SLIP_M = 0.02
LEFT_M = 0.25  # farther than this from the catch point is not in the hand
FAR_M = 0.35  # ...and this far, once past the arrival, ends the trial early

# ── The rig ──
START_S = 0.30  # the ball appears this far above the origin plane [m]
LEAD_S = 0.01  # earliest close command after the hand has settled [s]
SETTLE_S = 3.0
SETTLE_MAX_S = 12.0
REST_LIN = 1e-3  # palm [m/s]
REST_ANG = 5e-3  # palm [rad/s]
REST_JOINT = 5e-3  # hand joints [rad/s]


def rpy_matrix(rpy) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw as a rotation matrix (Rz Ry Rx)."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (
        math.cos(r),
        math.sin(r),
        math.cos(p),
        math.sin(p),
        math.cos(y),
        math.sin(y),
    )
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


_SOLVERS = {"PGS": mujoco.mjtSolver.mjSOL_PGS, "CG": mujoco.mjtSolver.mjSOL_CG}
_CONES = {"elliptic": mujoco.mjtCone.mjCONE_ELLIPTIC}
_JACOBIANS = {"dense": mujoco.mjtJacobian.mjJAC_DENSE, "sparse": mujoco.mjtJacobian.mjJAC_SPARSE}
_INTEGRATORS = {
    "RK4": mujoco.mjtIntegrator.mjINT_RK4,
    "implicit": mujoco.mjtIntegrator.mjINT_IMPLICIT,
    "implicitfast": mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
}
# YAML key -> disable bit: the flag is ON when the bit is CLEAR.
_DISABLE_FLAGS = {
    "warmstart": mujoco.mjtDisableBit.mjDSBL_WARMSTART,
    "refsafe": mujoco.mjtDisableBit.mjDSBL_REFSAFE,
    "island": mujoco.mjtDisableBit.mjDSBL_ISLAND,
    "eulerdamp": mujoco.mjtDisableBit.mjDSBL_EULERDAMP,
    "filterparent": mujoco.mjtDisableBit.mjDSBL_FILTERPARENT,
    "autoreset": mujoco.mjtDisableBit.mjDSBL_AUTORESET,
    "nativeccd": mujoco.mjtDisableBit.mjDSBL_NATIVECCD,
}


def apply_solver(model: mujoco.MjModel, solver: dict, scene_path: str) -> dict:
    """The simulator's ``ApplySolverConfig``: every option the scene file's own
    root ``<option>`` does not write takes the YAML value. Returns what was set.

    Like the simulator, only the SCENE file's root element is searched — an
    ``<option>`` in a file the scene includes is merged by MuJoCo and then
    overwritten here — and ``<flag>`` is looked for beside ``<option>``, where
    MJCF never puts it, so the YAML flags always apply.
    """
    root = ET.parse(scene_path).getroot()
    option = root.find("option")
    written = set(option.attrib) if option is not None else set()
    flag = root.find("flag")
    flagged = set(flag.attrib) if flag is not None else set()
    opt = model.opt
    applied = {}

    def take(key: str, setter) -> None:
        if key not in written and key in solver:
            setter(solver[key])
            applied[key] = solver[key]

    # Unknown names fall back the way the simulator's *NameToEnum do.
    take(
        "solver", lambda v: setattr(opt, "solver", _SOLVERS.get(v, mujoco.mjtSolver.mjSOL_NEWTON))
    )
    take("cone", lambda v: setattr(opt, "cone", _CONES.get(v, mujoco.mjtCone.mjCONE_PYRAMIDAL)))
    take(
        "jacobian",
        lambda v: setattr(opt, "jacobian", _JACOBIANS.get(v, mujoco.mjtJacobian.mjJAC_AUTO)),
    )
    take(
        "integrator",
        lambda v: setattr(
            opt, "integrator", _INTEGRATORS.get(v, mujoco.mjtIntegrator.mjINT_EULER)
        ),
    )
    for key in ("iterations", "ls_iterations", "noslip_iterations", "ccd_iterations"):
        take(key, lambda v, key=key: setattr(opt, key, int(v)))
    for key in ("sdf_iterations", "sdf_initpoints"):
        take(key, lambda v, key=key: setattr(opt, key, int(v)))
    for key in ("tolerance", "ls_tolerance", "noslip_tolerance", "ccd_tolerance", "impratio"):
        take(key, lambda v, key=key: setattr(opt, key, float(v)))

    for key, bit in _DISABLE_FLAGS.items():
        if key in flagged or key not in solver:
            continue
        if solver[key]:
            opt.disableflags &= ~int(bit)
        else:
            opt.disableflags |= int(bit)
        applied[key] = bool(solver[key])
    if "multiccd" not in flagged and "multiccd" in solver:
        if solver["multiccd"]:
            opt.enableflags |= int(mujoco.mjtEnableBit.mjENBL_MULTICCD)
        else:
            opt.enableflags &= ~int(mujoco.mjtEnableBit.mjENBL_MULTICCD)
        applied["multiccd"] = bool(solver["multiccd"])
    override = solver.get("contact_override") or {}
    if "override" not in flagged and "enable" in override:
        if override["enable"]:
            opt.enableflags |= int(mujoco.mjtEnableBit.mjENBL_OVERRIDE)
            opt.o_margin = float(override.get("o_margin", 0.0))
            opt.o_solref[:] = override["o_solref"]
            opt.o_solimp[:] = override["o_solimp"]
            opt.o_friction[:] = override["o_friction"]
        else:
            opt.enableflags &= ~int(mujoco.mjtEnableBit.mjENBL_OVERRIDE)
        applied["contact_override"] = bool(override["enable"])
    return applied


def _joint_actuator(model: mujoco.MjModel, joint: str) -> tuple[int, int]:
    """(joint id, id of the actuator driving it) — the simulator's lookup."""
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
    if jid < 0:
        raise ValueError(f"joint '{joint}' is not in the model")
    if model.jnt_type[jid] not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
        raise ValueError(f"joint '{joint}' is not a hinge or a slide")
    for act in range(model.nu):
        if (
            model.actuator_trntype[act] == mujoco.mjtTrn.mjTRN_JOINT
            and model.actuator_trnid[act, 0] == jid
        ):
            return jid, act
    raise ValueError(f"joint '{joint}' has no actuator")


def _compensated_bodies(model: mujoco.MjModel, joints) -> set[int]:
    """The bodies the simulator compensates gravity on for a group: the bodies
    its joints move, and every jointless body hanging off those."""
    bodies = {
        int(model.jnt_bodyid[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)])
        for j in joints
    }
    changed = True
    while changed:
        changed = False
        for body in range(1, model.nbody):
            if body in bodies or model.body_jntnum[body] != 0:
                continue
            if int(model.body_parentid[body]) in bodies:
                bodies.add(body)
                changed = True
    return bodies


def build_model(cfg: RigConfig, anvil: bool = False) -> tuple[mujoco.MjModel, dict]:
    """The profile's model the way the simulator runs it, plus the ball.

    ``anvil`` adds a fixed block far from the robot for the restitution check.
    Returns the model and a record of everything that was laid over the MJCF.
    """
    if cfg.ball.kind not in BALL_PRESETS:
        raise ValueError(f"unknown ball_type '{cfg.ball.kind}'")
    inertia_ratio, restitution, sliding, torsional, rolling = BALL_PRESETS[cfg.ball.kind]
    radius, mass = cfg.ball.radius, cfg.ball.mass

    spec = mujoco.MjSpec.from_file(cfg.model_path)
    ball = spec.worldbody.add_body(name=BALL_BODY, pos=list(BALL_PARK))
    # Explicit inertial, like the simulator: the preset's inertia ratio (hollow
    # against filled) cannot be said through a sphere's density.
    ball.explicitinertial = 1
    ball.mass = mass
    ball.ipos = [0.0, 0.0, 0.0]
    ball.iquat = [1.0, 0.0, 0.0, 0.0]
    ball.inertia = [inertia_ratio * mass * radius * radius] * 3
    ball.add_freejoint()
    ball.add_geom(
        name=f"{BALL_BODY}_geom",
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=[radius, 0.0, 0.0],
        group=0,
        contype=cfg.ball.contype,
        conaffinity=cfg.ball.conaffinity,
        condim=BALL_CONDIM,
        priority=BALL_PRIORITY,
    )
    if anvil:
        block = spec.worldbody.add_body(name="docking_ident_anvil", pos=[0.0, 0.0, 60.0])
        block.add_geom(
            name="docking_ident_anvil_geom",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[0.5, 0.5, 0.5],
            contype=3,
            conaffinity=3,
        )
    model = spec.compile()

    # Gravity compensation on the robot groups (position servo mode). The MJCF
    # declares none, so it is set on the spec and the model compiled again.
    compensated: set[int] = set()
    for group in cfg.groups:
        compensated |= _compensated_bodies(model, group.joints)
    for body in spec.bodies:
        if body.id in compensated:
            body.gravcomp = 1.0
    model = spec.compile()

    applied: dict = {"gravcomp_bodies": len(compensated)}
    xml_step = float(model.opt.timestep)
    if abs(xml_step - cfg.control_period) > 1e-9:
        raise ValueError(
            f"physics_timestep {cfg.control_period} is not the MJCF's {xml_step} "
            "(the simulator reports this as an error and runs the MJCF's)"
        )
    if cfg.n_substeps < 1:
        raise ValueError("n_substeps must be at least 1")
    model.opt.timestep = xml_step / cfg.n_substeps
    applied["substep_s"] = float(model.opt.timestep)
    applied["solver"] = apply_solver(model, cfg.solver, cfg.model_path)

    gains = {}
    if cfg.use_yaml_servo_gains:
        for group in cfg.groups:
            if len(group.servo_kp) != len(group.joints) or len(group.servo_kd) != len(
                group.joints
            ):
                raise ValueError(f"group '{group.name}': servo_kp/kd do not match its joints")
            for joint, kp, kd in zip(group.joints, group.servo_kp, group.servo_kd, strict=True):
                _, act = _joint_actuator(model, joint)
                model.actuator_gainprm[act, 0] = kp
                model.actuator_biasprm[act, 0] = 0.0
                model.actuator_biasprm[act, 1] = -kp
                model.actuator_biasprm[act, 2] = -kd
                model.actuator_biastype[act] = mujoco.mjtBias.mjBIAS_AFFINE
            gains[group.name] = {"kp": list(group.servo_kp), "kd": list(group.servo_kd)}
    applied["servo_gains"] = gains

    geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{BALL_BODY}_geom")
    for other in range(model.ngeom):
        if other != geom and model.geom_priority[other] >= BALL_PRIORITY:
            raise ValueError(
                f"geom {other} has priority {model.geom_priority[other]}: the ball's contact "
                "parameters would not win against it"
            )
    omega = math.pi / (CONTACT_SUBSTEPS * float(model.opt.timestep))
    zeta = damping_ratio_for(restitution)
    model.geom_solref[geom] = [-(omega * omega), -(2.0 * zeta * omega)]
    model.geom_solimp[geom] = BALL_SOLIMP
    model.geom_friction[geom] = [sliding, torsional * radius, rolling * radius]
    applied["ball"] = {
        "kind": cfg.ball.kind,
        "radius": radius,
        "mass": mass,
        "inertia": inertia_ratio * mass * radius * radius,
        "contype": cfg.ball.contype,
        "conaffinity": cfg.ball.conaffinity,
        "condim": BALL_CONDIM,
        "priority": BALL_PRIORITY,
        "solref": [float(v) for v in model.geom_solref[geom]],
        "solimp": list(BALL_SOLIMP),
        "friction": [float(v) for v in model.geom_friction[geom]],
        "restitution_target": restitution,
        "zeta": zeta,
    }
    model.opt.gravity[:] = 0.0  # the flight is a straight line; the shake sets it
    return model, applied


class DockingRig:
    """One profile's hand, settled at ``q_pre`` with the arm at the wait pose."""

    def __init__(self, cfg: RigConfig) -> None:
        self.cfg = cfg
        self.model, self.applied = build_model(cfg)
        m = self.model
        self.data = mujoco.MjData(m)
        self.h = float(m.opt.timestep)
        self.nsub = cfg.n_substeps

        def name2id(kind, name):
            ident = mujoco.mj_name2id(m, kind, name)
            if ident < 0:
                raise ValueError(f"'{name}' is not in the model")
            return ident

        self.ball_body = name2id(mujoco.mjtObj.mjOBJ_BODY, BALL_BODY)
        self.ball_geom = name2id(mujoco.mjtObj.mjOBJ_GEOM, f"{BALL_BODY}_geom")
        joint = m.body_jntadr[self.ball_body]
        self.ball_q = int(m.jnt_qposadr[joint])
        self.ball_v = int(m.jnt_dofadr[joint])
        self.palm = name2id(mujoco.mjtObj.mjOBJ_BODY, cfg.catch_parent)
        self.frame_xyz = np.asarray(cfg.catch_xyz, dtype=float)
        self.frame_rot = rpy_matrix(cfg.catch_rpy)

        arm = [_joint_actuator(m, j) for j in cfg.arm_joints]
        hand = [_joint_actuator(m, j) for j in cfg.hand_joints]
        self.arm_qpos = [int(m.jnt_qposadr[j]) for j, _ in arm]
        self.arm_act = [a for _, a in arm]
        self.hand_qpos = [int(m.jnt_qposadr[j]) for j, _ in hand]
        self.hand_dof = [int(m.jnt_dofadr[j]) for j, _ in hand]
        self.hand_act = [a for _, a in hand]
        self.caging = [i for i, on in enumerate(cfg.caging_mask) if on]

        # The hand = everything fixed to or hanging off the catch frame's parent.
        hand_bodies = set()
        for body in range(m.nbody):
            anc = body
            while anc != 0:
                if anc == self.palm:
                    hand_bodies.add(body)
                    break
                anc = int(m.body_parentid[anc])
        self.is_hand_geom = np.array([int(b) in hand_bodies for b in m.geom_bodyid])

        self.settle_s = self._settle()
        self.parked = mujoco.MjData(m)
        mujoco.mj_copyData(self.parked, m, self.data)
        self.origin, self.rot = self.frame()

    # ── State ────────────────────────────────────────────────────────────────

    def _settle(self) -> float:
        m, d = self.model, self.data
        self._ball_collides(False)
        for adr, act, q in zip(self.arm_qpos, self.arm_act, self.cfg.arm_pose, strict=True):
            d.qpos[adr] = q
            d.ctrl[act] = q
        for act, q in zip(self.hand_act, self.cfg.q_pre, strict=True):
            d.ctrl[act] = q
        span = SETTLE_S
        while d.time < SETTLE_MAX_S:
            for _ in range(int(round(span / self.h))):
                mujoco.mj_step(m, d)
            if self.at_rest():
                settled = float(d.time)
                d.time = 0.0
                return settled
            span = 0.5
        raise RuntimeError(f"the hand did not come to rest within {SETTLE_MAX_S} s")

    def _ball_collides(self, on: bool) -> None:
        """The simulator parks its ball with contacts OFF and turns them on at
        launch (a ball parked under a floor plane is otherwise thrown out)."""
        geom = self.ball_geom
        self.model.geom_contype[geom] = self.cfg.ball.contype if on else 0
        self.model.geom_conaffinity[geom] = self.cfg.ball.conaffinity if on else 0

    def at_rest(self) -> bool:
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(
            self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.palm, velocity, 0
        )
        return bool(
            np.linalg.norm(velocity[3:]) < REST_LIN
            and np.linalg.norm(velocity[:3]) < REST_ANG
            and np.max(np.abs(self.data.qvel[self.hand_dof])) < REST_JOINT
        )

    def restore(self) -> None:
        mujoco.mj_copyData(self.data, self.model, self.parked)
        self.model.opt.gravity[:] = 0.0
        self._ball_collides(False)

    def frame(self) -> tuple[np.ndarray, np.ndarray]:
        """The catch frame now: its origin in the world and R (frame → world)."""
        rot_parent = self.data.xmat[self.palm].reshape(3, 3)
        return self.data.xpos[self.palm] + rot_parent @ self.frame_xyz, rot_parent @ self.frame_rot

    def ball(self) -> np.ndarray:
        """The ball centre in the catch frame, now."""
        origin, rot = self.frame()
        return rot.T @ (self.data.xpos[self.ball_body] - origin)

    def put_ball(self, r, velocity=(0.0, 0.0, 0.0)) -> None:
        """Place the ball centre at ``r`` (catch frame of the SETTLED hand) and
        turn its contacts on."""
        d = self.data
        self._ball_collides(True)
        d.qpos[self.ball_q : self.ball_q + 3] = self.origin + self.rot @ np.asarray(r, dtype=float)
        d.qpos[self.ball_q + 3 : self.ball_q + 7] = [1.0, 0.0, 0.0, 0.0]
        d.qvel[self.ball_v : self.ball_v + 6] = 0.0
        d.qvel[self.ball_v : self.ball_v + 3] = self.rot @ np.asarray(velocity, dtype=float)

    def progress(self) -> float:
        """ρ: the closure progress of the caging set (L6 §4.2), now."""
        q = self.data.qpos
        return min(
            joint_progress(float(q[self.hand_qpos[i]]), self.cfg.q_pre[i], self.cfg.q_close[i])
            for i in self.caging
        )

    def hand_error(self) -> float:
        """Largest |q − q_pre| over the hand's driven joints, now [rad]."""
        q = self.data.qpos[self.hand_qpos]
        return float(np.max(np.abs(q - np.asarray(self.cfg.q_pre))))

    def _command_close(self) -> None:
        for act, q in zip(self.hand_act, self.cfg.q_close, strict=True):
            self.data.ctrl[act] = q

    def _ball_contacts(self) -> tuple[bool, int]:
        """(touches the hand, a geom that is NOT the hand or −1), now."""
        d = self.data
        if d.ncon == 0:
            return False, -1
        pairs = d.contact.geom[: d.ncon]
        mine = pairs == self.ball_geom
        rows = mine.any(axis=1)
        if not rows.any():
            return False, -1
        others = np.where(mine[rows, 0], pairs[rows, 1], pairs[rows, 0])
        on_hand = self.is_hand_geom[others]
        stray = others[~on_hand]
        return bool(on_hand.any()), int(stray[0]) if stray.size else -1

    def _geom_body(self, geom: int) -> str:
        name = mujoco.mj_id2name(
            self.model, mujoco.mjtObj.mjOBJ_BODY, int(self.model.geom_bodyid[geom])
        )
        return name or f"body{int(self.model.geom_bodyid[geom])}"

    # ── The trial ────────────────────────────────────────────────────────────

    def fly_in(
        self, rho, c: float, delta_o: float, nu_perp=(0.0, 0.0), s_pass: float = 0.0
    ) -> dict:
        """One time-triggered fly-in.

        The ball centre moves on the line through ``(rho, s_pass)`` with
        velocity ``(nu_perp, −c)`` in the catch frame. ``q_close`` is commanded
        on the control tick that makes the nominal closure complete
        ``delta_o`` after the ball's free-flight arrival at ``s = 0``.
        """
        if not c > 0.0:
            raise ValueError("the closing speed must be positive")
        m, d = self.model, self.data
        self.restore()
        period = self.h * self.nsub
        t_close = self.cfg.t_close_e2e
        nu = np.array([nu_perp[0], nu_perp[1], -c], dtype=float)
        # The command is ON a tick; the ball's arrival then follows from delta_o.
        earliest = max(LEAD_S, START_S / c - t_close + delta_o)
        tick = int(math.ceil(earliest / period - 1e-9))
        t_cmd = tick * period
        t_origin = t_cmd + t_close - delta_o
        at_origin = np.array([rho[0], rho[1], 0.0]) + np.array([nu[0], nu[1], 0.0]) * (s_pass / c)
        n_cmd = tick * self.nsub
        n_appear = int(math.ceil((t_origin - START_S / c) / self.h - 1e-9))
        n_origin = int(math.ceil(t_origin / self.h))
        n_end = int(math.ceil((max(t_origin, t_cmd + t_close) + PLAY_S) / self.h))

        out = {
            "rho": [float(rho[0]), float(rho[1])],
            "c": float(c),
            "delta_o": float(delta_o),
            "nu": [float(nu_perp[0]), float(nu_perp[1])],
            "s_pass": float(s_pass),
            "s_first": None,  # ball height when it first touched the hand
            "t_first": None,  # ...and when, relative to the arrival at s = 0
            "rho_first": None,  # ...and the hand's closure progress then
            "stray": None,  # a body that is not the hand, touched BEFORE the hand
        }
        left = False
        for n in range(n_end):
            if n == n_cmd:
                self._command_close()
            if n == n_appear:
                self.put_ball(at_origin + nu * (n * self.h - t_origin), nu)
            mujoco.mj_step(m, d)
            if n < n_appear:
                continue
            if out["s_first"] is None:
                on_hand, stray = self._ball_contacts()
                if stray >= 0 and out["stray"] is None and not on_hand:
                    out["stray"] = self._geom_body(stray)
                if on_hand:
                    out["s_first"] = round(float(self.ball()[2]), 5)
                    # A step's contacts are those of the positions it STARTED from.
                    out["t_first"] = round(n * self.h - t_origin, 5)
                    out["rho_first"] = round(self.progress(), 4)
            if n > n_origin and n % 50 == 0 and np.linalg.norm(self.ball()) > FAR_M:
                left = True
                break
        end = self.ball()
        out["rho_end"] = round(self.progress(), 4)
        out["ball_end"] = [round(float(v), 4) for v in end]
        if left or np.linalg.norm(end) > LEFT_M:
            out.update(held=False, why="missed" if out["s_first"] is None else "left", slip=None)
            return out
        slip = self._shake()
        out["slip"] = round(slip, 5)
        out.update(held=slip <= SLIP_M, why="held" if slip <= SLIP_M else "dropped")
        return out

    def _shake(self) -> float:
        """Largest slip of the ball under gravity along ±each catch axis; stops
        at the first direction that fails (the value is then a lower bound)."""
        m, d = self.model, self.data
        state = mujoco.MjData(m)
        mujoco.mj_copyData(state, m, d)
        worst = 0.0
        steps = int(round(SHAKE_S / self.h))
        try:
            for axis in range(3):
                for sign in (1.0, -1.0):
                    mujoco.mj_copyData(d, m, state)
                    mujoco.mj_kinematics(m, d)  # positions of THIS state, not of a step ago
                    _, rot = self.frame()
                    m.opt.gravity[:] = rot[:, axis] * (sign * SHAKE_G)
                    start = self.ball()
                    for _ in range(steps):
                        mujoco.mj_step(m, d)
                    worst = max(worst, float(np.linalg.norm(self.ball() - start)))
                    if worst > SLIP_M:
                        return worst
        finally:
            m.opt.gravity[:] = 0.0
        return worst

    # ── The open hand ────────────────────────────────────────────────────────

    def contact_field(self, xs, ys, ss) -> tuple[np.ndarray, np.ndarray]:
        """``(hand, stray)``: for the ball centre at each ``(x, y, s)`` of the
        catch frame, does it touch the settled preshape hand / anything else."""
        m, d = self.model, self.data
        self.restore()
        hand = np.zeros((len(xs), len(ys), len(ss)), dtype=bool)
        stray = np.zeros_like(hand)
        for i, x in enumerate(xs):
            for j, y in enumerate(ys):
                for k, s in enumerate(ss):
                    self.put_ball((x, y, s))
                    mujoco.mj_kinematics(m, d)
                    mujoco.mj_collision(m, d)
                    on_hand, other = self._ball_contacts()
                    hand[i, j, k] = on_hand
                    stray[i, j, k] = other >= 0
        self.restore()
        return hand, stray

    def first_contact_on_axis(self, s_top: float = START_S, step: float = 1e-3) -> dict | None:
        """Where the ball first touches the open hand coming down the approach
        axis: the ball centre's height and the contact point in the catch frame."""
        m, d = self.model, self.data
        self.restore()
        s = s_top
        while s > -0.05:
            self.put_ball((0.0, 0.0, s))
            mujoco.mj_kinematics(m, d)
            mujoco.mj_collision(m, d)
            for index in range(d.ncon):
                contact = d.contact[index]
                if self.ball_geom not in (contact.geom1, contact.geom2):
                    continue
                other = contact.geom2 if contact.geom1 == self.ball_geom else contact.geom1
                point = self.rot.T @ (np.array(contact.pos) - self.origin)
                self.restore()
                return {
                    "s": round(s, 5),
                    "point": [round(float(v), 5) for v in point],
                    "body": self._geom_body(int(other)),
                    "on_hand": bool(self.is_hand_geom[other]),
                }
            s -= step
        self.restore()
        return None

    def empty_close_time(self, limit_s: float = 2.0) -> float | None:
        """Time for the empty hand to reach ρ ≥ η after a ``q_close`` step [s]."""
        self.restore()
        self._command_close()
        for n in range(int(limit_s / self.h)):
            mujoco.mj_step(self.model, self.data)
            if self.progress() >= self.cfg.eta_close:
                self.restore()
                return (n + 1) * self.h
        self.restore()
        return None


def measure_restitution(cfg: RigConfig, speed: float = 3.0) -> float:
    """Rebound / impact speed of the rig's ball on a fixed block, no gravity."""
    model, _ = build_model(cfg, anvil=True)
    data = mujoco.MjData(model)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, BALL_BODY)
    joint = model.body_jntadr[body]
    q, v = int(model.jnt_qposadr[joint]), int(model.jnt_dofadr[joint])
    anvil = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "docking_ident_anvil")
    mujoco.mj_forward(model, data)
    top = data.xpos[anvil] + np.array([0.0, 0.0, 0.5 + cfg.ball.radius])
    data.qpos[q : q + 3] = top + np.array([0.0, 0.0, 0.1])
    data.qpos[q + 3 : q + 7] = [1.0, 0.0, 0.0, 0.0]
    data.qvel[v : v + 3] = [0.0, 0.0, -speed]
    for _ in range(int(0.5 / float(model.opt.timestep))):
        mujoco.mj_step(model, data)
        if data.qvel[v + 2] > 0.0 and data.xpos[body][2] > top[2] + 0.02:
            return float(data.qvel[v + 2]) / speed
    return math.nan
