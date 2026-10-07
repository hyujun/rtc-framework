"""S2.3b gate: is the MJCF palm BODY frame the same frame as the URDF palm LINK?

The S4.5 pocket point was measured in MuJoCo, against the palm BODY frame
(`fk_pocket.py`: origin = xpos[palm], rot = xmat[palm] @ R_catch_in_palm). The
YAML `urdf.extra_frames.catch_frame.xyz` it has to become is expressed in the
URDF parent LINK frame and is consumed by Pinocchio. Those two conventions are
NOT the same in general -- plan SS11 records UR5e `upper_arm_link` as a case
where they differ -- so the number cannot be carried across without this check.

Decomposition. If the MJCF arm root differs from the URDF one by a constant B
(mj_root = urdf_root * B) and the palm by a constant P (mj_palm = urdf_palm * P),
then T_mj = B^-1 * T_pin * P for T = root->palm. Hence

    L := T_mj * T_pin^-1   is constant over q  <=>  P = I   (and then L = B^-1)
    R := T_pin^-1 * T_mj   is constant over q  <=>  B = I   (and then R = P)

So we sample q, report both residuals and their spread, and read off which of
the two conventions moved. R == I to numerical precision is the gate: only then
does a pocket point measured in MuJoCo mean the same thing in the URDF model.

Part 2 then converts the measured point to the YAML offset and verifies the
round trip end to end (Pinocchio catch-frame origin in world == MuJoCo pocket
point in world). Part 3 measures how much the *rejected* definition (plan SS10,
"preshape fingertip centroid") depends on which body one calls the fingertip.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

from rtc_tools.utils.rotations import rpy_matrix

REPO = Path(__file__).resolve().parents[3]
CFG = REPO / "integrated_bringup/config"


def hand_description_file(rel: str) -> str:
    """A file of the `hand_description` package — another repository of the workspace."""
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory("hand_description")
    except Exception as exc:  # not sourced, or the package is not built
        raise SystemExit(
            f"hand_description is not in this environment ({exc}): source the workspace first"
        ) from exc
    return str((Path(share) / rel).resolve())


ROBOTS = {
    "ur5e_p1b": {
        "configs": [CFG / "ur5e_p1b/_base.yaml"],
        "mjcf": lambda: hand_description_file("robots/ur5e_p1b/mjcf/scene_with_table.xml"),
        "arm_joints": [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ],
        # The planner's base frame is `base` (robot config sub_models.ur5e.
        # root_link). The MJCF body called `base` is the URDF `base_link`, which
        # is the SAME frame turned 180 deg about z -- the plan SS11 trap. So the
        # frame-equality comparison must be anchored on `base_link`, and the
        # difference between the two is what world_T_base then carries.
        "urdf_arm_root": "base_link",
        "urdf_planner_root": "base",
        "mjcf_arm_root": "base",
        "palm": "l_palm_link",
        "rpy": [0.0, 0.0, 0.0],
        "pocket_catch": [0.015, 0.145, 0.052],
        "tip_sets": {
            "tip_link": [
                "l_thumb_tip_link",
                "l_index_tip_link",
                "l_middle_tip_link",
                "l_ring_tip_link",
            ],
        },
    },
    "iiwa7_leap": {
        "configs": [CFG / "iiwa7_leap/sim.yaml"],
        "mjcf": lambda: str(
            REPO / "robot_descriptions/robots/iiwa7_leap/mjcf/scene_right_with_object.xml"
        ),
        "arm_joints": [f"A{i}" for i in range(1, 8)],
        "urdf_arm_root": "link_0",
        "urdf_planner_root": "link_0",
        "mjcf_arm_root": "link_0",
        "palm": "palm_lower",
        "rpy": [np.pi, 0.0, 0.0],
        "pocket_catch": [-0.035, 0.015, 0.069],
        "tip_sets": {
            # Both are called "the fingertip" somewhere in this repo. That is
            # the point of part 3.
            "tip_head": [
                "thumb_tip_head",
                "index_tip_head",
                "middle_tip_head",
                "ring_tip_head",
            ],
            "fingertip": ["thumb_fingertip", "fingertip", "fingertip_2", "fingertip_3"],
        },
    },
}

# Shipped preshape, from integrated_bringup/config/<robot>/controllers/
# demo_catching_controller.yaml robot.hand.q_pre, in devices.<hand>.
# joint_state_names order.
Q_PRE = {
    "ur5e_p1b": {
        "order": [
            "thumb_cmc_aa_joint",
            "thumb_cmc_fe_joint",
            "thumb_mcp_joint",
            "thumb_dip_fe_joint",
            "index_mcp_aa_joint",
            "index_mcp_fe_joint",
            "index_dip_fe_joint",
            "middle_mcp_fe_joint",
            "middle_dip_fe_joint",
            "ring_mcp_fe_joint",
        ],
        "q": [
            1.160644,
            -1.047198,
            0.322886,
            -0.195477,
            0.146608,
            -0.185005,
            -0.410152,
            -0.256563,
            -0.122173,
            -0.376991,
        ],
    },
    "iiwa7_leap": {
        # LEAP controller slot -> URDF/MJCF joint name (digits), plan L6.
        "order": [str(i) for i in [12, 13, 14, 15, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]],
        "q": [
            1.664695,
            0.003316,
            -0.444361,
            0.200015,
            -0.349066,
            1.063255,
            0.300022,
            0.200015,
            0.0,
            0.801455,
            0.300022,
            0.200015,
            0.174533,
            1.325054,
            0.300022,
            0.200015,
        ],
    },
}


def se3(rot, pos):
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = rot, pos
    return t


def inv(t):
    out = np.eye(4)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -t[:3, :3].T @ t[:3, 3]
    return out


def angle_of(rot) -> float:
    return float(np.arccos(np.clip((np.trace(rot) - 1.0) / 2.0, -1.0, 1.0)))


def load_urdf_text(cfgs) -> str:
    sys.path.insert(0, str(REPO / "rtc_tools"))
    from rtc_tools.analysis.derive_accel_limits import (
        load_robot_params,
        resolve_urdf_text,
    )

    return resolve_urdf_text(load_robot_params(list(cfgs)), None)[0]


def run(name: str, samples: int, seed: int) -> dict:
    import mujoco
    import pinocchio as pin

    spec = {**ROBOTS[name], "mjcf": ROBOTS[name]["mjcf"]()}
    out: dict = {"robot": name, "samples": samples, "seed": seed}

    urdf_text = load_urdf_text(spec["configs"])
    model = pin.buildModelFromXML(urdf_text)
    data = model.createData()
    m = mujoco.MjModel.from_xml_path(spec["mjcf"])
    d = mujoco.MjData(m)

    # ── index both models ────────────────────────────────────────────────────
    pin_arm_q = []
    for j in spec["arm_joints"]:
        assert model.existJointName(j), f"URDF has no joint {j}"
        pin_arm_q.append(model.joints[model.getJointId(j)].idx_q)
    mj_arm_adr = []
    for j in spec["arm_joints"]:
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)
        assert jid >= 0, f"MJCF has no joint {j}"
        mj_arm_adr.append(m.jnt_qposadr[jid])

    f_palm = model.getFrameId(spec["palm"])
    assert f_palm < model.nframes, spec["palm"]
    f_root = model.getFrameId(spec["urdf_arm_root"])
    b_palm = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, spec["palm"])
    b_root = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, spec["mjcf_arm_root"])
    assert b_palm >= 0 and b_root >= 0

    lo = np.asarray(model.lowerPositionLimit)[pin_arm_q]
    hi = np.asarray(model.upperPositionLimit)[pin_arm_q]
    lo, hi = np.maximum(lo, -2.5), np.minimum(hi, 2.5)
    rng = np.random.default_rng(seed)

    def pin_fk(qa, frame):
        q = pin.neutral(model)
        q[pin_arm_q] = qa
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacement(model, data, frame)
        return se3(
            np.asarray(data.oMf[frame].rotation),
            np.asarray(data.oMf[frame].translation),
        )

    def mj_fk(qa):
        mujoco.mj_resetData(m, d)
        d.qpos[mj_arm_adr] = qa
        mujoco.mj_kinematics(m, d)
        return (
            se3(d.xmat[b_root].reshape(3, 3).copy(), d.xpos[b_root].copy()),
            se3(d.xmat[b_palm].reshape(3, 3).copy(), d.xpos[b_palm].copy()),
        )

    # ── part 1: frame equality ───────────────────────────────────────────────
    L, R = [], []
    for _ in range(samples):
        qa = rng.uniform(lo, hi)
        t_pin = inv(pin_fk(qa, f_root)) @ pin_fk(qa, f_palm)
        w_root, w_palm = mj_fk(qa)
        t_mj = inv(w_root) @ w_palm
        L.append(t_mj @ inv(t_pin))
        R.append(inv(t_pin) @ t_mj)

    def summarize(mats, label):
        mats = np.asarray(mats)
        mean_p = mats[:, :3, 3].mean(axis=0)
        return {
            "label": label,
            "max_translation_norm_m": float(np.max(np.linalg.norm(mats[:, :3, 3], axis=1))),
            "max_rotation_angle_rad": float(max(angle_of(t[:3, :3]) for t in mats)),
            "max_rotation_angle_deg": float(np.degrees(max(angle_of(t[:3, :3]) for t in mats))),
            # spread over q: near zero => this residual is a CONSTANT frame offset
            "translation_spread_m": float(np.max(np.linalg.norm(mats[:, :3, 3] - mean_p, axis=1))),
            "rotation_spread_rad": float(
                max(angle_of(t[:3, :3] @ mats[0, :3, :3].T) for t in mats)
            ),
            "mean_translation_m": [round(float(v), 9) for v in mean_p],
            "first_rotation": [[round(float(v), 9) for v in row] for row in mats[0, :3, :3]],
        }

    out["part1_frame_equality"] = {
        "urdf_arm_root": spec["urdf_arm_root"],
        "mjcf_arm_root": spec["mjcf_arm_root"],
        "palm": spec["palm"],
        "L_left_residual__constant_iff_palm_frames_agree": summarize(L, "T_mj * T_pin^-1"),
        "R_right_residual__constant_iff_root_frames_agree": summarize(R, "T_pin^-1 * T_mj"),
    }
    # world_T_base, the transform the map tool has to apply to every candidate:
    # MuJoCo world -> the PLANNER's arm base frame (sub_models.<arm>.root_link).
    # It is the MJCF root body's world pose composed with the URDF-side
    # difference between the body MuJoCo exposes and the frame the planner names.
    f_plan = model.getFrameId(spec["urdf_planner_root"])
    qa0 = rng.uniform(lo, hi)
    plan_vs_anchor = inv(pin_fk(qa0, f_plan)) @ pin_fk(qa0, f_root)
    out["part1_frame_equality"]["urdf_planner_root_vs_mjcf_anchor"] = {
        "planner_root": spec["urdf_planner_root"],
        "mjcf_anchor_frame": spec["urdf_arm_root"],
        "translation_m": [round(float(v), 9) for v in plan_vs_anchor[:3, 3]],
        "rotation_angle_deg": round(float(np.degrees(angle_of(plan_vs_anchor[:3, :3]))), 6),
        "note": "nonzero rotation => naming the wrong one is silently wrong (plan SS11)",
    }
    w_root0 = mj_fk(np.zeros(len(pin_arm_q)))[0]
    world_t_base = w_root0 @ inv(plan_vs_anchor)
    out["part1_world_T_base"] = {
        "translation_m": [round(float(v), 9) for v in world_t_base[:3, 3]],
        "rotation": [[round(float(v), 9) for v in row] for row in world_t_base[:3, :3]],
        "rotation_angle_deg": round(float(np.degrees(angle_of(world_t_base[:3, :3]))), 6),
    }

    # ── part 2: proposed YAML offset, verified end to end ────────────────────
    # Anchored on the MJCF-matching frame so the base convention above cannot
    # absorb an error in the offset; the residual left here is epsilon_model.
    r_catch = rpy_matrix(spec["rpy"])
    p_catch = np.asarray(spec["pocket_catch"], float)
    xyz = r_catch @ p_catch  # catch-frame point -> parent-link coordinates
    errs = []
    for _ in range(samples):
        qa = rng.uniform(lo, hi)
        t_pin = inv(pin_fk(qa, f_root)) @ pin_fk(qa, f_palm)
        pin_origin = t_pin[:3, :3] @ xyz + t_pin[:3, 3]
        w_root, w_palm = mj_fk(qa)
        t_mj = inv(w_root) @ w_palm
        mj_point = t_mj[:3, :3] @ (r_catch @ p_catch) + t_mj[:3, 3]
        errs.append(np.linalg.norm(pin_origin - mj_point))
    out["part2_proposed_offset"] = {
        "rpy_rad": [float(v) for v in spec["rpy"]],
        "pocket_point_catch_frame_m": [float(v) for v in p_catch],
        "proposed_xyz_parent_frame_m": [round(float(v), 6) for v in xyz],
        "max_position_error_m": float(np.max(errs)),
        "mean_position_error_m": float(np.mean(errs)),
        "anchor_frame": spec["urdf_arm_root"],
        "note": "Pinocchio catch-frame origin vs MuJoCo pocket point, same q, "
        "both in the arm-root frame; residual = epsilon_model",
    }

    # ── part 3: how ambiguous is the REJECTED definition? ────────────────────
    q_pre = Q_PRE[name]
    mujoco.mj_resetData(m, d)
    for jn, v in zip(q_pre["order"], q_pre["q"], strict=True):
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
        assert jid >= 0, f"MJCF has no hand joint {jn}"
        d.qpos[m.jnt_qposadr[jid]] = v
    mujoco.mj_kinematics(m, d)
    palm_o, palm_r = d.xpos[b_palm].copy(), d.xmat[b_palm].reshape(3, 3).copy()
    rot = palm_r @ r_catch
    tips = {}
    for label, names in spec["tip_sets"].items():
        pts = []
        for n in names:
            b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
            assert b >= 0, n
            pts.append(rot.T @ (d.xpos[b] - palm_o))
        c = np.mean(pts, axis=0)
        tips[label] = {
            "centroid_catch_m": [round(float(v), 4) for v in c],
            "distance_to_pocket_point_m": round(float(np.linalg.norm(c - p_catch)), 4),
        }
    labels = list(tips)
    if len(labels) > 1:
        a = np.asarray(tips[labels[0]]["centroid_catch_m"])
        b = np.asarray(tips[labels[1]]["centroid_catch_m"])
        tips["spread_between_conventions_m"] = round(float(np.linalg.norm(a - b)), 4)
    out["part3_fingertip_centroid_definition"] = {
        "note": "kinematics only (mj_kinematics at the shipped q_pre), no settling",
        "conventions": tips,
    }
    out["provenance"] = {
        "git_head": subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=REPO,
            capture_output=True,
            text=True,
        ).stdout.strip(),
        "mjcf": spec["mjcf"],
        "robot_config": [str(p) for p in spec["configs"]],
        "pinocchio": __import__("pinocchio").__version__,
        "mujoco": __import__("mujoco").__version__,
    }
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("robot", choices=sorted(ROBOTS))
    ap.add_argument("--samples", type=int, default=12)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    print(json.dumps(run(a.robot, a.samples, a.seed), indent=2))
