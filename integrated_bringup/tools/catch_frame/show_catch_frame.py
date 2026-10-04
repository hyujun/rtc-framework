"""Render the shipped catch frame onto the preshape hand, for the S2.3b user check.

Reads `urdf.extra_frames.catch_frame` straight out of the robot config and
`robot.hand.q_pre` out of the catching controller config -- so what you see is
what the model builder will build, not a number retyped into this script. Then
it puts a ball-radius sphere at the catch-frame ORIGIN and a marker on the
approach axis (+z of the catch frame) and renders the settled scene.

What to look for:
  1. the translucent sphere sits IN the pocket, resting on the palm, not
     floating above the fingers and not buried inside them;
  2. the small opaque marker chain leaves the palm along the approach axis,
     i.e. it points back the way an incoming ball would arrive.

Run it with the workspace environment, whose python has both mujoco and yaml:

  ( cd <workspace> \
    && source <repo>/repo_scripts/scripts/setup_env.sh >/dev/null 2>&1 \
    && OMP_NUM_THREADS=1 python3 \
         <repo>/integrated_bringup/tools/catch_frame/show_catch_frame.py \
         ur5e_p1b --out <dir>/catch_frame_ur5e_p1b.png )

Add `--viewer` instead of `--out` to open the interactive MuJoCo viewer.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import yaml

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
        "robot_config": CFG / "ur5e_p1b/_base.yaml",
        "controller_config": CFG / "ur5e_p1b/controllers/demo_catching_controller.yaml",
        "mjcf": lambda: hand_description_file("robots/ur5e_p1b/mjcf/scene_with_table.xml"),
        "hand_group": "p1b",
        # MJCF actuator names for the hand, in devices.<group>.joint_state_names
        # order (the order q_pre is written in).
        "actuator_of_joint": "act_{stem}_joint",
        "arm_actuators": [0, 1, 2, 3, 4, 5],
        "arm_hold": [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0],
        "cameras": [(0.30, 150.0, -10.0), (0.30, 240.0, -10.0), (0.30, 90.0, -55.0)],
    },
    "iiwa7_leap": {
        "robot_config": CFG / "iiwa7_leap/sim.yaml",
        "controller_config": CFG / "iiwa7_leap/controllers/demo_catching_controller.yaml",
        "mjcf": lambda: str(
            REPO / "robot_descriptions/robots/iiwa7_leap/mjcf/scene_right_with_object.xml"
        ),
        "hand_group": "leap",
        "actuator_of_joint": None,  # LEAP actuators are named by URDF joint index
        "leap_slot_to_joint": [12, 13, 14, 15, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11],
        "arm_actuators": [0, 1, 2, 3, 4, 5, 6],
        "arm_hold": [0.0, 0.4, 0.0, -1.2, 0.0, 1.2, 0.0],
        "cameras": [(0.30, 150.0, -10.0), (0.30, 240.0, -10.0), (0.30, 90.0, -55.0)],
    },
}


def ros_params(path: Path) -> dict:
    """The `/**: ros__parameters:` tree of a robot config."""
    doc = yaml.safe_load(Path(path).read_text()) or {}
    for node in doc.values():
        if isinstance(node, dict) and "ros__parameters" in node:
            return node["ros__parameters"]
    raise SystemExit(f"{path}: no ros__parameters block")


def catching_tree(path: Path) -> dict:
    """The `catching:` tree of a controller config, which is keyed by the
    controller name at the top level rather than by `ros__parameters`."""
    doc = yaml.safe_load(Path(path).read_text()) or {}
    for node in doc.values():
        if isinstance(node, dict) and "catching" in node:
            return node["catching"]
    raise SystemExit(f"{path}: no <controller>.catching block")


def rpy_to_mat(rpy) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (
        np.cos(r),
        np.sin(r),
        np.cos(p),
        np.sin(p),
        np.cos(y),
        np.sin(y),
    )
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("robot", choices=sorted(ROBOTS))
    ap.add_argument("--out", type=Path, help="write a PNG here (one file per camera)")
    ap.add_argument("--viewer", action="store_true", help="open the interactive viewer")
    ap.add_argument("--settle", type=float, default=6.0, help="seconds to settle [s]")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=960)
    ap.add_argument("--distance", type=float, help="override camera distance [m]")
    a = ap.parse_args()
    if not a.out and not a.viewer:
        raise SystemExit("pass --out <png> or --viewer")

    import mujoco

    spec_cfg = ROBOTS[a.robot]
    params = ros_params(spec_cfg["robot_config"])
    ctrl = catching_tree(spec_cfg["controller_config"])

    cf = params["urdf"]["extra_frames"]["catch_frame"]
    parent, xyz, rpy = cf["parent"], np.asarray(cf["xyz"], float), cf["rpy"]
    provisional = cf.get("provisional")
    q_pre = list(ctrl["robot"]["hand"]["q_pre"])
    r_ball = float(ctrl["core"]["ball"]["diameter"]) / 2.0
    pocket = ctrl.get("planner", {}).get("hand", {})
    hand_joints = list(params["devices"][spec_cfg["hand_group"]]["joint_state_names"])

    print(f"robot          : {a.robot}")
    print(f"catch_frame    : parent={parent}  xyz={xyz.tolist()}  rpy={rpy}")
    print(f"                 provisional={provisional}")
    print(f"ball radius    : {r_ball} m   (catching.core.ball.diameter / 2)")
    print(f"pocket (S4.5)  : d_eff={pocket.get('d_eff')}  r_cap={pocket.get('r_cap')}")
    print(f"preshape q_pre : {len(q_pre)} values from catching.robot.hand.q_pre")

    # ── build the scene with two massless marker bodies ─────────────────────
    spec = mujoco.MjSpec.from_file(spec_cfg["mjcf"]())
    for name, radius, rgba in (
        ("cf_ball", r_ball, [0.1, 0.9, 0.2, 0.55]),
        ("cf_origin", 0.006, [1.0, 0.1, 0.1, 1.0]),
        ("cf_axis1", 0.005, [1.0, 0.5, 0.0, 1.0]),
        ("cf_axis2", 0.005, [1.0, 0.8, 0.0, 1.0]),
    ):
        b = spec.worldbody.add_body(name=name, pos=[0, 0, -10.0])
        b.add_freejoint()
        b.add_geom(
            name=name + "_geom",
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[radius, 0, 0],
            contype=0,
            conaffinity=0,
            mass=1e-6,
            rgba=rgba,
        )
    m = spec.compile()
    m.opt.gravity[:] = 0.0
    # The offscreen framebuffer is a model property (default 640x480) and the
    # renderer refuses any larger image, so widen it before the GL context exists.
    m.vis.global_.offwidth = max(m.vis.global_.offwidth, a.width)
    m.vis.global_.offheight = max(m.vis.global_.offheight, a.height)
    d = mujoco.MjData(m)

    # ── drive the arm to its hold posture and the hand to the preshape ──────
    if spec_cfg["actuator_of_joint"]:
        act_names = [
            spec_cfg["actuator_of_joint"].format(stem=j.removesuffix("_joint"))
            for j in hand_joints
        ]
    else:
        act_names = [str(i) for i in spec_cfg["leap_slot_to_joint"]]
    hand_act = []
    for n in act_names:
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
        if i < 0:
            raise SystemExit(f"MJCF has no actuator '{n}'")
        hand_act.append(i)
    for i, v in zip(spec_cfg["arm_actuators"], spec_cfg["arm_hold"], strict=True):
        d.ctrl[i] = v
    for i, v in zip(hand_act, q_pre, strict=True):
        d.ctrl[i] = v
    for _ in range(int(a.settle / m.opt.timestep)):
        mujoco.mj_step(m, d)

    pid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, parent)
    if pid < 0:
        raise SystemExit(f"MJCF has no body '{parent}' (the catch frame's parent link)")
    v = np.zeros(6)
    mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, pid, v, 0)
    moving = float(np.linalg.norm(v[3:]))
    if moving > 1e-3:
        raise SystemExit(
            f"'{parent}' is still moving at {moving:.4f} m/s after {a.settle} s -- "
            "raise --settle; measuring against a travelling palm reads pure nonsense"
        )

    palm_o, palm_r = d.xpos[pid].copy(), d.xmat[pid].reshape(3, 3).copy()
    origin = palm_o + palm_r @ xyz  # the catch frame origin
    approach = palm_r @ rpy_to_mat(rpy) @ np.array([0.0, 0.0, 1.0])
    print(f"catch origin   : world {np.round(origin, 4).tolist()}")
    print(f"approach axis  : world {np.round(approach, 4).tolist()}  (catch +z)")

    for name, pos in (
        ("cf_ball", origin),
        ("cf_origin", origin),
        ("cf_axis1", origin + 0.05 * approach),
        ("cf_axis2", origin + 0.10 * approach),
    ):
        adr = m.jnt_qposadr[m.body_jntadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)]]
        d.qpos[adr : adr + 3] = pos
        d.qpos[adr + 3 : adr + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(m, d)

    if a.viewer:
        import mujoco.viewer

        mujoco.viewer.launch(m, d)
        return 0

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.lookat[:] = origin
    written = []
    with mujoco.Renderer(m, height=a.height, width=a.width) as r:
        for k, (dist, azim, elev) in enumerate(spec_cfg["cameras"]):
            cam.distance = a.distance if a.distance else dist
            cam.azimuth, cam.elevation = azim, elev
            r.update_scene(d, camera=cam)
            img = r.render()
            path = (
                a.out
                if len(spec_cfg["cameras"]) == 1
                else a.out.with_name(f"{a.out.stem}_{k}{a.out.suffix}")
            )
            try:
                from PIL import Image

                Image.fromarray(img).save(path)
            except ImportError:
                import matplotlib

                matplotlib.use("Agg")
                import matplotlib.pyplot as plt

                plt.imsave(path, img)
            written.append(str(path))
    print("wrote          : " + ", ".join(written))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
