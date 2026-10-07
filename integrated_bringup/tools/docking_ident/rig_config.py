"""What the docking identification rig needs, read from a profile's SHIPPED config.

dynamic_catching E1-F15 (#741). The rig (rig.py) flies the sim's ball into the
preshape hand of one robot profile. Nothing about that robot is written in
this directory: a profile is named on the command line and everything else —
the hand's postures and its closure time, the catch frame, the wait pose, the
MJCF, and every setting the simulator lays over that MJCF when it loads it —
is read here from the files the bring-up itself reads.

The catching side comes through ``rtc_tools.analysis.catching_trials.load_profile``
(the reader the trial tools already share); this module adds what only the
simulator knows: its parameter stack, its robot groups and its ball.

No simulator is imported here, so this part runs under any python.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from rtc_tools.analysis import catching_trials
from rtc_tools.utils.controller_config import deep_merge, load_controller_config

REPO = Path(__file__).resolve().parents[3]
CONFIG_ROOT = REPO / "integrated_bringup" / "config"
# The simulator's own defaults: every sim launch loads this first and the
# profile's mujoco_simulator.yaml on top of it.
SIM_DEFAULTS = REPO / "rtc_mujoco_sim" / "config" / "solver_param.yaml"

# `projectile_ball.collision_*` when the profile does not set them
# (rtc_mujoco_sim mujoco_simulator_node.cpp, declare_parameter).
DEFAULT_BALL_CONTYPE = 2
DEFAULT_BALL_CONAFFINITY = 1
DEFAULT_BALL_TYPE = "tennis"


# ── The simulator's ball ──────────────────────────────────────────────────────
# Its material constants and the way its contact is built live only in C++
# (rtc_mujoco_sim/src/projectile_ball.cpp, mujoco_simulator.cpp). This is a COPY;
# test_docking_ident_config.py compares it with those files.
# ball_type -> (inertia_ratio, restitution, sliding friction,
#               torsional friction / r, rolling friction / r)
BALL_PRESETS = {
    "tennis": (0.55, 0.75, 0.6, 0.05, 0.02),
    "beanbag": (0.4, 0.1, 0.5, 0.3, 0.5),
    "hard": (0.378, 0.55, 0.5, 0.02, 0.005),
}
CONTACT_SUBSTEPS = 12.0  # half an oscillation of the contact spans this many substeps
BALL_SOLIMP = (0.99, 0.99, 0.001, 0.5, 2.0)
BALL_PRIORITY = 100
BALL_CONDIM = 6
BALL_BODY = "docking_ident_ball"
BALL_PARK = (0.0, 0.0, -50.0)


def spring_damper_restitution(zeta: float) -> float:
    """Rebound speed ratio of x'' = max(−2 ζ x' − x, 0) entering at x = 0 with
    x' = −1 (the simulator's ``ClampedSpringDamperRestitution``)."""
    h, x, v = 1e-3, 0.0, -1.0
    for _ in range(100_000):
        a = max(-2.0 * zeta * v - x, 0.0)
        v += h * a
        x += h * v
        if x >= 0.0:
            return max(v, 0.0)
    return 0.0


def damping_ratio_for(restitution: float) -> float:
    """ζ whose push-only contact rebounds at ``restitution`` (the simulator's
    ``ProjectileBallDampingRatioForRestitution``: 50 bisections on [0, 5])."""
    target = min(max(restitution, 1e-3), 0.999)
    low, high = 0.0, 5.0
    for _ in range(50):
        mid = 0.5 * (low + high)
        if spring_damper_restitution(mid) > target:
            low = mid
        else:
            high = mid
    return 0.5 * (low + high)


class RigConfigError(SystemExit):
    """The shipped config does not say what the rig needs."""


@dataclass(frozen=True)
class SimGroup:
    """One ``robot_response`` group of the simulator."""

    name: str
    joints: tuple[str, ...]  # command_joint_names
    servo_kp: tuple[float, ...]  # empty when the simulator would not override
    servo_kd: tuple[float, ...]


@dataclass(frozen=True)
class BallConfig:
    kind: str  # the simulator's ball_type preset
    radius: float  # [m]
    mass: float  # [kg]
    contype: int
    conaffinity: int


@dataclass(frozen=True)
class RigConfig:
    profile: str
    model_path: str  # resolved file
    control_period: float  # [s] physics_timestep: the period commands are applied at
    n_substeps: int
    solver: dict  # the simulator's `solver` block after the profile's overrides
    use_yaml_servo_gains: bool
    groups: tuple[SimGroup, ...]
    arm_joints: tuple[str, ...]  # the arm device's joints, device order
    arm_pose: tuple[float, ...]  # planner.wait_pose, same order [rad]
    hand_joints: tuple[str, ...]  # the hand device's joints, device order
    q_pre: tuple[float, ...]  # same order [rad]
    q_close: tuple[float, ...]
    caging_mask: tuple[bool, ...]
    eta_close: float
    t_close_e2e: float  # [s] what the hand sequencer subtracts from t_c
    catch_parent: str  # the body the catch frame is fixed to
    catch_xyz: tuple[float, float, float]  # in the parent frame [m]
    catch_rpy: tuple[float, float, float]  # [rad]
    ball: BallConfig

    def __post_init__(self) -> None:
        # The closure progress is the minimum over the caging set: of nothing
        # it is undefined, and the rig would fail on its first fly-in instead.
        if not any(self.caging_mask):
            raise RigConfigError(f"{self.profile}: caging_mask puts no joint in the caging set")


def _ros_parameters(path: Path) -> dict:
    """Every ``ros__parameters`` block of a parameter file, merged."""
    doc = yaml.safe_load(path.read_text()) or {}
    merged: dict = {}
    for node in doc.values():
        if isinstance(node, dict) and isinstance(node.get("ros__parameters"), dict):
            merged = deep_merge(merged, node["ros__parameters"])
    return merged


def simulator_parameters(config_dir: Path) -> dict:
    """The simulator's parameters as a sim launch composes them: its defaults,
    then the profile's ``mujoco_simulator.yaml`` leaf by leaf."""
    path = config_dir / "mujoco_simulator.yaml"
    if not path.is_file():
        raise RigConfigError(f"{config_dir}: no mujoco_simulator.yaml — not a sim profile")
    return deep_merge(_ros_parameters(SIM_DEFAULTS), _ros_parameters(path))


def resolve_model_path(uri: str) -> str:
    """A ``package://<package>/<path>`` model path as a file."""
    prefix = "package://"
    if not uri.startswith(prefix):
        return str(Path(uri).resolve())
    package, _, rel = uri[len(prefix) :].partition("/")
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory(package)
    except Exception as exc:  # not sourced, or the package is not built
        raise RigConfigError(
            f"{package} is not in this environment ({exc}): source the workspace first"
        ) from exc
    return str((Path(share) / rel).resolve())


def _numbers(node: object, what: str, length: int | None = None) -> tuple[float, ...]:
    if not isinstance(node, list) or any(isinstance(v, (str, bool)) for v in node):
        raise RigConfigError(f"{what} is not a list of numbers: {node!r}")
    if length is not None and len(node) != length:
        raise RigConfigError(f"{what} has {len(node)} entries, expected {length}")
    return tuple(float(v) for v in node)


def _groups(sim: dict) -> tuple[SimGroup, ...]:
    response = sim.get("robot_response") or {}
    out = []
    for name in response.get("groups") or []:
        block = response.get(name) or {}
        joints = tuple(str(j) for j in block.get("command_joint_names") or [])
        if not joints:
            raise RigConfigError(f"robot_response.{name} has no command_joint_names")
        # The group's own gains, else the simulator-wide ones (the simulator's rule).
        kp = block.get("servo_kp") or sim.get("servo_kp") or []
        kd = block.get("servo_kd") or sim.get("servo_kd") or []
        out.append(
            SimGroup(str(name), joints, tuple(float(v) for v in kp), tuple(float(v) for v in kd))
        )
    if not out:
        raise RigConfigError("the simulator has no robot_response groups")
    return tuple(out)


def load_rig_config(profile: str, config_root: Path = CONFIG_ROOT) -> RigConfig:
    """Read one robot profile (a directory under the bring-up's ``config/``)."""
    config_dir = Path(config_root) / profile
    if not config_dir.is_dir():
        raise RigConfigError(f"{config_dir}: no such profile")
    catching = catching_trials.load_profile(config_dir)
    hand_device = catching.hand_device
    if hand_device is None:
        raise RigConfigError(f"{profile}: the catching controller has no hand device")

    devices = catching.robot_params.get("devices") or {}

    def device_joints(device: str) -> tuple[str, ...]:
        names = (devices.get(device) or {}).get("joint_state_names")
        if not names:
            raise RigConfigError(f"{profile}: devices.{device}.joint_state_names is missing")
        return tuple(str(n) for n in names)

    arm_joints = device_joints(catching.arm_device)
    hand_joints = device_joints(hand_device)

    hand = catching.hand_yaml
    for key in ("q_pre", "q_close", "eta_close", "T_close_e2e"):
        if key not in hand or isinstance(hand[key], str):
            raise RigConfigError(
                f"{profile}: catching.robot.hand.{key} is not set ({hand.get(key)!r})"
            )
    count = len(hand_joints)
    mask = hand.get("caging_mask")
    if mask is None:
        mask = [True] * count  # the controller's rule: no mask means every joint
    if len(mask) != count:
        raise RigConfigError(
            f"{profile}: caging_mask has {len(mask)} entries, the hand has {count}"
        )

    controller_path = config_dir / "controllers" / f"{catching.controller}.yaml"
    tree = load_controller_config(controller_path, config_key=catching.controller)
    planner = (tree[catching.controller].get("catching") or {}).get("planner") or {}
    arm_pose = _numbers(planner.get("wait_pose"), "catching.planner.wait_pose", len(arm_joints))

    sim = simulator_parameters(config_dir)
    groups = _groups(sim)
    sim_joints = {j for g in groups for j in g.joints}
    for joint in arm_joints + hand_joints:
        if joint not in sim_joints:
            raise RigConfigError(f"{profile}: device joint '{joint}' is in no simulator group")

    ball = sim.get("projectile_ball") or {}
    try:
        ball_cfg = BallConfig(
            kind=str(ball.get("ball_type", DEFAULT_BALL_TYPE)),
            radius=float(ball["radius_m"]),
            mass=float(ball["mass_kg"]),
            contype=int(ball.get("collision_contype", DEFAULT_BALL_CONTYPE)),
            conaffinity=int(ball.get("collision_conaffinity", DEFAULT_BALL_CONAFFINITY)),
        )
    except KeyError as exc:
        raise RigConfigError(f"{profile}: projectile_ball.{exc.args[0]} is missing") from exc
    # The hand profile was identified FOR the controller's ball: a simulator
    # that throws another one makes every number here about a different ball.
    diameter = catching.ball_diameter_m
    if diameter is None or abs(diameter - 2.0 * ball_cfg.radius) > 1e-6:
        raise RigConfigError(
            f"{profile}: catching.core.ball.diameter ({diameter}) is not twice the simulator's "
            f"projectile_ball.radius_m ({ball_cfg.radius})"
        )

    try:
        model_uri = str(sim["model_path"])
        control_period = float(sim["physics_timestep"])
        n_substeps = int(sim["n_substeps"])
    except KeyError as exc:
        raise RigConfigError(f"{profile}: simulator parameter {exc.args[0]} is missing") from exc

    frame = catching.catch_frame
    return RigConfig(
        profile=profile,
        model_path=resolve_model_path(model_uri),
        control_period=control_period,
        n_substeps=n_substeps,
        solver=dict(sim.get("solver") or {}),
        use_yaml_servo_gains=bool(sim.get("use_yaml_servo_gains", False)),
        groups=groups,
        arm_joints=arm_joints,
        arm_pose=arm_pose,
        hand_joints=hand_joints,
        q_pre=_numbers(hand["q_pre"], "catching.robot.hand.q_pre", count),
        q_close=_numbers(hand["q_close"], "catching.robot.hand.q_close", count),
        caging_mask=tuple(bool(v) for v in mask),
        eta_close=float(hand["eta_close"]),
        t_close_e2e=float(hand["T_close_e2e"]),
        catch_parent=frame.parent,
        catch_xyz=tuple(frame.xyz),
        catch_rpy=tuple(frame.rpy),
        ball=ball_cfg,
    )
